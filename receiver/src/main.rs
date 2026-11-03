//! Ground station for the E80-900MBL-02 board (STM32F103C8 + LR2021).
//! Follows the satellite: LoRa until a telemetry packet, then FLRC images until the time the image
//! packets announce (or FLRC silence), then LoRa again. KEY1 toggles LoRa-only; KEY2 tells the
//! monitor to start a new flight log. Everything goes to the PC over USART1 (see `output.rs`).
#![no_std]
#![no_main]

use defmt::unwrap;
use defmt_rtt as _;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::usart::{self, UartTx};
use embassy_stm32::{bind_interrupts, interrupt, rcc, time::Hertz};
use embassy_time::{Duration, Instant, Timer};
use lr2021::Lr2021;
use panic_probe as _;

use output::{Uart, print_telemetry, send_frame, send_image};
use radio::{Busy, Error, FLRC_HZ, LORA_HZ, Radio, Rx};

/// Sends a text line to the PC (as a `T` frame) and RTT.
macro_rules! log {
    ($uart:expr, $($arg:tt)*) => {{
        use core::fmt::Write as _;
        let mut s: heapless::String<300> = heapless::String::new();
        let _ = write!(s, $($arg)*);
        defmt::info!("{=str}", s.as_str());
        $crate::output::send_frame($uart, b'T', &[], s.as_bytes());
    }};
}

mod output;
mod radio;

/// How often the keys are checked while no packet arrives
const POLL: Duration = Duration::from_millis(50);
/// Back to LoRa if no image packet arrives this long after the telemetry
const FLRC_SILENCE: Duration = Duration::from_millis(300);
/// Switch to LoRa this much before the announced time
const SWITCH_EARLY_MS: u64 = 20;
const SILENCE_WARNING: Duration = Duration::from_secs(10);
const LED_FLASH: Duration = Duration::from_millis(60);

bind_interrupts!(struct Irqs {
    EXTI2 => exti::InterruptHandler<interrupt::typelevel::EXTI2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // 72 MHz from the board's 8 MHz crystal, like the E80 demo; 8 MHz can't do 921600 baud
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(rcc::Hse { freq: Hertz(8_000_000), mode: rcc::HseMode::Oscillator });
    config.rcc.pll = Some(rcc::Pll { src: rcc::PllSource::HSE, prediv: rcc::PllPreDiv::DIV1, mul: rcc::PllMul::MUL9 });
    config.rcc.sys = rcc::Sysclk::PLL1_P;
    config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;
    let p = embassy_stm32::init(config);

    let mut uart_cfg = usart::Config::default();
    uart_cfg.baudrate = 921_600;
    let mut uart = unwrap!(UartTx::new_blocking(p.USART1, p.PA9, uart_cfg));
    log!(&mut uart, "Receiver starting");

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(2_000_000);
    let spi = BlockingAsync::new(Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_cfg));
    let mut radio = Lr2021::new_blocking(
        Output::new(p.PB0, Level::High, Speed::Low),
        Busy(Input::new(p.PA3, Pull::Up)),
        spi,
        Output::new(p.PA4, Level::High, Speed::VeryHigh),
    );
    let mut dio8 = ExtiInput::new(p.PB2, p.EXTI2, Pull::Down, Irqs);
    // Board LEDs are active-low, KEY1 and KEY2 read low when pressed
    let mut leds = Leds {
        activity: Output::new(p.PB13, Level::High, Speed::Low),
        lora: Output::new(p.PB12, Level::High, Speed::Low),
    };
    let keys = Keys { lora_only: Input::new(p.PB15, Pull::Up), new_flight: Input::new(p.PB14, Pull::Up) };
    let mut lora_only = false;

    loop {
        while let Err((step, e)) = radio::setup(&mut radio).await {
            log!(&mut uart, "Radio setup failed at {step}: {:?}, retrying in 1 s", e);
            Timer::after_secs(1).await;
        }
        log!(&mut uart, "Listening: LoRa {} kHz, FLRC {} kHz, {}", LORA_HZ / 1000, FLRC_HZ / 1000, describe_mode(lora_only));
        // Only returns on a radio error; the loop then resets the radio
        let (step, e) = listen(&mut radio, &mut dio8, &mut uart, &mut leds, &keys, &mut lora_only).await;
        log!(&mut uart, "Radio error at {step}: {:?}, resetting radio", e);
    }
}

fn describe_mode(lora_only: bool) -> &'static str {
    if lora_only {
        "LoRa only (telemetry); press KEY1 for images too"
    } else {
        "telemetry (LoRa) and images (FLRC); press KEY1 for LoRa only"
    }
}

struct Leds {
    activity: Output<'static>,
    lora: Output<'static>,
}

struct Keys {
    /// KEY1
    lora_only: Input<'static>,
    /// KEY2
    new_flight: Input<'static>,
}

/// Follows the satellite's LoRa/FLRC schedule and handles the keys; only returns on a radio error.
async fn listen(
    radio: &mut Radio,
    dio8: &mut ExtiInput<'static>,
    uart: &mut Uart,
    leds: &mut Leds,
    keys: &Keys,
    lora_only: &mut bool,
) -> Error {
    let mut buf = [0u8; 255];
    // setup() leaves the radio in LoRa
    let mut flrc = false;
    let mut switch_at = Instant::now();
    let mut was_pressed = false;
    let mut key2_was_pressed = false;
    let mut last_packet = Instant::now();
    let mut warned = false;
    let mut led_off_at = Instant::now();

    loop {
        let result: Result<(), Error> = async {
            let pressed = keys.lora_only.is_low();
            if pressed && !was_pressed {
                *lora_only = !*lora_only;
                switch_at = Instant::now();
                log!(uart, "Switched to {}", describe_mode(*lora_only));
            }
            was_pressed = pressed;

            let pressed = keys.new_flight.is_low();
            if pressed && !key2_was_pressed {
                send_frame(uart, b'N', &[], &[]);
            }
            key2_was_pressed = pressed;

            if flrc && Instant::now() >= switch_at {
                radio::lora_mode(radio).await?;
                flrc = false;
            }
            leds.lora.set_level(if flrc { Level::High } else { Level::Low });

            let timeout = if flrc { POLL.min(switch_at.saturating_duration_since(Instant::now())) } else { POLL };
            match radio::receive(radio, dio8, &mut buf, timeout, !flrc).await? {
                Rx::Packet(p) => {
                    last_packet = Instant::now();
                    warned = false;
                    if !flrc {
                        print_telemetry(uart, &buf[..p.len], &p);
                        leds.activity.toggle();
                        if !*lora_only {
                            radio::flrc_mode(radio).await?;
                            flrc = true;
                            switch_at = Instant::now() + FLRC_SILENCE;
                        }
                    } else if p.len >= 2 {
                        // Each image packet starts with the ms from its end until the next telemetry
                        if p.crc_ok {
                            let ms = u16::from_le_bytes([buf[0], buf[1]]) as u64;
                            // The satellite's clock may be ~2 % off
                            let early = ms / 50 + SWITCH_EARLY_MS;
                            switch_at = Instant::now() + Duration::from_millis(ms.saturating_sub(early));
                        }
                        // ~3 ms at 921600 baud; the radio keeps receiving the next packet meanwhile
                        send_image(uart, &buf[2..p.len], &p);
                        leds.activity.set_low();
                        led_off_at = Instant::now() + LED_FLASH;
                    }
                }
                Rx::BadIrq(irq) => {
                    log!(uart, "Bad packet, IRQ flags 0x{irq:08X} ({})", radio::describe(irq));
                }
                Rx::Timeout => {
                    if !warned && last_packet.elapsed() > SILENCE_WARNING {
                        log!(uart, "No packet for {} s", SILENCE_WARNING.as_secs());
                        warned = true;
                    }
                }
            }
            if flrc && Instant::now() >= led_off_at {
                leds.activity.set_high();
            }
            Ok(())
        }
        .await;
        if let Err(e) = result {
            return e;
        }
    }
}

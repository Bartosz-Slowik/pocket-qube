#![no_std]
#![no_main]

mod camera;
mod gps;
mod image;
mod radio;
mod telemetry;

use core::cell::RefCell;

use bmp388_embedded::{Address, Bmp388, Oversampling};
use bno055::{BNO055OperationMode, Bno055};
use defmt::{Debug2Format, error, info, unwrap, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::usart::{self, BufferedUart};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, i2c::I2c, peripherals, rcc, spi, time::Hertz};
use embassy_time::{Delay, Duration, Instant};
use embedded_hal_bus::i2c::RefCellDevice;
use embedded_io_async::Read;
use heapless::Vec;
use libm::{powf, roundf};
use lr2021::Lr2021;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
});

/// One telemetry packet per cycle; FLRC image packets fill the rest.
const CYCLE: Duration = Duration::from_secs(3);
/// FLRC stops this long before the next telemetry, leaving time for `lora_mode` and the sensors
const FLRC_STOP: Duration = Duration::from_millis(200);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    // 4 MHz default is too slow to keep up with 115200 baud GPS
    config.rcc.msis = Some(rcc::MSIRange::RANGE_48MHZ);
    let p = embassy_stm32::init(config);
    info!("PocketQube starting");
    // Resets the MCU if the main loop stalls
    let mut watchdog = IndependentWatchdog::new(p.IWDG, 8_000_000);
    watchdog.unleash();

    // Radio (LR2021 on SPI1), set up inside the loop so it can recover
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(2_000_000);
    let spi = spi::Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.GPDMA1_CH0, p.GPDMA1_CH1, spi_cfg);
    let mut radio = Lr2021::new_blocking(
        Output::new(p.PB1, Level::High, Speed::Low),
        Input::new(p.PB0, Pull::Up),
        spi,
        Output::new(p.PA4, Level::High, Speed::VeryHigh),
    );
    let mut radio_ok = false;

    // ESP32-S3 camera on SPI2; it's the SPI slave and hands out ready-made image packets
    let mut cam_cfg = spi::Config::default();
    cam_cfg.frequency = Hertz(4_000_000);
    let mut camera = camera::Camera {
        spi: spi::Spi::new(p.SPI2, p.PB13, p.PB15, p.PB14, p.GPDMA1_CH2, p.GPDMA1_CH3, cam_cfg),
        cs: Output::new(p.PB12, Level::High, Speed::VeryHigh),
        last_packet: None,
        last_warning: None,
    };

    // Sensors (BNO055 + BMP388 sharing I2C1), set up inside the loop so they can recover
    let i2c = RefCell::new(I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Default::default()));
    let mut imu = Bno055::new(RefCellDevice::new(&i2c));
    let mut imu_ok = false;
    let mut baro = None;

    // GPS (GM10 Mini V3 on USART1)
    let mut uart_cfg = usart::Config::default();
    uart_cfg.baudrate = 115200;
    uart_cfg.rx_pull = Pull::Up;
    let mut tx_buf = [0; 16];
    let mut rx_buf = [0; 1024];
    let mut uart = unwrap!(BufferedUart::new(
        p.USART1, p.PA10, p.PA9, &mut tx_buf, &mut rx_buf, Irqs, uart_cfg
    ));
    let mut gps = gps::Gps::default();

    let mut images = image::ImageTx::default();

    info!("Ready");
    let mut seq: u8 = 0;
    let mut ground_pa: Option<f32> = None;
    let mut sea_level_pa: Option<f32> = None;
    let mut next_tx = Instant::now();
    loop {
        watchdog.pet();
        // Every minute, in case the GPS restarted and lost its RAM settings
        if seq % 20 == 0 {
            gps::Gps::configure(&mut uart, gps.fix.ok).await;
        }

        if !radio_ok {
            match radio::setup(&mut radio).await {
                Ok(()) => {
                    info!("Radio ready");
                    radio_ok = true;
                }
                Err((step, e)) => error!("Radio setup failed at {}: {}", step, e),
            }
        }
        // BNO055 needs ~650 ms after power-on, so early attempts can fail
        if !imu_ok {
            imu_ok = imu
                .init(&mut Delay)
                .and_then(|()| imu.set_mode(BNO055OperationMode::NDOF, &mut Delay))
                .inspect(|()| info!("BNO055 ready"))
                .inspect_err(|e| error!("BNO055 init failed: {}", Debug2Format(e)))
                .is_ok();
        }
        if baro.is_none() {
            baro = Bmp388::new(RefCellDevice::new(&i2c), Delay, Address::Primary)
                .or_else(|_| Bmp388::new(RefCellDevice::new(&i2c), Delay, Address::Secondary))
                .and_then(|mut b| b.set_oversampling(Oversampling::X2, Oversampling::X2).map(|()| b))
                .inspect(|_| info!("BMP388 ready"))
                .inspect_err(|e| error!("BMP388 init failed (tried 0x76 and 0x77): {}", Debug2Format(e)))
                .ok();
        }

        let (heading, roll, pitch) = match imu_ok.then(|| imu.euler_angles()) {
            Some(Ok(e)) => (e.c, e.a, e.b),
            Some(Err(e)) => {
                warn!("BNO055 read failed: {}, re-initializing", Debug2Format(&e));
                imu_ok = false;
                (f32::NAN, f32::NAN, f32::NAN)
            }
            None => (f32::NAN, f32::NAN, f32::NAN),
        };
        let (pa, temp) = match baro.as_mut().map(|b| b.forced_measurement()) {
            Some(Ok(m)) => (m.pressure as f32, m.temperature as f32),
            Some(Err(e)) => {
                warn!("BMP388 read failed: {}, re-initializing", Debug2Format(&e));
                baro = None;
                (f32::NAN, f32::NAN)
            }
            None => (f32::NAN, f32::NAN),
        };

        let g = &gps.fix;
        if ground_pa.is_none() && pa.is_finite() {
            info!("Ground pressure: {}Pa", pa as u32);
            ground_pa = Some(pa);
        }
        if sea_level_pa.is_none() && g.ok && pa.is_finite() {
            let p0 = pa / powf(1.0 - g.alt / 44330.0, 5.255);
            info!("Sea level pressure from GPS: {}Pa", p0 as u32);
            sea_level_pa = Some(p0);
        }
        let asl = altitude(pa, sea_level_pa.unwrap_or(101_325.0));
        let agl = altitude(pa, ground_pa.unwrap_or(f32::NAN));

        if gps.silent() {
            warn!("GPS: no data on PA10 for 5 s (check wiring/power)");
        }
        let fix = match (gps.silent(), g.ok) {
            (true, _) => 2,
            (_, true) => 1,
            _ => 0,
        };
        let telemetry = telemetry::Telemetry {
            seq,
            heading,
            roll,
            pitch,
            pressure: pa,
            temp,
            lat: g.lat,
            lon: g.lon,
            alt: g.alt,
            // Satellites used once there's a fix; before that, satellites heard shows progress
            sats: if g.ok { g.sats } else { g.tracked },
            fix,
            cn0: g.best_cn0,
        }
        .encode();
        info!(
            "#{:03} H{} R{} P{} {}Pa {}C {},{} {}m S{} F{} | {}m ASL, {}m above ground | GPS hears {} sats, best {}dBHz",
            seq, heading, roll, pitch, pa, temp, g.lat, g.lon, g.alt, g.sats, fix, roundf(asl), roundf(agl), g.tracked, g.best_cn0
        );

        if radio_ok {
            gps.listen(&mut uart, next_tx).await;
            next_tx = Instant::now() + CYCLE;
            if let Err((step, e)) = transmit_cycle(&mut radio, &mut gps, &mut uart, &telemetry, &mut camera, &mut images, next_tx).await {
                error!("TX #{} failed at {}: {}, resetting radio", seq, step, e);
                radio_ok = false;
            }
        } else {
            gps.listen(&mut uart, Instant::now() + Duration::from_millis(500)).await;
        }
        seq = seq.wrapping_add(1);
    }
}

/// Telemetry on LoRa, then FLRC image packets until shortly before `next_tx`. Each FLRC packet is
/// `ms from its end until next_tx (u16 LE)` + image packet, so the receiver knows when to switch.
async fn transmit_cycle(
    radio: &mut radio::Radio,
    gps: &mut gps::Gps,
    uart: &mut impl Read,
    telemetry: &[u8],
    camera: &mut camera::Camera,
    images: &mut image::ImageTx,
    next_tx: Instant,
) -> Result<(), radio::Error> {
    // Packets sent although the channel stayed busy
    let mut forced = 0;
    // Packets whose TX done flag never showed up
    let mut unconfirmed = 0;
    forced += !radio::channel_clear(radio).await? as u32;
    radio::send_lora(radio, telemetry).await?;
    gps.listen(uart, Instant::now() + radio::lora_airtime(telemetry.len())).await;

    radio::flrc_mode(radio).await?;
    // Gives the receiver time to switch to FLRC
    gps.listen(uart, Instant::now() + Duration::from_millis(30)).await;
    let mut image: Vec<u8, { image::MAX_PACKET }> = Vec::new();
    let mut packet: Vec<u8, { radio::MAX_LEN }> = Vec::new();
    while Instant::now() + radio::LBT_MAX + radio::flrc_airtime(radio::MAX_LEN) + FLRC_STOP < next_tx {
        if !camera.next(&mut image).await {
            if camera.alive() {
                // The ESP32 is still encoding the next photo
                gps.listen(uart, Instant::now() + Duration::from_millis(20)).await;
                continue;
            }
            images.next(&mut image);
        }
        // Before computing the timing prefix, so a wait for the channel can't make it wrong
        forced += !radio::channel_clear(radio).await? as u32;
        let end = Instant::now() + radio::flrc_airtime(image.len() + 2);
        let ms = next_tx.saturating_duration_since(end).as_millis() as u16;
        packet.clear();
        let _ = packet.extend_from_slice(&ms.to_le_bytes());
        let _ = packet.extend_from_slice(&image);
        radio::send_flrc(radio, &packet).await?;
        // The receiver forwards each packet while the next is on air, so no gap is needed.
        // `end` was taken before the FIFO write, hence the extra bound
        let give_up = end + Duration::from_millis(5);
        while !radio::sent(radio).await? {
            if Instant::now() > give_up {
                unconfirmed += 1;
                break;
            }
            gps.listen(uart, Instant::now() + Duration::from_millis(1)).await;
        }
    }
    if forced > 0 {
        warn!("LBT: channel busy, {} packets sent anyway", forced);
    }
    if unconfirmed > 0 {
        warn!("FLRC: no TX done for {} packets", unconfirmed);
    }
    radio::lora_mode(radio).await
}

/// Height in meters of pressure `pa` above the level where pressure is `ref_pa`.
fn altitude(pa: f32, ref_pa: f32) -> f32 {
    44330.0 * (1.0 - powf(pa / ref_pa, 0.1903))
}

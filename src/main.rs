#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Ticker};
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("Hello World!");

    // This is LED LD2 on STM32U545RE
    let mut led = Output::new(p.PA5, Level::Low, Speed::Medium);

    let mut ticker = Ticker::every(Duration::from_secs(1));

    let mut i = 0;

    loop {
        defmt::info!("on!");
        led.set_high();
        ticker.next().await;

        defmt::info!("off!");
        led.set_low();
        ticker.next().await;

        i += 1;
        assert!(i < 5, "Just testing if panicking works");
    }
}

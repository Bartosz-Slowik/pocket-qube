#![no_std]
#![no_main]

mod gps;
mod radio;
mod sensors;
mod telemetry;

use defmt::unwrap;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Input, Level, Output, Pull, Speed},
    i2c::I2c,
    spi::{Config as SpiConfig, Spi},
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use panic_probe as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("Starting PocketQube telemetry");

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = Hertz(2_000_000);

    let spi = radio::SpiWrapper::new(Spi::new_blocking(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        spi_config,
    ));

    let radio = radio::Radio::new_blocking(
        Output::new(p.PB1, Level::High, Speed::Low),
        Input::new(p.PB0, Pull::Up),
        spi,
        Output::new(p.PA4, Level::High, Speed::VeryHigh),
    );

    let radio = match radio::configure(radio).await {
        Ok(radio) => radio,
        Err(error) => {
            defmt::error!("LR2021 initialization failed: {}", error);
            loop {}
        }
    };

    let i2c = I2c::new_blocking(
        p.I2C1,
        p.PB6,
        p.PB7,
        Default::default(),
    );

    let (imu, barometer) = match sensors::initialize(i2c) {
        Some(sensors) => sensors,
        None => {
            defmt::error!("Sensor initialization failed");
            loop {}
        }
    };

    let gps_uart = match gps::initialize(p.USART1, p.PA10) {
        Ok(uart) => uart,
        Err(error) => {
            defmt::error!("GPS initialization failed: {}", error);
            loop {}
        }
    };

    defmt::info!("Initialization complete; starting tasks");
    unwrap!(spawner.spawn(gps::gps_task(gps_uart)));
    unwrap!(spawner.spawn(sensors::sensor_task(imu, barometer)));
    unwrap!(spawner.spawn(radio::radio_task(radio)));

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

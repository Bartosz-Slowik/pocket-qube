#![no_std]
#![no_main]

use bmp388_embedded::{Address, Bmp388, Oversampling, PowerMode};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{I2c, Master};
use embassy_time::{Delay, Duration, Ticker};
use panic_probe as _;

struct PressureReading {
    pressure_hpa: f32,
    temperature_c: f32,
}

impl PressureReading {
    fn log(&self) {
        defmt::info!(
            "BMP388: pressure={} hPa, temperature={} C",
            self.pressure_hpa,
            self.temperature_c
        );
    }
}

fn altitude_from_pressure_hpa(pressure_hpa: f32, sea_level_hpa: f32) -> f32 {
    if pressure_hpa <= 0.0 || sea_level_hpa <= 0.0 {
        return 0.0;
    }

    let ratio = pressure_hpa / sea_level_hpa;
    let ratio_fifth_power = ratio * ratio * ratio * ratio * ratio;
    44330.0 * (1.0 - ratio_fifth_power)
}

fn find_bmp388_address(i2c: &mut I2c<'_, embassy_stm32::mode::Blocking, Master>) -> Option<Address> {
    for candidate in [Address::Primary, Address::Secondary] {
        let address = candidate.addr();
        let mut buf = [0u8; 1];
        defmt::info!("Trying BMP388 at 0x{:02X}", address);

        if i2c.blocking_write_read(address, &[0x00], &mut buf).is_ok() {
            defmt::info!("BMP388 responded at 0x{:02X}", address);
            return Some(candidate);
        }
    }

    defmt::warn!("BMP388 did not respond at either 0x76 or 0x77");
    None
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("BMP388 I2C init on PB6/PB7 (SCL/SDA)");
    let mut ticker = Ticker::every(Duration::from_secs(1));

    let mut i2c = I2c::new_blocking(
        p.I2C1,
        p.PB6, // BMP388 SCL
        p.PB7, // BMP388 SDA
        Default::default(),
    );

    loop {
        defmt::info!("Probing BMP388 I2C address before init");
        let bmp_address = match find_bmp388_address(&mut i2c) {
            Some(address) => address,
            None => {
                defmt::error!("BMP388 init failed; no valid address responded");
                loop {}
            }
        };

        let delay = Delay;
        let mut sensor = match Bmp388::new(i2c, delay, bmp_address) {
            Ok(sensor) => sensor,
            Err(_err) => {
                defmt::error!("BMP388 init failed");
                loop {}
            }
        };

        if sensor.set_oversampling(Oversampling::X2, Oversampling::X2).is_err() {
            defmt::error!("BMP388 oversampling failed");
            loop {}
        }

        if sensor.set_power_control(true, true, PowerMode::Normal).is_err() {
            defmt::error!("BMP388 power control failed");
            loop {}
        }

        match sensor.forced_measurement() {
            Ok(measurement) => {
                let sample = PressureReading {
                    pressure_hpa: measurement.pressure as f32 / 100.0,
                    temperature_c: measurement.temperature as f32,
                };
                let altitude = altitude_from_pressure_hpa(sample.pressure_hpa, 1013.25);
                sample.log();
                defmt::info!("BMP388 altitude estimate={} m", altitude);
            }
            Err(_err) => {
                defmt::error!("BMP388 read failed");
            }
        }

        let (restored_i2c, _) = sensor.destroy();
        i2c = restored_i2c;

        ticker.next().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pressure_reading_is_converted_to_hpa() {
        let reading = PressureReading {
            pressure_hpa: 1013.25,
            temperature_c: 24.0,
        };
        assert!(reading.pressure_hpa > 1000.0);
    }

    #[test]
    fn altitude_is_zero_at_sea_level_reference() {
        let altitude = altitude_from_pressure_hpa(1013.25, 1013.25);
        assert!((altitude - 0.0).abs() < 0.001);
    }
}

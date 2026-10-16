use core::cell::RefCell;

use bmp388_embedded::{Address, Bmp388, Oversampling};
use bno055::{Bno055, BNO055OperationMode};
use embassy_stm32::i2c::{I2c, Master};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::i2c::RefCellDevice;
use static_cell::StaticCell;

use crate::gps::{GpsData, LATEST_GPS};
use crate::telemetry::{Telemetry, LATEST};

const SAMPLE_PERIOD: Duration = Duration::from_secs(1);

type I2cBus = I2c<'static, embassy_stm32::mode::Blocking, Master>;
type I2cDevice = RefCellDevice<'static, I2cBus>;
pub type Imu = Bno055<I2cDevice>;
pub type Barometer = Bmp388<I2cDevice, Delay>;

static I2C_BUS: StaticCell<RefCell<I2cBus>> = StaticCell::new();

fn find_bmp388_address(i2c: &mut I2cBus) -> Option<Address> {
    for address in [Address::Primary, Address::Secondary] {
        let mut id = [0u8; 1];

        if i2c
            .blocking_write_read(address.addr(), &[0x00], &mut id)
            .is_ok()
        {
            return Some(address);
        }
    }

    None
}

pub fn initialize(i2c: I2cBus) -> Option<(Imu, Barometer)> {
    let mut i2c = i2c;

    let bmp_address = match find_bmp388_address(&mut i2c) {
        Some(addr) => {
            defmt::info!("BMP388 found at address 0x{:02X}", addr.addr());
            addr
        }
        None => {
            defmt::error!("BMP388 not found on I2C bus (tried primary and secondary addresses)");
            return None;
        }
    };

    let bus = I2C_BUS.init(RefCell::new(i2c));
    let mut delay = Delay;
    let mut imu = Bno055::new(RefCellDevice::new(bus));

    if imu.init(&mut delay).is_err() {
        defmt::error!("BNO055 init failed");
        return None;
    }
    if imu.set_mode(BNO055OperationMode::NDOF, &mut delay).is_err() {
        defmt::error!("BNO055 set_mode(NDOF) failed");
        return None;
    }

    let mut barometer = match Bmp388::new(RefCellDevice::new(bus), Delay, bmp_address) {
        Ok(b) => b,
        Err(_) => {
            defmt::error!("BMP388 construction failed");
            return None;
        }
    };

    if barometer.set_oversampling(Oversampling::X2, Oversampling::X2).is_err() {
        defmt::error!("BMP388 set_oversampling failed");
        return None;
    }

    defmt::info!("BNO055 + BMP388 initialized OK");
    Some((imu, barometer))
}

#[embassy_executor::task]
pub async fn sensor_task(mut imu: Imu, mut barometer: Barometer) {
    let mut gps_data = GpsData::default();

    loop {
        if let Some(gps) = LATEST_GPS.try_take() {
            gps_data = gps;
        }

        match (imu.euler_angles(), barometer.forced_measurement()) {
            (Ok(angles), Ok(baro)) => {
                let telemetry = Telemetry {
                    heading: angles.a as f32,
                    roll: angles.b as f32,
                    pitch: angles.c as f32,
                    pressure_pa: baro.pressure as f32,
                    temperature_c: baro.temperature as f32,
                    gps_lat: gps_data.latitude,
                    gps_lon: gps_data.longitude,
                    gps_alt: gps_data.altitude_m,
                    gps_sats: gps_data.sats,
                    gps_fix: gps_data.fix,
                };
                defmt::info!("{}", telemetry);
                LATEST.signal(telemetry);
            }
            (Err(_), _) => defmt::warn!("BNO055 read failed"),
            (_, Err(_)) => defmt::warn!("BMP388 read failed"),
        }

        Timer::after(SAMPLE_PERIOD).await;
    }
}
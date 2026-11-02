//! 24-byte binary telemetry for the slow SF12 LoRa link, little-endian:
//!
//! | bytes | field       | encoding                                             |
//! |-------|-------------|------------------------------------------------------|
//! | 0     | seq         | u8                                                   |
//! | 1-2   | heading     | u16, 0.1 deg (0xFFFF = no IMU)                       |
//! | 3-6   | roll, pitch | i16 each, 0.1 deg (i16::MIN = no IMU)                |
//! | 7-9   | pressure    | u24, Pa (0xFFFFFF = no barometer)                    |
//! | 10-11 | temperature | i16, 0.1 C (i16::MIN = no barometer)                 |
//! | 12-19 | lat, lon    | i32 each, 1e-7 deg                                   |
//! | 20-21 | GPS alt     | u16, m                                               |
//! | 22    | status      | bits 0-5 sats, bits 6-7 fix (0 none, 1 fix, 2 no GPS) |
//! | 23    | GPS signal  | u8, strongest satellite C/N0 in dBHz                 |
//!
//! The receiver (`receiver/src/main.rs`) turns it back into the usual text line.
use libm::{round, roundf};

/// 24 bytes take as long on air as 23 at SF12
pub const LEN: usize = 24;

pub struct Telemetry {
    pub seq: u8,
    pub heading: f32,
    pub roll: f32,
    pub pitch: f32,
    pub pressure: f32,
    pub temp: f32,
    pub lat: f32,
    pub lon: f32,
    pub alt: f32,
    pub sats: u8,
    /// 0 no fix, 1 fix, 2 no data from the GPS
    pub fix: u8,
    pub cn0: u8,
}

impl Telemetry {
    pub fn encode(&self) -> [u8; LEN] {
        let tenths = |v: f32| if v.is_finite() { roundf(v * 10.0) as i16 } else { i16::MIN };
        let heading = if self.heading.is_finite() { roundf(self.heading * 10.0) as u16 } else { u16::MAX };
        let pressure = if self.pressure.is_finite() { (self.pressure as u32).min(0xFF_FFFE) } else { 0xFF_FFFF };
        let degrees = |v: f32| round(v as f64 * 1e7) as i32;

        let mut b = [0; LEN];
        b[0] = self.seq;
        b[1..3].copy_from_slice(&heading.to_le_bytes());
        b[3..5].copy_from_slice(&tenths(self.roll).to_le_bytes());
        b[5..7].copy_from_slice(&tenths(self.pitch).to_le_bytes());
        b[7..10].copy_from_slice(&pressure.to_le_bytes()[..3]);
        b[10..12].copy_from_slice(&tenths(self.temp).to_le_bytes());
        b[12..16].copy_from_slice(&degrees(self.lat).to_le_bytes());
        b[16..20].copy_from_slice(&degrees(self.lon).to_le_bytes());
        b[20..22].copy_from_slice(&(self.alt.clamp(0.0, 65535.0) as u16).to_le_bytes());
        b[22] = self.sats.min(63) | self.fix << 6;
        b[23] = self.cn0;
        b
    }
}

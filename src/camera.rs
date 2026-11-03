//! Reads image packets from the ESP32 camera over SPI2 (PB12 CS, PB13 SCK, PB14 MISO, PB15 MOSI).
//! Each 256-byte transfer is `len`, one image packet, its wrapping byte sum; see `camera/`.
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{Spi, mode::Master};
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec;

use crate::image::MAX_PACKET;

/// After this long without a packet the camera counts as dead and stored images are sent instead
const DEAD_AFTER: Duration = Duration::from_secs(10);

pub struct Camera {
    pub spi: Spi<'static, Async, Master>,
    pub cs: Output<'static>,
    pub last_packet: Option<Instant>,
    pub last_warning: Option<Instant>,
}

impl Camera {
    /// Reads the next packet into `out`; false if the ESP32 had none ready or isn't connected.
    pub async fn next(&mut self, out: &mut Vec<u8, MAX_PACKET>) -> bool {
        let mut frame = [0u8; 256];
        self.cs.set_low();
        Timer::after_micros(10).await;
        let ok = self.spi.read(&mut frame).await.is_ok();
        self.cs.set_high();

        let len = frame[0] as usize;
        let sum = frame[1..].iter().take(len).fold(0u8, |sum, b| sum.wrapping_add(*b));
        let valid = ok && (1..=MAX_PACKET).contains(&len) && frame[1] == 0xA5 && frame[len + 1] == sum;
        if !valid {
            if self.last_warning.is_none_or(|t| t.elapsed() > Duration::from_secs(5)) {
                defmt::warn!("Camera: invalid SPI frame (SPI ok: {}), starts {:X}", ok, frame[..8]);
                self.last_warning = Some(Instant::now());
            }
            return false;
        }
        if !self.alive() {
            defmt::info!("Camera: packets arriving");
        }
        out.clear();
        let _ = out.extend_from_slice(&frame[1..=len]);
        self.last_packet = Some(Instant::now());
        true
    }

    pub fn alive(&self) -> bool {
        self.last_packet.is_some_and(|t| t.elapsed() < DEAD_AFTER)
    }
}

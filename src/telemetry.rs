use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    signal::Signal,
};

pub const PACKET_LEN: usize = 80;

#[derive(Clone, Copy, Default)]
pub struct Telemetry {
    pub heading: f32,
    pub roll: f32,
    pub pitch: f32,
    pub pressure_pa: f32,
    pub temperature_c: f32,
    pub gps_lat: f32,
    pub gps_lon: f32,
    pub gps_alt: f32,
    pub gps_sats: u8,
    pub gps_fix: bool,
}

impl defmt::Format for Telemetry {
    fn format(&self, fmt: defmt::Formatter) {
        if self.gps_fix {
            defmt::write!(
                fmt,
                "H:{=f32}° R:{=f32}° P:{=f32}° | {=u32}Pa {=f32}°C | GPS: {=f32}, {=f32} ({=f32}m, {} sats)",
                self.heading,
                self.roll,
                self.pitch,
                self.pressure_pa as u32,
                self.temperature_c,
                self.gps_lat,
                self.gps_lon,
                self.gps_alt,
                self.gps_sats,
            );
        } else {
            defmt::write!(
                fmt,
                "H:{=f32}° R:{=f32}° P:{=f32}° | {=u32}Pa {=f32}°C | GPS: NO FIX ({} sats)",
                self.heading,
                self.roll,
                self.pitch,
                self.pressure_pa as u32,
                self.temperature_c,
                self.gps_sats,
            );
        }
    }
}

pub static LATEST: Signal<CriticalSectionRawMutex, Telemetry> = Signal::new();

struct SliceWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> SliceWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
}

impl<'a> core::fmt::Write for SliceWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len().saturating_sub(self.pos);
        let to_copy = bytes.len().min(remaining);
        self.buf[self.pos..self.pos + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.pos += to_copy;
        Ok(())
    }
}

pub fn encode(packet: &mut [u8; PACKET_LEN], sequence: u8, telemetry: Telemetry) {
    use core::fmt::Write;
    let mut writer = SliceWriter::new(packet);
    let fix_char = if telemetry.gps_fix { '3' } else { '0' };
    let _ = write!(
        writer,
        "#{:03} H:{:.1} R:{:.1} P:{:.1} Pres:{}Pa T:{:.1}C Lat:{:.5} Lon:{:.5} Alt:{:.0}m S:{} Fix:{}",
        sequence,
        telemetry.heading,
        telemetry.roll,
        telemetry.pitch,
        telemetry.pressure_pa as u32,
        telemetry.temperature_c,
        telemetry.gps_lat,
        telemetry.gps_lon,
        telemetry.gps_alt,
        telemetry.gps_sats,
        fix_char
    );
    let pos = writer.pos.min(PACKET_LEN - 2);
    for i in pos..PACKET_LEN - 2 {
        packet[i] = b' ';
    }
    packet[PACKET_LEN - 2] = b'\r';
    packet[PACKET_LEN - 1] = b'\n';
}
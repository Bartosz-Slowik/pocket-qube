//! The link to the PC: COBS frames ending in 0x00 on USART1 (PA9, 921600 baud), first byte = type:
//! `T` + UTF-8 text line, `I` + crc_ok (u8) + RSSI (i16 LE) + image packet, `N` = new flight.
use embassy_stm32::mode::Blocking;
use embassy_stm32::usart::UartTx;
use heapless::Vec;

use crate::radio::Pkt;

pub type Uart = UartTx<'static, Blocking>;

/// COBS-encodes `kind`, `head` and `body` as one frame and ends it with a 0x00 delimiter.
pub fn send_frame(uart: &mut Uart, kind: u8, head: &[u8], body: &[u8]) {
    let mut out: Vec<u8, 320> = Vec::new();
    let mut code_at = 0;
    let _ = out.push(0);
    let mut code = 1u8;
    for &b in [kind].iter().chain(head).chain(body) {
        if b == 0 {
            out[code_at] = code;
            code_at = out.len();
            let _ = out.push(0);
            code = 1;
            continue;
        }
        let _ = out.push(b);
        code += 1;
        if code == 0xFF {
            out[code_at] = code;
            code_at = out.len();
            let _ = out.push(0);
            code = 1;
        }
    }
    out[code_at] = code;
    let _ = out.push(0);
    let _ = uart.blocking_write(&out);
}

/// An image packet with its reception quality, for the monitor to rebuild the photo.
pub fn send_image(uart: &mut Uart, data: &[u8], p: &Pkt) {
    let mut head = [p.crc_ok as u8, 0, 0];
    head[1..].copy_from_slice(&p.rssi.to_le_bytes());
    send_frame(uart, b'I', &head, data);
}

/// Decodes the satellite's 24-byte telemetry (layout in its `src/telemetry.rs`) into the text
/// line the monitor understands.
pub fn print_telemetry(uart: &mut Uart, data: &[u8], p: &Pkt) {
    let (rssi, snr) = (p.rssi, p.snr);
    let Ok(b) = <&[u8; 24]>::try_from(data) else {
        log!(uart, "Unknown LoRa packet ({} bytes): {:02X?} | RSSI {rssi}dBm", data.len(), &data[..data.len().min(32)]);
        return;
    };
    if !p.crc_ok {
        // Corrupted binary would show up as wild values in the graphs
        log!(uart, "Telemetry with CRC error dropped: {:02X?} | RSSI {rssi}dBm SNR {snr}dB", b);
        return;
    }
    let u16_at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
    let tenths = |i: usize| match u16_at(i) as i16 {
        i16::MIN => f32::NAN,
        v => v as f32 / 10.0,
    };
    let degrees = |i: usize| i32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]) as f64 / 1e7;
    let heading = match u16_at(1) {
        u16::MAX => f32::NAN,
        v => v as f32 / 10.0,
    };
    let pressure = match u32::from_le_bytes([b[7], b[8], b[9], 0]) {
        0xFF_FFFF => f32::NAN,
        v => v as f32,
    };
    let fix = match b[22] >> 6 {
        0 => '0',
        1 => '1',
        _ => 'X',
    };
    log!(
        uart,
        "#{:03} H{:.1} R{:.1} P{:.1} {:.0}Pa {:.1}C {:.5},{:.5} {}m S{} F{fix} {}dBHz | RSSI {rssi}dBm SNR {snr}dB",
        b[0],
        heading,
        tenths(3),
        tenths(5),
        pressure,
        tenths(10),
        degrees(12),
        degrees(16),
        u16_at(20),
        b[22] & 0x3F,
        b[23]
    );
}

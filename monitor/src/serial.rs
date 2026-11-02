//! Reads the receiver's USB-UART: COBS frames ending in 0x00, first byte = type
//! (`T` text line, `I` image packet, `N` new flight). Format in `receiver/src/main.rs`.
use std::io::{BufRead, BufReader, ErrorKind};
use std::sync::mpsc::Sender;
use std::thread::sleep;
use std::time::Duration;

use serialport::SerialPortType;

use crate::Msg;

/// Reconnects whenever the receiver is unplugged; runs until the program ends.
pub fn read(tx: Sender<Msg>) {
    let send = |msg: Msg| {
        let _ = tx.send(msg);
    };
    let text = |s: String| send(Msg::Text(s));
    loop {
        let Some(port) = std::env::args().skip(1).find(|a| !a.starts_with("--")).or_else(find_usb_port) else {
            let ports: Vec<_> = serialport::available_ports().unwrap_or_default().into_iter().map(|p| p.port_name).collect();
            text(format!("[monitor] Waiting for receiver USB... (visible ports: {ports:?})"));
            sleep(Duration::from_secs(1));
            continue;
        };

        match serialport::new(&port, 921_600).timeout(Duration::from_secs(3600)).open() {
            Ok(serial) => {
                text(format!("[monitor] Connected to {port}"));
                let mut reader = BufReader::new(serial);
                let mut frame = Vec::new();
                loop {
                    frame.clear();
                    match reader.read_until(0, &mut frame) {
                        Ok(0) => {
                            text("[monitor] Disconnected".into());
                            break;
                        }
                        Ok(_) => match cobs_decode(&frame).as_deref() {
                            Some([b'T', s @ ..]) => text(String::from_utf8_lossy(s).into_owned()),
                            Some([b'N', ..]) => send(Msg::NewFlight),
                            Some([b'I', crc_ok, r0, r1, data @ ..]) => send(Msg::Image {
                                data: data.to_vec(),
                                crc_ok: *crc_ok != 0,
                                rssi: i16::from_le_bytes([*r0, *r1]),
                            }),
                            // Garbage from joining mid-frame or a receiver reset
                            _ => {}
                        },
                        Err(e) if e.kind() == ErrorKind::TimedOut => continue,
                        Err(e) => {
                            text(format!("[monitor] Disconnected: {e}"));
                            break;
                        }
                    }
                }
            }
            Err(e) => text(format!("[monitor] Can't open {port}: {e} (unplugged, or open in another program?)")),
        }
        sleep(Duration::from_secs(1));
    }
}

/// Decodes one COBS frame (with or without its trailing 0x00).
fn cobs_decode(frame: &[u8]) -> Option<Vec<u8>> {
    let frame = frame.strip_suffix(&[0]).unwrap_or(frame);
    let mut out = Vec::with_capacity(frame.len());
    let mut i = 0;
    while i < frame.len() {
        let code = frame[i] as usize;
        if code == 0 || i + code > frame.len() {
            return None;
        }
        out.extend_from_slice(&frame[i + 1..i + code]);
        i += code;
        if code < 0xFF && i < frame.len() {
            out.push(0);
        }
    }
    Some(out)
}

/// The receiver's USB-UART; skips the camera's ESP32 (Espressif USB id 0x303A).
fn find_usb_port() -> Option<String> {
    serialport::available_ports()
        .ok()?
        .into_iter()
        .find(|p| matches!(&p.port_type, SerialPortType::UsbPort(u) if u.vid != 0x303A))
        .map(|p| p.port_name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cobs_decodes() {
        assert_eq!(cobs_decode(&[3, 0x11, 0x22, 2, 0x33, 0]), Some(vec![0x11, 0x22, 0, 0x33]));
        assert_eq!(cobs_decode(&[1, 1, 0]), Some(vec![0]));
        let mut long = vec![0xFF];
        long.extend(1..=254u8);
        long.extend([1, 0]);
        assert_eq!(cobs_decode(&long), Some((1..=254u8).collect()));
    }
}

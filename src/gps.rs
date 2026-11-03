use embassy_time::{Duration, Instant, with_deadline};
use embedded_io_async::{Read, Write};
use heapless::Vec;

#[derive(Default, defmt::Format)]
pub struct Fix {
    pub ok: bool,
    pub sats: u8,
    pub lat: f32,
    pub lon: f32,
    pub alt: f32,
    /// Satellites heard (C/N0 > 0) and the strongest C/N0 in dBHz, from the last second's GSV
    pub tracked: u8,
    pub best_cn0: u8,
    tracking: u8,
    best: u8,
}

#[derive(Default)]
pub struct Gps {
    pub fix: Fix,
    line: Vec<u8, 128>,
    last_rx: Option<Instant>,
}

impl Gps {
    /// UBX-CFG-VALSET into RAM: airborne <1g dynamic model (the default stops fixing above
    /// 12 km) and, once there's a fix, cyclic-tracking power save (it hampers acquisition with
    /// weak signals). RAM settings are lost if the GPS restarts, so this is sent repeatedly.
    pub async fn configure(uart: &mut impl Write, power_save: bool) {
        const DYNMODEL_AIRBORNE_1G: [u8; 5] = [0x21, 0x00, 0x11, 0x20, 6];
        // CFG-PM-OPERATEMODE: 0 full power, 2 cyclic tracking
        let operate_mode = [0x01, 0x00, 0xD0, 0x20, if power_save { 2 } else { 0 }];
        let mut msg: Vec<u8, 24> = Vec::new();
        let _ = msg.extend_from_slice(&[0xB5, 0x62, 0x06, 0x8A, 14, 0, 0, 1, 0, 0]);
        let _ = msg.extend_from_slice(&DYNMODEL_AIRBORNE_1G);
        let _ = msg.extend_from_slice(&operate_mode);
        let (mut a, mut b) = (0u8, 0u8);
        for &x in &msg[2..] {
            a = a.wrapping_add(x);
            b = b.wrapping_add(a);
        }
        let _ = msg.extend_from_slice(&[a, b]);
        if let Err(e) = uart.write_all(&msg).await {
            defmt::warn!("GPS config write failed: {}", defmt::Debug2Format(&e));
        }
    }

    /// Reads NMEA lines until `until`.
    pub async fn listen(&mut self, uart: &mut impl Read, until: Instant) {
        let mut buf = [0; 64];
        while Instant::now() < until {
            match with_deadline(until, uart.read(&mut buf)).await {
                Ok(Ok(n)) => {
                    buf[..n].iter().for_each(|&c| self.feed(c));
                    self.last_rx = Some(Instant::now());
                }
                Ok(Err(e)) => defmt::warn!("GPS UART error: {}", defmt::Debug2Format(&e)),
                Err(_) => {}
            }
        }
    }

    /// True if no GPS data arrived for 5 s (counted from boot at first); the GPS sends every second.
    pub fn silent(&self) -> bool {
        self.last_rx.unwrap_or(Instant::from_ticks(0)).elapsed() > Duration::from_secs(5)
    }

    fn feed(&mut self, c: u8) {
        if c == b'\n' {
            if let Ok(s) = core::str::from_utf8(&self.line) {
                self.fix.parse(s);
            }
            self.line.clear();
            return;
        }
        if c == b'$' || self.line.is_full() {
            self.line.clear();
        }
        let _ = self.line.push(c);
    }
}

impl Fix {
    /// Parses `$..GGA` and `$..GSV` sentences; everything else is ignored.
    fn parse(&mut self, line: &str) {
        let Some((body, sum)) = line.trim().strip_prefix('$').and_then(|s| s.split_once('*')) else {
            return;
        };
        if u8::from_str_radix(sum, 16) != Ok(body.bytes().fold(0, |a, b| a ^ b)) {
            return;
        }
        let f = |i| body.split(',').nth(i).unwrap_or("");
        if f(0).ends_with("GSV") {
            // Up to 4 satellites per sentence: PRN, elevation, azimuth, C/N0
            for cn0 in (0..4).filter_map(|k| f(7 + 4 * k).parse::<u8>().ok()).filter(|&c| c > 0) {
                self.tracking = self.tracking.saturating_add(1);
                self.best = self.best.max(cn0);
            }
            return;
        }
        if !f(0).ends_with("GGA") {
            return;
        }
        // GGA starts each second's output, so the GSV counts so far cover the previous second
        (self.tracked, self.best_cn0) = (self.tracking, self.best);
        (self.tracking, self.best) = (0, 0);
        self.ok = f(6).parse().unwrap_or(0u8) > 0;
        self.sats = f(7).parse().unwrap_or(0);
        self.lat = coord(f(2), f(3)).unwrap_or(self.lat);
        self.lon = coord(f(4), f(5)).unwrap_or(self.lon);
        self.alt = f(9).parse().unwrap_or(self.alt);
    }
}

/// NMEA `dddmm.mmmm` + hemisphere -> signed decimal degrees.
fn coord(value: &str, dir: &str) -> Option<f32> {
    let split = value.find('.')?.checked_sub(2)?;
    let deg: f32 = value.get(..split)?.parse().ok()?;
    let min: f32 = value.get(split..)?.parse().ok()?;
    let x = deg + min / 60.0;
    Some(if dir == "S" || dir == "W" { -x } else { x })
}

//! The receiver's decoded telemetry line (see `print_telemetry` in `receiver/src/main.rs`).

pub struct Telemetry {
    pub seq: String,
    pub heading: f32,
    pub roll: f32,
    pub pitch: f32,
    pub pressure: f32,
    pub temp: f32,
    pub lat: f32,
    pub lon: f32,
    pub alt: f32,
    pub sats: String,
    pub fix: String,
    /// Strongest GPS satellite, dBHz
    pub cn0: f32,
    pub radio: String,
}

impl Telemetry {
    /// Parses `[CRC! ]#014 H183.0 R3.4 P3.9 99798Pa 26.1C 50.09921,19.95716 233m S12 F1 38dBHz | RSSI ..`
    pub fn parse(line: &str) -> Option<Telemetry> {
        let line = line.strip_prefix("CRC! ").unwrap_or(line);
        let (data, radio) = line.split_once(" | ")?;
        let seq = data.split_whitespace().next()?.strip_prefix('#')?;
        let num = |s: &str| s.parse::<f32>().unwrap_or(f32::NAN);
        let mut t = Telemetry {
            seq: seq.to_string(),
            heading: f32::NAN,
            roll: f32::NAN,
            pitch: f32::NAN,
            pressure: f32::NAN,
            temp: f32::NAN,
            lat: f32::NAN,
            lon: f32::NAN,
            alt: f32::NAN,
            sats: String::new(),
            fix: String::new(),
            cn0: f32::NAN,
            radio: radio.to_string(),
        };
        for tok in data.split_whitespace().skip(1) {
            // Suffixes first so e.g. "99798Pa" isn't read as pitch
            if let Some(v) = tok.strip_suffix("dBHz") {
                t.cn0 = num(v);
            } else if let Some(v) = tok.strip_suffix("Pa") {
                t.pressure = num(v);
            } else if let Some(v) = tok.strip_suffix('C') {
                t.temp = num(v);
            } else if let Some(v) = tok.strip_suffix('m') {
                t.alt = num(v);
            } else if let Some((a, b)) = tok.split_once(',') {
                (t.lat, t.lon) = (num(a), num(b));
            } else if let Some(v) = tok.strip_prefix('H') {
                t.heading = num(v);
            } else if let Some(v) = tok.strip_prefix('R') {
                t.roll = num(v);
            } else if let Some(v) = tok.strip_prefix('P') {
                t.pitch = num(v);
            } else if let Some(v) = tok.strip_prefix('S') {
                t.sats = v.to_string();
            } else if let Some(v) = tok.strip_prefix('F') {
                t.fix = v.to_string();
            }
        }
        Some(t)
    }

    pub fn has_fix(&self) -> bool {
        self.fix == "1"
    }

    /// From the receiver's "RSSI -27dBm SNR 15dB".
    pub fn rssi(&self) -> Option<f32> {
        radio_value(&self.radio, "RSSI ", "dBm")
    }

    pub fn snr(&self) -> Option<f32> {
        radio_value(&self.radio, "SNR ", "dB")
    }
}

/// The number between `before` and `after`.
fn radio_value(radio: &str, before: &str, after: &str) -> Option<f32> {
    radio.split(before).nth(1)?.split(after).next()?.trim().parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_line() {
        let t = Telemetry::parse("#014 H183.0 R3.4 P3.9 99798Pa 26.1C 50.09921,19.95716 233m S12 F1 38dBHz | RSSI -27dBm SNR 15dB")
            .unwrap();
        assert_eq!((t.seq.as_str(), t.alt, t.pressure, t.lat, t.cn0), ("014", 233.0, 99798.0, 50.09921, 38.0));
        assert!(t.has_fix());
        assert_eq!((t.rssi(), t.snr()), (Some(-27.0), Some(15.0)));
    }
}

//! Uploads GPS positions to SondeHub Amateur (https://amateur.sondehub.org), the public balloon
//! tracking map, when `sondehub.json` holds your callsigns (see `sondehub.example.json`).
//! Uploads are batched, and kept for a while when offline.
//! API: https://github.com/projecthorus/sondehub-infra/wiki/API-(Beta)
use std::sync::mpsc::Receiver;
use std::thread::sleep;
use std::time::{Duration, Instant};

use serde_json::{Value, json};

const FILE: &str = "sondehub.json";
const API: &str = "https://api.v2.sondehub.org/amateur";
const SOFTWARE: &str = "pocketqube-monitor";
const UPLOAD_EVERY: Duration = Duration::from_secs(15);
/// The station marker on the map needs refreshing now and then
const LISTENER_EVERY: Duration = Duration::from_secs(30 * 60);
/// Packets kept while offline (~20 min); older ones are dropped
const MAX_QUEUE: usize = 400;
/// Must match the satellite's LoRa telemetry frequency
const FREQUENCY_MHZ: f64 = 869.7;

pub struct Config {
    payload: String,
    uploader: String,
    /// [lat, lon, alt] of the ground station, to show it on the map
    position: Option<Value>,
    /// Test mode: SondeHub checks the data but doesn't store or show it
    dev: bool,
}

impl Config {
    /// Reads `sondehub.json`; the error says why uploading is off.
    pub fn load() -> Result<Config, String> {
        let text = std::fs::read_to_string(FILE)
            .map_err(|_| format!("Upload off: copy sondehub.example.json to {FILE} and set your callsigns"))?;
        let v: Value = serde_json::from_str(&text).map_err(|e| format!("Upload off, {FILE}: {e}"))?;
        let field = |k: &str| {
            v[k].as_str()
                .filter(|s| !s.is_empty() && !s.starts_with("CHANGE-ME"))
                .map(String::from)
                .ok_or(format!("Upload off: set \"{k}\" in {FILE}"))
        };
        Ok(Config {
            payload: field("payload_callsign")?,
            uploader: field("uploader_callsign")?,
            position: v.get("uploader_position").filter(|p| p.is_array()).cloned(),
            dev: v["dev"].as_bool().unwrap_or(false),
        })
    }
}

enum Outcome {
    Done,
    /// SondeHub refused the data; resending won't help
    Rejected(String),
    /// No internet or server busy
    Retry(String),
}

/// Uploads telemetry rows (as sent to the web page) until the program ends.
pub fn upload(rows: Receiver<Value>, config: Config, report: impl Fn(String)) {
    let mode = if config.dev { " (test mode: checked but not stored)" } else { "" };
    report(format!("Uploading positions as {}{mode}", config.payload));
    let mut queue: Vec<Value> = Vec::new();
    let mut listener_sent: Option<Instant> = None;
    let mut failing = false;
    loop {
        sleep(UPLOAD_EVERY);
        queue.extend(rows.try_iter().filter_map(|row| packet(&row, &config)));
        let excess = queue.len().saturating_sub(MAX_QUEUE);
        queue.drain(..excess);

        if !config.dev && config.position.is_some() && listener_sent.is_none_or(|t| t.elapsed() >= LISTENER_EVERY) {
            if let Outcome::Done = put("listeners", &listener(&config)) {
                listener_sent = Some(Instant::now());
            }
        }
        if queue.is_empty() {
            continue;
        }
        match put("telemetry", &Value::Array(queue.clone())) {
            Outcome::Done => {
                if failing {
                    report(format!("Upload working again, sent {} positions", queue.len()));
                }
                failing = false;
                queue.clear();
            }
            Outcome::Rejected(why) => {
                report(format!("SondeHub rejected {} positions: {why}", queue.len()));
                queue.clear();
            }
            Outcome::Retry(why) => {
                if !failing {
                    report(format!("Upload failed ({why}), keeping positions and retrying"));
                }
                failing = true;
            }
        }
    }
}

/// SondeHub's amateur telemetry format; only rows with a GPS fix have a position to show.
fn packet(row: &Value, c: &Config) -> Option<Value> {
    let (lat, lon) = (row["lat"].as_f64()?, row["lon"].as_f64()?);
    if row["fix"] != "1" || (lat == 0.0 && lon == 0.0) {
        return None;
    }
    // Telemetry has no GPS time, so the receive time stands in for it
    let time = iso8601(row["time"].as_u64()?);
    let mut p = json!({
        "software_name": SOFTWARE,
        "software_version": env!("CARGO_PKG_VERSION"),
        "uploader_callsign": c.uploader,
        "time_received": time,
        "payload_callsign": c.payload,
        "datetime": time,
        "lat": lat,
        "lon": lon,
        "alt": row["alt"].as_f64()?,
        "frequency": FREQUENCY_MHZ,
        "temp": row["temp"],
        "pressure": row["pressure"].as_f64().map(|pa| pa / 100.0),
        "sats": row["sats"].as_str().and_then(|s| s.parse::<u32>().ok()),
        "rssi": row["rssi"],
        "snr": row["snr"],
        "uploader_position": c.position,
        "dev": c.dev.then_some("true"),
    });
    p.as_object_mut()?.retain(|_, v| !v.is_null());
    Some(p)
}

fn listener(c: &Config) -> Value {
    json!({
        "software_name": SOFTWARE,
        "software_version": env!("CARGO_PKG_VERSION"),
        "uploader_callsign": c.uploader,
        "uploader_position": c.position,
        "uploader_radio": "LR2021 (E80-900MBL-02)",
        "mobile": false,
    })
}

fn put(path: &str, body: &Value) -> Outcome {
    let response = ureq::put(&format!("{API}/{path}"))
        .config()
        .http_status_as_error(false)
        .build()
        .header("User-Agent", concat!("pocketqube-monitor-", env!("CARGO_PKG_VERSION")))
        .header("Content-Type", "application/json")
        .send(body.to_string());
    let mut response = match response {
        Ok(r) => r,
        Err(e) => return Outcome::Retry(e.to_string()),
    };
    let status = response.status().as_u16();
    let text = response.body_mut().read_to_string().unwrap_or_default();
    match status {
        200..=299 => Outcome::Done,
        500.. => Outcome::Retry(format!("server busy, {status}")),
        _ => Outcome::Rejected(format!("{status} {text}")),
    }
}

/// Unix seconds as `YYYY-MM-DDTHH:MM:SSZ`.
fn iso8601(secs: u64) -> String {
    // Days since 1970 to a calendar date (Howard Hinnant's civil_from_days)
    let z = (secs / 86400) as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + (month <= 2) as i64;
    let s = secs % 86400;
    format!("{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z", s / 3600, s / 60 % 60, s % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats_dates() {
        assert_eq!(iso8601(0), "1970-01-01T00:00:00Z");
        assert_eq!(iso8601(1_700_000_000), "2023-11-14T22:13:20Z");
        assert_eq!(iso8601(951_782_400), "2000-02-29T00:00:00Z");
    }
}

//! Ground station: reads the receiver's USB-UART, rebuilds the images, logs everything and serves
//! it as a web page for phones and laptops (see `web.rs`), and uploads positions to SondeHub
//! (see `sondehub.rs`). Stop it with Ctrl+C.
//! Usage: `cargo run --release` (auto-detects the port) or `cargo run --release -- COM5`.
//! After a restart it continues the last flight's logs; add `--new` to start a new flight.
use std::sync::mpsc::{Sender, channel};
use std::thread::spawn;
use std::time::{Duration, Instant};

use serde_json::json;

use image_rx::{ImageRx, to_jpeg};
use telemetry::Telemetry;

mod image_rx;
mod serial;
mod sondehub;
mod telemetry;
mod web;

/// Encoding the live image for the page costs ~15 ms, so it isn't done for every packet
const LIVE_IMAGE_EVERY: Duration = Duration::from_millis(500);

/// What the other threads send to the main loop.
pub enum Msg {
    Text(String),
    Image { data: Vec<u8>, crc_ok: bool, rssi: i16 },
    /// Status line from the web page, tunnel or SondeHub uploader, already tagged
    Status(String),
    /// KEY2 on the receiver: start a new flight log
    NewFlight,
}

fn main() {
    // images/, logs/ and tiles/ go next to the monitor's source wherever it's started from
    let _ = std::env::set_current_dir(env!("CARGO_MANIFEST_DIR"));
    let (tx, rx) = channel();
    let (web_tx, web_rx) = channel();
    let report = |tx: Sender<Msg>, tag: &'static str| {
        move |text: String| {
            let _ = tx.send(Msg::Status(format!("[{tag}] {text}")));
        }
    };
    let web_report = report(tx.clone(), "web");
    let new_flight = std::env::args().any(|a| a == "--new");
    spawn(move || web::serve(web_rx, new_flight, web_report));
    let tunnel_report = report(tx.clone(), "web");
    spawn(move || web::tunnel(tunnel_report));
    report(tx.clone(), "web")(web::local_link());
    let sondehub = match sondehub::Config::load() {
        Ok(config) => {
            let (sondehub_tx, sondehub_rx) = channel();
            let sondehub_report = report(tx.clone(), "sondehub");
            spawn(move || sondehub::upload(sondehub_rx, config, sondehub_report));
            Some(sondehub_tx)
        }
        Err(why) => {
            report(tx.clone(), "sondehub")(why);
            None
        }
    };
    spawn(move || serial::read(tx));

    let mut station = Station {
        web: web_tx,
        sondehub,
        telemetry: None,
        sea_level_pa: None,
        image: ImageRx::default(),
        rssi: 0,
        link: LinkStats::default(),
        last_live_image: None,
        last_line: String::new(),
    };
    for msg in rx {
        station.handle(msg);
    }
}

struct Station {
    web: Sender<web::Update>,
    /// None when SondeHub uploading is off
    sondehub: Option<Sender<serde_json::Value>>,
    telemetry: Option<Telemetry>,
    /// Barometric altitude is calibrated to the first GPS fix, like on the satellite
    sea_level_pa: Option<f32>,
    image: ImageRx,
    /// Of the latest image packet
    rssi: i16,
    link: LinkStats,
    last_live_image: Option<Instant>,
    last_line: String,
}

/// Link quality between two telemetry packets; sent with the second one.
#[derive(Default)]
struct LinkStats {
    last_seq: Option<u8>,
    flrc_ok: u32,
    flrc_bad: u32,
    flrc_rssi_sum: f32,
}

impl LinkStats {
    /// Average over the cycle's image packets
    fn flrc_rssi(&self) -> Option<f32> {
        let packets = self.flrc_ok + self.flrc_bad;
        (packets > 0).then(|| self.flrc_rssi_sum / packets as f32)
    }
}

impl Station {
    fn handle(&mut self, msg: Msg) {
        let line = match msg {
            // Too many to log; the image status shows progress
            Msg::Image { data, crc_ok, rssi } => {
                self.image.add(&data, crc_ok);
                self.rssi = rssi;
                match crc_ok {
                    true => self.link.flrc_ok += 1,
                    false => self.link.flrc_bad += 1,
                }
                self.link.flrc_rssi_sum += rssi as f32;
                self.send_image();
                return;
            }
            Msg::Status(text) => text,
            Msg::NewFlight => {
                let _ = self.web.send(web::Update::NewFlight);
                (self.telemetry, self.sea_level_pa, self.link) = (None, None, LinkStats::default());
                "[monitor] New flight log (KEY2 on the receiver)".to_string()
            }
            Msg::Text(line) => line,
        };
        if line == self.last_line {
            return;
        }
        println!("{line}");
        if let Some(t) = Telemetry::parse(&line) {
            if t.has_fix() && t.alt.is_finite() && t.pressure.is_finite() && self.sea_level_pa.is_none() {
                self.sea_level_pa = Some(t.pressure / (1.0 - t.alt / 44330.0).powf(5.255));
            }
            let p0 = self.sea_level_pa.unwrap_or(101_325.0);
            let baro_alt = t.pressure.is_finite().then(|| 44330.0 * (1.0 - (t.pressure / p0).powf(0.1903)));
            let row = json!({
                "time": web::unix_secs(), "seq": t.seq, "heading": t.heading, "roll": t.roll,
                "pitch": t.pitch, "pressure": t.pressure, "temp": t.temp, "lat": t.lat, "lon": t.lon,
                "alt": t.alt, "baro_alt": baro_alt, "sats": t.sats, "fix": t.fix, "cn0": t.cn0, "radio": t.radio,
                "rssi": t.rssi(), "snr": t.snr(), "lora_lost": self.lora_lost(&t),
                "flrc_ok": self.link.flrc_ok, "flrc_bad": self.link.flrc_bad, "flrc_rssi": self.link.flrc_rssi(),
                "blocks_got": self.image.blocks_got, "blocks_lost": self.image.blocks_lost,
            });
            self.link = LinkStats { last_seq: self.link.last_seq, ..LinkStats::default() };
            (self.image.blocks_got, self.image.blocks_lost) = (0, 0);
            if let Some(sondehub) = &self.sondehub {
                let _ = sondehub.send(row.clone());
            }
            let _ = self.web.send(web::Update::Telemetry(row));
            self.telemetry = Some(t);
        }
        let _ = self.web.send(web::Update::Log(line.clone()));
        self.last_line = line;
    }

    /// Telemetry packets missed since the previous one, from the satellite's sequence number
    /// (counts mod 256; a big jump is a satellite restart, not loss).
    fn lora_lost(&mut self, t: &Telemetry) -> u8 {
        let Ok(seq) = t.seq.parse::<u8>() else { return 0 };
        let gap = self.link.last_seq.map_or(0, |last| seq.wrapping_sub(last).wrapping_sub(1));
        self.link.last_seq = Some(seq);
        if gap < 100 { gap } else { 0 }
    }

    /// Passes saved photos and the live image to the web page.
    fn send_image(&mut self) {
        let saved = self.image.saved.take();
        if let Some((name, complete)) = &saved {
            let t = self.telemetry.as_ref().filter(|t| t.has_fix());
            let _ = self.web.send(web::Update::Photo(json!({
                "name": name, "time": web::unix_secs(), "partial": *complete < 100, "complete": complete,
                "alt": t.map(|t| t.alt), "lat": t.map(|t| t.lat), "lon": t.map(|t| t.lon),
            })));
        }
        // A finished image goes out at once, one in progress only every LIVE_IMAGE_EVERY
        let due = self.last_live_image.is_none_or(|t| t.elapsed() >= LIVE_IMAGE_EVERY);
        if self.image.dirty && (due || saved.is_some()) {
            if let Some(img) = &self.image.rgba {
                let status = format!("{} | RSSI {} dBm", self.image.status, self.rssi);
                let _ = self.web.send(web::Update::Image(to_jpeg(img, 70), status));
            }
            self.image.dirty = false;
            self.last_live_image = Some(Instant::now());
        }
    }
}

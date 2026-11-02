//! Web page for phones and laptops, served on port 8080: telemetry, attitude, map, graphs, the
//! live image and a gallery of the run's photos. Map tiles go through a disk cache, so areas
//! viewed once also work offline. With `cloudflared` installed, a Cloudflare quick tunnel adds a
//! public link; it runs in the same console, so Ctrl+C stops it with the monitor.
use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc::Receiver;
use std::thread::{sleep, spawn};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use serde_json::{Value, json};
use tiny_http::{Header, Request, Response, Server};

pub const PORT: u16 = 8080;
const CSV_KEYS: [&str; 23] = [
    "time", "seq", "heading", "roll", "pitch", "pressure", "temp", "lat", "lon", "alt", "baro_alt", "sats", "fix",
    "cn0", "radio", "rssi", "snr", "lora_lost", "flrc_ok", "flrc_bad", "flrc_rssi", "blocks_got", "blocks_lost",
];
/// A restart within this long after the last log line continues that flight's logs
const RESUME_WITHIN: Duration = Duration::from_secs(6 * 3600);
const LOG_LINES: usize = 50;

/// What the window sends to the web thread.
pub enum Update {
    /// One telemetry row with the `CSV_KEYS` fields
    Telemetry(Value),
    Log(String),
    /// Latest live image as JPEG, and its status line
    Image(Vec<u8>, String),
    /// A photo saved in `images/`: name, time and where it was taken
    Photo(Value),
    /// Start a new flight folder; the page then only shows the new flight
    NewFlight,
}

pub fn unix_secs() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs())
}

/// Serves the page until the program ends; `report` shows problems in the console.
/// Everything is also written to the flight's folder in `logs/`, and reloaded from it after a
/// restart, unless `new_flight`.
pub fn serve(updates: Receiver<Update>, new_flight: bool, report: impl Fn(String)) {
    let server = match Server::http(("0.0.0.0", PORT)) {
        Ok(s) => s,
        Err(e) => return report(format!("Web page unavailable, port {PORT}: {e}")),
    };
    let mut flight = Flight::open(flight_dir(new_flight), &report);
    let (mut image, mut image_status, mut image_version) = (Vec::new(), String::new(), 0u32);

    loop {
        for update in updates.try_iter() {
            match update {
                Update::Telemetry(row) => {
                    append(&mut flight.telemetry_file, &row.to_string());
                    let cells: Vec<String> = CSV_KEYS.iter().map(|k| csv_cell(&row[k])).collect();
                    append(&mut flight.csv, &cells.join(","));
                    flight.telemetry.push(row);
                }
                Update::Log(line) => {
                    let s = unix_secs();
                    let line = format!("{:02}:{:02}:{:02}Z {line}", s / 3600 % 24, s / 60 % 60, s % 60);
                    append(&mut flight.log_file, &line);
                    flight.log.push_back(line);
                    if flight.log.len() > LOG_LINES {
                        flight.log.pop_front();
                    }
                }
                Update::Image(jpeg, status) => (image, image_status, image_version) = (jpeg, status, image_version + 1),
                Update::Photo(info) => {
                    append(&mut flight.photos_file, &info.to_string());
                    flight.photos.push(info);
                }
                // The old flight's folder stays on disk
                Update::NewFlight => flight = Flight::open(flight_dir(true), &report),
            }
        }

        let Ok(Some(request)) = server.recv_timeout(Duration::from_millis(100)) else { continue };
        let url = request.url().to_string();
        let (path, query) = url.split_once('?').unwrap_or((&url, ""));
        match path {
            "/" => respond(request, include_bytes!("../web/index.html").to_vec(), "text/html; charset=utf-8"),
            "/leaflet.js" => respond(request, include_bytes!("../web/leaflet.js").to_vec(), "text/javascript"),
            "/leaflet.css" => respond(request, include_bytes!("../web/leaflet.css").to_vec(), "text/css"),
            // The page asks for everything after what it already has
            "/api/state" => {
                let state = json!({
                    "flight": flight.dir.file_name().map(|n| n.to_string_lossy()),
                    "telemetry": flight.telemetry.get(param(query, "t")..).unwrap_or(&[]),
                    "t": flight.telemetry.len(),
                    "photos": flight.photos.get(param(query, "p")..).unwrap_or(&[]),
                    "p": flight.photos.len(),
                    "log": flight.log,
                    "image": { "version": image_version, "status": image_status },
                });
                respond(request, state.to_string().into_bytes(), "application/json")
            }
            "/api/image.jpg" => respond(request, image.clone(), "image/jpeg"),
            // Files and tiles are slow, so they don't hold up the page's other requests
            _ if path.starts_with("/photos/") => {
                let name = path["/photos/".len()..].to_string();
                spawn(move || photo(request, &name));
            }
            _ if path.starts_with("/tiles/") => {
                let tile_path = path.to_string();
                spawn(move || tile(request, &tile_path));
            }
            _ => not_found(request),
        }
    }
}

/// Starts a Cloudflare quick tunnel (no account needed) and reports its public link; retries
/// while there's no internet. The link changes with every new tunnel.
pub fn tunnel(report: impl Fn(String)) {
    let mut warned = false;
    loop {
        let child = Command::new("cloudflared")
            // HTTP/2 over TCP: many networks block the default QUIC (UDP 7844).
            // Not "localhost", which may resolve to IPv6 while the server listens on IPv4.
            .args(["tunnel", "--no-autoupdate", "--protocol", "http2", "--url", &format!("http://127.0.0.1:{PORT}")])
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn();
        let Ok(mut child) = child else {
            return report("No public link: cloudflared isn't installed (winget install Cloudflare.cloudflared)".into());
        };
        let stderr = child.stderr.take().expect("stderr is piped");
        let mut link = None;
        for line in BufReader::new(stderr).lines().map_while(Result::ok) {
            if let Some(url) = line.split_whitespace().find(|w| w.starts_with("https://") && w.ends_with(".trycloudflare.com")) {
                link = Some(url.to_string());
            }
            // The link only works once the tunnel has connected
            if line.contains("Registered tunnel connection")
                && let Some(url) = &link
            {
                report(format!("Public link: {url}"));
                warned = false;
            }
        }
        let _ = child.wait();
        if !warned {
            report("Public link down (no internet?), retrying every 30 s".into());
            warned = true;
        }
        sleep(Duration::from_secs(30));
    }
}

/// This machine's address on the local network, found by asking the OS which interface it
/// would use for the internet (nothing is sent).
pub fn local_link() -> String {
    let ip = std::net::UdpSocket::bind("0.0.0.0:0")
        .and_then(|s| s.connect("8.8.8.8:80").and(s.local_addr()))
        .map_or("<this computer's IP>".to_string(), |a| a.ip().to_string());
    format!("Local link: http://{ip}:{PORT}")
}

fn photo(request: Request, name: &str) {
    // Only plain file names, so nothing outside images/ can be read
    let safe = !name.is_empty() && !name.starts_with('.') && name.chars().all(|c| c.is_ascii_alphanumeric() || "-_.".contains(c));
    match std::fs::read(format!("images/{name}")) {
        Ok(data) if safe => respond(request, data, "image/jpeg"),
        _ => not_found(request),
    }
}

/// OpenStreetMap tile from the disk cache, fetched and cached on first use.
fn tile(request: Request, path: &str) {
    let nums: Vec<u32> =
        path["/tiles/".len()..].trim_end_matches(".png").split('/').filter_map(|s| s.parse().ok()).collect();
    let [z, x, y] = nums[..] else { return not_found(request) };
    let file = format!("tiles/{z}/{x}/{y}.png");
    let data = std::fs::read(&file).ok().or_else(|| {
        let url = format!("https://tile.openstreetmap.org/{z}/{x}/{y}.png");
        // OpenStreetMap's tile policy requires an identifying User-Agent
        let mut response = ureq::get(&url).header("User-Agent", "PocketQube-monitor/0.1").call().ok()?;
        let data = response.body_mut().read_to_vec().ok()?;
        let _ = std::fs::create_dir_all(format!("tiles/{z}/{x}")).and_then(|()| std::fs::write(&file, &data));
        Some(data)
    });
    match data {
        Some(data) => respond(request, data, "image/png"),
        None => not_found(request),
    }
}

/// One flight: what the page shows, and the files in its `logs/` folder.
struct Flight {
    dir: PathBuf,
    telemetry: Vec<Value>,
    photos: Vec<Value>,
    log: VecDeque<String>,
    telemetry_file: Option<File>,
    photos_file: Option<File>,
    log_file: Option<File>,
    csv: Option<File>,
}

impl Flight {
    /// Opens `dir`, loading whatever an earlier run already logged there.
    fn open(dir: PathBuf, report: &impl Fn(String)) -> Self {
        let old_log = std::fs::read_to_string(dir.join("log.txt")).unwrap_or_default();
        let old_log: Vec<&str> = old_log.lines().collect();
        let open = |name: &str| {
            let file = std::fs::create_dir_all(&dir).and_then(|()| OpenOptions::new().create(true).append(true).open(dir.join(name)));
            file.inspect_err(|e| report(format!("Can't write {}: {e}", dir.join(name).display()))).ok()
        };
        let mut flight = Flight {
            telemetry: read_jsonl(&dir.join("telemetry.jsonl")),
            photos: read_jsonl(&dir.join("photos.jsonl")),
            log: old_log[old_log.len().saturating_sub(LOG_LINES)..].iter().map(|l| l.to_string()).collect(),
            telemetry_file: open("telemetry.jsonl"),
            photos_file: open("photos.jsonl"),
            log_file: open("log.txt"),
            csv: open("telemetry.csv"),
            dir: dir.clone(),
        };
        if flight.csv.as_ref().is_some_and(|f| f.metadata().is_ok_and(|m| m.len() == 0)) {
            append(&mut flight.csv, &CSV_KEYS.join(","));
        }
        let (rows, photos) = (flight.telemetry.len(), flight.photos.len());
        report(match rows == 0 && photos == 0 {
            true => format!("Logging to {}", dir.display()),
            false => format!("Continuing {}: {rows} telemetry rows, {photos} photos", dir.display()),
        });
        flight
    }
}

/// The latest flight's folder if it was written to recently, otherwise a new one.
fn flight_dir(new_flight: bool) -> PathBuf {
    // Folders are named by their start time in unix seconds
    let latest = std::fs::read_dir("logs").into_iter().flatten().flatten().filter(|e| e.path().is_dir()).max_by_key(|e| e.file_name());
    if !new_flight && let Some(latest) = latest {
        let modified = std::fs::metadata(latest.path().join("log.txt")).and_then(|m| m.modified());
        if modified.is_ok_and(|t| t.elapsed().is_ok_and(|age| age < RESUME_WITHIN)) {
            return latest.path();
        }
    }
    PathBuf::from(format!("logs/{}", unix_secs()))
}

/// One JSON value per line; a line cut short by a crash is skipped.
fn read_jsonl(path: &Path) -> Vec<Value> {
    let text = std::fs::read_to_string(path).unwrap_or_default();
    text.lines().filter_map(|line| serde_json::from_str(line).ok()).collect()
}

fn append(file: &mut Option<File>, line: &str) {
    if let Some(f) = file {
        let _ = writeln!(f, "{line}");
    }
}

fn param(query: &str, name: &str) -> usize {
    query
        .split('&')
        .find_map(|kv| kv.strip_prefix(name)?.strip_prefix('=')?.parse().ok())
        .unwrap_or(0)
}

fn csv_cell(v: &Value) -> String {
    match v {
        Value::Null => String::new(),
        Value::String(s) => s.replace(',', ";"),
        other => other.to_string(),
    }
}

fn respond(request: Request, body: Vec<u8>, content_type: &str) {
    let header = Header::from_bytes("Content-Type", content_type).expect("valid header");
    let _ = request.respond(Response::from_data(body).with_header(header));
}

fn not_found(request: Request) {
    let _ = request.respond(Response::from_string("not found").with_status_code(404));
}

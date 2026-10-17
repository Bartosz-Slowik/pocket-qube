//! Ground station GUI: telemetry, 3D attitude and the live image from the receiver's USB-UART.
//! Usage: `cargo run --release` (auto-detects the port) or `cargo run --release -- COM5`.
use std::collections::VecDeque;
use std::io::{BufRead, BufReader, ErrorKind};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::sleep;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use eframe::egui::{self, Align2, Color32, FontId, Stroke, TextureOptions, pos2, vec2};
use image::ImageFormat;
use serialport::SerialPortType;
use walkers::{HttpTiles, Map, MapMemory, Plugin, Position, Projector, lon_lat, sources::OpenStreetMap};

/// One all-grey 16x16 block (4:2:0, standard Huffman tables); keeps the JPEG decodable until
/// `conceal` paints over it.
const GREY_BLOCK: [u8; 4] = [0x28, 0xA2, 0x8A, 0x00];

fn main() -> eframe::Result {
    let (tx, rx) = channel();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1200.0, 900.0]),
        ..Default::default()
    };
    eframe::run_native(
        "PocketQube monitor",
        options,
        Box::new(|cc| {
            let ctx = cc.egui_ctx.clone();
            std::thread::spawn(move || read_serial(tx, ctx));
            Ok(Box::new(Monitor {
                rx,
                log: VecDeque::new(),
                telemetry: None,
                history: History::default(),
                image: ImageRx::default(),
                tiles: HttpTiles::new(OpenStreetMap, cc.egui_ctx.clone()),
                map: MapMemory::default(),
            }))
        }),
    )
}

struct Monitor {
    rx: Receiver<String>,
    log: VecDeque<String>,
    telemetry: Option<Telemetry>,
    history: History,
    image: ImageRx,
    tiles: HttpTiles,
    map: MapMemory,
}

impl eframe::App for Monitor {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        for line in self.rx.try_iter().collect::<Vec<_>>() {
            self.handle(line, ui.ctx());
        }

        egui::CentralPanel::default().show(ui, |ui| {
            ui.columns(2, |cols| {
                let t = self.telemetry.as_ref();
                let h = &self.history;
                let (blue, orange, red) =
                    (Color32::from_rgb(70, 130, 220), Color32::from_rgb(230, 120, 30), Color32::from_rgb(220, 80, 80));
                egui::ScrollArea::vertical().id_salt("left").show(&mut cols[0], |ui| {
                    ui.heading("Attitude");
                    draw_model(ui, t.map_or([f32::NAN; 3], |t| [t.heading, t.roll, t.pitch]));
                    ui.heading("Telemetry");
                    telemetry_grid(ui, t);
                    ui.heading("Altitude");
                    graph(ui, "m", &[("GPS", blue, &h.gps_alt), ("Baro", orange, &h.baro_alt)]);
                    ui.heading("Temperature");
                    graph(ui, "C", &[("Temp", red, &h.temp)]);
                });

                cols[1].heading("Map");
                // Before the first fix, centre on Krakow
                let here = h.track.last().copied().unwrap_or(lon_lat(19.94, 50.06));
                let width = cols[1].available_width();
                cols[1].allocate_ui(vec2(width, 250.0), |ui| {
                    ui.add(Map::new(Some(&mut self.tiles), &mut self.map, here).with_plugin(Track(h.track.clone())));
                });
                cols[1].horizontal(|ui| {
                    ui.small("© OpenStreetMap contributors");
                    if self.map.detached().is_some() && ui.small_button("Follow").clicked() {
                        self.map.follow_my_position();
                    }
                });

                cols[1].heading("Image");
                cols[1].label(&self.image.status);
                if let Some(tex) = &self.image.texture {
                    // Small images are zoomed up to 4x, large ones shrunk to the column width
                    let scale = (cols[1].available_width() / tex.size_vec2().x).min(4.0);
                    cols[1].add(egui::Image::new(tex).fit_to_exact_size(tex.size_vec2() * scale));
                }
                cols[1].heading("Log");
                egui::ScrollArea::vertical().stick_to_bottom(true).show(&mut cols[1], |ui| {
                    for line in &self.log {
                        ui.monospace(line);
                    }
                });
            });
        });
    }
}

impl Monitor {
    fn handle(&mut self, line: String, ctx: &egui::Context) {
        let mut shown = line.clone();
        if let Some((kind, rest)) = line.split_once(' ').filter(|(k, _)| *k == "BIN" || *k == "BAD") {
            let (hex, radio) = rest.split_once(" | ").unwrap_or((rest, ""));
            let bytes: Option<Vec<u8>> = (0..hex.len())
                .step_by(2)
                .map(|i| hex.get(i..i + 2).and_then(|h| u8::from_str_radix(h, 16).ok()))
                .collect();
            if let Some(bytes) = bytes {
                self.image.add(&bytes, kind == "BIN", ctx);
                shown = format!("{kind} image packet, {} bytes | {radio}", bytes.len());
            }
        } else if let Some(t) = parse_telemetry(&line) {
            self.history.add(&t);
            self.telemetry = Some(t);
        }
        if self.log.back() != Some(&shown) {
            self.log.push_back(shown);
            if self.log.len() > 300 {
                self.log.pop_front();
            }
        }
    }
}

/// Draws the flight path and the latest position on the map.
struct Track(Vec<Position>);

impl Plugin for Track {
    fn run(self: Box<Self>, ui: &mut egui::Ui, _: &egui::Response, projector: &Projector, _: &MapMemory) {
        let points: Vec<_> = self.0.iter().map(|p| projector.project(*p).to_pos2()).collect();
        let red = Color32::from_rgb(230, 40, 40);
        if let Some(last) = points.last().copied() {
            ui.painter().add(egui::Shape::line(points, Stroke::new(2.5, red)));
            ui.painter().circle(last, 6.0, red, Stroke::new(2.0, Color32::WHITE));
        }
    }
}

/// Graph data as [seconds since the first packet, value].
#[derive(Default)]
struct History {
    start: Option<Instant>,
    sea_level_pa: Option<f32>,
    gps_alt: Vec<[f64; 2]>,
    baro_alt: Vec<[f64; 2]>,
    temp: Vec<[f64; 2]>,
    track: Vec<Position>,
}

impl History {
    fn add(&mut self, t: &Telemetry) {
        let now = self.start.get_or_insert_with(Instant::now).elapsed().as_secs_f64();
        let fix = t.fix == "1" && t.alt.is_finite();
        // Calibrate barometric altitude to the GPS once, like the satellite does
        if fix && t.pressure.is_finite() && self.sea_level_pa.is_none() {
            self.sea_level_pa = Some(t.pressure / (1.0 - t.alt / 44330.0).powf(5.255));
        }
        if fix {
            self.gps_alt.push([now, t.alt as f64]);
            if t.lat.is_finite() && t.lon.is_finite() {
                self.track.push(lon_lat(t.lon as f64, t.lat as f64));
            }
        }
        if t.pressure.is_finite() {
            let p0 = self.sea_level_pa.unwrap_or(101_325.0);
            self.baro_alt.push([now, (44330.0 * (1.0 - (t.pressure / p0).powf(0.1903))) as f64]);
        }
        if t.temp.is_finite() {
            self.temp.push([now, t.temp as f64]);
        }
    }
}

/// Line graph of (name, colour, points) series over time, auto-scaled to the data.
fn graph(ui: &mut egui::Ui, unit: &str, series: &[(&str, Color32, &Vec<[f64; 2]>)]) {
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), 150.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, Color32::from_gray(25));
    let font = FontId::monospace(11.0);

    let all = || series.iter().flat_map(|s| s.2.iter());
    let Some(first) = all().next() else {
        painter.text(rect.center(), Align2::CENTER_CENTER, "No data yet", font, Color32::GRAY);
        return;
    };
    let (mut x0, mut x1, mut y0, mut y1) = (first[0], first[0], first[1], first[1]);
    for p in all() {
        (x0, x1, y0, y1) = (x0.min(p[0]), x1.max(p[0]), y0.min(p[1]), y1.max(p[1]));
    }
    let pad = ((y1 - y0) * 0.1).max(1.0);
    let (y0, y1, x1) = (y0 - pad, y1 + pad, x1.max(x0 + 1.0));

    let plot = rect.shrink2(vec2(8.0, 18.0));
    let to_screen = |p: &[f64; 2]| {
        pos2(
            plot.left() + ((p[0] - x0) / (x1 - x0)) as f32 * plot.width(),
            plot.bottom() - ((p[1] - y0) / (y1 - y0)) as f32 * plot.height(),
        )
    };
    let grey = Color32::GRAY;
    painter.line_segment([plot.left_bottom(), plot.right_bottom()], Stroke::new(1.0, Color32::from_gray(60)));
    painter.text(plot.left_top(), Align2::LEFT_BOTTOM, format!("{y1:.1} {unit}"), font.clone(), grey);
    painter.text(plot.left_bottom(), Align2::LEFT_TOP, format!("{y0:.1} {unit}"), font.clone(), grey);
    painter.text(plot.right_bottom(), Align2::RIGHT_TOP, format!("{:.0} s", x1 - x0), font.clone(), grey);

    for (i, (name, color, points)) in series.iter().enumerate() {
        painter.add(egui::Shape::line(points.iter().map(to_screen).collect(), Stroke::new(1.5, *color)));
        let last = points.last().map_or("-".to_string(), |p| format!("{:.1} {unit}", p[1]));
        let pos = rect.right_top() + vec2(-6.0, 4.0 + 14.0 * i as f32);
        painter.text(pos, Align2::RIGHT_TOP, format!("{name}: {last}"), font.clone(), *color);
    }
}

struct Telemetry {
    seq: String,
    heading: f32,
    roll: f32,
    pitch: f32,
    pressure: f32,
    temp: f32,
    lat: f32,
    lon: f32,
    alt: f32,
    sats: String,
    fix: String,
    radio: String,
}

/// Parses `[CRC! ]#014 H183.0 R3.4 P3.9 99798Pa 26.1C 50.09921,19.95716 233m S12 F1 | RSSI ..`
fn parse_telemetry(line: &str) -> Option<Telemetry> {
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
        radio: radio.to_string(),
    };
    for tok in data.split_whitespace().skip(1) {
        // Suffixes first so e.g. "99798Pa" isn't read as pitch
        if let Some(v) = tok.strip_suffix("Pa") {
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

fn telemetry_grid(ui: &mut egui::Ui, t: Option<&Telemetry>) {
    let Some(t) = t else {
        ui.label("Waiting for telemetry...");
        return;
    };
    let fix = match t.fix.as_str() {
        "1" => "fix",
        "0" => "no fix",
        "X" => "no data from GPS",
        _ => "?",
    };
    egui::Grid::new("telemetry").striped(true).show(ui, |ui| {
        for (name, value) in [
            ("Packet", t.seq.clone()),
            ("Heading / roll / pitch", format!("{:.1}° / {:.1}° / {:.1}°", t.heading, t.roll, t.pitch)),
            ("Pressure", format!("{:.0} Pa", t.pressure)),
            ("Temperature", format!("{:.1} °C", t.temp)),
            ("Position", format!("{:.5}, {:.5}", t.lat, t.lon)),
            ("GPS altitude", format!("{:.0} m", t.alt)),
            ("GPS", format!("{} sats, {fix}", t.sats)),
            ("Signal", t.radio.clone()),
        ] {
            ui.label(name);
            ui.label(value);
            ui.end_row();
        }
    });
}

/// Cube with a tether up and a base plate sticking out left/right, rotated by heading/roll/pitch
/// in degrees; the orange face is the front.
fn draw_model(ui: &mut egui::Ui, [heading, roll, pitch]: [f32; 3]) {
    // Normal and corner indices (bit 0 = x, bit 1 = y, bit 2 = z) per box face
    const FACES: [([f32; 3], [usize; 4]); 6] = [
        ([0.0, 0.0, 1.0], [4, 5, 7, 6]),
        ([0.0, 0.0, -1.0], [0, 2, 3, 1]),
        ([1.0, 0.0, 0.0], [1, 3, 7, 5]),
        ([-1.0, 0.0, 0.0], [0, 4, 6, 2]),
        ([0.0, 1.0, 0.0], [2, 6, 7, 3]),
        ([0.0, -1.0, 0.0], [0, 1, 5, 4]),
    ];
    let (orange, grey, green, blue, dark, plate) = (
        Color32::from_rgb(230, 120, 30),
        Color32::from_rgb(120, 120, 120),
        Color32::from_rgb(60, 160, 90),
        Color32::from_rgb(70, 130, 220),
        Color32::from_rgb(50, 50, 70),
        Color32::from_rgb(170, 170, 180),
    );
    // (min corner, max corner, colour per face in FACES order)
    let boxes = [
        ([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], [orange, grey, green, green, blue, dark]),
        ([-1.8, -1.2, -1.0], [1.8, -1.0, 1.0], [plate; 6]),
    ];

    let (rect, _) = ui.allocate_exact_size(vec2(260.0, 300.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, Color32::from_gray(25));

    let rad = |deg: f32| if deg.is_finite() { deg.to_radians() } else { 0.0 };
    // The BNO055 sits upside down in the satellite, so flip about the forward axis first
    const MOUNT_ROLL: f32 = 180.0;
    let (h, r, p, tilt) = (-rad(heading), rad(roll + MOUNT_ROLL), rad(pitch), 20f32.to_radians());
    // Roll about the forward axis, pitch about the side axis, heading about vertical, then a
    // fixed camera tilt so the top is visible
    let rotate = |[x, y, z]: [f32; 3]| {
        let (x, y) = (x * r.cos() - y * r.sin(), x * r.sin() + y * r.cos());
        let (y, z) = (y * p.cos() - z * p.sin(), y * p.sin() + z * p.cos());
        let (x, z) = (x * h.cos() + z * h.sin(), -x * h.sin() + z * h.cos());
        let (y, z) = (y * tilt.cos() - z * tilt.sin(), y * tilt.sin() + z * tilt.cos());
        [x, y, z]
    };
    // Everything turns around the tether's tie-off point, which stays pinned near the top
    const PIVOT: [f32; 3] = [0.0, 2.6, 0.0];
    let place = |[x, y, z]: [f32; 3]| rotate([x - PIVOT[0], y - PIVOT[1], z - PIVOT[2]]);
    let project = |[x, y, z]: [f32; 3]| {
        let s = 40.0 * 12.0 / (12.0 - z);
        rect.center_top() + vec2(x * s, 25.0 - y * s)
    };

    // (depth, shape), drawn far to near
    let mut shapes = Vec::new();
    for (min, max, colors) in boxes {
        let corner = |i: usize| [0, 1, 2].map(|axis| if i >> axis & 1 == 1 { max[axis] } else { min[axis] });
        for ((normal, corners), color) in FACES.into_iter().zip(colors) {
            let n = rotate(normal);
            if n[2] <= 0.0 {
                continue; // facing away
            }
            let k = 0.35 + 0.65 * n[2];
            let shade = |c: u8| (c as f32 * k) as u8;
            let fill = Color32::from_rgb(shade(color.r()), shade(color.g()), shade(color.b()));
            let placed = corners.map(|i| place(corner(i)));
            let depth = placed.iter().map(|c| c[2]).sum::<f32>() / 4.0;
            let points = placed.map(project).to_vec();
            shapes.push((depth, egui::Shape::convex_polygon(points, fill, egui::Stroke::new(1.0, Color32::BLACK))));
        }
    }
    let (bottom, top) = (place([0.0, 1.0, 0.0]), place(PIVOT));
    let tether = egui::Shape::line_segment([project(bottom), project(top)], egui::Stroke::new(2.0, Color32::WHITE));
    shapes.push(((bottom[2] + top[2]) / 2.0, tether));

    shapes.sort_by(|a, b| a.0.total_cmp(&b.0));
    painter.extend(shapes.into_iter().map(|(_, shape)| shape));
    painter.circle_filled(project(top), 4.0, Color32::WHITE);
}

/// Rebuilds a restart-marker JPEG from header chunks and 16x16 blocks; lost blocks are grey.
#[derive(Default)]
struct ImageRx {
    id: Option<u8>,
    header: Vec<Option<Vec<u8>>>,
    blocks: Vec<Option<Vec<u8>>>,
    texture: Option<egui::TextureHandle>,
    last_saved: Vec<u8>,
    status: String,
}

impl ImageRx {
    /// Packet layout is documented in the satellite's `src/image.rs`.
    fn add(&mut self, packet: &[u8], crc_ok: bool, ctx: &egui::Context) {
        let [_magic, id, kind, rest @ ..] = packet else { return };
        // A corrupted header or image id would break the whole image, so only blocks may be corrupted
        if !crc_ok && (*kind == 0 || self.id != Some(*id)) {
            return;
        }
        if self.id != Some(*id) {
            self.id = Some(*id);
            self.header.clear();
            self.blocks.clear();
        }
        match kind {
            0 => {
                let [index, count, data @ ..] = rest else { return };
                self.header.resize(*count as usize, None);
                if let Some(slot) = self.header.get_mut(*index as usize) {
                    *slot = Some(data.to_vec());
                }
            }
            1 => {
                let [f0, f1, t0, t1, data @ ..] = rest else { return };
                let first = u16::from_le_bytes([*f0, *f1]) as usize;
                let total = u16::from_le_bytes([*t0, *t1]) as usize;
                let mut parsed = Vec::new();
                let mut data = data;
                while let [len, tail @ ..] = data {
                    let Some(block) = tail.get(..*len as usize) else { return };
                    parsed.push(block);
                    data = &tail[block.len()..];
                }
                if total > 4096 || first + parsed.len() > total {
                    return;
                }
                self.blocks.resize(total, None);
                for (slot, block) in self.blocks[first..].iter_mut().zip(parsed) {
                    // A corrupted block only fills a gap, never replaces a good one
                    if !block.is_empty() && (crc_ok || slot.is_none()) {
                        *slot = Some(block.to_vec());
                    }
                }
            }
            _ => return,
        }
        self.redraw(ctx);
    }

    fn redraw(&mut self, ctx: &egui::Context) {
        let have = |v: &[Option<Vec<u8>>]| v.iter().filter(|s| s.is_some()).count();
        self.status = format!(
            "Image {}: header {}/{}, blocks {}/{}",
            self.id.unwrap_or(0),
            have(&self.header),
            self.header.len(),
            have(&self.blocks),
            self.blocks.len()
        );
        let Some(header) = self.header.iter().cloned().collect::<Option<Vec<_>>>() else { return };
        if header.is_empty() || have(&self.blocks) == 0 {
            return;
        }

        let mut jpeg = header.concat();
        for (i, block) in self.blocks.iter().enumerate() {
            if i > 0 {
                jpeg.extend([0xFF, 0xD0 + (i - 1) as u8 % 8]);
            }
            jpeg.extend_from_slice(block.as_deref().unwrap_or(&GREY_BLOCK));
        }
        jpeg.extend([0xFF, 0xD9]);

        let mut img = match image::load_from_memory_with_format(&jpeg, ImageFormat::Jpeg) {
            Ok(img) => img.to_rgba8(),
            Err(e) => {
                self.status += &format!(" (decode failed: {e})");
                return;
            }
        };
        let missing: Vec<bool> = self.blocks.iter().map(Option::is_none).collect();
        conceal(&mut img, &missing);
        let size = [img.width() as usize, img.height() as usize];
        let color = egui::ColorImage::from_rgba_unmultiplied(size, img.as_raw());
        match &mut self.texture {
            Some(tex) => tex.set(color, TextureOptions::NEAREST),
            None => self.texture = Some(ctx.load_texture("image", color, TextureOptions::NEAREST)),
        }

        if have(&self.blocks) == self.blocks.len() && jpeg != self.last_saved {
            let secs = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
            let path = format!("images/{secs}.jpg");
            match std::fs::create_dir_all("images").and_then(|()| std::fs::write(&path, &jpeg)) {
                Ok(()) => self.status += &format!(" (saved {path})"),
                Err(e) => self.status += &format!(" (save failed: {e})"),
            }
            self.last_saved = jpeg;
        }
    }
}

/// Fills each missing 16x16 block with an inverse-distance average of the pixels just outside it.
/// Runs in raster order, so blocks above and left are always usable (received or already filled);
/// blocks below and right are used only if they were received.
fn conceal(img: &mut image::RgbaImage, missing: &[bool]) {
    let (w, h) = (img.width() as i32, img.height() as i32);
    let cols = (w + 15) / 16;
    let rows = (h + 15) / 16;
    let received = |bx: i32, by: i32| {
        bx >= 0 && by >= 0 && bx < cols && by < rows && !missing.get((by * cols + bx) as usize).copied().unwrap_or(true)
    };

    for (i, _) in missing.iter().enumerate().filter(|(_, m)| **m) {
        let (bx, by) = (i as i32 % cols, i as i32 / cols);
        let (x0, y0) = (bx * 16, by * 16);
        let (x1, y1) = ((x0 + 16).min(w), (y0 + 16).min(h));
        if y0 >= h {
            continue;
        }
        // (usable, neighbour pixel for (x, y), distance)
        let up = by > 0;
        let left = bx > 0;
        let down = y1 < h && received(bx, by + 1);
        let right = x1 < w && received(bx + 1, by);

        for y in y0..y1 {
            for x in x0..x1 {
                let samples = [
                    (up, (x, y0 - 1), y - y0 + 1),
                    (down, (x, y1), y1 - y),
                    (left, (x0 - 1, y), x - x0 + 1),
                    (right, (x1, y), x1 - x),
                ];
                let (mut sum, mut total) = ([0f32; 3], 0f32);
                for (_, (sx, sy), d) in samples.into_iter().filter(|s| s.0) {
                    let p = img.get_pixel(sx as u32, sy as u32);
                    let weight = 1.0 / d as f32;
                    (0..3).for_each(|c| sum[c] += p[c] as f32 * weight);
                    total += weight;
                }
                if total > 0.0 {
                    let rgb = sum.map(|s| (s / total).round() as u8);
                    img.put_pixel(x as u32, y as u32, image::Rgba([rgb[0], rgb[1], rgb[2], 255]));
                }
            }
        }
    }
}

fn read_serial(tx: Sender<String>, ctx: egui::Context) {
    let send = |line: String| {
        let _ = tx.send(line);
        ctx.request_repaint();
    };
    loop {
        let Some(port) = std::env::args().nth(1).or_else(find_usb_port) else {
            let ports: Vec<_> = serialport::available_ports().unwrap_or_default().into_iter().map(|p| p.port_name).collect();
            send(format!("[monitor] Waiting for receiver USB... (visible ports: {ports:?})"));
            sleep(Duration::from_secs(1));
            continue;
        };

        match serialport::new(&port, 115_200).timeout(Duration::from_secs(3600)).open() {
            Ok(serial) => {
                send(format!("[monitor] Connected to {port}"));
                for line in BufReader::new(serial).lines() {
                    match line {
                        Ok(line) => send(line.trim_end().to_string()),
                        Err(e) if e.kind() == ErrorKind::TimedOut => continue,
                        Err(e) => {
                            send(format!("[monitor] Disconnected: {e}"));
                            break;
                        }
                    }
                }
            }
            Err(e) => send(format!("[monitor] Can't open {port}: {e} (unplugged, or open in another program?)")),
        }
        sleep(Duration::from_secs(1));
    }
}

fn find_usb_port() -> Option<String> {
    serialport::available_ports()
        .ok()?
        .into_iter()
        .find(|p| matches!(p.port_type, SerialPortType::UsbPort(_)))
        .map(|p| p.port_name)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Packs `assets/image1.jpg` like the satellite (one block per packet here) and checks that the
    /// monitor rebuilds a decodable image even with lost blocks.
    #[test]
    fn rebuilds_image_with_lost_blocks() {
        let jpeg = std::fs::read("../assets/image1.jpg").unwrap();
        let sos = jpeg.windows(2).position(|w| w == [0xFF, 0xDA]).unwrap();
        let header_end = sos + 2 + u16::from_be_bytes([jpeg[sos + 2], jpeg[sos + 3]]) as usize;
        let (header, scan) = (&jpeg[..header_end], &jpeg[header_end..jpeg.len() - 2]);

        let mut blocks = vec![Vec::new()];
        let mut i = 0;
        while i < scan.len() {
            if scan[i] == 0xFF && (0xD0..=0xD7).contains(scan.get(i + 1).unwrap_or(&0)) {
                blocks.push(Vec::new());
                i += 2;
            } else {
                blocks.last_mut().unwrap().push(scan[i]);
                i += 1;
            }
        }

        let ctx = egui::Context::default();
        let mut rx = ImageRx::default();
        let chunks: Vec<_> = header.chunks(240).collect();
        for (n, chunk) in chunks.iter().enumerate() {
            rx.add(&[&[0xA5, 7, 0, n as u8, chunks.len() as u8][..], chunk].concat(), true, &ctx);
        }
        let total = (blocks.len() as u16).to_le_bytes();
        for (n, block) in blocks.iter().enumerate() {
            if n % 3 == 1 {
                continue; // lost
            }
            let first = (n as u16).to_le_bytes();
            let packet = [&[0xA5, 7, 1, first[0], first[1], total[0], total[1], block.len() as u8][..], block].concat();
            rx.add(&packet, true, &ctx);
        }

        assert!(rx.texture.is_some(), "image did not decode: {}", rx.status);
        let size = rx.texture.as_ref().unwrap().size();
        assert_eq!(size, [640, 480], "{}", rx.status);
    }

    #[test]
    fn conceal_blends_neighbours() {
        // Left half black, right half white, 32x16 = two blocks; block 1 (right) is lost
        let mut img = image::RgbaImage::from_fn(32, 16, |x, _| {
            image::Rgba(if x < 16 { [0, 0, 0, 255] } else { [128, 128, 128, 255] })
        });
        conceal(&mut img, &[false, true]);
        // Only the left neighbour is usable, so the lost block continues it
        assert_eq!(img.get_pixel(20, 8).0, [0, 0, 0, 255]);
    }
}

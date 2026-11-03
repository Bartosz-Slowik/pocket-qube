//! Rebuilds the satellite's restart-marker JPEGs from image packets, filling lost blocks, and
//! saves each photo to `images/`. Packet layout is documented in the satellite's `src/image.rs`;
//! the camera's interleaved type 2 in `camera/src/packets.rs`.
use image::ImageFormat;

use crate::web::unix_secs;

/// One all-grey 16x16 block (4:2:0, standard Huffman tables); keeps the JPEG decodable until
/// `conceal` paints over it.
const GREY_BLOCK: [u8; 4] = [0x28, 0xA2, 0x8A, 0x00];

#[derive(Default)]
pub struct ImageRx {
    id: Option<u8>,
    header: Vec<Option<Vec<u8>>>,
    blocks: Vec<Option<Vec<u8>>>,
    last_saved: Vec<u8>,
    pub status: String,
    /// Latest picture with lost blocks filled in
    pub rgba: Option<image::RgbaImage>,
    /// `rgba` changed since the web page last got it
    pub dirty: bool,
    /// Whether this image was saved complete
    complete: bool,
    /// File name in `images/` of a just-saved photo, and the % of its blocks that arrived
    pub saved: Option<(String, u8)>,
    /// Blocks received and skipped since the counters were last reset
    pub blocks_got: u32,
    pub blocks_lost: u32,
    /// Block the next image packet should start with; None after joining mid-image
    next_block: Option<usize>,
    /// Last complete header and its image's block count, used when a header is lost
    fallback_header: Option<(usize, Vec<u8>)>,
}

impl ImageRx {
    pub fn add(&mut self, packet: &[u8], crc_ok: bool) {
        let [_magic, id, kind, rest @ ..] = packet else { return };
        // A corrupted header or image id would break the whole image, so only blocks may be corrupted
        if !crc_ok && (*kind == 0 || self.id != Some(*id)) {
            return;
        }
        if self.id != Some(*id) {
            self.start(*id, *kind == 0);
        }
        match kind {
            0 => {
                let [index, count, data @ ..] = rest else { return };
                self.header.resize(*count as usize, None);
                if let Some(slot) = self.header.get_mut(*index as usize) {
                    *slot = Some(data.to_vec());
                }
            }
            1 | 2 => {
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
                // Packets carry consecutive blocks, so a jump means packets were lost
                if crc_ok {
                    if let Some(expected) = self.next_block {
                        self.blocks_lost += first.saturating_sub(expected) as u32;
                    }
                    self.blocks_got += parsed.len() as u32;
                    self.next_block = Some(first + parsed.len());
                }
                self.blocks.resize(total, None);
                let order = if *kind == 2 { shuffled(total) } else { (0..total).collect() };
                for (j, block) in parsed.into_iter().enumerate() {
                    let slot = &mut self.blocks[order[first + j]];
                    // A corrupted block only fills a gap, never replaces a good one
                    if !block.is_empty() && (crc_ok || slot.is_none()) {
                        *slot = Some(block.to_vec());
                    }
                }
            }
            _ => return,
        }
        self.redraw();
    }

    /// Starts image `id`, saving the previous one if it never completed.
    fn start(&mut self, id: u8, from_header: bool) {
        if let Some(expected) = self.next_block {
            self.blocks_lost += self.blocks.len().saturating_sub(expected) as u32;
        }
        self.next_block = from_header.then_some(0);
        if !self.complete
            && let Some(img) = self.rgba.take()
        {
            let name = format!("{}-partial.jpg", unix_secs());
            let got = self.blocks.iter().filter(|b| b.is_some()).count();
            if std::fs::create_dir_all("images").and_then(|()| std::fs::write(format!("images/{name}"), to_jpeg(&img, 85))).is_ok() {
                self.saved = Some((name, (100 * got / self.blocks.len().max(1)) as u8));
            }
        }
        self.id = Some(id);
        self.header.clear();
        self.blocks.clear();
        self.complete = false;
        self.rgba = None;
    }

    fn redraw(&mut self) {
        let have = |v: &[Option<Vec<u8>>]| v.iter().filter(|s| s.is_some()).count();
        self.status = format!(
            "Image {}: header {}/{}, blocks {}/{}",
            self.id.unwrap_or(0),
            have(&self.header),
            self.header.len(),
            have(&self.blocks),
            self.blocks.len()
        );
        if have(&self.blocks) == 0 {
            return;
        }
        let mut jpeg = match self.header.iter().cloned().collect::<Option<Vec<_>>>().filter(|h| !h.is_empty()) {
            Some(header) => {
                let header = header.concat();
                self.fallback_header = Some((self.blocks.len(), header.clone()));
                header
            }
            // Photos of the same size and quality have identical headers
            None => match &self.fallback_header {
                Some((total, header)) if *total == self.blocks.len() => {
                    self.status += " (previous header)";
                    header.clone()
                }
                _ => return,
            },
        };
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
        self.rgba = Some(img);
        self.dirty = true;

        if have(&self.blocks) == self.blocks.len() && jpeg != self.last_saved {
            let secs = unix_secs();
            let path = format!("images/{secs}.jpg");
            match std::fs::create_dir_all("images").and_then(|()| std::fs::write(&path, &jpeg)) {
                Ok(()) => {
                    self.status += &format!(" (saved {path})");
                    self.saved = Some((format!("{secs}.jpg"), 100));
                    self.complete = true;
                }
                Err(e) => self.status += &format!(" (save failed: {e})"),
            }
            self.last_saved = jpeg;
        }
    }
}

pub fn to_jpeg(img: &image::RgbaImage, quality: u8) -> Vec<u8> {
    let mut out = Vec::new();
    let rgb = image::DynamicImage::ImageRgba8(img.clone()).to_rgb8();
    let _ = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, quality).encode_image(&rgb);
    out
}

/// Block order of type 2 packets: a fixed pseudo-random shuffle (xorshift32 + Fisher-Yates).
/// Must match `camera/src/packets.rs`.
fn shuffled(total: usize) -> Vec<usize> {
    let mut order: Vec<usize> = (0..total).collect();
    let mut x = 0x9E37_79B9u32;
    for i in (1..total).rev() {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        order.swap(i, x as usize % (i + 1));
    }
    order
}

/// Paints each missing 16x16 block as a smooth blend of block colours: received blocks' average
/// colours are spread into the gaps, then blended between block centres. A half-arrived image
/// looks like a blurred preview instead of streaks.
fn conceal(img: &mut image::RgbaImage, missing: &[bool]) {
    let (w, h) = (img.width(), img.height());
    let (cols, rows) = (w.div_ceil(16), h.div_ceil(16));
    let at = |bx: u32, by: u32| (by * cols + bx) as usize;
    let lost = |i: usize| missing.get(i).copied().unwrap_or(true);

    let mut avg: Vec<Option<[f32; 3]>> = vec![None; (cols * rows) as usize];
    for by in 0..rows {
        for bx in 0..cols {
            if lost(at(bx, by)) {
                continue;
            }
            let (mut sum, mut n) = ([0f32; 3], 0f32);
            for y in by * 16..(by * 16 + 16).min(h) {
                for x in bx * 16..(bx * 16 + 16).min(w) {
                    let p = img.get_pixel(x, y);
                    (0..3).for_each(|c| sum[c] += p[c] as f32);
                    n += 1.0;
                }
            }
            avg[at(bx, by)] = Some(sum.map(|s| s / n));
        }
    }
    if avg.iter().all(Option::is_none) {
        return;
    }
    // One ring of neighbours per pass
    while avg.iter().any(Option::is_none) {
        let prev = avg.clone();
        for by in 0..rows {
            for bx in 0..cols {
                if prev[at(bx, by)].is_some() {
                    continue;
                }
                let (mut sum, mut n) = ([0f32; 3], 0f32);
                for (nx, ny) in [(bx.wrapping_sub(1), by), (bx + 1, by), (bx, by.wrapping_sub(1)), (bx, by + 1)] {
                    if nx < cols && ny < rows
                        && let Some(c) = prev[at(nx, ny)]
                    {
                        (0..3).for_each(|i| sum[i] += c[i]);
                        n += 1.0;
                    }
                }
                if n > 0.0 {
                    avg[at(bx, by)] = Some(sum.map(|s| s / n));
                }
            }
        }
    }

    let colour = |bx: u32, by: u32| avg[at(bx, by)].unwrap_or_default();
    for by in 0..rows {
        for bx in 0..cols {
            if !lost(at(bx, by)) {
                continue;
            }
            for y in by * 16..(by * 16 + 16).min(h) {
                for x in bx * 16..(bx * 16 + 16).min(w) {
                    // Position in block-centre coordinates
                    let fx = ((x as f32 - 7.5) / 16.0).clamp(0.0, (cols - 1) as f32);
                    let fy = ((y as f32 - 7.5) / 16.0).clamp(0.0, (rows - 1) as f32);
                    let (x0, y0) = (fx as u32, fy as u32);
                    let (x1, y1) = ((x0 + 1).min(cols - 1), (y0 + 1).min(rows - 1));
                    let (tx, ty) = (fx - x0 as f32, fy - y0 as f32);
                    let (a, b, c, d) = (colour(x0, y0), colour(x1, y0), colour(x0, y1), colour(x1, y1));
                    let rgb = [0, 1, 2].map(|i| {
                        let top = a[i] + (b[i] - a[i]) * tx;
                        let bottom = c[i] + (d[i] - c[i]) * tx;
                        (top + (bottom - top) * ty).round() as u8
                    });
                    img.put_pixel(x, y, image::Rgba([rgb[0], rgb[1], rgb[2], 255]));
                }
            }
        }
    }
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

        let mut rx = ImageRx::default();
        let chunks: Vec<_> = header.chunks(240).collect();
        for (n, chunk) in chunks.iter().enumerate() {
            rx.add(&[&[0xA5, 7, 0, n as u8, chunks.len() as u8][..], chunk].concat(), true);
        }
        // Image 7 in order, then image 8 interleaved and without its header
        for (id, kind) in [(7, 1), (8, 2)] {
            // Keeps image 7 from being saved to images/ as a partial photo
            rx.complete = true;
            rx.blocks_got = 0;
            rx.blocks_lost = 0;
            let order: Vec<usize> = if kind == 2 { shuffled(blocks.len()) } else { (0..blocks.len()).collect() };
            let total = (blocks.len() as u16).to_le_bytes();
            for n in 0..blocks.len() {
                if n % 3 == 1 {
                    continue; // lost
                }
                let block = &blocks[order[n]];
                let first = (n as u16).to_le_bytes();
                let packet = [&[0xA5, id, kind, first[0], first[1], total[0], total[1], block.len() as u8][..], block].concat();
                rx.add(&packet, true);
            }

            let img = rx.rgba.as_ref().unwrap_or_else(|| panic!("image did not decode: {}", rx.status));
            assert_eq!(img.dimensions(), (640, 480), "{}", rx.status);
            // Every third of the 1200 blocks was dropped
            assert_eq!((rx.blocks_got, rx.blocks_lost), (800, 400));
        }
        assert!(rx.status.contains("previous header"), "{}", rx.status);
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

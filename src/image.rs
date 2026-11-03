//! Sends the stored images (used while the ESP32 camera is silent) in loss-tolerant packets: the
//! JPEG header in chunks, then whole 16x16
//! blocks cut at the restart markers, so a lost packet only loses its blocks. The images
//! come from `monitor`'s `prepare_image` tool and are sent alternately.
//!
//! Header chunk: `A5 id 00 chunk_index chunk_count data..`
//! Blocks:       `A5 id 01 first_block(u16 LE) total_blocks(u16 LE) [len data..]..`
use heapless::Vec;

static IMAGES: [&[u8]; 2] = [include_bytes!("../assets/image1.jpg"), include_bytes!("../assets/image2.jpg")];
const MAGIC: u8 = 0xA5;
const MAX_DATA: usize = 240;
/// Leaves room in a 255-byte FLRC packet for the 2-byte timing prefix
pub const MAX_PACKET: usize = 253;

#[derive(Default)]
pub struct ImageTx {
    id: u8,
    chunk: usize,
    pos: usize,
    block: u16,
}

impl ImageTx {
    /// Writes the next packet; after the last block the next image starts with a new id.
    pub fn next(&mut self, out: &mut Vec<u8, MAX_PACKET>) {
        out.clear();
        let (header, scan) = split(IMAGES[self.id as usize % IMAGES.len()]);

        let chunks = header.len().div_ceil(MAX_DATA);
        if self.chunk < chunks {
            let start = self.chunk * MAX_DATA;
            let _ = out.extend_from_slice(&[MAGIC, self.id, 0, self.chunk as u8, chunks as u8]);
            let _ = out.extend_from_slice(&header[start..(start + MAX_DATA).min(header.len())]);
            self.chunk += 1;
            return;
        }

        let total = scan.windows(2).filter(|w| is_rst(w)).count() as u16 + 1;
        let _ = out.extend_from_slice(&[MAGIC, self.id, 1]);
        let _ = out.extend_from_slice(&self.block.to_le_bytes());
        let _ = out.extend_from_slice(&total.to_le_bytes());
        while self.block < total {
            let (data, next) = block_at(scan, self.pos);
            // A block too big for any packet is sent empty, which the monitor shows as missing
            let data = if data.len() > MAX_DATA { &[][..] } else { data };
            if out.len() + 1 + data.len() > out.capacity() {
                break;
            }
            let _ = out.push(data.len() as u8);
            let _ = out.extend_from_slice(data);
            self.pos = next;
            self.block += 1;
        }
        if self.block == total {
            // Ids 0x80.. belong to the ESP32 camera
            *self = Self { id: (self.id + 1) & 0x7F, ..Self::default() };
        }
    }
}

/// Splits a JPEG into header (up to and including the scan header) and entropy-coded data.
fn split(jpeg: &[u8]) -> (&[u8], &[u8]) {
    let mut i = 2;
    while i + 4 <= jpeg.len() {
        let len = u16::from_be_bytes([jpeg[i + 2], jpeg[i + 3]]) as usize;
        if jpeg[i + 1] == 0xDA {
            let start = i + 2 + len;
            return (&jpeg[..start], &jpeg[start..jpeg.len() - 2]);
        }
        i += 2 + len;
    }
    (jpeg, &[])
}

/// Returns the block starting at `pos` and where the next one starts.
fn block_at(scan: &[u8], pos: usize) -> (&[u8], usize) {
    let end = scan[pos..].windows(2).position(is_rst).map_or(scan.len(), |i| pos + i);
    (&scan[pos..end], (end + 2).min(scan.len()))
}

fn is_rst(w: &[u8]) -> bool {
    w[0] == 0xFF && (0xD0..=0xD7).contains(&w[1])
}

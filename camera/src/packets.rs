//! Cuts a JPEG into image packets for the satellite to relay. Must match the monitor's
//! `image_rx.rs`:
//!
//! Header chunk: `A5 id 00 chunk_index chunk_count data..`
//! Blocks:       `A5 id 02 first_position(u16 LE) total_blocks(u16 LE) [len data..]..`
//!
//! Blocks go out shuffled: position `p` carries block `shuffled(total)[p]`, so the whole frame
//! fills in like a dissolve and a lost packet leaves scattered blocks that are easy to hide.
//! (The satellite's stored images use type 01, in order.)

const MAGIC: u8 = 0xA5;
const MAX_DATA: usize = 240;
/// Leaves room in a 255-byte FLRC packet for the satellite's 2-byte timing prefix
pub const MAX_PACKET: usize = 253;

pub fn packets(jpeg: &[u8], id: u8) -> Vec<Vec<u8>> {
    let (header, scan) = split(jpeg);
    let mut out = Vec::new();

    let chunks: Vec<&[u8]> = header.chunks(MAX_DATA).collect();
    for (i, chunk) in chunks.iter().enumerate() {
        out.push([&[MAGIC, id, 0, i as u8, chunks.len() as u8][..], chunk].concat());
    }

    let blocks = blocks(scan);
    let total = blocks.len() as u16;
    let blocks: Vec<&[u8]> = shuffled(blocks.len()).into_iter().map(|b| blocks[b]).collect();
    let mut first = 0;
    while first < total {
        let mut p = vec![MAGIC, id, 2];
        p.extend_from_slice(&first.to_le_bytes());
        p.extend_from_slice(&total.to_le_bytes());
        let mut block = first;
        while block < total {
            // A block too big for any packet is sent empty, which the monitor shows as missing
            let data = Some(blocks[block as usize]).filter(|d| d.len() <= MAX_DATA).unwrap_or(&[]);
            if p.len() + 1 + data.len() > MAX_PACKET {
                break;
            }
            p.push(data.len() as u8);
            p.extend_from_slice(data);
            block += 1;
        }
        out.push(p);
        first = block;
    }
    out
}

/// Block order of type 2 packets: a fixed pseudo-random shuffle (xorshift32 + Fisher-Yates).
/// Must match the monitor's `shuffled`.
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

/// The 16x16 blocks between restart markers.
fn blocks(scan: &[u8]) -> Vec<&[u8]> {
    let mut blocks = Vec::new();
    let (mut start, mut i) = (0, 0);
    while i + 1 < scan.len() {
        if scan[i] == 0xFF && (0xD0..=0xD7).contains(&scan[i + 1]) {
            blocks.push(&scan[start..i]);
            start = i + 2;
            i += 2;
        } else {
            i += 1;
        }
    }
    blocks.push(&scan[start..]);
    blocks
}

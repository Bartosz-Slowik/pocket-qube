//! Re-encodes an image for LoRa: max 640x480, 4:2:0, standard Huffman tables and a restart
//! marker after every 16x16 block, so each block decodes on its own.
//! Usage: `cargo run --bin prepare_image -- [input] [output] [quality]`
use jpeg_encoder::{ColorType, Encoder, SamplingFactor};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let input = args.get(1).map_or("../assets/test.jpg", String::as_str);
    let output = args.get(2).map_or("../assets/image.jpg", String::as_str);
    let quality = args.get(3).and_then(|q| q.parse().ok()).unwrap_or(40);

    let img = image::open(input).expect("can't read input image");
    let img = if img.width() > 640 || img.height() > 480 { img.thumbnail(640, 480) } else { img };
    let rgb = img.to_rgb8();

    let mut encoder = Encoder::new_file(output, quality).expect("can't create output file");
    // The monitor's grey filler for lost blocks assumes exactly these settings
    encoder.set_sampling_factor(SamplingFactor::R_4_2_0);
    encoder.set_optimized_huffman_tables(false);
    encoder.set_restart_interval(1);
    encoder
        .encode(rgb.as_raw(), rgb.width() as u16, rgb.height() as u16, ColorType::Rgb)
        .expect("JPEG encoding failed");

    let size = std::fs::metadata(output).map(|m| m.len()).unwrap_or(0);
    println!("{output}: {}x{}, {size} bytes", rgb.width(), rgb.height());
}

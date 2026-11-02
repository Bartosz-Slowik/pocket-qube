//! XIAO ESP32S3 Sense camera for the PocketQube: takes OV3660 photos, re-encodes them in the
//! satellite's loss-tolerant JPEG format and hands the image packets to the STM32 over SPI
//! (mode 0, pins in `board.rs`).
//! Each 256-byte transfer is `len`, one image packet, the packet's wrapping byte sum, zeros.
//! To save power the sensor sleeps between photos and the CPU runs at 80 MHz except while encoding.
//! LED: fast blink while booting, a short flash every 2 s while the STM32 reads photos, else off.
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering::Relaxed};
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::thread;
use std::time::Duration;

use esp_idf_svc::hal::delay::BLOCK;
use esp_idf_svc::sys::{self, EspError, camera, esp};
use jpeg_encoder::{Encoder, ImageBuffer, JpegColorType, SamplingFactor};
use log::{error, info, warn};

mod board;
mod packets;

const QUALITY: u8 = 70;
/// Frames per photo; the sharpest is encoded and sent
const BURST: usize = 5;
/// The next photo is asked for this many packets before the current one is sent. A burst takes
/// ~2.5 s and the satellite sends ~95 packets per 3 s cycle, so ~100 packets has it ready just
/// in time without it waiting stale
const EARLY_WANT: usize = 100;
const FRAME: usize = 256;
/// The STM32 only reads while sending images, so pauses of ~0.3 s are normal
const LINK_TIMEOUT_MS: u32 = 2000;

const BOOTING: u8 = 0;
const WORKING: u8 = 1;
const FAILED: u8 = 2;
static CAMERA: AtomicU8 = AtomicU8::new(BOOTING);
static LINK_OK: AtomicBool = AtomicBool::new(false);

/// SPI DMA needs word-aligned buffers.
#[repr(align(4))]
struct Frame([u8; FRAME]);

fn main() {
    sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    thread::Builder::new().stack_size(3072).spawn(led).unwrap();

    // 80 MHz when idle; `cpu_max` raises it to 240 MHz while encoding
    let pm = sys::esp_pm_config_t { max_freq_mhz: 240, min_freq_mhz: 80, light_sleep_enable: false };
    esp!(unsafe { sys::esp_pm_configure(&pm as *const _ as *const _) }).unwrap();
    let mut cpu_max = std::ptr::null_mut();
    esp!(unsafe {
        sys::esp_pm_lock_create(sys::esp_pm_lock_type_t_ESP_PM_CPU_FREQ_MAX, 0, c"encode".as_ptr(), &mut cpu_max)
    })
    .unwrap();

    // The SPI thread asks for the next photo shortly before it's needed
    let (want_tx, want_rx) = sync_channel(1);
    let (photo_tx, photo_rx) = sync_channel(1);
    thread::Builder::new().stack_size(8192).spawn(move || serve_spi(want_tx, photo_rx)).unwrap();

    while let Err(e) = init_camera() {
        CAMERA.store(FAILED, Relaxed);
        error!("Camera init failed: {e}, retrying in 1 s");
        thread::sleep(Duration::from_secs(1));
    }
    loop {
        want_rx.recv().unwrap();
        let start = std::time::Instant::now();
        unsafe { sys::esp_pm_lock_acquire(cpu_max) };
        let jpeg = loop {
            match burst(QUALITY) {
                Some(jpeg) => break jpeg,
                None => {
                    CAMERA.store(FAILED, Relaxed);
                    error!("Capture failed, retrying in 1 s");
                    thread::sleep(Duration::from_secs(1));
                }
            }
        };
        unsafe { sys::esp_pm_lock_release(cpu_max) };
        CAMERA.store(WORKING, Relaxed);
        info!("Photo ready in {} ms, {} bytes", start.elapsed().as_millis(), jpeg.len());
        photo_tx.send(jpeg).unwrap();
    }
}

/// Wakes the sensor, encodes the sharpest of a few frames and puts the sensor back to standby.
fn burst(quality: u8) -> Option<Vec<u8>> {
    sensor_standby(false);
    // Frames still buffered from before standby, then one only partly exposed
    for _ in 0..3 {
        let fb = unsafe { camera::esp_camera_fb_get() };
        if !fb.is_null() {
            unsafe { camera::esp_camera_fb_return(fb) };
        }
    }
    // Copied out: holding a driver buffer while grabbing the next starves the 2-buffer driver
    let (mut best, mut best_score, mut size) = (Vec::new(), None, (0, 0));
    for _ in 0..BURST {
        let fb = unsafe { camera::esp_camera_fb_get() };
        if !fb.is_null() {
            let f = unsafe { &*fb };
            let (w, h) = (f.width as usize, f.height as usize);
            let yuyv = unsafe { std::slice::from_raw_parts(f.buf, f.len) };
            if yuyv.len() == w * h * 2 {
                let score = sharpness(yuyv, w, h);
                if best_score.is_none_or(|s| score > s) {
                    best.clear();
                    best.extend_from_slice(yuyv);
                    (best_score, size) = (Some(score), (w as u16, h as u16));
                }
            }
            unsafe { camera::esp_camera_fb_return(fb) };
        }
        // Lets the idle task run, or the task watchdog complains
        thread::sleep(Duration::from_millis(10));
    }
    sensor_standby(true);
    best_score?;
    encode(&best, size.0, size.1, quality)
}

/// Summed brightness steps between neighbouring pixels, sampled every 4th pixel of every 4th
/// row: motion blur lowers it. Nearly black or white frames score 0.
fn sharpness(yuyv: &[u8], w: usize, h: usize) -> u64 {
    let row = w * 2;
    let (mut steps, mut sum, mut n) = (0u64, 0u64, 0u64);
    for r in (0..h - 1).step_by(4) {
        for c in (0..w - 1).step_by(4) {
            // Brightness (Y) is every other byte
            let i = r * row + c * 2;
            let y = yuyv[i] as i32;
            steps += (y - yuyv[i + 2] as i32).unsigned_abs() as u64 + (y - yuyv[i + row] as i32).unsigned_abs() as u64;
            sum += y as u64;
            n += 1;
        }
    }
    if (16..=240).contains(&(sum / n.max(1))) { steps } else { 0 }
}

/// OV3660 software standby (SYSTEM_CTROL0 bit 6); register settings survive it.
fn sensor_standby(on: bool) {
    let s = unsafe { camera::esp_camera_sensor_get() };
    if let Some(set_reg) = unsafe { (*s).set_reg } {
        unsafe { set_reg(s, 0x3008, 0x40, if on { 0x40 } else { 0 }) };
    }
}

fn led() {
    unsafe {
        sys::gpio_reset_pin(board::LED);
        sys::gpio_set_direction(board::LED, sys::gpio_mode_t_GPIO_MODE_OUTPUT);
    }
    for tick in 0u32.. {
        let on = match CAMERA.load(Relaxed) {
            BOOTING => tick % 2 == 0,
            WORKING => LINK_OK.load(Relaxed) && tick % 20 == 0,
            _ => false,
        };
        // Lit when low
        unsafe { sys::gpio_set_level(board::LED, !on as u32) };
        thread::sleep(Duration::from_millis(100));
    }
}

fn init_camera() -> Result<(), EspError> {
    let d = board::CAM_DATA;
    let mut config = camera::camera_config_t {
        pin_pwdn: -1,
        pin_reset: -1,
        pin_xclk: board::CAM_XCLK,
        pin_d7: d[7],
        pin_d6: d[6],
        pin_d5: d[5],
        pin_d4: d[4],
        pin_d3: d[3],
        pin_d2: d[2],
        pin_d1: d[1],
        pin_d0: d[0],
        pin_vsync: board::CAM_VSYNC,
        pin_href: board::CAM_HREF,
        pin_pclk: board::CAM_PCLK,
        xclk_freq_hz: board::CAM_XCLK_HZ,
        ledc_timer: camera::ledc_timer_t_LEDC_TIMER_0,
        ledc_channel: camera::ledc_channel_t_LEDC_CHANNEL_0,
        // Raw pixels: the sensor's own JPEG has no restart markers for loss-tolerant packets
        pixel_format: camera::pixformat_t_PIXFORMAT_YUV422,
        // XGA: the largest size whose two frame buffers fit in PSRAM and the monitor's block limit
        frame_size: camera::framesize_t_FRAMESIZE_XGA,
        fb_count: 2,
        fb_location: camera::camera_fb_location_t_CAMERA_FB_IN_PSRAM,
        grab_mode: camera::camera_grab_mode_t_CAMERA_GRAB_LATEST,
        ..Default::default()
    };
    config.__bindgen_anon_1.pin_sccb_sda = board::CAM_SDA;
    config.__bindgen_anon_2.pin_sccb_scl = board::CAM_SCL;
    esp!(unsafe { camera::esp_camera_init(&config) })?;

    let s = unsafe { camera::esp_camera_sensor_get() };
    unsafe {
        // One step darker than auto: shorter exposures blur less, and the bright sky doesn't clip
        if let Some(set_ae_level) = (*s).set_ae_level {
            set_ae_level(s, -1);
        }
        // Night mode would allow long exposures
        if let Some(set_aec2) = (*s).set_aec2 {
            set_aec2(s, 0);
        }
    }

    // The first frames are dark until auto exposure settles
    for _ in 0..10 {
        let fb = unsafe { camera::esp_camera_fb_get() };
        if !fb.is_null() {
            unsafe { camera::esp_camera_fb_return(fb) };
        }
    }
    sensor_standby(true);
    info!("Camera ready");
    Ok(())
}

/// Encodes a YUV422 frame like the monitor's `prepare_image`.
fn encode(yuyv: &[u8], w: u16, h: u16, quality: u8) -> Option<Vec<u8>> {
    let mut jpeg = Vec::new();
    let mut encoder = Encoder::new(&mut jpeg, quality);
    // The monitor's grey filler for lost blocks assumes exactly these settings
    encoder.set_sampling_factor(SamplingFactor::R_4_2_0);
    encoder.set_optimized_huffman_tables(false);
    encoder.set_restart_interval(1);
    let ok = yuyv.len() >= w as usize * h as usize * 2 && encoder.encode_image(Yuyv(yuyv, w, h)).is_ok();
    ok.then_some(jpeg)
}

/// Camera frame in YUV422 (Y0 U Y1 V per pixel pair); JPEG takes the same YCbCr per pixel.
struct Yuyv<'a>(&'a [u8], u16, u16);

impl ImageBuffer for Yuyv<'_> {
    fn get_jpeg_color_type(&self) -> JpegColorType {
        JpegColorType::Ycbcr
    }
    fn width(&self) -> u16 {
        self.1
    }
    fn height(&self) -> u16 {
        self.2
    }
    fn fill_buffers(&self, y: u16, buffers: &mut [Vec<u8>; 4]) {
        let row = self.1 as usize * 2;
        for px in self.0[y as usize * row..][..row].chunks_exact(4) {
            buffers[0].extend_from_slice(&[px[0], px[2]]);
            buffers[1].extend_from_slice(&[px[1], px[1]]);
            buffers[2].extend_from_slice(&[px[3], px[3]]);
        }
    }
}

/// Hands out one image packet per SPI transfer; the STM32 is the master and reads when it has
/// air time. Image ids have the top bit set so they never clash with the STM32's stored images.
fn serve_spi(want: SyncSender<()>, photos: Receiver<Vec<u8>>) {
    init_spi().expect("SPI slave init failed");
    let mut frame = Frame([0; FRAME]);
    let mut id = 0u8;
    want.send(()).unwrap();
    loop {
        let jpeg = photos.recv().unwrap();
        let packets = packets::packets(&jpeg, 0x80 | id);
        for (i, p) in packets.iter().enumerate() {
            if packets.len() - i == EARLY_WANT.min(packets.len()) {
                want.send(()).unwrap();
            }
            frame.0.fill(0);
            frame.0[0] = p.len() as u8;
            frame.0[1..=p.len()].copy_from_slice(&p);
            frame.0[p.len() + 1] = p.iter().fold(0u8, |sum, b| sum.wrapping_add(*b));
            // Repeats the packet if the STM32 cut the transfer short
            while transfer(&frame) < FRAME * 8 {}
        }
        info!("Photo {id} sent");
        id = (id + 1) & 0x7F;
    }
}

/// Waits until the STM32 has read `frame`; returns the number of bits it clocked.
fn transfer(frame: &Frame) -> usize {
    let mut t = sys::spi_slave_transaction_t {
        length: FRAME * 8,
        tx_buffer: frame.0.as_ptr() as *const _,
        ..Default::default()
    };
    esp!(unsafe { sys::spi_slave_queue_trans(sys::spi_host_device_t_SPI2_HOST, &mut t, BLOCK) })
        .expect("SPI queue failed");
    let mut done = std::ptr::null_mut();
    // FREERTOS_HZ is 1000, so ticks are ms
    while unsafe { sys::spi_slave_get_trans_result(sys::spi_host_device_t_SPI2_HOST, &mut done, LINK_TIMEOUT_MS) }
        != sys::ESP_OK
    {
        if LINK_OK.swap(false, Relaxed) {
            warn!("STM32 stopped reading");
        }
    }
    if !LINK_OK.swap(true, Relaxed) {
        info!("STM32 is reading");
    }
    unsafe { (*done).trans_len }
}

fn init_spi() -> Result<(), EspError> {
    let mut bus = sys::spi_bus_config_t {
        sclk_io_num: board::SPI_SCK,
        data4_io_num: -1,
        data5_io_num: -1,
        data6_io_num: -1,
        data7_io_num: -1,
        ..Default::default()
    };
    bus.__bindgen_anon_1.mosi_io_num = board::SPI_MOSI;
    bus.__bindgen_anon_2.miso_io_num = board::SPI_MISO;
    bus.__bindgen_anon_3.quadwp_io_num = -1;
    bus.__bindgen_anon_4.quadhd_io_num = -1;
    let slave =
        sys::spi_slave_interface_config_t { spics_io_num: board::SPI_CS, mode: 0, queue_size: 1, ..Default::default() };
    esp!(unsafe {
        sys::spi_slave_initialize(sys::spi_host_device_t_SPI2_HOST, &bus, &slave, sys::spi_common_dma_t_SPI_DMA_CH_AUTO)
    })?;
    // Keeps CS idle while the STM32 is resetting, so no transfer eats a packet
    esp!(unsafe { sys::gpio_set_pull_mode(board::SPI_CS, sys::gpio_pull_mode_t_GPIO_PULLUP_ONLY) })
}

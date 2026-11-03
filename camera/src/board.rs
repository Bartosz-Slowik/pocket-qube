//! XIAO ESP32S3 Sense wiring (GPIO numbers).

// OV3660 camera, from CameraWebServer's camera_pins.h
pub const CAM_XCLK: i32 = 10;
pub const CAM_XCLK_HZ: i32 = 20_000_000;
pub const CAM_SDA: i32 = 40;
pub const CAM_SCL: i32 = 39;
/// D0..D7 (Y2..Y9 in camera_pins.h)
pub const CAM_DATA: [i32; 8] = [15, 17, 18, 16, 14, 12, 11, 48];
pub const CAM_VSYNC: i32 = 38;
pub const CAM_HREF: i32 = 47;
pub const CAM_PCLK: i32 = 13;

// SPI slave to the STM32's SPI2 (PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS)
pub const SPI_SCK: i32 = 7; // D8
pub const SPI_MISO: i32 = 8; // D9
pub const SPI_MOSI: i32 = 9; // D10
pub const SPI_CS: i32 = 4; // D3

/// User LED, lit when low. It's also the SD card's CS, so leave the SD slot empty.
pub const LED: i32 = 21;

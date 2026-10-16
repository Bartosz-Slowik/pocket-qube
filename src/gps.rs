use embassy_stm32::{
    bind_interrupts,
    peripherals::{PA10, USART1},
    usart::{BufferedUartRx, Config as UartConfig, InterruptHandler},
    Peri,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    signal::Signal,
};
use embassy_time::{with_timeout, Duration};
use static_cell::StaticCell;

bind_interrupts!(struct GpsIrqs {
    USART1 => InterruptHandler<USART1>;
});

static GPS_RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();

#[derive(Clone, Copy, Debug, Default)]
pub struct GpsData {
    pub fix: bool,
    pub sats: u8,
    pub latitude: f32,   // Decimal degrees (+ = N, - = S)
    pub longitude: f32,  // Decimal degrees (+ = E, - = W)
    pub altitude_m: f32, // Altitude in meters
}

impl defmt::Format for GpsData {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "fix={} sats={} lat={=f32} lon={=f32} alt={=f32}m",
            self.fix,
            self.sats,
            self.latitude,
            self.longitude,
            self.altitude_m,
        );
    }
}

pub static LATEST_GPS: Signal<CriticalSectionRawMutex, GpsData> = Signal::new();

pub type GpsUart = BufferedUartRx<'static>;

pub fn initialize(
    usart: impl Into<Peri<'static, USART1>>,
    rx_pin: impl Into<Peri<'static, PA10>>,
) -> Result<GpsUart, embassy_stm32::usart::ConfigError> {
    let mut config = UartConfig::default();
    config.baudrate = 9600;
    let rx_buf = GPS_RX_BUF.init([0u8; 256]);
    BufferedUartRx::new(usart.into(), GpsIrqs, rx_pin.into(), rx_buf, config)
}

fn parse_nmea_coord(coord_str: &str, dir: char) -> Option<f32> {
    if coord_str.is_empty() {
        return None;
    }
    let dot_pos = coord_str.find('.')?;
    if dot_pos < 2 {
        return None;
    }
    let deg_len = dot_pos - 2;
    let degrees: f32 = coord_str[..deg_len].parse().ok()?;
    let minutes: f32 = coord_str[deg_len..].parse().ok()?;
    let mut val = degrees + (minutes / 60.0);
    if dir == 'S' || dir == 'W' {
        val = -val;
    }
    Some(val)
}

pub fn parse_nmea_sentence(sentence: &str, gps_data: &mut GpsData) {
    let sentence = sentence.trim();
    if !sentence.starts_with('$') {
        return;
    }

    // Validate checksum if present (*XX)
    if let Some(star_pos) = sentence.find('*') {
        let payload = &sentence[1..star_pos];
        let checksum_str = &sentence[star_pos + 1..];
        if checksum_str.len() >= 2 {
            if let Ok(expected_ck) = u8::from_str_radix(&checksum_str[..2], 16) {
                let calculated_ck = payload.bytes().fold(0u8, |acc, b| acc ^ b);
                if calculated_ck != expected_ck {
                    return; // Corrupted sentence
                }
            }
        }
    }

    // Split sentence fields
    let mut fields = sentence.split(',');
    let talker = fields.next().unwrap_or("");

    // Process GGA sentences ($GPGGA, $GNGGA)
    if talker.ends_with("GGA") {
        let _utc = fields.next();
        let lat_str = fields.next().unwrap_or("");
        let lat_dir = fields.next().unwrap_or("").chars().next().unwrap_or(' ');
        let lon_str = fields.next().unwrap_or("");
        let lon_dir = fields.next().unwrap_or("").chars().next().unwrap_or(' ');
        let quality_str = fields.next().unwrap_or("0");
        let sats_str = fields.next().unwrap_or("0");
        let _hdop = fields.next();
        let alt_str = fields.next().unwrap_or("0");

        let quality: u8 = quality_str.parse().unwrap_or(0);
        gps_data.fix = quality > 0;
        gps_data.sats = sats_str.parse().unwrap_or(0);

        if let Some(lat) = parse_nmea_coord(lat_str, lat_dir) {
            gps_data.latitude = lat;
        }
        if let Some(lon) = parse_nmea_coord(lon_str, lon_dir) {
            gps_data.longitude = lon;
        }
        if let Ok(alt) = alt_str.parse::<f32>() {
            gps_data.altitude_m = alt;
        }
    }
    // Process RMC sentences ($GPRMC, $GNRMC)
    else if talker.ends_with("RMC") {
        let _utc = fields.next();
        let status = fields.next().unwrap_or("V");
        let lat_str = fields.next().unwrap_or("");
        let lat_dir = fields.next().unwrap_or("").chars().next().unwrap_or(' ');
        let lon_str = fields.next().unwrap_or("");
        let lon_dir = fields.next().unwrap_or("").chars().next().unwrap_or(' ');

        if status == "A" {
            gps_data.fix = true;
            if let Some(lat) = parse_nmea_coord(lat_str, lat_dir) {
                gps_data.latitude = lat;
            }
            if let Some(lon) = parse_nmea_coord(lon_str, lon_dir) {
                gps_data.longitude = lon;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn gps_task(mut uart: GpsUart) {
    defmt::info!("GPS ready (MAX-M10S)");
    let mut line_buf = [0u8; 128];
    let mut line_idx = 0;
    let mut current_gps = GpsData::default();
    let mut last_fix = false;

    loop {
        let mut byte = [0u8; 1];
        // with_timeout yields to other tasks while waiting — no more executor starvation.
        // If no byte arrives within 5 s, the GPS module is considered absent.
        match with_timeout(Duration::from_secs(5), uart.read(&mut byte)).await {
            Ok(Ok(_)) => {
                let ch = byte[0];
                if ch == b'\n' || ch == b'\r' {
                    if line_idx > 0 {
                        if let Ok(line_str) = core::str::from_utf8(&line_buf[..line_idx]) {
                            defmt::info!("NMEA: {=str}", line_str);
                            parse_nmea_sentence(line_str, &mut current_gps);
                            if current_gps.fix != last_fix {
                                if current_gps.fix {
                                    defmt::info!(
                                        "GPS 3D Fix Acquired: {=f32}, {=f32} ({=f32}m, {} sats)",
                                        current_gps.latitude,
                                        current_gps.longitude,
                                        current_gps.altitude_m,
                                        current_gps.sats
                                    );
                                } else {
                                    defmt::warn!("GPS Fix Lost");
                                }
                                last_fix = current_gps.fix;
                            }
                            LATEST_GPS.signal(current_gps);
                        }
                        line_idx = 0;
                    }
                } else if ch == b'$' {
                    line_buf[0] = b'$';
                    line_idx = 1;
                } else if line_idx > 0 {
                    if line_idx < line_buf.len() {
                        line_buf[line_idx] = ch;
                        line_idx += 1;
                    } else {
                        line_idx = 0; // Buffer overflow safeguard
                    }
                }
            }
            Ok(Err(_)) => {
                // UART framing/overrun error on a single byte — ignore and continue
            }
            Err(_) => {
                // Timeout: no data for 5 s — GPS module not responding
                defmt::warn!("GPS not connected");
            }
        }
    }
}


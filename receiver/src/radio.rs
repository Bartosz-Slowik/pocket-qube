//! LR2021 setup and reception. Every setting here must match the satellite's `src/radio.rs`.
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{Spi, mode::Master};
use embassy_time::{Duration, Timer, with_timeout};
use lr2021::flrc::{AgcPblLen, Crc, FlrcBitrate, FlrcCr, FlrcPacketParams, PktFormat, SwLen, SwMatch, SwTx};
use lr2021::lora::{HeaderType, Ldro, LoraBw, LoraCr, LoraModulationParams, LoraPacketParams, Sf};
use lr2021::radio::{FallbackMode, PacketType, RxBoost, RxPath};
use lr2021::status::{IRQ_MASK_CRC_ERROR, IRQ_MASK_RX_DONE, Intr};
use lr2021::system::{ChipMode, DioNum, LfClock, TcxoVoltage};
use lr2021::{BusyBlocking, Lr2021, Lr2021Error, PulseShape};

pub const LORA_HZ: u32 = 869_700_000;
pub const FLRC_HZ: u32 = 869_500_000;
const FLRC_SYNC: u32 = 0x5A3C_9E71;

// Blocking SPI: with DMA, reads got corrupted after the first packet
pub type Radio = Lr2021<Output<'static>, BlockingAsync<Spi<'static, Blocking, Master>>, BusyBlocking<Busy>>;

/// BUSY pin with a settle delay: the LR2021 raises BUSY a moment after NSS goes high,
/// and lr2021 checks it immediately, so it can read a response before it's ready.
pub struct Busy(pub Input<'static>);

impl embedded_hal::digital::ErrorType for Busy {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::InputPin for Busy {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        cortex_m::asm::delay(576); // ~8 us at 72 MHz
        Ok(self.0.is_high())
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        cortex_m::asm::delay(576);
        Ok(self.0.is_low())
    }
}

/// Radio error plus the step that failed.
pub type Error = (&'static str, Lr2021Error);

trait At<T> {
    fn at(self, step: &'static str) -> Result<T, Error>;
}

impl<T> At<T> for Result<T, Lr2021Error> {
    fn at(self, step: &'static str) -> Result<T, Error> {
        self.map_err(|e| (step, e))
    }
}

pub struct Pkt {
    pub len: usize,
    pub rssi: i16,
    /// Only LoRa reports SNR
    pub snr: i8,
    pub crc_ok: bool,
}

pub enum Rx {
    Packet(Pkt),
    /// Raw IRQ flags of a failed reception
    BadIrq(u32),
    Timeout,
}

/// Waits for one packet in the current mode; the radio stays in continuous RX.
pub async fn receive(
    radio: &mut Radio,
    dio8: &mut ExtiInput<'static>,
    buf: &mut [u8],
    timeout: Duration,
    lora: bool,
) -> Result<Rx, Error> {
    if with_timeout(timeout, dio8.wait_for_high()).await.is_err() {
        return Ok(Rx::Timeout);
    }

    let irq = radio.get_and_clear_irq().await.at("get_irq")?;
    // A CRC error still leaves the payload in the FIFO, so read it anyway
    if !irq.rx_done() && !irq.crc_error() {
        radio.clear_rx_fifo().await.at("clear_fifo")?;
        return Ok(Rx::BadIrq(irq.value()));
    }

    let (len, rssi, snr) = if lora {
        let s = radio.get_lora_packet_status().await.at("lora_status")?;
        (s.pkt_length() as usize, -(s.rssi_pkt() as i16) / 2, s.snr_pkt() / 4)
    } else {
        let s = radio.get_flrc_packet_status().await.at("flrc_status")?;
        (s.pkt_len() as usize, -(s.rssi_avg() as i16) / 2, 0)
    };
    let len = len.min(buf.len());
    radio.rd_rx_fifo_to(&mut buf[..len]).await.at("read_fifo")?;
    Ok(Rx::Packet(Pkt { len, rssi, snr, crc_ok: !irq.crc_error() }))
}

pub fn describe(irq: u32) -> &'static str {
    let irq = Intr::new(irq);
    if irq.len_error() {
        "length error"
    } else if irq.timeout() {
        "RX timeout"
    } else {
        "no RX done"
    }
}

/// Resets and configures the radio; leaves it receiving LoRa.
pub async fn setup(r: &mut Radio) -> Result<(), Error> {
    // lr2021 leaves NSS (PA4) low after a failed command, which breaks every later command
    embassy_stm32::pac::GPIOA.bsrr().write(|w| w.set_bs(4, true));
    r.reset().await.at("reset")?;
    r.wait_ready(Duration::from_millis(500)).await.at("boot")?;
    r.set_chip_mode(ChipMode::StandbyRc).await.at("standby")?;
    // LDO, not DC-DC: Semtech notes DC-DC degrades sub-GHz FLRC reception without a workaround
    r.set_regulator_mode(false).await.at("regulator")?;
    r.set_tcxo(TcxoVoltage::Tcxo2v2, 64_000).await.at("tcxo")?;
    r.set_lf_clk(LfClock::Rc).await.at("lf_clock")?;
    r.calibrate(true, true, true, true, true, true).await.at("calibrate")?;
    Timer::after_millis(20).await;

    r.set_rf(LORA_HZ).await.at("frequency")?;
    r.set_rx_path(RxPath::LfPath, RxBoost::Off).await.at("rx_path")?;
    r.set_fallback(FallbackMode::StandbyRc).await.at("fallback")?;
    r.set_dio_irq(DioNum::Dio8, Intr::new(IRQ_MASK_RX_DONE | IRQ_MASK_CRC_ERROR))
        .await
        .at("dio_irq")?;
    lora_mode(r).await
}

/// Telemetry: LoRa SF12/BW250/CR4-5 with LDRO, sync word 0x34, 8 preamble, explicit header, CRC on.
pub async fn lora_mode(r: &mut Radio) -> Result<(), Error> {
    r.set_chip_mode(ChipMode::StandbyRc).await.at("lora_standby")?;
    r.clear_rx_fifo().await.at("lora_clear_fifo")?;
    r.set_rf(LORA_HZ).await.at("lora_frequency")?;
    r.set_packet_type(PacketType::Lora).await.at("lora_type")?;
    r.set_lora_modulation(&LoraModulationParams::new(Sf::Sf12, LoraBw::Bw250, LoraCr::Cr1Ham45Si, Ldro::On))
        .await
        .at("lora_modulation")?;
    r.set_lora_syncword(0x34).await.at("lora_syncword")?;
    r.set_lora_packet(&LoraPacketParams::new(8, 255, HeaderType::Explicit, true, false))
        .await
        .at("lora_packet")?;
    // 0xFFFFFF = continuous RX: the chip keeps listening after each packet
    r.set_rx(0xFF_FFFF, false).await.at("lora_rx")
}

/// Images: FLRC 260 kb/s, CR 1/2, BT 0.5, 32-bit preamble and sync word, CRC-32.
pub async fn flrc_mode(r: &mut Radio) -> Result<(), Error> {
    r.set_chip_mode(ChipMode::StandbyRc).await.at("flrc_standby")?;
    r.clear_rx_fifo().await.at("flrc_clear_fifo")?;
    r.set_rf(FLRC_HZ).await.at("flrc_frequency")?;
    r.set_packet_type(PacketType::Flrc).await.at("flrc_type")?;
    r.set_flrc_modulation(FlrcBitrate::Br0260, FlrcCr::Cr12, PulseShape::Bt0p5)
        .await
        .at("flrc_modulation")?;
    r.set_flrc_syncword(1, FLRC_SYNC, false).await.at("flrc_syncword")?;
    let params = FlrcPacketParams::new(
        AgcPblLen::Len32Bits,
        SwLen::Sw32b,
        SwTx::Sw1,
        SwMatch::Match1,
        PktFormat::Dynamic,
        Crc::Crc32,
        255,
    );
    r.set_flrc_packet(&params).await.at("flrc_packet")?;
    r.set_rx(0xFF_FFFF, false).await.at("flrc_rx")
}

//! Telemetry goes out on robust LoRa, images on the most robust FLRC setting.
//! Every setting here must match the receiver's `receiver/src/main.rs`.
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{Spi, mode::Master};
use embassy_time::{Duration, Instant, Timer};
use lr2021::flrc::{AgcPblLen, Crc, FlrcBitrate, FlrcCr, FlrcPacketParams, PktFormat, SwLen, SwMatch, SwTx};
use lr2021::lora::{HeaderType, Ldro, LoraBw, LoraCr, LoraModulationParams, LoraPacketParams, Sf};
use lr2021::radio::{FallbackMode, PaLfMode, PacketType, RampTime, RxBoost, RxPath};
use lr2021::system::{ChipMode, LfClock, TcxoVoltage};
use lr2021::{BusyBlocking, Lr2021, Lr2021Error, PulseShape};

pub const MAX_LEN: usize = 255;
const LORA_HZ: u32 = 869_700_000;
const FLRC_HZ: u32 = 869_500_000;
const FLRC_SYNC: u32 = 0x5A3C_9E71;
/// Listen before talk: only a strong, nearby signal counts as the channel being in use
const LBT_THRESHOLD_DBM: i16 = -60;
/// Busy checks in a row before sending anyway, so a noisy channel can't silence the satellite
const LBT_TRIES: u32 = 3;
/// The longest `channel_clear` can take
pub const LBT_MAX: Duration = Duration::from_millis(40);

pub type Radio = Lr2021<Output<'static>, Spi<'static, Async, Master>, BusyBlocking<Input<'static>>>;

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

pub async fn setup(r: &mut Radio) -> Result<(), Error> {
    // lr2021 leaves NSS (PA4) low after a failed command, which breaks every later command
    embassy_stm32::pac::GPIOA.bsrr().write(|w| w.set_bs(4, true));
    r.reset().await.at("reset")?;
    r.wait_ready(Duration::from_millis(500)).await.at("boot")?;
    r.set_chip_mode(ChipMode::StandbyRc).await.at("standby")?;
    r.set_regulator_mode(true).await.at("regulator")?;
    r.set_tcxo(TcxoVoltage::Tcxo2v2, 64_000).await.at("tcxo")?;
    r.set_lf_clk(LfClock::Rc).await.at("lf_clock")?;
    r.calibrate(true, true, true, true, true, true).await.at("calibrate")?;
    Timer::after_millis(20).await;

    r.set_rf(LORA_HZ).await.at("frequency")?;
    r.set_rx_path(RxPath::LfPath, RxBoost::Off).await.at("rx_path")?;
    r.patch_simo(None).await.at("patch_simo")?;
    r.set_pa_lf(PaLfMode::LfPaFsm, 7, 7).await.at("pa")?;
    r.set_tx_params(44, RampTime::Ramp208u).await.at("tx_params")?;
    // Keeps the TCXO running between packets instead of restarting it for every TX and LBT
    r.set_fallback(FallbackMode::StandbyXosc).await.at("fallback")?;
    lora_mode(r).await
}

/// Telemetry: LoRa SF12/BW250/CR4-5 with LDRO, sync word 0x34, 8 preamble, explicit header, CRC on.
pub async fn lora_mode(r: &mut Radio) -> Result<(), Error> {
    // Mode changes are rejected right after TX or in StandbyXosc, like on the receiver
    r.set_chip_mode(ChipMode::StandbyRc).await.at("lora_standby")?;
    r.set_rf(LORA_HZ).await.at("lora_frequency")?;
    r.set_packet_type(PacketType::Lora).await.at("lora_type")?;
    // Semtech's drivers (and so the LoRa-E5) enable LDRO for SF12/BW250, so both ends must too
    r.set_lora_modulation(&LoraModulationParams::new(Sf::Sf12, LoraBw::Bw250, LoraCr::Cr1Ham45Si, Ldro::On))
        .await
        .at("lora_modulation")?;
    r.set_lora_syncword(0x34).await.at("lora_syncword")
}

/// Images: FLRC 260 kb/s, CR 1/2, BT 0.5, 32-bit preamble and sync word, CRC-32.
pub async fn flrc_mode(r: &mut Radio) -> Result<(), Error> {
    r.set_chip_mode(ChipMode::StandbyRc).await.at("flrc_standby")?;
    r.set_rf(FLRC_HZ).await.at("flrc_frequency")?;
    r.set_packet_type(PacketType::Flrc).await.at("flrc_type")?;
    r.set_flrc_modulation(FlrcBitrate::Br0260, FlrcCr::Cr12, PulseShape::Bt0p5)
        .await
        .at("flrc_modulation")?;
    r.set_flrc_syncword(1, FLRC_SYNC, false).await.at("flrc_syncword")
}

/// Listen before talk on the current frequency and modulation. Returns false if the channel
/// stayed busy for `LBT_TRIES` checks and the packet should go out anyway.
pub async fn channel_clear(r: &mut Radio) -> Result<bool, Error> {
    for _ in 0..LBT_TRIES {
        r.set_rx(0xFF_FFFF, true).await.at("lbt_rx")?;
        // Lets the receiver's gain settle before measuring
        Timer::after_micros(300).await;
        let dbm = -(r.get_rssi_avg(4).await.at("lbt_rssi")? as i16) / 2;
        r.set_chip_mode(ChipMode::StandbyXosc).await.at("lbt_standby")?;
        if dbm < LBT_THRESHOLD_DBM {
            return Ok(true);
        }
        // 2-10 ms, varied so two waiting transmitters don't retry in step
        Timer::after_millis(2 + Instant::now().as_ticks() % 9).await;
    }
    Ok(false)
}

/// Starts a LoRa packet; it's on air for `lora_airtime(data.len())`.
pub async fn send_lora(r: &mut Radio, data: &[u8]) -> Result<(), Error> {
    r.set_lora_packet(&LoraPacketParams::new(8, data.len() as u8, HeaderType::Explicit, true, false))
        .await
        .at("lora_packet")?;
    transmit(r, data).await
}

/// Starts an FLRC packet; it's on air for `flrc_airtime(data.len())`.
pub async fn send_flrc(r: &mut Radio, data: &[u8]) -> Result<(), Error> {
    let params = FlrcPacketParams::new(
        AgcPblLen::Len32Bits,
        SwLen::Sw32b,
        SwTx::Sw1,
        SwMatch::Match1,
        PktFormat::Dynamic,
        Crc::Crc32,
        data.len() as u16,
    );
    r.set_flrc_packet(&params).await.at("flrc_packet")?;
    transmit(r, data).await
}

async fn transmit(r: &mut Radio, data: &[u8]) -> Result<(), Error> {
    r.get_and_clear_irq().await.at("clear_irq")?;
    r.wr_tx_fifo_from(data).await.at("write_fifo")?;
    r.set_tx(0).await.at("set_tx")
}

/// Whether the packet started by `send_lora`/`send_flrc` has finished.
pub async fn sent(r: &mut Radio) -> Result<bool, Error> {
    Ok(r.get_status().await.at("tx_status")?.1.tx_done())
}

/// LoRa SF12/BW250/CR4-5/LDRO time on air, plus a 10 ms margin.
pub fn lora_airtime(len: usize) -> Duration {
    // Semtech formula: 20.25 symbols of preamble+header, then 5 symbols per 40 payload bits
    let blocks = (8 * len as u64 - 4).div_ceil(40);
    Duration::from_micros(331_776 + blocks * 5 * 16_384 + 10_000)
}

/// FLRC 260 kb/s CR 1/2 time on air (preamble + sync uncoded, rest coded), plus a 2 ms margin.
/// Only an upper bound: `sent` tells when a packet really finished.
pub fn flrc_airtime(len: usize) -> Duration {
    let bits = 64 + 2 * (16 + 8 * len as u64 + 32);
    Duration::from_micros(bits * 1_000_000 / 260_000 + 2_000)
}

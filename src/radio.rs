use embassy_stm32::{
    gpio::{Input, Output},
    spi::Spi,
};
use embassy_time::{Duration, Timer};
use lr2021::{
    lora::{HeaderType, Ldro, LoraBw, LoraCr, LoraModulationParams, LoraPacketParams, Sf},
    radio::{FallbackMode, PaLfMode, PacketType, RampTime, RxBoost, RxPath},
    system::{ChipMode, LfClock, TcxoVoltage},
    BusyBlocking, Lr2021, Lr2021Error,
};

use crate::telemetry::{self, PACKET_LEN};

const FREQUENCY_HZ: u32 = 850_000_000;
const TX_AIRTIME_WAIT: Duration = Duration::from_millis(300);

pub struct SpiWrapper(
    Spi<
        'static,
        embassy_stm32::mode::Blocking,
        embassy_stm32::spi::mode::Master,
    >,
);

impl SpiWrapper {
    pub fn new(
        spi: Spi<
            'static,
            embassy_stm32::mode::Blocking,
            embassy_stm32::spi::mode::Master,
        >,
    ) -> Self {
        Self(spi)
    }
}

impl embedded_hal::spi::ErrorType for SpiWrapper {
    type Error = embassy_stm32::spi::Error;
}

impl<W: embassy_stm32::spi::Word> embedded_hal_async::spi::SpiBus<W> for SpiWrapper {
    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.0.blocking_write(words)
    }

    async fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.0.blocking_read(words)
    }

    async fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Self::Error> {
        self.0.blocking_transfer(read, write)
    }

    async fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.0.blocking_transfer_in_place(words)
    }
}

pub type Radio = Lr2021<Output<'static>, SpiWrapper, BusyBlocking<Input<'static>>>;

async fn check_status(radio: &mut Radio) -> Result<(), Lr2021Error> {
    radio.get_status().await.map(|_| ())
}

pub async fn configure(mut radio: Radio) -> Result<Radio, Lr2021Error> {
    radio.reset().await?;
    radio.set_chip_mode(ChipMode::StandbyRc).await?;
    check_status(&mut radio).await?;
    radio.set_regulator_mode(true).await?; // DCDC mode
    check_status(&mut radio).await?;
    radio.set_tcxo(TcxoVoltage::Tcxo2v2, 64_000).await?; // 2.2V TCXO
    check_status(&mut radio).await?;
    radio.set_lf_clk(LfClock::Rc).await?; // RC LF clock
    check_status(&mut radio).await?;
    radio.calibrate(true, true, true, true, true, true).await?;
    Timer::after(Duration::from_millis(20)).await;
    check_status(&mut radio).await?;
    radio.get_version().await?;

    // Match exact Ebyte receiver default PHY configuration:
    // Frequency: 850.000 MHz
    // Modulation: LoRa (SF8, BW 125kHz, CR 4/5, No LDRO/PPM)
    // Packet: 8 Preamble symbols, 14-byte Payload, Explicit Header, CRC Enabled, Normal IQ
    // Sync Word: 0x34
    let modulation = LoraModulationParams::new(
        Sf::Sf8,
        LoraBw::Bw125,
        LoraCr::Cr1Ham45Si, // 4/5
        Ldro::Off,          // No PPM / LDRO
    );
    let packet_params = LoraPacketParams::new(
        8,                   // 8 preamble symbols
        PACKET_LEN as u8,    // 14 bytes payload
        HeaderType::Explicit,// Explicit header
        true,                // CRC enabled
        false,               // IQ inversion disabled (normal IQ)
    );

    radio.set_packet_type(PacketType::Lora).await?;
    check_status(&mut radio).await?;
    radio.set_lora_packet(&packet_params).await?;
    check_status(&mut radio).await?;
    radio.set_lora_syncword(0x34).await?;
    check_status(&mut radio).await?;
    radio.set_lora_modulation(&modulation).await?;
    check_status(&mut radio).await?;
    radio.set_rf(FREQUENCY_HZ).await?;
    check_status(&mut radio).await?;
    radio.set_rx_path(RxPath::LfPath, RxBoost::Off).await?;
    check_status(&mut radio).await?;
    radio.patch_simo(None).await?;
    radio.set_pa_lf(PaLfMode::LfPaFsm, 7, 6).await?;
    check_status(&mut radio).await?;
    radio.set_tx_params(0, RampTime::Ramp208u).await?;
    check_status(&mut radio).await?;
    radio.set_fallback(FallbackMode::StandbyRc).await?;
    check_status(&mut radio).await?;

    defmt::info!("LR2021 ready: 850 MHz, SF8, BW125, CR4/5, Sync 0x34, Preamble 8, Explicit, CRC ON");
    Ok(radio)
}

#[embassy_executor::task]
pub async fn radio_task(mut radio: Radio) {
    let mut sequence = 0u8;

    loop {
        let telemetry = telemetry::LATEST.wait().await;
        let mut packet = [0u8; PACKET_LEN];
        telemetry::encode(&mut packet, sequence, telemetry);
        radio.buffer_mut()[..PACKET_LEN].copy_from_slice(&packet);

        if let Err(error) = radio.get_and_clear_irq().await {
            defmt::error!("IRQ clear before TX failed: {}", error);
            continue;
        }

        if let Err(error) = radio.wr_tx_fifo(PACKET_LEN).await {
            defmt::error!("FIFO write failed: {}", error);
            continue;
        }

        if let Err(error) = check_status(&mut radio).await {
            defmt::error!("FIFO status failed: {}", error);
            continue;
        }

        if let Err(error) = radio.set_tx(0).await {
            defmt::error!("TX start failed: {}", error);
            continue;
        }

        Timer::after(TX_AIRTIME_WAIT).await;

        match radio.get_and_clear_irq().await {
            Ok(_) => defmt::debug!("TX #{:03} | {}", sequence, telemetry),
            Err(error) => defmt::error!("TX #{:03} failed: {}", sequence, error),
        }

        sequence = sequence.wrapping_add(1);
    }
}

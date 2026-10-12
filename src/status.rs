//! Reading the chip's state back: what its registers hold, decoded into the driver's types.
//!
//! The state is spread over two pages, so it comes as two snapshots: [`DigitalStatus`] for page 0
//! ([`Tlv320dac3203::read_digital_status`]) and [`AnalogStatus`] for page 1
//! ([`Tlv320dac3203::read_analog_status`]). Each reads the registers of its list, every one after
//! a select of its page, [`DigitalStatus::REGISTERS`] or
//! [`AnalogStatus::REGISTERS`]: the registers the stock kernel programs for playback, plus the
//! three flag registers that are not sticky. A compile-time check keeps the clear-on-read
//! registers and the page-select register off both lists.
//!
//! # What a snapshot is and is not
//!
//! - **One register per read.** A longer read would let the chip's address counter run on into
//!   registers that are not on the list (§2.8.1), the sticky ones among them.
//! - **Not one instant.** The registers are read one after the other, and the live flags (page 0,
//!   registers 37 and 38; page 1, register 63) can change in between.
//! - **The bytes as read.** A snapshot keeps the raw register values ([`DigitalStatus::raw`],
//!   [`DigitalStatus::register`]) and decodes them on request. A decoder that finds a code the
//!   guide marks reserved returns [`ReservedCode`] and leaves the other decoders usable.
//! - **Only what the driver has types for.** Bits outside the fields of the driver's types are
//!   not decoded: reserved bits, and the features the other modules leave out (loopback,
//!   auto-mute, volume ganging, the DRC's threshold and timing, the analog bypass). They are
//!   still in the raw values.
//!
//! The decoders are plain functions of the bytes, so a snapshot can also be made from bytes that
//! came from elsewhere ([`DigitalStatus::from_raw`], [`AnalogStatus::from_raw`]).

use embedded_hal::i2c::I2c;

use crate::clock::{
    ClockConfig, CodecInput, DividerConfig, Dosr, Pll, PllD, PllInput, PllJ, PllP, PllR, PllRange,
};
use crate::dac::{DacSetup, DacVolume, ProcessingBlock};
use crate::headphone::{HeadphoneGain, HeadphoneStartup};
use crate::interface::InterfaceConfig;
use crate::pins::Mfp2Config;
use crate::register::{
    self, Register, dac_flags_1, dac_flags_2, dac_setup_1, dac_setup_2, drc_control_1, hp_gain,
    hp_gain_flags, hpl_routing, hpr_routing, ldo_control, output_driver_power, page,
};
use crate::supply::{ReferencePowerUp, SupplyConfig};
use crate::{Channels, Error, Tlv320dac3203};

/// A register holds a code that the guide marks reserved, so the value read has no meaning the
/// driver's types could carry.
///
/// This is an answer about the bytes, not a bus error: the read itself worked. The raw value is
/// in the snapshot ([`DigitalStatus::register`], [`AnalogStatus::register`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReservedCode {
    /// The register with the reserved code. For a number that spans two registers (the PLL's D,
    /// the oversampling ratio) it is the first of the two.
    pub register: Register,
}

/// Which blocks report that they are powered up: page 0, register 37 (§5.2.36).
///
/// These are the chip's own status flags, not the power bits somebody wrote. A DAC channel's flag
/// follows its power bit only once the clocks run and the chip's power sequence has finished
/// (§2.7), so the two can differ for a while.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PowerFlags {
    /// The DAC channels that report powered up: bits D7 (left) and D3 (right).
    pub dacs: Channels,
    /// The headphone drivers that report powered up: bits D5 (HPL) and D1 (HPR).
    pub headphones: Channels,
}

/// How many registers a [`DigitalStatus`] holds.
const DIGITAL_LEN: usize = 24;

/// How many registers an [`AnalogStatus`] holds.
const ANALOG_LEN: usize = 11;

/// Where `register` sits in `list`.
///
/// Only called inside `const` blocks (the two macros below), so a register that is missing from
/// its list stops the build and can never panic on the device.
const fn position(list: &[Register], register: Register) -> usize {
    let mut index = 0;
    while index < list.len() {
        if list[index].page() == register.page() && list[index].addr() == register.addr() {
            return index;
        }
        index += 1;
    }
    panic!("the register is not on the snapshot's list");
}

/// The byte a [`DigitalStatus`] holds for one of its registers. The position is worked out at
/// compile time.
macro_rules! digital {
    ($status:expr, $register:ident) => {
        $status.raw[const { position(&DigitalStatus::REGISTERS, register::$register) }]
    };
}

/// The byte an [`AnalogStatus`] holds for one of its registers. The position is worked out at
/// compile time.
macro_rules! analog {
    ($status:expr, $register:ident) => {
        $status.raw[const { position(&AnalogStatus::REGISTERS, register::$register) }]
    };
}

/// The channels whose bit is set in `value`.
const fn channels(value: u8, left: u8, right: u8) -> Channels {
    Channels {
        left: value & left != 0,
        right: value & right != 0,
    }
}

/// The value `list` and `raw` hold for `register`, if it is on the list.
fn lookup(list: &[Register], raw: &[u8], register: Register) -> Option<u8> {
    list.iter()
        .zip(raw)
        .find(|(listed, _)| **listed == register)
        .map(|(_, value)| *value)
}

/// What page 0 holds: clocks, serial interface, status flags, the MFP2 pin, the processing
/// block, the DAC channels, the digital volume and the DRC switches.
///
/// Made by [`Tlv320dac3203::read_digital_status`]; the module
/// documentation says what a snapshot is and is not.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DigitalStatus {
    /// The register values as read, in the order of [`Self::REGISTERS`].
    raw: [u8; DIGITAL_LEN],
}

impl DigitalStatus {
    /// The registers of the snapshot, in the order they are read and in which [`Self::raw`]
    /// holds them: page 0, registers 4 to 8, 11 to 14, 27 to 30, 37, 38, 53, 60, 63 to 66 and 68
    /// to 70.
    ///
    /// These are the page 0 registers that are safe as plain reads. None of them changes when it is
    /// read: 37 and 38 are live status flags (§5.2.36 and §5.2.37), the rest are settings. The
    /// sticky registers 42, 44 and 45 are not among them ([`register::CLEAR_ON_READ`]).
    pub const REGISTERS: [Register; DIGITAL_LEN] = [
        register::CLOCK_MUX,
        register::PLL_P_R,
        register::PLL_J,
        register::PLL_D_MSB,
        register::PLL_D_LSB,
        register::NDAC,
        register::MDAC,
        register::DOSR_MSB,
        register::DOSR_LSB,
        register::INTERFACE_1,
        register::DATA_OFFSET,
        register::INTERFACE_3,
        register::BCLK_N,
        register::DAC_FLAGS_1,
        register::DAC_FLAGS_2,
        register::MFP2_CONTROL,
        register::DAC_PROCESSING_BLOCK,
        register::DAC_SETUP_1,
        register::DAC_SETUP_2,
        register::LEFT_DAC_VOLUME,
        register::RIGHT_DAC_VOLUME,
        register::DRC_CONTROL_1,
        register::DRC_CONTROL_2,
        register::DRC_CONTROL_3,
    ];

    /// A snapshot from register values that were obtained elsewhere, in the order of
    /// [`Self::REGISTERS`]. Touches no bus.
    pub const fn from_raw(raw: [u8; DIGITAL_LEN]) -> Self {
        Self { raw }
    }

    /// The register values as read, in the order of [`Self::REGISTERS`].
    pub const fn raw(&self) -> &[u8; DIGITAL_LEN] {
        &self.raw
    }

    /// The value read from `register`, or `None` if the snapshot does not hold that register.
    pub fn register(&self, register: Register) -> Option<u8> {
        lookup(&Self::REGISTERS, &self.raw, register)
    }

    /// The clock tree: registers 4 to 8, 11 to 14 and 30 (§5.2.5 to §5.2.14 and §5.2.30). The power
    /// bits of the PLL and the dividers are part of it, as in [`ClockConfig`].
    ///
    /// Fails for a reserved PLL multiplier R (register 5) or J (register 6), a PLL fraction D
    /// above 9999 (registers 7 and 8) or a reserved oversampling ratio (registers 13 and 14).
    pub fn clocks(&self) -> Result<ClockConfig, ReservedCode> {
        let mux = digital!(self, CLOCK_MUX);
        let p_r = digital!(self, PLL_P_R);
        let reserved = |register| ReservedCode { register };
        let r = PllR::from_bits(p_r).ok_or(reserved(register::PLL_P_R))?;
        let j = PllJ::from_bits(digital!(self, PLL_J)).ok_or(reserved(register::PLL_J))?;
        let d = PllD::from_bytes(digital!(self, PLL_D_MSB), digital!(self, PLL_D_LSB))
            .ok_or(reserved(register::PLL_D_MSB))?;
        let dosr = Dosr::from_bytes(digital!(self, DOSR_MSB), digital!(self, DOSR_LSB))
            .ok_or(reserved(register::DOSR_MSB))?;
        Ok(ClockConfig {
            pll_range: PllRange::from_bits(mux),
            pll_input: PllInput::from_bits(mux),
            codec_input: CodecInput::from_bits(mux),
            pll: Pll {
                powered: Pll::powered_from_bits(p_r),
                p: PllP::from_bits(p_r),
                r,
                j,
                d,
            },
            ndac: DividerConfig::from_bits(digital!(self, NDAC)),
            mdac: DividerConfig::from_bits(digital!(self, MDAC)),
            dosr,
            bclk_n: DividerConfig::from_bits(digital!(self, BCLK_N)),
        })
    }

    /// The serial interface: registers 27, 28 and 29 (§5.2.27 to §5.2.29).
    ///
    /// Fails for a reserved BCLK divider input (register 29). The loopback bit and the data
    /// output's high-impedance bit are not part of [`InterfaceConfig`] and not decoded.
    pub fn interface(&self) -> Result<InterfaceConfig, ReservedCode> {
        InterfaceConfig::from_registers(
            digital!(self, INTERFACE_1),
            digital!(self, DATA_OFFSET),
            digital!(self, INTERFACE_3),
        )
        .ok_or(ReservedCode {
            register: register::INTERFACE_3,
        })
    }

    /// Which DAC channels and headphone drivers report powered up: register 37 (§5.2.36).
    pub fn power_flags(&self) -> PowerFlags {
        let flags = digital!(self, DAC_FLAGS_1);
        PowerFlags {
            dacs: channels(
                flags,
                dac_flags_1::LEFT_DAC_POWERED,
                dac_flags_1::RIGHT_DAC_POWERED,
            ),
            headphones: channels(flags, dac_flags_1::HPL_POWERED, dac_flags_1::HPR_POWERED),
        }
    }

    /// The DAC channels whose applied digital volume equals the programmed one, that is, whose
    /// soft-stepping has finished: register 38 (§5.2.37).
    pub fn volume_applied(&self) -> Channels {
        channels(
            digital!(self, DAC_FLAGS_2),
            dac_flags_2::LEFT_GAIN_APPLIED,
            dac_flags_2::RIGHT_GAIN_APPLIED,
        )
    }

    /// The MFP2 pin: register 53 (§5.2.49). Every value of its fields has a meaning, so this cannot
    /// fail.
    pub fn mfp2(&self) -> Mfp2Config {
        Mfp2Config::from_bits(digital!(self, MFP2_CONTROL))
    }

    /// The DAC processing block: register 60 (§5.2.54). Fails for the reserved codes 0 and 26
    /// to 31.
    pub fn processing_block(&self) -> Result<ProcessingBlock, ReservedCode> {
        ProcessingBlock::from_bits(digital!(self, DAC_PROCESSING_BLOCK)).ok_or(ReservedCode {
            register: register::DAC_PROCESSING_BLOCK,
        })
    }

    /// The DAC channels whose power bit is set: register 63, bits D7 and D6 (§5.2.57). This is the
    /// setting; [`power_flags`](Self::power_flags) is what the chip reports.
    pub fn dac_power(&self) -> Channels {
        channels(
            digital!(self, DAC_SETUP_1),
            dac_setup_1::LEFT_POWER_UP,
            dac_setup_1::RIGHT_POWER_UP,
        )
    }

    /// What the two DACs play and how the digital volume steps: register 63, bits D5 to D0
    /// (§5.2.57). Fails for the reserved soft-stepping code.
    pub fn dac_setup(&self) -> Result<DacSetup, ReservedCode> {
        DacSetup::from_bits(digital!(self, DAC_SETUP_1)).ok_or(ReservedCode {
            register: register::DAC_SETUP_1,
        })
    }

    /// The DAC channels that are muted: register 64, bits D3 and D2 (§5.2.58). The auto-mute and
    /// volume-ganging fields of the register are not decoded.
    pub fn dac_mute(&self) -> Channels {
        channels(
            digital!(self, DAC_SETUP_2),
            dac_setup_2::LEFT_MUTE,
            dac_setup_2::RIGHT_MUTE,
        )
    }

    /// The digital volume, left and right: registers 65 and 66 (§5.2.59 and §5.2.60). Fails for a
    /// reserved code, naming the register that holds it.
    pub fn volume(&self) -> Result<(DacVolume, DacVolume), ReservedCode> {
        let left = DacVolume::from_bits(digital!(self, LEFT_DAC_VOLUME)).ok_or(ReservedCode {
            register: register::LEFT_DAC_VOLUME,
        })?;
        let right = DacVolume::from_bits(digital!(self, RIGHT_DAC_VOLUME)).ok_or(ReservedCode {
            register: register::RIGHT_DAC_VOLUME,
        })?;
        Ok((left, right))
    }

    /// The channels whose dynamic range compression is enabled: register 68, bits D6 and D5
    /// (§5.2.62). Both are enabled after a reset, and the stock kernel clears both for every stream
    /// (`aic32x4.c:953-959`).
    ///
    /// The DRC's threshold, hysteresis, hold, attack and decay (the rest of register 68 and
    /// registers 69 and 70) are in the snapshot as raw values only, because the driver has no
    /// types for them.
    pub fn drc_enabled(&self) -> Channels {
        channels(
            digital!(self, DRC_CONTROL_1),
            drc_control_1::LEFT_ENABLE,
            drc_control_1::RIGHT_ENABLE,
        )
    }

    /// Whether every clock register holds its reset value ([`CLOCK_RESET_VALUES`]): nothing has
    /// set the clocks up since the chip's last reset. False as soon as one differs; the stock
    /// kernel's values on the Echo Dot 2 differ in five of them (R4, R5, R6, R11, R12), and stay
    /// different after a stream stops, which only clears power bits.
    pub fn clocks_at_reset(&self) -> bool {
        let mut index = 0;
        while index < CLOCK_RESET_VALUES.len() {
            let (register, value) = CLOCK_RESET_VALUES[index];
            if self.register(register) != Some(value) {
                return false;
            }
            index += 1;
        }
        true
    }
}

/// The clock registers of page 0 with their reset values, all part of
/// [`DigitalStatus::REGISTERS`]: R4 `0x00` (MCLK into the PLL and the codec, low PLL range,
/// §5.2.5), R5 `0x11` (PLL powered down, P = 1, R = 1, §5.2.6), R6 `0x04` (J = 4, §5.2.7), R7 and
/// R8 `0x00` (D = 0, §5.2.8 and §5.2.9), R11 and R12 `0x01` (NDAC and MDAC 1, powered down, §5.2.11
/// and §5.2.12), R13 `0x00` and R14 `0x80` (DOSR = 128, §5.2.13 and §5.2.14).
pub const CLOCK_RESET_VALUES: [(Register, u8); 9] = [
    (register::CLOCK_MUX, 0x00),
    (register::PLL_P_R, 0x11),
    (register::PLL_J, 0x04),
    (register::PLL_D_MSB, 0x00),
    (register::PLL_D_LSB, 0x00),
    (register::NDAC, 0x01),
    (register::MDAC, 0x01),
    (register::DOSR_MSB, 0x00),
    (register::DOSR_LSB, 0x80),
];

/// What page 1 holds: supply settings, the headphone drivers with their routing, gain, mute and
/// start-up behaviour, their gain flags, and the reference setting.
///
/// Made by [`Tlv320dac3203::read_analog_status`]; the module
/// documentation says what a snapshot is and is not.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AnalogStatus {
    /// The register values as read, in the order of [`Self::REGISTERS`].
    raw: [u8; ANALOG_LEN],
}

impl AnalogStatus {
    /// The registers of the snapshot, in the order they are read and in which [`Self::raw`]
    /// holds them: page 1, registers 1, 2, 9, 10, 12, 13, 16, 17, 20, 63 and 123.
    ///
    /// These are the page 1 registers that are safe as plain reads. None of them changes when it is
    /// read: register 63 holds live flags (§5.3.25), and so do bits D2 and D1 of register 2
    /// (§5.3.3); the rest are settings.
    pub const REGISTERS: [Register; ANALOG_LEN] = [
        register::POWER_CONFIG,
        register::LDO_CONTROL,
        register::OUTPUT_DRIVER_POWER,
        register::COMMON_MODE,
        register::HPL_ROUTING,
        register::HPR_ROUTING,
        register::HPL_GAIN,
        register::HPR_GAIN,
        register::HP_STARTUP,
        register::HP_GAIN_FLAGS,
        register::REFERENCE_POWER_UP,
    ];

    /// A snapshot from register values that were obtained elsewhere, in the order of
    /// [`Self::REGISTERS`]. Touches no bus.
    pub const fn from_raw(raw: [u8; ANALOG_LEN]) -> Self {
        Self { raw }
    }

    /// The register values as read, in the order of [`Self::REGISTERS`].
    pub const fn raw(&self) -> &[u8; ANALOG_LEN] {
        &self.raw
    }

    /// The value read from `register`, or `None` if the snapshot does not hold that register.
    pub fn register(&self, register: Register) -> Option<u8> {
        lookup(&Self::REGISTERS, &self.raw, register)
    }

    /// The supply settings: registers 1, 2 and 10 (§5.3.2, §5.3.3 and §5.3.8). Fails for the
    /// reserved LDO voltage code (register 2).
    pub fn supply(&self) -> Result<SupplyConfig, ReservedCode> {
        SupplyConfig::from_registers(
            analog!(self, POWER_CONFIG),
            analog!(self, LDO_CONTROL),
            analog!(self, COMMON_MODE),
        )
        .ok_or(ReservedCode {
            register: register::LDO_CONTROL,
        })
    }

    /// Whether the chip reports its offset correction as completed: register 2, bit D2, a read-only
    /// flag (§5.3.3).
    pub fn offset_correction_done(&self) -> bool {
        analog!(self, LDO_CONTROL) & ldo_control::OFFSET_CORRECTION_DONE != 0
    }

    /// Whether the chip reports over-current on its AVDD LDO: register 2, bit D1, a read-only flag
    /// (§5.3.3).
    pub fn avdd_ldo_over_current(&self) -> bool {
        analog!(self, LDO_CONTROL) & ldo_control::AVDD_LDO_OVER_CURRENT != 0
    }

    /// The headphone drivers whose power bit is set: register 9, bits D5 and D4 (§5.3.7). What the
    /// chip reports for them is in page 0 ([`DigitalStatus::power_flags`]).
    pub fn headphone_power(&self) -> Channels {
        channels(
            analog!(self, OUTPUT_DRIVER_POWER),
            output_driver_power::HPL_POWER_UP,
            output_driver_power::HPR_POWER_UP,
        )
    }

    /// The channels whose DAC is routed to the headphone driver of its side: bit D3 of register 12
    /// (left DAC to HPL) and of register 13 (right DAC to HPR) (§5.3.10 and §5.3.11). The analog
    /// bypass and the left DAC's negative terminal are not decoded.
    pub fn dac_routing(&self) -> Channels {
        Channels {
            left: analog!(self, HPL_ROUTING) & hpl_routing::LEFT_DAC != 0,
            right: analog!(self, HPR_ROUTING) & hpr_routing::RIGHT_DAC != 0,
        }
    }

    /// The gain of the two headphone drivers, HPL and HPR: bits D5 to D0 of registers 16 and 17
    /// (§5.3.13 and §5.3.14). Fails for a reserved code, naming the register that holds it.
    pub fn headphone_gain(&self) -> Result<(HeadphoneGain, HeadphoneGain), ReservedCode> {
        let hpl = HeadphoneGain::from_bits(analog!(self, HPL_GAIN)).ok_or(ReservedCode {
            register: register::HPL_GAIN,
        })?;
        let hpr = HeadphoneGain::from_bits(analog!(self, HPR_GAIN)).ok_or(ReservedCode {
            register: register::HPR_GAIN,
        })?;
        Ok((hpl, hpr))
    }

    /// The headphone drivers that are muted: bit D6 of registers 16 and 17 (§5.3.13 and §5.3.14).
    pub fn headphone_mute(&self) -> Channels {
        Channels {
            left: analog!(self, HPL_GAIN) & hp_gain::MUTE != 0,
            right: analog!(self, HPR_GAIN) & hp_gain::MUTE != 0,
        }
    }

    /// The headphone drivers' start-up behaviour: register 20 (§5.3.16). Fails for the reserved
    /// resistor code.
    pub fn headphone_startup(&self) -> Result<HeadphoneStartup, ReservedCode> {
        HeadphoneStartup::from_bits(analog!(self, HP_STARTUP)).ok_or(ReservedCode {
            register: register::HP_STARTUP,
        })
    }

    /// The headphone drivers whose applied gain equals the programmed one: register 63, bits D7 and
    /// D6 (§5.3.25). These are the flags
    /// [`wait_for_headphone_gain`](Tlv320dac3203::wait_for_headphone_gain) polls. The two bypass
    /// volume flags in the same register are not decoded.
    pub fn headphone_gain_applied(&self) -> Channels {
        channels(
            analog!(self, HP_GAIN_FLAGS),
            hp_gain_flags::HPL_GAIN_APPLIED,
            hp_gain_flags::HPR_GAIN_APPLIED,
        )
    }

    /// How the analog reference powers up: register 123 (§5.3.29). Every value of its fields has a
    /// meaning, so this cannot fail.
    pub fn reference_power_up(&self) -> ReferencePowerUp {
        ReferencePowerUp::from_bits(analog!(self, REFERENCE_POWER_UP))
    }
}

// The two lists are what a snapshot reads. These assertions fail the build if one of them names a
// register of the wrong page, the page-select register, a register whose read clears sticky flags,
// or the same register twice.
const _: () = {
    const fn check(list: &[Register], page: u8) {
        let mut index = 0;
        while index < list.len() {
            let register = list[index];
            assert!(register.page() == page);
            assert!(register.addr() != register::PAGE_SELECT);
            assert!(!register.is_clear_on_read());
            // Ascending addresses: no register is read twice, and the order on the bus is the
            // order of the documentation.
            assert!(index == 0 || list[index - 1].addr() < register.addr());
            index += 1;
        }
    }
    check(&DigitalStatus::REGISTERS, page::DIGITAL);
    check(&AnalogStatus::REGISTERS, page::ANALOG);
};

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Reads the page 0 snapshot: the registers of [`DigitalStatus::REGISTERS`] in order. The first
    /// failed read ends the call.
    pub fn read_digital_status(&mut self) -> Result<DigitalStatus, Error<I2C::Error>> {
        let mut raw = [0u8; DIGITAL_LEN];
        for (value, register) in raw.iter_mut().zip(DigitalStatus::REGISTERS) {
            *value = self.read(register)?;
        }
        Ok(DigitalStatus { raw })
    }

    /// Reads the page 1 snapshot: the registers of [`AnalogStatus::REGISTERS`] in order. The first
    /// failed read ends the call.
    pub fn read_analog_status(&mut self) -> Result<AnalogStatus, Error<I2C::Error>> {
        let mut raw = [0u8; ANALOG_LEN];
        for (value, register) in raw.iter_mut().zip(AnalogStatus::REGISTERS) {
            *value = self.read(register)?;
        }
        Ok(AnalogStatus { raw })
    }
}

#[cfg(test)]
mod tests {
    //! Decoding only: these tests build snapshots from bytes and involve no bus. The read path
    //! has been exercised on a chip, read-only, on the Echo Dot 2; the probe on top of it has
    //! mock tests in
    //! [`crate::probe`].

    use super::*;
    use crate::{
        ClockDirection, Divisor, Dosr, Format, HeadphoneCommonMode, HeadphoneSupply, LdoVoltage,
        Mfp2Function, PllJ, PllP, PllR, ProcessingBlock, ReferenceChargeTime, Rpop, SlowCharge,
        SoftRoutingStep, SoftStepping, WordLength,
    };

    /// Page 0 of a chip that plays a stereo stream at 48 kHz from a 9.6 MHz master clock through
    /// the PLL (P = 5, R = 1, J = 48, NDAC 5, MDAC 3, DOSR 128), I2S with 16-bit words from the
    /// host, the left DAC muted: in the order of `DigitalStatus::REGISTERS`.
    const PLAYING_PAGE_0: [u8; DIGITAL_LEN] = [
        0x03, 0xD1, 0x30, 0x00, 0x00, // R4 to R8: clock multiplexers and PLL
        0x85, 0x83, 0x00, 0x80, // R11 to R14: NDAC, MDAC, DOSR
        0x00, 0x00, 0x01, 0x81, // R27 to R30: interface, BCLK divider
        0xAA, 0x11, // R37, R38: everything powered, volume applied
        0x05, // R53: MFP2 a high output
        0x02, // R60: PRB_P2
        0xD6, 0x08, 0x00, 0x00, // R63 to R66: both DACs on, left one muted, 0 dB
        0x0F, 0x00, 0x00, // R68 to R70: DRC off
    ];

    /// Page 1 for the same state, in the order of `AnalogStatus::REGISTERS`.
    const PLAYING_PAGE_1: [u8; ANALOG_LEN] = [
        0x08, 0x01, // R1, R2: weak AVDD connection off, LDO on, analog blocks enabled
        0x30, 0x33, // R9, R10: both drivers powered, common modes and supply
        0x08, 0x08, // R12, R13: each DAC routed to its driver
        0x00, 0x00, // R16, R17: 0 dB, unmuted
        0x1D, // R20: de-pop
        0xC0, // R63: both gains applied
        0x01, // R123: reference in 40 ms
    ];

    /// The interface of [`PLAYING_PAGE_0`]: I2S, 16 bits, both clocks inputs, the BCLK divider
    /// fed from DAC_MOD_CLK.
    const PLAYING_INTERFACE: InterfaceConfig = InterfaceConfig {
        bclk_divider_input: crate::BclkDividerInput::DacModClk,
        ..InterfaceConfig::RESET
    };

    /// The DAC setup of [`PLAYING_PAGE_0`]: each DAC its own side, soft-stepping disabled.
    const PLAYING_DAC: DacSetup = DacSetup {
        soft_stepping: SoftStepping::Disabled,
        ..DacSetup::RESET
    };

    /// The decoders are the inverse of the encoders the write tests pin.
    /// The snapshots read their lists in order, each register after a select of its page.
    #[test]
    fn the_snapshots_select_the_page_before_every_register() {
        use crate::ADDRESS;
        use embedded_hal_mock::eh1::i2c::{Mock, Transaction};
        let select = |page| Transaction::write(ADDRESS, vec![register::PAGE_SELECT, page]);
        let read = |register: Register, value| {
            Transaction::write_read(ADDRESS, vec![register.addr()], vec![value])
        };
        let mut expected = vec![];
        for (register, value) in AnalogStatus::REGISTERS.into_iter().zip(PLAYING_PAGE_1) {
            expected.push(select(1));
            expected.push(read(register, value));
        }
        for (register, value) in DigitalStatus::REGISTERS.into_iter().zip(PLAYING_PAGE_0) {
            expected.push(select(0));
            expected.push(read(register, value));
        }
        let mut i2c = Mock::new(&expected);
        let mut dac = Tlv320dac3203::new(&mut i2c, ADDRESS);
        assert_eq!(
            dac.read_analog_status(),
            Ok(AnalogStatus::from_raw(PLAYING_PAGE_1))
        );
        assert_eq!(
            dac.read_digital_status(),
            Ok(DigitalStatus::from_raw(PLAYING_PAGE_0))
        );
        i2c.done();
    }

    #[test]
    fn digital_status_decodes_a_playing_chip() {
        let status = DigitalStatus::from_raw(PLAYING_PAGE_0);
        let clocks = status.clocks().unwrap();
        assert_eq!(clocks.pll_range, crate::PllRange::Low);
        assert_eq!(clocks.pll_input, crate::PllInput::Mclk);
        assert_eq!(clocks.codec_input, crate::CodecInput::Pll);
        assert!(clocks.pll.powered);
        assert_eq!(clocks.pll.p, PllP::new(5).unwrap());
        assert_eq!(clocks.pll.r, PllR::new(1).unwrap());
        assert_eq!(clocks.pll.j, PllJ::new(48).unwrap());
        assert_eq!(clocks.pll.d.get(), 0);
        let powered = |divisor| DividerConfig {
            powered: true,
            divisor: Divisor::new(divisor).unwrap(),
        };
        assert_eq!(clocks.ndac, powered(5));
        assert_eq!(clocks.mdac, powered(3));
        assert_eq!(clocks.dosr, Dosr::new(128).unwrap());
        assert_eq!(clocks.bclk_n, powered(1));
        let interface = status.interface().unwrap();
        assert_eq!(interface, PLAYING_INTERFACE);
        assert_eq!(interface.format, Format::I2s);
        assert_eq!(interface.word_length, WordLength::Bits16);
        assert_eq!(interface.bclk, ClockDirection::Input);
        assert_eq!(
            status.power_flags(),
            PowerFlags {
                dacs: Channels::BOTH,
                headphones: Channels::BOTH,
            }
        );
        assert_eq!(status.volume_applied(), Channels::BOTH);
        assert_eq!(status.mfp2().function, Mfp2Function::Output { high: true });
        assert!(status.mfp2().bus_keeper);
        assert_eq!(status.processing_block(), Ok(ProcessingBlock::PRB_P2));
        assert_eq!(status.dac_power(), Channels::BOTH);
        assert_eq!(status.dac_setup(), Ok(PLAYING_DAC));
        assert_eq!(status.dac_mute(), Channels::LEFT);
        assert_eq!(
            status.volume(),
            Ok((DacVolume::ZERO_DB, DacVolume::ZERO_DB))
        );
        assert_eq!(status.drc_enabled(), Channels::NONE);
        assert_eq!(status.register(register::PLL_J), Some(0x30));
        assert_eq!(status.register(register::HP_STARTUP), None);
    }

    /// A snapshot is at the clock reset values only if all nine clock registers are; a playing
    /// chip's are not.
    #[test]
    fn clocks_at_reset_needs_every_clock_register_at_its_reset_value() {
        let mut raw = PLAYING_PAGE_0;
        assert!(!DigitalStatus::from_raw(raw).clocks_at_reset());
        // R4 to R8, then R11 to R14: the first nine of `REGISTERS`.
        raw[..9].copy_from_slice(&[0x00, 0x11, 0x04, 0x00, 0x00, 0x01, 0x01, 0x00, 0x80]);
        assert!(DigitalStatus::from_raw(raw).clocks_at_reset());
        // NDAC powered up, nothing else changed.
        raw[5] = 0x81;
        assert!(!DigitalStatus::from_raw(raw).clocks_at_reset());
        // Every entry of the table is in the snapshot.
        for (register, _) in CLOCK_RESET_VALUES {
            assert!(DigitalStatus::REGISTERS.contains(&register));
        }
    }

    /// The same for page 1.
    #[test]
    fn analog_status_decodes_a_playing_chip() {
        let status = AnalogStatus::from_raw(PLAYING_PAGE_1);
        assert_eq!(
            status.supply(),
            Ok(SupplyConfig {
                avdd_weakly_connected_to_dvdd: false,
                avdd_ldo_powered: true,
                avdd_ldo_voltage: LdoVoltage::V1_72,
                analog_blocks_enabled: true,
                common_mode: crate::CommonMode::V0_9,
                headphone_common_mode: HeadphoneCommonMode::V1_65,
                headphone_supply: HeadphoneSupply::LdoinHighRange,
            })
        );
        assert!(!status.offset_correction_done());
        assert!(!status.avdd_ldo_over_current());
        assert_eq!(status.headphone_power(), Channels::BOTH);
        assert_eq!(status.dac_routing(), Channels::BOTH);
        assert_eq!(
            status.headphone_gain(),
            Ok((HeadphoneGain::ZERO_DB, HeadphoneGain::ZERO_DB))
        );
        assert_eq!(status.headphone_mute(), Channels::NONE);
        assert_eq!(
            status.headphone_startup(),
            Ok(HeadphoneStartup {
                soft_routing: SoftRoutingStep::Off,
                slow_charge: SlowCharge::N3,
                rpop: Rpop::K6,
            })
        );
        assert_eq!(status.headphone_gain_applied(), Channels::BOTH);
        assert_eq!(
            status.reference_power_up(),
            ReferencePowerUp {
                forced: false,
                charge_time: ReferenceChargeTime::Ms40,
            }
        );
        assert_eq!(status.register(register::COMMON_MODE), Some(0x33));
        assert_eq!(status.register(register::PLL_J), None);
    }

    /// What a decoder returns for a reserved code in `register`.
    fn reserved<T>(register: Register) -> Result<T, ReservedCode> {
        Err(ReservedCode { register })
    }

    /// [`PLAYING_PAGE_0`] with one register replaced.
    fn digital_with(register: Register, value: u8) -> DigitalStatus {
        let mut raw = PLAYING_PAGE_0;
        raw[position(&DigitalStatus::REGISTERS, register)] = value;
        DigitalStatus::from_raw(raw)
    }

    /// [`PLAYING_PAGE_1`] with one register replaced.
    fn analog_with(register: Register, value: u8) -> AnalogStatus {
        let mut raw = PLAYING_PAGE_1;
        raw[position(&AnalogStatus::REGISTERS, register)] = value;
        AnalogStatus::from_raw(raw)
    }

    /// Every field with reserved codes reports them, names its register, and leaves the other
    /// decoders of the snapshot usable.
    #[test]
    fn reserved_codes_are_reported_with_their_register() {
        // PLL R = 5, J = 3, D = 10000, DOSR = 1 and DOSR = 1023.
        let status = digital_with(register::PLL_P_R, 0xD5);
        assert_eq!(status.clocks(), reserved(register::PLL_P_R));
        assert_eq!(status.interface(), Ok(PLAYING_INTERFACE));
        let status = digital_with(register::PLL_J, 0x03);
        assert_eq!(status.clocks(), reserved(register::PLL_J));
        let mut raw = PLAYING_PAGE_0;
        raw[position(&DigitalStatus::REGISTERS, register::PLL_D_MSB)] = 0x27;
        raw[position(&DigitalStatus::REGISTERS, register::PLL_D_LSB)] = 0x10;
        assert_eq!(
            DigitalStatus::from_raw(raw).clocks(),
            reserved(register::PLL_D_MSB)
        );
        let status = digital_with(register::DOSR_LSB, 0x01);
        assert_eq!(status.clocks(), reserved(register::DOSR_MSB));
        let mut raw = PLAYING_PAGE_0;
        raw[position(&DigitalStatus::REGISTERS, register::DOSR_MSB)] = 0x03;
        raw[position(&DigitalStatus::REGISTERS, register::DOSR_LSB)] = 0xFF;
        assert_eq!(
            DigitalStatus::from_raw(raw).clocks(),
            reserved(register::DOSR_MSB)
        );

        // BCLK divider input 10, processing block 0 and 26, soft-stepping 11, volume 0x80 and
        // +24.5 dB.
        let status = digital_with(register::INTERFACE_3, 0x02);
        assert_eq!(status.interface(), reserved(register::INTERFACE_3));
        let status = digital_with(register::DAC_PROCESSING_BLOCK, 0x00);
        assert_eq!(
            status.processing_block(),
            reserved(register::DAC_PROCESSING_BLOCK)
        );
        let status = digital_with(register::DAC_PROCESSING_BLOCK, 26);
        assert_eq!(
            status.processing_block(),
            reserved(register::DAC_PROCESSING_BLOCK)
        );
        let status = digital_with(register::DAC_SETUP_1, 0xD7);
        assert_eq!(status.dac_setup(), reserved(register::DAC_SETUP_1));
        assert_eq!(status.dac_power(), Channels::BOTH);
        let status = digital_with(register::LEFT_DAC_VOLUME, 0x80);
        assert_eq!(status.volume(), reserved(register::LEFT_DAC_VOLUME));
        let status = digital_with(register::RIGHT_DAC_VOLUME, 0x31);
        assert_eq!(status.volume(), reserved(register::RIGHT_DAC_VOLUME));

        // LDO voltage 11, gains of +30 dB and -7 dB, resistor code 11.
        let status = analog_with(register::LDO_CONTROL, 0x31);
        assert_eq!(status.supply(), reserved(register::LDO_CONTROL));
        let status = analog_with(register::HPL_GAIN, 0b01_1110);
        assert_eq!(status.headphone_gain(), reserved(register::HPL_GAIN));
        let status = analog_with(register::HPR_GAIN, 0b11_1001);
        assert_eq!(status.headphone_gain(), reserved(register::HPR_GAIN));
        let status = analog_with(register::HP_STARTUP, 0x1F);
        assert_eq!(status.headphone_startup(), reserved(register::HP_STARTUP));
        assert_eq!(status.headphone_power(), Channels::BOTH);
    }

    /// The values whose encoding is not the plain number, and the bits that share a register
    /// with something else, decode to what the register descriptions say.
    #[test]
    fn special_encodings_and_shared_registers_decode() {
        // Page 0 after a stream has stopped: the power bits are clear and both DACs muted.
        let mut raw = PLAYING_PAGE_0;
        for (register, value) in [
            (register::PLL_P_R, 0x01),          // P = 8 is the code 0; powered down
            (register::NDAC, 0x00),             // 128 is the code 0; powered down
            (register::DOSR_MSB, 0x00),         // 1024 is the code 0,
            (register::DOSR_LSB, 0x00),         // in both registers
            (register::DAC_FLAGS_1, 0x08),      // only the right DAC still reports powered
            (register::DAC_FLAGS_2, 0x10),      // only the left volume is applied
            (register::MFP2_CONTROL, 0x12),     // the reset value: data output, bus keeper off
            (register::DAC_SETUP_1, 0x16),      // both DACs off
            (register::DAC_SETUP_2, 0x0C),      // both muted
            (register::LEFT_DAC_VOLUME, 0x81),  // -63.5 dB
            (register::RIGHT_DAC_VOLUME, 0x30), // +24 dB
            (register::DRC_CONTROL_1, 0x6F),    // the reset value: both enabled
        ] {
            raw[position(&DigitalStatus::REGISTERS, register)] = value;
        }
        let status = DigitalStatus::from_raw(raw);
        let clocks = status.clocks().unwrap();
        assert!(!clocks.pll.powered);
        assert_eq!(clocks.pll.p.get(), 8);
        assert_eq!(clocks.pll.r.get(), 1);
        assert_eq!(clocks.ndac, DividerConfig::from_bits(0x00));
        assert!(!clocks.ndac.powered);
        assert_eq!(clocks.ndac.divisor.get(), 128);
        assert_eq!(clocks.dosr.get(), 1024);
        assert_eq!(
            status.power_flags(),
            PowerFlags {
                dacs: Channels::RIGHT,
                headphones: Channels::NONE,
            }
        );
        assert_eq!(status.volume_applied(), Channels::LEFT);
        assert_eq!(status.mfp2(), Mfp2Config::RESET);
        assert_eq!(status.dac_power(), Channels::NONE);
        assert_eq!(status.dac_setup(), Ok(PLAYING_DAC));
        assert_eq!(status.dac_mute(), Channels::BOTH);
        assert_eq!(status.volume(), Ok((DacVolume::MIN, DacVolume::MAX)));
        assert_eq!(status.drc_enabled(), Channels::BOTH);

        // Page 1 after a reset, with the two flags of register 2 set and a negative gain.
        let mut raw = [0u8; ANALOG_LEN];
        for (register, value) in [
            (register::LDO_CONTROL, 0x0E), // analog blocks disabled, both read-only flags set
            (register::HPL_GAIN, 0x7A),    // muted, -6 dB
            (register::HPR_GAIN, 0x05),    // unmuted, +5 dB
            (register::HPR_ROUTING, 0x08), // only the right DAC is routed
            (register::HP_GAIN_FLAGS, 0x4C), // HPR applied; the bypass flags are not decoded
            (register::REFERENCE_POWER_UP, 0x07), // forced, 120 ms
        ] {
            raw[position(&AnalogStatus::REGISTERS, register)] = value;
        }
        let status = AnalogStatus::from_raw(raw);
        let supply = status.supply().unwrap();
        assert_eq!(
            supply,
            SupplyConfig {
                analog_blocks_enabled: false,
                ..SupplyConfig::RESET
            }
        );
        assert!(status.offset_correction_done());
        assert!(status.avdd_ldo_over_current());
        assert_eq!(status.headphone_power(), Channels::NONE);
        assert_eq!(status.dac_routing(), Channels::RIGHT);
        assert_eq!(
            status.headphone_gain(),
            Ok((HeadphoneGain::MIN, HeadphoneGain::new(5).unwrap()))
        );
        assert_eq!(status.headphone_mute(), Channels::LEFT);
        assert_eq!(status.headphone_startup(), Ok(HeadphoneStartup::RESET));
        assert_eq!(status.headphone_gain_applied(), Channels::RIGHT);
        assert_eq!(
            status.reference_power_up(),
            ReferencePowerUp {
                forced: true,
                charge_time: crate::ReferenceChargeTime::Ms120,
            }
        );
    }
}

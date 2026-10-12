//! The digital side of the playback path: the processing block, what each DAC channel plays, the
//! digital volume, and the channels' power and mute switches (§2.4).
//!
//! # What the driver does not cover
//!
//! The filter coefficients, the DRC beyond switching it off, the auto-mute and the ganging of the
//! two volume registers (page 0, register 64, bits D6 to D4 and D1 to D0). All of them keep their
//! reset values. The beep generator of processing block PRB_P25 is `crate::beep`, behind the
//! feature `beep`.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::register::{
    self, dac_flags_1, dac_processing_block, dac_setup_1, dac_setup_2, dac_volume, drc_control_1,
};
use crate::{Channels, Error, Tlv320dac3203};

/// How long [`Tlv320dac3203::wait_for_dac_power_down`] waits between two looks at the power
/// flags, in milliseconds. [`DAC_POWER_DOWN_POLL_LIMIT`] explains the number.
pub const DAC_POWER_DOWN_POLL_MS: u32 = 10;

/// How many times [`Tlv320dac3203::wait_for_dac_power_down`] waits before it gives up.
///
/// The guide names no time for a DAC channel's power-down. What it does describe is that the
/// digital volume steps down to -63.5 dB before the channel powers down (§2.4.4.1). The longest
/// that ramp can be is 175 steps of 0.5 dB, from +24 dB (§5.2.59 and §5.2.60), at one step per two
/// sample periods (§5.2.57) and the chip's lowest sample rate of 8 kHz (§2.4): 350 / 8000 s, just
/// under 44 ms. Ten waits of 10 ms are more than twice that. With soft-stepping disabled (as the
/// kernel driver sets it) the first look should already find the channels down; nobody has measured
/// it.
pub const DAC_POWER_DOWN_POLL_LIMIT: u32 = 10;

/// How long [`Tlv320dac3203::wait_for_dac_power_up`] waits between two looks at the power flags,
/// in milliseconds: the interval the power-down wait uses ([`DAC_POWER_DOWN_POLL_MS`]).
pub const DAC_POWER_UP_POLL_MS: u32 = 10;

/// How many times
/// [`Tlv320dac3203::wait_for_dac_power_up`] waits: as for the power-down ([`DAC_POWER_DOWN_POLL_LIMIT`]),
/// 110 ms in all. The guide names no time for a DAC channel's power-up; soft-stepping, when
/// enabled, ramps from -63.5 dB at power-up (§2.4.4.1), at most 350 sample periods at 8 kHz, under
/// 44 ms.
pub const DAC_POWER_UP_POLL_LIMIT: u32 = 10;

/// A DAC processing block, PRB_P1 to PRB_P25: page 0, register 60 (§5.2.54). The block decides
/// which interpolation filter, how many biquads and which extras (first-order IIR, DRC, 3D, beep
/// generator) the channel has; Table 2-13 lists all of them with their resource classes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProcessingBlock(u8);

impl ProcessingBlock {
    /// PRB_P1, the reset value: stereo, interpolation filter A, three biquads, resource class 8
    /// (Table 2-13).
    pub const PRB_P1: Self = Self(dac_processing_block::PRB_P1);
    /// PRB_P2: stereo, interpolation filter A, a first-order IIR, six biquads and the DRC, resource
    /// class 12 (Table 2-13). The block the kernel driver selects (`aic32x4.c:949-950`).
    pub const PRB_P2: Self = Self(dac_processing_block::PRB_P2);
    /// PRB_P25: stereo, interpolation filter A, a first-order IIR, five biquads, the DRC, the
    /// 3D-PGA and the beep generator, resource class 12 (Table 2-13): the block the beep needs
    /// (`crate::beep`, feature `beep`).
    pub const PRB_P25: Self = Self(dac_processing_block::PRB_P25);
    /// PRB_P7: stereo, interpolation filter B, a first-order IIR, no biquads, no DRC, resource
    /// class 6 (Table 2-13). One of the two blocks that fit a 48 kHz tree with MDAC 2 and DOSR 100
    /// (MDAC x DOSR / 32 = 6.25, at least the 6 that resource class 6 needs; §2.4.6).
    pub const PRB_P7: Self = Self(dac_processing_block::PRB_P7);
    /// PRB_P9: stereo, interpolation filter B, four biquads, no IIR, no DRC, resource class 6
    /// (Table 2-13). A block for a 48 kHz tree on the master clock alone, such as MDAC 2 and DOSR
    /// 100: a DOSR that is a multiple of 4 takes filter B (§2.4.6, step 1).
    pub const PRB_P9: Self = Self(dac_processing_block::PRB_P9);

    /// PRB_P`number`. `None` unless `number` is 1 to 25; 0 and 26 to 31 are reserved (§5.2.54).
    pub const fn new(number: u8) -> Option<Self> {
        if number >= 1 && number <= 25 {
            Some(Self(number))
        } else {
            None
        }
    }

    /// The block's number, 1 to 25.
    pub const fn number(self) -> u8 {
        self.0
    }

    /// The register value (§5.2.54). Bits D7 to D5 are reserved and written as 0.
    const fn bits(self) -> u8 {
        self.0 & dac_processing_block::SELECT_MASK
    }

    /// The block a value of register 60 selects, or `None` for the codes 0 and 26 to 31, which are
    /// reserved (§5.2.54).
    pub(crate) const fn from_bits(value: u8) -> Option<Self> {
        Self::new(value & dac_processing_block::SELECT_MASK)
    }
}

// Fails the build if the range drifts from the register description.
const _: () = {
    assert!(ProcessingBlock::new(0).is_none() && ProcessingBlock::new(1).is_some());
    assert!(ProcessingBlock::new(25).is_some() && ProcessingBlock::new(26).is_none());
};

/// The digital volume of a DAC channel in half decibels: -63.5 dB to +24 dB in steps of 0.5 dB
/// (§5.2.59 and §5.2.60; §2.4.4.1).
///
/// The value counts half decibels, so that no fraction is needed: -127 is -63.5 dB, 0 is 0 dB
/// and 48 is +24 dB. This is the volume to change while playing; how it moves to a new value is
/// [`SoftStepping`]. The kernel's `PCM Playback Volume` control has the same limits
/// (`aic32x4.c:98-99` and `:520-521`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct DacVolume(i8);

impl DacVolume {
    /// -63.5 dB, the lowest setting. It is not a mute: [`Tlv320dac3203::set_dac_mute`] is.
    pub const MIN: Self = Self(-127);
    /// 0 dB, the reset value.
    pub const ZERO_DB: Self = Self(0);
    /// +24 dB, the highest setting.
    pub const MAX: Self = Self(48);

    /// A volume of `half_db` half decibels. `None` unless `half_db` is -127 to 48: the code below
    /// that range and all codes above it are reserved (§5.2.59 and §5.2.60).
    pub const fn new(half_db: i8) -> Option<Self> {
        if half_db >= Self::MIN.0 && half_db <= Self::MAX.0 {
            Some(Self(half_db))
        } else {
            None
        }
    }

    /// A volume of `db` whole decibels. `None` unless `db` is -63 to 24.
    pub const fn from_db(db: i8) -> Option<Self> {
        match db.checked_mul(2) {
            Some(half_db) => Self::new(half_db),
            None => None,
        }
    }

    /// The volume in half decibels, -127 to 48.
    pub const fn half_db(self) -> i8 {
        self.0
    }

    /// The register value: the half decibels as an 8-bit two's complement number (§5.2.59 and
    /// §5.2.60).
    const fn bits(self) -> u8 {
        self.0.cast_unsigned()
    }

    /// The volume a value of register 65 or 66 holds: the inverse of `bits`. `None` for
    /// `0x80` and for `0x31` to `0x7F`, which are reserved (§5.2.59 and §5.2.60).
    pub(crate) const fn from_bits(value: u8) -> Option<Self> {
        Self::new(value.cast_signed())
    }
}

// Fails the build if the range or the encoding drifts from the register description.
const _: () = {
    assert!(DacVolume::new(-128).is_none() && DacVolume::new(49).is_none());
    assert!(DacVolume::from_db(-64).is_none() && DacVolume::from_db(25).is_none());
    assert!(DacVolume::from_db(-100).is_none() && DacVolume::from_db(100).is_none());
    assert!(DacVolume::MIN.bits() == dac_volume::MIN_MINUS_63_5_DB);
    assert!(DacVolume::ZERO_DB.bits() == dac_volume::ZERO_DB);
    assert!(DacVolume::MAX.bits() == dac_volume::MAX_PLUS_24_DB);
    assert!(matches!(DacVolume::from_bits(dac_volume::MIN_MINUS_63_5_DB), Some(v) if v.0 == -127));
    assert!(matches!(DacVolume::from_bits(dac_volume::MAX_PLUS_24_DB), Some(v) if v.0 == 48));
    assert!(DacVolume::from_bits(0x80).is_none() && DacVolume::from_bits(0x31).is_none());
};

/// What one DAC channel plays: page 0, register 63, bits D5 to D4 for the left DAC and D3 to D2 for
/// the right one (§5.2.57).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DacData {
    /// Nothing.
    Off,
    /// The left channel of the audio interface. The left DAC's reset value.
    Left,
    /// The right channel of the audio interface. The right DAC's reset value.
    Right,
    /// The mono mix of both channels.
    MonoMix,
}

impl DacData {
    /// The left DAC's field, in place (§5.2.57). The two DACs encode "own side" and "other side"
    /// alike, so left and right swap their codes between the two fields.
    const fn left_dac_bits(self) -> u8 {
        match self {
            DacData::Off => dac_setup_1::LEFT_DATA_OFF,
            DacData::Left => dac_setup_1::LEFT_DATA_LEFT,
            DacData::Right => dac_setup_1::LEFT_DATA_RIGHT,
            DacData::MonoMix => dac_setup_1::LEFT_DATA_MONO_MIX,
        }
    }

    /// The right DAC's field, in place (§5.2.57).
    const fn right_dac_bits(self) -> u8 {
        match self {
            DacData::Off => dac_setup_1::RIGHT_DATA_OFF,
            DacData::Left => dac_setup_1::RIGHT_DATA_LEFT,
            DacData::Right => dac_setup_1::RIGHT_DATA_RIGHT,
            DacData::MonoMix => dac_setup_1::RIGHT_DATA_MONO_MIX,
        }
    }

    /// What the left DAC plays according to a value of register 63: the inverse of `left_dac_bits`.
    /// The two-bit field has no reserved code (§5.2.57).
    const fn from_left_dac_bits(value: u8) -> Self {
        match value & dac_setup_1::LEFT_DATA_MASK {
            dac_setup_1::LEFT_DATA_OFF => DacData::Off,
            dac_setup_1::LEFT_DATA_LEFT => DacData::Left,
            dac_setup_1::LEFT_DATA_RIGHT => DacData::Right,
            _ => DacData::MonoMix,
        }
    }

    /// What the right DAC plays according to a value of register 63: the inverse of
    /// `right_dac_bits` (§5.2.57).
    const fn from_right_dac_bits(value: u8) -> Self {
        match value & dac_setup_1::RIGHT_DATA_MASK {
            dac_setup_1::RIGHT_DATA_OFF => DacData::Off,
            dac_setup_1::RIGHT_DATA_RIGHT => DacData::Right,
            dac_setup_1::RIGHT_DATA_LEFT => DacData::Left,
            _ => DacData::MonoMix,
        }
    }
}

/// How the digital volume moves to a new value: page 0, register 63, bits D1 to D0 (§5.2.57;
/// §2.4.4.1). The fourth code is reserved.
///
/// Soft-stepping also runs when a channel powers up (from -63.5 dB to the programmed volume) and
/// when it powers down (§2.4.4.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoftStepping {
    /// One 0.5 dB step per sample period, the reset value.
    PerSample,
    /// One 0.5 dB step per two sample periods.
    PerTwoSamples,
    /// No soft-stepping: a volume change applies at once. The kernel driver's default
    /// (`aic32x4.h:161`, `aic32x4.c:123`).
    Disabled,
}

impl SoftStepping {
    /// The field's bits, in place (§5.2.57).
    const fn bits(self) -> u8 {
        match self {
            SoftStepping::PerSample => dac_setup_1::SOFT_STEP_PER_SAMPLE,
            SoftStepping::PerTwoSamples => dac_setup_1::SOFT_STEP_PER_TWO_SAMPLES,
            SoftStepping::Disabled => dac_setup_1::SOFT_STEP_DISABLED,
        }
    }

    /// The soft-stepping a value of register 63 selects, or `None` for the code 11, which is
    /// reserved (§5.2.57).
    const fn from_bits(value: u8) -> Option<Self> {
        match value & dac_setup_1::SOFT_STEP_MASK {
            dac_setup_1::SOFT_STEP_PER_SAMPLE => Some(SoftStepping::PerSample),
            dac_setup_1::SOFT_STEP_PER_TWO_SAMPLES => Some(SoftStepping::PerTwoSamples),
            dac_setup_1::SOFT_STEP_DISABLED => Some(SoftStepping::Disabled),
            _ => None,
        }
    }
}

/// The part of page 0, register 63 that is configuration and not power: what the two DACs play and
/// how the volume steps (§5.2.57).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DacSetup {
    /// What the left DAC plays.
    pub left: DacData,
    /// What the right DAC plays.
    pub right: DacData,
    /// How the digital volume moves.
    pub soft_stepping: SoftStepping,
}

impl DacSetup {
    /// The setup after a reset: each DAC plays its own side, one volume step per sample period
    /// (§5.2.57).
    pub const RESET: Self = Self {
        left: DacData::Left,
        right: DacData::Right,
        soft_stepping: SoftStepping::PerSample,
    };

    /// Bits D5 to D0 of register 63 (§5.2.57).
    const fn bits(self) -> u8 {
        self.left.left_dac_bits() | self.right.right_dac_bits() | self.soft_stepping.bits()
    }

    /// The setup in bits D5 to D0 of a value of register 63: the inverse of
    /// `bits`. The two power bits are not part of it. `None` for the reserved soft-stepping code
    /// (§5.2.57).
    pub(crate) const fn from_bits(value: u8) -> Option<Self> {
        match SoftStepping::from_bits(value) {
            Some(soft_stepping) => Some(Self {
                left: DacData::from_left_dac_bits(value),
                right: DacData::from_right_dac_bits(value),
                soft_stepping,
            }),
            None => None,
        }
    }
}

/// The bits of register 63 that [`DacSetup`] covers: both data fields and the soft-step field.
/// The kernel driver updates the same six bits in one go (`AIC32X4_DAC_CHAN_MASK` and
/// `AIC32X4_DAC_SOFT_STEPPING_MASK`, `aic32x4.h:159` and `:162`, `aic32x4.c:971-973`).
const DAC_SETUP_MASK: u8 =
    dac_setup_1::LEFT_DATA_MASK | dac_setup_1::RIGHT_DATA_MASK | dac_setup_1::SOFT_STEP_MASK;

/// The value
/// [`Tlv320dac3203::disable_drc`] writes to DRC control 1: both enables cleared, the threshold code 3 (-12 dBFS) and the hysteresis code 3 (3 dB) at their reset values (§5.2.62). It comes to `0x0F`,
/// the value the kernel driver writes (`aic32x4.c:57-58`).
const DRC_DISABLED: u8 = (3 << drc_control_1::THRESHOLD_SHIFT) | drc_control_1::HYSTERESIS_MASK;

const _: () = assert!(DRC_DISABLED == 0x0F);

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Selects the DAC processing block: page 0, register 60, written whole (§5.2.54).
    ///
    /// Select it while the DAC channels are powered down, as the guide's setup list does (§2.4.6,
    /// step 2). `MDAC * DOSR / 32` has to be at least the block's resource class (§2.4.6).
    pub fn set_processing_block(
        &mut self,
        block: ProcessingBlock,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(register::DAC_PROCESSING_BLOCK, block.bits())
    }

    /// Switches the dynamic range compression off on both channels: page 0, register 68, written
    /// whole as `0x0F` (§5.2.62).
    ///
    /// The DRC is *enabled* after a reset and takes effect in the processing blocks that contain
    /// it, PRB_P2 among them (Table 2-13). The kernel driver switches it off with this value
    /// whenever a stream is set up (`aic32x4.c:55-59` and `:953-959`). The threshold and the
    /// hysteresis are written at their reset values, which is all this driver offers: tuning the
    /// DRC needs its filter coefficients as well, and those are out of scope ([`crate::register`],
    /// "What is left out on purpose").
    pub fn disable_drc(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write(register::DRC_CONTROL_1, DRC_DISABLED)
    }

    /// Sets what the two DACs play and how the digital volume steps: bits D5 to D0 of page 0,
    /// register 63 (§5.2.57). The two power bits in the same register keep their state.
    ///
    /// It reads the register and writes it back with the six bits changed, so it needs a chip
    /// this driver owns.
    pub fn configure_dac(&mut self, setup: DacSetup) -> Result<(), Error<I2C::Error>> {
        self.modify(register::DAC_SETUP_1, DAC_SETUP_MASK, setup.bits())
    }

    /// Powers the DAC channels named in `powered` up and the others down: bits D7 and D6 of page 0,
    /// register 63 (§5.2.57). The rest of the register keeps its state.
    ///
    /// A channel needs its clocks to power up: NDAC and MDAC have to be powered (§2.7). On the way
    /// down the chip runs a shutdown sequence of its own, during which those dividers must stay
    /// powered; [`wait_for_dac_power_down`](Self::wait_for_dac_power_down) waits for its end.
    ///
    /// It reads the register and writes it back with the two bits changed, so it needs a chip
    /// this driver owns.
    pub fn set_dac_power(&mut self, powered: Channels) -> Result<(), Error<I2C::Error>> {
        let mut bits = 0;
        if powered.left {
            bits |= dac_setup_1::LEFT_POWER_UP;
        }
        if powered.right {
            bits |= dac_setup_1::RIGHT_POWER_UP;
        }
        self.modify(
            register::DAC_SETUP_1,
            dac_setup_1::LEFT_POWER_UP | dac_setup_1::RIGHT_POWER_UP,
            bits,
        )
    }

    /// Mutes the DAC channels named in `muted` and unmutes the others: bits D3 and D2 of page 0,
    /// register 64 (§5.2.58). Both channels are muted after a reset. The auto-mute and
    /// volume-ganging fields in the same register keep their state.
    ///
    /// After a headphone driver has been powered up, unmute only once its gain flag is set
    /// ([`wait_for_headphone_gain`](Self::wait_for_headphone_gain); §2.2.2.1, step 7).
    ///
    /// It reads the register and writes it back with the two bits changed, so it needs a chip
    /// this driver owns.
    pub fn set_dac_mute(&mut self, muted: Channels) -> Result<(), Error<I2C::Error>> {
        let mut bits = 0;
        if muted.left {
            bits |= dac_setup_2::LEFT_MUTE;
        }
        if muted.right {
            bits |= dac_setup_2::RIGHT_MUTE;
        }
        self.modify(
            register::DAC_SETUP_2,
            dac_setup_2::LEFT_MUTE | dac_setup_2::RIGHT_MUTE,
            bits,
        )
    }

    /// Sets the digital volume of the two DAC channels: page 0, registers 65 (left) and 66 (right),
    /// each written whole (§5.2.59 and §5.2.60). Both are 0 dB after a reset, and the kernel driver
    /// never writes them by itself.
    ///
    /// On a powered channel the volume moves to the new value as [`SoftStepping`] says; with
    /// soft-stepping disabled it jumps. It can be set at any time, before or after
    /// [`power_up`](Self::power_up).
    pub fn set_volume(
        &mut self,
        left: DacVolume,
        right: DacVolume,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(register::LEFT_DAC_VOLUME, left.bits())?;
        self.write(register::RIGHT_DAC_VOLUME, right.bits())
    }

    /// Waits until both DAC channels report powered down in page 0, register 37 (§5.2.36), looking
    /// every [`DAC_POWER_DOWN_POLL_MS`] and at most [`DAC_POWER_DOWN_POLL_LIMIT`] + 1 times. Fails
    /// with [`Error::Timeout`] if a channel is still up then.
    ///
    /// This is the condition for switching the clocks off: "When both flags indicate power-down,
    /// the MDAC divider may be powered down, followed by the NDAC divider" (§2.7). The shutdown
    /// sequence it waits for needs those clocks, so a chip whose master clock has stopped will not
    /// get there.
    pub fn wait_for_dac_power_down<D: DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        self.wait_for(
            register::DAC_FLAGS_1,
            dac_flags_1::LEFT_DAC_POWERED | dac_flags_1::RIGHT_DAC_POWERED,
            0,
            DAC_POWER_DOWN_POLL_MS,
            DAC_POWER_DOWN_POLL_LIMIT,
            delay,
        )
    }

    /// Waits until the DAC channels named in
    /// `channels` report powered up in register 37 (§5.2.36), looking every [`DAC_POWER_UP_POLL_MS`]
    /// and at most [`DAC_POWER_UP_POLL_LIMIT`] + 1 times. With no channel named it returns at once.
    ///
    /// A DAC channel powers up only with its clocks running (§2.7), so this is the first flag that
    /// proves the master clock reaches the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] on register 37 if a channel is still down then.
    pub fn wait_for_dac_power_up<D: DelayNs>(
        &mut self,
        channels: Channels,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        let mut flags = 0;
        if channels.left {
            flags |= dac_flags_1::LEFT_DAC_POWERED;
        }
        if channels.right {
            flags |= dac_flags_1::RIGHT_DAC_POWERED;
        }
        if flags == 0 {
            return Ok(());
        }
        self.wait_for(
            register::DAC_FLAGS_1,
            flags,
            flags,
            DAC_POWER_UP_POLL_MS,
            DAC_POWER_UP_POLL_LIMIT,
            delay,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;
    use crate::test_support::{BusDelay, wait_ms};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins what goes over the bus when the power flags never clear: eleven reads with ten waits
    /// of 10 ms between them, then the timeout, and no write beyond the page select.
    #[test]
    fn wait_for_dac_power_down_gives_up_after_the_bounded_wait() {
        let select = || Transaction::write(ADDRESS, vec![0, 0]);
        let mut expectations = vec![];
        for _ in 0..DAC_POWER_DOWN_POLL_LIMIT {
            // The right DAC still reports powered up.
            expectations.push(select());
            expectations.push(Transaction::write_read(ADDRESS, vec![37], vec![0x08]));
            expectations.push(wait_ms(10));
        }
        expectations.push(select());
        expectations.push(Transaction::write_read(ADDRESS, vec![37], vec![0x08]));

        let mut bus = Mock::new(&expectations);
        let mut delay = BusDelay(bus.clone());
        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        assert_eq!(
            dac.wait_for_dac_power_down(&mut delay),
            Err(Error::Timeout(register::DAC_FLAGS_1))
        );
        bus.done();
    }

    /// Pins the two whole-register writes: the processing block and the DRC switch.
    #[test]
    fn processing_block_and_drc_write_registers_60_and_68() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![60, 25]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![68, 0x0F]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_processing_block(ProcessingBlock::new(25).unwrap())
            .unwrap();
        dac.disable_drc().unwrap();
        dac.release().done();
    }

    /// Pins the two volume registers, written whole, with the ends of the range and a value in
    /// whole decibels: -63.5 dB is 0x81, +24 dB is 0x30, -10 dB is 0xEC.
    #[test]
    fn set_volume_writes_registers_65_and_66() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![65, 0x81]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![66, 0x30]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![65, 0xEC]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![66, 0xEC]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_volume(DacVolume::MIN, DacVolume::MAX).unwrap();
        let quiet = DacVolume::from_db(-10).unwrap();
        dac.set_volume(quiet, quiet).unwrap();
        dac.release().done();
    }

    /// Pins the three read-modify-writes of the DAC channel registers: the setup changes bits 5
    /// to 0 of register 63, the power switch bits 7 and 6, the mute bits 3 and 2 of register 64.
    #[test]
    fn dac_setup_power_and_mute_change_only_their_own_bits() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            // Both channels powered, reset data paths: swap the channels, step per two samples.
            Transaction::write_read(ADDRESS, vec![63], vec![0xD4]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![63, 0xE9]),
            // Left DAC off and playing nothing, right DAC playing the mono mix, no soft-stepping.
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write_read(ADDRESS, vec![63], vec![0xE9]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![63, 0xCE]),
            // Power: only the right channel stays up.
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write_read(ADDRESS, vec![63], vec![0xCE]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![63, 0x4E]),
            // Mute: auto-mute code 1 and ganging code 1 stay, the left channel is muted alone.
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write_read(ADDRESS, vec![64], vec![0x1D]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![64, 0x19]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.configure_dac(DacSetup {
            left: DacData::Right,
            right: DacData::Left,
            soft_stepping: SoftStepping::PerTwoSamples,
        })
        .unwrap();
        dac.configure_dac(DacSetup {
            left: DacData::Off,
            right: DacData::MonoMix,
            soft_stepping: SoftStepping::Disabled,
        })
        .unwrap();
        dac.set_dac_power(Channels::RIGHT).unwrap();
        dac.set_dac_mute(Channels::LEFT).unwrap();
        dac.release().done();
    }

    /// The power-up wait: eleven reads of register 37 with ten waits of 10 ms, then the timeout.
    #[test]
    fn wait_for_dac_power_up_gives_up_after_the_bounded_wait() {
        let read =
            |register: u8, value: u8| Transaction::write_read(ADDRESS, vec![register], vec![value]);
        let select = || Transaction::write(ADDRESS, vec![0, 0]);
        let mut expectations = vec![];
        for _ in 0..DAC_POWER_UP_POLL_LIMIT {
            expectations.push(select());
            expectations.push(read(37, 0x80));
            expectations.push(wait_ms(10));
        }
        expectations.push(select());
        expectations.push(read(37, 0x80));
        let mut bus = Mock::new(&expectations);
        let mut delay = BusDelay(bus.clone());
        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        assert_eq!(
            dac.wait_for_dac_power_up(Channels::BOTH, &mut delay),
            Err(Error::Timeout(register::DAC_FLAGS_1))
        );
        assert_eq!(
            dac.wait_for_dac_power_up(Channels::NONE, &mut delay),
            Ok(())
        );
        bus.done();
    }
}

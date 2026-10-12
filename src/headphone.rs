//! The two headphone drivers, HPL and HPR: what is routed to them, their gain, their pop-free
//! start-up, their power and mute switches, and the flags that say when a driver has settled
//! (§2.2.2).
//!
//! # What the driver does not cover
//!
//! The analog bypass from the IN1L and IN1R inputs to the drivers (its routing bits in page 1,
//! registers 12 and 13, and its volumes in registers 22 and 23), the mono differential output
//! (register 13, bit D4), the class-D mode and the PowerTune modes (registers 3 and 4) and the
//! over-current protection settings (register 11). These are gaps of this driver: the registers
//! keep their reset values.
//!
//! What the two outputs drive (a jack, or a mux and a separate speaker amplifier) is the board's;
//! neither a mux nor an amplifier is part of this chip.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::register::{
    self, hp_gain, hp_gain_flags, hp_startup, hpl_routing, hpr_routing, output_driver_power,
};
use crate::{Channels, Error, Tlv320dac3203};

/// How long
/// [`Tlv320dac3203::wait_for_headphone_gain`] waits between two looks at the gain flags, in milliseconds: the kernel driver's interval (`mdelay(100)`,
/// `aic32x4.c:361`). The guide only says to "poll Page 1, Register 63" (§2.4.6, step 2) and gives
/// no interval.
pub const HEADPHONE_GAIN_POLL_MS: u32 = 100;

/// How many times [`Tlv320dac3203::wait_for_headphone_gain`] waits before it gives up: the
/// kernel driver's limit of 22 (`aic32x4.c:357`), which bounds the wait at 2.2 s.
///
/// The kernel's comment says the gain "can take upto 2sec" to apply
/// (`aic32x4.c:353-355`). With the kernel's start-up setting (`aic32x4.h:177`) the slow charge
/// alone is 3 time constants of 6 kOhm and the 47 uF the guide assumes, 0.85 s (§2.2.2.1; computed,
/// not measured). A start-up setting with a longer slow charge needs a longer wait than this one:
/// six time constants of 6 kOhm already come to 1.7 s, for which TI's example waits 2.5 s (§4.4).
pub const HEADPHONE_GAIN_POLL_LIMIT: u32 = 22;

/// The gain of a headphone driver in whole decibels: -6 dB to +29 dB in steps of 1 dB (§5.3.13 and
/// §5.3.14; §2.2.2).
///
/// This is an output level to choose once for the board: the guide says these controls "are not
/// meant to be used as dynamic volume control, but more to set output levels during initial device
/// configuration" (§2.2.2). The volume to change while playing is [`DacVolume`](crate::DacVolume).
///
/// The kernel's `HP Driver Gain Volume` control counts the same steps from 0, so its value `v`
/// is `v - 6` dB here (`aic32x4.c:100-101` and `:522-524`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct HeadphoneGain(i8);

impl HeadphoneGain {
    /// -6 dB, the lowest setting. A driver cannot be muted from it (§2.2.2 footnote).
    pub const MIN: Self = Self(-6);
    /// 0 dB, the reset value.
    pub const ZERO_DB: Self = Self(0);
    /// +29 dB, the highest setting.
    pub const MAX: Self = Self(29);

    /// `None` unless `db` is -6 to 29; every other code of the field is reserved (§5.3.13 and
    /// §5.3.14).
    pub const fn new(db: i8) -> Option<Self> {
        if db >= Self::MIN.0 && db <= Self::MAX.0 {
            Some(Self(db))
        } else {
            None
        }
    }

    /// The gain in decibels, -6 to 29.
    pub const fn db(self) -> i8 {
        self.0
    }

    /// The field's bits: the gain as a 6-bit two's complement number (§5.3.13 and §5.3.14).
    const fn bits(self) -> u8 {
        self.0.cast_unsigned() & hp_gain::GAIN_MASK
    }

    /// The gain in bits D5 to D0 of a value of register 16 or 17: the inverse of
    /// `bits`. The field is a 6-bit two's complement number, so its bit D5 is the sign. `None` for
    /// the codes below -6 dB and above +29 dB, which are reserved (§5.3.13 and §5.3.14).
    pub(crate) const fn from_bits(value: u8) -> Option<Self> {
        let code = value & hp_gain::GAIN_MASK;
        // Bit D5 set means a negative number: fill the two bits above the field with ones, which
        // makes the same number in eight bits.
        let extended = if code & GAIN_SIGN_BIT != 0 {
            code | !hp_gain::GAIN_MASK
        } else {
            code
        };
        Self::new(extended.cast_signed())
    }
}

/// The sign bit of the 6-bit two's complement gain field: bit D5, the field's highest (§5.3.13 and
/// §5.3.14).
const GAIN_SIGN_BIT: u8 = 0b0010_0000;

// Fails the build if the range or the encoding drifts from the register description.
const _: () = {
    assert!(HeadphoneGain::new(-7).is_none() && HeadphoneGain::new(30).is_none());
    assert!(HeadphoneGain::MIN.bits() == hp_gain::GAIN_MIN_MINUS_6_DB);
    assert!(HeadphoneGain::ZERO_DB.bits() == hp_gain::GAIN_ZERO_DB);
    assert!(HeadphoneGain::MAX.bits() == hp_gain::GAIN_MAX_PLUS_29_DB);
    // The decoder has to undo the encoding, above all for the negative gains, and must ignore
    // the mute bit that shares the register.
    assert!(matches!(HeadphoneGain::from_bits(hp_gain::GAIN_MIN_MINUS_6_DB), Some(g) if g.0 == -6));
    assert!(matches!(
        HeadphoneGain::from_bits(hp_gain::MUTE | hp_gain::GAIN_MAX_PLUS_29_DB),
        Some(g) if g.0 == 29
    ));
    assert!(HeadphoneGain::from_bits(0b01_1110).is_none());
    assert!(HeadphoneGain::from_bits(0b11_1001).is_none());
};

/// The step time of the soft routing, the gradual release of an attenuation when a signal is first
/// routed to a driver: page 1, register 20, bits D7 to D6 (§5.3.16; §2.2.2.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoftRoutingStep {
    /// 0 ms: soft routing is off. The reset value.
    Off,
    /// 50 ms per step.
    Ms50,
    /// 100 ms per step.
    Ms100,
    /// 200 ms per step.
    Ms200,
}

impl SoftRoutingStep {
    /// The field's bits, in place (§5.3.16).
    const fn bits(self) -> u8 {
        match self {
            SoftRoutingStep::Off => hp_startup::SOFT_ROUTING_OFF,
            SoftRoutingStep::Ms50 => hp_startup::SOFT_ROUTING_50_MS,
            SoftRoutingStep::Ms100 => hp_startup::SOFT_ROUTING_100_MS,
            SoftRoutingStep::Ms200 => hp_startup::SOFT_ROUTING_200_MS,
        }
    }

    /// The step time a value of register 20 selects: the inverse of `bits`. The two-bit field has
    /// no reserved code (§5.3.16).
    const fn from_bits(value: u8) -> Self {
        match value & hp_startup::SOFT_ROUTING_MASK {
            hp_startup::SOFT_ROUTING_OFF => SoftRoutingStep::Off,
            hp_startup::SOFT_ROUTING_50_MS => SoftRoutingStep::Ms50,
            hp_startup::SOFT_ROUTING_100_MS => SoftRoutingStep::Ms100,
            _ => SoftRoutingStep::Ms200,
        }
    }
}

/// How long a driver charges its output capacitor slowly at power-up, as a number N of time
/// constants of [`Rpop`] and an assumed 47 uF capacitor: page 1, register 20, bits D5 to D2
/// (§5.3.16; the table in §2.2.2.1).
///
/// No signal reaches the output during the slow charge (§2.2.2.1). The guide expects N = 5 to give
/// good results with 47 uF (p. 17).
///
/// The variants are in the order of their codes, 0 to 15.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlowCharge {
    /// No slow charge: the driver powers up at once. The reset value.
    Off,
    /// N = 0.5.
    N0_5,
    /// N = 0.625.
    N0_625,
    /// N = 0.75. (The register description prints 0.725; the table on p. 17 has 0.75.)
    N0_75,
    /// N = 0.875.
    N0_875,
    /// N = 1.
    N1,
    /// N = 2.
    N2,
    /// N = 3. What the kernel driver uses (`aic32x4.h:177`).
    N3,
    /// N = 4.
    N4,
    /// N = 5.
    N5,
    /// N = 6.
    N6,
    /// N = 7.
    N7,
    /// N = 8.
    N8,
    /// N = 16. Not valid with the 25 kOhm resistor.
    N16,
    /// N = 24. Not valid with the 25 kOhm resistor.
    N24,
    /// N = 32. Not valid with the 25 kOhm resistor.
    N32,
}

impl SlowCharge {
    /// The field's bits, in place: the variant's position is its code (§5.3.16).
    const fn bits(self) -> u8 {
        ((self as u8) << hp_startup::SLOW_CHARGE_SHIFT) & hp_startup::SLOW_CHARGE_MASK
    }

    /// The slow charge a value of register 20 selects: the inverse of `bits`. All sixteen codes of
    /// the field have a meaning (§5.3.16).
    const fn from_bits(value: u8) -> Self {
        match (value & hp_startup::SLOW_CHARGE_MASK) >> hp_startup::SLOW_CHARGE_SHIFT {
            0 => SlowCharge::Off,
            1 => SlowCharge::N0_5,
            2 => SlowCharge::N0_625,
            3 => SlowCharge::N0_75,
            4 => SlowCharge::N0_875,
            5 => SlowCharge::N1,
            6 => SlowCharge::N2,
            7 => SlowCharge::N3,
            8 => SlowCharge::N4,
            9 => SlowCharge::N5,
            10 => SlowCharge::N6,
            11 => SlowCharge::N7,
            12 => SlowCharge::N8,
            13 => SlowCharge::N16,
            14 => SlowCharge::N24,
            _ => SlowCharge::N32,
        }
    }
}

// The cast above relies on the declaration order. These two pin its ends.
const _: () = {
    assert!(SlowCharge::Off as u8 == 0);
    assert!(SlowCharge::N3 as u8 == 0b0111);
    assert!(SlowCharge::N32 as u8 == 0b1111);
};

// The decoder lists the codes by hand. This walks all sixteen and fails the build if one of them
// does not come back as the variant that encodes to it.
const _: () = {
    let mut code = 0u8;
    while code < 16 {
        let bits = code << hp_startup::SLOW_CHARGE_SHIFT;
        assert!(SlowCharge::from_bits(bits).bits() == bits);
        code += 1;
    }
};

/// The resistor the slow charge goes through, Rpop: page 1, register 20, bits D1 to D0 (§5.3.16).
/// The fourth code is reserved.
///
/// Two variants are named after their bits and not after a resistance, because the guide
/// contradicts itself about them: the register description has `00` = 25 kOhm and `10` = 2 kOhm
/// (§5.3.16), and the table in the text has `00` = 2 kOhm and `10` = 25 kOhm (Table 2-3). Both
/// places agree on `01`. Nobody has measured which of the two is right.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rpop {
    /// `00`, the reset value: 25 kOhm or 2 kOhm. See the type's description.
    Bits00,
    /// `01`: 6 kOhm. What the kernel driver uses (`aic32x4.h:177-178`).
    K6,
    /// `10`: 2 kOhm or 25 kOhm. See the type's description.
    Bits10,
}

impl Rpop {
    /// The field's bits, in place (§5.3.16).
    const fn bits(self) -> u8 {
        match self {
            Rpop::Bits00 => 0b00,
            Rpop::K6 => hp_startup::RPOP_6K,
            Rpop::Bits10 => 0b10,
        }
    }

    /// The resistor code a value of register 20 holds, or `None` for the code 11, which is reserved
    /// (§5.3.16).
    const fn from_bits(value: u8) -> Option<Self> {
        match value & hp_startup::RPOP_MASK {
            0b00 => Some(Rpop::Bits00),
            hp_startup::RPOP_6K => Some(Rpop::K6),
            0b10 => Some(Rpop::Bits10),
            _ => None,
        }
    }
}

/// The headphone drivers' start-up behaviour, the chip's de-pop settings: page 1, register 20
/// (§5.3.16).
///
/// These settings have to be written before a driver is powered up and must not change while one is
/// up (§2.2.2.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HeadphoneStartup {
    /// The soft routing step time.
    pub soft_routing: SoftRoutingStep,
    /// How long the slow charge lasts.
    pub slow_charge: SlowCharge,
    /// The resistor the slow charge goes through.
    pub rpop: Rpop,
}

impl HeadphoneStartup {
    /// The start-up behaviour after a reset: no soft routing, no slow charge (§5.3.16).
    pub const RESET: Self = Self {
        soft_routing: SoftRoutingStep::Off,
        slow_charge: SlowCharge::Off,
        rpop: Rpop::Bits00,
    };

    /// The register value (§5.3.16).
    const fn bits(self) -> u8 {
        self.soft_routing.bits() | self.slow_charge.bits() | self.rpop.bits()
    }

    /// The start-up behaviour a value of register 20 describes: the inverse of `bits`. `None` for
    /// the reserved resistor code (§5.3.16).
    pub(crate) const fn from_bits(value: u8) -> Option<Self> {
        match Rpop::from_bits(value) {
            Some(rpop) => Some(Self {
                soft_routing: SoftRoutingStep::from_bits(value),
                slow_charge: SlowCharge::from_bits(value),
                rpop,
            }),
            None => None,
        }
    }
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Routes each DAC to the headphone driver of its side for the channels named in `routed`, and
    /// nothing to the drivers of the others: page 1, registers 12 and 13, each written whole
    /// (§5.3.10 and §5.3.11). Left is the left DAC to HPL, right is the right DAC to HPR; nothing
    /// is routed after a reset.
    ///
    /// These are the kernel's two mixer switches, `HPL Output Mixer L_DAC Switch` and `HPR Output
    /// Mixer R_DAC Switch` (`aic32x4.c:618-626`). The registers' other routing bits, the analog
    /// bypass and the left DAC's negative terminal for a mono differential output, are written as
    /// 0: this driver does not offer them.
    ///
    /// Route before the DACs and the drivers are powered, which is where the guide's pop-free
    /// sequence has it (§2.2.2.1, step 3); the soft routing of [`HeadphoneStartup`] only acts when
    /// a signal is first routed to a driver.
    pub fn set_dac_routing(&mut self, routed: Channels) -> Result<(), Error<I2C::Error>> {
        let hpl = if routed.left {
            hpl_routing::LEFT_DAC
        } else {
            0
        };
        let hpr = if routed.right {
            hpr_routing::RIGHT_DAC
        } else {
            0
        };
        self.write(register::HPL_ROUTING, hpl)?;
        self.write(register::HPR_ROUTING, hpr)
    }

    /// Sets the gain of the two headphone drivers: bits D5 to D0 of page 1, registers 16 (HPL) and
    /// 17 (HPR) (§5.3.13 and §5.3.14). The mute bit in the same registers keeps its state.
    ///
    /// A powered driver moves to a new gain in steps and reports the end of that in page 1,
    /// register 63 ([`wait_for_headphone_gain`](Self::wait_for_headphone_gain)). The gains are not
    /// valid while the drivers run in class-D mode (§5.3.13 note), which this driver never selects.
    ///
    /// It reads each register and writes it back with the gain changed, so it needs a chip this
    /// driver owns. Both registers are always written, HPL first.
    pub fn set_headphone_gain(
        &mut self,
        hpl: HeadphoneGain,
        hpr: HeadphoneGain,
    ) -> Result<(), Error<I2C::Error>> {
        self.modify(register::HPL_GAIN, hp_gain::GAIN_MASK, hpl.bits())?;
        self.modify(register::HPR_GAIN, hp_gain::GAIN_MASK, hpr.bits())
    }

    /// Sets the headphone drivers' start-up behaviour: page 1, register 20, written whole
    /// (§5.3.16).
    ///
    /// Call it only while both drivers are powered down: the register "should not be changed when
    /// the headphone drivers are powered up" (§2.2.2.1).
    pub fn set_headphone_startup(
        &mut self,
        startup: HeadphoneStartup,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(register::HP_STARTUP, startup.bits())
    }

    /// Powers the headphone drivers named in `powered` up and the others down: page 1, register 9,
    /// written whole (§5.3.7). The register's other bits are reserved and written as 0, their reset
    /// value.
    ///
    /// Powering up starts the slow charge set with
    /// [`set_headphone_startup`](Self::set_headphone_startup); [`wait_for_headphone_gain`](Self::wait_for_headphone_gain)
    /// tells when the driver has settled. A driver should only be powered down again once it has
    /// settled (§2.2.2.1).
    pub fn set_headphone_power(&mut self, powered: Channels) -> Result<(), Error<I2C::Error>> {
        let mut bits = 0;
        if powered.left {
            bits |= output_driver_power::HPL_POWER_UP;
        }
        if powered.right {
            bits |= output_driver_power::HPR_POWER_UP;
        }
        self.write(register::OUTPUT_DRIVER_POWER, bits)
    }

    /// Mutes the headphone drivers named in `muted` and unmutes the others: bit D6 of page 1,
    /// registers 16 and 17 (§5.3.13 and §5.3.14). Both drivers are muted after a reset. The gain in
    /// the same registers keeps its value.
    ///
    /// Two rules from the guide are the caller's to keep. While a driver is powered, mute or unmute
    /// it only once its gain has settled
    /// ([`wait_for_headphone_gain`](Self::wait_for_headphone_gain); §2.2.2.1). And a driver cannot
    /// be muted from the -6 dB gain setting: set -5 dB first (§2.2.2 footnote).
    ///
    /// It reads each register and writes it back with the mute bit changed, so it needs a chip
    /// this driver owns. Both registers are always written, HPL first.
    pub fn set_headphone_mute(&mut self, muted: Channels) -> Result<(), Error<I2C::Error>> {
        let left = if muted.left { hp_gain::MUTE } else { 0 };
        let right = if muted.right { hp_gain::MUTE } else { 0 };
        self.modify(register::HPL_GAIN, hp_gain::MUTE, left)?;
        self.modify(register::HPR_GAIN, hp_gain::MUTE, right)
    }

    /// Waits until the drivers named in
    /// `drivers` report that their applied gain equals the programmed one, in page 1, register 63 (§5.3.25), looking every [`HEADPHONE_GAIN_POLL_MS`]
    /// and at most [`HEADPHONE_GAIN_POLL_LIMIT`] + 1 times. Fails with [`Error::Timeout`] if a flag
    /// is still missing then. With no driver named it returns at once and reads nothing.
    ///
    /// After a driver power-up the guide takes these flags "to indicate completion of soft-stepping
    /// after power-up" and makes them the condition for unmuting the DAC (§2.2.2.1, step 7). The
    /// kernel driver polls the same register before it unmutes, and unmutes when its counter runs
    /// out whether the flags came or not (`aic32x4.c:352-367`); this method reports that case
    /// instead and leaves the decision to the caller.
    pub fn wait_for_headphone_gain<D: DelayNs>(
        &mut self,
        drivers: Channels,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        let flags = gain_flags(drivers);
        if flags == 0 {
            return Ok(());
        }
        self.wait_for(
            register::HP_GAIN_FLAGS,
            flags,
            flags,
            HEADPHONE_GAIN_POLL_MS,
            HEADPHONE_GAIN_POLL_LIMIT,
            delay,
        )
    }
}

/// The gain-applied bits of page 1, register 63 for the drivers named in `drivers` (§5.3.25): HPL's
/// D7, HPR's D6.
pub(crate) const fn gain_flags(drivers: Channels) -> u8 {
    let mut flags = 0;
    if drivers.left {
        flags |= hp_gain_flags::HPL_GAIN_APPLIED;
    }
    if drivers.right {
        flags |= hp_gain_flags::HPR_GAIN_APPLIED;
    }
    flags
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins the start-up register with the two values the kernel driver knows (`0x1D` and
    /// `0x01`, `aic32x4.h:177-178`) and one that uses the remaining fields.
    #[test]
    fn set_headphone_startup_writes_register_20_of_page_1() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![20, 0x1D]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![20, 0x01]),
            // 200 ms soft routing steps, N = 32, Rpop code 10.
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![20, 0xFE]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_headphone_startup(HeadphoneStartup {
            soft_routing: SoftRoutingStep::Off,
            slow_charge: SlowCharge::N3,
            rpop: Rpop::K6,
        })
        .unwrap();
        dac.set_headphone_startup(HeadphoneStartup {
            soft_routing: SoftRoutingStep::Off,
            slow_charge: SlowCharge::Off,
            rpop: Rpop::K6,
        })
        .unwrap();
        dac.set_headphone_startup(HeadphoneStartup {
            soft_routing: SoftRoutingStep::Ms200,
            slow_charge: SlowCharge::N32,
            rpop: Rpop::Bits10,
        })
        .unwrap();
        dac.release().done();
    }

    /// Pins the two routing registers, written whole: both DACs routed, then the right one alone.
    #[test]
    fn set_dac_routing_writes_registers_12_and_13_of_page_1() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![12, 0x08]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![13, 0x08]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![12, 0x00]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![13, 0x08]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_dac_routing(Channels::BOTH).unwrap();
        dac.set_dac_routing(Channels::RIGHT).unwrap();
        dac.release().done();
    }

    /// Pins the gain's read-modify-write: the six gain bits change, in two's complement, and the
    /// mute bit stays.
    #[test]
    fn set_headphone_gain_changes_only_the_gain_bits() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 1]),
            // HPL muted at 0 dB becomes muted at -6 dB (0x3A).
            Transaction::write_read(ADDRESS, vec![16], vec![0x40]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![16, 0x7A]),
            // HPR unmuted at -5 dB becomes unmuted at +29 dB (0x1D).
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write_read(ADDRESS, vec![17], vec![0x3B]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![17, 0x1D]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_headphone_gain(HeadphoneGain::MIN, HeadphoneGain::MAX)
            .unwrap();
        dac.release().done();
    }

    /// Pins the driver power register, written whole, and the mute bit's read-modify-write in
    /// the two gain registers.
    #[test]
    fn headphone_power_and_mute_write_registers_9_16_17_of_page_1() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 1]),
            // HPR alone, then both, then none.
            Transaction::write(ADDRESS, vec![9, 0x10]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![9, 0x30]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![9, 0x00]),
            // HPL at +5 dB and muted: unmute it, the gain stays.
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write_read(ADDRESS, vec![16], vec![0x45]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![16, 0x05]),
            // HPR at -5 dB and unmuted: mute it, the gain stays.
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write_read(ADDRESS, vec![17], vec![0x3B]),
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![17, 0x7B]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_headphone_power(Channels::RIGHT).unwrap();
        dac.set_headphone_power(Channels::BOTH).unwrap();
        dac.set_headphone_power(Channels::NONE).unwrap();
        dac.set_headphone_mute(Channels::RIGHT).unwrap();
        dac.release().done();
    }
}

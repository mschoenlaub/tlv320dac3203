//! The beep generator: a sine burst of programmed frequency, length and volume that the chip makes
//! itself, in processing block PRB_P25 only (§2.4.5.1; Table 2-13; registers in §5.2.65 to
//! §5.2.73).
//!
//! The beep needs no audio interface: its samples come from the generator, at the DAC's sample
//! rate, which the clock tree derives from CODEC_CLKIN (Equation 15). It does need that clock, the
//! DAC channels powered and routed, and the headphone drivers powered, like any other output ("the
//! DAC must be configured with regards to clock setup and routing", §2.4.5.1). In PRB_P25 it is
//! added after the digital volume and the DRC, just before the modulator, with its own volume
//! (Figure 2-30), so the DAC volume does not scale it [I: read from the figure].
//!
//! The chip counts the beep's length down itself and clears the enable bit at the end, so a beep
//! cannot outlast its length even if nothing ever stops it ([`BeepLength`]); that bit is also what
//! tells, without listening, that the DAC's clock runs and how fast ([`Tlv320dac3203::beep_running`]).
//!
//! What each value may be is in its type: a [`BeepVolume`] cannot be louder than the generator's
//! own 0 dB, a [`BeepLength`] is 1 to 2^24 - 1 samples, a [`BeepTone`] has no zero coefficient. A
//! board narrows those to its own ceilings.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::clock::Divider;
use crate::register::{self, beep_left, beep_right, dac_flags_2};
use crate::{Channels, Error, Tlv320dac3203};

/// How long [`Tlv320dac3203::insert_beep`] waits between two looks at the DAC gain flags, in
/// milliseconds. With soft-stepping disabled the flags should be set at the first look (§2.4.4.1);
/// the guide's script only says to wait for them (§2.4.5.1).
pub const BEEP_GAIN_FLAGS_POLL_MS: u32 = 1;

/// How many times [`Tlv320dac3203::insert_beep`] waits for the gain flags before it gives up:
/// 20 ms, far beyond a volume ramp of a muted channel with soft-stepping disabled.
pub const BEEP_GAIN_FLAGS_POLL_LIMIT: u32 = 20;

/// The volume of the beep, as an attenuation from the generator's maximum: 0 dB to -63 dB in 1 dB
/// steps (§5.2.65 and §5.2.66). Nothing louder than the generator's own 0 dB exists, so no value of
/// this type is louder than the chip can make.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct BeepVolume(u8);

impl BeepVolume {
    /// 0 dB, the generator's maximum output level and the reset value (§2.4.5.1).
    pub const MAX: Self = Self(0);
    /// -63 dB, the quietest setting.
    pub const MIN: Self = Self(63);

    /// The volume `attenuation_db` below the maximum. `None` above 63: the six-bit field has no
    /// code for it (§5.2.65).
    pub const fn from_attenuation_db(attenuation_db: u8) -> Option<Self> {
        if attenuation_db <= Self::MIN.0 {
            Some(Self(attenuation_db))
        } else {
            None
        }
    }

    /// How far below the maximum, in dB: 0 to 63.
    pub const fn attenuation_db(self) -> u8 {
        self.0
    }

    /// The field's bits: the attenuation in dB is the code (§5.2.65).
    const fn bits(self) -> u8 {
        self.0 & beep_left::VOLUME_MASK
    }
}

/// The length of the beep in samples of the DAC's sample rate: 1 to 16 777 215, the 24-bit counter
/// of registers 73 to 75 (§2.4.5.1: "This 24-bit length timer supports 16,777,215 sample times";
/// §5.2.67 to §5.2.69).
///
/// The section's formula `Beep Length = floor(Cycles * Fs / Fin)` disagrees with its own example
/// (96000 samples at 48 kHz are two seconds) and with the registers' name, "Beep Sample Length";
/// the length is counted in samples here. Zero is refused: what the chip does with it is not
/// documented.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct BeepLength(u32);

impl BeepLength {
    /// The longest length the counter holds.
    pub const MAX_SAMPLES: u32 = 0xFF_FFFF;

    /// `None` unless `samples` is 1 to [`Self::MAX_SAMPLES`].
    pub const fn from_samples(samples: u32) -> Option<Self> {
        if samples >= 1 && samples <= Self::MAX_SAMPLES {
            Some(Self(samples))
        } else {
            None
        }
    }

    /// `ms` milliseconds at a sample rate of `fs_hz`, rounded down; `None` if that is no valid
    /// length.
    pub const fn from_ms(ms: u32, fs_hz: u32) -> Option<Self> {
        Self::from_samples(((ms as u64 * fs_hz as u64) / 1000) as u32)
    }

    /// The length in samples.
    pub const fn samples(self) -> u32 {
        self.0
    }

    /// The length in milliseconds at `fs_hz`, rounded up (0 for a rate of 0).
    pub const fn ms(self, fs_hz: u32) -> u32 {
        if fs_hz == 0 {
            0
        } else {
            ((self.0 as u64 * 1000).div_ceil(fs_hz as u64)) as u32
        }
    }

    /// The values of registers 73, 74 and 75: bits 23 to 16, 15 to 8 and 7 to 0.
    const fn bytes(self) -> [u8; 3] {
        let [_, high, mid, low] = self.0.to_be_bytes();
        [high, mid, low]
    }
}

/// The tone: the generator's sine and cosine coefficients, `round(sin(2 pi Fin / Fs) * 2^15)` and
/// `round(cos(2 pi Fin / Fs) * 2^15)` as 16-bit two's complement numbers (§2.4.5.1; §5.2.70 to
/// §5.2.73). "The sine should not be zero", nor the cosine (§2.4.5.1), so neither can be.
///
/// The coefficients are computed for one frequency at one sample rate; the driver takes them
/// ready-made, so it needs no floating point. A board keeps its tone as constants with the
/// arithmetic in a host test.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BeepTone {
    sin: i16,
    cos: i16,
}

impl BeepTone {
    /// The tone after a reset: sine `0x10D8`, cosine `0x7EE3` (§5.2.70 to §5.2.73), about 1 kHz at
    /// 48 kHz.
    pub const RESET: Self = Self {
        sin: 0x10D8,
        cos: 0x7EE3,
    };

    /// `None` if either coefficient is zero (§2.4.5.1).
    pub const fn from_coefficients(sin: i16, cos: i16) -> Option<Self> {
        if sin == 0 || cos == 0 {
            None
        } else {
            Some(Self { sin, cos })
        }
    }

    /// The sine coefficient.
    pub const fn sin(self) -> i16 {
        self.sin
    }

    /// The cosine coefficient.
    pub const fn cos(self) -> i16 {
        self.cos
    }
}

/// Everything [`Tlv320dac3203::configure_beep`] programs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BeepConfig {
    /// The tone.
    pub tone: BeepTone,
    /// The length.
    pub length: BeepLength,
    /// The volume of both channels: the right one follows the left
    /// ([`beep_right::MASTER_RIGHT_FOLLOWS_LEFT`]).
    pub volume: BeepVolume,
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Programs the beep without starting it: page 0, register 72 (the right channel follows the
    /// left's volume), the length in 73 to 75, the sine in 76 and 77, the cosine in 78 and 79, and
    /// last register 71 with the volume and the enable bit clear. Each register is written whole,
    /// one write each (§5.2.65 to §5.2.73).
    ///
    /// The parameters "cannot be changed" while a beep plays (§2.4.5.1): call it while none does,
    /// for example right after a reset.
    pub fn configure_beep(&mut self, beep: &BeepConfig) -> Result<(), Error<I2C::Error>> {
        self.write(register::BEEP_RIGHT, beep_right::MASTER_RIGHT_FOLLOWS_LEFT)?;
        let [high, mid, low] = beep.length.bytes();
        self.write(register::BEEP_LENGTH_HIGH, high)?;
        self.write(register::BEEP_LENGTH_MID, mid)?;
        self.write(register::BEEP_LENGTH_LOW, low)?;
        let [sin_high, sin_low] = beep.tone.sin.to_be_bytes();
        self.write(register::BEEP_SIN_MSB, sin_high)?;
        self.write(register::BEEP_SIN_LSB, sin_low)?;
        let [cos_high, cos_low] = beep.tone.cos.to_be_bytes();
        self.write(register::BEEP_COS_MSB, cos_high)?;
        self.write(register::BEEP_COS_LSB, cos_low)?;
        self.write(register::BEEP_LEFT, beep.volume.bits())
    }

    /// Starts the beep: register 71 = the enable bit and
    /// `volume`, one write (§5.2.65). Pass the volume [`configure_beep`](Self::configure_beep) was
    /// given: the register holds both.
    pub fn start_beep(&mut self, volume: BeepVolume) -> Result<(), Error<I2C::Error>> {
        self.write(register::BEEP_LEFT, beep_left::ENABLE | volume.bits())
    }

    /// Stops a beep early: register 71 = `volume` with the enable bit clear, one write (§2.4.5.1:
    /// "To stop the sine wave before the predefined time period expires, set the Beep Generator
    /// Enable Bit to 0").
    pub fn stop_beep(&mut self, volume: BeepVolume) -> Result<(), Error<I2C::Error>> {
        self.write(register::BEEP_LEFT, volume.bits())
    }

    /// Whether a beep is playing: register 71 bit D7, which the chip clears at the end of the beep
    /// (§5.2.65).
    pub fn beep_running(&mut self) -> Result<bool, Error<I2C::Error>> {
        Ok(self.read(register::BEEP_LEFT)? & beep_left::ENABLE != 0)
    }

    /// Starts the beep on a DAC that is already powered, in the order TI gives for inserting a beep
    /// into running playback (§2.4.5.1): mute both DAC channels (register 64); wait until both DAC
    /// gain flags are set (register 38 `xxx1xxx1`, every [`BEEP_GAIN_FLAGS_POLL_MS`] for
    /// [`BEEP_GAIN_FLAGS_POLL_LIMIT`] waits at most); power NDAC down; register 71 = enable and `volume`;
    /// power NDAC up; unmute the channels in `unmute` and keep the others muted. The beep starts
    /// with NDAC.
    ///
    /// The mute, the NDAC switch and the unmute are read-modify-writes of their registers. The
    /// beep's parameters have to be programmed before ([`configure_beep`](Self::configure_beep)).
    ///
    /// # Errors
    ///
    /// [`Error::Timeout`] on register 38 if the gain flags never come: nothing after the mute was
    /// written, both channels stay muted and no beep starts.
    pub fn insert_beep<D: DelayNs>(
        &mut self,
        volume: BeepVolume,
        unmute: Channels,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        self.set_dac_mute(Channels::BOTH)?;
        let applied = dac_flags_2::LEFT_GAIN_APPLIED | dac_flags_2::RIGHT_GAIN_APPLIED;
        self.wait_for(
            register::DAC_FLAGS_2,
            applied,
            applied,
            BEEP_GAIN_FLAGS_POLL_MS,
            BEEP_GAIN_FLAGS_POLL_LIMIT,
            delay,
        )?;
        self.set_divider_power(Divider::Ndac, false)?;
        self.start_beep(volume)?;
        self.set_divider_power(Divider::Ndac, true)?;
        self.set_dac_mute(unmute.inverted())
    }
}

// The ranges above keep a reserved code out of the chip; these fail the build if one drifts.
const _: () = {
    assert!(BeepVolume::from_attenuation_db(63).is_some());
    assert!(BeepVolume::from_attenuation_db(64).is_none());
    assert!(BeepLength::from_samples(0).is_none());
    assert!(BeepLength::from_samples(0xFF_FFFF).is_some());
    assert!(BeepLength::from_samples(0x100_0000).is_none());
    assert!(BeepTone::from_coefficients(0, 1).is_none());
    assert!(BeepTone::from_coefficients(1, 0).is_none());
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;
    use crate::test_support::{BusDelay, wait_ms};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    fn write(register: u8, value: u8) -> Transaction {
        Transaction::write(ADDRESS, vec![register, value])
    }

    fn read(register: u8, value: u8) -> Transaction {
        Transaction::write_read(ADDRESS, vec![register], vec![value])
    }

    /// 1 kHz at 25 kHz for 300 ms at -30 dB: the values of `.superpowers/beep/dac.md`, §2.
    fn beep() -> BeepConfig {
        BeepConfig {
            tone: BeepTone::from_coefficients(0x1FD5, 0x7BFB).unwrap(),
            length: BeepLength::from_ms(300, 25_000).unwrap(),
            volume: BeepVolume::from_attenuation_db(30).unwrap(),
        }
    }

    /// Pins the configuration: page 0, register 72 (right follows left), the length, the sine and
    /// the cosine high byte first, and register 71 last with the enable bit clear.
    #[test]
    fn configure_beep_writes_registers_72_to_79_then_71() {
        let expectations = [
            write(0, 0),
            write(72, 0x80),
            write(0, 0),
            write(73, 0x00),
            write(0, 0),
            write(74, 0x1D),
            write(0, 0),
            write(75, 0x4C),
            write(0, 0),
            write(76, 0x1F),
            write(0, 0),
            write(77, 0xD5),
            write(0, 0),
            write(78, 0x7B),
            write(0, 0),
            write(79, 0xFB),
            write(0, 0),
            write(71, 0x1E),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.configure_beep(&beep()).unwrap();
        dac.release().done();
    }

    /// A negative cosine (a tone above a quarter of the sample rate) goes out in two's complement.
    #[test]
    fn configure_beep_writes_a_negative_coefficient_in_twos_complement() {
        let expectations = [
            write(0, 0),
            write(72, 0x80),
            write(0, 0),
            write(73, 0xFF),
            write(0, 0),
            write(74, 0xFF),
            write(0, 0),
            write(75, 0xFF),
            write(0, 0),
            write(76, 0x7F),
            write(0, 0),
            write(77, 0xFF),
            write(0, 0),
            write(78, 0x80),
            write(0, 0),
            write(79, 0x00),
            write(0, 0),
            write(71, 0x00),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.configure_beep(&BeepConfig {
            tone: BeepTone::from_coefficients(i16::MAX, i16::MIN).unwrap(),
            length: BeepLength::from_samples(BeepLength::MAX_SAMPLES).unwrap(),
            volume: BeepVolume::MAX,
        })
        .unwrap();
        dac.release().done();
    }

    /// Pins the start and the stop: one write of register 71 each, with and without the enable.
    #[test]
    fn start_and_stop_write_register_71() {
        let expectations = [
            write(0, 0),
            write(71, 0x9E),
            write(0, 0),
            write(71, 0x1E),
            write(0, 0),
            write(71, 0xBF),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        let volume = BeepVolume::from_attenuation_db(30).unwrap();
        dac.start_beep(volume).unwrap();
        dac.stop_beep(volume).unwrap();
        dac.start_beep(BeepVolume::MIN).unwrap();
        dac.release().done();
    }

    /// Pins TI's insertion sequence (§2.4.5.1): mute both (register 64), the gain flags (38) twice
    /// with a 1 ms wait, NDAC off (11), the beep on (71), NDAC on, and only the right channel
    /// unmuted.
    #[test]
    fn insert_beep_follows_tis_sequence() {
        let expectations = [
            write(0, 0),
            read(64, 0x00),
            write(0, 0),
            write(64, 0x0C),
            write(0, 0),
            read(38, 0x10),
            wait_ms(1),
            write(0, 0),
            read(38, 0x11),
            write(0, 0),
            read(11, 0x81),
            write(0, 0),
            write(11, 0x01),
            write(0, 0),
            write(71, 0x9E),
            write(0, 0),
            read(11, 0x01),
            write(0, 0),
            write(11, 0x81),
            write(0, 0),
            read(64, 0x0C),
            write(0, 0),
            write(64, 0x08),
        ];
        let mut bus = Mock::new(&expectations);
        let mut delay = BusDelay(bus.clone());
        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        let volume = BeepVolume::from_attenuation_db(30).unwrap();
        dac.insert_beep(volume, Channels::RIGHT, &mut delay)
            .unwrap();
        bus.done();
    }

    /// When the gain flags never come, nothing after the mute is written: 21 reads of register 38
    /// with 20 waits, then the timeout.
    #[test]
    fn insert_beep_without_gain_flags_writes_nothing_more() {
        let mut expectations = vec![write(0, 0), read(64, 0x00), write(0, 0), write(64, 0x0C)];
        for _ in 0..BEEP_GAIN_FLAGS_POLL_LIMIT {
            expectations.push(write(0, 0));
            expectations.push(read(38, 0x01));
            expectations.push(wait_ms(1));
        }
        expectations.push(write(0, 0));
        expectations.push(read(38, 0x01));
        let mut bus = Mock::new(&expectations);
        let mut delay = BusDelay(bus.clone());
        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        assert_eq!(
            dac.insert_beep(BeepVolume::MIN, Channels::BOTH, &mut delay),
            Err(Error::Timeout(register::DAC_FLAGS_2))
        );
        bus.done();
    }

    #[test]
    fn the_value_types_hold_only_what_the_registers_encode() {
        assert_eq!(
            BeepVolume::from_attenuation_db(30)
                .unwrap()
                .attenuation_db(),
            30
        );
        assert_eq!(BeepVolume::from_attenuation_db(64), None);
        assert!(BeepVolume::MAX < BeepVolume::MIN);
        let length = BeepLength::from_ms(300, 25_000).unwrap();
        assert_eq!(length.samples(), 7500);
        assert_eq!(length.ms(25_000), 300);
        assert_eq!(length.ms(0), 0);
        assert_eq!(BeepLength::from_ms(0, 25_000), None);
        assert_eq!(BeepLength::from_ms(700_000, 25_000), None);
        assert_eq!(
            BeepLength::from_ms(2000, 48_000).unwrap().samples(),
            96_000,
            "section 2.4.5.1's example: 96000 samples at 48 kHz are two seconds"
        );
        assert_eq!(BeepTone::from_coefficients(0, 5), None);
        assert_eq!(BeepTone::RESET.sin(), 0x10D8);
        assert_eq!(BeepTone::RESET.cos(), 0x7EE3);
    }
}

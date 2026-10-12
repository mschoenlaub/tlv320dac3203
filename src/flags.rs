//! The flags that show, without listening, how far playback has come ([`PlaybackFlags`]), read in
//! one go by [`Tlv320dac3203::read_playback_flags`].
//!
//! How a board stores them (a result record, a log line) is the board's business: this module
//! only reads and decodes. Until 2026-10-07 [`PlaybackFlags`] also packed itself into record
//! words; that packing moved to the one payload that writes such a record, since no other codec
//! driver carries a record format and this chip's registers say nothing about one.

use embedded_hal::i2c::I2c;

use crate::register::{self, beep_left, dac_flags_1, hp_gain_flags};
use crate::{Error, Tlv320dac3203};

/// The flags that show, without listening, how far playback has come: page 0, registers 37, 38, 43,
/// 46, 64 and 71, and page 1, register 63. Read by [`Tlv320dac3203::read_playback_flags`]; none of
/// them is a clear-on-read register (43 and 46 are the live copies of the sticky 42 and 44, §5.2.40
/// and §5.2.43).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PlaybackFlags {
    /// Page 0, register 37: DAC and headphone driver power (§5.2.36).
    pub dac_flags_1: u8,
    /// Page 0, register 38: DAC gain applied (§5.2.37).
    pub dac_flags_2: u8,
    /// Page 0, register 43: the DAC overflow flags, live (§5.2.40).
    pub interrupt_flags_1: u8,
    /// Page 0, register 46: headphone over-current and the other event flags, live (§5.2.43).
    pub interrupt_flags_2: u8,
    /// Page 0, register 64: the DAC mute bits and the rest of the channel setup (§5.2.58).
    pub dac_setup_2: u8,
    /// Page 0, register 71: the beep's enable bit and volume (§5.2.65). Read in every processing
    /// block: the register exists whether or not the beep generator is used.
    pub beep_left: u8,
    /// Page 1, register 63: headphone driver gain applied (§5.3.25).
    pub hp_gain_flags: u8,
}

impl PlaybackFlags {
    /// Both DACs and both headphone drivers report powered: register 37 = `0xAA`.
    pub const fn all_powered(&self) -> bool {
        let all = dac_flags_1::LEFT_DAC_POWERED
            | dac_flags_1::HPL_POWERED
            | dac_flags_1::RIGHT_DAC_POWERED
            | dac_flags_1::HPR_POWERED;
        self.dac_flags_1 & all == all
    }

    /// Both headphone drivers report their gain applied: page 1, register 63 bits D7 and D6.
    pub const fn headphone_gain_applied(&self) -> bool {
        let both = hp_gain_flags::HPL_GAIN_APPLIED | hp_gain_flags::HPR_GAIN_APPLIED;
        self.hp_gain_flags & both == both
    }

    /// The beep is playing: register 71 bit D7.
    pub const fn beep_running(&self) -> bool {
        self.beep_left & beep_left::ENABLE != 0
    }
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Reads [`PlaybackFlags`]: page 0, registers 37, 38, 43, 46, 64 and 71 in that order, then
    /// page 1, register 63, selecting each page as needed. Writes only the page-select register,
    /// so it is for a chip this driver owns.
    pub fn read_playback_flags(&mut self) -> Result<PlaybackFlags, Error<I2C::Error>> {
        Ok(PlaybackFlags {
            dac_flags_1: self.read(register::DAC_FLAGS_1)?,
            dac_flags_2: self.read(register::DAC_FLAGS_2)?,
            interrupt_flags_1: self.read(register::INTERRUPT_FLAGS_1)?,
            interrupt_flags_2: self.read(register::INTERRUPT_FLAGS_2)?,
            dac_setup_2: self.read(register::DAC_SETUP_2)?,
            beep_left: self.read(register::BEEP_LEFT)?,
            hp_gain_flags: self.read(register::HP_GAIN_FLAGS)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    fn read(register: u8, value: u8) -> Transaction {
        Transaction::write_read(ADDRESS, vec![register], vec![value])
    }

    #[test]
    fn the_flags_decode() {
        let flags = PlaybackFlags {
            dac_flags_1: 0xAA,
            dac_flags_2: 0x11,
            interrupt_flags_1: 0,
            interrupt_flags_2: 0,
            dac_setup_2: 0x0C,
            beep_left: 0x9E,
            hp_gain_flags: 0xC0,
        };
        assert!(flags.all_powered() && flags.headphone_gain_applied() && flags.beep_running());
        let off = PlaybackFlags {
            dac_flags_1: 0x88,
            hp_gain_flags: 0x80,
            ..flags
        };
        assert!(!off.all_powered() && !off.headphone_gain_applied());
    }

    /// The read path: page 0's six registers in order, then page 1's, one page select each.
    #[test]
    fn read_playback_flags_reads_page_0_then_page_1() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            read(37, 0xAA),
            Transaction::write(ADDRESS, vec![0, 0]),
            read(38, 0x11),
            Transaction::write(ADDRESS, vec![0, 0]),
            read(43, 0x00),
            Transaction::write(ADDRESS, vec![0, 0]),
            read(46, 0x00),
            Transaction::write(ADDRESS, vec![0, 0]),
            read(64, 0x08),
            Transaction::write(ADDRESS, vec![0, 0]),
            read(71, 0x00),
            Transaction::write(ADDRESS, vec![0, 1]),
            read(63, 0xC0),
        ];
        let mut bus = Mock::new(&expectations);
        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        let flags = dac.read_playback_flags().unwrap();
        assert!(flags.all_powered() && flags.headphone_gain_applied());
        assert_eq!(flags.dac_setup_2, 0x08);
        bus.done();
    }
}

//! The serial audio interface: format, word length, who drives the clocks and where in the frame
//! the data starts (§2.6).
//!
//! # What the driver does not cover
//!
//! The chip's data output is the MFP2 pin, which on this DAC only carries loopback data. The two
//! bits that belong to it, loopback (page 0, register 29, bit D5) and the high-impedance control
//! (register 27, bit D0), are always written as 0, their reset values (§5.2.27 and §5.2.29). The
//! secondary audio interface (registers 31 to 33) is not touched.

use embedded_hal::i2c::I2c;

use crate::register::{self, interface_1, interface_3};
use crate::{Error, Tlv320dac3203};

/// The serial format: page 0, register 27, bits D7 to D6 (§5.2.27).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// I2S, the reset value: each channel's first bit is valid on the second rising bit clock edge
    /// after its edge of the word clock (§2.6.3).
    I2s,
    /// DSP: the rising edge of the word clock starts the left channel and the right channel follows
    /// it immediately (§2.6.4).
    Dsp,
    /// Right-justified (§2.6.1).
    RightJustified,
    /// Left-justified (§2.6.2).
    LeftJustified,
}

impl Format {
    /// The field's bits, in place (§5.2.27).
    const fn bits(self) -> u8 {
        match self {
            Format::I2s => interface_1::FORMAT_I2S,
            Format::Dsp => interface_1::FORMAT_DSP,
            Format::RightJustified => interface_1::FORMAT_RIGHT_JUSTIFIED,
            Format::LeftJustified => interface_1::FORMAT_LEFT_JUSTIFIED,
        }
    }

    /// The format a value of register 27 selects: the inverse of `bits`. The two-bit field has no
    /// reserved code (§5.2.27).
    const fn from_bits(value: u8) -> Self {
        match value & interface_1::FORMAT_MASK {
            interface_1::FORMAT_I2S => Format::I2s,
            interface_1::FORMAT_DSP => Format::Dsp,
            interface_1::FORMAT_RIGHT_JUSTIFIED => Format::RightJustified,
            _ => Format::LeftJustified,
        }
    }
}

/// The word length: page 0, register 27, bits D5 to D4 (§5.2.27).
///
/// The word length also decides between the two highest PowerTune modes: PTM_P4 needs words of 20
/// bits or more (§2.5.1.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WordLength {
    /// 16 bits, the reset value.
    Bits16,
    /// 20 bits.
    Bits20,
    /// 24 bits.
    Bits24,
    /// 32 bits.
    Bits32,
}

impl WordLength {
    /// The field's bits, in place (§5.2.27).
    const fn bits(self) -> u8 {
        match self {
            WordLength::Bits16 => interface_1::WORD_LENGTH_16,
            WordLength::Bits20 => interface_1::WORD_LENGTH_20,
            WordLength::Bits24 => interface_1::WORD_LENGTH_24,
            WordLength::Bits32 => interface_1::WORD_LENGTH_32,
        }
    }

    /// The word length a value of register 27 selects: the inverse of `bits`. The two-bit field has
    /// no reserved code (§5.2.27).
    const fn from_bits(value: u8) -> Self {
        match value & interface_1::WORD_LENGTH_MASK {
            interface_1::WORD_LENGTH_16 => WordLength::Bits16,
            interface_1::WORD_LENGTH_20 => WordLength::Bits20,
            interface_1::WORD_LENGTH_24 => WordLength::Bits24,
            _ => WordLength::Bits32,
        }
    }
}

/// Whether the chip listens to a clock pin or drives it: page 0, register 27, bit D3 for BCLK and
/// bit D2 for WCLK (§5.2.27).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockDirection {
    /// The pin is an input and another device provides the clock. The reset value.
    Input,
    /// The pin is an output and this chip provides the clock.
    Output,
}

impl ClockDirection {
    /// The direction a pin's output bit of register 27 stands for (§5.2.27).
    const fn from_output_bit(output: bool) -> Self {
        if output {
            ClockDirection::Output
        } else {
            ClockDirection::Input
        }
    }
}

/// What feeds the BCLK N divider, BDIV_CLKIN: page 0, register 29, bits D1 to D0 (§5.2.29). The
/// other two codes are reserved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BclkDividerInput {
    /// DAC_CLK, the output of NDAC. The reset value.
    DacClk,
    /// DAC_MOD_CLK, the output of MDAC.
    DacModClk,
}

impl BclkDividerInput {
    /// The field's bits, in place (§5.2.29).
    const fn bits(self) -> u8 {
        match self {
            BclkDividerInput::DacClk => interface_3::BDIV_CLKIN_DAC_CLK,
            BclkDividerInput::DacModClk => interface_3::BDIV_CLKIN_DAC_MOD_CLK,
        }
    }

    /// The divider input a value of register 29 selects, or `None` for the codes 10 and 11, which
    /// are reserved (§5.2.29).
    const fn from_bits(value: u8) -> Option<Self> {
        match value & interface_3::BDIV_CLKIN_MASK {
            interface_3::BDIV_CLKIN_DAC_CLK => Some(BclkDividerInput::DacClk),
            interface_3::BDIV_CLKIN_DAC_MOD_CLK => Some(BclkDividerInput::DacModClk),
            _ => None,
        }
    }
}

/// The serial interface: registers 27, 28 and 29 of page 0.
///
/// The guide leaves two conditions to the designer, because they depend on the number of bit clocks
/// per frame (§2.6.2 to §2.6.4): a frame must hold two words (in I2S, each half of it at least one;
/// in the DSP and left-justified formats, more than two), and [`data_offset`](Self::data_offset)
/// plus the word length must not exceed the number of bit clocks per frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InterfaceConfig {
    /// The serial format.
    pub format: Format,
    /// The word length.
    pub word_length: WordLength,
    /// Whether this chip drives the bit clock.
    pub bclk: ClockDirection,
    /// Whether this chip drives the word clock.
    pub wclk: ClockDirection,
    /// How many bit clocks after the word clock edge the data starts. The register takes any value
    /// from 0 to 255 (§5.2.28).
    pub data_offset: u8,
    /// Whether the bit clock is inverted: register 29, bit D3 (§5.2.29).
    pub bclk_inverted: bool,
    /// Whether the BCLK and WCLK buffers stay powered while the DAC is powered down: register 29,
    /// bit D2 (§5.2.29). Only meaningful on a chip that drives them (§2.6).
    pub clocks_active_when_powered_down: bool,
    /// What feeds the BCLK N divider.
    pub bclk_divider_input: BclkDividerInput,
}

impl InterfaceConfig {
    /// The interface after a reset: I2S, 16 bits, both clocks inputs, no offset, the BCLK divider
    /// fed from DAC_CLK (§5.2.27 to §5.2.29).
    pub const RESET: Self = Self {
        format: Format::I2s,
        word_length: WordLength::Bits16,
        bclk: ClockDirection::Input,
        wclk: ClockDirection::Input,
        data_offset: 0,
        bclk_inverted: false,
        clocks_active_when_powered_down: false,
        bclk_divider_input: BclkDividerInput::DacClk,
    };

    /// The value of register 27 (§5.2.27).
    const fn setting_1_bits(&self) -> u8 {
        let mut bits = self.format.bits() | self.word_length.bits();
        if matches!(self.bclk, ClockDirection::Output) {
            bits |= interface_1::BCLK_OUTPUT;
        }
        if matches!(self.wclk, ClockDirection::Output) {
            bits |= interface_1::WCLK_OUTPUT;
        }
        bits
    }

    /// The value of register 29, with loopback off (§5.2.29).
    const fn setting_3_bits(&self) -> u8 {
        let mut bits = self.bclk_divider_input.bits();
        if self.bclk_inverted {
            bits |= interface_3::BCLK_INVERTED;
        }
        if self.clocks_active_when_powered_down {
            bits |= interface_3::CLOCKS_ACTIVE_WHEN_POWERED_DOWN;
        }
        bits
    }

    /// The interface the values of registers 27, 28 and 29 describe: the inverse of
    /// `setting_1_bits` and `setting_3_bits`. `None` for a reserved divider input in register 29,
    /// the only field of the three registers with reserved codes (§5.2.27 to §5.2.29).
    ///
    /// The two bits this type has no field for are not looked at: the high-impedance control of
    /// the data output (register 27, bit D0) and the loopback (register 29, bit D5).
    pub(crate) fn from_registers(setting_1: u8, data_offset: u8, setting_3: u8) -> Option<Self> {
        Some(Self {
            format: Format::from_bits(setting_1),
            word_length: WordLength::from_bits(setting_1),
            bclk: ClockDirection::from_output_bit(setting_1 & interface_1::BCLK_OUTPUT != 0),
            wclk: ClockDirection::from_output_bit(setting_1 & interface_1::WCLK_OUTPUT != 0),
            data_offset,
            bclk_inverted: setting_3 & interface_3::BCLK_INVERTED != 0,
            clocks_active_when_powered_down: setting_3
                & interface_3::CLOCKS_ACTIVE_WHEN_POWERED_DOWN
                != 0,
            bclk_divider_input: BclkDividerInput::from_bits(setting_3)?,
        })
    }
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Programs the serial interface: registers 27, 28 and 29 of page 0, in that order, each
    /// written whole.
    ///
    /// The kernel driver writes the same three registers when the machine driver sets the format
    /// (`aic32x4.c:840-842`), and register 27 once more for the word length
    /// (`aic32x4.c:923-938`); it reads registers 27 and 29 first and keeps their other bits, which
    /// are 0 after a reset. Writing the registers whole leaves the same contents and does not
    /// depend on what was there before.
    pub fn configure_interface(
        &mut self,
        interface: &InterfaceConfig,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(register::INTERFACE_1, interface.setting_1_bits())?;
        self.write(register::DATA_OFFSET, interface.data_offset)?;
        self.write(register::INTERFACE_3, interface.setting_3_bits())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins the write sequence of the interface with every field away from its reset value.
    #[test]
    fn configure_interface_writes_registers_27_28_29() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            // DSP, 24 bits, BCLK and WCLK driven by the chip.
            Transaction::write(ADDRESS, vec![27, 0x6C]),
            // The data starts one bit clock after the word clock edge.
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![28, 0x01]),
            // BCLK inverted, clocks kept running, BCLK divider fed from DAC_MOD_CLK.
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![29, 0x0D]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        let interface = InterfaceConfig {
            format: Format::Dsp,
            word_length: WordLength::Bits24,
            bclk: ClockDirection::Output,
            wclk: ClockDirection::Output,
            data_offset: 1,
            bclk_inverted: true,
            clocks_active_when_powered_down: true,
            bclk_divider_input: BclkDividerInput::DacModClk,
        };
        dac.configure_interface(&interface).unwrap();
        dac.release().done();
    }
}

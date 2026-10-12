//! The two multifunction pins the kernel driver configures: MFP2 and SCLK/MFP3 (§2.1.1.1; §5.2.49
//! and §5.2.52).
//!
//! The chip has more such pins (DIN/MFP1, MISO/MFP4 and, in the WCSP package, GPIO/MFP5). They
//! are a gap of this driver: it has no method for them yet, and they keep their reset values.

use embedded_hal::i2c::I2c;

use crate::register::{self, mfp2_control, sclk_mfp3_control};
use crate::{Error, Tlv320dac3203};

/// What the MFP2 pin does: page 0, register 53, bits D3 to D1, and for the general-purpose output
/// its level in bit D0 (§5.2.49).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mfp2Function {
    /// The pin is disabled.
    Disabled,
    /// The primary data output, which on this chip carries loopback data. The reset value.
    DataOutput,
    /// A general-purpose output that drives the given level.
    Output {
        /// Whether the pin is driven high.
        high: bool,
    },
    /// The CLKOUT output.
    Clkout,
    /// The INT1 interrupt output.
    Int1,
    /// The INT2 interrupt output.
    Int2,
    /// The bit clock of the secondary audio interface.
    SecondaryBclk,
    /// The word clock of the secondary audio interface.
    SecondaryWclk,
}

impl Mfp2Function {
    /// The function field and the output level, in place (§5.2.49). The level bit is written as 0
    /// for every function but the general-purpose output.
    const fn bits(self) -> u8 {
        match self {
            Mfp2Function::Disabled => mfp2_control::FUNCTION_DISABLED,
            Mfp2Function::DataOutput => mfp2_control::FUNCTION_PRIMARY_DOUT,
            Mfp2Function::Output { high: false } => mfp2_control::FUNCTION_GENERAL_PURPOSE_OUTPUT,
            Mfp2Function::Output { high: true } => {
                mfp2_control::FUNCTION_GENERAL_PURPOSE_OUTPUT | mfp2_control::GENERAL_PURPOSE_VALUE
            }
            Mfp2Function::Clkout => mfp2_control::FUNCTION_CLKOUT,
            Mfp2Function::Int1 => mfp2_control::FUNCTION_INT1,
            Mfp2Function::Int2 => mfp2_control::FUNCTION_INT2,
            Mfp2Function::SecondaryBclk => mfp2_control::FUNCTION_SECONDARY_BCLK,
            Mfp2Function::SecondaryWclk => mfp2_control::FUNCTION_SECONDARY_WCLK,
        }
    }

    /// The function a value of register 53 selects: the inverse of `bits`. All eight codes of the
    /// field have a meaning. The level bit is only looked at for the general-purpose output, which
    /// is the one function it belongs to (§5.2.49).
    const fn from_bits(value: u8) -> Self {
        match value & mfp2_control::FUNCTION_MASK {
            mfp2_control::FUNCTION_DISABLED => Mfp2Function::Disabled,
            mfp2_control::FUNCTION_PRIMARY_DOUT => Mfp2Function::DataOutput,
            mfp2_control::FUNCTION_GENERAL_PURPOSE_OUTPUT => Mfp2Function::Output {
                high: value & mfp2_control::GENERAL_PURPOSE_VALUE != 0,
            },
            mfp2_control::FUNCTION_CLKOUT => Mfp2Function::Clkout,
            mfp2_control::FUNCTION_INT1 => Mfp2Function::Int1,
            mfp2_control::FUNCTION_INT2 => Mfp2Function::Int2,
            mfp2_control::FUNCTION_SECONDARY_BCLK => Mfp2Function::SecondaryBclk,
            _ => Mfp2Function::SecondaryWclk,
        }
    }
}

/// The MFP2 pin: page 0, register 53 (§5.2.49).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mfp2Config {
    /// What the pin does.
    pub function: Mfp2Function,
    /// Whether the pin's bus keeper is enabled: bit D4, inverted. It is disabled after a reset.
    pub bus_keeper: bool,
}

impl Mfp2Config {
    /// The pin after a reset: the data output, bus keeper disabled (§5.2.49).
    pub const RESET: Self = Self {
        function: Mfp2Function::DataOutput,
        bus_keeper: false,
    };

    /// The register value (§5.2.49).
    const fn bits(self) -> u8 {
        let keeper = if self.bus_keeper {
            0
        } else {
            mfp2_control::BUS_KEEPER_DISABLED
        };
        keeper | self.function.bits()
    }

    /// The pin configuration a value of register 53 describes: the inverse of `bits` (§5.2.49).
    pub(crate) const fn from_bits(value: u8) -> Self {
        Self {
            function: Mfp2Function::from_bits(value),
            bus_keeper: value & mfp2_control::BUS_KEEPER_DISABLED == 0,
        }
    }
}

/// What the SCLK/MFP3 pin does while the chip is controlled over I2C: page 0, register 56, bits D2
/// to D1 (§5.2.52). The fourth code is reserved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SclkMfp3Function {
    /// The pin is disabled.
    Disabled,
    /// The pin is an input of the secondary audio interface or of a digital microphone. The reset
    /// value.
    Enabled,
    /// A general-purpose input, whose level reads back in bit D0 of the register.
    GeneralPurposeInput,
}

impl SclkMfp3Function {
    /// The register value (§5.2.52). Bit D0 is read-only and written as 0.
    const fn bits(self) -> u8 {
        match self {
            SclkMfp3Function::Disabled => sclk_mfp3_control::FUNCTION_DISABLED,
            SclkMfp3Function::Enabled => sclk_mfp3_control::FUNCTION_ENABLED,
            SclkMfp3Function::GeneralPurposeInput => {
                sclk_mfp3_control::FUNCTION_GENERAL_PURPOSE_INPUT
            }
        }
    }
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Configures the MFP2 pin: page 0, register 53, written whole (§5.2.49).
    ///
    /// The kernel driver makes the pin a general-purpose output and drives it high at probe and
    /// at every stream stop, "to mute AMP if present" (`aic32x4.c:1178-1181` and `:332-338`).
    /// Whether the pin is wired to anything is the board's business.
    pub fn configure_mfp2(&mut self, mfp2: Mfp2Config) -> Result<(), Error<I2C::Error>> {
        self.write(register::MFP2_CONTROL, mfp2.bits())
    }

    /// Configures the SCLK/MFP3 pin: page 0, register 56, written whole (§5.2.52). The kernel
    /// driver disables the pin at probe (`aic32x4.c:1182-1183`).
    pub fn set_sclk_mfp3(&mut self, function: SclkMfp3Function) -> Result<(), Error<I2C::Error>> {
        self.write(register::SCLK_MFP3_CONTROL, function.bits())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins the two pin registers: MFP2 as INT1 with the bus keeper off, then as a low output
    /// with the bus keeper on, and SCLK/MFP3 as a general-purpose input.
    #[test]
    fn pin_functions_write_registers_53_and_56() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![53, 0x18]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![53, 0x04]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![56, 0x04]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.configure_mfp2(Mfp2Config {
            function: Mfp2Function::Int1,
            bus_keeper: false,
        })
        .unwrap();
        dac.configure_mfp2(Mfp2Config {
            function: Mfp2Function::Output { high: false },
            bus_keeper: true,
        })
        .unwrap();
        dac.set_sclk_mfp3(SclkMfp3Function::GeneralPurposeInput)
            .unwrap();
        dac.release().done();
    }
}

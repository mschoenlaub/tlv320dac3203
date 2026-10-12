//! Power supply configuration: the AVDD LDO, the master switch of the analog blocks, the common
//! mode voltages and how the analog reference powers up (§2.9; §2.10; §3.3 and §3.5).
//!
//! These are the settings a board fixes once. The kernel driver writes them at probe and never
//! again (`aic32x4.c:1133-1150` and `:1185`), and the guide's setup list has them as "Program
//! Analog Blocks": disable the coarse AVDD generation, enable the master analog power control,
//! program the common mode voltage, program the reference fast charging (§2.4.6, step 2). Which
//! values are right depends on how the board supplies the chip, which the driver cannot know.

use embedded_hal::i2c::I2c;

use crate::register::{self, common_mode, ldo_control, power_config, reference_power_up};
use crate::{Error, Tlv320dac3203};

/// The AVDD LDO's nominal output voltage: page 1, register 2, bits D5 to D4 (§5.3.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LdoVoltage {
    /// 1.72 V, the reset value.
    V1_72,
    /// 1.67 V.
    V1_67,
    /// 1.77 V.
    V1_77,
}

impl LdoVoltage {
    /// The field's bits, in place (§5.3.3).
    const fn bits(self) -> u8 {
        match self {
            LdoVoltage::V1_72 => ldo_control::AVDD_LDO_1V72,
            LdoVoltage::V1_67 => ldo_control::AVDD_LDO_1V67,
            LdoVoltage::V1_77 => ldo_control::AVDD_LDO_1V77,
        }
    }

    /// The voltage a value of register 2 selects, or `None` for the code 11, which is "do not use"
    /// (§5.3.3).
    const fn from_bits(value: u8) -> Option<Self> {
        match value & ldo_control::AVDD_LDO_VOLTAGE_MASK {
            ldo_control::AVDD_LDO_1V72 => Some(LdoVoltage::V1_72),
            ldo_control::AVDD_LDO_1V67 => Some(LdoVoltage::V1_67),
            ldo_control::AVDD_LDO_1V77 => Some(LdoVoltage::V1_77),
            _ => None,
        }
    }
}

/// The chip's common mode voltage, which the DAC and the analog bypass share: page 1, register 10,
/// bit D6 (§5.3.8; §3.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommonMode {
    /// 0.9 V, the reset value. For an analog supply of 1.8 V or more (§3.5).
    V0_9,
    /// 0.75 V. Required when the analog supply is below 1.8 V (§3.5).
    V0_75,
}

/// The output common mode of the two headphone drivers: page 1, register 10, bits D5 to D4
/// (§5.3.8). The three higher settings are for drivers powered from LDOIN ([`HeadphoneSupply`];
/// §2.2.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeadphoneCommonMode {
    /// The same as the chip's common mode, the reset value.
    SameAsChip,
    /// 1.25 V.
    V1_25,
    /// 1.5 V.
    V1_5,
    /// 1.65 V, or 1.5 V if the chip's common mode is [`CommonMode::V0_75`] (§5.3.8).
    V1_65,
}

impl HeadphoneCommonMode {
    /// The field's bits, in place (§5.3.8).
    const fn bits(self) -> u8 {
        match self {
            HeadphoneCommonMode::SameAsChip => common_mode::HP_COMMON_MODE_SAME_AS_CHIP,
            HeadphoneCommonMode::V1_25 => common_mode::HP_COMMON_MODE_1V25,
            HeadphoneCommonMode::V1_5 => common_mode::HP_COMMON_MODE_1V5,
            HeadphoneCommonMode::V1_65 => common_mode::HP_COMMON_MODE_1V65,
        }
    }

    /// The common mode a value of register 10 selects: the inverse of `bits`. The two-bit field has
    /// no reserved code (§5.3.8).
    const fn from_bits(value: u8) -> Self {
        match value & common_mode::HP_COMMON_MODE_MASK {
            common_mode::HP_COMMON_MODE_SAME_AS_CHIP => HeadphoneCommonMode::SameAsChip,
            common_mode::HP_COMMON_MODE_1V25 => HeadphoneCommonMode::V1_25,
            common_mode::HP_COMMON_MODE_1V5 => HeadphoneCommonMode::V1_5,
            _ => HeadphoneCommonMode::V1_65,
        }
    }
}

/// What powers the output stage of the two headphone drivers: page 1, register 10, bits D1 and D0
/// (§5.3.8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeadphoneSupply {
    /// The AVDD supply, the reset value.
    Avdd,
    /// The LDOIN pin, carrying 1.5 V to 1.95 V.
    LdoinLowRange,
    /// The LDOIN pin, carrying 1.8 V to 3.6 V. This is what makes the higher output common modes
    /// and the larger output swing possible (§2.2.2).
    LdoinHighRange,
}

impl HeadphoneSupply {
    /// The two bits, in place (§5.3.8). The range bit only has a meaning while the supply bit is
    /// set, so it is written as 0 for AVDD.
    const fn bits(self) -> u8 {
        match self {
            HeadphoneSupply::Avdd => 0,
            HeadphoneSupply::LdoinLowRange => common_mode::HP_POWERED_FROM_LDOIN,
            HeadphoneSupply::LdoinHighRange => {
                common_mode::HP_POWERED_FROM_LDOIN | common_mode::LDOIN_RANGE_1V8_TO_3V6
            }
        }
    }

    /// The supply a value of register 10 selects: the inverse of `bits`. The range bit only has a
    /// meaning while the supply bit is set, so it is not looked at for AVDD (§5.3.8).
    const fn from_bits(value: u8) -> Self {
        if value & common_mode::HP_POWERED_FROM_LDOIN == 0 {
            HeadphoneSupply::Avdd
        } else if value & common_mode::LDOIN_RANGE_1V8_TO_3V6 == 0 {
            HeadphoneSupply::LdoinLowRange
        } else {
            HeadphoneSupply::LdoinHighRange
        }
    }
}

/// The supply settings: registers 1, 2 and 10 of page 1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SupplyConfig {
    /// Whether AVDD stays weakly connected to DVDD: register 1, bit D3, inverted (§5.3.2). The
    /// connection is on after a reset, for the time AVDD has no supply of its own, and "must be
    /// turned off ... at the time AVdd is applied, either from internal LDO or through external
    /// LDO" (§2.9).
    pub avdd_weakly_connected_to_dvdd: bool,
    /// Whether the chip's own AVDD LDO is powered: register 2, bit D0 (§5.3.3).
    pub avdd_ldo_powered: bool,
    /// The AVDD LDO's output voltage.
    pub avdd_ldo_voltage: LdoVoltage,
    /// Whether the analog blocks are enabled: register 2, bit D3, inverted (§5.3.3). This is the
    /// guide's "Master Analog Power Control". The blocks are disabled after a reset and must stay
    /// disabled while AVDD or LDOIN has no supply (§2.9.1.2).
    pub analog_blocks_enabled: bool,
    /// The chip's common mode voltage.
    pub common_mode: CommonMode,
    /// The headphone drivers' output common mode.
    pub headphone_common_mode: HeadphoneCommonMode,
    /// What powers the headphone drivers' output stage.
    pub headphone_supply: HeadphoneSupply,
}

impl SupplyConfig {
    /// The supply settings after a reset: AVDD weakly connected to DVDD, the LDO off, the analog
    /// blocks disabled, 0.9 V common mode everywhere, headphone drivers on AVDD (§5.3.2, §5.3.3 and
    /// §5.3.8).
    pub const RESET: Self = Self {
        avdd_weakly_connected_to_dvdd: true,
        avdd_ldo_powered: false,
        avdd_ldo_voltage: LdoVoltage::V1_72,
        analog_blocks_enabled: false,
        common_mode: CommonMode::V0_9,
        headphone_common_mode: HeadphoneCommonMode::SameAsChip,
        headphone_supply: HeadphoneSupply::Avdd,
    };

    /// The value of register 1 (§5.3.2).
    const fn power_config_bits(&self) -> u8 {
        if self.avdd_weakly_connected_to_dvdd {
            0
        } else {
            power_config::AVDD_WEAK_CONNECTION_DISABLED
        }
    }

    /// The value of register 2 (§5.3.3). Bits D2 and D1 are read-only flags and written as 0.
    const fn ldo_control_bits(&self) -> u8 {
        let mut bits = self.avdd_ldo_voltage.bits();
        if !self.analog_blocks_enabled {
            bits |= ldo_control::ANALOG_BLOCKS_DISABLED;
        }
        if self.avdd_ldo_powered {
            bits |= ldo_control::AVDD_LDO_POWER_UP;
        }
        bits
    }

    /// The value of register 10 (§5.3.8).
    const fn common_mode_bits(&self) -> u8 {
        let chip = match self.common_mode {
            CommonMode::V0_9 => 0,
            CommonMode::V0_75 => common_mode::FULL_CHIP_0V75,
        };
        chip | self.headphone_common_mode.bits() | self.headphone_supply.bits()
    }

    /// The supply settings the values of registers 1, 2 and 10 describe: the inverse of the three
    /// encoders above. `None` for the reserved LDO voltage code in register 2, the only field of
    /// the three registers with a reserved code (§5.3.2, §5.3.3 and §5.3.8).
    ///
    /// The two read-only flags of register 2, bits D2 and D1, are not settings and not part of
    /// this type.
    pub(crate) fn from_registers(register_1: u8, register_2: u8, register_10: u8) -> Option<Self> {
        Some(Self {
            avdd_weakly_connected_to_dvdd: register_1 & power_config::AVDD_WEAK_CONNECTION_DISABLED
                == 0,
            avdd_ldo_powered: register_2 & ldo_control::AVDD_LDO_POWER_UP != 0,
            avdd_ldo_voltage: LdoVoltage::from_bits(register_2)?,
            analog_blocks_enabled: register_2 & ldo_control::ANALOG_BLOCKS_DISABLED == 0,
            common_mode: if register_10 & common_mode::FULL_CHIP_0V75 != 0 {
                CommonMode::V0_75
            } else {
                CommonMode::V0_9
            },
            headphone_common_mode: HeadphoneCommonMode::from_bits(register_10),
            headphone_supply: HeadphoneSupply::from_bits(register_10),
        })
    }
}

/// How long the analog reference takes to charge its capacitor on the REF pin: page 1, register
/// 123, bits D1 to D0 (§5.3.29; §2.10).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReferenceChargeTime {
    /// No fast charge, the reset value. With the 1 uF capacitor the guide asks for, the reference
    /// then takes about one second to settle (§2.10).
    Slow,
    /// 40 ms.
    Ms40,
    /// 80 ms.
    Ms80,
    /// 120 ms.
    Ms120,
}

impl ReferenceChargeTime {
    /// The field's bits, in place (§5.3.29).
    const fn bits(self) -> u8 {
        match self {
            ReferenceChargeTime::Slow => reference_power_up::TIME_SLOW,
            ReferenceChargeTime::Ms40 => reference_power_up::TIME_40_MS,
            ReferenceChargeTime::Ms80 => reference_power_up::TIME_80_MS,
            ReferenceChargeTime::Ms120 => reference_power_up::TIME_120_MS,
        }
    }

    /// The charge time a value of register 123 selects: the inverse of `bits`. The two-bit field
    /// has no reserved code (§5.3.29).
    const fn from_bits(value: u8) -> Self {
        // The mask of the two time bits alone: the field mask without the force bit.
        match value & (reference_power_up::MODE_MASK & !reference_power_up::FORCE) {
            reference_power_up::TIME_SLOW => ReferenceChargeTime::Slow,
            reference_power_up::TIME_40_MS => ReferenceChargeTime::Ms40,
            reference_power_up::TIME_80_MS => ReferenceChargeTime::Ms80,
            _ => ReferenceChargeTime::Ms120,
        }
    }
}

/// How the analog reference powers up: page 1, register 123 (§5.3.29).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReferencePowerUp {
    /// Whether the reference is forced on now and kept on, instead of powering up with the first
    /// analog block and down with the last: bit D2. A reference that stays on costs about 125 uA
    /// from AVDD and saves its charge time at every start (§2.10).
    pub forced: bool,
    /// How fast it charges.
    pub charge_time: ReferenceChargeTime,
}

impl ReferencePowerUp {
    /// The reference after a reset: powered with the analog blocks, no fast charge (§5.3.29).
    pub const RESET: Self = Self {
        forced: false,
        charge_time: ReferenceChargeTime::Slow,
    };

    /// The register value (§5.3.29).
    const fn bits(self) -> u8 {
        let force = if self.forced {
            reference_power_up::FORCE
        } else {
            0
        };
        force | self.charge_time.bits()
    }

    /// The reference setting a value of register 123 describes: the inverse of `bits` (§5.3.29).
    pub(crate) const fn from_bits(value: u8) -> Self {
        Self {
            forced: value & reference_power_up::FORCE != 0,
            charge_time: ReferenceChargeTime::from_bits(value),
        }
    }
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Programs the supply settings: registers 1, 2 and 10 of page 1, in that order, each written
    /// whole.
    ///
    /// The order is the guide's (§2.4.6, step 2) and the kernel driver's (`aic32x4.c:1138-1150`):
    /// the weak AVDD connection, then the LDO and the analog blocks, then the common modes. By this
    /// point the analog supply has to be present (§2.4.6).
    ///
    /// The guide's pop-free sequence chooses the headphone drivers' common mode and supply before
    /// anything is routed to them or powered (§2.2.2.1, step 2), so call this before
    /// [`power_up`](Self::power_up).
    pub fn configure_supply(&mut self, supply: &SupplyConfig) -> Result<(), Error<I2C::Error>> {
        self.write(register::POWER_CONFIG, supply.power_config_bits())?;
        self.write(register::LDO_CONTROL, supply.ldo_control_bits())?;
        self.write(register::COMMON_MODE, supply.common_mode_bits())
    }

    /// Sets how the analog reference powers up: page 1, register 123, written whole (§5.3.29).
    ///
    /// Without [`ReferencePowerUp::forced`] this powers nothing: the reference starts with the
    /// first analog block that is powered, and until it is up the chip queues further power-up
    /// requests by itself (§3.3). So no wait is needed after this call.
    pub fn set_reference_power_up(
        &mut self,
        reference: ReferencePowerUp,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(register::REFERENCE_POWER_UP, reference.bits())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins the write sequence of the supply settings, with values that differ from the kernel
    /// driver's so that every field shows: an external AVDD of less than 1.8 V and drivers on a low LDOIN.
    #[test]
    fn configure_supply_writes_registers_1_2_10_of_page_1() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 1]),
            // Weak connection off.
            Transaction::write(ADDRESS, vec![1, 0x08]),
            // LDO off but set to 1.77 V, analog blocks enabled.
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![2, 0x20]),
            // 0.75 V chip common mode, 1.25 V at the drivers, drivers on LDOIN, low range.
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![10, 0x52]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        let supply = SupplyConfig {
            avdd_weakly_connected_to_dvdd: false,
            avdd_ldo_powered: false,
            avdd_ldo_voltage: LdoVoltage::V1_77,
            analog_blocks_enabled: true,
            common_mode: CommonMode::V0_75,
            headphone_common_mode: HeadphoneCommonMode::V1_25,
            headphone_supply: HeadphoneSupply::LdoinLowRange,
        };
        dac.configure_supply(&supply).unwrap();
        dac.release().done();
    }

    /// Pins the reference register write: forced on, 120 ms.
    #[test]
    fn set_reference_power_up_writes_register_123_of_page_1() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 1]),
            Transaction::write(ADDRESS, vec![123, 0x07]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_reference_power_up(ReferencePowerUp {
            forced: true,
            charge_time: ReferenceChargeTime::Ms120,
        })
        .unwrap();
        dac.release().done();
    }
}

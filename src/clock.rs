//! The clock tree: where the codec clock comes from, the PLL, and the dividers down to the sample
//! rate (§2.7; the whole tree is drawn in Figure 2-45).
//!
//! The sample rate is `CODEC_CLKIN / (NDAC * MDAC * DOSR)` (Equation 15). CODEC_CLKIN is the MCLK
//! pin, the BCLK pin, the GPIO pin or the PLL's output, and the PLL multiplies its own input by
//! `R * J.D / P` (Equation 17). A chip that drives the bit clock derives it from DAC_CLK or
//! DAC_MOD_CLK through the BCLK N divider (Figure 2-46).
//!
//! Every value type here can only hold what its register field documents as usable, so
//! [`Tlv320dac3203::configure_clocks`] cannot be handed a value the chip has no encoding for.
//! What the types do not check is whether the resulting frequencies are in range, because that
//! depends on the input clock and the supply voltages, which the driver does not know:
//!
//! - with D = 0 the PLL needs 512 kHz <= PLL_CLKIN / P <= 20 MHz, with any other D 10 MHz <=
//!   PLL_CLKIN / P <= 20 MHz (Equations 18 and 19);
//! - the PLL's output must lie in the range Table 2-22 gives for the analog supply and the selected
//!   [`PllRange`], 80 MHz to 103 MHz at the least;
//! - `DOSR * sample rate` must lie between 2.8 MHz and 6.2 MHz (§2.4.6);
//! - `MDAC * DOSR / 32` must be at least the resource class of the selected processing block
//!   (§2.4.6; the classes are in Table 2-13);
//! - CODEC_CLKIN, DAC_CLK and DAC_MOD_CLK have upper limits that depend on the digital supply
//!   (Table 2-21);
//! - DOSR has to be a multiple of 2, 4 or 8 depending on the interpolation filter. The guide
//!   states the rule both ways round ([`crate::register::DOSR_MSB`]); a multiple of 8 satisfies
//!   both.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::register::{self, clock_mux, divider, dosr_msb, pll_d_msb, pll_j, pll_p_r};
use crate::{Error, Tlv320dac3203};

/// How long [`Tlv320dac3203::configure_clocks`] waits after the last PLL register when the
/// configuration powers the PLL, in milliseconds.
///
/// The guide gives one figure: "a startup delay of approximately 10ms is involved after the power
/// up command of the PLL and before the clocks are available to the codec" (§3.4; "typically after
/// 10ms", §2.7.1). That figure is used as it stands. The kernel driver does not wait at all between
/// the PLL and the dividers (`aic32x4.c:878-898`), and TI's own example script writes them back to
/// back (§4.7), so nothing depends on this wait being exact.
///
/// It does not cover the analog reference. After a reset the first request to power an analog
/// block, the PLL included, is queued until the reference is up (§3.3), which takes the time set
/// with [`Tlv320dac3203::set_reference_power_up`]. The power-up sequence does not rely on the
/// clocks before it has polled the headphone drivers' flags, which come later than both.
pub const PLL_STARTUP_MS: u32 = 10;

/// Which output range the PLL runs in: page 0, register 4, bit D6 (§5.2.5). The frequencies each
/// range allows depend on the analog supply (Table 2-22).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PllRange {
    /// The low range, the reset value: 80 MHz up to 103 MHz, 118 MHz or 132 MHz for an AVDD of at
    /// least 1.5 V, 1.65 V or 1.8 V (Table 2-22).
    Low,
    /// The high range: 95 MHz to 110 MHz, 92 MHz to 123 MHz or 92 MHz to 137 MHz for the same
    /// three supplies (Table 2-22).
    High,
}

impl PllRange {
    /// The bit, in place (§5.2.5).
    const fn bits(self) -> u8 {
        match self {
            PllRange::Low => 0,
            PllRange::High => clock_mux::PLL_RANGE_HIGH,
        }
    }

    /// The range a value of register 4 selects: the inverse of `bits` (§5.2.5).
    pub(crate) const fn from_bits(value: u8) -> Self {
        if value & clock_mux::PLL_RANGE_HIGH != 0 {
            PllRange::High
        } else {
            PllRange::Low
        }
    }
}

/// Where the PLL takes its input from, PLL_CLKIN: page 0, register 4, bits D3 to D2 (§5.2.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PllInput {
    /// The MCLK pin, the reset value.
    Mclk,
    /// The BCLK pin.
    Bclk,
    /// The GPIO pin, which only the WCSP package has.
    Gpio,
    /// The DIN pin.
    Din,
}

impl PllInput {
    /// The field's bits, in place (§5.2.5).
    const fn bits(self) -> u8 {
        match self {
            PllInput::Mclk => clock_mux::PLL_CLKIN_MCLK,
            PllInput::Bclk => clock_mux::PLL_CLKIN_BCLK,
            PllInput::Gpio => clock_mux::PLL_CLKIN_GPIO,
            PllInput::Din => clock_mux::PLL_CLKIN_DIN,
        }
    }

    /// The input a value of register 4 selects: the inverse of `bits`. The two-bit field has no
    /// reserved code (§5.2.5).
    pub(crate) const fn from_bits(value: u8) -> Self {
        match value & clock_mux::PLL_CLKIN_MASK {
            clock_mux::PLL_CLKIN_MCLK => PllInput::Mclk,
            clock_mux::PLL_CLKIN_BCLK => PllInput::Bclk,
            clock_mux::PLL_CLKIN_GPIO => PllInput::Gpio,
            _ => PllInput::Din,
        }
    }
}

/// Where the codec takes its clock from, CODEC_CLKIN, the clock NDAC divides: page 0, register 4,
/// bits D1 to D0 (§5.2.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CodecInput {
    /// The MCLK pin, the reset value.
    Mclk,
    /// The BCLK pin.
    Bclk,
    /// The GPIO pin, which only the WCSP package has.
    Gpio,
    /// The PLL's output. The PLL then has to be powered up first and powered down last (§2.7.1).
    Pll,
}

impl CodecInput {
    /// The field's bits, in place (§5.2.5).
    const fn bits(self) -> u8 {
        match self {
            CodecInput::Mclk => clock_mux::CODEC_CLKIN_MCLK,
            CodecInput::Bclk => clock_mux::CODEC_CLKIN_BCLK,
            CodecInput::Gpio => clock_mux::CODEC_CLKIN_GPIO,
            CodecInput::Pll => clock_mux::CODEC_CLKIN_PLL,
        }
    }

    /// The input a value of register 4 selects: the inverse of `bits`. The two-bit field has no
    /// reserved code (§5.2.5).
    pub(crate) const fn from_bits(value: u8) -> Self {
        match value & clock_mux::CODEC_CLKIN_MASK {
            clock_mux::CODEC_CLKIN_MCLK => CodecInput::Mclk,
            clock_mux::CODEC_CLKIN_BCLK => CodecInput::Bclk,
            clock_mux::CODEC_CLKIN_GPIO => CodecInput::Gpio,
            _ => CodecInput::Pll,
        }
    }
}

/// The PLL's input divider P, 1 to 8 (§5.2.6; §2.7.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PllP(u8);

impl PllP {
    /// P = 1, the reset value (§5.2.6).
    pub const RESET: Self = Self(1);

    /// `None` unless `p` is 1 to 8, the values the three-bit field encodes (§5.2.6).
    pub const fn new(p: u8) -> Option<Self> {
        if p >= 1 && p <= 8 {
            Some(Self(p))
        } else {
            None
        }
    }

    /// The divider, 1 to 8.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// The field's bits, in place: 1 to 7 are themselves and 8 is written as 0 (§5.2.6).
    const fn bits(self) -> u8 {
        ((self.0 & 0b111) << pll_p_r::P_SHIFT) & pll_p_r::P_MASK
    }

    /// The divider a value of register 5 holds: the inverse of `bits`, so the code 0 is 8. Every
    /// code of the three-bit field is a divider (§5.2.6).
    pub(crate) const fn from_bits(value: u8) -> Self {
        let code = (value & pll_p_r::P_MASK) >> pll_p_r::P_SHIFT;
        Self(if code == 0 { 8 } else { code })
    }
}

/// The PLL's multiplier R, 1 to 4 (§5.2.6; "R = 1, 2, 3, 4", §2.7.1).
///
/// The field is four bits wide, but every other code is "reserved, do not use". This is narrower
/// than on the codecs the kernel driver was written for, which is why the type exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PllR(u8);

impl PllR {
    /// R = 1, the reset value (§5.2.6).
    pub const RESET: Self = Self(1);

    /// `None` unless `r` is 1 to 4 (§5.2.6).
    pub const fn new(r: u8) -> Option<Self> {
        if r >= 1 && r <= 4 {
            Some(Self(r))
        } else {
            None
        }
    }

    /// The multiplier, 1 to 4.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// The field's bits (§5.2.6).
    const fn bits(self) -> u8 {
        self.0 & pll_p_r::R_MASK
    }

    /// The multiplier a value of register 5 holds, or `None` for the codes 0 and 5 to 15, which are
    /// "reserved, do not use" (§5.2.6).
    pub(crate) const fn from_bits(value: u8) -> Option<Self> {
        Self::new(value & pll_p_r::R_MASK)
    }
}

/// The PLL's multiplier J, the integer part of K = J.D: 4 to 63 (§5.2.7).
///
/// The PLL section gives the range as 1 to 63 (§2.7.1), but the register description marks 0 to 3
/// "do not use". The stricter of the two is enforced here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PllJ(u8);

impl PllJ {
    /// J = 4, the reset value (§5.2.7).
    pub const RESET: Self = Self(4);

    /// `None` unless `j` is 4 to 63 (§5.2.7).
    pub const fn new(j: u8) -> Option<Self> {
        if j >= 4 && j <= 63 {
            Some(Self(j))
        } else {
            None
        }
    }

    /// The multiplier, 4 to 63.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// The field's bits (§5.2.7).
    const fn bits(self) -> u8 {
        self.0 & pll_j::J_MASK
    }

    /// The multiplier a value of register 6 holds, or `None` for the codes 0 to 3, which are "do
    /// not use" (§5.2.7).
    pub(crate) const fn from_bits(value: u8) -> Option<Self> {
        Self::new(value & pll_j::J_MASK)
    }
}

/// The PLL's fractional multiplier D, the four decimal digits after the point of K = J.D: 0 to 9999
/// (§5.2.8; §2.7.1). K = 7.168 is J = 7 and D = 1680.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PllD(u16);

impl PllD {
    /// D = 0, the reset value (§5.2.8 and §5.2.9): an integer multiplier.
    pub const RESET: Self = Self(0);

    /// `None` unless `d` is 0 to 9999; larger values are "do not use" (§5.2.8).
    pub const fn new(d: u16) -> Option<Self> {
        if d <= 9999 { Some(Self(d)) } else { None }
    }

    /// The fraction in ten-thousandths, 0 to 9999.
    pub const fn get(self) -> u16 {
        self.0
    }

    /// The values of registers 7 and 8: D\[13:8\] and D\[7:0\] (§5.2.8 and §5.2.9).
    const fn bytes(self) -> [u8; 2] {
        let [high, low] = self.0.to_be_bytes();
        [high & pll_d_msb::D_HIGH_MASK, low]
    }

    /// The fraction the values of registers 7 and 8 hold: the inverse of `bytes`. `None` for a
    /// number above 9999, which the 14 bits can hold but the guide marks "do not use" (§5.2.8).
    pub(crate) const fn from_bytes(high: u8, low: u8) -> Option<Self> {
        Self::new(u16::from_be_bytes([high & pll_d_msb::D_HIGH_MASK, low]))
    }
}

/// The PLL: its power bit and the four values of `PLL_CLKIN * R * J.D / P` (Equation 17).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pll {
    /// Whether the PLL is powered: page 0, register 5, bit D7 (§5.2.6).
    pub powered: bool,
    /// The input divider.
    pub p: PllP,
    /// The multiplier R.
    pub r: PllR,
    /// The integer part of the multiplier K.
    pub j: PllJ,
    /// The fractional part of the multiplier K.
    pub d: PllD,
}

impl Pll {
    /// The PLL after a reset: powered down, P = 1, R = 1, J = 4, D = 0 (§5.2.6 to §5.2.9).
    pub const RESET: Self = Self {
        powered: false,
        p: PllP::RESET,
        r: PllR::RESET,
        j: PllJ::RESET,
        d: PllD::RESET,
    };

    /// The value of register 5: power, P and R (§5.2.6).
    const fn p_r_bits(self) -> u8 {
        let power = if self.powered { pll_p_r::POWER_UP } else { 0 };
        power | self.p.bits() | self.r.bits()
    }

    /// Whether a value of register 5 has the PLL powered: bit D7 (§5.2.6).
    pub(crate) const fn powered_from_bits(value: u8) -> bool {
        value & pll_p_r::POWER_UP != 0
    }
}

/// What the NDAC, MDAC and BCLK N dividers divide by: 1 to 128 (§5.2.11, §5.2.12 and §5.2.30 to
/// 90).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Divisor(u8);

impl Divisor {
    /// Divide by 1, the reset value of all three dividers (§5.2.11, §5.2.12 and §5.2.30).
    pub const RESET: Self = Self(1);

    /// `None` unless `divisor` is 1 to 128, the values the seven-bit field encodes (§5.2.11,
    /// §5.2.12 and §5.2.30).
    pub const fn new(divisor: u8) -> Option<Self> {
        if divisor >= 1 && divisor <= 128 {
            Some(Self(divisor))
        } else {
            None
        }
    }

    /// The divisor, 1 to 128.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// The field's bits: 1 to 127 are themselves and 128 is written as 0 (§5.2.11, §5.2.12 and
    /// §5.2.30).
    const fn bits(self) -> u8 {
        self.0 & divider::VALUE_MASK
    }

    /// The divisor a divider register holds: the inverse of `bits`, so the code 0 is 128. Every
    /// code of the seven-bit field is a divisor (§5.2.11, §5.2.12 and §5.2.30).
    const fn from_bits(value: u8) -> Self {
        let code = value & divider::VALUE_MASK;
        Self(if code == 0 { 128 } else { code })
    }
}

/// One of the NDAC, MDAC and BCLK N dividers: its power bit and its divisor, which share a register
/// (§5.2.11, §5.2.12 and §5.2.30 to 90).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DividerConfig {
    /// Whether the divider is powered: bit D7. A divider that is powered down passes no clock on.
    pub powered: bool,
    /// What it divides by.
    pub divisor: Divisor,
}

impl DividerConfig {
    /// A divider after a reset: powered down, dividing by 1 (§5.2.11, §5.2.12 and §5.2.30).
    pub const RESET: Self = Self {
        powered: false,
        divisor: Divisor::RESET,
    };

    /// The register value (§5.2.11, §5.2.12 and §5.2.30).
    const fn bits(self) -> u8 {
        let power = if self.powered { divider::POWER_UP } else { 0 };
        power | self.divisor.bits()
    }

    /// The divider a register value describes: the inverse of `bits` (§5.2.11, §5.2.12 and
    /// §5.2.30).
    pub(crate) const fn from_bits(value: u8) -> Self {
        Self {
            powered: value & divider::POWER_UP != 0,
            divisor: Divisor::from_bits(value),
        }
    }
}

/// The oversampling ratio DOSR, from DAC_MOD_CLK to the sample rate: 2 to 1022, or 1024 (§5.2.13).
///
/// 1 and 1023 have encodings but are "reserved, do not use". Which multiples the interpolation
/// filters need is not checked, for the reason given at [`crate::register::DOSR_MSB`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Dosr(u16);

impl Dosr {
    /// DOSR = 128, the reset value (§5.2.13 and §5.2.14).
    pub const RESET: Self = Self(128);

    /// `None` unless `dosr` is 2 to 1022, or 1024 (§5.2.13).
    pub const fn new(dosr: u16) -> Option<Self> {
        if (dosr >= 2 && dosr <= 1022) || dosr == 1024 {
            Some(Self(dosr))
        } else {
            None
        }
    }

    /// The ratio: 2 to 1022, or 1024.
    pub const fn get(self) -> u16 {
        self.0
    }

    /// The values of registers 13 and 14: DOSR\[9:8\] and DOSR\[7:0\]. 2 to 1022 are themselves and
    /// 1024 is written as 0 (§5.2.13).
    const fn bytes(self) -> [u8; 2] {
        let [high, low] = self.0.to_be_bytes();
        [high & dosr_msb::DOSR_HIGH_MASK, low]
    }

    /// The ratio the values of registers 13 and 14 hold: the inverse of `bytes`, so the code 0 is
    /// 1024. `None` for the codes 1 and 1023, which are "reserved, do not use" (§5.2.13).
    pub(crate) const fn from_bytes(high: u8, low: u8) -> Option<Self> {
        let code = u16::from_be_bytes([high & dosr_msb::DOSR_HIGH_MASK, low]);
        Self::new(if code == 0 { 1024 } else { code })
    }
}

/// The whole clock tree: registers 4 to 8, 11 to 14 and 30 of page 0.
///
/// The fields mirror the registers, power bits included, so a configuration can describe an
/// intermediate state too.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockConfig {
    /// The PLL's output range.
    pub pll_range: PllRange,
    /// The PLL's input.
    pub pll_input: PllInput,
    /// The codec's input, the clock NDAC divides.
    pub codec_input: CodecInput,
    /// The PLL. Its values are written even when it stays powered down.
    pub pll: Pll,
    /// NDAC, from CODEC_CLKIN to DAC_CLK.
    pub ndac: DividerConfig,
    /// MDAC, from DAC_CLK to DAC_MOD_CLK.
    pub mdac: DividerConfig,
    /// DOSR, from DAC_MOD_CLK to the sample rate.
    pub dosr: Dosr,
    /// BCLK N, from DAC_CLK or DAC_MOD_CLK ([`crate::InterfaceConfig::bclk_divider_input`]) to
    /// the BCLK pin. It only matters on a chip that drives BCLK.
    pub bclk_n: DividerConfig,
}

impl ClockConfig {
    /// The clock tree after a reset: everything from the MCLK pin, the PLL and all dividers powered
    /// down, DOSR 128 (§5.2.5 to §5.2.14 and §5.2.30).
    pub const RESET: Self = Self {
        pll_range: PllRange::Low,
        pll_input: PllInput::Mclk,
        codec_input: CodecInput::Mclk,
        pll: Pll::RESET,
        ndac: DividerConfig::RESET,
        mdac: DividerConfig::RESET,
        dosr: Dosr::RESET,
        bclk_n: DividerConfig::RESET,
    };
}

/// Names one of the three dividers for [`Tlv320dac3203::set_divider_power`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Divider {
    /// NDAC: page 0, register 11 (§5.2.11).
    Ndac,
    /// MDAC: page 0, register 12 (§5.2.12).
    Mdac,
    /// BCLK N: page 0, register 30 (§5.2.30).
    BclkN,
}

impl Divider {
    /// The divider's register.
    const fn register(self) -> crate::Register {
        match self {
            Divider::Ndac => register::NDAC,
            Divider::Mdac => register::MDAC,
            Divider::BclkN => register::BCLK_N,
        }
    }
}

// The ranges above are what keeps a reserved code out of the chip. These assertions fail the build
// if one of them drifts from its register description.
const _: () = {
    assert!(PllP::new(0).is_none() && PllP::new(8).is_some() && PllP::new(9).is_none());
    assert!(PllR::new(0).is_none() && PllR::new(4).is_some() && PllR::new(5).is_none());
    assert!(PllJ::new(3).is_none() && PllJ::new(4).is_some() && PllJ::new(64).is_none());
    assert!(PllD::new(9999).is_some() && PllD::new(10_000).is_none());
    assert!(
        Divisor::new(0).is_none() && Divisor::new(128).is_some() && Divisor::new(129).is_none()
    );
    assert!(Dosr::new(1).is_none() && Dosr::new(2).is_some() && Dosr::new(1022).is_some());
    assert!(Dosr::new(1023).is_none() && Dosr::new(1024).is_some() && Dosr::new(1025).is_none());
};

// The decoders have to undo the three encodings that write a value as 0: a P of 8, a divisor of
// 128 and a DOSR of 1024. These assertions fail the build if one of them drifts from its encoder.
const _: () = {
    assert!(PllP::from_bits(PllP(8).bits()).get() == 8);
    assert!(PllP::from_bits(PllP(5).bits()).get() == 5);
    assert!(Divisor::from_bits(Divisor(128).bits()).get() == 128);
    assert!(Divisor::from_bits(Divisor(127).bits()).get() == 127);
    let [high, low] = Dosr(1024).bytes();
    assert!(matches!(Dosr::from_bytes(high, low), Some(dosr) if dosr.get() == 1024));
    let [high, low] = Dosr(128).bytes();
    assert!(matches!(Dosr::from_bytes(high, low), Some(dosr) if dosr.get() == 128));
    assert!(Dosr::from_bytes(0, 1).is_none() && Dosr::from_bytes(0x03, 0xFF).is_none());
    let [high, low] = PllD(9999).bytes();
    assert!(matches!(PllD::from_bytes(high, low), Some(d) if d.get() == 9999));
    assert!(PllD::from_bytes(0x27, 0x10).is_none());
};

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Programs the clock tree: registers 4 to 8, 11 to 14 and 30 of page 0, in that order, each
    /// written whole.
    ///
    /// This is the order of the kernel driver
    /// (`aic32x4.c:876-920`) and of the guide's setup list: PLL, then NDAC, MDAC and the oversampling ratio (§2.4.6, step 2). Register 5 carries the PLL's power bit together with P and R, so a configuration that powers the PLL starts it before J and D are written; the kernel driver does that, and so does TI's example script (§4.7). If the configuration powers the PLL, the method then waits [`PLL_STARTUP_MS`]
    /// before it touches the dividers, counted from the last PLL register.
    ///
    /// Meant for a chip whose DAC channels are powered down: call it after
    /// [`reset`](Self::reset) or [`power_down`](Self::power_down), or let
    /// [`power_up`](Self::power_up) call it.
    pub fn configure_clocks<D: DelayNs>(
        &mut self,
        clocks: &ClockConfig,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(
            register::CLOCK_MUX,
            clocks.pll_range.bits() | clocks.pll_input.bits() | clocks.codec_input.bits(),
        )?;
        self.write(register::PLL_P_R, clocks.pll.p_r_bits())?;
        self.write(register::PLL_J, clocks.pll.j.bits())?;
        // D is a 14-bit number split over two registers. The chip only takes the high part when the
        // low part is written immediately after it (§5.2.8 note; §2.7.1), so nothing may come
        // between these two writes.
        let [d_high, d_low] = clocks.pll.d.bytes();
        self.write(register::PLL_D_MSB, d_high)?;
        self.write(register::PLL_D_LSB, d_low)?;
        if clocks.pll.powered {
            delay.delay_ms(PLL_STARTUP_MS);
        }
        self.write(register::NDAC, clocks.ndac.bits())?;
        self.write(register::MDAC, clocks.mdac.bits())?;
        // DOSR is split the same way and has the same rule: register 14 immediately after register
        // 13 (§5.2.13 note).
        let [dosr_high, dosr_low] = clocks.dosr.bytes();
        self.write(register::DOSR_MSB, dosr_high)?;
        self.write(register::DOSR_LSB, dosr_low)?;
        self.write(register::BCLK_N, clocks.bclk_n.bits())
    }

    /// Powers the PLL up or down and leaves P and R as they are: page 0, register 5, bit D7
    /// (§5.2.6).
    ///
    /// A PLL that feeds CODEC_CLKIN has to be powered up first and powered down last (§2.7.1). This
    /// method does not wait; after powering up, the clock needs [`PLL_STARTUP_MS`].
    ///
    /// It reads the register and writes it back with the power bit changed, so it needs a chip
    /// this driver owns.
    pub fn set_pll_power(&mut self, powered: bool) -> Result<(), Error<I2C::Error>> {
        let power = if powered { pll_p_r::POWER_UP } else { 0 };
        self.modify(register::PLL_P_R, pll_p_r::POWER_UP, power)
    }

    /// Powers one divider up or down and leaves its divisor as it is: bit D7 of its register
    /// (§5.2.11, §5.2.12 and §5.2.30).
    ///
    /// NDAC and MDAC must stay powered until the DAC channels report powered down, and then MDAC
    /// goes first and NDAC after it (§2.7). [`power_down`](Self::power_down) keeps to that.
    ///
    /// It reads the register and writes it back with the power bit changed, so it needs a chip
    /// this driver owns.
    pub fn set_divider_power(
        &mut self,
        divider: Divider,
        powered: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let power = if powered { divider::POWER_UP } else { 0 };
        self.modify(divider.register(), divider::POWER_UP, power)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;
    use crate::test_support::{BusDelay, wait_ms};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins the write sequence of the clock tree with values that show every encoding rule: the
    /// high PLL range, a P of 8 and divisors of 128 written as 0, D and DOSR split over two
    /// registers, DOSR 1024 written as 0, and the wait between the PLL and the dividers.
    #[test]
    fn configure_clocks_writes_mux_pll_wait_dividers() {
        let write = |register: u8, value: u8| Transaction::write(ADDRESS, vec![register, value]);
        let expectations = [
            write(0, 0),
            // High range, PLL from BCLK, codec clock from the PLL.
            write(4, 0x47),
            // Powered, P = 8 (written as 0), R = 4.
            write(0, 0),
            write(5, 0x84),
            // J = 7.
            write(0, 0),
            write(6, 0x07),
            // D = 1680 = 0x0690.
            write(0, 0),
            write(7, 0x06),
            write(0, 0),
            write(8, 0x90),
            wait_ms(10),
            // NDAC = 128 (written as 0), powered.
            write(0, 0),
            write(11, 0x80),
            // MDAC = 7, powered.
            write(0, 0),
            write(12, 0x87),
            // DOSR = 1024 (written as 0).
            write(0, 0),
            write(13, 0x00),
            write(0, 0),
            write(14, 0x00),
            // BCLK N = 4, not powered.
            write(0, 0),
            write(30, 0x04),
        ];
        let mut bus = Mock::new(&expectations);
        let mut delay = BusDelay(bus.clone());
        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        let clocks = ClockConfig {
            pll_range: PllRange::High,
            pll_input: PllInput::Bclk,
            codec_input: CodecInput::Pll,
            pll: Pll {
                powered: true,
                p: PllP::new(8).unwrap(),
                r: PllR::new(4).unwrap(),
                j: PllJ::new(7).unwrap(),
                d: PllD::new(1680).unwrap(),
            },
            ndac: DividerConfig {
                powered: true,
                divisor: Divisor::new(128).unwrap(),
            },
            mdac: DividerConfig {
                powered: true,
                divisor: Divisor::new(7).unwrap(),
            },
            dosr: Dosr::new(1024).unwrap(),
            bclk_n: DividerConfig {
                powered: false,
                divisor: Divisor::new(4).unwrap(),
            },
        };
        dac.configure_clocks(&clocks, &mut delay).unwrap();
        bus.done();
    }

    /// Pins the read-modify-write of the two power switches: only bit 7 changes.
    #[test]
    fn pll_and_divider_power_change_only_the_power_bit() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            // The PLL register holds P = 5, R = 1, powered: power it down.
            Transaction::write_read(ADDRESS, vec![5], vec![0xD1]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![5, 0x51]),
            // MDAC = 3, not powered: power it up.
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write_read(ADDRESS, vec![12], vec![0x03]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![12, 0x83]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_pll_power(false).unwrap();
        dac.set_divider_power(Divider::Mdac, true).unwrap();
        dac.release().done();
    }
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Programs the BCLK N divider alone: page 0, register 30 written whole, power bit and divisor
    /// (§5.2.30).
    /// [`configure_clocks`](Self::configure_clocks) writes the same register as its last step; this is for putting it back on its own, for example to stop a bit clock the chip generates (`0x01`,
    /// the reset value) without rewriting the other dividers while the DAC channels still run.
    pub fn set_bclk_divider(&mut self, bclk_n: DividerConfig) -> Result<(), Error<I2C::Error>> {
        self.write(register::BCLK_N, bclk_n.bits())
    }
}

#[cfg(test)]
mod bclk_divider_tests {
    use super::*;
    use crate::ADDRESS;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins register 30 alone: powered with N = 4 (`0x84`), then back to the reset value `0x01`.
    #[test]
    fn set_bclk_divider_writes_register_30_whole() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![30, 0x84]),
            Transaction::write(ADDRESS, vec![0, 0]),
            Transaction::write(ADDRESS, vec![30, 0x01]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.set_bclk_divider(DividerConfig {
            powered: true,
            divisor: Divisor::new(4).unwrap(),
        })
        .unwrap();
        dac.set_bclk_divider(DividerConfig::RESET).unwrap();
        dac.release().done();
    }
}

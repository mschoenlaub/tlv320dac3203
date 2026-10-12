//! The register map: addresses and bit fields, as far as this driver covers the chip so far.
//!
//! # How the constants are cited
//!
//! Bit meanings come from TI's *TLV320DAC3203 Applications Reference Guide*, SLAU434A (May 2012,
//! revised February 2019); "§N", "Table N" and "p. N" below always mean that document. Its
//! chapter 5 describes every register in a numbered section of its own that holds one unnumbered
//! bit table, so a register is cited by that section ("§5.2.5"), once, in the register's own
//! documentation; its bits cite only sources beyond that section. The guide's numbered tables
//! ("Table 2-13") are cited where a value comes from one. The datasheet, SLOS756B (May 2012,
//! revised December 2018), only has the list of register names (Table 4). It is cited where it
//! disagrees with the guide.
//!
//! Addresses and values that were cross-checked against the kernel driver the stock system binds
//! cite it as well: `aic32x4.h:N` and `aic32x4.c:N` are lines of
//! `sound/soc/codecs/tlv320aic32x4.h` and `.c` in `gitlab.com/echo-pmos/amazon-biscuit-kernel` at
//! commit `3923647` (Linux 3.18). That is read in source, not verified against the kernel
//! running on the Dot. A register without such a citation is one the kernel driver never names.
//!
//! Register numbers are written in decimal because the guide and the kernel header number them
//! that way.
//!
//! Multi-bit field values are already shifted into place, so they can be OR-ed together and used
//! with their `*_MASK` directly. Numeric fields (dividers, gains, codes) come as a mask plus a
//! shift where the field does not start at bit 0.
//!
//! # What is left out on purpose
//!
//! The map holds what this driver's power-up, clocking, routing, volume, beep and status code
//! needs, which covers everything the kernel driver programs for playback. The rest are gaps of
//! this driver so far, not things the chip lacks:
//!
//! - **The coefficient registers.** Only the pages are named ([`page`]). The biquad, first-order
//!   IIR and DRC filter coefficients in them stay at their reset values, which are flat; the driver
//!   has no coefficient writes yet. The adaptive-filter control register in page 44 (register 1,
//!   §5.10.2) is left out with them.
//! - **The digital microphone path**: page 0, registers 18 to 20 (NADC, MADC, AOSR) and 81 to 85
//!   (channel power, volume, phase), and the microphone bias in page 1, register 51.
//! - **Further pins and interfaces**: the CLKOUT multiplexer and divider (page
//!   0, registers 25 and 26), the secondary audio interface (31 to 33), the general-call enable
//!   (34), the interrupt controls (48 and 49), the GPIO/MFP5, DIN/MFP1 and MISO/MFP4 pin controls
//!   (52, 54 and 55).
//! - **Headset detection** (page 0, register 67). (The beep generator, registers 71 to 79, is at the end of this map, for
//!   `crate::beep`, feature `beep`.)
//! - **The analog bypass and protection settings**: over-current protection (page 1, register 11),
//!   the IN1L/IN1R bypass volumes (22 and 23), the analog input settings (58 and 71) and the
//!   offset calibration (125).
//! - **Registers the kernel driver writes although this chip does not have them** (page 1,
//!   registers 14, 15, 18, 19, 52 to 57, 59 and 60, and more). The crate documentation lists them
//!   under "Where the DAC3203 differs from what `tlv320aic32x4` assumes".

/// One register: the page it lives in and its address inside that page.
///
/// The chip has 8-bit addresses 0 to 127 in each page and selects the page through register 0,
/// which exists in every page (§5.2.1 and §5.3.1). The pages that exist are listed in
/// [`page`]; every other page is reserved (Table 5-1). This type does not enforce that, so a `Register`
/// built by hand is the caller's responsibility.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Register {
    page: u8,
    addr: u8,
}

impl Register {
    /// A register at `addr` in `page`.
    pub const fn new(page: u8, addr: u8) -> Self {
        Self { page, addr }
    }

    /// The page this register lives in.
    pub const fn page(self) -> u8 {
        self.page
    }

    /// The address inside the page.
    pub const fn addr(self) -> u8 {
        self.addr
    }

    /// Whether reading this register changes it (see [`CLEAR_ON_READ`]).
    pub const fn is_clear_on_read(self) -> bool {
        let mut i = 0;
        while i < CLEAR_ON_READ.len() {
            if CLEAR_ON_READ[i].page == self.page && CLEAR_ON_READ[i].addr == self.addr {
                return true;
            }
            i += 1;
        }
        false
    }
}

/// The page-select register. It sits at address 0 of every page, so it is an address and not a
/// [`Register`]: writing `N` to it selects page `N` for the next read or write, whatever page was
/// selected before (§5.2.1; described again for page 1 in §5.3.1, and for every coefficient page).
/// It resets to page 0, "the default home page after hardware reset" (chapter 5 introduction).
///
/// The kernel driver names it `AIC32X4_PSEL` (`aic32x4.h:21`) and hands it to regmap as the
/// selector of a 128-register window (`aic32x4.c:731-740`).
pub const PAGE_SELECT: u8 = 0;

/// The pages that exist (Table 5-1).
///
/// Pages 0 and 1 hold the control registers and are the only ones this map describes. Pages 8 to
/// 10 and 26 to 28 hold the coefficients of the digital microphone path, which this driver does
/// not cover. Everything else is reserved.
pub mod page {
    use core::ops::RangeInclusive;

    /// Page 0: serial interface, digital pins, clocking and the DAC's digital configuration (Table
    /// 5-1; §5.2).
    pub const DIGITAL: u8 = 0;
    /// Page 1: power supply, output drivers, routing and analog gain (Table 5-1; §5.3). The kernel
    /// header calls it `AIC32X4_PAGE1` (`aic32x4.h:16`).
    pub const ANALOG: u8 = 1;
    /// Pages 44 to 46: DAC coefficient buffer A, coefficients C0 to C76 (Table 5-1; Table 5-5).
    /// These are the coefficients the DAC runs on unless adaptive filtering has switched buffers.
    ///
    /// Each coefficient is 24 bits in three consecutive registers followed by a reserved one, from
    /// register 8 of each page on (Table 5-5). The left channel's biquads A to F are C1 to C30, the
    /// right channel's are C33 to C62 (Table 2-15), the first-order IIR filters are C65 to C70
    /// (Table 2-14) and the DRC's high-pass and low-pass filters are C71 to C76 (Table 2-19). Reset
    /// values are in Table 5-7.
    ///
    /// While the DAC is running these registers are locked, for reading and for writing, unless
    /// adaptive filtering is on (§2.4.5.3; §5.10.4). They must not be touched during the first
    /// millisecond after a reset either (§3.2).
    ///
    /// The kernel driver reaches exactly these three pages
    /// (`aic32x4.h:17-19`; its register range ends at page 46, `aic32x4.h:99`). The datasheet's
    /// summary gives a longer range, pages 44 to 52 with coefficients up to C255 (SLOS756B Table
    /// 4); the guide's coefficient tables end at page 46 and C76.
    pub const DAC_COEFFICIENTS_A: RangeInclusive<u8> = 44..=46;
    /// Pages 62 to 64: DAC coefficient buffer B, the second copy that adaptive filtering swaps in
    /// (Table 5-1; Table 5-6; §2.4.5.3). Laid out like buffer A. The kernel driver cannot reach
    /// these pages (`aic32x4.h:99`) and never turns adaptive filtering on.
    pub const DAC_COEFFICIENTS_B: RangeInclusive<u8> = 62..=64;
}

/// Every register with sticky flag bits: a read returns them and clears them, so the next reader
/// no longer sees the event. No method of the driver reads one of these.
///
/// - Page 0, register 42, "Sticky Flag Register 1": the DAC overflow flags, each of which "will
///   self clear on read" (§5.2.39).
/// - Page 0, register 44, "Sticky Flag Register 2": headphone over-current, headset button and
///   insertion, DRC threshold; each "will be cleared when the register is read" (§5.2.41).
/// - Page 0, register 45. The guide calls it reserved (§5.2.42), but the datasheet names it "Sticky
///   Flag Register 3" (SLOS756B Table 4) and the guide itself lists it next to 42 and 44 as a
///   register whose read stops an interrupt pulse train (§5.2.45). It is on the list as a
///   precaution: refusing a reserved register costs nothing.
///
/// The live copies are safe to read: register 43 for 42 and register 46 for 44 (§5.2.40 and
/// §5.2.43; "non-sticky versions of the interrupt flags", §2.4.4.2.1). So are the power and gain
/// flags in page 0, registers 37 and 38, and in page 1, register 63 (§5.2.36, §5.2.37 and §5.3.25).
/// The kernel driver reads none of the three sticky registers.
pub const CLEAR_ON_READ: [Register; 3] = [STICKY_FLAGS_1, STICKY_FLAGS_2, STICKY_FLAGS_3];

// The list above is what every read method checks. These assertions fail the build if a sticky
// register drops off the list or a live copy lands on it.
const _: () = {
    assert!(STICKY_FLAGS_1.is_clear_on_read());
    assert!(STICKY_FLAGS_2.is_clear_on_read());
    assert!(STICKY_FLAGS_3.is_clear_on_read());
    assert!(!INTERRUPT_FLAGS_1.is_clear_on_read());
    assert!(!INTERRUPT_FLAGS_2.is_clear_on_read());
    assert!(!DAC_FLAGS_1.is_clear_on_read());
    assert!(!DAC_FLAGS_2.is_clear_on_read());
    assert!(!HP_GAIN_FLAGS.is_clear_on_read());
};

// ---------------------------------------------------------------------------------------------
// Page 0: clocks, serial interface, flags, pins, processing block, digital volume (§5.2).
// ---------------------------------------------------------------------------------------------

/// Software reset (§5.2.2; `aic32x4.h:22`). Write-only.
///
/// All registers take their reset values within 1 ms, and during that time no block may be powered
/// up and no coefficient register touched (§3.1 and §3.2). The kernel driver writes it at probe and
/// waits 1 ms (`aic32x4.c:1129-1131`).
pub const SOFTWARE_RESET: Register = Register::new(0, 1);
/// Bits of [`SOFTWARE_RESET`].
pub mod software_reset {
    /// D0: writing 1 resets the chip; the bit clears itself. The kernel driver writes this value
    /// (`aic32x4.c:1129`).
    pub const RESET: u8 = 0b0000_0001;
}

/// Clock setting 1, multiplexers: where the PLL and the codec take their clock from (§5.2.5;
/// `aic32x4.h:23`). Resets to MCLK for both.
pub const CLOCK_MUX: Register = Register::new(0, 4);
/// Bits of [`CLOCK_MUX`].
pub mod clock_mux {
    /// D6: the PLL runs in its high clock range. Which output frequencies each range allows depends
    /// on AVDD (Table 2-22).
    pub const PLL_RANGE_HIGH: u8 = 0b0100_0000;
    /// D3 to D2: the PLL's input, PLL_CLKIN.
    pub const PLL_CLKIN_MASK: u8 = 0b0000_1100;
    /// D3 to D2 = 00: PLL_CLKIN is the MCLK pin.
    pub const PLL_CLKIN_MCLK: u8 = 0b0000_0000;
    /// D3 to D2 = 01: PLL_CLKIN is the BCLK pin.
    pub const PLL_CLKIN_BCLK: u8 = 0b0000_0100;
    /// D3 to D2 = 10: PLL_CLKIN is the GPIO pin, which only the WCSP package has.
    pub const PLL_CLKIN_GPIO: u8 = 0b0000_1000;
    /// D3 to D2 = 11: PLL_CLKIN is the DIN pin.
    pub const PLL_CLKIN_DIN: u8 = 0b0000_1100;
    /// D1 to D0: the codec's input, CODEC_CLKIN, the clock NDAC divides (Figure 2-45).
    pub const CODEC_CLKIN_MASK: u8 = 0b0000_0011;
    /// D1 to D0 = 00: CODEC_CLKIN is the MCLK pin.
    pub const CODEC_CLKIN_MCLK: u8 = 0b0000_0000;
    /// D1 to D0 = 01: CODEC_CLKIN is the BCLK pin.
    pub const CODEC_CLKIN_BCLK: u8 = 0b0000_0001;
    /// D1 to D0 = 10: CODEC_CLKIN is the GPIO pin, which only the WCSP package has.
    pub const CODEC_CLKIN_GPIO: u8 = 0b0000_0010;
    /// D1 to D0 = 11: CODEC_CLKIN is the PLL's output. With the PLL's input left on MCLK this is
    /// the `0x03` the kernel driver writes (`AIC32X4_PLLCLKIN`,
    /// `aic32x4.h:133`, written at `aic32x4.c:878`).
    pub const CODEC_CLKIN_PLL: u8 = 0b0000_0011;
}

/// Clock setting 2: PLL power, P divider and R multiplier (§5.2.6; `aic32x4.h:24`). Resets to P =
/// 1, R = 1, powered down.
///
/// PLL_CLK = PLL_CLKIN x R x J.D / P (Equation 17). The clock is available about 10 ms after
/// power-up (§3.4).
pub const PLL_P_R: Register = Register::new(0, 5);
/// Bits of [`PLL_P_R`].
pub mod pll_p_r {
    /// D7: the PLL is powered up (`AIC32X4_PLLEN`, `aic32x4.h:148`).
    pub const POWER_UP: u8 = 0b1000_0000;
    /// D6 to D4: the divider P; 1 to 7 mean themselves and 0 means 8. The kernel driver shifts P by
    /// the same four bits (`aic32x4.c:881-882`).
    pub const P_MASK: u8 = 0b0111_0000;
    /// Bit position of P.
    pub const P_SHIFT: u8 = 4;
    /// D3 to D0: the multiplier R. Only 1 to 4 are allowed; 0 and 5 to 15 are "reserved, do not
    /// use" ("R = 1, 2, 3, 4", §2.7.1). The kernel driver always writes 1 (`aic32x4.c:881-882`).
    pub const R_MASK: u8 = 0b0000_1111;
}

/// Clock setting 3: PLL multiplier J, the integer part of J.D (§5.2.7; `aic32x4.h:25`). Resets
/// to 4.
pub const PLL_J: Register = Register::new(0, 6);
/// Bits of [`PLL_J`].
pub mod pll_j {
    /// D5 to D0: J = 4 to 63; 0 to 3 are "do not use". The PLL section gives the range as 1 to 63
    /// (§2.7.1); the register description is the stricter of the two and the one to follow.
    pub const J_MASK: u8 = 0b0011_1111;
}

/// Clock setting 4: PLL fractional multiplier D, bits 13 to 8 (§5.2.8;
/// `aic32x4.h:26`). The chip only takes it over when [`PLL_D_LSB`] is written immediately
/// afterwards (§5.2.8 note; §2.7.1).
pub const PLL_D_MSB: Register = Register::new(0, 7);
/// Bits of [`PLL_D_MSB`].
pub mod pll_d_msb {
    /// D5 to D0: D\[13:8\], of a D that ranges over 0 to 9999. With D = 0 the PLL's input divided
    /// by P must lie between 512 kHz and 20 MHz; with any other D between 10 MHz and 20 MHz
    /// (Equations 18 and 19).
    pub const D_HIGH_MASK: u8 = 0b0011_1111;
}

/// Clock setting 5: PLL fractional multiplier D, bits 7 to 0 (§5.2.9;
/// `aic32x4.h:27`). Must be written immediately after [`PLL_D_MSB`] (§5.2.9 note).
pub const PLL_D_LSB: Register = Register::new(0, 8);

/// Clock setting 6: NDAC, the divider from CODEC_CLKIN to DAC_CLK (§5.2.11; `aic32x4.h:28`). Resets
/// to 1, powered down.
pub const NDAC: Register = Register::new(0, 11);
/// Clock setting 7: MDAC, the divider from DAC_CLK to DAC_MOD_CLK (§5.2.12; `aic32x4.h:29`). Resets
/// to 1, powered down.
///
/// MDAC x DOSR / 32 must be at least the resource class of the selected processing block (§2.4.6;
/// classes in Table 2-13).
pub const MDAC: Register = Register::new(0, 12);
/// Clock setting 10: BCLK N, the divider from BDIV_CLKIN to the BCLK output (§5.2.30;
/// `aic32x4.h:40`). Resets to 1, powered down. It only matters while BCLK is an output ([`interface_1::BCLK_OUTPUT`]).
pub const BCLK_N: Register = Register::new(0, 30);
/// The layout [`NDAC`], [`MDAC`] and [`BCLK_N`] share.
pub mod divider {
    /// D7: the divider is powered up (§5.2.11, §5.2.12 and §5.2.30;
    /// `AIC32X4_NDACEN`, `AIC32X4_MDACEN` and `AIC32X4_BCLKEN`, `aic32x4.h:149-150` and `:153`).
    ///
    /// NDAC and MDAC must stay powered until the DAC channels report powered down in
    /// [`DAC_FLAGS_1`](super::DAC_FLAGS_1); then MDAC goes first and NDAC after it (§2.7).
    pub const POWER_UP: u8 = 0b1000_0000;
    /// D6 to D0: the divisor; 1 to 127 mean themselves and 0 means 128 (§5.2.11, §5.2.12 and
    /// §5.2.30).
    pub const VALUE_MASK: u8 = 0b0111_1111;
}

/// DAC OSR setting 1: DOSR, bits 9 to 8 (§5.2.13;
/// `aic32x4.h:30`). The chip only takes it over when [`DOSR_LSB`] is written immediately afterwards
/// (§5.2.13 note).
///
/// DOSR is the oversampling ratio from DAC_MOD_CLK to the sample rate. 2 to 1022 mean themselves, 0
/// means 1024, and 1 and 1023 are reserved (§5.2.13). It resets to 128. DOSR x sample rate must lie
/// between 2.8 MHz and 6.2 MHz (§2.4.6).
///
/// Which multiples the interpolation filters need is stated both ways round: a multiple of 2 for
/// filter A, 4 for B and 8 for C in the register description (§5.2.13 note), but 8 for A, 4 for B
/// and 2 for C in the setup section (§2.4.6). A multiple of 8 satisfies both, and the 128 the
/// kernel driver uses with filter A is one (`aic32x4.c:611`).
pub const DOSR_MSB: Register = Register::new(0, 13);
/// Bits of [`DOSR_MSB`].
pub mod dosr_msb {
    /// D1 to D0: DOSR\[9:8\]. The kernel driver writes `dosr >> 8` here (`aic32x4.c:901`).
    pub const DOSR_HIGH_MASK: u8 = 0b0000_0011;
}

/// DAC OSR setting 2: DOSR, bits 7 to 0 (§5.2.14;
/// `aic32x4.h:31`). Must be written immediately after [`DOSR_MSB`] (§5.2.14 note).
pub const DOSR_LSB: Register = Register::new(0, 14);

/// ADC oversampling ratio AOSR (§5.2.20; `AIC32X4_AOSR`, `aic32x4.h:34`). On this chip it belongs
/// to the unused digital microphone path and is read-only, reset value 128: it always reads
/// [`AOSR_VALUE`]. The kernel driver writes that same value with every stream (`aic32x4.c:915`).
/// Nothing in this crate writes it. It is in the map only to be read: it is the one page 0 register
/// whose value neither a reset, a write nor a stream changes, so reading it tells whether the chip
/// at the address is a TLV320DAC3203 (or a part of its family) without writing anything
/// ([`Tlv320dac3203::probe`](crate::Tlv320dac3203::probe)).
pub const AOSR: Register = Register::new(0, 20);
/// What [`AOSR`] reads: `1000 0000`, AOSR = 128, read-only (§5.2.20).
pub const AOSR_VALUE: u8 = 0x80;

/// Audio interface setting 1: format, word length, clock directions (§5.2.27; `aic32x4.h:37`).
/// Resets to I2S, 16 bits, both clocks inputs.
pub const INTERFACE_1: Register = Register::new(0, 27);
/// Bits of [`INTERFACE_1`].
pub mod interface_1 {
    /// D7 to D6: the serial format. The kernel driver shifts its mode numbers
    /// (`aic32x4.h:115-118`) by six bits into this field (`aic32x4.c:819-833`).
    pub const FORMAT_MASK: u8 = 0b1100_0000;
    /// D7 to D6 = 00: I2S (`AIC32X4_I2S_MODE`, `aic32x4.h:115`).
    pub const FORMAT_I2S: u8 = 0b0000_0000;
    /// D7 to D6 = 01: DSP (`AIC32X4_DSP_MODE`, `aic32x4.h:116`).
    pub const FORMAT_DSP: u8 = 0b0100_0000;
    /// D7 to D6 = 10: right-justified (`aic32x4.h:117`).
    pub const FORMAT_RIGHT_JUSTIFIED: u8 = 0b1000_0000;
    /// D7 to D6 = 11: left-justified (`aic32x4.h:118`).
    pub const FORMAT_LEFT_JUSTIFIED: u8 = 0b1100_0000;
    /// D5 to D4: the word length. The kernel driver shifts its length codes
    /// (`aic32x4.h:107-110`) by four bits into this field (`aic32x4.c:923-938`).
    pub const WORD_LENGTH_MASK: u8 = 0b0011_0000;
    /// D5 to D4 = 00: 16 bits (`aic32x4.h:107`).
    pub const WORD_LENGTH_16: u8 = 0b0000_0000;
    /// D5 to D4 = 01: 20 bits (`aic32x4.h:108`).
    pub const WORD_LENGTH_20: u8 = 0b0001_0000;
    /// D5 to D4 = 10: 24 bits (`aic32x4.h:109`).
    pub const WORD_LENGTH_24: u8 = 0b0010_0000;
    /// D5 to D4 = 11: 32 bits (`aic32x4.h:110`).
    pub const WORD_LENGTH_32: u8 = 0b0011_0000;
    /// D3: BCLK is an output, so this chip is the bit-clock master
    /// (`AIC32X4_BCLKMASTER`, `aic32x4.h:146`).
    pub const BCLK_OUTPUT: u8 = 0b0000_1000;
    /// D2: WCLK is an output, so this chip is the word-clock master
    /// (`AIC32X4_WCLKMASTER`, `aic32x4.h:147`).
    pub const WCLK_OUTPUT: u8 = 0b0000_0100;
    /// D0: the MFP2 pin goes high-impedance after the loopback data has been sent. It only matters
    /// while MFP2 carries the data output.
    pub const MFP2_HIGH_IMPEDANCE: u8 = 0b0000_0001;
}

/// Audio interface setting 2: the data offset, how many BCLKs after the word clock edge the data
/// starts, 0 to 255 (§5.2.28;
/// `aic32x4.h:38`). All eight bits are the value. The kernel driver writes 0 (`aic32x4.c:797` and
/// `:841`).
pub const DATA_OFFSET: Register = Register::new(0, 28);

/// Audio interface setting 3: loopback, BCLK polarity and what the BCLK divider is fed from
/// (§5.2.29; `aic32x4.h:39`).
pub const INTERFACE_3: Register = Register::new(0, 29);
/// Bits of [`INTERFACE_3`].
pub mod interface_3 {
    /// D5: the audio data input is routed back out on the MFP2 pin.
    pub const LOOPBACK: u8 = 0b0010_0000;
    /// D3: BCLK is inverted. The kernel driver clears this bit for the normal polarity the machine
    /// driver asks for (`aic32x4.c:798-799`).
    pub const BCLK_INVERTED: u8 = 0b0000_1000;
    /// D2: the BCLK and WCLK buffers stay powered while the codec is powered down, for when they
    /// are used in clock generation.
    pub const CLOCKS_ACTIVE_WHEN_POWERED_DOWN: u8 = 0b0000_0100;
    /// D1 to D0: the input of the BCLK N divider, BDIV_CLKIN. 10 and 11 are reserved there,
    /// although the clock section also names the two digital-microphone clocks as inputs (§2.7).
    pub const BDIV_CLKIN_MASK: u8 = 0b0000_0011;
    /// D1 to D0 = 00: BDIV_CLKIN is DAC_CLK, the reset value.
    pub const BDIV_CLKIN_DAC_CLK: u8 = 0b0000_0000;
    /// D1 to D0 = 01: BDIV_CLKIN is DAC_MOD_CLK (`AIC32X4_DACMOD2BCLK`, `aic32x4.h:167`, set at
    /// `aic32x4.c:807`).
    pub const BDIV_CLKIN_DAC_MOD_CLK: u8 = 0b0000_0001;
}

/// Audio interface setting 6: what the BCLK, WCLK and secondary clock outputs carry (§5.2.33). The
/// driver only reads it: its reset value `0x00` makes a WCLK output (register 27, bit D2) carry the
/// generated DAC_FS (bits D5 to D4 = 00), so a host can count word-clock edges to see whether the
/// chip's clock tree runs.
pub const INTERFACE_6: Register = Register::new(0, 33);

/// DAC flag register 1: power status of the DACs and the headphone drivers (§5.2.36). Read-only,
/// plain status, not sticky.
pub const DAC_FLAGS_1: Register = Register::new(0, 37);
/// Bits of [`DAC_FLAGS_1`].
pub mod dac_flags_1 {
    /// D7: the left DAC is powered up.
    pub const LEFT_DAC_POWERED: u8 = 0b1000_0000;
    /// D5: the left headphone driver, HPL, is powered up.
    pub const HPL_POWERED: u8 = 0b0010_0000;
    /// D3: the right DAC is powered up.
    pub const RIGHT_DAC_POWERED: u8 = 0b0000_1000;
    /// D1: the right headphone driver, HPR, is powered up.
    pub const HPR_POWERED: u8 = 0b0000_0010;
}

/// DAC flag register 2: whether the digital volume has finished soft-stepping (§5.2.37; §2.4.4.1).
/// Read-only, plain status, not sticky.
pub const DAC_FLAGS_2: Register = Register::new(0, 38);
/// Bits of [`DAC_FLAGS_2`].
pub mod dac_flags_2 {
    /// D4: the left DAC's applied gain equals the programmed one.
    pub const LEFT_GAIN_APPLIED: u8 = 0b0001_0000;
    /// D0: the right DAC's applied gain equals the programmed one.
    pub const RIGHT_GAIN_APPLIED: u8 = 0b0000_0001;
}

/// Sticky flag register 1: DAC overflow since the last read (§5.2.39). Bits as in
/// [`overflow_flags`].
///
/// **Clear-on-read**: both flags "self clear on read". Listed in [`CLEAR_ON_READ`]. Read
/// [`INTERRUPT_FLAGS_1`] for the present state without the side effect.
pub const STICKY_FLAGS_1: Register = Register::new(0, 42);
/// Interrupt flag register 1: DAC overflow at the time of reading (§5.2.40). Bits as in
/// [`overflow_flags`]. Read-only and not sticky.
pub const INTERRUPT_FLAGS_1: Register = Register::new(0, 43);
/// Bits of [`STICKY_FLAGS_1`] and [`INTERRUPT_FLAGS_1`].
pub mod overflow_flags {
    /// D7: the left DAC overflowed (§5.2.39 and §5.2.40).
    pub const LEFT_DAC_OVERFLOW: u8 = 0b1000_0000;
    /// D6: the right DAC overflowed (§5.2.39 and §5.2.40).
    pub const RIGHT_DAC_OVERFLOW: u8 = 0b0100_0000;
}

/// Sticky flag register 2: headphone over-current, headset events and DRC threshold since the last
/// read (§5.2.41). Bits as in [`event_flags`].
///
/// **Clear-on-read**: every flag "will be cleared when the register is read". Listed in
/// [`CLEAR_ON_READ`]. Read [`INTERRUPT_FLAGS_2`] for the present state without the side effect.
pub const STICKY_FLAGS_2: Register = Register::new(0, 44);
/// Interrupt flag register 2: the same conditions at the time of reading (§5.2.43). Bits as in
/// [`event_flags`]. Read-only and not sticky.
pub const INTERRUPT_FLAGS_2: Register = Register::new(0, 46);
/// Bits of [`STICKY_FLAGS_2`] and [`INTERRUPT_FLAGS_2`].
pub mod event_flags {
    /// D7: over-current on the left headphone driver (§5.2.41 and §5.2.43).
    pub const HPL_OVER_CURRENT: u8 = 0b1000_0000;
    /// D6: over-current on the right headphone driver (§5.2.41 and §5.2.43).
    pub const HPR_OVER_CURRENT: u8 = 0b0100_0000;
    /// D5: a headset button press (§5.2.41 and §5.2.43).
    pub const HEADSET_BUTTON: u8 = 0b0010_0000;
    /// D4: in the sticky register, a headset was inserted or removed; in the live register, a
    /// headset is inserted (§5.2.41 and §5.2.43). This driver leaves the chip's detection off.
    pub const HEADSET_INSERTION: u8 = 0b0001_0000;
    /// D3: the left channel's signal power exceeded the DRC threshold (§5.2.41 and §5.2.43).
    pub const LEFT_DRC_THRESHOLD: u8 = 0b0000_1000;
    /// D2: the right channel's signal power exceeded the DRC threshold (§5.2.41 and §5.2.43).
    pub const RIGHT_DRC_THRESHOLD: u8 = 0b0000_0100;
}

/// Page 0, register 45: reserved in the guide (§5.2.42), "Sticky Flag Register 3" in the datasheet
/// (SLOS756B Table 4). No bit of it is documented for this chip.
///
/// **Treated as clear-on-read** and listed in [`CLEAR_ON_READ`], because the guide names it with
/// registers 42 and 44 as one whose read ends an interrupt pulse train (§5.2.45). It exists in the
/// map only so that the read methods can refuse it.
pub const STICKY_FLAGS_3: Register = Register::new(0, 45);

/// MFP2 function control: what the MFP2 pin does (§5.2.49). Resets to the loopback data output with
/// the bus keeper off.
///
/// The kernel driver calls this register `AIC32X4_DOUTCTL` (`aic32x4.h:44`), a name from the
/// codecs it was written for, where the pin is the data output. It uses the pin as a
/// general-purpose output "to mute AMP if present": it writes `0x05` (output, high) at probe
/// (`aic32x4.c:1179`) and at every stream stop (`aic32x4.c:335-336`). Whether the pin is wired to
/// anything is the board's business.
pub const MFP2_CONTROL: Register = Register::new(0, 53);
/// Bits of [`MFP2_CONTROL`].
pub mod mfp2_control {
    /// D4: the bus keeper on MFP2 is disabled, the reset value; the bit is the disable.
    pub const BUS_KEEPER_DISABLED: u8 = 0b0001_0000;
    /// D3 to D1: what the pin carries.
    pub const FUNCTION_MASK: u8 = 0b0000_1110;
    /// D3 to D1 = 000: the pin is disabled.
    pub const FUNCTION_DISABLED: u8 = 0b0000_0000;
    /// D3 to D1 = 001: the primary data output, which on this chip carries loopback data; the reset
    /// value.
    pub const FUNCTION_PRIMARY_DOUT: u8 = 0b0000_0010;
    /// D3 to D1 = 010: a general-purpose output driven by bit D0
    /// (`MFP2_GPIO_ENABLE`, `aic32x4.h:180`).
    pub const FUNCTION_GENERAL_PURPOSE_OUTPUT: u8 = 0b0000_0100;
    /// D3 to D1 = 011: the CLKOUT output.
    pub const FUNCTION_CLKOUT: u8 = 0b0000_0110;
    /// D3 to D1 = 100: the INT1 output.
    pub const FUNCTION_INT1: u8 = 0b0000_1000;
    /// D3 to D1 = 101: the INT2 output.
    pub const FUNCTION_INT2: u8 = 0b0000_1010;
    /// D3 to D1 = 110: the secondary BCLK.
    pub const FUNCTION_SECONDARY_BCLK: u8 = 0b0000_1100;
    /// D3 to D1 = 111: the secondary WCLK.
    pub const FUNCTION_SECONDARY_WCLK: u8 = 0b0000_1110;
    /// D0: the pin's level as a general-purpose output (`MFP2_GPIO_HI`, `aic32x4.h:181`).
    pub const GENERAL_PURPOSE_VALUE: u8 = 0b0000_0001;
}

/// SCLK/MFP3 function control (§5.2.52;
/// `aic32x4.h:46`). Resets to enabled. The kernel driver writes 0, pin disabled, at probe (`aic32x4.c:1183`).
pub const SCLK_MFP3_CONTROL: Register = Register::new(0, 56);
/// Bits of [`SCLK_MFP3_CONTROL`].
pub mod sclk_mfp3_control {
    /// D2 to D1: what the pin does. 11 is reserved.
    pub const FUNCTION_MASK: u8 = 0b0000_0110;
    /// D2 to D1 = 00: the pin is disabled.
    pub const FUNCTION_DISABLED: u8 = 0b0000_0000;
    /// D2 to D1 = 01: the pin is an input for the secondary audio interface or a digital microphone
    /// (with I2C control), or the SPI clock; the reset value.
    pub const FUNCTION_ENABLED: u8 = 0b0000_0010;
    /// D2 to D1 = 10: a general-purpose input, read back in bit D0.
    pub const FUNCTION_GENERAL_PURPOSE_INPUT: u8 = 0b0000_0100;
    /// D0: the pin's level as a general-purpose input. Read-only.
    pub const INPUT_VALUE: u8 = 0b0000_0001;
}

/// DAC signal processing block control (§5.2.54; `aic32x4.h:47`). Resets to PRB_P1.
pub const DAC_PROCESSING_BLOCK: Register = Register::new(0, 60);
/// Bits of [`DAC_PROCESSING_BLOCK`].
pub mod dac_processing_block {
    /// D4 to D0: 1 to 25 select PRB_P1 to PRB_P25; 0 and 26 to 31 are reserved
    /// (`AIC32X4_DACSPBLOCK_MASK`, `aic32x4.h:127`). What each block contains is in Table 2-13.
    pub const SELECT_MASK: u8 = 0b0001_1111;
    /// PRB_P1: stereo, interpolation filter A, three biquads, resource class 8 (Table 2-13). The
    /// reset value.
    pub const PRB_P1: u8 = 1;
    /// PRB_P2: stereo, interpolation filter A, first-order IIR, six biquads and the DRC, resource
    /// class 12 (Table 2-13). The block the kernel driver selects (`aic32x4.c:950`).
    pub const PRB_P2: u8 = 2;
    /// PRB_P7: stereo, interpolation filter B, a first-order IIR, no biquads, no DRC, resource
    /// class 6 (Table 2-13).
    pub const PRB_P7: u8 = 7;
    /// PRB_P9: stereo, interpolation filter B, four biquads, no IIR, no DRC, resource class 6
    /// (Table 2-13).
    pub const PRB_P9: u8 = 9;
    /// PRB_P25: stereo, interpolation filter A, a first-order IIR, five biquads, the DRC, the
    /// 3D-PGA and the beep generator, resource class 12 (Table 2-13). The only block with the beep
    /// generator (Table 2-13; §2.4.5.1).
    pub const PRB_P25: u8 = 25;
}

/// DAC channel setup 1: channel power, data routing and volume soft-stepping (§5.2.57;
/// `aic32x4.h:49`). Resets to both channels off, each taking its own side of the data.
pub const DAC_SETUP_1: Register = Register::new(0, 63);
/// Bits of [`DAC_SETUP_1`].
pub mod dac_setup_1 {
    /// D7: the left DAC channel is powered up.
    pub const LEFT_POWER_UP: u8 = 0b1000_0000;
    /// D6: the right DAC channel is powered up (the kernel's "Right DAC" power bit,
    /// `aic32x4.c:660`).
    pub const RIGHT_POWER_UP: u8 = 0b0100_0000;
    /// D5 to D4: what the left DAC plays.
    pub const LEFT_DATA_MASK: u8 = 0b0011_0000;
    /// D5 to D4 = 00: nothing.
    pub const LEFT_DATA_OFF: u8 = 0b0000_0000;
    /// D5 to D4 = 01: the left channel of the interface, the reset value
    /// (`AIC32X4_LDAC2LCHN`, `aic32x4.h:157`).
    pub const LEFT_DATA_LEFT: u8 = 0b0001_0000;
    /// D5 to D4 = 10: the right channel of the interface (`AIC32X4_LDAC2RCHN`, `aic32x4.h:156`).
    pub const LEFT_DATA_RIGHT: u8 = 0b0010_0000;
    /// D5 to D4 = 11: the mono mix of both channels.
    pub const LEFT_DATA_MONO_MIX: u8 = 0b0011_0000;
    /// D3 to D2: what the right DAC plays.
    pub const RIGHT_DATA_MASK: u8 = 0b0000_1100;
    /// D3 to D2 = 00: nothing.
    pub const RIGHT_DATA_OFF: u8 = 0b0000_0000;
    /// D3 to D2 = 01: the right channel of the interface, the reset value
    /// (`AIC32X4_RDAC2RCHN`, `aic32x4.h:158`).
    pub const RIGHT_DATA_RIGHT: u8 = 0b0000_0100;
    /// D3 to D2 = 10: the left channel of the interface (`AIC32X4_RDAC2LCHN`, `aic32x4.h:155`).
    pub const RIGHT_DATA_LEFT: u8 = 0b0000_1000;
    /// D3 to D2 = 11: the mono mix of both channels.
    pub const RIGHT_DATA_MONO_MIX: u8 = 0b0000_1100;
    /// D1 to D0: soft-stepping of the digital volume
    /// (`AIC32X4_DAC_SOFT_STEPPING_MASK`, `aic32x4.h:162`). 11 is reserved.
    pub const SOFT_STEP_MASK: u8 = 0b0000_0011;
    /// D1 to D0 = 00: one 0.5 dB step per sample, the reset value (§2.4.4.1).
    pub const SOFT_STEP_PER_SAMPLE: u8 = 0b0000_0000;
    /// D1 to D0 = 01: one step per two samples.
    pub const SOFT_STEP_PER_TWO_SAMPLES: u8 = 0b0000_0001;
    /// D1 to D0 = 10: soft-stepping is off and volume changes apply at once. The kernel driver's
    /// default (`AIC32X4_DAC_DEFAULT_SOFT_STEPPING`, `aic32x4.h:161`, applied at
    /// `aic32x4.c:123` and `:970-973`).
    pub const SOFT_STEP_DISABLED: u8 = 0b0000_0010;
}

/// DAC channel setup 2: mute, auto-mute and volume ganging (§5.2.58). Both channels reset to muted.
/// The kernel header calls it `AIC32X4_DACMUTE` (`aic32x4.h:50`).
pub const DAC_SETUP_2: Register = Register::new(0, 64);
/// Bits of [`DAC_SETUP_2`].
pub mod dac_setup_2 {
    /// D7: while the right DAC channel is powered down, its modulator outputs the inverse of the
    /// left one, for a differential mono output.
    pub const RIGHT_MODULATOR_INVERTS_LEFT: u8 = 0b1000_0000;
    /// D6 to D4: auto-mute after a run of constant input samples. 0 is off, the reset value; 1 to 7
    /// mean 100, 200, 400, 800, 1600, 3200 and 6400 samples (§2.4.5.2). The kernel exposes the
    /// field as the "Auto-mute Switch" control (`aic32x4.c:545`).
    pub const AUTO_MUTE_MASK: u8 = 0b0111_0000;
    /// Bit position of the auto-mute code.
    pub const AUTO_MUTE_SHIFT: u8 = 4;
    /// D3: the left DAC channel is muted. With
    /// [`RIGHT_MUTE`] this is the kernel's `AIC32X4_MUTEON`, `0x0C` (`aic32x4.h:165`); alone it is
    /// `AIC32X4_UNMUTE_MONO`, which leaves the left channel muted when only the right one plays (`aic32x4.h:166`,
    /// `aic32x4.c:342-346`).
    pub const LEFT_MUTE: u8 = 0b0000_1000;
    /// D2: the right DAC channel is muted.
    pub const RIGHT_MUTE: u8 = 0b0000_0100;
    /// D1 to D0: whether the two volume registers act independently. 11 is reserved.
    pub const VOLUME_CONTROL_MASK: u8 = 0b0000_0011;
    /// D1 to D0 = 00: each channel follows its own volume register, the reset value.
    pub const VOLUME_CONTROL_INDEPENDENT: u8 = 0b0000_0000;
    /// D1 to D0 = 01: the left channel follows the right volume register.
    pub const VOLUME_CONTROL_LEFT_FOLLOWS_RIGHT: u8 = 0b0000_0001;
    /// D1 to D0 = 10: the right channel follows the left volume register.
    pub const VOLUME_CONTROL_RIGHT_FOLLOWS_LEFT: u8 = 0b0000_0010;
}

/// Left DAC channel digital volume (§5.2.59; `aic32x4.h:51`). Resets to 0 dB.
pub const LEFT_DAC_VOLUME: Register = Register::new(0, 65);
/// Right DAC channel digital volume (§5.2.60; `aic32x4.h:52`). Resets to 0 dB.
pub const RIGHT_DAC_VOLUME: Register = Register::new(0, 66);
/// The values [`LEFT_DAC_VOLUME`] and [`RIGHT_DAC_VOLUME`] take: the gain in half decibels as an
/// 8-bit two's complement number (§5.2.59 and §5.2.60; §2.4.4.1). The kernel's "PCM Playback
/// Volume" control has the same limits (`aic32x4.c:98-99` and `:520-521`).
pub mod dac_volume {
    /// The lowest setting, -63.5 dB: -127 half decibels. `0x80` is reserved (§5.2.59 and §5.2.60).
    pub const MIN_MINUS_63_5_DB: u8 = 0b1000_0001;
    /// 0 dB, the reset value (§5.2.59 and §5.2.60).
    pub const ZERO_DB: u8 = 0b0000_0000;
    /// The highest setting, +24 dB: 48 half decibels. Everything from 49 up to 127 is reserved
    /// (§5.2.59 and §5.2.60).
    pub const MAX_PLUS_24_DB: u8 = 0b0011_0000;
}

/// DRC control 1: enables, threshold and hysteresis (§5.2.62; `aic32x4.h:53`).
///
/// It resets to
/// `0x6F`, both enables set. The DRC only exists in some processing blocks (Table 2-13), and the kernel driver writes `0x0F`,
/// both enables cleared, whenever a stream is set up (`aic32x4.c:55-59` and `:953-959`).
pub const DRC_CONTROL_1: Register = Register::new(0, 68);
/// Bits of [`DRC_CONTROL_1`].
pub mod drc_control_1 {
    /// D6: the left channel's DRC is enabled (the kernel's "DRC Control" uses bits 6 and 5,
    /// `aic32x4.c:105-107`).
    pub const LEFT_ENABLE: u8 = 0b0100_0000;
    /// D5: the right channel's DRC is enabled.
    pub const RIGHT_ENABLE: u8 = 0b0010_0000;
    /// D4 to D2: the threshold; code n means -3 x (n + 1) dBFS, so 0 is -3 dBFS and 7 is -24 dBFS.
    /// Resets to 3, -12 dBFS (§2.4.4.2.1).
    pub const THRESHOLD_MASK: u8 = 0b0001_1100;
    /// Bit position of the threshold code.
    pub const THRESHOLD_SHIFT: u8 = 2;
    /// D1 to D0: the hysteresis in decibels, 0 to 3. Resets to 3.
    pub const HYSTERESIS_MASK: u8 = 0b0000_0011;
}

/// DRC control 2: hold time (§5.2.63; `aic32x4.h:54`). Resets to `0x38`.
pub const DRC_CONTROL_2: Register = Register::new(0, 69);
/// Bits of [`DRC_CONTROL_2`].
pub mod drc_control_2 {
    /// D6 to D3: the hold time. 0 disables the hold; 1 to 15 select a number of sample periods from
    /// 32 upwards (§5.2.63, which lists the steps). D2 to D0 are reserved and must stay 0.
    pub const HOLD_MASK: u8 = 0b0111_1000;
    /// Bit position of the hold code.
    pub const HOLD_SHIFT: u8 = 3;
}

/// DRC control 3: attack and decay rates (§5.2.64; `aic32x4.h:55`). Resets to 0.
pub const DRC_CONTROL_3: Register = Register::new(0, 70);
/// Bits of [`DRC_CONTROL_3`].
pub mod drc_control_3 {
    /// D7 to D4: the attack rate; 0 is 4 dB per sample period and every step halves it.
    pub const ATTACK_MASK: u8 = 0b1111_0000;
    /// Bit position of the attack code.
    pub const ATTACK_SHIFT: u8 = 4;
    /// D3 to D0: the decay rate; 0 is 1.5625e-2 dB per sample period and every step halves it.
    pub const DECAY_MASK: u8 = 0b0000_1111;
}

// ---------------------------------------------------------------------------------------------
// Page 1: power supply, output drivers, routing, analog gain (§5.3).
// ---------------------------------------------------------------------------------------------

/// Power configuration (§5.3.2; `aic32x4.h:74`).
pub const POWER_CONFIG: Register = Register::new(1, 1);
/// Bits of [`POWER_CONFIG`].
pub mod power_config {
    /// D3: the weak connection of AVDD to DVDD is disabled
    /// (`AIC32X4_AVDDWEAKDISABLE`, `aic32x4.h:120`). The connection is on after reset, for a system
    /// whose AVDD is neither powered externally nor by the LDO; the setup sequence turns it off
    /// before the analog blocks are enabled (§2.4.6), and the kernel driver does so at probe
    /// (`aic32x4.c:1138-1139`).
    pub const AVDD_WEAK_CONNECTION_DISABLED: u8 = 0b0000_1000;
}

/// LDO control: the AVDD LDO and the master switch of the analog blocks (§5.3.3; `aic32x4.h:75`).
/// Resets to LDO off, analog blocks disabled.
pub const LDO_CONTROL: Register = Register::new(1, 2);
/// Bits of [`LDO_CONTROL`].
pub mod ldo_control {
    /// D5 to D4: the AVDD LDO's output voltage. 11 is "do not use".
    pub const AVDD_LDO_VOLTAGE_MASK: u8 = 0b0011_0000;
    /// D5 to D4 = 00: nominally 1.72 V, the reset value.
    pub const AVDD_LDO_1V72: u8 = 0b0000_0000;
    /// D5 to D4 = 01: nominally 1.67 V.
    pub const AVDD_LDO_1V67: u8 = 0b0001_0000;
    /// D5 to D4 = 10: nominally 1.77 V.
    pub const AVDD_LDO_1V77: u8 = 0b0010_0000;
    /// D3: the analog blocks are disabled, the reset value; the bit is the disable. The kernel
    /// driver clears it by writing `0x01` to the register (`aic32x4.c:1141-1143`).
    pub const ANALOG_BLOCKS_DISABLED: u8 = 0b0000_1000;
    /// D2: the offset correction has completed. Read-only.
    pub const OFFSET_CORRECTION_DONE: u8 = 0b0000_0100;
    /// D1: over-current was detected on the AVDD LDO. Read-only.
    pub const AVDD_LDO_OVER_CURRENT: u8 = 0b0000_0010;
    /// D0: the AVDD LDO is powered up (`AIC32X4_LDOCTLEN`, `aic32x4.h:121`).
    pub const AVDD_LDO_POWER_UP: u8 = 0b0000_0001;
}

/// Playback configuration 1: the left channel's driver class and PowerTune mode (§5.3.4). Resets to
/// class AB, PTM_P3/PTM_P4. The kernel driver never writes it.
pub const PLAYBACK_CONFIG_1: Register = Register::new(1, 3);
/// Playback configuration 2: the same for the right channel (§5.3.5). The kernel driver never
/// writes it.
pub const PLAYBACK_CONFIG_2: Register = Register::new(1, 4);
/// The layout [`PLAYBACK_CONFIG_1`] and [`PLAYBACK_CONFIG_2`] share.
pub mod playback_config {
    /// D7 to D6: the headphone driver's class (§5.3.4 and §5.3.5). 01 and 10 are reserved.
    pub const DRIVER_MASK: u8 = 0b1100_0000;
    /// D7 to D6 = 00: class AB, the reset value (§5.3.4 and §5.3.5).
    pub const DRIVER_CLASS_AB: u8 = 0b0000_0000;
    /// D7 to D6 = 11: class D (§5.3.4 and §5.3.5). The driver gains are not valid in this mode
    /// (§5.3.13 note).
    pub const DRIVER_CLASS_D: u8 = 0b1100_0000;
    /// D4 to D2: the PowerTune mode (§5.3.4 and §5.3.5; §2.5.1.1). 011 to 111 are reserved.
    pub const PTM_MASK: u8 = 0b0001_1100;
    /// D4 to D2 = 000: PTM_P3 with 16-bit words, PTM_P4 with 20 bits or more; the reset value and
    /// the highest-performance setting (§2.5.1.1).
    pub const PTM_P3_P4: u8 = 0b0000_0000;
    /// D4 to D2 = 001: PTM_P2 (§2.5.1.1).
    pub const PTM_P2: u8 = 0b0000_0100;
    /// D4 to D2 = 010: PTM_P1, the lowest-power setting (§2.5.1.1).
    pub const PTM_P1: u8 = 0b0000_1000;
}

/// Output driver power control (§5.3.7; `aic32x4.h:76`). Both drivers reset to powered down.
///
/// The de-pop settings in [`HP_STARTUP`] must be written before a driver is powered up and not
/// changed while it is up (§2.2.2.1).
pub const OUTPUT_DRIVER_POWER: Register = Register::new(1, 9);
/// Bits of [`OUTPUT_DRIVER_POWER`].
pub mod output_driver_power {
    /// D5: the left headphone driver, HPL, is powered up (the kernel's "HPL Power" bit,
    /// `aic32x4.c:653`).
    pub const HPL_POWER_UP: u8 = 0b0010_0000;
    /// D4: the right headphone driver, HPR, is powered up (the kernel's "HPR Power" bit,
    /// `aic32x4.c:664`).
    pub const HPR_POWER_UP: u8 = 0b0001_0000;
}

/// Common mode control: the chip's and the headphone drivers' common-mode voltage and the drivers'
/// supply (§5.3.8; `aic32x4.h:77`). The kernel driver writes `0x33` at probe
/// (`aic32x4.c:1145-1150`, with the flags set at `aic32x4.c:1218-1221`).
pub const COMMON_MODE: Register = Register::new(1, 10);
/// Bits of [`COMMON_MODE`].
pub mod common_mode {
    /// D6: the full-chip common mode is 0.75 V instead of 0.9 V. 0.75 V is required when the analog
    /// supply is below 1.8 V (§3.5).
    pub const FULL_CHIP_0V75: u8 = 0b0100_0000;
    /// D5 to D4: the output common mode of HPL and HPR (`AIC32X4_HP_CMMODE`, `aic32x4.h:125`).
    pub const HP_COMMON_MODE_MASK: u8 = 0b0011_0000;
    /// D5 to D4 = 00: the same as the full-chip common mode, the reset value.
    pub const HP_COMMON_MODE_SAME_AS_CHIP: u8 = 0b0000_0000;
    /// D5 to D4 = 01: 1.25 V.
    pub const HP_COMMON_MODE_1V25: u8 = 0b0001_0000;
    /// D5 to D4 = 10: 1.5 V.
    pub const HP_COMMON_MODE_1V5: u8 = 0b0010_0000;
    /// D5 to D4 = 11: 1.65 V, or 1.5 V if [`FULL_CHIP_0V75`] is set. The value the kernel driver
    /// writes.
    pub const HP_COMMON_MODE_1V65: u8 = 0b0011_0000;
    /// D1: the output stage of HPL and HPR is powered from LDOIN instead of AVDD
    /// (`AIC32X4_LDOIN2HP`, `aic32x4.h:124`). That is what allows the higher common modes (§2.2.2).
    pub const HP_POWERED_FROM_LDOIN: u8 = 0b0000_0010;
    /// D0: with
    /// [`HP_POWERED_FROM_LDOIN`] set, the LDOIN supply is 1.8 V to 3.6 V instead of 1.5 V to 1.95 V (`AIC32X4_LDOIN_18_36`,
    /// `aic32x4.h:123`).
    pub const LDOIN_RANGE_1V8_TO_3V6: u8 = 0b0000_0001;
}

/// HPL routing selection: what feeds the left headphone driver (§5.3.10; `aic32x4.h:78`). Resets to
/// nothing routed.
pub const HPL_ROUTING: Register = Register::new(1, 12);
/// Bits of [`HPL_ROUTING`].
pub mod hpl_routing {
    /// D3: the left DAC is routed to HPL (the kernel's "L_DAC Switch", `aic32x4.c:619`).
    pub const LEFT_DAC: u8 = 0b0000_1000;
    /// D2: the analog input IN1L is routed to HPL (the kernel's "IN1_L Switch", `aic32x4.c:620`).
    pub const IN1L: u8 = 0b0000_0100;
}

/// HPR routing selection: what feeds the right headphone driver (§5.3.11; `aic32x4.h:79`). Resets
/// to nothing routed.
pub const HPR_ROUTING: Register = Register::new(1, 13);
/// Bits of [`HPR_ROUTING`].
pub mod hpr_routing {
    /// D4: the negative terminal of the left DAC is routed to HPR, for a differential mono output
    /// between HPL and HPR (§2.2.2.2).
    pub const LEFT_DAC_NEGATIVE: u8 = 0b0001_0000;
    /// D3: the right DAC is routed to HPR (the kernel's "R_DAC Switch", `aic32x4.c:624`).
    pub const RIGHT_DAC: u8 = 0b0000_1000;
    /// D2: the analog input IN1R is routed to HPR (the kernel's "IN1_R Switch", `aic32x4.c:625`).
    pub const IN1R: u8 = 0b0000_0100;
}

/// HPL driver gain setting (§5.3.13; `aic32x4.h:82`). Resets to muted, 0 dB.
pub const HPL_GAIN: Register = Register::new(1, 16);
/// HPR driver gain setting (§5.3.14; `aic32x4.h:83`). Resets to muted, 0 dB.
pub const HPR_GAIN: Register = Register::new(1, 17);
/// The layout [`HPL_GAIN`] and [`HPR_GAIN`] share.
pub mod hp_gain {
    /// D6: the driver is muted (§5.3.13 and §5.3.14; `HP_DRIVER_MUTE`, `aic32x4.h:171`).
    ///
    /// A driver cannot be muted from the -6 dB setting: go to -5 dB first (§5.3.13 note; §2.2.2
    /// footnote). Mute and unmute only once the gain has settled, which
    /// [`HP_GAIN_FLAGS`](super::HP_GAIN_FLAGS) reports (§2.2.2.1).
    pub const MUTE: u8 = 0b0100_0000;
    /// D5 to D0: the gain in decibels as a 6-bit two's complement number (§5.3.13 and §5.3.14). The
    /// kernel's "HP Driver Gain Volume" control covers the same field and range
    /// (`aic32x4.c:100-101` and `:522-524`).
    pub const GAIN_MASK: u8 = 0b0011_1111;
    /// The lowest setting, -6 dB. Everything below it, `0b10_0000` to `0b11_1001`, is reserved
    /// (§5.3.13 and §5.3.14).
    pub const GAIN_MIN_MINUS_6_DB: u8 = 0b0011_1010;
    /// 0 dB, the reset value (§5.3.13 and §5.3.14).
    pub const GAIN_ZERO_DB: u8 = 0b0000_0000;
    /// The highest setting, +29 dB. 30 and 31 are reserved (§5.3.13 and §5.3.14).
    pub const GAIN_MAX_PLUS_29_DB: u8 = 0b0001_1101;
}

/// Headphone driver startup control: the de-pop settings (§5.3.16; `aic32x4.h:86`). Resets to 0,
/// everything off.
///
/// The kernel driver writes `0x1D` for a stream (`HP_AMP_SOFT_ROUTE_STARTUP_DELAY`,
/// `aic32x4.h:177`, `aic32x4.c:961-967`): no soft routing, slow charging for 3 time constants,
/// 6 kOhm. With its "Ignore Ramp Up" control on it writes `0x01`, slow charging off
/// (`aic32x4.h:178`).
pub const HP_STARTUP: Register = Register::new(1, 20);
/// Bits of [`HP_STARTUP`].
pub mod hp_startup {
    /// D7 to D6: the step time of the soft routing, the gradual release of attenuation when a
    /// signal is first routed to a driver (§2.2.2.1).
    pub const SOFT_ROUTING_MASK: u8 = 0b1100_0000;
    /// D7 to D6 = 00: 0 ms, soft routing off; the reset value.
    pub const SOFT_ROUTING_OFF: u8 = 0b0000_0000;
    /// D7 to D6 = 01: 50 ms per step.
    pub const SOFT_ROUTING_50_MS: u8 = 0b0100_0000;
    /// D7 to D6 = 10: 100 ms per step.
    pub const SOFT_ROUTING_100_MS: u8 = 0b1000_0000;
    /// D7 to D6 = 11: 200 ms per step.
    pub const SOFT_ROUTING_200_MS: u8 = 0b1100_0000;
    /// D5 to D2: how long the drivers charge the output capacitor slowly at power-up, as a code for
    /// a number N of time constants of Rpop and an assumed 47 uF capacitor (§2.2.2.1).
    ///
    /// Code 0 turns slow charging off. Codes 1 to 15 mean N = 0.5, 0.625, 0.75, 0.875, 1, 2, 3, 4,
    /// 5, 6, 7, 8, 16, 24 and 32; the last three are not valid with the 25 kOhm resistor. (The
    /// register description prints code 3 as 0.725; the table on p. 17 has 0.75.) No signal reaches
    /// the output during the slow charge (§2.2.2.1).
    pub const SLOW_CHARGE_MASK: u8 = 0b0011_1100;
    /// Bit position of the slow-charge code.
    pub const SLOW_CHARGE_SHIFT: u8 = 2;
    /// D1 to D0: the resistor Rpop the slow charge goes through. 11 is reserved.
    ///
    /// Only one value is named here, because the guide contradicts itself on the other two: the
    /// register description has 00 = 25 kOhm and 10 = 2 kOhm, while Table 2-3 has 00 = 2 kOhm and
    /// 10 = 25 kOhm.
    pub const RPOP_MASK: u8 = 0b0000_0011;
    /// D1 to D0 = 01: Rpop is 6 kOhm. Both places in the guide agree on this one (Table 2-3), and
    /// it is the value the kernel driver uses (`aic32x4.h:177`).
    pub const RPOP_6K: u8 = 0b0000_0001;
}

/// DAC analog gain control flags: whether the headphone driver gains and the bypass volumes have
/// reached their programmed values (§5.3.25). Read-only, plain status, not sticky.
///
/// This is the register to poll before unmuting the DACs after a driver power-up (§2.2.2.1, step 7;
/// §2.4.6). The kernel driver calls it `AIC32X4_GAIN_APPLIED` (`aic32x4.h:95`) and polls it every
/// 100 ms, at most 22 times (`aic32x4.c:352-364`).
pub const HP_GAIN_FLAGS: Register = Register::new(1, 63);
/// Bits of [`HP_GAIN_FLAGS`].
pub mod hp_gain_flags {
    /// D7: HPL's applied gain equals the programmed one. With
    /// [`HPR_GAIN_APPLIED`] this is the kernel's `DAC_GAIN_STEREO_APPLIED`,
    /// `0xC0` (`aic32x4.h:174`).
    pub const HPL_GAIN_APPLIED: u8 = 0b1000_0000;
    /// D6: HPR's applied gain equals the programmed one (`DAC_GAIN_MONO_APPLIED`, `aic32x4.h:175`).
    pub const HPR_GAIN_APPLIED: u8 = 0b0100_0000;
    /// D3: the IN1L to HPL bypass volume has reached its programmed value.
    pub const IN1L_VOLUME_APPLIED: u8 = 0b0000_1000;
    /// D2: the IN1R to HPR bypass volume has reached its programmed value.
    pub const IN1R_VOLUME_APPLIED: u8 = 0b0000_0100;
}

/// Reference power-up configuration: how fast the analog reference comes up (§5.3.29;
/// `aic32x4.h:97`). Resets to 0, slow.
pub const REFERENCE_POWER_UP: Register = Register::new(1, 123);
/// Bits of [`REFERENCE_POWER_UP`].
pub mod reference_power_up {
    /// D2 to D0: the whole configuration field.
    pub const MODE_MASK: u8 = 0b0000_0111;
    /// D2: the reference is forced on now instead of coming up with the first analog block. OR it
    /// with one of the times below.
    pub const FORCE: u8 = 0b0000_0100;
    /// D1 to D0 = 00: the reference powers up slowly, the reset value.
    pub const TIME_SLOW: u8 = 0b0000_0000;
    /// D1 to D0 = 01: the reference powers up in 40 ms. The value the kernel driver writes at probe
    /// (`REF_POWERUP_DELAY`, `aic32x4.h:169`, `aic32x4.c:1185`).
    pub const TIME_40_MS: u8 = 0b0000_0001;
    /// D1 to D0 = 10: the reference powers up in 80 ms.
    pub const TIME_80_MS: u8 = 0b0000_0010;
    /// D1 to D0 = 11: the reference powers up in 120 ms.
    pub const TIME_120_MS: u8 = 0b0000_0011;
}

// ---------------------------------------------------------------------------------------------
// Page 0: the beep generator (§2.4.5.1; §5.2.65 to §5.2.73). Only processing block PRB_P25 has it
// (Table 2-13). The kernel driver never names these.
// ---------------------------------------------------------------------------------------------

/// Beep generator register 1: the enable bit and the left channel's beep volume (§5.2.65). Resets
/// to 0: disabled, 0 dB.
pub const BEEP_LEFT: Register = Register::new(0, 71);
/// Bits of [`BEEP_LEFT`].
pub mod beep_left {
    /// D7: 1 starts the beep. The chip clears it itself once the programmed length has played
    /// ("This bit will self clear after the beep has been generated", §5.2.65); writing 0 stops the
    /// beep early (§2.4.5.1). Section 2.4.5.1 calls it "Page 1, Register 71, D(7)", but the
    /// register map and TI's own script in the same section (`w 30 47 80` after `w 30 00 00`) have
    /// it in page 0.
    pub const ENABLE: u8 = 0b1000_0000;
    /// D5 to D0: the left channel's beep volume, 0 dB (code 0, the generator's maximum) to -63 dB
    /// (code 63) in 1 dB steps. D6 is reserved and written 0.
    pub const VOLUME_MASK: u8 = 0b0011_1111;
}

/// Beep generator register 2: the master volume setting and the right channel's beep volume
/// (§5.2.66). Resets to 0.
pub const BEEP_RIGHT: Register = Register::new(0, 72);
/// Bits of [`BEEP_RIGHT`].
pub mod beep_right {
    /// D7 to D6: which channel's volume setting applies to both. 11 is reserved.
    pub const MASTER_MASK: u8 = 0b1100_0000;
    /// D7 to D6 = 00: each channel has its own volume, the reset value.
    pub const MASTER_INDEPENDENT: u8 = 0b0000_0000;
    /// D7 to D6 = 01: the left channel takes the right channel's volume.
    pub const MASTER_LEFT_FOLLOWS_RIGHT: u8 = 0b0100_0000;
    /// D7 to D6 = 10: the right channel takes the left channel's volume. The driver uses this, so
    /// one volume (in [`BEEP_LEFT`](super::BEEP_LEFT)) sets both and the right field, which the
    /// register table marks read-only although §2.4.5.1 says it controls the right volume, never
    /// matters.
    pub const MASTER_RIGHT_FOLLOWS_LEFT: u8 = 0b1000_0000;
    /// D5 to D0: the right channel's beep volume, coded as in
    /// [`beep_left::VOLUME_MASK`](super::beep_left::VOLUME_MASK).
    pub const VOLUME_MASK: u8 = 0b0011_1111;
}

/// Beep generator register 3: the beep length in samples, bits 23 to 16 (§5.2.67). The length
/// registers reset to 238 samples, about 5 ms at 48 kHz: a key click.
pub const BEEP_LENGTH_HIGH: Register = Register::new(0, 73);
/// Beep generator register 4: the beep length, bits 15 to 8 (§5.2.68).
pub const BEEP_LENGTH_MID: Register = Register::new(0, 74);
/// Beep generator register 5: the beep length, bits 7 to 0 (§5.2.69). Resets to `0xEE`.
pub const BEEP_LENGTH_LOW: Register = Register::new(0, 75);
/// Beep generator register 6: the sine coefficient, bits 15 to 8 (§5.2.70). Resets to `0x10`.
pub const BEEP_SIN_MSB: Register = Register::new(0, 76);
/// Beep generator register 7: the sine coefficient, bits 7 to 0 (§5.2.71). Resets to `0xD8`.
pub const BEEP_SIN_LSB: Register = Register::new(0, 77);
/// Beep generator register 8: the cosine coefficient, bits 15 to 8 (§5.2.72). Resets to `0x7E`.
pub const BEEP_COS_MSB: Register = Register::new(0, 78);
/// Beep generator register 9: the cosine coefficient, bits 7 to 0 (§5.2.73). Resets to `0xE3`.
pub const BEEP_COS_LSB: Register = Register::new(0, 79);

// None of the beep registers is clear-on-read: register 71 is read to see whether a beep plays.
const _: () = {
    assert!(!BEEP_LEFT.is_clear_on_read());
};

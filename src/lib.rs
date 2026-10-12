#![doc = include_str!("../README.md")]
//!
//! ## Calling order
//!
//! From a reset: [`Tlv320dac3203::reset`]; the supply and the reference
//! ([`configure_supply`](Tlv320dac3203::configure_supply),
//! [`set_reference_power_up`](Tlv320dac3203::set_reference_power_up)); the routing and the
//! headphone gain ([`set_dac_routing`](Tlv320dac3203::set_dac_routing),
//! [`set_headphone_gain`](Tlv320dac3203::set_headphone_gain)); then
//! [`power_up`](Tlv320dac3203::power_up), which programs the clocks, the interface and the DACs,
//! powers everything up in the order that avoids a pop, waits for the headphone drivers and
//! unmutes. [`set_volume`](Tlv320dac3203::set_volume) and
//! [`set_dac_mute`](Tlv320dac3203::set_dac_mute) are for the time in between, and
//! [`power_down`](Tlv320dac3203::power_down) ends it. A caller that waits asynchronously can split
//! the start into [`power_up_start`](Tlv320dac3203::power_up_start),
//! [`headphone_gain_applied`](Tlv320dac3203::headphone_gain_applied) and
//! [`power_up_finish`](Tlv320dac3203::power_up_finish). The single steps of both sequences are
//! public, for a board that needs another order ([`playback`]).
//!
//! What the outputs drive (a jack, a mux, a speaker amplifier), the reset pin and the master clock
//! are the board's, not this chip's: a board crate owns those pins and any wait before an
//! amplifier is switched on.
//!
//! ## Pages
//!
//! The chip groups its registers into pages of 128, and address 0 of every page is the same
//! page-select register (§5.2.1). Page 0 (clocks, interface, flags, digital volume) and page 1
//! (power, output drivers, routing, analog gain) hold the control registers; pages 44 to 46 and 62
//! to 64 hold the two copies of the filter coefficients (Table 5-1). A register is therefore a
//! [`Register`]`{ page, addr }`; the map is [`register`].
//!
//! Every access selects its page first, so the driver keeps no state about the chip: an access is
//! right whatever happened before it, a reset the driver did not see or an operation that was
//! interrupted. The price is one short write per access. Only [`probe`](Tlv320dac3203::probe)
//! reads the page instead of selecting one, so that it writes nothing to a chip that has not
//! identified. The README's "Ownership" section has the rules for sharing the chip.
//!
//! ## Clear-on-read registers
//!
//! Page 0 registers 42, 44 and 45 hold sticky flags that a read clears
//! ([`register::CLEAR_ON_READ`]). Every read method refuses them with [`Error::ClearOnRead`], so
//! no latched event is lost by accident; registers 43 and 46 hold the same flags without the side
//! effect. No method reads more than one register per address sent, since the chip's
//! auto-increment (§2.8.1) would run a longer read into them.
//!
//! ## Where the DAC3203 differs from what `tlv320aic32x4` assumes
//!
//! The kernel driver was written for the TLV320AIC32x4 codecs, which have an analog record path and
//! line outputs. The DAC3203 has neither: two DACs, two headphone drivers, an analog bypass and a
//! digital microphone input (chapter 1). The driver's register numbers and bit positions agree with
//! the guide for everything the playback path uses; the differences are registers the driver
//! touches that this chip does not document, or documents differently. This crate leaves all of
//! them out of its map and never writes them.
//!
//! - **Page 0, registers 18 and 19** (`AIC32X4_NADC`, `AIC32X4_MADC`, `aic32x4.h:32-33`). The
//!   driver writes record-path dividers with every stream (`aic32x4.c:905-912`) and switches their
//!   power bits with the bias level (`aic32x4.c:1024-1029` and `:1045-1050`).
//!   The guide's register descriptions call both reserved, "write only default values" (§5.2.18 and
//!   §5.2.19). Its clock chapter and the datasheet's register list do name them, as the dividers
//!   that clock the digital microphone path (Table 2-20; SLOS756B Table 4). Either way they have no
//!   part in playback.
//! - **Page 0, register 20** (`AIC32X4_AOSR`, `aic32x4.h:34`), written with 128 for the 48 kHz rate
//!   (`aic32x4.c:915`). That is its reset value; the guide marks the register read-only (§5.2.20).
//!   The one exception to "out of the map": it is there as [`register::AOSR`] to be read as an
//!   identity check, and is never written.
//! - **Page 0, register 81** (`AIC32X4_ADCSETUP`, `aic32x4.h:56`). At probe the driver sets and
//!   clears bits 7 and 6, a workaround for the first capture on the codecs it was written for
//!   (`aic32x4.c:1168-1176`). On this chip those bits power the two digital microphone channels
//!   (§5.2.74), so the pulse switches them on and off once.
//! - **Page 0, registers 82 to 84 and 86 to 100**: the driver's record volume and AGC controls
//!   (`aic32x4.c:537-566`). 82 to 84 exist here as digital microphone controls (§5.2.75 to
//!   §5.2.77); 86 to 100 are reserved (§5.2.79). They are only written if userspace sets those
//!   controls.
//! - **Page 1, registers 54 and 57** (`AIC32X4_LMICPGANIN`, `AIC32X4_RMICPGANIN`,
//!   `aic32x4.h:89` and `:91`). The driver writes
//!   `0x40` to both at probe, the microphone amplifiers' input routing (`aic32x4.c:1152-1164`).
//!   Registers 52 to 57 are reserved on this chip (§5.3.22). The same goes for the input mixer
//!   switches in 52 and 55
//!   (`aic32x4.c:637-645`) and the microphone amplifier gains in 59 and 60 (`aic32x4.c:532-533` and
//!   `:542-543`; §5.3.24), which only userspace controls write.
//! - **The line outputs**: page 1, registers 14 and 15 (routing, `aic32x4.c:629` and `:633`), 18
//!   and 19 (gain and mute, `aic32x4.c:525-526` and `:530-531`) and bits 3 and 2 of register 9
//!   (power, `aic32x4.c:658` and `:668`). All reserved here (§5.3.12, §5.3.15 and §5.3.7), and only
//!   written if userspace sets the "LO" controls.
//! - **Page 0, register 53** is `AIC32X4_DOUTCTL` to the driver (`aic32x4.h:44`). On this chip the
//!   pin is MFP2 and the register is
//!   [`register::MFP2_CONTROL`]; the driver uses it as a general-purpose output (`aic32x4.c:1179`).
//! - **PLL multiplier R.** The driver always programs R = 1 (`aic32x4.c:880-882`), which is fine,
//!   but note for anything written from its divider table that this chip only allows R = 1 to 4 and
//!   J = 4 to 63 (§5.2.6 and §5.2.7).
//! - **Pages.** The driver's register range ends at page 46 (`aic32x4.h:99`), so the second
//!   coefficient buffer in pages 62 to 64 is out of its reach, and it never enables adaptive
//!   filtering. Its coefficient control therefore only takes effect while the DAC is powered down
//!   (§2.4.5.3).
//! - **Misleading names, right values.** The driver shifts the interface format with
//!   `AIC32X4_PLLJ_SHIFT` and the word length with `AIC32X4_DOSRMSB_SHIFT`
//!   (`aic32x4.h:130-131`, used at `aic32x4.c:819-833` and
//!   `:929-935`). The shifts, 6 and 4, are the right ones for [`register::INTERFACE_1`]; only the
//!   names belong to other registers.
//! - **Sticky flags and headset detection.** The driver reads none of the sticky flag registers and
//!   does not use the chip's headset detection (on the Echo Dot 2 the jack is detected by a SoC
//!   GPIO).
#![cfg_attr(not(test), no_std)]

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

#[cfg(feature = "beep")]
pub mod beep;
pub mod clock;
pub mod dac;
pub mod flags;
pub mod headphone;
pub mod interface;
pub mod pins;
pub mod playback;
pub mod probe;
pub mod register;
pub mod status;
pub mod supply;
#[cfg(test)]
mod test_support;

#[cfg(feature = "beep")]
pub use beep::{
    BEEP_GAIN_FLAGS_POLL_LIMIT, BEEP_GAIN_FLAGS_POLL_MS, BeepConfig, BeepLength, BeepTone,
    BeepVolume,
};
pub use clock::{
    ClockConfig, CodecInput, Divider, DividerConfig, Divisor, Dosr, PLL_STARTUP_MS, Pll, PllD,
    PllInput, PllJ, PllP, PllR, PllRange,
};
pub use dac::{
    DAC_POWER_DOWN_POLL_LIMIT, DAC_POWER_DOWN_POLL_MS, DAC_POWER_UP_POLL_LIMIT,
    DAC_POWER_UP_POLL_MS, DacData, DacSetup, DacVolume, ProcessingBlock, SoftStepping,
};
pub use flags::PlaybackFlags;
pub use headphone::{
    HEADPHONE_GAIN_POLL_LIMIT, HEADPHONE_GAIN_POLL_MS, HeadphoneGain, HeadphoneStartup, Rpop,
    SlowCharge, SoftRoutingStep,
};
pub use interface::{BclkDividerInput, ClockDirection, Format, InterfaceConfig, WordLength};
pub use pins::{Mfp2Config, Mfp2Function, SclkMfp3Function};
pub use playback::PlaybackConfig;
pub use probe::Probe;
pub use register::Register;
pub use status::{AnalogStatus, DigitalStatus, PowerFlags, ReservedCode};
pub use supply::{
    CommonMode, HeadphoneCommonMode, HeadphoneSupply, LdoVoltage, ReferenceChargeTime,
    ReferencePowerUp, SupplyConfig,
};

use register::PAGE_SELECT;

/// How long [`Tlv320dac3203::reset`] waits after the software reset, in milliseconds.
///
/// After a reset the registers take their default values "within 1ms", and during that time no
/// block may be powered up and no coefficient register read or written (§3.2). The kernel driver
/// waits the same 1 ms, calling it the "required delay after reset" (`aic32x4.c:1129-1131`).
pub const RESET_SETTLE_MS: u32 = 1;

/// The DAC's 7-bit I2C address. The chip answers to the one address
/// `0011000` (§2.8.1), so every TLV320DAC3203 is at `0x18`; which bus it is on is the board's.
pub const ADDRESS: u8 = 0x18;

/// Errors from the driver. `Clone` and `Copy` follow the bus error's. Non-exhaustive, so that a
/// new kind of failure is not a breaking change for the boards that match on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error<E> {
    /// The I2C transaction failed.
    Bus(E),
    /// The read was refused because reading this register clears its sticky flags
    /// ([`register::CLEAR_ON_READ`]). Nothing was sent to the chip.
    ClearOnRead(Register),
    /// A status register did not show the expected flags within the bounded wait. The register
    /// that was polled is given; the methods that wait say how long and what state they leave.
    Timeout(Register),
}

/// Names the failure. A bus error is shown with its `Debug` form, the one representation every
/// `embedded-hal` bus error has; registers by page and address, as the datasheet names them.
impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Bus(error) => write!(f, "TLV320DAC3203 I2C error: {error:?}"),
            Self::ClearOnRead(r) => write!(
                f,
                "TLV320DAC3203 read of clear-on-read register page {} / {} refused",
                r.page(),
                r.addr()
            ),
            Self::Timeout(r) => write!(
                f,
                "TLV320DAC3203 timed out polling page {} / {}",
                r.page(),
                r.addr()
            ),
        }
    }
}

impl<E: core::fmt::Debug> core::error::Error for Error<E> {}

/// A choice among the two channels, left and right, for the methods that switch something per
/// channel (power, mute). What a `true` means is the method's to say.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Channels {
    /// The left channel: the left DAC, or the left headphone driver HPL.
    pub left: bool,
    /// The right channel: the right DAC, or the right headphone driver HPR.
    pub right: bool,
}

impl Channels {
    /// Neither channel.
    pub const NONE: Self = Self {
        left: false,
        right: false,
    };
    /// The left channel only.
    pub const LEFT: Self = Self {
        left: true,
        right: false,
    };
    /// The right channel only.
    pub const RIGHT: Self = Self {
        left: false,
        right: true,
    };
    /// Both channels.
    pub const BOTH: Self = Self {
        left: true,
        right: true,
    };

    /// The other choice: every channel that is not in this one.
    pub const fn inverted(self) -> Self {
        Self {
            left: !self.left,
            right: !self.right,
        }
    }
}

/// A TLV320DAC3203 on an `embedded-hal` I2C bus.
pub struct Tlv320dac3203<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Wraps the bus; talks to nothing until a method is called.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Gives the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// The configured 7-bit address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Reads `reg`, selecting its page first, and refuses the clear-on-read registers
    /// ([`Error::ClearOnRead`], nothing sent).
    ///
    /// For reading back what was programmed, or a register the driver has no method for. The
    /// status snapshots ([`status`]) decode the registers playback uses.
    pub fn read_register(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        self.read(reg)
    }

    /// Resets all registers to their defaults (§5.2.2; §3.1) and waits [`RESET_SETTLE_MS`].
    ///
    /// This is the first step of the guide's setup list (§2.4.6, step 2) and of the kernel driver's
    /// probe (`aic32x4.c:1129-1131`). After it everything is powered down, both DAC channels and
    /// both headphone drivers are muted and nothing is routed.
    ///
    /// The chip's reset pin is a GPIO of the board and not part of this driver. The guide asks for
    /// a hardware reset after the supplies come up and allows software resets from then on (§3.1).
    pub fn reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.software_reset()?;
        delay.delay_ms(RESET_SETTLE_MS);
        Ok(())
    }

    /// Resets all registers to their defaults (§5.2.2; §3.1), without waiting.
    /// [`reset`](Self::reset) is this plus the wait.
    ///
    /// The caller has to let [`RESET_SETTLE_MS`] pass before the next access: until then nothing
    /// may be powered up and no coefficient register may be read or written (§3.2).
    pub fn software_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write(register::SOFTWARE_RESET, register::software_reset::RESET)
    }

    /// Reads the page-select register: the page the chip has selected.
    fn read_page_select(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut page = [0u8];
        self.i2c
            .write_read(self.address, &[PAGE_SELECT], &mut page)
            .map_err(Error::Bus)?;
        Ok(page[0])
    }

    /// Writes the page-select register. Every access does this first: the driver remembers no
    /// page, so an access is right whatever happened to the chip before it (a reset the driver
    /// did not see, a cancelled operation).
    fn select_page(&mut self, page: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[PAGE_SELECT, page])
            .map_err(Error::Bus)
    }

    /// Writes `value` to `reg`, selecting its page first.
    fn write(&mut self, reg: Register, value: u8) -> Result<(), Error<I2C::Error>> {
        self.select_page(reg.page())?;
        self.i2c
            .write(self.address, &[reg.addr(), value])
            .map_err(Error::Bus)
    }

    /// Reads `reg`, selecting its page first. Refuses clear-on-read registers.
    fn read(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        // Refused before the page select, so a refused read sends nothing at all.
        if reg.is_clear_on_read() {
            return Err(Error::ClearOnRead(reg));
        }
        self.select_page(reg.page())?;
        self.read_selected(reg)
    }

    /// Reads `reg` on the page the chip has selected, without selecting one: for a caller that
    /// has just read or selected that page itself ([`probe`](Self::probe)). Refuses
    /// clear-on-read registers.
    fn read_selected(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        if reg.is_clear_on_read() {
            return Err(Error::ClearOnRead(reg));
        }
        let mut value = [0u8];
        self.i2c
            .write_read(self.address, &[reg.addr()], &mut value)
            .map_err(Error::Bus)?;
        Ok(value[0])
    }

    /// Replaces the bits of `reg` selected by `mask` with the same bits of `value` and leaves the
    /// others as they are. Always writes, so the bus traffic does not depend on the old value.
    fn modify(&mut self, reg: Register, mask: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        let old = self.read(reg)?;
        self.write(reg, (old & !mask) | (value & mask))
    }

    /// Reads `reg` until the bits selected by `mask` equal `value`, waiting `interval_ms` between
    /// two reads and at most `waits` times, so that it returns in bounded time: `waits` + 1 reads
    /// at the most, the last one after the last wait. Fails with [`Error::Timeout`] if the bits
    /// never matched.
    fn wait_for<D: DelayNs>(
        &mut self,
        reg: Register,
        mask: u8,
        value: u8,
        interval_ms: u32,
        waits: u32,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        for attempt in 0..=waits {
            if self.read(reg)? & mask == value {
                return Ok(());
            }
            if attempt < waits {
                delay.delay_ms(interval_ms);
            }
        }
        Err(Error::Timeout(reg))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins the page-select write sequence: every access selects its page first, also one in the
    /// page the access before it used.
    #[test]
    fn every_access_selects_its_page_first() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x01]),
            Transaction::write(ADDRESS, vec![16, 0x40]),
            // The same page again: selected again, as the driver remembers no page.
            Transaction::write(ADDRESS, vec![0x00, 0x01]),
            Transaction::write(ADDRESS, vec![17, 0x40]),
            // The reset register is in page 0.
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
            Transaction::write(ADDRESS, vec![1, 0x01]),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
            Transaction::write(ADDRESS, vec![4, 0x03]),
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        dac.write(register::HPL_GAIN, register::hp_gain::MUTE)
            .unwrap();
        dac.write(register::HPR_GAIN, register::hp_gain::MUTE)
            .unwrap();
        dac.software_reset().unwrap();
        dac.write(register::CLOCK_MUX, 0x03).unwrap();
        dac.release().done();
    }

    /// Pins the owned read: the page select once, then one write-read per register; a
    /// clear-on-read register is refused with nothing sent.
    #[test]
    fn read_register_selects_the_page_and_refuses_sticky_flags() {
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
            Transaction::write_read(ADDRESS, vec![27], vec![0x04]),
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
            Transaction::write_read(ADDRESS, vec![33], vec![0x00]),
            // The refused read sends nothing, not even its page select.
        ];
        let mut dac = Tlv320dac3203::new(Mock::new(&expectations), ADDRESS);
        assert_eq!(dac.read_register(register::INTERFACE_1), Ok(0x04));
        assert_eq!(dac.read_register(register::INTERFACE_6), Ok(0x00));
        assert_eq!(
            dac.read_register(register::STICKY_FLAGS_2),
            Err(Error::ClearOnRead(register::STICKY_FLAGS_2))
        );
        dac.release().done();
    }
}

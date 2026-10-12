//! Starting and stopping playback: the order in which the clocks, the DAC channels and the
//! headphone drivers come up and go down.
//!
//! # Where the order comes from
//!
//! Two sources, which agree on everything that matters:
//!
//! - **TI's guide.** The setup list in §2.4.6 (step 2) and the pop-free sequence in §2.2.2.1:
//!   reset; clocks; word length and processing block; supply settings; de-pop settings; routing;
//!   driver unmuted with its gain; driver powered; wait on the gain flags of page 1, register 63;
//!   DAC unmuted. The datasheet's own "Typical Application" (SLOS756B §8.2) shows the circuit and
//!   no register sequence, and sends the reader to the guide.
//! - **The stock kernel** of the 2nd-generation Echo Dot (`tlv320aic32x4`; crate documentation,
//!   "Sources"). Read in source; both sequences ran on that board from bare metal and the speaker
//!   sounded (2026-10-06, 16 kHz on the master clock alone; 48 kHz everyday playback since).
//!
//! The kernel splits the work in two. What it writes once at probe (supply, pins, reference) is
//! [`Tlv320dac3203::reset`], [`configure_supply`](Tlv320dac3203::configure_supply),
//! [`configure_mfp2`](Tlv320dac3203::configure_mfp2),
//! [`set_sclk_mfp3`](Tlv320dac3203::set_sclk_mfp3) and
//! [`set_reference_power_up`](Tlv320dac3203::set_reference_power_up) here. What it writes for
//! every stream is [`Tlv320dac3203::power_up`], and what it writes when the stream ends is
//! [`Tlv320dac3203::power_down`]. The crate's README has an example of the whole.
//!
//! # What is not in the sequence
//!
//! - **Writes the kernel makes for the codecs it was written for**: the record-path dividers and
//!   oversampling ratio (page 0, registers 18 to 20), the record channel power pulse (register
//!   81) and the microphone amplifier routing (page 1, registers 54 and 57). On this chip they
//!   configure the unused digital microphone path or hit reserved registers (crate
//!   documentation, "Where the DAC3203 differs").
//! - **The kernel's standby step at probe**, which clears the power bits of the PLL and the
//!   dividers (`aic32x4.c:1038-1066` and `:1166`). Directly after a reset they are clear already.
//! - **Filter coefficients.** The kernel's 117-byte `biquad coefficients` control does no
//!   equalising (the bytes Amazon's software writes only invert the polarity), so nothing is
//!   written to the coefficient pages and the filters stay flat.
//! - **Whatever the outputs drive** (an amplifier, a mux): its pins and any wait before an
//!   amplifier is switched on are the board's. See [`Tlv320dac3203::power_up`].

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::clock::{ClockConfig, Divider};
use crate::dac::{DacSetup, ProcessingBlock};
use crate::headphone::HeadphoneStartup;
use crate::interface::InterfaceConfig;
use crate::{Channels, Error, Tlv320dac3203};

/// Everything [`Tlv320dac3203::power_up`] programs for one run of playback.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlaybackConfig {
    /// The serial interface.
    pub interface: InterfaceConfig,
    /// The clock tree. Its PLL and dividers have to be marked powered as far as the sample rate
    /// needs them: `power_up` writes them as given and powers nothing on its own.
    pub clocks: ClockConfig,
    /// The processing block.
    pub processing_block: ProcessingBlock,
    /// What the DACs play and how the digital volume steps.
    pub dac: DacSetup,
    /// The headphone drivers' de-pop settings.
    pub startup: HeadphoneStartup,
    /// The channels to bring up: for each one named, the DAC and the headphone driver are
    /// powered and the driver is unmuted. The drivers of the other channels stay muted and off.
    pub powered: Channels,
    /// The DAC channels to unmute at the end. A channel that is powered but not named here keeps
    /// running muted, which is how a board that feeds a mono speaker from one channel plays that
    /// channel alone.
    pub unmuted: Channels,
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Brings playback up, in the order the stock kernel uses for a stream (read in source, run
    /// on the Echo Dot 2 since 2026-10-06):
    ///
    /// 1. Mute both headphone drivers (page 1, registers 16 and 17, bit D6; `aic32x4.c:869-874`).
    /// 2. Program the serial interface, the clock tree (with the wait for the PLL,
    ///    [`crate::PLL_STARTUP_MS`]), the processing block, the DRC switch (off) and the DAC
    ///    setup (`aic32x4.c:840-842` and `:876-973`).
    /// 3. Write the de-pop settings (page 1, register 20; `aic32x4.c:961-967`).
    /// 4. Unmute the headphone drivers of the channels in `powered` (`aic32x4.c:975-983`).
    /// 5. Power those channels' DACs (page 0, register 63, bits D7 and D6; `aic32x4.c:649` and
    ///    `:660`).
    /// 6. Power those channels' headphone drivers (page 1, register 9; `aic32x4.c:653` and
    ///    `:664`).
    /// 7. Wait for those drivers' gain flags (page 1, register 63), every
    ///    [`crate::HEADPHONE_GAIN_POLL_MS`] and for [`crate::HEADPHONE_GAIN_POLL_LIMIT`] waits at
    ///    most (`aic32x4.c:352-364`).
    /// 8. Unmute the DAC channels in `unmuted` (page 0, register 64; `aic32x4.c:367`).
    ///
    /// Against the guide's two lists (§2.4.6, step 2; §2.2.2.1) this is the same sequence with one
    /// step moved: the guide unmutes the drivers after the DACs are powered, the kernel before.
    /// Everything the guide orders for a reason is kept: the de-pop settings are written before a
    /// driver is powered, the DACs are powered while still muted, and they are unmuted only after
    /// the gain flags.
    ///
    /// Two details differ from the kernel's code while following its order.
    /// The kernel writes the de-pop register just before the DAC setup and not after it; the two
    /// do not depend on each other, and this order keeps the writes of each page together. And
    /// the kernel's three interface writes come from the machine driver's format call, which runs
    /// before step 1 (`mt_soc_machine.c:465-470`).
    ///
    /// # Before calling
    ///
    /// - The supply settings and the reference have to be programmed
    ///   ([`configure_supply`](Self::configure_supply),
    ///   [`set_reference_power_up`](Self::set_reference_power_up)); after a reset nothing analog
    ///   is enabled.
    /// - The DACs have to be routed to the headphone drivers and the drivers' gain set
    ///   ([`set_dac_routing`](Self::set_dac_routing), [`set_headphone_gain`](Self::set_headphone_gain));
    ///   after a reset nothing is routed. The guide routes and sets the gain before a driver is
    ///   powered (§2.2.2.1, steps 3 and 5), and on the stock system userspace has set both through
    ///   the kernel's mixer controls before a stream starts.
    /// - The DAC channels have to be muted and powered down, which they are after
    ///   [`reset`](Self::reset) and after [`power_down`](Self::power_down). This method does not
    ///   mute them itself.
    /// - The master clock has to be running. Without it nothing in the chip powers up and step 7
    ///   ends in [`Error::Timeout`].
    ///
    /// # On failure
    ///
    /// If the gain flags do not come, the method returns [`Error::Timeout`] with the DACs still
    /// muted and everything else powered. The kernel driver unmutes in that case anyway; here the
    /// caller decides, with [`set_dac_mute`](Self::set_dac_mute) or
    /// [`power_down`](Self::power_down).
    ///
    /// # Not this chip: an amplifier after it
    ///
    /// A speaker amplifier's enable, a mux between speaker and jack, and any time to let pass
    /// between setting the chip up and switching an amplifier on are the board's, not properties
    /// of this chip. None of that happens here; the board crate owns those pins and that wait.
    pub fn power_up<D: DelayNs>(
        &mut self,
        playback: &PlaybackConfig,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        self.power_up_start(playback, delay)?;
        self.wait_for_headphone_gain(playback.powered, delay)?;
        self.power_up_finish(playback)
    }

    /// Steps 1 to 6 of [`power_up`](Self::power_up): everything up to the headphone drivers'
    /// power, the same writes in the same order. The wait of step 7 is the caller's: poll
    /// [`headphone_gain_applied`](Self::headphone_gain_applied) every
    /// [`crate::HEADPHONE_GAIN_POLL_MS`], at most [`crate::HEADPHONE_GAIN_POLL_LIMIT`] + 1
    /// times, then call [`power_up_finish`](Self::power_up_finish). For a caller that must not
    /// block that long (up to 2.2 s): an async board awaits its own timer between the polls.
    ///
    /// The only wait in here is the PLL's start-up ([`crate::PLL_STARTUP_MS`]), and only when the
    /// clock tree powers the PLL.
    pub fn power_up_start<D: DelayNs>(
        &mut self,
        playback: &PlaybackConfig,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        // Not a mute at -6 dB gain: "It is not possible to mute HPR while programmed to -6dB"
        // (SLAU434A §5.3.13). The drivers are still off here and TI's own order unmutes them before
        // power-up, so it is harmless; but never rely on the driver mute at that gain. The DAC mute
        // (`set_dac_mute`) is the one that silences the path.
        self.set_headphone_mute(Channels::BOTH)?;

        self.configure_interface(&playback.interface)?;
        self.configure_clocks(&playback.clocks, delay)?;
        self.set_processing_block(playback.processing_block)?;
        self.disable_drc()?;
        self.configure_dac(playback.dac)?;

        self.set_headphone_startup(playback.startup)?;
        self.set_headphone_mute(playback.powered.inverted())?;

        self.set_dac_power(playback.powered)?;
        self.set_headphone_power(playback.powered)
    }

    /// One look at the gain flags of the drivers named in `drivers` (page 1, register 63): whether
    /// all of them report their gain applied. Step 7 of [`power_up`](Self::power_up), one poll
    /// of [`wait_for_headphone_gain`](Self::wait_for_headphone_gain); with no driver named it is
    /// `true` and reads nothing.
    pub fn headphone_gain_applied(&mut self, drivers: Channels) -> Result<bool, Error<I2C::Error>> {
        let flags = crate::headphone::gain_flags(drivers);
        if flags == 0 {
            return Ok(true);
        }
        Ok(self.read(crate::register::HP_GAIN_FLAGS)? & flags == flags)
    }

    /// Step 8 of [`power_up`](Self::power_up): unmutes the DAC channels in `playback.unmuted`.
    /// Call it only once the gain flags are set ([`headphone_gain_applied`](Self::headphone_gain_applied)).
    pub fn power_up_finish(&mut self, playback: &PlaybackConfig) -> Result<(), Error<I2C::Error>> {
        self.set_dac_mute(playback.unmuted.inverted())
    }

    /// Takes playback down, in the order the stock kernel uses when a stream ends (read in source,
    /// run on the Echo Dot 2 since 2026-10-06), with the one wait the guide demands added:
    ///
    /// 1. Mute both DAC channels (page 0, register 64).
    /// 2. Power both headphone drivers down (page 1, register 9).
    /// 3. Power both DAC channels down (page 0, register 63).
    /// 4. Wait until both DACs report powered down
    ///    ([`wait_for_dac_power_down`](Self::wait_for_dac_power_down)). The kernel does not wait
    ///    here; the guide requires it: NDAC and MDAC "must not be powered down" during the DAC's
    ///    shutdown sequence, "or else a proper low power shut-down may not take place" (§2.7).
    /// 5. Power down the BCLK divider, then MDAC, then NDAC, then the PLL. That is the guide's
    ///    order too: MDAC before NDAC (§2.7), and a PLL that feeds them last (§2.7.1).
    ///
    /// A caller that must not block during the wait (up to 100 ms) takes the same steps in three
    /// calls: [`power_down_start`](Self::power_down_start) (steps 1 to 3), its own polls of
    /// [`dacs_powered_down`](Self::dacs_powered_down) (step 4), then
    /// [`power_down_finish`](Self::power_down_finish) (step 5).
    ///
    /// The kernel's first step at a stream's end, driving the MFP2 pin high again
    /// (`aic32x4.c:332-338`), is left out: this method does not know what the pin is used for.
    /// Call [`configure_mfp2`](Self::configure_mfp2) first if it matters.
    ///
    /// The supply settings, the reference, the clock dividers' values, the interface, the routing
    /// and the gains all stay as they are, so [`power_up`](Self::power_up) can follow directly.
    /// The headphone drivers' mute bits are not touched either; `power_up` sets them.
    ///
    /// The guide recommends powering a headphone driver down only once it has settled after its
    /// power-up (§2.2.2.1). This method does not wait for that, so that it can also clean up after
    /// a `power_up` that timed out.
    ///
    /// # On failure
    ///
    /// If the DACs do not report powered down, the method returns [`Error::Timeout`] before step
    /// 5, with the dividers and the PLL still running. A stopped master clock is the likely
    /// cause. [`set_divider_power`](Self::set_divider_power) and
    /// [`set_pll_power`](Self::set_pll_power) switch them off regardless.
    pub fn power_down<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.power_down_start()?;
        self.wait_for_dac_power_down(delay)?;
        self.power_down_finish()
    }

    /// Steps 1 to 3 of [`power_down`](Self::power_down): the DACs muted, the headphone drivers and
    /// the DACs powered down, the same writes in the same order. No wait.
    ///
    /// The wait of step 4 is the caller's: poll [`dacs_powered_down`](Self::dacs_powered_down)
    /// every [`crate::DAC_POWER_DOWN_POLL_MS`], at most [`crate::DAC_POWER_DOWN_POLL_LIMIT`] + 1
    /// times, and call [`power_down_finish`](Self::power_down_finish) only once it is `true`. If
    /// it never is, leave the clocks running, as `power_down` does: stopping NDAC or MDAC during
    /// the DACs' shutdown is what the guide forbids (§2.7).
    pub fn power_down_start(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_dac_mute(Channels::BOTH)?;
        self.set_headphone_power(Channels::NONE)?;
        self.set_dac_power(Channels::NONE)
    }

    /// One look at the DACs' power flags (page 0, register 37): whether both report powered down.
    /// Step 4 of [`power_down`](Self::power_down), one poll of
    /// [`wait_for_dac_power_down`](Self::wait_for_dac_power_down).
    pub fn dacs_powered_down(&mut self) -> Result<bool, Error<I2C::Error>> {
        let powered = crate::register::dac_flags_1::LEFT_DAC_POWERED
            | crate::register::dac_flags_1::RIGHT_DAC_POWERED;
        Ok(self.read(crate::register::DAC_FLAGS_1)? & powered == 0)
    }

    /// Step 5 of [`power_down`](Self::power_down): the BCLK divider, MDAC, NDAC and the PLL
    /// powered down, in that order. Call it only once
    /// [`dacs_powered_down`](Self::dacs_powered_down) is `true`.
    pub fn power_down_finish(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_divider_power(Divider::BclkN, false)?;
        self.set_divider_power(Divider::Mdac, false)?;
        self.set_divider_power(Divider::Ndac, false)?;
        self.set_pll_power(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::{DividerConfig, Divisor};
    use crate::headphone::{Rpop, SlowCharge, SoftRoutingStep};
    use crate::test_support::{BusDelay, wait_ms};
    use crate::{ADDRESS, Tlv320dac3203};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    fn write(register: u8, value: u8) -> Transaction {
        Transaction::write(ADDRESS, vec![register, value])
    }

    fn read(register: u8, value: u8) -> Transaction {
        Transaction::write_read(ADDRESS, vec![register], vec![value])
    }

    /// The README's playback setting, TI's example board (SLAU434A §4.4): a 12.288 MHz master clock
    /// straight into the codec, NDAC 1, MDAC 2, DOSR 128 (48 kHz), the interface at its reset
    /// values, PRB_P1, a slow charge of 5 time constants of 6 kOhm.
    const PLAYBACK: PlaybackConfig = PlaybackConfig {
        interface: InterfaceConfig::RESET,
        clocks: ClockConfig {
            ndac: DividerConfig {
                powered: true,
                divisor: Divisor::new(1).expect("a divisor is 1 to 128"),
            },
            mdac: DividerConfig {
                powered: true,
                divisor: Divisor::new(2).expect("a divisor is 1 to 128"),
            },
            ..ClockConfig::RESET
        },
        processing_block: ProcessingBlock::PRB_P1,
        dac: DacSetup::RESET,
        startup: HeadphoneStartup {
            soft_routing: SoftRoutingStep::Off,
            slow_charge: SlowCharge::N5,
            rpop: Rpop::K6,
        },
        powered: Channels::BOTH,
        unmuted: Channels::BOTH,
    };

    /// Pins the eight steps of `power_up`: drivers muted, interface, clocks (no PLL wait: the
    /// PLL stays off), processing block, DRC off and DAC setup, de-pop, drivers unmuted, DACs on,
    /// drivers on, the wait for the gain flags, then the DACs unmuted.
    #[test]
    fn power_up_writes_the_eight_steps_in_order() {
        let expected = [
            // Step 1: mute both headphone drivers (page 1).
            write(0, 1),
            read(16, 0x00),
            write(0, 1),
            write(16, 0x40),
            write(0, 1),
            read(17, 0x00),
            write(0, 1),
            write(17, 0x40),
            // Step 2: interface, clocks, processing block, DRC off, DAC setup (page 0).
            write(0, 0),
            write(27, 0x00),
            write(0, 0),
            write(28, 0x00),
            write(0, 0),
            write(29, 0x00),
            write(0, 0),
            write(4, 0x00),
            write(0, 0),
            write(5, 0x11),
            write(0, 0),
            write(6, 0x04),
            write(0, 0),
            write(7, 0x00),
            write(0, 0),
            write(8, 0x00),
            write(0, 0),
            write(11, 0x81),
            write(0, 0),
            write(12, 0x82),
            write(0, 0),
            write(13, 0x00),
            write(0, 0),
            write(14, 0x80),
            write(0, 0),
            write(30, 0x01),
            write(0, 0),
            write(60, 0x01),
            write(0, 0),
            write(68, 0x0F),
            write(0, 0),
            read(63, 0x14),
            write(0, 0),
            write(63, 0x14),
            // Step 3: de-pop, N = 5 (code 9) through 6 kOhm (page 1).
            write(0, 1),
            write(20, 0x25),
            // Step 4: unmute both headphone drivers.
            write(0, 1),
            read(16, 0x40),
            write(0, 1),
            write(16, 0x00),
            write(0, 1),
            read(17, 0x40),
            write(0, 1),
            write(17, 0x00),
            // Step 5: both DACs on (page 0).
            write(0, 0),
            read(63, 0x14),
            write(0, 0),
            write(63, 0xD4),
            // Step 6: both headphone drivers on (page 1).
            write(0, 1),
            write(9, 0x30),
            // Step 7: the gain flags are not there at the first look, and there at the second.
            write(0, 1),
            read(63, 0x00),
            wait_ms(100),
            write(0, 1),
            read(63, 0xC0),
            // Step 8: unmute both DACs (page 0). Register 64 resets to 0x0C.
            write(0, 0),
            read(64, 0x0C),
            write(0, 0),
            write(64, 0x00),
        ];
        let mut bus = Mock::new(&expected);
        let mut delay = BusDelay(bus.clone());

        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        dac.power_up(&PLAYBACK, &mut delay).unwrap();

        bus.done();
    }

    /// The bus traffic of `power_down` from a playing state with the PLL on, the DACs still up at
    /// the first look at register 37 and down at the second. With `wait`, the 10 ms wait of the
    /// blocking version sits between the two looks; the split version waits outside the driver.
    fn power_down_traffic(wait: bool) -> Vec<Transaction> {
        let mut traffic = vec![
            // Mute both DACs (page 0).
            write(0, 0),
            read(64, 0x08),
            write(0, 0),
            write(64, 0x0C),
            // Both headphone drivers off (page 1).
            write(0, 1),
            write(9, 0x00),
            // Both DACs off (page 0).
            write(0, 0),
            read(63, 0xD6),
            write(0, 0),
            write(63, 0x16),
            // Register 37: both DACs still up at the first look (the driver flags in it are
            // ignored), down at the second.
            write(0, 0),
            read(37, 0x88),
        ];
        if wait {
            traffic.push(wait_ms(10));
        }
        traffic.extend([
            write(0, 0),
            read(37, 0x22),
            // BCLK divider, MDAC, NDAC, PLL: bit 7 cleared, the values stay.
            write(0, 0),
            read(30, 0x81),
            write(0, 0),
            write(30, 0x01),
            write(0, 0),
            read(12, 0x83),
            write(0, 0),
            write(12, 0x03),
            write(0, 0),
            read(11, 0x85),
            write(0, 0),
            write(11, 0x05),
            write(0, 0),
            read(5, 0xD1),
            write(0, 0),
            write(5, 0x51),
        ]);
        traffic
    }

    /// Pins the power-down sequence from a playing state with the PLL on: DACs muted, drivers
    /// off, DACs off, the wait for the power flags, then the BCLK divider, MDAC, NDAC and the
    /// PLL.
    #[test]
    fn power_down_waits_for_the_dacs_before_it_stops_the_clocks() {
        let expected = power_down_traffic(true);
        let mut bus = Mock::new(&expected);
        let mut delay = BusDelay(bus.clone());

        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        dac.power_down(&mut delay).unwrap();

        bus.done();
    }

    /// The three calls of the split power-down send exactly what `power_down` sends, minus the
    /// wait, which is the caller's.
    #[test]
    fn the_split_power_down_is_the_same_traffic() {
        let expected = power_down_traffic(false);
        let mut bus = Mock::new(&expected);

        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        dac.power_down_start().unwrap();
        assert!(!dac.dacs_powered_down().unwrap());
        assert!(dac.dacs_powered_down().unwrap());
        dac.power_down_finish().unwrap();

        bus.done();
    }

    /// The three steps in a row put the same bytes on the bus as `power_up`, the wait between the
    /// two looks at the gain flags being the caller's.
    #[test]
    fn the_split_power_up_is_the_same_traffic() {
        let expected = [
            write(0, 1),
            read(16, 0x00),
            write(0, 1),
            write(16, 0x40),
            write(0, 1),
            read(17, 0x00),
            write(0, 1),
            write(17, 0x40),
            write(0, 0),
            write(27, 0x00),
            write(0, 0),
            write(28, 0x00),
            write(0, 0),
            write(29, 0x00),
            write(0, 0),
            write(4, 0x00),
            write(0, 0),
            write(5, 0x11),
            write(0, 0),
            write(6, 0x04),
            write(0, 0),
            write(7, 0x00),
            write(0, 0),
            write(8, 0x00),
            write(0, 0),
            write(11, 0x81),
            write(0, 0),
            write(12, 0x82),
            write(0, 0),
            write(13, 0x00),
            write(0, 0),
            write(14, 0x80),
            write(0, 0),
            write(30, 0x01),
            write(0, 0),
            write(60, 0x01),
            write(0, 0),
            write(68, 0x0F),
            write(0, 0),
            read(63, 0x14),
            write(0, 0),
            write(63, 0x14),
            write(0, 1),
            write(20, 0x25),
            write(0, 1),
            read(16, 0x40),
            write(0, 1),
            write(16, 0x00),
            write(0, 1),
            read(17, 0x40),
            write(0, 1),
            write(17, 0x00),
            write(0, 0),
            read(63, 0x14),
            write(0, 0),
            write(63, 0xD4),
            write(0, 1),
            write(9, 0x30),
            write(0, 1),
            read(63, 0x00),
            write(0, 1),
            read(63, 0xC0),
            write(0, 0),
            read(64, 0x0C),
            write(0, 0),
            write(64, 0x00),
        ];
        let mut bus = Mock::new(&expected);
        let mut delay = BusDelay(bus.clone());

        let mut dac = Tlv320dac3203::new(&mut bus, ADDRESS);
        dac.power_up_start(&PLAYBACK, &mut delay).unwrap();
        assert!(!dac.headphone_gain_applied(PLAYBACK.powered).unwrap());
        assert!(dac.headphone_gain_applied(PLAYBACK.powered).unwrap());
        assert!(dac.headphone_gain_applied(Channels::NONE).unwrap());
        dac.power_up_finish(&PLAYBACK).unwrap();

        bus.done();
    }
}

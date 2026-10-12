# tlv320dac3203

[![crates.io](https://img.shields.io/crates/v/tlv320dac3203.svg)](https://crates.io/crates/tlv320dac3203)
[![docs.rs](https://docs.rs/tlv320dac3203/badge.svg)](https://docs.rs/tlv320dac3203)
[![CI](https://github.com/mschoenlaub/tlv320dac3203/actions/workflows/ci.yml/badge.svg)](https://github.com/mschoenlaub/tlv320dac3203/actions/workflows/ci.yml)

`no_std` driver for the TI **TLV320DAC3203**, a stereo audio DAC with two headphone drivers, a PLL
and an I2S/DSP serial interface, controlled over I2C. It depends only on `embedded-hal` 1.0 and
allocates nothing.

- **Typed configuration.** Values with a range are checked when they are built, so in a `const`
  they are checked at compile time. Reserved codes read back from the chip are errors, not panics.
- **Playback in the right order.** `power_up` and `power_down` follow TI's pop-free sequence and
  wait, bounded, for the chip's own flags.
- **Status read back, decoded.** Snapshots of both control pages, decoded into the same types the
  write side takes.
- **Every register and bit cited** to TI's guide, and to the Linux driver where they were
  cross-checked.

## Usage

The example brings the chip from a reset to playing on both headphone outputs, on TI's example
board: a 12.288 MHz master clock, an external analog supply, I2S with the host driving the clocks,
48 kHz (SLAU434A §4.4). With that clock no PLL is needed: 12.288 MHz / (NDAC 1 x MDAC 2 x DOSR
128) = 48 kHz.

```rust
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use tlv320dac3203::{
    Channels, ClockConfig, DacSetup, DacVolume, DividerConfig, Divisor, Error, HeadphoneGain,
    HeadphoneStartup, InterfaceConfig, PlaybackConfig, ProcessingBlock, ReferenceChargeTime,
    ReferencePowerUp, Rpop, SlowCharge, SoftRoutingStep, SupplyConfig, Tlv320dac3203,
};

// Values with a range are checked when they are built. Building them in constants lets the
// compiler do that check, so nothing can fail at run time.
const NDAC: Divisor = Divisor::new(1).expect("a divisor is 1 to 128");
const MDAC: Divisor = Divisor::new(2).expect("a divisor is 1 to 128");
const QUIET: DacVolume = DacVolume::from_db(-30).expect("a volume is -63 dB to +24 dB");
const NORMAL: DacVolume = DacVolume::from_db(-12).expect("a volume is -63 dB to +24 dB");

const PLAYBACK: PlaybackConfig = PlaybackConfig {
    // I2S, 16-bit words, bit clock and word clock from the host: the reset values.
    interface: InterfaceConfig::RESET,
    // The codec clock straight from the MCLK pin, the PLL off, DOSR 128: the reset values, with
    // the two dividers set and powered.
    clocks: ClockConfig {
        ndac: DividerConfig { powered: true, divisor: NDAC },
        mdac: DividerConfig { powered: true, divisor: MDAC },
        ..ClockConfig::RESET
    },
    processing_block: ProcessingBlock::PRB_P1,
    // Each DAC plays its own side; the volume moves one step per sample.
    dac: DacSetup::RESET,
    // De-pop: charge the output capacitors slowly, for 5 time constants of 6 kOhm and 47 uF.
    startup: HeadphoneStartup {
        soft_routing: SoftRoutingStep::Off,
        slow_charge: SlowCharge::N5,
        rpop: Rpop::K6,
    },
    powered: Channels::BOTH,
    unmuted: Channels::BOTH,
};

fn play<I2C: I2c, D: DelayNs>(i2c: I2C, delay: &mut D) -> Result<I2C, Error<I2C::Error>> {
    // The chip has a single I2C address, 0x18.
    let mut dac = Tlv320dac3203::new(i2c, 0x18);

    // Start from a known state. After the reset everything is off, muted and unrouted.
    dac.reset(delay)?;

    // Supply: AVDD comes from outside, so the weak AVDD-to-DVDD connection goes off and the
    // analog blocks are enabled. The reference charges in 40 ms once something analog powers up.
    dac.configure_supply(&SupplyConfig {
        avdd_weakly_connected_to_dvdd: false,
        analog_blocks_enabled: true,
        ..SupplyConfig::RESET
    })?;
    dac.set_reference_power_up(ReferencePowerUp {
        forced: false,
        charge_time: ReferenceChargeTime::Ms40,
    })?;

    // Route each DAC to its headphone driver, set the drivers' gain and the digital volume.
    dac.set_dac_routing(Channels::BOTH)?;
    dac.set_headphone_gain(HeadphoneGain::ZERO_DB, HeadphoneGain::ZERO_DB)?;
    dac.set_volume(QUIET, QUIET)?;

    // Power up: clocks, DACs, headphone drivers. This waits until the drivers have finished
    // their slow start (up to 2.2 s) and then unmutes both DACs. From here on the chip plays
    // what arrives on the serial interface.
    dac.power_up(&PLAYBACK, delay)?;

    // While playing: change the volume, mute and unmute.
    dac.set_volume(NORMAL, NORMAL)?;
    dac.set_dac_mute(Channels::BOTH)?;
    dac.set_dac_mute(Channels::NONE)?;

    // Stop: mute, power the drivers and the DACs down, then the clocks. The configuration
    // stays, so power_up can follow again.
    dac.power_down(delay)?;

    Ok(dac.release())
}
```

Things to know:

- **`power_up` only does its own part.** The supply, the routing and the driver gain have to be set
  before it, and the master clock must run: without it nothing in the chip powers up.
- **Two volumes.** `set_headphone_gain` is the board's output level (-6 to +29 dB), chosen once.
  `set_volume` is the one to change while playing (-63.5 to +24 dB in 0.5 dB steps; `DacVolume`
  counts half decibels, so nothing needs a float).
- **Two mutes.** `set_dac_mute` is the one to use while playing. `set_headphone_mute` mutes a
  driver; a driver cannot be muted from its -6 dB setting (set -5 dB first).
- **`Channels` names the channels a call is about**: `set_dac_mute(Channels::LEFT)` mutes the left
  DAC and unmutes the right one.
- **Waits are bounded.** `power_up` gives up after 2.2 s and returns `Error::Timeout` with the DACs
  still muted; `power_down` gives up after 100 ms and leaves the clocks running.
- **No state about the chip.** Every access selects its page first, and methods that change one
  field of a shared register read it first.

## Reading the chip's state

`read_digital_status` and `read_analog_status` return snapshots of page 0 and page 1, decoded into
the types the write side uses:

```rust
use embedded_hal::i2c::I2c;
use tlv320dac3203::{Error, Tlv320dac3203};

/// Whether the chip reports a DAC channel as powered.
fn is_playing<I2C: I2c>(dac: &mut Tlv320dac3203<I2C>) -> Result<bool, Error<I2C::Error>> {
    let flags = dac.read_digital_status()?.power_flags();
    Ok(flags.dacs.left || flags.dacs.right)
}
```

The reads never touch the sticky flag registers, which a read would clear.

## Ownership

The driver assumes it is the only one writing to the chip while it holds it. Its sequences and
read-modify-writes span several bus transactions, and another writer in between would break them.

- **Taking the chip over** from a previous owner, such as a bootloader, is fine: start with `new`
  and `reset`. `probe` writes nothing, so it can check what answers at the address before anything
  is written to it.
- **One instance per chip.** To use the chip from several threads, tasks or cores, share one
  `Tlv320dac3203` behind a lock (a `Mutex` around the driver, held for a whole method call), not
  the bus: a shared-bus wrapper locks each transaction, and two instances would interleave their
  page selects and sequences.
- **Not beside another driver** of the same chip, such as Linux's `tlv320aic32x4`.
- **Other devices on the same bus** are no problem: the page register is inside this chip.

## Features

- `beep`: the beep generator of processing block PRB_P25. Off by default. It has run on a chip, but
  was never heard there.

## Status

The driver covers what playback needs. It was developed on the 2nd-generation Echo Dot (MediaTek
MT8163, I2C address `0x18`), where it runs that board's everyday playback from bare metal at 48 kHz
on the 9.6 MHz master clock alone (NDAC 1, MDAC 2, DOSR 100, PRB_P9). It has not run on another
board yet, and its PLL configuration has not played audio on a chip.

Left out: the filter coefficients (the biquads stay flat; the DRC can only be switched off),
volume ganging and auto-mute, headset detection, interrupts, the analog bypass, the mono
differential output, class-D mode, the PowerTune modes, the over-current settings, the digital
microphone path, the secondary audio interface and CLKOUT. The reset pin is the board's.

Linux binds this chip with `tlv320aic32x4`, a driver for the related TLV320AIC32x4 codecs. For
playback the two agree, except that this driver never writes the registers this chip does not
have (the AIC32x4's record path and line outputs), waits for the DACs to power down before
stopping their clocks as TI requires, reports a timeout instead of unmuting anyway, and writes some
registers whole where the kernel updates bits (same result from a reset). The crate documentation
lists the differences register by register.

## Testing

`cargo test` runs one `embedded-hal-mock` test per write path, pinning the bytes on the bus with
the page selects and the waits, plus unit tests of the status decoders and this README's examples.
On the chip, on the Echo Dot: the status read beside the running Linux driver (2026-09-30), `probe`
from bare metal (2026-10-05), and playback from bare metal (audible since 2026-10-06).

## Sources

- TI **SLAU434A**, [TLV320DAC3203 Applications Reference Guide](https://www.ti.com/lit/ug/slau434a/slau434a.pdf)
  (2012, revised 2019): the register descriptions, the sequences and the example setups. "§N",
  "Table N" and "Figure N" in this crate refer to it.
- TI **SLOS756B**, [TLV320DAC3203 datasheet](https://www.ti.com/lit/ds/symlink/tlv320dac3203.pdf)
  (2012, revised 2018): the register summary and the typical application circuit.
- The Linux codec driver as Amazon ships it for the Echo Dot (Linux 3.18.19):
  [`tlv320aic32x4.c`](https://gitlab.com/echo-pmos/amazon-biscuit-kernel/-/blob/3923647bdeca8fdaeb943fb31234dbe361668d28/sound/soc/codecs/tlv320aic32x4.c) and
  [`.h`](https://gitlab.com/echo-pmos/amazon-biscuit-kernel/-/blob/3923647bdeca8fdaeb943fb31234dbe361668d28/sound/soc/codecs/tlv320aic32x4.h), cited as `aic32x4.c:N`, with
  [`regmap.c`](https://gitlab.com/echo-pmos/amazon-biscuit-kernel/-/blob/3923647bdeca8fdaeb943fb31234dbe361668d28/drivers/base/regmap/regmap.c) and the machine driver
  [`mt_soc_machine.c`](https://gitlab.com/echo-pmos/amazon-biscuit-kernel/-/blob/3923647bdeca8fdaeb943fb31234dbe361668d28/sound/soc/mediatek/mt_soc_audio_8163_amzn/mt_soc_machine.c), in
  <https://gitlab.com/echo-pmos/amazon-biscuit-kernel> at commit `3923647` (read in source).

## Minimum supported Rust version

Rust 1.87, tested in CI. Raising it is a minor version bump.

## License

Licensed under either of [Apache License, Version 2.0](https://github.com/mschoenlaub/tlv320dac3203/blob/main/LICENSE-APACHE) or
[MIT license](https://github.com/mschoenlaub/tlv320dac3203/blob/main/LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.

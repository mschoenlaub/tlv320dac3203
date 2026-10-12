# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the crate follows
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-10-12

First release.

- Paged register access that selects the page with every access, the register map with its bit
  fields, and a typed error.
- Reset, supply settings and the analog reference.
- The clock tree: multiplexers, PLL, NDAC, MDAC, DOSR and the BCLK divider.
- The serial audio interface: I2S, DSP, left- and right-justified, word length, clock directions.
- The processing block, the DAC channels (data path, power, mute, digital volume) and DRC off.
- The headphone drivers: routing, gain, de-pop settings, power and mute.
- The MFP2 and SCLK/MFP3 pins.
- `power_up` and `power_down`, the ordered playback sequences with bounded waits, and
  `power_up_start`/`headphone_gain_applied`/`power_up_finish` for callers that wait
  asynchronously.
- Status snapshots of pages 0 and 1, decoded into the write side's types, and `probe`, an
  identity check that writes nothing. The driver assumes it is the only one writing to the chip
  while it holds it.
- The beep generator of `PRB_P25`, behind the feature `beep`.

[Unreleased]: https://github.com/mschoenlaub/tlv320dac3203/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/mschoenlaub/tlv320dac3203/releases/tag/v0.1.0

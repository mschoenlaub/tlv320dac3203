//! An identity check of whatever answers at the chip's address ([`Tlv320dac3203::probe`]): which
//! page it has selected, whether it is a TLV320DAC3203, and whether its clocks are still at their
//! reset values.
//!
//! It writes no register, so nothing is written to a chip before it has identified: it reads the
//! page-select register instead of selecting a page, and identifies only a chip that is on page 0,
//! where its reset leaves it.

use embedded_hal::i2c::I2c;

use crate::status::CLOCK_RESET_VALUES;
use crate::{Error, Tlv320dac3203, register};

/// What [`Tlv320dac3203::probe`] found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Probe {
    /// The page-select register: the page the chip had selected. 0 after a reset (§5.2.1).
    pub page: u8,
    /// Page 0 only (`None` on any other page, which the probe does not leave): whether
    /// [`register::AOSR`] read [`register::AOSR_VALUE`], the one page 0 value no write changes.
    pub identified: Option<bool>,
    /// Page 0 and identified only: whether every clock register holds its reset value
    /// ([`CLOCK_RESET_VALUES`]), i.e. nothing has set the chip up since its last reset.
    pub clocks_at_reset: Option<bool>,
}

impl<I2C: I2c> Tlv320dac3203<I2C> {
    /// Reads, without writing a register: the page-select register; then, only on page 0,
    /// [`register::AOSR`] (the identity check); then, only if that read [`register::AOSR_VALUE`],
    /// the nine clock registers of [`CLOCK_RESET_VALUES`] in order, up to the first that differs
    /// from its reset value. Having read the page, it reads those registers on it without
    /// selecting one.
    ///
    /// # Errors
    ///
    /// The first failed transfer ([`Error::Bus`]): a NACK when nothing answers at the address.
    pub fn probe(&mut self) -> Result<Probe, Error<I2C::Error>> {
        let page = self.read_page_select()?;
        if page != register::page::DIGITAL {
            return Ok(Probe {
                page,
                identified: None,
                clocks_at_reset: None,
            });
        }
        let identified = self.read_selected(register::AOSR)? == register::AOSR_VALUE;
        let clocks_at_reset = if identified {
            Some(self.clocks_at_reset()?)
        } else {
            None
        };
        Ok(Probe {
            page,
            identified: Some(identified),
            clocks_at_reset,
        })
    }

    /// Whether the clock registers hold their reset values, read up to the first that does not.
    fn clocks_at_reset(&mut self) -> Result<bool, Error<I2C::Error>> {
        for (register, reset_value) in CLOCK_RESET_VALUES {
            if self.read_selected(register)? != reset_value {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ADDRESS;

    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    fn read(addr: u8, value: u8) -> Transaction {
        Transaction::write_read(ADDRESS, vec![addr], vec![value])
    }

    /// A reset chip: the page read (page 0), the identity read, then the nine clock registers at
    /// their reset values. Nothing is written: no page select anywhere.
    #[test]
    fn a_reset_chip_is_identified_and_at_reset_without_a_write() {
        let mut expected = vec![read(register::PAGE_SELECT, 0), read(20, 0x80)];
        for (register, value) in CLOCK_RESET_VALUES {
            expected.push(read(register.addr(), value));
        }
        let mut i2c = Mock::new(&expected);
        let mut dac = Tlv320dac3203::new(&mut i2c, ADDRESS);
        assert_eq!(
            dac.probe(),
            Ok(Probe {
                page: 0,
                identified: Some(true),
                clocks_at_reset: Some(true),
            })
        );
        i2c.done();
    }

    /// A chip that has been set up: the clock reads stop at the first register off its reset
    /// value (here R5, the PLL powered: the stock kernel's `0xD1`).
    #[test]
    fn a_configured_chip_stops_at_the_first_clock_register_off_reset() {
        let mut i2c = Mock::new(&[
            read(register::PAGE_SELECT, 0),
            read(20, 0x80),
            read(4, 0x00),
            read(5, 0xD1),
        ]);
        let mut dac = Tlv320dac3203::new(&mut i2c, ADDRESS);
        assert_eq!(
            dac.probe(),
            Ok(Probe {
                page: 0,
                identified: Some(true),
                clocks_at_reset: Some(false),
            })
        );
        i2c.done();
    }

    /// A chip whose register 20 is not 128 is not read any further.
    #[test]
    fn a_wrong_aosr_ends_the_probe() {
        let mut i2c = Mock::new(&[read(register::PAGE_SELECT, 0), read(20, 0x40)]);
        let mut dac = Tlv320dac3203::new(&mut i2c, ADDRESS);
        assert_eq!(
            dac.probe(),
            Ok(Probe {
                page: 0,
                identified: Some(false),
                clocks_at_reset: None,
            })
        );
        i2c.done();
    }

    /// On page 1 only the page register is read: no page write to get to page 0.
    #[test]
    fn on_another_page_only_the_page_register_is_read() {
        let mut i2c = Mock::new(&[read(register::PAGE_SELECT, 1)]);
        let mut dac = Tlv320dac3203::new(&mut i2c, ADDRESS);
        assert_eq!(
            dac.probe(),
            Ok(Probe {
                page: 1,
                identified: None,
                clocks_at_reset: None,
            })
        );
        i2c.done();
    }

    #[test]
    fn a_nack_on_the_first_read_is_the_result() {
        let nack = ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address);
        let mut i2c = Mock::new(&[read(register::PAGE_SELECT, 0).with_error(nack)]);
        let mut dac = Tlv320dac3203::new(&mut i2c, ADDRESS);
        assert_eq!(dac.probe(), Err(Error::Bus(nack)));
        i2c.done();
    }
}

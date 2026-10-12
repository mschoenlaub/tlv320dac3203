//! Helpers for the host tests: a delay that is checked against the same expectation list as the
//! bus traffic.
//!
//! `embedded-hal-mock` checks I2C transfers and delays with two separate mocks, which would pin
//! each of them but not their order relative to one another, and the order is the point of the
//! sequences tested here (a wait has to sit between two particular writes). So a wait is put
//! into the I2C expectation list as a marker transfer, and [`BusDelay`] takes the next
//! expectation off that list and checks that it is the marker for the wait it was asked for.

use embedded_hal::delay::DelayNs;
use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

/// The address the marker transfers carry. It is the I2C general-call address, which no driver
/// value in these tests is created with, so a marker can never be mistaken for a chip access.
const MARKER_ADDRESS: u8 = 0x00;

/// The expectation that stands for "the driver waits `ms` milliseconds here".
pub(crate) fn wait_ms(ms: u32) -> Transaction {
    wait_ns(u64::from(ms) * 1_000_000)
}

/// The marker for a wait of `ns` nanoseconds.
fn wait_ns(ns: u64) -> Transaction {
    Transaction::write(MARKER_ADDRESS, ns.to_be_bytes().to_vec())
}

/// A delay that consumes wait markers from the I2C mock's expectation list. Make it from a clone
/// of the mock: clones share one list.
pub(crate) struct BusDelay(pub(crate) Mock);

impl BusDelay {
    /// Takes the next expectation and checks that it is the marker for `ns`.
    fn expect(&mut self, ns: u64) {
        let next = self
            .0
            .next()
            .expect("a wait where no more expectations are left");
        assert_eq!(
            next,
            wait_ns(ns),
            "a wait where the test expected something else"
        );
    }
}

impl DelayNs for BusDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.expect(u64::from(ns));
    }

    // The trait's default implementations split a long wait into several `delay_ns` calls, so the
    // coarser methods are overridden to see each wait as the one call the driver made.
    fn delay_us(&mut self, us: u32) {
        self.expect(u64::from(us) * 1_000);
    }

    fn delay_ms(&mut self, ms: u32) {
        self.expect(u64::from(ms) * 1_000_000);
    }
}

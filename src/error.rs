/// Strongly-typed error enum for the VL53L1X driver.
///
/// The generic parameter `E` carries the underlying I2C bus error type so that
/// callers can recover and inspect the original HAL error when needed. Every
/// fallible function in this crate returns `Result<_, Error<E>>`.
///
/// **Never** use `unwrap()` or `expect()` anywhere in this crate.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// An I2C bus transaction failed. The inner value is the HAL-layer error
    /// so that the caller can decide whether to retry or escalate.
    Bus(E),

    /// A polling loop (boot-state, data-ready, calibration) exceeded its
    /// maximum iteration / time budget and the sensor did not respond.
    Timeout,

    /// The sensor returned data that cannot be parsed or the model-ID did not
    /// match 0xEEAC. Indicates a wiring fault, wrong I2C address, or corrupted
    /// register state.
    InvalidData,

    /// The range-status register reported an unrecoverable hardware fault
    /// (status code 255 / `NotUpdated`). The driver cannot produce a valid
    /// distance until the sensor is re-initialised.
    SensorFault,

    /// A ranging result was requested but no new sample is available yet.
    /// Callers should wait for `wait_for_data_ready()` before calling
    /// `get_result()`.
    NotReady,

    /// A function argument was outside the legal range (e.g. ROI size < 4,
    /// timing budget not in the discrete set, sigma value overflow).
    InvalidArgument,
}

impl<E> From<E> for Error<E> {
    /// Automatically wraps an I2C HAL error in `Error::Bus`, enabling the
    /// `?` operator on raw bus calls.
    #[inline]
    fn from(e: E) -> Self {
        Error::Bus(e)
    }
}

//! # `vl53l1x` — Async `no_std` driver for the STMicroelectronics VL53L1X ToF sensor
//!
//! Built for the [Embassy](https://embassy.dev) async framework, but compatible
//! with any executor that implements `embedded-hal-async`.
//!
//! ## Quick start
//!
//! ```ignore
//! use vl53l1x::{Vl53l1x, config::{DistanceMode, TimingBudget}};
//! use embassy_time::Duration;
//!
//! // `i2c` implements embedded_hal_async::i2c::I2c
//! let mut sensor = Vl53l1x::new(i2c, Vl53l1x::DEFAULT_ADDR);
//! sensor.init().await?;
//! sensor.set_distance_mode(DistanceMode::Long).await?;
//! sensor.set_timing_budget(TimingBudget::Ms100).await?;
//! sensor.start_ranging().await?;
//!
//! loop {
//!     sensor.wait_for_data_ready(Duration::from_millis(200)).await?;
//!     let r = sensor.get_result().await?;
//!     sensor.clear_interrupt().await?;
//!     if r.status.is_valid() {
//!         // r.distance_mm is the distance in millimetres
//!     }
//! }
//! ```
//!
//! ## Features
//!
//! | Feature | Description |
//! |---------|-------------|
//! | `defmt` | Enable zero-cost structured logging via the `defmt` crate |
//!
//! ## Coding standards
//!
//! This crate strictly follows the **Adom Industries Factory-Grade Component**
//! standards:
//! - `#![no_std]` — no heap allocation, no `std` dependency.
//! - No `unwrap()` or `expect()` in library code.
//! - All I/O is async via `embedded-hal-async`.
//! - All timeouts are bounded and enforced via `embassy_time::Timer`.
//! - Zero global mutable state.

#![no_std]
#![deny(unsafe_code)]
#![warn(missing_docs)]

/// Configuration types for the VL53L1X sensor
pub mod config;
/// Error types returned by the driver
pub mod error;
/// High-level gesture detection module
pub mod gesture;
/// Fundamental data types returned by the sensor
pub mod types;

// Public re-export of the driver struct
pub use driver::Vl53l1x;

// Public re-export of frequently-used types at crate root for ergonomics
pub use config::{DistanceMode, InterruptPolarity, ThresholdWindow, TimingBudget};
pub use error::Error;
pub use gesture::{GestureEngine, GestureEvent};
pub use types::{DriverVersion, RangeStatus, RangingResult};

// Internal modules (not part of the public API surface)
mod driver;
mod register;
mod transport;

// ─── Register address constants accessible to consumers and tests ─────────────
/// Re-export of all 16-bit register address constants.
///
/// These are derived from `VL53L1X_api.h` (ULD v3.5.5) and exposed here so
/// that application code and tests can reference named constants rather than
/// raw hex literals.
pub mod regs {
    pub use crate::register::*;
}

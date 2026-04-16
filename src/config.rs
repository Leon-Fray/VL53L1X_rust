/// Type-safe configuration enumerations for the VL53L1X driver.
///
/// All values are derived from the ULD User Manual (UM2356) and the
/// reference C implementation (VL53L1X_api.c, STSW-IMG009 v3.5.5).
/// Using enums instead of raw integers prevents impossible configurations
/// at compile time.

use crate::error::Error;

// ─── Distance Mode ────────────────────────────────────────────────────────────

/// Sensing distance mode.
///
/// | Mode  | Max range (dark) | Ambient immunity |
/// |-------|-----------------|-----------------|
/// | Short | ~1.3 m          | better          |
/// | Long  | ~4.0 m          | standard        |
///
/// The C reference encodes this as 1 = Short, 2 = Long.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DistanceMode {
    /// Short-distance mode: up to ~1.3 m, better ambient light immunity.
    Short,
    /// Long-distance mode (default): up to ~4.0 m in darkness.
    Long,
}

impl DistanceMode {
    /// Returns the `PHASECAL_CONFIG__TIMEOUT_MACROP` value used to identify
    /// the current distance mode when reading back from the sensor.
    #[inline]
    #[allow(dead_code)]
    pub(crate) fn phasecal_timeout(self) -> u8 {
        match self {
            DistanceMode::Short => 0x14,
            DistanceMode::Long => 0x0A,
        }
    }

    /// Attempt to decode the distance mode from a raw `PHASECAL_CONFIG__TIMEOUT_MACROP`
    /// register value read from the sensor.
    pub fn from_phasecal<E>(raw: u8) -> Result<Self, Error<E>> {
        match raw {
            0x14 => Ok(DistanceMode::Short),
            0x0A => Ok(DistanceMode::Long),
            _ => Err(Error::InvalidData),
        }
    }
}

// ─── Timing Budget ─────────────────────────────────────────────────────────────

/// Ranging timing budget in milliseconds.
///
/// These are the **only** legal values accepted by the sensor hardware. The
/// `Ms15` variant is only valid in [`DistanceMode::Short`]. Choosing a longer
/// budget improves measurement accuracy but reduces throughput.
///
/// Attempting to set a timing budget that is invalid for the current distance
/// mode returns [`Error::InvalidArgument`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimingBudget {
    /// 15 ms — **Short mode only**. Fastest, lowest accuracy.
    Ms15,
    /// 20 ms
    Ms20,
    /// 33 ms
    Ms33,
    /// 50 ms
    Ms50,
    /// 100 ms (default)
    Ms100,
    /// 200 ms
    Ms200,
    /// 500 ms — slowest, highest accuracy.
    Ms500,
}

impl TimingBudget {
    /// Returns the millisecond value as a plain `u16`.
    #[inline]
    pub fn as_ms(self) -> u16 {
        match self {
            Self::Ms15 => 15,
            Self::Ms20 => 20,
            Self::Ms33 => 33,
            Self::Ms50 => 50,
            Self::Ms100 => 100,
            Self::Ms200 => 200,
            Self::Ms500 => 500,
        }
    }

    /// Decode from the raw `RANGE_CONFIG__TIMEOUT_MACROP_A_HI` register value.
    /// Both Short-mode and Long-mode magic numbers are handled.
    pub fn from_macrop_a<E>(raw: u16) -> Result<Self, Error<E>> {
        match raw {
            0x001D => Ok(Self::Ms15),
            0x0051 | 0x001E => Ok(Self::Ms20),
            0x00D6 | 0x0060 => Ok(Self::Ms33),
            0x01AE | 0x00AD => Ok(Self::Ms50),
            0x02E1 | 0x01CC => Ok(Self::Ms100),
            0x03E1 | 0x02D9 => Ok(Self::Ms200),
            0x0591 | 0x048F => Ok(Self::Ms500),
            _ => Err(Error::InvalidData),
        }
    }

    /// Returns the `(MACROP_A_HI, MACROP_B_HI)` register pair for this budget
    /// and distance mode. Returns `None` when the combination is illegal
    /// (i.e., `Ms15` in Long mode).
    pub fn register_pair(self, mode: DistanceMode) -> Option<(u16, u16)> {
        match (mode, self) {
            // Short-mode pairs (from ULD VL53L1X_SetTimingBudgetInMs)
            (DistanceMode::Short, Self::Ms15) => Some((0x001D, 0x0027)),
            (DistanceMode::Short, Self::Ms20) => Some((0x0051, 0x006E)),
            (DistanceMode::Short, Self::Ms33) => Some((0x00D6, 0x006E)),
            (DistanceMode::Short, Self::Ms50) => Some((0x01AE, 0x01E8)),
            (DistanceMode::Short, Self::Ms100) => Some((0x02E1, 0x0388)),
            (DistanceMode::Short, Self::Ms200) => Some((0x03E1, 0x0496)),
            (DistanceMode::Short, Self::Ms500) => Some((0x0591, 0x05C1)),
            // Long-mode pairs (from ULD VL53L1X_SetTimingBudgetInMs)
            (DistanceMode::Long, Self::Ms15) => None, // illegal
            (DistanceMode::Long, Self::Ms20) => Some((0x001E, 0x0022)),
            (DistanceMode::Long, Self::Ms33) => Some((0x0060, 0x006E)),
            (DistanceMode::Long, Self::Ms50) => Some((0x00AD, 0x00C6)),
            (DistanceMode::Long, Self::Ms100) => Some((0x01CC, 0x01EA)),
            (DistanceMode::Long, Self::Ms200) => Some((0x02D9, 0x02F8)),
            (DistanceMode::Long, Self::Ms500) => Some((0x048F, 0x04A4)),
        }
    }
}

// ─── Interrupt Polarity ────────────────────────────────────────────────────────

/// Interrupt pin polarity.
///
/// The default configuration patch sets the sensor to active-low (bit 4 of
/// `GPIO_HV_MUX__CTRL` = 1). `CheckForDataReady` reads this polarity and
/// compares it to bit 0 of `GPIO__TIO_HV_STATUS`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptPolarity {
    /// GPIO goes HIGH when a new sample is ready (default after reset).
    ActiveHigh,
    /// GPIO goes LOW when a new sample is ready.
    ActiveLow,
}

// ─── Distance Threshold Window ────────────────────────────────────────────────

/// Window condition for the hardware distance threshold interrupt.
///
/// Configures when the sensor fires an interrupt based on measured distance
/// relative to the programmed low/high thresholds.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThresholdWindow {
    /// Interrupt fires when distance < low threshold.
    Below = 0,
    /// Interrupt fires when distance > high threshold.
    Above = 1,
    /// Interrupt fires when distance is outside the [low, high] window.
    OutOfWindow = 2,
    /// Interrupt fires when distance is inside the [low, high] window.
    InWindow = 3,
}

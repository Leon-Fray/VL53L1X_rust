/// Public result and status types returned by the VL53L1X driver.
///
/// All types implement `Debug` unconditionally, and `defmt::Format`
/// when the `defmt` feature is enabled — making them zero-cost to log
/// in production builds where that feature is absent.

// ─── Range Status ─────────────────────────────────────────────────────────────

/// Decoded ranging status from the sensor's RESULT__RANGE_STATUS register.
///
/// The raw hardware value is converted via the `status_rtn[24]` lookup table
/// found in `VL53L1X_api.c`. Values not present in the table map to
/// [`RangeStatus::NotUpdated`].
///
/// Source: AN5573 Application Note, Table 3 — Range Error Codes.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RangeStatus {
    /// 0 — Range valid. `distance_mm` is usable.
    Valid,
    /// 1 — Sigma estimator check failed. Distance may be inaccurate.
    SigmaFailed,
    /// 2 — Signal check failed. Target too weak or too far.
    SignalFailed,
    /// 3 — Target is below the minimum detection threshold.
    TargetBelowMinRange,
    /// 4 — Invalid phase — typically caused by a highly reflective target.
    InvalidPhase,
    /// 5 — Hardware / laser safety failure.
    HardwareFail,
    /// 6 — No valid measurement (wrap-around check failed).
    NoWrapAroundCheck,
    /// 7 — Wrap-around target failure.
    WrapTargetFail,
    /// 9 — Internal algorithm processing failed.
    ProcessingFail,
    /// 10 — Cross-talk signal failed.
    XtalkSignalFail,
    /// 11 — Synchronisation interrupt — distance not representative.
    SynchronisationInt,
    /// 12 — Merged pulse — two targets merged into one return.
    MergedPulseDetected,
    /// 13 — Target is too close to the sensor cover glass.
    TargetTooClose,
    /// 255 — Result register not updated since last clear.
    NotUpdated,
}

impl RangeStatus {
    /// Map a raw hardware status byte (already masked to bits [4:0] and
    /// looked up via the ULD `status_rtn[24]` table) to a `RangeStatus`.
    ///
    /// The C lookup table is: `{ 255,255,255,5,2,4,1,7,3,0,255,255,9,13,255,
    /// 255,255,255,10,6,255,255,11,12 }` — raw index → returned code.
    pub fn from_uld_code(code: u8) -> Self {
        match code {
            0 => Self::Valid,
            1 => Self::SigmaFailed,
            2 => Self::SignalFailed,
            3 => Self::TargetBelowMinRange,
            4 => Self::InvalidPhase,
            5 => Self::HardwareFail,
            6 => Self::NoWrapAroundCheck,
            7 => Self::WrapTargetFail,
            9 => Self::ProcessingFail,
            10 => Self::XtalkSignalFail,
            11 => Self::SynchronisationInt,
            12 => Self::MergedPulseDetected,
            13 => Self::TargetTooClose,
            _ => Self::NotUpdated,
        }
    }

    /// The raw lookup table from `VL53L1X_api.c` (`status_rtn[24]`).
    ///
    /// `raw_hw_status & 0x1F` is the index into this table; the table value
    /// is the user-facing code passed to `from_uld_code`.
    const STATUS_RTN: [u8; 24] = [
        255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255,
        255, 11, 12,
    ];

    /// Convert a raw `RESULT__RANGE_STATUS` byte as read from the sensor into
    /// a `RangeStatus`. Handles the mask and table lookup internally.
    #[inline]
    pub fn from_raw(raw: u8) -> Self {
        let idx = (raw & 0x1F) as usize;
        let code = if idx < Self::STATUS_RTN.len() {
            Self::STATUS_RTN[idx]
        } else {
            255
        };
        Self::from_uld_code(code)
    }

    /// Returns `true` if the measurement is valid and `distance_mm` can be
    /// used by application code.
    #[inline]
    pub fn is_valid(self) -> bool {
        self == RangeStatus::Valid
    }
}

// ─── Ranging Result ────────────────────────────────────────────────────────────

/// A single fully-parsed ranging measurement from the VL53L1X.
///
/// Obtained via [`Vl53l1x::get_result()`] which performs a single 17-byte
/// burst read — the most efficient way to retrieve all fields at once.
///
/// # Units
/// - `distance_mm`: millimetres
/// - `ambient_kcps`: kilo-counts-per-second (ambient light on SPADs)
/// - `signal_kcps`: kilo-counts-per-second (return signal)
/// - `num_spads`: raw SPAD count >> 8
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RangingResult {
    /// Decoded ranging status — check `is_valid()` before using `distance_mm`.
    pub status: RangeStatus,
    /// Distance to target in millimetres. Only valid when `status == Valid`.
    pub distance_mm: u16,
    /// Ambient light count rate per SPAD (scaled ×8 from raw register).
    pub ambient_kcps: u16,
    /// Signal return count rate per SPAD (scaled ×8 from raw register).
    pub signal_kcps: u16,
    /// Number of active SPADs (raw >> 8).
    pub num_spads: u16,
}

impl RangingResult {
    /// Parse a `RangingResult` from the 17-byte burst read starting at
    /// `RESULT__RANGE_STATUS` (0x0089).
    ///
    /// Byte layout from `VL53L1X_GetResult()` in VL53L1X_api.c:
    /// ```text
    /// [0]      RANGE_STATUS
    /// [1..2]   (unused in this function)
    /// [3]      DSS_ACTUAL_EFFECTIVE_SPADS (only MSB used for num_spads)
    /// [4..6]   (unused)
    /// [7..8]   AMBIENT_COUNT_RATE_MCPS (×8)
    /// [9..12]  (unused)
    /// [13..14] FINAL_CROSSTALK_CORRECTED_RANGE_MM
    /// [15..16] PEAK_SIGNAL_COUNT_RATE (×8)
    /// ```
    pub fn from_burst_bytes(buf: &[u8; 17]) -> Self {
        let status = RangeStatus::from_raw(buf[0]);
        let ambient_kcps = u16::from_be_bytes([buf[7], buf[8]]).wrapping_mul(8);
        let num_spads = buf[3] as u16;
        let signal_kcps = u16::from_be_bytes([buf[15], buf[16]]).wrapping_mul(8);
        let distance_mm = u16::from_be_bytes([buf[13], buf[14]]);

        RangingResult {
            status,
            distance_mm,
            ambient_kcps,
            signal_kcps,
            num_spads,
        }
    }
}

// ─── Driver Version ────────────────────────────────────────────────────────────

/// Rust driver version — independent of the ULD firmware version.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DriverVersion {
    /// Major version number
    pub major: u8,
    /// Minor version number
    pub minor: u8,
    /// Patch version number
    pub patch: u8,
}

impl DriverVersion {
    /// Current driver version.
    pub const CURRENT: Self = Self {
        major: 0,
        minor: 1,
        patch: 0,
    };
}

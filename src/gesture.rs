/// Gesture detection engine for the VL53L1X.
///
/// Uses ROI (Region of Interest) switching to divide the sensor's SPAD array
/// into two horizontal zones — **Left** and **Right** — and alternates
/// measurements between them. By tracking which zone first detects an object
/// and which detects it afterwards, the engine can classify hand swipes and
/// taps.
///
/// ## Algorithm (from Arduino VL53L1X library)
/// 1. Configure sensor for short ROI (8 SPADs wide × 16 tall).
/// 2. Alternate between Left zone (centre SPAD 167) and Right zone
///    (centre SPAD 231) on every measurement cycle.
/// 3. If a zone reading drops below `threshold_mm`, record it as "object seen".
/// 4. A swipe is confirmed when:
///    - The object first appears in one zone then the other, within
///      `gesture_timeout_ms`.
///    - The object fully leaves both zones to complete the gesture.
/// 5. A tap is detected when the object appears and disappears from a single
///    zone within `tap_max_ms`.
///
/// ## Usage
/// ```ignore
/// let mut engine = GestureEngine::new(1500);   // threshold 1500 mm
/// let event = engine.poll(&mut sensor, Duration::from_millis(30)).await?;
/// match event {
///     GestureEvent::SwipeLeftToRight => { /* … */ }
///     GestureEvent::SwipeRightToLeft => { /* … */ }
///     GestureEvent::TapLeft          => { /* … */ }
///     GestureEvent::TapRight         => { /* … */ }
///     GestureEvent::None             => {}
/// }
/// ```

use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

use crate::{driver::Vl53l1x, error::Error};

// ─── ROI SPAD centres (from Arduino SPAD map) ─────────────────────────────────
//
// The VL53L1X SPAD array is a 16×16 grid, addressed by a flat index.
// The value 0xC7 (= 199) is the full-field optical centre.
// Offsets below correspond to:
//   Left zone  → column 7, rows 1–16 (left half of the SPAD array)
//   Right zone → column 7, rows 11–16 (right half, shifted down by 4)
//
// These match the Arduino vl53l1x_class.h ROI_CENTER_LEFT / ROI_CENTER_RIGHT
// constants used throughout that library's gesture examples.
const SPAD_LEFT: u8 = 167;
const SPAD_RIGHT: u8 = 231;

// ROI size for gesture sensing: narrow in X, full height in Y
const GESTURE_ROI_X: u8 = 8;
const GESTURE_ROI_Y: u8 = 16;

// ─── Distance that signals "no object / object too far" ───────────────────────
/// A reading at or above this value is treated as "zone is empty".
const NO_OBJECT_MM: u16 = 2000;

// ─── Gesture event ────────────────────────────────────────────────────────────

/// A classified gesture event produced by [`GestureEngine`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GestureEvent {
    /// Hand moved from left to right across the sensor.
    SwipeLeftToRight,
    /// Hand moved from right to left across the sensor.
    SwipeRightToLeft,
    /// Brief presence detected in the left zone only.
    TapLeft,
    /// Brief presence detected in the right zone only.
    TapRight,
    /// No gesture detected in this poll cycle.
    None,
}

// ─── Internal zone enum ───────────────────────────────────────────────────────

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Zone {
    Left,
    Right,
}

impl Zone {
    fn spad_centre(self) -> u8 {
        match self {
            Zone::Left => SPAD_LEFT,
            Zone::Right => SPAD_RIGHT,
        }
    }

    fn next(self) -> Self {
        match self {
            Zone::Left => Zone::Right,
            Zone::Right => Zone::Left,
        }
    }
}

// ─── State machine ────────────────────────────────────────────────────────────

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum GestureState {
    /// Waiting for first detection.
    Idle,
    /// Object first seen in Left zone; checking if it moves to Right.
    LeftSeen,
    /// Object first seen in Right zone; checking if it moves to Left.
    RightSeen,
    /// Object seen in both zones within timeout (swipe confirmed, waiting for
    /// object to leave so we can classify direction and reset).
    BothSeen { left_first: bool },
}

// ─── GestureEngine ────────────────────────────────────────────────────────────

/// Stateful gesture recognition engine.
///
/// Call [`poll`](GestureEngine::poll) in a loop from an Embassy task.
/// The engine alternates the sensor ROI between Left and Right zones on
/// each call and drives the state machine forward.
pub struct GestureEngine {
    /// Maximum distance (mm) at which an object is considered "present" in a zone.
    threshold_mm: u16,
    /// Latest reading from the Left zone.
    left_dist: u16,
    /// Latest reading from the Right zone.
    right_dist: u16,
    /// Zone that will be measured on the next `poll` call.
    next_zone: Zone,
    /// Current state of the gesture recognition FSM.
    state: GestureState,
    /// How many consecutive poll cycles the object has been in only one zone.
    single_zone_ticks: u32,
}

/// Number of poll cycles an object must be visible in a single zone before it
/// is classified as a "tap" rather than the first leg of a swipe.
const TAP_MAX_TICKS: u32 = 6;
/// When state is `BothSeen`, how many ticks to wait for the object to clear
/// before resetting. Prevents mis-triggering on slow gestures.
const SWIPE_CLEAR_TICKS: u32 = 4;

impl GestureEngine {
    /// Create a new engine.
    ///
    /// * `threshold_mm` — objects closer than this are considered "present".
    ///   1 500 mm is a sensible default for hand-gesture use.
    pub fn new(threshold_mm: u16) -> Self {
        Self {
            threshold_mm,
            left_dist: NO_OBJECT_MM,
            right_dist: NO_OBJECT_MM,
            next_zone: Zone::Left,
            state: GestureState::Idle,
            single_zone_ticks: 0,
        }
    }

    /// Configure the sensor ROI for gesture mode, then run one measurement
    /// cycle and advance the state machine.
    ///
    /// This function:
    /// 1. Sets the ROI centre to the `next_zone`'s SPAD centre.
    /// 2. Starts ranging, waits for a result, stops ranging.
    /// 3. Updates the zone distance cache.
    /// 4. Advances the FSM and returns any completed gesture.
    ///
    /// **The sensor must be stopped (not ranging) before the first call.**
    pub async fn poll<I: I2c>(
        &mut self,
        sensor: &mut Vl53l1x<I>,
        measurement_timeout: Duration,
    ) -> Result<GestureEvent, Error<I::Error>> {
        // ── 1. Point ROI at the current zone ────────────────────────────────
        let zone = self.next_zone;
        sensor.set_roi(GESTURE_ROI_X, GESTURE_ROI_Y).await?;
        sensor.set_roi_center(zone.spad_centre()).await?;

        // ── 2. Take one measurement ──────────────────────────────────────────
        sensor.start_ranging().await?;
        sensor.wait_for_data_ready(measurement_timeout).await?;
        let result = sensor.get_result().await?;
        sensor.clear_interrupt().await?;
        sensor.stop_ranging().await?;

        // ── 3. Update zone cache ─────────────────────────────────────────────
        let dist = if result.status.is_valid() {
            result.distance_mm
        } else {
            NO_OBJECT_MM
        };

        match zone {
            Zone::Left => self.left_dist = dist,
            Zone::Right => self.right_dist = dist,
        }

        // Advance to the other zone next time
        self.next_zone = zone.next();

        // ── 4. Drive state machine ───────────────────────────────────────────
        let event = self.advance_fsm();

        #[cfg(feature = "defmt")]
        defmt::trace!(
            "vl53l1x/gesture: zone={:?} dist={}mm state={:?} event={:?}",
            zone,
            dist,
            self.state,
            event
        );

        // Small yield between measurements to allow other tasks to run
        Timer::after(Duration::from_millis(5)).await;

        Ok(event)
    }

    // ─── FSM (no I/O, pure logic) ────────────────────────────────────────────

    fn object_in_left(&self) -> bool {
        self.left_dist < self.threshold_mm
    }

    fn object_in_right(&self) -> bool {
        self.right_dist < self.threshold_mm
    }

    fn advance_fsm(&mut self) -> GestureEvent {
        let left = self.object_in_left();
        let right = self.object_in_right();

        match self.state {
            // ── Idle: watch for any activation ──────────────────────────────
            GestureState::Idle => {
                if left && !right {
                    self.state = GestureState::LeftSeen;
                    self.single_zone_ticks = 1;
                } else if right && !left {
                    self.state = GestureState::RightSeen;
                    self.single_zone_ticks = 1;
                }
                GestureEvent::None
            }

            // ── Object first seen in Left ────────────────────────────────────
            GestureState::LeftSeen => {
                if left && right {
                    // Object moved into both zones: swipe heading right
                    self.state = GestureState::BothSeen { left_first: true };
                    self.single_zone_ticks = 0;
                    GestureEvent::None
                } else if left {
                    self.single_zone_ticks += 1;
                    if self.single_zone_ticks >= TAP_MAX_TICKS {
                        // Object lingered in left only — treat as tap
                        self.reset();
                        GestureEvent::TapLeft
                    } else {
                        GestureEvent::None
                    }
                } else {
                    // Object disappeared from left without entering right — abort
                    self.reset();
                    GestureEvent::None
                }
            }

            // ── Object first seen in Right ───────────────────────────────────
            GestureState::RightSeen => {
                if left && right {
                    // Object moved into both zones: swipe heading left
                    self.state = GestureState::BothSeen { left_first: false };
                    self.single_zone_ticks = 0;
                    GestureEvent::None
                } else if right {
                    self.single_zone_ticks += 1;
                    if self.single_zone_ticks >= TAP_MAX_TICKS {
                        self.reset();
                        GestureEvent::TapRight
                    } else {
                        GestureEvent::None
                    }
                } else {
                    self.reset();
                    GestureEvent::None
                }
            }

            // ── Object in both zones — waiting for it to clear ───────────────
            GestureState::BothSeen { left_first } => {
                if !left && !right {
                    // Object has left — emit the swipe direction
                    let event = if left_first {
                        GestureEvent::SwipeLeftToRight
                    } else {
                        GestureEvent::SwipeRightToLeft
                    };
                    self.reset();
                    event
                } else {
                    self.single_zone_ticks += 1;
                    if self.single_zone_ticks > SWIPE_CLEAR_TICKS * 10 {
                        // Stuck — object never cleared; reset to avoid lockup
                        self.reset();
                    }
                    GestureEvent::None
                }
            }
        }
    }

    fn reset(&mut self) {
        self.state = GestureState::Idle;
        self.single_zone_ticks = 0;
        self.left_dist = NO_OBJECT_MM;
        self.right_dist = NO_OBJECT_MM;
    }

    /// Returns the raw distance most recently measured in each zone.
    /// Useful for debugging or building custom gesture classifiers.
    #[inline]
    pub fn zone_distances(&self) -> (u16, u16) {
        (self.left_dist, self.right_dist)
    }
}

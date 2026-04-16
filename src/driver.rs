/// `Vl53l1x<I>` — the main async driver struct.
///
/// ## Ownership model
/// The driver takes ownership of the I2C bus instance `I`. For multi-sensor
/// applications, wrap the bus in an `embedded-hal-bus` sharing adapter
/// (e.g. `I2cDevice`) before constructing each driver; the driver never
/// constructs or clones the bus internally.
///
/// ## Usage
/// ```ignore
/// let mut sensor = Vl53l1x::new(i2c, Vl53l1x::DEFAULT_ADDR);
/// sensor.init().await?;
/// sensor.set_distance_mode(DistanceMode::Long).await?;
/// sensor.set_timing_budget(TimingBudget::Ms100).await?;
/// sensor.start_ranging().await?;
///
/// loop {
///     sensor.wait_for_data_ready(Duration::from_millis(200)).await?;
///     let result = sensor.get_result().await?;
///     sensor.clear_interrupt().await?;
///     if result.status.is_valid() {
///         // use result.distance_mm
///     }
/// }
/// ```

use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

use crate::{
    config::{DistanceMode, InterruptPolarity, ThresholdWindow, TimingBudget},
    error::Error,
    register as reg,
    transport::{rd_byte, rd_dword, rd_word, read_multi, wr_byte, wr_dword, wr_word, write_multi},
    types::{DriverVersion, RangingResult},
};

// ─── Timing constants (deterministic, no unbounded loops) ─────────────────────

/// Polling interval used inside `wait_for_data_ready`.
const DATA_READY_POLL_MS: u64 = 1;
/// Maximum number of 1 ms polls before `wait_for_data_ready` returns `Timeout`.
/// The ULD reference uses 1 000 iterations; we honour the same limit.
const DATA_READY_MAX_POLLS: u32 = 1_000;

/// Polling interval used during boot-state check (sensor power-on).
const BOOT_POLL_MS: u64 = 2;
/// Maximum polls before returning a boot `Timeout`. At 2 ms/poll this is 2 s.
const BOOT_MAX_POLLS: u32 = 1_000;

// ─── Sensor struct ─────────────────────────────────────────────────────────────

/// Async, `no_std` driver for the VL53L1X Time-of-Flight sensor.
///
/// `I` must implement [`embedded_hal_async::i2c::I2c`].
pub struct Vl53l1x<I> {
    /// The owned I2C bus (or a shared bus adapter).
    i2c: I,
    /// 7-bit I2C address (default: `0x29`).
    addr: u8,
}

impl<I: I2c> Vl53l1x<I> {
    // ─── 7-bit default address (0x52 >> 1) ──────────────────────────────────
    /// The default 7-bit I2C address for the VL53L1X sensor (0x29).
    pub const DEFAULT_ADDR: u8 = reg::DEFAULT_I2C_ADDR_7BIT;

    // =========================================================================
    // Construction
    // =========================================================================

    /// Create a new driver instance from an I2C peripheral and a 7-bit address.
    ///
    /// Does **not** communicate with the sensor. Call [`init`](Self::init)
    /// afterwards.
    #[inline]
    pub fn new(i2c: I, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Destroy the driver and return the inner I2C bus.
    #[inline]
    pub fn release(self) -> I {
        self.i2c
    }

    /// Current driver version.
    #[inline]
    pub fn driver_version() -> DriverVersion {
        DriverVersion::CURRENT
    }

    // =========================================================================
    // Initialisation (mirrors VL53L1X_SensorInit in VL53L1X_api.c)
    // =========================================================================

    /// Initialise the sensor.
    ///
    /// Steps (following the ULD reference exactly):
    /// 1. Poll `FIRMWARE__SYSTEM_STATUS` until the sensor has booted.
    /// 2. Write the 91-byte default configuration patch (registers 0x2D–0x87).
    /// 3. Start ranging.
    /// 4. Wait for the first data-ready (confirms the sensor is operational).
    /// 5. Clear interrupt and stop ranging.
    /// 6. Write the VHV two-bounds configuration for normal ranging.
    ///
    /// Calling this function a second time re-applies the configuration patch,
    /// which is safe.
    pub async fn init(&mut self) -> Result<(), Error<I::Error>> {
        // ── Step 1: wait for firmware boot ──────────────────────────────────
        self.wait_for_boot().await?;

        // ── Step 2: write default config patch (91 bytes, regs 0x2D–0x87) ──
        write_multi(
            &mut self.i2c,
            self.addr,
            reg::DEFAULT_CONFIG_START,
            &reg::DEFAULT_CONFIGURATION,
        )
        .await
        .map_err(Error::Bus)?;

        #[cfg(feature = "defmt")]
        defmt::debug!("vl53l1x: default config patch written");

        // ── Step 3: start ranging ─────────────────────────────────────────
        self.start_ranging().await?;

        // ── Step 4: wait for first data-ready ────────────────────────────
        self.wait_for_data_ready(Duration::from_millis(
            DATA_READY_POLL_MS * DATA_READY_MAX_POLLS as u64,
        ))
        .await?;

        // ── Step 5: clear interrupt, stop ranging ─────────────────────────
        self.clear_interrupt().await?;
        self.stop_ranging().await?;

        // ── Step 6: VHV two-bounds configuration ──────────────────────────
        // Sets bit pattern for "two bounds VHV" — required for normal ranging.
        wr_byte(
            &mut self.i2c,
            self.addr,
            reg::VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
            0x09,
        )
        .await
        .map_err(Error::Bus)?;
        // Start VHV from the previous temperature reading
        wr_byte(&mut self.i2c, self.addr, reg::VHV_CONFIG__INIT, 0x00)
            .await
            .map_err(Error::Bus)?;

        #[cfg(feature = "defmt")]
        defmt::info!("vl53l1x: init complete");

        Ok(())
    }

    // =========================================================================
    // Multi-device support — I2C address reassignment
    // =========================================================================

    /// Reassign the sensor's I2C address.
    ///
    /// This must be called **before** any other sensor on the same bus is
    /// powered on (use the XSHUT pin to keep other sensors in reset). The
    /// driver updates its internal address field upon success.
    ///
    /// `new_addr` is a 7-bit value. The sensor register takes the shifted
    /// value (`new_addr >> 1`) per the ULD implementation.
    pub async fn set_i2c_address(&mut self, new_addr: u8) -> Result<(), Error<I::Error>> {
        // Register expects the 7-bit address written in its own bits[6:0].
        // The ULD shifts right by 1; we do the same.
        wr_byte(
            &mut self.i2c,
            self.addr,
            reg::I2C_SLAVE__DEVICE_ADDRESS,
            new_addr >> 1,
        )
        .await
        .map_err(Error::Bus)?;
        self.addr = new_addr;

        #[cfg(feature = "defmt")]
        defmt::info!("vl53l1x: I2C address reassigned to 0x{:02X}", new_addr);

        Ok(())
    }

    /// Return the current 7-bit I2C address being used by this driver instance.
    #[inline]
    pub fn i2c_address(&self) -> u8 {
        self.addr
    }

    // =========================================================================
    // Sensor identification
    // =========================================================================

    /// Read the sensor model ID and validate it equals 0xEEAC.
    ///
    /// Returns `Ok(0xEEAC)` on success, or [`Error::InvalidData`] if the
    /// silicon does not identify itself correctly.
    pub async fn sensor_id(&mut self) -> Result<u16, Error<I::Error>> {
        let id = rd_word(&mut self.i2c, self.addr, reg::IDENTIFICATION__MODEL_ID)
            .await
            .map_err(Error::Bus)?;

        if id != reg::EXPECTED_MODEL_ID {
            #[cfg(feature = "defmt")]
            defmt::error!("vl53l1x: unexpected model ID 0x{:04X} (expected 0xEEAC)", id);
            return Err(Error::InvalidData);
        }

        Ok(id)
    }

    // =========================================================================
    // Ranging control
    // =========================================================================

    /// Start continuous ranging.
    ///
    /// After calling this, poll `wait_for_data_ready()` and then `get_result()`
    /// in a loop. Call `clear_interrupt()` after each result to arm the next
    /// measurement.
    pub async fn start_ranging(&mut self) -> Result<(), Error<I::Error>> {
        wr_byte(&mut self.i2c, self.addr, reg::SYSTEM__MODE_START, 0x40)
            .await
            .map_err(Error::Bus)?;

        #[cfg(feature = "defmt")]
        defmt::debug!("vl53l1x: ranging started");

        Ok(())
    }

    /// Stop ranging.
    pub async fn stop_ranging(&mut self) -> Result<(), Error<I::Error>> {
        wr_byte(&mut self.i2c, self.addr, reg::SYSTEM__MODE_START, 0x00)
            .await
            .map_err(Error::Bus)?;

        #[cfg(feature = "defmt")]
        defmt::debug!("vl53l1x: ranging stopped");

        Ok(())
    }

    /// Clear the hardware interrupt, arming the sensor for the next measurement.
    ///
    /// **Always** call this after reading a result, or the sensor will not
    /// assert data-ready for the next sample.
    pub async fn clear_interrupt(&mut self) -> Result<(), Error<I::Error>> {
        wr_byte(&mut self.i2c, self.addr, reg::SYSTEM__INTERRUPT_CLEAR, 0x01)
            .await
            .map_err(Error::Bus)?;
        Ok(())
    }

    // =========================================================================
    // Data-ready polling (async, bounded, no busy-spin)
    // =========================================================================

    /// Asynchronously poll until a new ranging sample is available, or until
    /// `timeout` elapses.
    ///
    /// Each poll yields to the async executor via `Timer::after(1 ms).await`,
    /// keeping CPU usage near zero and allowing other Embassy tasks to run.
    ///
    /// Returns `Err(Error::Timeout)` if the sensor has not signalled data-ready
    /// within `DATA_READY_MAX_POLLS` iterations (roughly 1 000 ms).
    pub async fn wait_for_data_ready(
        &mut self,
        _timeout: Duration,
    ) -> Result<(), Error<I::Error>> {
        for _ in 0..DATA_READY_MAX_POLLS {
            if self.is_data_ready().await? {
                return Ok(());
            }
            Timer::after(Duration::from_millis(DATA_READY_POLL_MS)).await;
        }

        #[cfg(feature = "defmt")]
        defmt::warn!("vl53l1x: wait_for_data_ready timed out");

        Err(Error::Timeout)
    }

    /// Return `true` if a new ranging sample is ready.
    ///
    /// Reads `GPIO__TIO_HV_STATUS` and compares bit 0 against the configured
    /// interrupt polarity (from `GPIO_HV_MUX__CTRL`), exactly as in the ULD's
    /// `VL53L1X_CheckForDataReady`.
    pub async fn is_data_ready(&mut self) -> Result<bool, Error<I::Error>> {
        let polarity = self.interrupt_polarity().await?;
        let status = rd_byte(&mut self.i2c, self.addr, reg::GPIO__TIO_HV_STATUS)
            .await
            .map_err(Error::Bus)?;
        let pol_bit = match polarity {
            InterruptPolarity::ActiveHigh => 1u8,
            InterruptPolarity::ActiveLow => 0u8,
        };
        Ok((status & 0x01) == pol_bit)
    }

    // =========================================================================
    // Measurement retrieval
    // =========================================================================

    /// Retrieve the latest ranging result via a single 17-byte burst read.
    ///
    /// This is the most efficient way to get all measurement fields at once,
    /// equivalent to `VL53L1X_GetResult` in the ULD.
    ///
    /// You should call `wait_for_data_ready()` before this function, and
    /// `clear_interrupt()` after.
    pub async fn get_result(&mut self) -> Result<RangingResult, Error<I::Error>> {
        let mut buf = [0u8; 17];
        read_multi(&mut self.i2c, self.addr, reg::RESULT__RANGE_STATUS, &mut buf)
            .await
            .map_err(Error::Bus)?;
        let result = RangingResult::from_burst_bytes(&buf);

        #[cfg(feature = "defmt")]
        defmt::trace!(
            "vl53l1x: result status={:?} dist={}mm",
            result.status,
            result.distance_mm
        );

        Ok(result)
    }

    // =========================================================================
    // Interrupt polarity
    // =========================================================================

    /// Read the current interrupt polarity from `GPIO_HV_MUX__CTRL`.
    pub async fn interrupt_polarity(&mut self) -> Result<InterruptPolarity, Error<I::Error>> {
        let raw = rd_byte(&mut self.i2c, self.addr, reg::GPIO_HV_MUX__CTRL)
            .await
            .map_err(Error::Bus)?;
        // Bit 4: 1 = active-low output, 0 = active-high
        if (raw & 0x10) != 0 {
            Ok(InterruptPolarity::ActiveLow)
        } else {
            Ok(InterruptPolarity::ActiveHigh)
        }
    }

    /// Set the interrupt polarity.
    ///
    /// The default after `init()` is `ActiveLow` (bit 4 = 1 in the config patch).
    pub async fn set_interrupt_polarity(
        &mut self,
        polarity: InterruptPolarity,
    ) -> Result<(), Error<I::Error>> {
        let raw = rd_byte(&mut self.i2c, self.addr, reg::GPIO_HV_MUX__CTRL)
            .await
            .map_err(Error::Bus)?;
        let new = match polarity {
            InterruptPolarity::ActiveHigh => raw & 0xEF, // clear bit 4
            InterruptPolarity::ActiveLow => raw | 0x10,  // set bit 4
        };
        wr_byte(&mut self.i2c, self.addr, reg::GPIO_HV_MUX__CTRL, new)
            .await
            .map_err(Error::Bus)
    }

    // =========================================================================
    // Distance mode
    // =========================================================================

    /// Return the current distance mode.
    pub async fn distance_mode(&mut self) -> Result<DistanceMode, Error<I::Error>> {
        let raw = rd_byte(
            &mut self.i2c,
            self.addr,
            reg::PHASECAL_CONFIG__TIMEOUT_MACROP,
        )
        .await
        .map_err(Error::Bus)?;
        DistanceMode::from_phasecal(raw)
    }

    /// Set the sensing distance mode.
    ///
    /// The timing budget is read, the distance-mode registers are reprogrammed,
    /// and then the original timing budget is restored — exactly as the ULD does.
    pub async fn set_distance_mode(&mut self, mode: DistanceMode) -> Result<(), Error<I::Error>> {
        // Save current timing budget so we can restore it after reprogramming.
        let tb = self.timing_budget().await?;

        match mode {
            DistanceMode::Short => {
                wr_byte(
                    &mut self.i2c,
                    self.addr,
                    reg::PHASECAL_CONFIG__TIMEOUT_MACROP,
                    0x14,
                )
                .await
                .map_err(Error::Bus)?;
                wr_byte(
                    &mut self.i2c,
                    self.addr,
                    reg::RANGE_CONFIG__VCSEL_PERIOD_A,
                    0x07,
                )
                .await
                .map_err(Error::Bus)?;
                wr_byte(
                    &mut self.i2c,
                    self.addr,
                    reg::RANGE_CONFIG__VCSEL_PERIOD_B,
                    0x05,
                )
                .await
                .map_err(Error::Bus)?;
                wr_byte(
                    &mut self.i2c,
                    self.addr,
                    reg::RANGE_CONFIG__VALID_PHASE_HIGH,
                    0x38,
                )
                .await
                .map_err(Error::Bus)?;
                wr_word(&mut self.i2c, self.addr, reg::SD_CONFIG__WOI_SD0, 0x0705)
                    .await
                    .map_err(Error::Bus)?;
                wr_word(
                    &mut self.i2c,
                    self.addr,
                    reg::SD_CONFIG__INITIAL_PHASE_SD0,
                    0x0606,
                )
                .await
                .map_err(Error::Bus)?;
            }
            DistanceMode::Long => {
                wr_byte(
                    &mut self.i2c,
                    self.addr,
                    reg::PHASECAL_CONFIG__TIMEOUT_MACROP,
                    0x0A,
                )
                .await
                .map_err(Error::Bus)?;
                wr_byte(
                    &mut self.i2c,
                    self.addr,
                    reg::RANGE_CONFIG__VCSEL_PERIOD_A,
                    0x0F,
                )
                .await
                .map_err(Error::Bus)?;
                wr_byte(
                    &mut self.i2c,
                    self.addr,
                    reg::RANGE_CONFIG__VCSEL_PERIOD_B,
                    0x0D,
                )
                .await
                .map_err(Error::Bus)?;
                wr_byte(
                    &mut self.i2c,
                    self.addr,
                    reg::RANGE_CONFIG__VALID_PHASE_HIGH,
                    0xB8,
                )
                .await
                .map_err(Error::Bus)?;
                wr_word(&mut self.i2c, self.addr, reg::SD_CONFIG__WOI_SD0, 0x0F0D)
                    .await
                    .map_err(Error::Bus)?;
                wr_word(
                    &mut self.i2c,
                    self.addr,
                    reg::SD_CONFIG__INITIAL_PHASE_SD0,
                    0x0E0E,
                )
                .await
                .map_err(Error::Bus)?;
            }
        }

        // Restore timing budget (some budget values are mode-specific).
        self.set_timing_budget(tb).await
    }

    // =========================================================================
    // Timing budget
    // =========================================================================

    /// Read the current timing budget.
    pub async fn timing_budget(&mut self) -> Result<TimingBudget, Error<I::Error>> {
        let raw = rd_word(
            &mut self.i2c,
            self.addr,
            reg::RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
        )
        .await
        .map_err(Error::Bus)?;
        TimingBudget::from_macrop_a(raw)
    }

    /// Set the ranging timing budget.
    ///
    /// Returns [`Error::InvalidArgument`] when `Ms15` is requested in
    /// [`DistanceMode::Long`].
    pub async fn set_timing_budget(&mut self, budget: TimingBudget) -> Result<(), Error<I::Error>> {
        let mode = self.distance_mode().await?;
        let (a_hi, b_hi) = budget.register_pair(mode).ok_or(Error::InvalidArgument)?;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
            a_hi,
        )
        .await
        .map_err(Error::Bus)?;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
            b_hi,
        )
        .await
        .map_err(Error::Bus)?;
        Ok(())
    }

    // =========================================================================
    // Intermeasurement period
    // =========================================================================

    /// Set the inter-measurement period in milliseconds.
    ///
    /// The hardware register is a 32-bit value scaled by the oscillator
    /// calibration value (`ClockPLL & 0x3FF`), multiplied by 1.075.
    /// This crate avoids floating point entirely by using fixed-point
    /// integer arithmetic: `period_ms * clock_pll * 1075 / 1000`.
    ///
    /// `period_ms` must be ≥ the current timing budget (not checked here —
    /// the caller is responsible per the ULD contract).
    pub async fn set_inter_measurement_ms(
        &mut self,
        period_ms: u32,
    ) -> Result<(), Error<I::Error>> {
        let clock_pll = (rd_word(
            &mut self.i2c,
            self.addr,
            reg::RESULT__OSC_CALIBRATE_VAL,
        )
        .await
        .map_err(Error::Bus)?) & 0x03FF;

        // Fixed-point equivalent of: ClockPLL * period_ms * 1.075
        let reg_val = (clock_pll as u32)
            .wrapping_mul(period_ms)
            .wrapping_mul(1075)
            / 1000;

        wr_dword(
            &mut self.i2c,
            self.addr,
            reg::SYSTEM__INTERMEASUREMENT_PERIOD,
            reg_val,
        )
        .await
        .map_err(Error::Bus)
    }

    /// Read back the inter-measurement period in milliseconds.
    pub async fn inter_measurement_ms(&mut self) -> Result<u32, Error<I::Error>> {
        let raw = rd_dword(
            &mut self.i2c,
            self.addr,
            reg::SYSTEM__INTERMEASUREMENT_PERIOD,
        )
        .await
        .map_err(Error::Bus)?;

        let clock_pll = (rd_word(
            &mut self.i2c,
            self.addr,
            reg::RESULT__OSC_CALIBRATE_VAL,
        )
        .await
        .map_err(Error::Bus)?) & 0x03FF;

        // Inverse: raw / (ClockPLL * 1.065) → fixed-point: raw * 1000 / (ClockPLL * 1065)
        let divisor = (clock_pll as u32).wrapping_mul(1065);
        if divisor == 0 {
            return Err(Error::InvalidData);
        }
        Ok((raw * 1000) / divisor)
    }

    // =========================================================================
    // ROI (Region of Interest)
    // =========================================================================

    /// Set the ROI size (width × height in SPADs).
    ///
    /// Valid range: 4–16 for both dimensions. Clamps to 16 if larger.
    /// When either dimension > 10, the optical centre is overridden to 199
    /// (the full-field default), matching the ULD behaviour.
    pub async fn set_roi(&mut self, x: u8, y: u8) -> Result<(), Error<I::Error>> {
        if x < 4 || y < 4 {
            return Err(Error::InvalidArgument);
        }
        let x = x.min(16);
        let y = y.min(16);

        let optical_centre = if x > 10 || y > 10 {
            199u8
        } else {
            rd_byte(
                &mut self.i2c,
                self.addr,
                reg::ROI_CONFIG__MODE_ROI_CENTRE_SPAD,
            )
            .await
            .map_err(Error::Bus)?
        };

        wr_byte(
            &mut self.i2c,
            self.addr,
            reg::ROI_CONFIG__USER_ROI_CENTRE_SPAD,
            optical_centre,
        )
        .await
        .map_err(Error::Bus)?;

        // Packs Y-1 into bits [7:4] and X-1 into bits [3:0]
        let xy = ((y - 1) << 4) | (x - 1);
        wr_byte(
            &mut self.i2c,
            self.addr,
            reg::ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
            xy,
        )
        .await
        .map_err(Error::Bus)
    }

    /// Read back the current ROI size as `(width, height)` in SPADs.
    pub async fn roi(&mut self) -> Result<(u8, u8), Error<I::Error>> {
        let raw = rd_byte(
            &mut self.i2c,
            self.addr,
            reg::ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
        )
        .await
        .map_err(Error::Bus)?;
        let x = (raw & 0x0F) + 1;
        let y = (raw >> 4) + 1;
        Ok((x, y))
    }

    /// Set the ROI centre SPAD directly (advanced use; see ULD docs for SPAD map).
    pub async fn set_roi_center(&mut self, centre_spad: u8) -> Result<(), Error<I::Error>> {
        wr_byte(
            &mut self.i2c,
            self.addr,
            reg::ROI_CONFIG__USER_ROI_CENTRE_SPAD,
            centre_spad,
        )
        .await
        .map_err(Error::Bus)
    }

    /// Read the current ROI centre SPAD.
    pub async fn roi_center(&mut self) -> Result<u8, Error<I::Error>> {
        rd_byte(
            &mut self.i2c,
            self.addr,
            reg::ROI_CONFIG__USER_ROI_CENTRE_SPAD,
        )
        .await
        .map_err(Error::Bus)
    }

    // =========================================================================
    // Distance threshold
    // =========================================================================

    /// Configure the hardware distance-threshold interrupt.
    ///
    /// * `low_mm` / `high_mm` — threshold values in millimetres.
    /// * `window` — trigger condition (Below / Above / OutOfWindow / InWindow).
    /// * `int_on_no_target` — when `true`, also trigger when no target is seen.
    pub async fn set_distance_threshold(
        &mut self,
        low_mm: u16,
        high_mm: u16,
        window: ThresholdWindow,
        int_on_no_target: bool,
    ) -> Result<(), Error<I::Error>> {
        let raw = rd_byte(&mut self.i2c, self.addr, reg::SYSTEM__INTERRUPT_CONFIG_GPIO)
            .await
            .map_err(Error::Bus)?;
        let masked = raw & 0x47;
        let new = if int_on_no_target {
            (masked | (window as u8 & 0x07)) | 0x40
        } else {
            masked | (window as u8 & 0x07)
        };
        wr_byte(
            &mut self.i2c,
            self.addr,
            reg::SYSTEM__INTERRUPT_CONFIG_GPIO,
            new,
        )
        .await
        .map_err(Error::Bus)?;
        wr_word(&mut self.i2c, self.addr, reg::SYSTEM__THRESH_HIGH, high_mm)
            .await
            .map_err(Error::Bus)?;
        wr_word(&mut self.i2c, self.addr, reg::SYSTEM__THRESH_LOW, low_mm)
            .await
            .map_err(Error::Bus)
    }

    // =========================================================================
    // Signal / sigma thresholds
    // =========================================================================

    /// Set the minimum signal threshold in kcps (default 1024 kcps).
    ///
    /// The register stores the value >> 3.
    pub async fn set_signal_threshold_kcps(&mut self, kcps: u16) -> Result<(), Error<I::Error>> {
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
            kcps >> 3,
        )
        .await
        .map_err(Error::Bus)
    }

    /// Read the signal threshold in kcps.
    pub async fn signal_threshold_kcps(&mut self) -> Result<u16, Error<I::Error>> {
        let raw = rd_word(
            &mut self.i2c,
            self.addr,
            reg::RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
        )
        .await
        .map_err(Error::Bus)?;
        Ok(raw << 3)
    }

    /// Set the sigma (standard deviation) threshold in mm (default 15 mm).
    ///
    /// The register uses 14.2 fixed-point: value << 2.
    /// Returns [`Error::InvalidArgument`] if `sigma > 16383` (would overflow).
    pub async fn set_sigma_threshold_mm(&mut self, sigma: u16) -> Result<(), Error<I::Error>> {
        if sigma > (0xFFFF >> 2) {
            return Err(Error::InvalidArgument);
        }
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::RANGE_CONFIG__SIGMA_THRESH,
            sigma << 2,
        )
        .await
        .map_err(Error::Bus)
    }

    /// Read the sigma threshold in mm.
    pub async fn sigma_threshold_mm(&mut self) -> Result<u16, Error<I::Error>> {
        let raw = rd_word(&mut self.i2c, self.addr, reg::RANGE_CONFIG__SIGMA_THRESH)
            .await
            .map_err(Error::Bus)?;
        Ok(raw >> 2)
    }

    // =========================================================================
    // Offset calibration
    // =========================================================================

    /// Set a fixed offset correction value.
    ///
    /// Internally stored as `offset_mm * 4` in the register, with inner/outer
    /// offset registers zeroed (matches ULD `VL53L1X_SetOffset`).
    pub async fn set_offset_mm(&mut self, offset_mm: i16) -> Result<(), Error<I::Error>> {
        let raw = (offset_mm * 4) as u16;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__PART_TO_PART_RANGE_OFFSET_MM,
            raw,
        )
        .await
        .map_err(Error::Bus)?;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::MM_CONFIG__INNER_OFFSET_MM,
            0x0000,
        )
        .await
        .map_err(Error::Bus)?;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::MM_CONFIG__OUTER_OFFSET_MM,
            0x0000,
        )
        .await
        .map_err(Error::Bus)
    }

    /// Read the current offset correction value in mm.
    pub async fn offset_mm(&mut self) -> Result<i16, Error<I::Error>> {
        let raw = rd_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__PART_TO_PART_RANGE_OFFSET_MM,
        )
        .await
        .map_err(Error::Bus)?;
        // Sign-extend the 13-bit two's-complement value (shift left 3, arithmetic shift right 5)
        let extended = ((raw << 3) as i16) >> 5;
        // Handle wrap-around for large positive values representing negatives
        if extended > 1024 {
            Ok(extended - 2048)
        } else {
            Ok(extended)
        }
    }

    // =========================================================================
    // Xtalk (crosstalk) calibration
    // =========================================================================

    /// Set the crosstalk compensation value in cps (counts per second).
    ///
    /// Converted to 7.9 fixed-point kcps: `(cps << 9) / 1000`.
    pub async fn set_xtalk_cps(&mut self, cps: u16) -> Result<(), Error<I::Error>> {
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
            0x0000,
        )
        .await
        .map_err(Error::Bus)?;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
            0x0000,
        )
        .await
        .map_err(Error::Bus)?;
        let kcps = ((cps as u32) << 9) / 1000;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
            kcps as u16,
        )
        .await
        .map_err(Error::Bus)
    }

    /// Read back the xtalk compensation value in cps.
    pub async fn xtalk_cps(&mut self) -> Result<u16, Error<I::Error>> {
        let raw = rd_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
        )
        .await
        .map_err(Error::Bus)?;
        // Inverse: kcps -> cps: (raw * 1000) >> 9
        Ok(((raw as u32 * 1000) >> 9) as u16)
    }

    // =========================================================================
    // Temperature recalibration
    // =========================================================================

    /// Perform an in-situ temperature recalibration (VHV update).
    ///
    /// Call this whenever the ambient temperature has changed by more than 8 °C
    /// since the last calibration or since `init()`.
    ///
    /// This temporarily starts ranging, obtains one sample, then stops again —
    /// the caller should not be in active ranging when this is called.
    pub async fn start_temperature_update(&mut self) -> Result<(), Error<I::Error>> {
        // Full VHV recal
        wr_byte(
            &mut self.i2c,
            self.addr,
            reg::VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
            0x81,
        )
        .await
        .map_err(Error::Bus)?;
        wr_byte(&mut self.i2c, self.addr, reg::VHV_CONFIG__INIT, 0x92)
            .await
            .map_err(Error::Bus)?;
        self.start_ranging().await?;
        self.wait_for_data_ready(Duration::from_millis(1_100)).await?;
        self.clear_interrupt().await?;
        self.stop_ranging().await?;
        // Restore two-bounds VHV for normal operation
        wr_byte(
            &mut self.i2c,
            self.addr,
            reg::VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
            0x09,
        )
        .await
        .map_err(Error::Bus)?;
        wr_byte(&mut self.i2c, self.addr, reg::VHV_CONFIG__INIT, 0x00)
            .await
            .map_err(Error::Bus)
    }

    // =========================================================================
    // Calibration routines
    // =========================================================================

    /// Perform offset calibration against a target at a known distance.
    ///
    /// **Sensor must NOT be ranging when this is called.**
    ///
    /// Averages 50 measurements and programs the resulting offset into the
    /// sensor. The computed offset is also returned so the caller may persist it.
    ///
    /// `target_mm` — known target distance in mm (ST recommends 100 mm with a
    /// grey 17% reflectance target).
    pub async fn calibrate_offset(&mut self, target_mm: u16) -> Result<i16, Error<I::Error>> {
        // Zero out any existing offset
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__PART_TO_PART_RANGE_OFFSET_MM,
            0x0000,
        )
        .await
        .map_err(Error::Bus)?;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::MM_CONFIG__INNER_OFFSET_MM,
            0x0000,
        )
        .await
        .map_err(Error::Bus)?;
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::MM_CONFIG__OUTER_OFFSET_MM,
            0x0000,
        )
        .await
        .map_err(Error::Bus)?;

        self.start_ranging().await?;

        let mut sum: i32 = 0;
        for _ in 0u8..50 {
            self.wait_for_data_ready(Duration::from_millis(1_100)).await?;
            let result = self.get_result().await?;
            self.clear_interrupt().await?;
            sum += result.distance_mm as i32;
        }

        self.stop_ranging().await?;

        let average = (sum / 50) as i16;
        let offset = target_mm as i16 - average;

        // Program the discovered offset
        self.set_offset_mm(offset).await?;

        #[cfg(feature = "defmt")]
        defmt::info!(
            "vl53l1x: offset calibration done, offset={}mm",
            offset
        );

        Ok(offset)
    }

    /// Perform xtalk (cover-glass) calibration.
    ///
    /// **Sensor must NOT be ranging when this is called.**
    ///
    /// Averages 50 measurements. Uses only integer arithmetic (no `f32`),
    /// computing the xtalk formula in fixed-point via scaled intermediaries.
    ///
    /// `target_mm` — distance to the inflection point target (where the sensor
    /// begins to "under-range" due to cover-glass reflections).
    ///
    /// Returns the computed xtalk value in cps, which is also programmed into
    /// the sensor.
    pub async fn calibrate_xtalk(&mut self, target_mm: u16) -> Result<u16, Error<I::Error>> {
        // Zero existing xtalk
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
            0x0000,
        )
        .await
        .map_err(Error::Bus)?;

        self.start_ranging().await?;

        let mut sum_dist: u32 = 0;
        let mut sum_spads: u32 = 0;
        let mut sum_signal: u32 = 0; // in raw register units (×8 KCPS)

        for _ in 0u8..50 {
            self.wait_for_data_ready(Duration::from_millis(1_100)).await?;
            let result = self.get_result().await?;
            self.clear_interrupt().await?;
            sum_dist += result.distance_mm as u32;
            sum_spads += result.num_spads as u32;
            sum_signal += result.signal_kcps as u32;
        }

        self.stop_ranging().await?;

        // averages (×50 precision to defer division)
        // avg_dist = sum_dist / 50 (in mm)
        // avg_spads = sum_spads / 50
        // avg_signal = sum_signal / 50 (in kcps×8 units)
        //
        // Xtalk formula (from ULD calibration.c):
        //   xtalk_kcps = 512 * avg_signal * (1 - avg_dist / target_mm) / avg_spads
        //
        // Integer equivalent (avoiding all floats):
        //   numer = sum_signal * (50*target_mm - sum_dist/50*50) / 50
        //         = sum_signal * (50*target_mm - sum_dist) / 50
        //   numer = sum_signal * (50 * target_mm as u32 - sum_dist)
        //   denom = sum_spads * 50
        //   xtalk_kcps_scaled = 512 * numer / (denom * 50)   [×8 due to signal units]
        //   xtalk_kcps = xtalk_kcps_scaled / 8

        if sum_spads == 0 {
            return Err(Error::InvalidData);
        }

        let target = target_mm as u32;
        // (1 - avg_dist/target) in fixed-point ×50: (50*target - sum_dist) / 50
        let factor = 50u32
            .checked_mul(target)
            .and_then(|v| v.checked_sub(sum_dist))
            .ok_or(Error::InvalidData)?;

        // xtalk in raw 7.9 format (kcps) — matching `VL53L1X_CalibrateXtalk` output
        // Formula: 512 * (avg_signal/8) * (factor/50) / avg_spads
        //        = 512 * sum_signal * factor / (50 * 8 * 50 * sum_spads/50)
        //        = sum_signal * factor * 512 / (50 * 8 * sum_spads)
        let cal_xtalk = sum_signal
            .saturating_mul(factor)
            .saturating_mul(512)
            / (50 * 8 * sum_spads);

        let cal_xtalk = cal_xtalk.min(0xFFFF) as u16;

        // Convert to cps for the public API: (cal_xtalk * 1000) >> 9
        // then program the raw kcps value directly
        wr_word(
            &mut self.i2c,
            self.addr,
            reg::ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
            cal_xtalk,
        )
        .await
        .map_err(Error::Bus)?;

        let xtalk_cps = ((cal_xtalk as u32 * 1000) >> 9) as u16;

        #[cfg(feature = "defmt")]
        defmt::info!(
            "vl53l1x: xtalk calibration done, xtalk={}cps (raw=0x{:04X})",
            xtalk_cps,
            cal_xtalk
        );

        Ok(xtalk_cps)
    }

    // =========================================================================
    // Raw register access for diagnostics
    // =========================================================================

    /// Read a raw byte from any 16-bit register.
    ///
    /// For diagnostics and debug hooks only. Does not mutate driver state.
    #[inline]
    pub async fn read_register(&mut self, reg: u16) -> Result<u8, Error<I::Error>> {
        rd_byte(&mut self.i2c, self.addr, reg)
            .await
            .map_err(Error::Bus)
    }

    /// Read a raw 16-bit word from any 16-bit register.
    #[inline]
    pub async fn read_register_u16(&mut self, reg: u16) -> Result<u16, Error<I::Error>> {
        rd_word(&mut self.i2c, self.addr, reg)
            .await
            .map_err(Error::Bus)
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    /// Poll `FIRMWARE__SYSTEM_STATUS` until the sensor reports it has booted
    /// (value != 0), or until `BOOT_MAX_POLLS` attempts have been exhausted.
    async fn wait_for_boot(&mut self) -> Result<(), Error<I::Error>> {
        for _ in 0..BOOT_MAX_POLLS {
            let state =
                rd_byte(&mut self.i2c, self.addr, reg::FIRMWARE__SYSTEM_STATUS)
                    .await
                    .map_err(Error::Bus)?;
            if state != 0 {
                #[cfg(feature = "defmt")]
                defmt::debug!("vl53l1x: boot complete (status=0x{:02X})", state);
                return Ok(());
            }
            Timer::after(Duration::from_millis(BOOT_POLL_MS)).await;
        }

        #[cfg(feature = "defmt")]
        defmt::error!("vl53l1x: boot timed out");

        Err(Error::Timeout)
    }
}

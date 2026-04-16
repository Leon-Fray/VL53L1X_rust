//! Mock-HAL integration tests for the VL53L1X driver.
//!
//! These tests run on the host using `embedded-hal-mock` to simulate
//! I2C bus transactions without real hardware.
//!
//! Run with:
//! ```sh
//! cargo test --tests
//! ```

use embedded_hal_async::i2c::{I2c, Operation};
use embedded_hal::i2c::I2c as BlockingI2c;
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};
use vl53l1x::{
    config::{DistanceMode, TimingBudget},
    types::{RangeStatus, RangingResult},
    Error, Vl53l1x,
};

// ─── Async wrapper for Mock ───────────────────────────────────────────────────
// embedded-hal-mock 0.11 only implements the blocking I2C traits. We write a
// trivial adapter to implement the embedded-hal-async I2c trait for tests.

struct AsyncMock(I2cMock);

impl embedded_hal::i2c::ErrorType for AsyncMock {
    type Error = embedded_hal::i2c::ErrorKind;
}

impl I2c for AsyncMock {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(address, read)
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.0.write(address, write)
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.write_read(address, write, read)
    }

    async fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        unimplemented!("transaction mock not used in driver")
    }
}

// ─── Async executor ───────────────────────────────────────────────────────────

fn block_on<F: core::future::Future>(f: F) -> F::Output {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(f)
}

// ─── RangeStatus decode tests ─────────────────────────────────────────────────

#[test]
fn range_status_valid_from_raw() {
    // Raw HW byte 0x09 → status_rtn[9] = 0 → Valid
    assert_eq!(RangeStatus::from_raw(0x09), RangeStatus::Valid);
}

#[test]
fn range_status_sigma_failed() {
    // Raw HW byte 0x06 → status_rtn[6] = 1 → SigmaFailed
    assert_eq!(RangeStatus::from_raw(0x06), RangeStatus::SigmaFailed);
}

#[test]
fn range_status_not_updated_for_unknown() {
    // Any raw value not in the table → 255 → NotUpdated
    assert_eq!(RangeStatus::from_raw(0xFF), RangeStatus::NotUpdated);
}

#[test]
fn range_status_all_known_codes() {
    // Exhaustive check of all 13 known codes from AN5573
    let cases: &[(u8, RangeStatus)] = &[
        (9, RangeStatus::Valid),
        (6, RangeStatus::SigmaFailed),
        (4, RangeStatus::SignalFailed),
        (8, RangeStatus::TargetBelowMinRange),
        (5, RangeStatus::InvalidPhase),
        (3, RangeStatus::HardwareFail),
        (19, RangeStatus::NoWrapAroundCheck),
        (7, RangeStatus::WrapTargetFail),
        (12, RangeStatus::ProcessingFail),
        (18, RangeStatus::XtalkSignalFail),
        (22, RangeStatus::SynchronisationInt),
        (23, RangeStatus::MergedPulseDetected),
        (13, RangeStatus::TargetTooClose),
    ];
    for &(raw, expected) in cases {
        assert_eq!(RangeStatus::from_raw(raw), expected, "raw = 0x{:02X}", raw);
    }
}

// ─── RangingResult burst-parse test ───────────────────────────────────────────

#[test]
fn ranging_result_parses_burst_correctly() {
    let mut buf = [0u8; 17];
    buf[0] = 0x09;
    buf[3] = 0x20;
    buf[7] = 0x01;
    buf[8] = 0x00;
    buf[13] = 0x03;
    buf[14] = 0xE8;
    buf[15] = 0x00;
    buf[16] = 0x50;

    let r = RangingResult::from_burst_bytes(&buf);
    assert_eq!(r.status, RangeStatus::Valid);
    assert!(r.status.is_valid());
    assert_eq!(r.distance_mm, 1000);
    assert_eq!(r.num_spads, 32);
    assert_eq!(r.ambient_kcps, 2048);
    assert_eq!(r.signal_kcps, 640);
}

// ─── TimingBudget register-pair tests ────────────────────────────────────────

#[test]
fn timing_budget_short_ms100_pair() {
    let pair = TimingBudget::Ms100.register_pair(DistanceMode::Short);
    assert_eq!(pair, Some((0x02E1, 0x0388)));
}

#[test]
fn timing_budget_long_ms100_pair() {
    let pair = TimingBudget::Ms100.register_pair(DistanceMode::Long);
    assert_eq!(pair, Some((0x01CC, 0x01EA)));
}

#[test]
fn timing_budget_ms15_only_valid_in_short() {
    assert!(TimingBudget::Ms15.register_pair(DistanceMode::Short).is_some());
    assert!(TimingBudget::Ms15.register_pair(DistanceMode::Long).is_none());
}

#[test]
fn timing_budget_from_macrop_a_roundtrip() {
    let cases: &[(u16, TimingBudget)] = &[
        (0x001D, TimingBudget::Ms15),
        (0x0051, TimingBudget::Ms20),
        (0x001E, TimingBudget::Ms20),
        (0x00D6, TimingBudget::Ms33),
        (0x0060, TimingBudget::Ms33),
        (0x01AE, TimingBudget::Ms50),
        (0x00AD, TimingBudget::Ms50),
        (0x02E1, TimingBudget::Ms100),
        (0x01CC, TimingBudget::Ms100),
        (0x03E1, TimingBudget::Ms200),
        (0x02D9, TimingBudget::Ms200),
        (0x0591, TimingBudget::Ms500),
        (0x048F, TimingBudget::Ms500),
    ];
    for &(raw, expected) in cases {
        let decoded: Result<TimingBudget, Error<()>> = TimingBudget::from_macrop_a(raw);
        assert_eq!(decoded.unwrap(), expected, "raw = 0x{:04X}", raw);
    }
}

#[test]
fn timing_budget_invalid_macrop_returns_invalid_data() {
    let result: Result<TimingBudget, Error<()>> = TimingBudget::from_macrop_a(0xDEAD);
    assert!(matches!(result, Err(Error::InvalidData)));
}

// ─── DistanceMode decode tests ────────────────────────────────────────────────

#[test]
fn distance_mode_from_phasecal_short() {
    let mode: Result<DistanceMode, Error<()>> = DistanceMode::from_phasecal(0x14);
    assert_eq!(mode.unwrap(), DistanceMode::Short);
}

#[test]
fn distance_mode_from_phasecal_long() {
    let mode: Result<DistanceMode, Error<()>> = DistanceMode::from_phasecal(0x0A);
    assert_eq!(mode.unwrap(), DistanceMode::Long);
}

#[test]
fn distance_mode_from_phasecal_invalid() {
    let mode: Result<DistanceMode, Error<()>> = DistanceMode::from_phasecal(0xFF);
    assert!(matches!(mode, Err(Error::InvalidData)));
}

// ─── Error type tests ─────────────────────────────────────────────────────────

#[test]
fn error_from_bus_wraps_correctly() {
    let bus_err: u8 = 42;
    let err: Error<u8> = Error::from(bus_err);
    assert!(matches!(err, Error::Bus(42)));
}

// ─── Register address constant sanity checks ─────────────────────────────────

#[test]
fn register_constants_correct_addresses() {
    use vl53l1x::regs;
    assert_eq!(regs::SYSTEM__MODE_START, 0x0087);
    assert_eq!(regs::SYSTEM__INTERRUPT_CLEAR, 0x0086);
    assert_eq!(regs::RESULT__RANGE_STATUS, 0x0089);
    assert_eq!(regs::IDENTIFICATION__MODEL_ID, 0x010F);
    assert_eq!(regs::FIRMWARE__SYSTEM_STATUS, 0x00E5);
    assert_eq!(regs::ROI_CONFIG__USER_ROI_CENTRE_SPAD, 0x007F);
    assert_eq!(regs::DEFAULT_CONFIG_START, 0x002D);
}

#[test]
fn default_config_patch_length_matches_register_range() {
    use vl53l1x::regs;
    let expected_len = (regs::DEFAULT_CONFIG_END - regs::DEFAULT_CONFIG_START + 1) as usize;
    assert_eq!(expected_len, 91);
    assert_eq!(regs::DEFAULT_CONFIGURATION.len(), 91);
}

// ─── Fault injection: sensor_id mismatch ─────────────────────────────────────

#[test]
fn sensor_id_returns_invalid_data_on_bad_id() {
    let expectations = [I2cTx::write_read(0x29, vec![0x01u8, 0x0F], vec![0xDE, 0xAD])];
    let mock = I2cMock::new(&expectations);
    let mut i2c = AsyncMock(mock);

    let result = block_on(async {
        let mut sensor = Vl53l1x::new(&mut i2c, 0x29);
        sensor.sensor_id().await
    });

    assert!(matches!(result, Err(Error::InvalidData)));
    i2c.0.done();
}

// ─── Wire format verification ─────────────────────────────────────────────────

#[test]
fn start_ranging_writes_correct_bytes() {
    let expectations = [I2cTx::write(0x29, vec![0x00u8, 0x87, 0x40])];
    let mock = I2cMock::new(&expectations);
    let mut i2c = AsyncMock(mock);

    let result = block_on(async {
        let mut sensor = Vl53l1x::new(&mut i2c, 0x29);
        sensor.start_ranging().await
    });

    assert!(result.is_ok());
    i2c.0.done();
}

#[test]
fn stop_ranging_writes_correct_bytes() {
    let expectations = [I2cTx::write(0x29, vec![0x00u8, 0x87, 0x00])];
    let mock = I2cMock::new(&expectations);
    let mut i2c = AsyncMock(mock);

    let result = block_on(async {
        let mut sensor = Vl53l1x::new(&mut i2c, 0x29);
        sensor.stop_ranging().await
    });

    assert!(result.is_ok());
    i2c.0.done();
}

#[test]
fn clear_interrupt_writes_correct_bytes() {
    let expectations = [I2cTx::write(0x29, vec![0x00u8, 0x86, 0x01])];
    let mock = I2cMock::new(&expectations);
    let mut i2c = AsyncMock(mock);

    let result = block_on(async {
        let mut sensor = Vl53l1x::new(&mut i2c, 0x29);
        sensor.clear_interrupt().await
    });

    assert!(result.is_ok());
    i2c.0.done();
}

#[test]
fn set_i2c_address_writes_shifted_value() {
    let expectations = [I2cTx::write(0x29, vec![0x00u8, 0x01, 0x18])];
    let mock = I2cMock::new(&expectations);
    let mut i2c = AsyncMock(mock);

    let result = block_on(async {
        let mut sensor = Vl53l1x::new(&mut i2c, 0x29);
        sensor.set_i2c_address(0x30).await
    });

    assert!(result.is_ok());
    i2c.0.done();
}

#[test]
fn set_offset_mm_encodes_times_four() {
    let expectations = [
        I2cTx::write(0x29, vec![0x00u8, 0x1E, 0x00, 0x28]), // 40 = 0x0028
        I2cTx::write(0x29, vec![0x00u8, 0x20, 0x00, 0x00]),
        I2cTx::write(0x29, vec![0x00u8, 0x22, 0x00, 0x00]),
    ];
    let mock = I2cMock::new(&expectations);
    let mut i2c = AsyncMock(mock);

    let result = block_on(async {
        let mut sensor = Vl53l1x::new(&mut i2c, 0x29);
        sensor.set_offset_mm(10).await
    });

    assert!(result.is_ok());
    i2c.0.done();
}

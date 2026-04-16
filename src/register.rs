#![allow(missing_docs)]

/// VL53L1X register map — 16-bit addresses, Big-Endian on the wire.
///
/// Every constant is derived verbatim from VL53L1X_api.h (ULD v3.5.5).
/// Do NOT use raw hex literals for register addresses anywhere else in this
/// crate; always reference these names so that future silicon revisions only
/// require changes in this file.

// ─── Soft reset / I2C ────────────────────────────────────────────────────────
pub const SOFT_RESET: u16 = 0x0000;
pub const I2C_SLAVE__DEVICE_ADDRESS: u16 = 0x0001;

// ─── VHV / temperature ───────────────────────────────────────────────────────
pub const VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND: u16 = 0x0008;
/// Undocumented temperature-compensation register (see SensorInit & StartTemperatureUpdate)
pub const VHV_CONFIG__INIT: u16 = 0x000B;

// ─── Xtalk compensation ───────────────────────────────────────────────────────
pub const ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS: u16 = 0x0016;
pub const ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS: u16 = 0x0018;
pub const ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS: u16 = 0x001A;

// ─── Offset ───────────────────────────────────────────────────────────────────
pub const ALGO__PART_TO_PART_RANGE_OFFSET_MM: u16 = 0x001E;
pub const MM_CONFIG__INNER_OFFSET_MM: u16 = 0x0020;
pub const MM_CONFIG__OUTER_OFFSET_MM: u16 = 0x0022;

// ─── GPIO / interrupt ─────────────────────────────────────────────────────────
pub const GPIO_HV_MUX__CTRL: u16 = 0x0030;
pub const GPIO__TIO_HV_STATUS: u16 = 0x0031;
pub const SYSTEM__INTERRUPT_CONFIG_GPIO: u16 = 0x0046;
pub const SYSTEM__INTERRUPT_CLEAR: u16 = 0x0086;

// ─── Phase / VCSEL configuration ──────────────────────────────────────────────
pub const PHASECAL_CONFIG__TIMEOUT_MACROP: u16 = 0x004B;
pub const RANGE_CONFIG__TIMEOUT_MACROP_A_HI: u16 = 0x005E;
pub const RANGE_CONFIG__VCSEL_PERIOD_A: u16 = 0x0060;
pub const RANGE_CONFIG__TIMEOUT_MACROP_B_HI: u16 = 0x0061;
pub const RANGE_CONFIG__TIMEOUT_MACROP_B_LO: u16 = 0x0062;
pub const RANGE_CONFIG__VCSEL_PERIOD_B: u16 = 0x0063;
pub const RANGE_CONFIG__SIGMA_THRESH: u16 = 0x0064;
pub const RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS: u16 = 0x0066;
pub const RANGE_CONFIG__VALID_PHASE_HIGH: u16 = 0x0069;

// ─── Intermeasurement & distance threshold ────────────────────────────────────
pub const SYSTEM__INTERMEASUREMENT_PERIOD: u16 = 0x006C;
pub const SYSTEM__THRESH_HIGH: u16 = 0x0072;
pub const SYSTEM__THRESH_LOW: u16 = 0x0074;

// ─── SD config ────────────────────────────────────────────────────────────────
pub const SD_CONFIG__WOI_SD0: u16 = 0x0078;
pub const SD_CONFIG__INITIAL_PHASE_SD0: u16 = 0x007A;

// ─── ROI ──────────────────────────────────────────────────────────────────────
pub const ROI_CONFIG__USER_ROI_CENTRE_SPAD: u16 = 0x007F;
pub const ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE: u16 = 0x0080;
/// Read-only optical centre determined during manufacturing
pub const ROI_CONFIG__MODE_ROI_CENTRE_SPAD: u16 = 0x013E;

// ─── Sequencer & mode ────────────────────────────────────────────────────────
pub const SYSTEM__SEQUENCE_CONFIG: u16 = 0x0081;
pub const SYSTEM__GROUPED_PARAMETER_HOLD: u16 = 0x0082;
pub const SYSTEM__MODE_START: u16 = 0x0087;

// ─── Result registers ────────────────────────────────────────────────────────
/// Base of the 17-byte result burst read (used in GetResult)
pub const RESULT__RANGE_STATUS: u16 = 0x0089;
pub const RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0: u16 = 0x008C;
pub const RESULT__AMBIENT_COUNT_RATE_MCPS_SD: u16 = 0x0090;
pub const RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0: u16 = 0x0096;
pub const RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0: u16 = 0x0098;

// ─── Oscillator calibration ───────────────────────────────────────────────────
pub const RESULT__OSC_CALIBRATE_VAL: u16 = 0x00DE;

// ─── Firmware & identification ────────────────────────────────────────────────
pub const FIRMWARE__SYSTEM_STATUS: u16 = 0x00E5;
/// Expected value: 0xEEAC
pub const IDENTIFICATION__MODEL_ID: u16 = 0x010F;

// ─── Sensor-specific constants ────────────────────────────────────────────────
/// 7-bit default I2C address (0x52 >> 1 = 0x29)
pub const DEFAULT_I2C_ADDR_7BIT: u8 = 0x29;
/// Model ID that identifies genuine VL53L1X silicon
pub const EXPECTED_MODEL_ID: u16 = 0xEEAC;

/// Start address of the 91-byte default configuration patch (registers 0x2D–0x87)
pub const DEFAULT_CONFIG_START: u16 = 0x2D;
/// Last address of the default configuration patch
pub const DEFAULT_CONFIG_END: u16 = 0x87;

/// The 91-byte default configuration payload.
///
/// Copied verbatim from `VL51L1X_DEFAULT_CONFIGURATION[]` in VL53L1X_api.c
/// (STSW-IMG009 v3.5.5). Written sequentially starting at register 0x2D.
pub const DEFAULT_CONFIGURATION: [u8; 91] = [
    0x00, // 0x2D
    0x00, // 0x2E
    0x00, // 0x2F
    0x01, // 0x30 — GPIO_HV_MUX__CTRL  (active-low interrupt bit set)
    0x02, // 0x31
    0x00, // 0x32
    0x02, // 0x33
    0x08, // 0x34
    0x00, // 0x35
    0x08, // 0x36
    0x10, // 0x37
    0x01, // 0x38
    0x01, // 0x39
    0x00, // 0x3A
    0x00, // 0x3B
    0x00, // 0x3C
    0x00, // 0x3D
    0xFF, // 0x3E
    0x00, // 0x3F
    0x0F, // 0x40
    0x00, // 0x41
    0x00, // 0x42
    0x00, // 0x43
    0x00, // 0x44
    0x00, // 0x45
    0x20, // 0x46 — SYSTEM__INTERRUPT_CONFIG_GPIO (new sample ready)
    0x0B, // 0x47
    0x00, // 0x48
    0x00, // 0x49
    0x02, // 0x4A
    0x0A, // 0x4B — PHASECAL_CONFIG__TIMEOUT_MACROP (Long mode default)
    0x21, // 0x4C
    0x00, // 0x4D
    0x00, // 0x4E
    0x05, // 0x4F
    0x00, // 0x50
    0x00, // 0x51
    0x00, // 0x52
    0x00, // 0x53
    0xC8, // 0x54
    0x00, // 0x55
    0x00, // 0x56
    0x38, // 0x57
    0xFF, // 0x58
    0x01, // 0x59
    0x00, // 0x5A
    0x08, // 0x5B
    0x00, // 0x5C
    0x00, // 0x5D
    0x01, // 0x5E — RANGE_CONFIG__TIMEOUT_MACROP_A_HI (100 ms Long mode)
    0xCC, // 0x5F
    0x0F, // 0x60 — RANGE_CONFIG__VCSEL_PERIOD_A (Long mode)
    0x01, // 0x61 — RANGE_CONFIG__TIMEOUT_MACROP_B_HI
    0xF1, // 0x62 — RANGE_CONFIG__TIMEOUT_MACROP_B_LO
    0x0D, // 0x63 — RANGE_CONFIG__VCSEL_PERIOD_B (Long mode)
    0x01, // 0x64 — RANGE_CONFIG__SIGMA_THRESH MSB (90 mm default)
    0x68, // 0x65 — RANGE_CONFIG__SIGMA_THRESH LSB
    0x00, // 0x66 — RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS MSB
    0x80, // 0x67 — RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS LSB
    0x08, // 0x68
    0xB8, // 0x69 — RANGE_CONFIG__VALID_PHASE_HIGH (Long mode)
    0x00, // 0x6A
    0x00, // 0x6B
    0x00, // 0x6C — SYSTEM__INTERMEASUREMENT_PERIOD MSB (default ~1000 clocks)
    0x00, // 0x6D
    0x0F, // 0x6E
    0x89, // 0x6F — SYSTEM__INTERMEASUREMENT_PERIOD LSB
    0x00, // 0x70
    0x00, // 0x71
    0x00, // 0x72 — SYSTEM__THRESH_HIGH MSB
    0x00, // 0x73
    0x00, // 0x74 — SYSTEM__THRESH_LOW MSB
    0x00, // 0x75
    0x00, // 0x76
    0x01, // 0x77
    0x0F, // 0x78 — SD_CONFIG__WOI_SD0 MSB
    0x0D, // 0x79 — SD_CONFIG__WOI_SD0 LSB
    0x0E, // 0x7A — SD_CONFIG__INITIAL_PHASE_SD0 MSB
    0x0E, // 0x7B — SD_CONFIG__INITIAL_PHASE_SD0 LSB
    0x00, // 0x7C
    0x00, // 0x7D
    0x02, // 0x7E
    0xC7, // 0x7F — ROI_CONFIG__USER_ROI_CENTRE_SPAD (optical centre)
    0xFF, // 0x80 — ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE (16×16)
    0x9B, // 0x81 — SYSTEM__SEQUENCE_CONFIG
    0x00, // 0x82
    0x00, // 0x83
    0x00, // 0x84
    0x01, // 0x85
    0x00, // 0x86 — SYSTEM__INTERRUPT_CLEAR
    0x00, // 0x87 — SYSTEM__MODE_START (ranging stopped)
];

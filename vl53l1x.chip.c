// VL53L1X Wokwi Custom Chip Simulation
//
// Wire protocol: every I2C transaction starts with a 16-bit register address
// (MSB first), then data bytes.  All reads use write_read() which sends the
// address in a WRITE phase and then switches to a READ phase via repeated-START.
//
// Key bugs fixed:
//   1. on_i2c_connect resets addr_state only on STOP, not on START, so the
//      register address set during the WRITE phase survives the repeated-START
//      into the READ phase.
//   2. Data-ready polarity: after the 91-byte config patch, GPIO_HV_MUX__CTRL
//      is 0x01 (bit4=0) → ActiveHigh polarity.  Driver checks bit0=1 for ready.
//      So set_data_ready must write TIO=0x01, clear must write TIO=0x00.
//   3. FIRMWARE__SYSTEM_STATUS is above 0x00FF — we need REG_FILE_SIZE > 0x01FF.
//      Extended to 0x0200 to cover model-ID at 0x010F/0x0110.

#include "wokwi-api.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VL53L1X_I2C_ADDR 0x29

// ── Register addresses ────────────────────────────────────────────────────────
#define REG_GPIO_HV_MUX_CTRL   0x0030
#define REG_GPIO_TIO_HV_STATUS 0x0031
#define REG_INT_CLEAR          0x0086
#define REG_MODE_START         0x0087
#define REG_RESULT_BASE        0x0089  // first of 17-byte result burst
#define REG_FIRMWARE_STATUS    0x00E5
#define REG_MODEL_ID_HI        0x010F
#define REG_MODEL_ID_LO        0x0110

#define REG_FILE_SIZE 0x0200  // covers 0x0000–0x01FF

// ── I2C address-byte state machine ───────────────────────────────────────────
typedef enum {
  ADDR_NEED_HI = 0,  // next write byte = high byte of 16-bit register addr
  ADDR_NEED_LO,      // next write byte = low  byte of 16-bit register addr
  ADDR_DATA,         // address complete; subsequent bytes are register data
} addr_state_t;

typedef struct {
  pin_t    pin_sda;
  pin_t    pin_scl;
  uint32_t i2c_dev;
  uint32_t distance_attr;
  int      last_distance;

  addr_state_t addr_state;  // current state of the address decoder
  uint16_t     reg_addr;    // current 16-bit register address (auto-increments)

  uint8_t regs[REG_FILE_SIZE];  // flat register backing store

  bool ranging;      // true while SYSTEM__MODE_START = 0x40
  bool data_pending; // true after timer fires, cleared when driver reads + clears INT
  uint32_t timer;
} chip_state_t;

// ── Helpers ───────────────────────────────────────────────────────────────────

static inline uint8_t reg_read(chip_state_t *c, uint16_t addr) {
  return (addr < REG_FILE_SIZE) ? c->regs[addr] : 0x00;
}

static inline void reg_write(chip_state_t *c, uint16_t addr, uint8_t v) {
  if (addr < REG_FILE_SIZE) c->regs[addr] = v;
}

static int read_distance_mm(chip_state_t *c) {
  int dist = attr_read(c->distance_attr);
  if (dist < 0)    dist = 0;
  if (dist > 4000) dist = 4000;
  return dist;
}

// ── Result burst ──────────────────────────────────────────────────────────────
// 17-byte burst starting at REG_RESULT_BASE (0x0089).
// Driver byte-map from RangingResult::from_burst_bytes():
//   [0]       RANGE_STATUS  raw=0x09 → STATUS_RTN[9]=0 → Valid
//   [3]       DSS_SPADS     dummy
//   [7..8]    AMBIENT_KCPS  dummy
//   [13..14]  DISTANCE_MM   big-endian
//   [15..16]  SIGNAL_KCPS   dummy
static void publish_result(chip_state_t *c) {
  int dist = read_distance_mm(c);

  uint8_t *r = &c->regs[REG_RESULT_BASE];
  memset(r, 0, 17);
  r[0]  = 0x09;
  r[3]  = 0x10;
  r[13] = (uint8_t)((dist >> 8) & 0xFF);
  r[14] = (uint8_t)(dist & 0xFF);
  r[15] = 0x01;
}

// ── Data-ready signalling ─────────────────────────────────────────────────────
// We use ActiveLow polarity throughout (MUX_CTRL bit4=1 → ActiveLow).
// Driver checks: (TIO & 0x01) == pol_bit.
//   ActiveLow pol_bit = 0 → data ready when TIO bit0 = 0
//   ActiveLow pol_bit = 0 → data NOT ready when TIO bit0 = 1
//
// on_i2c_read hardcodes MUX_CTRL=0x10 and returns TIO based on data_pending.
// The backing store values below are only for consistency; reads bypass them.

static void set_data_ready(chip_state_t *c) {
  int dist = read_distance_mm(c);
  publish_result(c);
  c->regs[REG_GPIO_TIO_HV_STATUS] = 0x00;  // bit0=0 → ready (ActiveLow)
  c->data_pending = true;
  printf("[chip-vl53l1x] Data ready! Distance = %d mm (ranging=%s, pending=%s)\n",
         dist, c->ranging ? "yes" : "no", c->data_pending ? "yes" : "no");
}

static void clear_data_ready(chip_state_t *c) {
  c->regs[REG_GPIO_TIO_HV_STATUS] = 0x01;  // bit0=1 → not ready (ActiveLow)
  c->data_pending = false;
  printf("[chip-vl53l1x] Data-ready cleared (ranging=%s, pending=%s)\n",
         c->ranging ? "yes" : "no", c->data_pending ? "yes" : "no");
}

// ── Timer — fires every 100 ms ────────────────────────────────────────────────
static void on_timer_event(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  int dist = read_distance_mm(c);
  if (dist != c->last_distance) {
    c->last_distance = dist;
    printf("[chip-vl53l1x] Slider distance changed -> %d mm (ranging=%s, pending=%s)\n",
           dist, c->ranging ? "yes" : "no", c->data_pending ? "yes" : "no");
  }
  if (c->ranging && !c->data_pending) {
    set_data_ready(c);
  }
}

// ── I2C connect/disconnect ────────────────────────────────────────────────────
// Called with connect=true on START, connect=false on STOP.
//
// CRITICAL: write_read() (used for all register reads) does:
//   START → WRITE[addr_hi, addr_lo] → repeated-START → READ[bytes] → STOP
// Both the initial START and the repeated-START call connect(true).
// We must NOT reset reg_addr on the repeated-START or we'd lose the address
// that was just decoded in the write phase.
//
// Fix: reset the state machine ONLY on STOP (connect=false).
// On a fresh START the state is already ADDR_NEED_HI from the previous STOP,
// so the first write byte is correctly treated as the address high byte.
bool on_i2c_connect(void *user_data, uint32_t address, bool connect) {
  chip_state_t *c = (chip_state_t *)user_data;
  (void)address;
  if (!connect) {
    // STOP: arm state machine for the next transaction
    c->addr_state = ADDR_NEED_HI;
    c->reg_addr   = 0x0000;
  }
  // START (initial or repeated): do NOT touch addr_state or reg_addr
  return true;  // ACK the address
}

// ── I2C write ─────────────────────────────────────────────────────────────────
bool on_i2c_write(void *user_data, uint8_t data) {
  chip_state_t *c = (chip_state_t *)user_data;

  switch (c->addr_state) {
    case ADDR_NEED_HI:
      c->reg_addr   = (uint16_t)data << 8;
      c->addr_state = ADDR_NEED_LO;
      return true;

    case ADDR_NEED_LO:
      c->reg_addr  |= data;
      c->addr_state = ADDR_DATA;
      printf("[chip-vl53l1x] TX reg=0x%04X\n", c->reg_addr);
      return true;

    case ADDR_DATA: {
      uint16_t ra = c->reg_addr;

      // Side-effects on specific registers
      if (ra == REG_MODE_START) {
        if (data == 0x40 && !c->ranging) {
          c->ranging = true;
          clear_data_ready(c);
          printf("[chip-vl53l1x] Ranging STARTED at %d mm\n", read_distance_mm(c));
        } else if (data == 0x00 && c->ranging) {
          c->ranging = false;
          printf("[chip-vl53l1x] Ranging STOPPED\n");
        }
      } else if (ra == REG_INT_CLEAR && (data & 0x01)) {
        clear_data_ready(c);
        printf("[chip-vl53l1x] Interrupt cleared — rearmed for next sample\n");
      }

      reg_write(c, ra, data);
      c->reg_addr = ra + 1;  // auto-increment for burst writes
      return true;
    }
  }
  return true;
}

// ── I2C read ──────────────────────────────────────────────────────────────────
uint8_t on_i2c_read(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  uint16_t ra = c->reg_addr;
  c->reg_addr = ra + 1;  // auto-increment for burst reads

  // ── Hardcoded critical registers ─────────────────────────────────────────

  if (ra == REG_FIRMWARE_STATUS) {
    // FIRMWARE__SYSTEM_STATUS: always report booted
    printf("[chip-vl53l1x] Boot poll → 0x03\n");
    return 0x03;

  } else if (ra == REG_GPIO_HV_MUX_CTRL) {
    // GPIO_HV_MUX__CTRL: bit4=1 → ActiveLow interrupt polarity
    printf("[chip-vl53l1x] MUX_CTRL read → 0x10 (ActiveLow)\n");
    return 0x10;

  } else if (ra == REG_GPIO_TIO_HV_STATUS) {
    // TIO status: ActiveLow → bit0=0 means data ready, bit0=1 means not ready
    uint8_t tio = c->data_pending ? 0x00 : 0x01;
    static int tio_n = 0;
    if ((tio_n++ % 20) == 0)
      printf("[chip-vl53l1x] TIO poll → 0x%02X (%s)\n", tio,
             c->data_pending ? "READY" : "not-ready");
    return tio;

  } else if (ra >= REG_RESULT_BASE && ra < REG_RESULT_BASE + 17) {
    // 17-byte result burst starting at 0x0089.
    // from_burst_bytes() layout:
    //   [0]       RANGE_STATUS  — 0x09 → STATUS_RTN[9]=0 → Valid
    //   [3]       num_spads     — dummy
    //   [7..8]    ambient_kcps  — dummy
    //   [13..14]  distance_mm   — big-endian from slider
    //   [15..16]  signal_kcps   — dummy
    int dist = read_distance_mm(c);
    uint8_t result_buf[17] = {
      0x09,                          // [0]  range status → Valid
      0x00,                          // [1]
      0x00,                          // [2]
      0x10,                          // [3]  num_spads (dummy)
      0x00,                          // [4]
      0x00,                          // [5]
      0x00,                          // [6]
      0x00,                          // [7]  ambient hi
      0x00,                          // [8]  ambient lo
      0x00,                          // [9]
      0x00,                          // [10]
      0x00,                          // [11]
      0x00,                          // [12]
      (uint8_t)((dist >> 8) & 0xFF), // [13] distance hi
      (uint8_t)(dist & 0xFF),        // [14] distance lo
      0x00,                          // [15] signal hi
      0x00,                          // [16] signal lo
    };
    uint8_t byte_val = result_buf[ra - REG_RESULT_BASE];
    if (ra == REG_RESULT_BASE)
      printf("[chip-vl53l1x] Result burst read by driver — dist=%d mm (pending=%s)\n",
             dist, c->data_pending ? "yes" : "no");
    return byte_val;

  } else if (ra == REG_MODEL_ID_HI) {
    printf("[chip-vl53l1x] Model ID hi → 0x%02X\n", c->regs[ra]);
    return c->regs[ra];

  } else if (ra == REG_MODEL_ID_LO) {
    printf("[chip-vl53l1x] Model ID lo → 0x%02X\n", c->regs[ra]);
    return c->regs[ra];

  } else {
    return reg_read(c, ra);
  }
}

// ── chip_init ─────────────────────────────────────────────────────────────────
void chip_init(void) {
  chip_state_t *c = malloc(sizeof(chip_state_t));
  memset(c, 0, sizeof(chip_state_t));

  // ── Register defaults ──────────────────────────────────────────────────────

  // FIRMWARE__SYSTEM_STATUS = 0x03 → boot complete immediately
  c->regs[REG_FIRMWARE_STATUS] = 0x03;

  // IDENTIFICATION__MODEL_ID = 0xEEAC (two adjacent bytes, burst-read as u16)
  c->regs[REG_MODEL_ID_HI] = 0xEE;
  c->regs[REG_MODEL_ID_LO] = 0xAC;

  // GPIO_HV_MUX__CTRL = 0x11 (bit4=1 → ActiveLow before config patch)
  // The 91-byte config patch overwrites this with 0x01 (bit4=0 → ActiveHigh).
  // We pre-populate 0x11 so readbacks before the patch return a sane value.
  c->regs[REG_GPIO_HV_MUX_CTRL] = 0x11;

  // GPIO__TIO_HV_STATUS = 0x00 → not ready (ActiveHigh: bit0=0 = not ready)
  c->regs[REG_GPIO_TIO_HV_STATUS] = 0x00;

  // PHASECAL_CONFIG__TIMEOUT_MACROP (0x004B) = 0x0A → Long mode
  // Read by distance_mode() readback inside set_distance_mode()
  c->regs[0x004B] = 0x0A;

  // RANGE_CONFIG__TIMEOUT_MACROP_A_HI/LO (0x005E/5F) = 0x01CC → 100 ms
  // Read by timing_budget() inside set_distance_mode()
  c->regs[0x005E] = 0x01;
  c->regs[0x005F] = 0xCC;

  // RANGE_CONFIG__TIMEOUT_MACROP_B_HI (0x0061/62) = 0x01F1
  c->regs[0x0061] = 0x01;
  c->regs[0x0062] = 0xF1;

  // ROI_CONFIG__MODE_ROI_CENTRE_SPAD (0x013E) = 0xC7 (manufacturing optical centre)
  c->regs[0x013E] = 0xC7;

  // RESULT__OSC_CALIBRATE_VAL (0x00DE/DF) — non-zero so inter-measurement works
  c->regs[0x00DF] = 0x64;  // 100 decimal

  // ── Pins ──────────────────────────────────────────────────────────────────
  c->pin_sda = pin_init("SDA", INPUT_PULLUP);
  c->pin_scl = pin_init("SCL", INPUT_PULLUP);

  // ── UI attribute (distance slider 0–4000 mm, default 1000) ───────────────
  c->distance_attr = attr_init("distance", 1000);
  c->last_distance = read_distance_mm(c);

  // ── I2C ───────────────────────────────────────────────────────────────────
  const i2c_config_t i2c_cfg = {
    .user_data = c,
    .address   = VL53L1X_I2C_ADDR,
    .scl       = c->pin_scl,
    .sda       = c->pin_sda,
    .connect   = on_i2c_connect,
    .read      = on_i2c_read,
    .write     = on_i2c_write,
  };
  c->i2c_dev = i2c_init(&i2c_cfg);

  // ── 100 ms measurement timer ───────────────────────────────────────────────
  const timer_config_t timer_cfg = {
    .user_data = c,
    .callback  = on_timer_event,
  };
  c->timer = timer_init(&timer_cfg);
  timer_start(c->timer, 100000, true);  // 100 ms, repeating

  printf("[chip-vl53l1x] VL53L1X chip initialized. Distance Control via UI.\n");
  printf("[chip-vl53l1x] Initial slider distance = %d mm\n", c->last_distance);
}

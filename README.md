# VL53L1X — Async no_std Rust Driver

[![crate](https://img.shields.io/crates/v/vl53l1x.svg)](https://crates.io/crates/vl53l1x)
[![docs](https://docs.rs/vl53l1x/badge.svg)](https://docs.rs/vl53l1x)

Production-grade, `no_std`, async-first driver for the **STMicroelectronics VL53L1X** Time-of-Flight sensor, built for the [Embassy](https://embassy.dev) async embedded framework.

Developed by **Adom Industries** to factory-grade component standards.

---

## Features

| Feature | Status |
|---------|--------|
| `#![no_std]` — zero heap allocation | ✅ |
| Async-first via `embedded-hal-async` | ✅ |
| Strongly-typed `Error<E>` (no panics) | ✅ |
| ULD-accurate 91-byte init patch | ✅ |
| All 7 timing budgets (Short & Long mode) | ✅ |
| ROI configuration | ✅ |
| Distance threshold interrupt | ✅ |
| Offset & xtalk calibration (integer-only, no `f32`) | ✅ |
| Temperature recalibration | ✅ |
| Multi-sensor I2C address reassignment | ✅ |
| Gesture detection (Swipe L/R, Tap) via ROI switching | ✅ |
| `defmt` structured logging (feature-gated) | ✅ |
| Mock-HAL unit tests | ✅ |

---

## Quick Start

```rust
use vl53l1x::{Vl53l1x, DistanceMode, TimingBudget};
use embassy_time::Duration;

let mut sensor = Vl53l1x::new(i2c, Vl53l1x::DEFAULT_ADDR);
sensor.init().await?;
sensor.set_distance_mode(DistanceMode::Long).await?;
sensor.set_timing_budget(TimingBudget::Ms100).await?;
sensor.start_ranging().await?;

loop {
    sensor.wait_for_data_ready(Duration::from_millis(500)).await?;
    let r = sensor.get_result().await?;
    sensor.clear_interrupt().await?;

    if r.status.is_valid() {
        // r.distance_mm is valid
    }
}
```

## Gesture Detection

```rust
use vl53l1x::{GestureEngine, GestureEvent};

let mut engine = GestureEngine::new(1500); // 1.5 m threshold

loop {
    match engine.poll(&mut sensor, Duration::from_millis(200)).await? {
        GestureEvent::SwipeLeftToRight => { /* → */ }
        GestureEvent::SwipeRightToLeft => { /* ← */ }
        GestureEvent::TapLeft  => { /* tap */ }
        GestureEvent::TapRight => { /* tap */ }
        GestureEvent::None     => {}
    }
}
```

## Multi-sensor (N sensors on one bus)

```rust
// Bring up sensors one at a time using XSHUT pins.
// While only sensor 1 is powered:
sensor1.set_i2c_address(0x30).await?;
// Now power on sensor 2 (it will be at default 0x29)
sensor2.set_i2c_address(0x31).await?;
// Both now co-exist on the same bus with unique addresses
```

---

## Cargo Dependencies

```toml
[dependencies]
vl53l1x = "0.1"

# For defmt logging:
vl53l1x = { version = "0.1", features = ["defmt"] }
```

---

## Register Reference

All register constants are derived verbatim from **STSW-IMG009 v3.5.5** (`VL53L1X_api.h`).  
Range status codes follow **AN5573** Table 3.  
Timing budget register pairs follow **UM2356** § 3.  
Gesture zone SPAD centres follow the **Arduino VL53L1X library** ROI-switching pattern.

---

## Running Tests

```sh
cargo test --tests
```

Tests execute on the host using `embedded-hal-mock` — no hardware required.

---

## License

MIT OR Apache-2.0 — © 2026 Adom Industries

//! Embassy single-sensor basic ranging example
//!
//! Demonstrates initialising a VL53L1X over I2C on an STM32 target and
//! printing measurement results via defmt RTT at 10 Hz.
//!
//! ## Wiring (STM32F411 "BlackPill" example)
//! | STM32 Pin | VL53L1X Pin | Notes                   |
//! |-----------|-------------|-------------------------|
//! | PB7       | SDA         | 4.7 kΩ pull-up to 3.3 V |
//! | PB6       | SCL         | 4.7 kΩ pull-up to 3.3 V |
//! | 3.3 V     | VIN         |                          |
//! | GND       | GND         |                          |
//! | PA0       | XSHUT       | pull low to reset sensor |
//!
//! ## Flash
//! ```sh
//! cargo run --example embassy_basic --release
//! ```
//!
//! Requires a `.cargo/config.toml` pointing at your probe-rs / flip-link
//! linker configuration.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _; // RTT logging transport
use embassy_executor::Spawner;
use embassy_stm32::{
    i2c::{self, I2c},
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use panic_probe as _; // sets the panic handler
use vl53l1x::{
    config::{DistanceMode, TimingBudget},
    GestureEngine, GestureEvent, Vl53l1x,
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // ── Board initialisation ─────────────────────────────────────────────────
    let p = embassy_stm32::init(Default::default());

    // ── Configure I2C1 @ 400 kHz ─────────────────────────────────────────────
    let i2c = I2c::new(
        p.I2C1,
        p.PB6, // SCL
        p.PB7, // SDA
        p.DMA1_CH1,
        p.DMA1_CH0,
        Hertz(400_000),
        i2c::Config::default(),
    );

    // ── Construct driver ─────────────────────────────────────────────────────
    let mut sensor = Vl53l1x::new(i2c, Vl53l1x::DEFAULT_ADDR);

    // ── Verify silicon identity ──────────────────────────────────────────────
    match sensor.sensor_id().await {
        Ok(id) => info!("VL53L1X model ID: 0x{:04X}", id),
        Err(e) => {
            defmt::error!("sensor_id failed: {:?}", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    // ── Initialise sensor ────────────────────────────────────────────────────
    if let Err(e) = sensor.init().await {
        defmt::error!("init failed: {:?}", e);
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    // ── Configure for long-distance / 100 ms budget ──────────────────────────
    sensor
        .set_distance_mode(DistanceMode::Long)
        .await
        .unwrap_or_else(|e| defmt::error!("set_distance_mode: {:?}", e));

    sensor
        .set_timing_budget(TimingBudget::Ms100)
        .await
        .unwrap_or_else(|e| defmt::error!("set_timing_budget: {:?}", e));

    // Set inter-measurement period slightly above budget (110 ms)
    sensor
        .set_inter_measurement_ms(110)
        .await
        .unwrap_or_else(|e| defmt::error!("set_inter_measurement_ms: {:?}", e));

    // ── Start continuous ranging ─────────────────────────────────────────────
    sensor
        .start_ranging()
        .await
        .unwrap_or_else(|e| defmt::error!("start_ranging: {:?}", e));

    info!("VL53L1X ranging started — Long mode, 100 ms budget");

    // ── Measurement loop ─────────────────────────────────────────────────────
    loop {
        match sensor
            .wait_for_data_ready(Duration::from_millis(500))
            .await
        {
            Err(vl53l1x::Error::Timeout) => {
                defmt::warn!("data-ready timeout — sensor not responding");
                continue;
            }
            Err(e) => defmt::error!("wait_for_data_ready: {:?}", e),
            Ok(()) => {}
        }

        match sensor.get_result().await {
            Ok(r) => {
                if r.status.is_valid() {
                    info!(
                        "distance: {} mm  signal: {} kcps  ambient: {} kcps  spads: {}",
                        r.distance_mm, r.signal_kcps, r.ambient_kcps, r.num_spads
                    );
                } else {
                    info!("range status: {:?}", r.status);
                }
            }
            Err(e) => defmt::error!("get_result: {:?}", e),
        }

        if let Err(e) = sensor.clear_interrupt().await {
            defmt::error!("clear_interrupt: {:?}", e);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// BONUS: Gesture detection task
//
// Spawn this task instead of the loop above to enable gesture recognition.
// The sensor is reconfigured into ROI-switching mode automatically.
// ─────────────────────────────────────────────────────────────────────────────

/// A standalone Embassy task that drives the gesture engine.
///
/// Spawn this from `main` instead of running the basic ranging loop to
/// enable swipe / tap detection.
///
/// ```ignore
/// spawner.spawn(gesture_task(sensor)).ok();
/// ```
#[embassy_executor::task]
pub async fn gesture_task(
    mut sensor: Vl53l1x<embassy_stm32::i2c::I2c<'static, embassy_stm32::peripherals::I2C1>>,
) {
    use vl53l1x::config::TimingBudget;

    // Configure sensor for gesture sensing
    if let Err(e) = sensor.init().await {
        defmt::error!("gesture_task: init failed {:?}", e);
        return;
    }
    if let Err(e) = sensor
        .set_distance_mode(DistanceMode::Short)
        .await
    {
        defmt::error!("gesture_task: set_distance_mode {:?}", e);
        return;
    }
    if let Err(e) = sensor.set_timing_budget(TimingBudget::Ms33).await {
        defmt::error!("gesture_task: set_timing_budget {:?}", e);
        return;
    }

    let mut engine = GestureEngine::new(1_500); // threshold: 1.5 m

    loop {
        match engine
            .poll(&mut sensor, Duration::from_millis(200))
            .await
        {
            Ok(GestureEvent::SwipeLeftToRight) => info!("GESTURE: Swipe LEFT → RIGHT"),
            Ok(GestureEvent::SwipeRightToLeft) => info!("GESTURE: Swipe RIGHT → LEFT"),
            Ok(GestureEvent::TapLeft) => info!("GESTURE: Tap LEFT"),
            Ok(GestureEvent::TapRight) => info!("GESTURE: Tap RIGHT"),
            Ok(GestureEvent::None) => {}
            Err(e) => defmt::error!("gesture poll error: {:?}", e),
        }
    }
}

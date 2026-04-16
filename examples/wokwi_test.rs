#![no_std]
#![no_main]

use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{Config, InterruptHandler};
use embassy_rp::peripherals::I2C0;
use embassy_rp::uart::{Blocking, Config as UartConfig, Instance, Uart};
use embassy_time::{Duration, Timer};
use vl53l1x::{DistanceMode, RangeStatus, TimingBudget, Vl53l1x};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
});

fn uart_write_str<T: Instance>(uart: &mut Uart<'_, T, Blocking>, s: &str) {
    let _ = uart.blocking_write(s.as_bytes());
}

fn uart_write_line<T: Instance>(uart: &mut Uart<'_, T, Blocking>, s: &str) {
    uart_write_str(uart, s);
    uart_write_str(uart, "\r\n");
}

fn uart_write_u16<T: Instance>(uart: &mut Uart<'_, T, Blocking>, value: u16) {
    let mut buf = [0u8; 5];
    let mut value = value;
    let mut idx = buf.len();

    if value == 0 {
        uart_write_str(uart, "0");
        return;
    }

    while value > 0 {
        idx -= 1;
        buf[idx] = b'0' + (value % 10) as u8;
        value /= 10;
    }

    let _ = uart.blocking_write(&buf[idx..]);
}

fn status_str(status: RangeStatus) -> &'static str {
    match status {
        RangeStatus::Valid => "Valid",
        RangeStatus::SigmaFailed => "SigmaFailed",
        RangeStatus::SignalFailed => "SignalFailed",
        RangeStatus::TargetBelowMinRange => "TargetBelowMinRange",
        RangeStatus::InvalidPhase => "InvalidPhase",
        RangeStatus::HardwareFail => "HardwareFail",
        RangeStatus::NoWrapAroundCheck => "NoWrapAroundCheck",
        RangeStatus::WrapTargetFail => "WrapTargetFail",
        RangeStatus::ProcessingFail => "ProcessingFail",
        RangeStatus::XtalkSignalFail => "XtalkSignalFail",
        RangeStatus::SynchronisationInt => "SynchronisationInt",
        RangeStatus::MergedPulseDetected => "MergedPulseDetected",
        RangeStatus::TargetTooClose => "TargetTooClose",
        RangeStatus::NotUpdated => "NotUpdated",
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let mut uart = Uart::new_blocking(p.UART0, p.PIN_0, p.PIN_1, uart_cfg);

    uart_write_line(&mut uart, "VL53L1X Wokwi test starting");
    uart_write_line(&mut uart, "Open the Serial Monitor to view live readings");

    let i2c = embassy_rp::i2c::I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, Config::default());
    let mut sensor = Vl53l1x::new(i2c, 0x29);

    uart_write_line(&mut uart, "Initializing sensor...");
    match sensor.init().await {
        Ok(_) => uart_write_line(&mut uart, "Sensor initialized"),
        Err(_) => {
            uart_write_line(&mut uart, "Sensor init failed; halting");
            loop {
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }

    let _ = sensor.set_distance_mode(DistanceMode::Long).await;
    let _ = sensor.set_timing_budget(TimingBudget::Ms100).await;
    let _ = sensor.start_ranging().await;
    uart_write_line(&mut uart, "Ranging started");

    loop {
        match sensor.wait_for_data_ready(Duration::from_millis(200)).await {
            Ok(_) => {
                match sensor.get_result().await {
                    Ok(r) => {
                        uart_write_str(&mut uart, "Distance: ");
                        uart_write_u16(&mut uart, r.distance_mm);
                        uart_write_str(&mut uart, " mm, Status: ");
                        uart_write_line(&mut uart, status_str(r.status));
                    }
                    Err(_) => uart_write_line(&mut uart, "Failed to get result"),
                }
                let _ = sensor.clear_interrupt().await;
            }
            Err(_) => uart_write_line(&mut uart, "Data not ready yet"),
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}

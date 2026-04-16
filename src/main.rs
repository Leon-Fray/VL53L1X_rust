#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use rp_pico::entry;
use rp_pico::hal::{
    clocks::init_clocks_and_plls,
    fugit::RateExtU32,
    i2c::I2C,
    pac,
    sio::Sio,
    watchdog::Watchdog,
    Clock,
};

// Also ensuring we bring embedded_hal into scope for the trait
use embedded_hal::i2c::I2c;

/// Hypothetical VL53L1X Driver interface
pub struct Vl53l1x<I2C> {
    i2c: I2C,
}

impl<I2C_BUS> Vl53l1x<I2C_BUS>
where
    I2C_BUS: embedded_hal::i2c::I2c,
{
    pub fn new(i2c: I2C_BUS) -> Self {
        Self { i2c }
    }

    /// Read the distance placeholder register value
    pub fn read_distance(&mut self) -> Result<u8, I2C_BUS::Error> {
        let mut buffer = [0u8; 1];
        // Read from 0x0062 where our mock stores the lower 8 bits of distance
        self.i2c.write_read(0x29, &[0x00, 0x62], &mut buffer)?;
        Ok(buffer[0])
    }

    /// Read the sensor Model ID
    pub fn read_model_id(&mut self) -> Result<u8, I2C_BUS::Error> {
        let mut buffer = [0u8; 1];
        // Read 0x010F where our mock should return 0xEA
        self.i2c.write_read(0x29, &[0x01, 0x0F], &mut buffer)?;
        Ok(buffer[0])
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // Initialize clocks
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up I2C0 on GP4 (SDA) and GP5 (SCL)
    let i2c = I2C::i2c0(
        pac.I2C0,
        pins.gpio4.into_function::<rp_pico::hal::gpio::FunctionI2C>().into_pull_type::<rp_pico::hal::gpio::PullUp>(),
        pins.gpio5.into_function::<rp_pico::hal::gpio::FunctionI2C>().into_pull_type::<rp_pico::hal::gpio::PullUp>(),
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Set up delay provider
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    defmt::info!("Starting VL53L1X Simulator Test...");

    // Instantiate hypothetical driver
    let mut sensor = Vl53l1x::new(i2c);

    // Validate connection by reading model ID
    match sensor.read_model_id() {
        Ok(id) => defmt::info!("Model ID: {:#04X} (expected 0xEA)", id),
        Err(_) => defmt::error!("Failed to read Model ID"),
    }

    loop {
        match sensor.read_distance() {
            Ok(dist) => defmt::info!("Simulation Distance (Lower 8 Bits): {}", dist),
            Err(_) => defmt::error!("Failed to read distance"),
        }
        delay.delay_ms(500);
    }
}

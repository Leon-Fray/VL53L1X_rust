/// Low-level I2C transport helpers for the VL53L1X.
///
/// The VL53L1X uses **16-bit register addresses** sent in Big-Endian order,
/// then the payload.  All public types are behind `pub(crate)` — callers use
/// the `Vl53l1x` driver API, not these primitives directly.
///
/// # Wire format
/// ```text
/// [ ADDR_HI, ADDR_LO, DATA_0, DATA_1, ..., DATA_N ]
///   2 bytes   1-4 bytes depending on width
/// ```
/// This is encoded in every write below and confirmed by the Arduino
/// reference implementation (VL53L1X_I2CWrite in vl53l1x_class.cpp).

use embedded_hal_async::i2c::I2c;

// ─── Internal buffer size ────────────────────────────────────────────────────
// Each write transaction: 2-byte register address + 1 data byte.
const MAX_WRITE_BUF: usize = 3;

// ─── Write helpers ─────────────────────────────────────────────────────────────

/// Write a single byte to a 16-bit register address.
#[inline]
pub(crate) async fn wr_byte<I: I2c>(
    i2c: &mut I,
    addr: u8,
    reg: u16,
    val: u8,
) -> Result<(), I::Error> {
    let buf = [(reg >> 8) as u8, reg as u8, val];
    i2c.write(addr, &buf).await
}

/// Write a 16-bit word (Big-Endian) to a 16-bit register address.
#[inline]
pub(crate) async fn wr_word<I: I2c>(
    i2c: &mut I,
    addr: u8,
    reg: u16,
    val: u16,
) -> Result<(), I::Error> {
    let buf = [(reg >> 8) as u8, reg as u8, (val >> 8) as u8, val as u8];
    i2c.write(addr, &buf).await
}

/// Write a 32-bit dword (Big-Endian) to a 16-bit register address.
#[inline]
pub(crate) async fn wr_dword<I: I2c>(
    i2c: &mut I,
    addr: u8,
    reg: u16,
    val: u32,
) -> Result<(), I::Error> {
    let buf = [
        (reg >> 8) as u8,
        reg as u8,
        (val >> 24) as u8,
        (val >> 16) as u8,
        (val >> 8) as u8,
        val as u8,
    ];
    i2c.write(addr, &buf).await
}

/// Write an arbitrary byte slice starting at a 16-bit register address.
///
/// Each byte is sent as a separate 3-byte I2C transaction
/// `[addr_hi, addr_lo, value]` with the register address auto-incrementing.
/// The VL53L1X accepts individual register writes, so this is functionally
/// equivalent to a burst write and avoids large DMA transfers that may not
/// be supported by all I2C bus simulators.
pub(crate) async fn write_multi<I: I2c>(
    i2c: &mut I,
    addr: u8,
    reg: u16,
    data: &[u8],
) -> Result<(), I::Error> {
    let mut buf = [0u8; MAX_WRITE_BUF]; // [addr_hi, addr_lo, value]
    for (i, &byte) in data.iter().enumerate() {
        let r = reg.wrapping_add(i as u16);
        buf[0] = (r >> 8) as u8;
        buf[1] = r as u8;
        buf[2] = byte;
        i2c.write(addr, &buf).await?;
    }
    Ok(())
}

// ─── Read helpers ──────────────────────────────────────────────────────────────

/// Read a single byte from a 16-bit register address.
#[inline]
pub(crate) async fn rd_byte<I: I2c>(i2c: &mut I, addr: u8, reg: u16) -> Result<u8, I::Error> {
    let reg_buf = [(reg >> 8) as u8, reg as u8];
    let mut out = [0u8; 1];
    i2c.write_read(addr, &reg_buf, &mut out).await?;
    Ok(out[0])
}

/// Read a 16-bit word (Big-Endian) from a 16-bit register address.
#[inline]
pub(crate) async fn rd_word<I: I2c>(i2c: &mut I, addr: u8, reg: u16) -> Result<u16, I::Error> {
    let reg_buf = [(reg >> 8) as u8, reg as u8];
    let mut out = [0u8; 2];
    i2c.write_read(addr, &reg_buf, &mut out).await?;
    Ok(u16::from_be_bytes(out))
}

/// Read a 32-bit dword (Big-Endian) from a 16-bit register address.
#[inline]
pub(crate) async fn rd_dword<I: I2c>(i2c: &mut I, addr: u8, reg: u16) -> Result<u32, I::Error> {
    let reg_buf = [(reg >> 8) as u8, reg as u8];
    let mut out = [0u8; 4];
    i2c.write_read(addr, &reg_buf, &mut out).await?;
    Ok(u32::from_be_bytes(out))
}

/// Burst-read `buf.len()` bytes starting at a 16-bit register address.
pub(crate) async fn read_multi<I: I2c>(
    i2c: &mut I,
    addr: u8,
    reg: u16,
    buf: &mut [u8],
) -> Result<(), I::Error> {
    let reg_buf = [(reg >> 8) as u8, reg as u8];
    i2c.write_read(addr, &reg_buf, buf).await
}

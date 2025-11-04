#![allow(dead_code)]

//! ICM42688 Driver Implementation
//!
//! This module provides an async Rust driver for the ICM42688 IMU sensor,
//! using embedded-hal traits for portable hardware abstraction.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::registers::{self, REG_BANK_SEL};

/// Error type for ICM42688 operations
#[derive(Debug, Clone, PartialEq)]
pub enum Error<SpiError> {
    /// SPI communication error
    Spi(SpiError),
    /// Invalid register value
    InvalidRegister,
    /// WHO_AM_I check failed
    WrongDevice,
    /// Calibration failed
    CalibrationFailed,
    /// Bank switching failed
    BankSwitchFailed,
    /// Timeout error
    Timeout,
}

impl<SpiError> core::fmt::Display for Error<SpiError>
where
    SpiError: core::fmt::Display,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Spi(e) => write!(f, "SPI communication error: {}", e),
            Error::InvalidRegister => write!(f, "Invalid register value"),
            Error::WrongDevice => write!(f, "WHO_AM_I check failed"),
            Error::CalibrationFailed => write!(f, "Calibration failed"),
            Error::BankSwitchFailed => write!(f, "Bank switching failed"),
            Error::Timeout => write!(f, "Operation timeout"),
        }
    }
}

/// Gyroscope full scale range options
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum GyroFS {
    #[default]
    Dps2000 = 0x00,
    Dps1000 = 0x01,
    Dps500 = 0x02,
    Dps250 = 0x03,
    Dps125 = 0x04,
    Dps62_5 = 0x05,
    Dps31_25 = 0x06,
    Dps15_625 = 0x07,
}

/// Accelerometer full scale range options
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum AccelFS {
    #[default]
    Gpm16 = 0x00,
    Gpm8 = 0x01,
    Gpm4 = 0x02,
    Gpm2 = 0x03,
}

/// Output data rate options
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[repr(u8)]
pub enum ODR {
    Odr32k = 0x01, // LN mode only
    Odr16k = 0x02, // LN mode only
    Odr8k = 0x03,  // LN mode only
    Odr4k = 0x04,  // LN mode only
    Odr2k = 0x05,  // LN mode only
    #[default]
    Odr1k = 0x06, // LN mode only
    Odr200 = 0x07,
    Odr100 = 0x08,
    Odr50 = 0x09,
    Odr25 = 0x0A,
    Odr12_5 = 0x0B,
    Odr6a25 = 0x0C,   // LP mode only (accel only)
    Odr3a125 = 0x0D,  // LP mode only (accel only)
    Odr1a5625 = 0x0E, // LP mode only (accel only)
    Odr500 = 0x0F,
}

impl ODR {
    /// Convert ODR enum to frequency in Hz
    pub fn to_hz(self) -> f32 {
        match self {
            ODR::Odr32k => 32000.0,
            ODR::Odr16k => 16000.0,
            ODR::Odr8k => 8000.0,
            ODR::Odr4k => 4000.0,
            ODR::Odr2k => 2000.0,
            ODR::Odr1k => 1000.0,
            ODR::Odr200 => 200.0,
            ODR::Odr100 => 100.0,
            ODR::Odr50 => 50.0,
            ODR::Odr25 => 25.0,
            ODR::Odr12_5 => 12.5,
            ODR::Odr6a25 => 6.25,
            ODR::Odr3a125 => 3.125,
            ODR::Odr1a5625 => 1.5625,
            ODR::Odr500 => 500.0,
        }
    }
}

/// Gyro notch filter bandwidth selection
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum GyroNFBWsel {
    NfBW1449Hz = 0x00,
    NfBW680z = 0x01,
    NfBW329Hz = 0x02,
    NfBW162Hz = 0x03,
    NfBW80Hz = 0x04,
    NfBW40Hz = 0x05,
    NfBW20Hz = 0x06,
    NfBW10Hz = 0x07,
}

/// UI filter order
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum UIFiltOrd {
    FirstOrder = 0x00,
    SecondOrder = 0x01,
    ThirdOrder = 0x02,
}

/// Raw sensor data
#[derive(Debug, Clone, Default)]
pub struct RawData {
    pub temperature: i16,
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
}

/// Processed sensor data
/// - temperature: Temperature in Celsius
/// - accel: Acceleration in m/s^2
/// - gyro: Angular velocity in rad/s
#[derive(Debug, Clone, Default)]
pub struct SensorData {
    pub temperature: f32,
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
}

/// ICM42688 driver
///
/// # Type Parameters
/// - `SPI`: SPI device implementation
/// - `D`: Delay implementation
/// - `GYRO_ACCEL_FIFO_SIZE`: Size of the FIFO buffers for gyroscope and accelerometer data (default: 85)
/// - `TEMP_FIFO_SIZE`: Size of the FIFO buffer for temperature data (default: 256)
pub struct ICM42688<
    SPI,
    D,
    const GYRO_ACCEL_FIFO_SIZE: usize = 85,
    const TEMP_FIFO_SIZE: usize = 256,
> {
    spi: SPI,
    delay: D,
    bank: u8,

    // Scale factors
    accel_scale: f32,
    gyro_scale: f32,

    // Full scale selections
    accel_fs: AccelFS,
    gyro_fs: GyroFS,

    // Calibration data
    accel_bias: [f32; 3],
    gyro_bias: [f32; 3],
    accel_scale_factor: [f32; 3],

    // Raw offset data
    raw_accel_bias: [i32; 3],
    raw_gyro_bias: [i32; 3],
    accel_offset: [i16; 3],
    gyro_offset: [i16; 3],

    // FIFO configuration
    en_fifo_accel: bool,
    en_fifo_gyro: bool,
    en_fifo_temp: bool,
    en_fifo_timestamp: bool,
    en_fifo_header: bool,
    fifo_frame_size: usize,
    fifo_size: usize,

    // FIFO data buffers
    ax_fifo: [f32; GYRO_ACCEL_FIFO_SIZE],
    ay_fifo: [f32; GYRO_ACCEL_FIFO_SIZE],
    az_fifo: [f32; GYRO_ACCEL_FIFO_SIZE],
    gx_fifo: [f32; GYRO_ACCEL_FIFO_SIZE],
    gy_fifo: [f32; GYRO_ACCEL_FIFO_SIZE],
    gz_fifo: [f32; GYRO_ACCEL_FIFO_SIZE],
    t_fifo: [f32; TEMP_FIFO_SIZE],
    a_size: usize,
    g_size: usize,
    t_size: usize,
}

impl<SPI, D, const GYRO_ACCEL_FIFO_SIZE: usize, const TEMP_FIFO_SIZE: usize>
    ICM42688<SPI, D, GYRO_ACCEL_FIFO_SIZE, TEMP_FIFO_SIZE>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs,
{
    /// Create a new ICM42688 driver
    pub fn new(spi: SPI, delay: D) -> Self {
        Self {
            spi,
            delay,
            bank: 0,
            accel_scale: (16.0 / 32768.0) * 9.81, // Default 16g
            gyro_scale: (2000.0 / 32768.0f32).to_radians(), // Default 2000dps
            accel_fs: AccelFS::Gpm16,
            gyro_fs: GyroFS::Dps2000,
            accel_bias: [0.0; 3],
            gyro_bias: [0.0; 3],
            accel_scale_factor: [1.0; 3],
            raw_accel_bias: [0; 3],
            raw_gyro_bias: [0; 3],
            accel_offset: [0; 3],
            gyro_offset: [0; 3],

            // FIFO configuration
            en_fifo_accel: false,
            en_fifo_gyro: false,
            en_fifo_temp: false,
            en_fifo_timestamp: false,
            en_fifo_header: false,
            fifo_frame_size: 0,
            fifo_size: 0,

            // FIFO data buffers
            ax_fifo: [0.0; GYRO_ACCEL_FIFO_SIZE],
            ay_fifo: [0.0; GYRO_ACCEL_FIFO_SIZE],
            az_fifo: [0.0; GYRO_ACCEL_FIFO_SIZE],
            gx_fifo: [0.0; GYRO_ACCEL_FIFO_SIZE],
            gy_fifo: [0.0; GYRO_ACCEL_FIFO_SIZE],
            gz_fifo: [0.0; GYRO_ACCEL_FIFO_SIZE],
            t_fifo: [0.0; TEMP_FIFO_SIZE],
            a_size: 0,
            g_size: 0,
            t_size: 0,
        }
    }

    /// Initialize the ICM42688 sensor
    pub async fn begin(&mut self) -> Result<(), Error<SPI::Error>> {
        // Reset the sensor
        self.reset().await?;

        // Check WHO_AM_I register
        if self.who_am_i().await? != 0x47 {
            return Err(Error::WrongDevice);
        }

        // Turn on accel and gyro in Low Noise (LN) Mode
        self.write_register(registers::ub0::REG_PWR_MGMT0, 0x0F)
            .await?;

        // Set default full scale ranges
        self.set_accel_fs(AccelFS::Gpm16).await?;
        self.set_gyro_fs(GyroFS::Dps2000).await?;

        // Disable inner filters
        self.set_filters(false, false).await?;

        Ok(())
    }

    /// Set accelerometer full scale range
    pub async fn set_accel_fs(&mut self, fs: AccelFS) -> Result<(), Error<SPI::Error>> {
        self.set_bank(0).await?;

        // Read current register value
        let mut reg = self
            .read_register(registers::ub0::REG_ACCEL_CONFIG0)
            .await?;

        // Update FS_SEL bits (bits 7-5)
        reg = (fs as u8) << 5 | (reg & 0x1F);

        self.write_register(registers::ub0::REG_ACCEL_CONFIG0, reg)
            .await?;

        // Update scale factor
        self.accel_scale = match fs {
            AccelFS::Gpm2 => (2.0 / 32768.0) * 9.81,
            AccelFS::Gpm4 => (4.0 / 32768.0) * 9.81,
            AccelFS::Gpm8 => (8.0 / 32768.0) * 9.81,
            AccelFS::Gpm16 => (16.0 / 32768.0) * 9.81,
        };

        self.accel_fs = fs;
        Ok(())
    }

    /// Set gyroscope full scale range
    pub async fn set_gyro_fs(&mut self, fs: GyroFS) -> Result<(), Error<SPI::Error>> {
        self.set_bank(0).await?;

        // Read current register value
        let mut reg = self.read_register(registers::ub0::REG_GYRO_CONFIG0).await?;

        // Update FS_SEL bits (bits 7-5)
        reg = (fs as u8) << 5 | (reg & 0x1F);

        self.write_register(registers::ub0::REG_GYRO_CONFIG0, reg)
            .await?;

        // Update scale factor
        self.gyro_scale = match fs {
            GyroFS::Dps2000 => (2000.0 / 32768.0f32).to_radians(),
            GyroFS::Dps1000 => (1000.0 / 32768.0f32).to_radians(),
            GyroFS::Dps500 => (500.0 / 32768.0f32).to_radians(),
            GyroFS::Dps250 => (250.0 / 32768.0f32).to_radians(),
            GyroFS::Dps125 => (125.0 / 32768.0f32).to_radians(),
            GyroFS::Dps62_5 => (62.5 / 32768.0f32).to_radians(),
            GyroFS::Dps31_25 => (31.25 / 32768.0f32).to_radians(),
            GyroFS::Dps15_625 => (15.625 / 32768.0f32).to_radians(),
        };

        self.gyro_fs = fs;
        Ok(())
    }

    /// Set accelerometer output data rate
    pub async fn set_accel_odr(&mut self, odr: ODR) -> Result<(), Error<SPI::Error>> {
        self.set_bank(0).await?;

        // Read current register value
        let mut reg = self
            .read_register(registers::ub0::REG_ACCEL_CONFIG0)
            .await?;

        // Update ODR bits (bits 3-0)
        reg = (odr as u8) | (reg & 0xF0);

        self.write_register(registers::ub0::REG_ACCEL_CONFIG0, reg)
            .await?;
        Ok(())
    }

    /// Set gyroscope output data rate
    pub async fn set_gyro_odr(&mut self, odr: ODR) -> Result<(), Error<SPI::Error>> {
        self.set_bank(0).await?;

        // Read current register value
        let mut reg = self.read_register(registers::ub0::REG_GYRO_CONFIG0).await?;

        // Update ODR bits (bits 3-0)
        reg = (odr as u8) | (reg & 0xF0);

        self.write_register(registers::ub0::REG_GYRO_CONFIG0, reg)
            .await?;
        Ok(())
    }

    /// Set filters (gyro and accelerometer)
    pub async fn set_filters(
        &mut self,
        gyro_filters: bool,
        accel_filters: bool,
    ) -> Result<(), Error<SPI::Error>> {
        // Set gyro filters
        self.set_bank(1).await?;

        let gyro_config = if gyro_filters {
            0x00 // Enable filters
        } else {
            0x03 // Disable filters
        };

        self.write_register(registers::ub1::REG_GYRO_CONFIG_STATIC2, gyro_config)
            .await?;

        // Set accelerometer filters
        self.set_bank(2).await?;

        let accel_config = if accel_filters {
            0x00 // Enable filters
        } else {
            0x01 // Disable filters
        };

        self.write_register(registers::ub2::REG_ACCEL_CONFIG_STATIC2, accel_config)
            .await?;

        // Return to bank 0
        self.set_bank(0).await?;
        Ok(())
    }

    /// Read raw sensor data
    pub async fn get_raw_data(&mut self) -> Result<RawData, Error<SPI::Error>> {
        let mut buffer = [0u8; 14];

        // Read temperature and sensor data (14 bytes)
        self.read_registers(registers::ub0::REG_TEMP_DATA1, &mut buffer)
            .await?;

        // Combine bytes into 16-bit values
        let mut raw_data = RawData::default();

        raw_data.temperature = ((buffer[0] as i16) << 8) | buffer[1] as i16;
        raw_data.accel[0] = ((buffer[2] as i16) << 8) | buffer[3] as i16;
        raw_data.accel[1] = ((buffer[4] as i16) << 8) | buffer[5] as i16;
        raw_data.accel[2] = ((buffer[6] as i16) << 8) | buffer[7] as i16;
        raw_data.gyro[0] = ((buffer[8] as i16) << 8) | buffer[9] as i16;
        raw_data.gyro[1] = ((buffer[10] as i16) << 8) | buffer[11] as i16;
        raw_data.gyro[2] = ((buffer[12] as i16) << 8) | buffer[13] as i16;

        Ok(raw_data)
    }

    /// Read and process sensor data
    pub async fn get_sensor_data(&mut self) -> Result<SensorData, Error<SPI::Error>> {
        let raw_data = self.get_raw_data().await?;

        let mut sensor_data = SensorData::default();

        // Convert temperature
        sensor_data.temperature = (raw_data.temperature as f32 / 132.48) + 25.0;

        // Convert accelerometer data
        for i in 0..3 {
            sensor_data.accel[i] = ((raw_data.accel[i] as f32 * self.accel_scale)
                - self.accel_bias[i])
                * self.accel_scale_factor[i];
        }

        // Convert gyroscope data
        for i in 0..3 {
            sensor_data.gyro[i] = (raw_data.gyro[i] as f32 * self.gyro_scale) - self.gyro_bias[i];
        }

        Ok(sensor_data)
    }

    /// Set gyroscope bias (useful for custom calibration procedures)
    pub fn set_gyro_bias(&mut self, bias: [f32; 3]) {
        self.gyro_bias = bias;
    }

    /// Calibrate gyroscope
    pub async fn calibrate_gyro(&mut self) -> Result<(), Error<SPI::Error>> {
        // Take samples and find bias
        let mut bias_sum = [0.0f32; 3];
        const NUM_SAMPLES: usize = 1000;

        for _ in 0..NUM_SAMPLES {
            let data = self.get_sensor_data().await?;

            // If the gyro is moving, fail the calibration
            if data.gyro[0].abs() > 3f32.to_radians()
                || data.gyro[1].abs() > 3f32.to_radians()
                || data.gyro[2].abs() > 3f32.to_radians()
            {
                return Err(Error::CalibrationFailed);
            }

            bias_sum[0] += data.gyro[0];
            bias_sum[1] += data.gyro[1];
            bias_sum[2] += data.gyro[2];
            self.delay.delay_ms(1).await;
        }
        bias_sum[0] /= NUM_SAMPLES as f32;
        bias_sum[1] /= NUM_SAMPLES as f32;
        bias_sum[2] /= NUM_SAMPLES as f32;

        // Update bias values
        self.gyro_bias = bias_sum;

        Ok(())
    }

    /// Get accelerometer resolution
    /// Returns the resolution in m/s^2
    pub fn get_accel_resolution(&self) -> f32 {
        match self.accel_fs {
            AccelFS::Gpm2 => (2.0 / 32768.0) * 9.81,
            AccelFS::Gpm4 => (4.0 / 32768.0) * 9.81,
            AccelFS::Gpm8 => (8.0 / 32768.0) * 9.81,
            AccelFS::Gpm16 => (16.0 / 32768.0) * 9.81,
        }
    }

    /// Get gyroscope resolution
    /// Returns the resolution in rad/s
    pub fn get_gyro_resolution(&self) -> f32 {
        match self.gyro_fs {
            GyroFS::Dps2000 => (2000.0 / 32768.0f32).to_radians(),
            GyroFS::Dps1000 => (1000.0 / 32768.0f32).to_radians(),
            GyroFS::Dps500 => (500.0 / 32768.0f32).to_radians(),
            GyroFS::Dps250 => (250.0 / 32768.0f32).to_radians(),
            GyroFS::Dps125 => (125.0 / 32768.0f32).to_radians(),
            GyroFS::Dps62_5 => (62.5 / 32768.0f32).to_radians(),
            GyroFS::Dps31_25 => (31.25 / 32768.0f32).to_radians(),
            GyroFS::Dps15_625 => (15.625 / 32768.0f32).to_radians(),
        }
    }

    /// Get current accelerometer full scale setting
    pub fn get_accel_fs(&self) -> AccelFS {
        self.accel_fs
    }

    /// Get current gyroscope full scale setting
    pub fn get_gyro_fs(&self) -> GyroFS {
        self.gyro_fs
    }

    /// Get current bank
    pub fn get_bank(&self) -> u8 {
        self.bank
    }

    // Low-level register access methods

    /// Switch to a different register bank
    async fn set_bank(&mut self, bank: u8) -> Result<(), Error<SPI::Error>> {
        if self.bank == bank {
            return Ok(());
        }

        self.write_register(REG_BANK_SEL, bank).await?;
        self.bank = bank;
        Ok(())
    }

    /// Write a single register
    async fn write_register(&mut self, address: u8, data: u8) -> Result<(), Error<SPI::Error>> {
        let buffer = [address, data];
        self.spi.write(&buffer).await.map_err(Error::Spi)?;

        // Small delay for register write to complete
        self.delay.delay_ms(1).await;

        // Verify write by reading back
        let read_data = self.read_register(address).await?;
        if read_data != data {
            return Err(Error::InvalidRegister);
        }

        Ok(())
    }

    /// Read a single register
    async fn read_register(&mut self, address: u8) -> Result<u8, Error<SPI::Error>> {
        let mut buffer = [0u8; 1];
        self.read_registers(address, &mut buffer).await?;
        Ok(buffer[0])
    }

    /// Read multiple registers
    async fn read_registers(
        &mut self,
        address: u8,
        buffer: &mut [u8],
    ) -> Result<(), Error<SPI::Error>> {
        // Create tx buffer with read bit set
        let mut tx_buffer = [0u8; 32]; // Fixed size buffer for no-std
        let mut rx_buffer = [0u8; 32]; // Fixed size buffer for no-std

        let buffer_len = buffer.len();
        if buffer_len > 31 {
            return Err(Error::InvalidRegister); // Buffer too large
        }

        tx_buffer[0] = address | 0x80; // Set read bit

        // Transfer the data
        self.spi
            .transfer(
                &mut rx_buffer[..buffer_len + 1],
                &tx_buffer[..buffer_len + 1],
            )
            .await
            .map_err(Error::Spi)?;

        // Copy data (skip first byte which is the address)
        buffer.copy_from_slice(&rx_buffer[1..buffer_len + 1]);

        Ok(())
    }

    /// Reset the device
    async fn reset(&mut self) -> Result<(), Error<SPI::Error>> {
        self.set_bank(0).await?;
        if let Err(e) = self
            .write_register(registers::ub0::REG_DEVICE_CONFIG, 0x01)
            .await
        {
            match e {
                Error::InvalidRegister => {
                    // Ignore invalidRegister because it's expected as the reset is cleared
                }
                _ => {
                    return Err(e);
                }
            }
        }

        // Wait for device to come back up (longer delay for better reliability)
        self.delay.delay_ms(10).await;
        Ok(())
    }

    /// Read WHO_AM_I register
    async fn who_am_i(&mut self) -> Result<u8, Error<SPI::Error>> {
        self.set_bank(0).await?;
        self.read_register(registers::ub0::REG_WHO_AM_I).await
    }

    // FIFO methods

    /// Enable FIFO for specified sensors
    pub async fn enable_fifo(&mut self, accel: bool, gyro: bool) -> Result<(), Error<SPI::Error>> {
        self.en_fifo_accel = accel;
        self.en_fifo_gyro = gyro;
        self.en_fifo_temp = true; // All structures have 1-byte temp
        self.en_fifo_timestamp = accel && gyro; // Can only read both accel and gyro in Structure 3 or 4
        self.en_fifo_header = accel || gyro; // If neither sensor requested, FIFO will not send any more packets

        // Calculate frame size
        self.fifo_frame_size = (self.en_fifo_header as usize) * 1
            + (self.en_fifo_accel as usize) * 6
            + (self.en_fifo_gyro as usize) * 6
            + (self.en_fifo_temp as usize)
            + (self.en_fifo_timestamp as usize) * 2;

        // Set FIFO enable register
        let fifo_en = (self.en_fifo_accel as u8)
            | ((self.en_fifo_gyro as u8) << 1)
            | ((self.en_fifo_temp as u8) << 2);
        self.write_register(registers::ub0::REG_FIFO_CONFIG1, fifo_en)
            .await?;

        Ok(())
    }

    /// Start streaming to FIFO
    pub async fn stream_to_fifo(&mut self) -> Result<(), Error<SPI::Error>> {
        self.write_register(registers::ub0::REG_FIFO_CONFIG, 1 << 6)
            .await?;
        Ok(())
    }

    /// Read data from FIFO and store in buffers
    #[inline]
    pub async fn read_fifo(&mut self) -> Result<(), Error<SPI::Error>> {
        // Get FIFO size
        let mut buffer = [0u8; 2];
        self.read_registers(registers::ub0::REG_FIFO_COUNTH, &mut buffer)
            .await?;

        self.fifo_size = (((buffer[0] & 0x0F) as usize) << 8) + (buffer[1] as usize);

        if self.fifo_size == 0 || self.fifo_frame_size == 0 {
            return Ok(());
        }

        let num_frames = self.fifo_size / self.fifo_frame_size;
        let acc_index = 1;
        let gyro_index = acc_index + (self.en_fifo_accel as usize) * 6;
        let temp_index = gyro_index + (self.en_fifo_gyro as usize) * 6;

        // Read and parse the buffer
        for i in 0..num_frames {
            let mut frame_buffer = [0u8; 16]; // Maximum frame size
            self.read_registers(
                registers::ub0::REG_FIFO_DATA,
                &mut frame_buffer[..self.fifo_frame_size],
            )
            .await?;

            if self.en_fifo_accel {
                // Combine into 16-bit values
                let raw_meas = [
                    ((frame_buffer[0 + acc_index] as i16) << 8)
                        | frame_buffer[1 + acc_index] as i16,
                    ((frame_buffer[2 + acc_index] as i16) << 8)
                        | frame_buffer[3 + acc_index] as i16,
                    ((frame_buffer[4 + acc_index] as i16) << 8)
                        | frame_buffer[5 + acc_index] as i16,
                ];

                // Transform and convert to float values
                if i < GYRO_ACCEL_FIFO_SIZE {
                    self.ax_fifo[i] = ((raw_meas[0] as f32 * self.accel_scale)
                        - self.accel_bias[0])
                        * self.accel_scale_factor[0];
                    self.ay_fifo[i] = ((raw_meas[1] as f32 * self.accel_scale)
                        - self.accel_bias[1])
                        * self.accel_scale_factor[1];
                    self.az_fifo[i] = ((raw_meas[2] as f32 * self.accel_scale)
                        - self.accel_bias[2])
                        * self.accel_scale_factor[2];
                }
                self.a_size = num_frames;
            }

            if self.en_fifo_temp {
                let raw_meas = frame_buffer[temp_index] as i8;
                // Transform and convert to float values (temperature conversion)
                if i < TEMP_FIFO_SIZE {
                    self.t_fifo[i] = (raw_meas as f32 / 132.48) + 25.0;
                }
                self.t_size = num_frames;
            }

            if self.en_fifo_gyro {
                // Combine into 16-bit values
                let raw_meas = [
                    ((frame_buffer[0 + gyro_index] as i16) << 8)
                        | frame_buffer[1 + gyro_index] as i16,
                    ((frame_buffer[2 + gyro_index] as i16) << 8)
                        | frame_buffer[3 + gyro_index] as i16,
                    ((frame_buffer[4 + gyro_index] as i16) << 8)
                        | frame_buffer[5 + gyro_index] as i16,
                ];

                // Transform and convert to float values
                if i < GYRO_ACCEL_FIFO_SIZE {
                    self.gx_fifo[i] = (raw_meas[0] as f32 * self.gyro_scale) - self.gyro_bias[0];
                    self.gy_fifo[i] = (raw_meas[1] as f32 * self.gyro_scale) - self.gyro_bias[1];
                    self.gz_fifo[i] = (raw_meas[2] as f32 * self.gyro_scale) - self.gyro_bias[2];
                }
                self.g_size = num_frames;
            }
        }

        Ok(())
    }

    /// Get accelerometer FIFO data in X direction (m/s²)
    pub fn get_fifo_accel_x(&self) -> (&[f32], usize) {
        (
            &self.ax_fifo[..self.a_size.min(GYRO_ACCEL_FIFO_SIZE)],
            self.a_size,
        )
    }

    /// Get accelerometer FIFO data in Y direction (m/s²)
    pub fn get_fifo_accel_y(&self) -> (&[f32], usize) {
        (
            &self.ay_fifo[..self.a_size.min(GYRO_ACCEL_FIFO_SIZE)],
            self.a_size,
        )
    }

    /// Get accelerometer FIFO data in Z direction (m/s²)
    pub fn get_fifo_accel_z(&self) -> (&[f32], usize) {
        (
            &self.az_fifo[..self.a_size.min(GYRO_ACCEL_FIFO_SIZE)],
            self.a_size,
        )
    }

    /// Get gyroscope FIFO data in X direction (rad/s)
    #[inline(always)]
    pub fn get_fifo_gyro_x(&self) -> (&[f32], usize) {
        (
            &self.gx_fifo[..self.g_size.min(GYRO_ACCEL_FIFO_SIZE)],
            self.g_size,
        )
    }

    /// Get gyroscope FIFO data in Y direction (rad/s)
    #[inline(always)]
    pub fn get_fifo_gyro_y(&self) -> (&[f32], usize) {
        (
            &self.gy_fifo[..self.g_size.min(GYRO_ACCEL_FIFO_SIZE)],
            self.g_size,
        )
    }

    /// Get gyroscope FIFO data in Z direction (rad/s)
    #[inline(always)]
    pub fn get_fifo_gyro_z(&self) -> (&[f32], usize) {
        (
            &self.gz_fifo[..self.g_size.min(GYRO_ACCEL_FIFO_SIZE)],
            self.g_size,
        )
    }

    /// Get temperature FIFO data (°C)
    pub fn get_fifo_temperature(&self) -> (&[f32], usize) {
        (&self.t_fifo[..self.t_size.min(TEMP_FIFO_SIZE)], self.t_size)
    }

    /// Get FIFO size
    pub fn get_fifo_size(&self) -> usize {
        self.fifo_size
    }

    /// Get FIFO frame size
    pub fn get_fifo_frame_size(&self) -> usize {
        self.fifo_frame_size
    }
}

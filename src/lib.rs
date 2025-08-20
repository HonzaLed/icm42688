#![no_std]
#![deny(unsafe_code)]

//! # ICM42688 Driver
//!
//! A comprehensive Rust driver for the ICM42688 IMU sensor using embedded-hal traits.
//!
//! This driver provides:
//! - Async/await support with embedded-hal-async traits
//! - Complete sensor configuration (full scale ranges, output data rates, filters)
//! - Sensor data reading (raw and processed values)
//! - Gyroscope calibration with bias compensation
//! - Accelerometer calibration with bias and scale factor compensation
//! - Data ready interrupt configuration
//! - FIFO support for buffered data reading
//! - No-std compatibility
//!
//! ## Features
//!
//! - `serde`: Enable serde Serialize/Deserialize support for configuration enums
//!
//! ## Usage
//!
//! ```no_run
//! use icm42688::{ICM42688, AccelFS, GyroFS, ODR};
//!
//! // Create driver instance with your SPI device and delay
//! let mut imu = ICM42688::new(spi_device, delay);
//!
//! // Initialize the sensor
//! imu.begin().await?;
//!
//! // Configure sensor
//! imu.set_accel_fs(AccelFS::Gpm8).await?;
//! imu.set_gyro_fs(GyroFS::Dps1000).await?;
//!
//! // Read sensor data
//! let data = imu.get_sensor_data().await?;
//! # Ok::<(), icm42688::Error>(())
//! ```

pub mod registers;

pub use registers::*;

mod driver;
pub use driver::*;

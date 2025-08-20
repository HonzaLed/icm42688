#![allow(dead_code)]

//! ICM42688 Register Definitions
//!
//! This module contains all register addresses for the ICM42688 IMU unit,
//! organized by user banks as defined in the datasheet.

/// Accessible from all user banks
pub const REG_BANK_SEL: u8 = 0x76;

/// User Bank 0 Registers
pub mod ub0 {
    pub const REG_DEVICE_CONFIG: u8 = 0x11;
    pub const REG_DRIVE_CONFIG: u8 = 0x13;
    pub const REG_INT_CONFIG: u8 = 0x14;
    pub const REG_FIFO_CONFIG: u8 = 0x16;

    // Data registers
    pub const REG_TEMP_DATA1: u8 = 0x1D;
    pub const REG_TEMP_DATA0: u8 = 0x1E;
    pub const REG_ACCEL_DATA_X1: u8 = 0x1F;
    pub const REG_ACCEL_DATA_X0: u8 = 0x20;
    pub const REG_ACCEL_DATA_Y1: u8 = 0x21;
    pub const REG_ACCEL_DATA_Y0: u8 = 0x22;
    pub const REG_ACCEL_DATA_Z1: u8 = 0x23;
    pub const REG_ACCEL_DATA_Z0: u8 = 0x24;
    pub const REG_GYRO_DATA_X1: u8 = 0x25;
    pub const REG_GYRO_DATA_X0: u8 = 0x26;
    pub const REG_GYRO_DATA_Y1: u8 = 0x27;
    pub const REG_GYRO_DATA_Y0: u8 = 0x28;
    pub const REG_GYRO_DATA_Z1: u8 = 0x29;
    pub const REG_GYRO_DATA_Z0: u8 = 0x2A;
    pub const REG_TMST_FSYNCH: u8 = 0x2B;
    pub const REG_TMST_FSYNCL: u8 = 0x2C;
    pub const REG_INT_STATUS: u8 = 0x2D;
    pub const REG_FIFO_COUNTH: u8 = 0x2E;
    pub const REG_FIFO_COUNTL: u8 = 0x2F;
    pub const REG_FIFO_DATA: u8 = 0x30;
    pub const REG_APEX_DATA0: u8 = 0x31;
    pub const REG_APEX_DATA1: u8 = 0x32;
    pub const REG_APEX_DATA2: u8 = 0x33;
    pub const REG_APEX_DATA3: u8 = 0x34;
    pub const REG_APEX_DATA4: u8 = 0x35;
    pub const REG_APEX_DATA5: u8 = 0x36;
    pub const REG_INT_STATUS2: u8 = 0x37;
    pub const REG_INT_STATUS3: u8 = 0x38;

    // Configuration registers
    pub const REG_SIGNAL_PATH_RESET: u8 = 0x4B;
    pub const REG_INTF_CONFIG0: u8 = 0x4C;
    pub const REG_INTF_CONFIG1: u8 = 0x4D;
    pub const REG_PWR_MGMT0: u8 = 0x4E;
    pub const REG_GYRO_CONFIG0: u8 = 0x4F;
    pub const REG_ACCEL_CONFIG0: u8 = 0x50;
    pub const REG_GYRO_CONFIG1: u8 = 0x51;
    pub const REG_GYRO_ACCEL_CONFIG0: u8 = 0x52;
    pub const REG_ACCEFL_CONFIG1: u8 = 0x53;
    pub const REG_TMST_CONFIG: u8 = 0x54;
    pub const REG_APEX_CONFIG0: u8 = 0x56;
    pub const REG_SMD_CONFIG: u8 = 0x57;
    pub const REG_FIFO_CONFIG1: u8 = 0x5F;
    pub const REG_FIFO_CONFIG2: u8 = 0x60;
    pub const REG_FIFO_CONFIG3: u8 = 0x61;
    pub const REG_FSYNC_CONFIG: u8 = 0x62;
    pub const REG_INT_CONFIG0: u8 = 0x63;
    pub const REG_INT_CONFIG1: u8 = 0x64;
    pub const REG_INT_SOURCE0: u8 = 0x65;
    pub const REG_INT_SOURCE1: u8 = 0x66;
    pub const REG_INT_SOURCE3: u8 = 0x68;
    pub const REG_INT_SOURCE4: u8 = 0x69;
    pub const REG_INT_SOURCE6: u8 = 0x6A;
    pub const REG_INT_SOURCE7: u8 = 0x6B;
    pub const REG_INT_SOURCE8: u8 = 0x6C;
    pub const REG_INT_SOURCE9: u8 = 0x6D;
    pub const REG_INT_SOURCE10: u8 = 0x6E;
    pub const REG_FIFO_LOST_PKT0: u8 = 0x6C;
    pub const REG_FIFO_LOST_PKT1: u8 = 0x6D;
    pub const REG_SELF_TEST_CONFIG: u8 = 0x70;
    pub const REG_WHO_AM_I: u8 = 0x75;
    pub const REG_REG_BANK_SEL: u8 = 0x76;
}

/// User Bank 1 Registers
pub mod ub1 {
    pub const REG_SENSOR_CONFIG0: u8 = 0x03;

    // Gyro configuration static registers
    pub const REG_GYRO_CONFIG_STATIC2: u8 = 0x0B;
    pub const REG_GYRO_CONFIG_STATIC3: u8 = 0x0C;
    pub const REG_GYRO_CONFIG_STATIC4: u8 = 0x0D;
    pub const REG_GYRO_CONFIG_STATIC5: u8 = 0x0E;
    pub const REG_GYRO_CONFIG_STATIC6: u8 = 0x0F;
    pub const REG_GYRO_CONFIG_STATIC7: u8 = 0x10;
    pub const REG_GYRO_CONFIG_STATIC8: u8 = 0x11;
    pub const REG_GYRO_CONFIG_STATIC9: u8 = 0x12;
    pub const REG_GYRO_CONFIG_STATIC10: u8 = 0x13;

    // Self-test and timestamp data
    pub const REG_XG_ST_DATA: u8 = 0x5F;
    pub const REG_YG_ST_DATA: u8 = 0x60;
    pub const REG_ZG_ST_DATA: u8 = 0x61;
    pub const REG_TMSTVAL0: u8 = 0x62;
    pub const REG_TMSTVAL1: u8 = 0x63;
    pub const REG_TMSTVAL2: u8 = 0x64;

    // Interface configuration
    pub const REG_INTF_CONFIG4: u8 = 0x7A;
    pub const REG_INTF_CONFIG5: u8 = 0x7B;
    pub const REG_INTF_CONFIG6: u8 = 0x7C;
}

/// User Bank 2 Registers
pub mod ub2 {
    // Accelerometer configuration static registers
    pub const REG_ACCEL_CONFIG_STATIC2: u8 = 0x03;
    pub const REG_ACCEL_CONFIG_STATIC3: u8 = 0x04;
    pub const REG_ACCEL_CONFIG_STATIC4: u8 = 0x05;

    // Accelerometer self-test data
    pub const REG_XA_ST_DATA: u8 = 0x3B;
    pub const REG_YA_ST_DATA: u8 = 0x3C;
    pub const REG_ZA_ST_DATA: u8 = 0x3D;
}

/// User Bank 4 Registers
pub mod ub4 {
    // APEX configuration registers
    pub const REG_APEX_CONFIG1: u8 = 0x40;
    pub const REG_APEX_CONFIG2: u8 = 0x41;
    pub const REG_APEX_CONFIG3: u8 = 0x42;
    pub const REG_APEX_CONFIG4: u8 = 0x43;
    pub const REG_APEX_CONFIG5: u8 = 0x44;
    pub const REG_APEX_CONFIG6: u8 = 0x45;
    pub const REG_APEX_CONFIG7: u8 = 0x46;
    pub const REG_APEX_CONFIG8: u8 = 0x47;
    pub const REG_APEX_CONFIG9: u8 = 0x48;

    // Wake-on-motion thresholds
    pub const REG_ACCEL_WOM_X_THR: u8 = 0x4A;
    pub const REG_ACCEL_WOM_Y_THR: u8 = 0x4B;
    pub const REG_ACCEL_WOM_Z_THR: u8 = 0x4C;

    // Interrupt source registers
    pub const REG_INT_SOURCE6: u8 = 0x4D;
    pub const REG_INT_SOURCE7: u8 = 0x4E;
    pub const REG_INT_SOURCE8: u8 = 0x4F;
    pub const REG_INT_SOURCE9: u8 = 0x50;
    pub const REG_INT_SOURCE10: u8 = 0x51;

    // Offset user registers
    pub const REG_OFFSET_USER0: u8 = 0x77;
    pub const REG_OFFSET_USER1: u8 = 0x78;
    pub const REG_OFFSET_USER2: u8 = 0x79;
    pub const REG_OFFSET_USER3: u8 = 0x7A;
    pub const REG_OFFSET_USER4: u8 = 0x7B;
    pub const REG_OFFSET_USER5: u8 = 0x7C;
    pub const REG_OFFSET_USER6: u8 = 0x7D;
    pub const REG_OFFSET_USER7: u8 = 0x7E;
    pub const REG_OFFSET_USER8: u8 = 0x7F;
}

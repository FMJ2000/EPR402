#ifndef _IMU_H
#define _IMU_H

#include "master.h"
#include "bot.h"

//MPU9250
#define MPU9250_AD 0x68
#define FIFO_EN_AD 0x23
#define PWR_MGMT_1_AD 0x6B
#define ACCEL_XOUT_H_AD 0x3B
#define GYRO_XOUT_H_AD 0x43
#define GYRO_XOUT_L_AD 0x44
#define GYRO_YOUT_H_AD 0x45
#define GYRO_YOUT_L_AD 0x46
#define GYRO_ZOUT_H_AD 0x47
#define GYRO_ZOUT_L_AD 0x48
#define EXT_SENS_DATA_00_AD 0x49
#define ACCEL_CONFIG_1_AD 0x1C
#define ACCEL_CONFIG_2_AD 0x1D
#define GYRO_CONFIG_AD 0x1B
#define CONFIG_AD 0x1A
#define I2C_MST_CTRL_AD 0x24
#define I2C_SLV0_ADDR_AD 0x25
#define I2C_SLV0_REG_AD 0x26
#define I2C_SLV0_CTRL_AD 0x27
#define INT_PIN_CFG_AD 0x37
#define INT_ENABLE 0x38
#define USER_CTRL_AD 0x6A
#define WHO_AM_I_AD 0x75    // should return 0x71 (113)
#define ACCEL_SENS 4096.0f
#define GYRO_SENS 65.536f

//Magnetometer
#define MAG_AD 0x0C
#define WIA_AD 0x00     // should return 0x48 (72))
#define INFO 0x01
#define STATUS_1_AD 0x02
#define HXL_AD 0x03    //X-axis measurement data lower 8bit
#define HXH_AD 0x04    //X-axis measurement data higher 8bit
#define HYL_AD 0x05    //Y-axis measurement data lower 8bit
#define HYH_AD 0x06    //Y-axis measurement data higher 8bit
#define HZL_AD 0x07    //Z-axis measurement data lower 8bit
#define HZH_AD 0x08    //Z-axis measurement data higher 8bit
#define STATUS_2_AD 0x09
#define CNTL1_AD 0x0A   //control 1
#define CNTL2_AD 0x0B   //control 2
#define ASTC_AD 0x0C    //Self-Test Control
#define TS1_AD 0x0D    //test 1
#define TS2_AD 0x0E   //test 2
#define I2CDIS_AD 0x0F    //I2C disable
#define ASAX_AD 0x10    //Magnetic sensor X-axis sensitivity adjustment value
#define ASAY_AD 0x11    //Magnetic sensor Y-axis sensitivity adjustment value
#define ASAZ_AD 0x12    //Magnetic sensor Z-axis sensitivity adjustment value
#define MAGNE_SENS 6.67f
#define SCALE 0.1499f  // 4912/32760 uT/tick
#define DATA_READY 0x01
#define MAGIC_OVERFLOW 0x8
#define TEMP_SCALE 16.0
#define G_CONSTANT 9.80665
#define TEMP_OFFSET 25
#define IMU_RES 32768.0

void IMU_Init(uint8_t whoami[2], float asa[2]);
void IMU_Read(float imu[4], float asa[2]);

#endif
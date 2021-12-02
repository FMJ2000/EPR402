#include "imu.h"

void IMU_Init(uint8_t whoami[2], float asa[2]) {
    // gyroscope & accelerometer
    I2C_Write(MPU9250_AD, PWR_MGMT_1_AD, 0x01);
    I2C_Write(MPU9250_AD, GYRO_CONFIG_AD, 0x08);
    I2C_Write(MPU9250_AD, ACCEL_CONFIG_1_AD, 0x08);
    I2C_Write(MPU9250_AD, ACCEL_CONFIG_2_AD, 0x05);
    I2C_Write(MPU9250_AD, CONFIG_AD, 0x05);
    
    // magnetometer
		I2C_Write(MPU9250_AD, USER_CTRL_AD, 0x00);
    I2C_Write(MPU9250_AD, INT_PIN_CFG_AD, 0x02);
    I2C_Write(MPU9250_AD, INT_ENABLE, 0x01);
		I2C_Write(MAG_AD, CNTL1_AD, 0x1F);
    delay(50000l);
    uint8_t asaInt[2];
    I2C_Read(MAG_AD, ASAX_AD, asaInt, 2);
    for (uint8_t i = 0; i < 2; i++) asa[i] = ((float)asaInt[i] - 128) / 256 + 1;
    I2C_Write(MAG_AD, CNTL1_AD, 0x0);
    delay(50000l);
    I2C_Write(MAG_AD, CNTL1_AD, 0x16);
    delay(50000l);

		// test everything is working
		uint8_t status;
    I2C_Read(MPU9250_AD, WHO_AM_I_AD, whoami, 1);
		I2C_Read(MAG_AD, STATUS_1_AD, &status, 1);
    if ((status & DATA_READY) == DATA_READY) {
			I2C_Read(MAG_AD, WIA_AD, &whoami[1], 1);
		}
}

void IMU_Read(float imu[4], float asa[2]) {
    // Get IMU sensor data
    uint8_t data[13];		// 6 sensor readings + status
    int16_t values[5];
    I2C_Read(MPU9250_AD, ACCEL_XOUT_H_AD, data, 4);
    I2C_Read(MPU9250_AD, GYRO_ZOUT_H_AD, &data[4], 2);
		I2C_Read(MAG_AD, STATUS_1_AD, &data[12], 1);
    if ((data[12] & DATA_READY) == DATA_READY) {
			I2C_Read(MAG_AD, HXL_AD, &data[6], 7);
		}
    
    for (uint8_t i = 0; i < 3; i++) values[i] = (data[2*i] << 8) | data[2*i+1];
		for (uint8_t i = 3; i < 5; i++) values[i] = (data[2*i+1] << 8) | data[2*i];
    imu[0] = (float)values[0] / ACCEL_SENS;
    imu[1] = (float)values[1] / ACCEL_SENS;
    imu[2] = (float)values[2] * M_PI / (GYRO_SENS * 180.0);
		if (!(data[12] & MAGIC_OVERFLOW)) {
			imu[4] = atan2(-(float)values[4] * asa[1] * SCALE, (float)values[3] * asa[0] * SCALE);
		}
}
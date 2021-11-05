#include "imu.h"

void IMU_Init(float asa[3], float * yaw, uint8_t whoami[2]) {
    //snprintf(bot.buf, 100, "init: %d (%d)\r\n", I2C2STAT, PORTB);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    I2C_Write(MPU9250_AD, PWR_MGMT_1_AD, 0x01);
    I2C_Write(MPU9250_AD, GYRO_CONFIG_AD, 0x08);
    I2C_Write(MPU9250_AD, ACCEL_CONFIG_1_AD, 0x08);
    I2C_Write(MPU9250_AD, ACCEL_CONFIG_2_AD, 0x05);
    I2C_Write(MPU9250_AD, CONFIG_AD, 0x05);
    I2C_Write(MPU9250_AD, USER_CTRL_AD, 0x00);
    I2C_Write(MPU9250_AD, INT_PIN_CFG_AD, 0x02);
    I2C_Write(MPU9250_AD, INT_ENABLE, 0x01);
    
    I2C_Write(MAG_AD, CNTL1_AD, 0x1F);
    delay(100000);
    uint8_t asaInt[3];
    I2C_Read(MAG_AD, ASAX_AD, asaInt, 3);
    for (uint8_t i = 0; i < 3; i++) asa[i] = ((float)asaInt[i] - 128) / 256 + 1;
    I2C_Write(MAG_AD, CNTL1_AD, 0x0);
    delay(100000);
    I2C_Write(MAG_AD, CNTL1_AD, 0x16);
    delay(100000);
    
    // test everything is working
    I2C_Read(MPU9250_AD, WHO_AM_I_AD, whoami, 1);
    I2C_Read(MAG_AD, WIA_AD, &whoami[1], 1);
    
    // set initial yaw
    uint8_t status;
    uint8_t data[2];
    I2C_Read(MAG_AD, STATUS_1_AD, &status, 1);
    if ((status & DATA_READY) == DATA_READY) {
	I2C_Read(MAG_AD, HZL_AD, data, 2);
	I2C_Read(MAG_AD, STATUS_2_AD, &status, 1);
    }
    int16_t val = (data[1] << 8) | data[0];
    //*yaw = (float)val * asa[2] * SCALE;
}

void IMU_Read(float gyro[3], float acc[3], float mag[3], float asa[3]) {
    // Get IMU sensor data
    uint8_t data[18];		// 9 sensor readings
    uint8_t status;	// 2 magnetometer status bits
    int16_t values[9];
    I2C_Read(MPU9250_AD, GYRO_XOUT_H_AD, data, 6);
    I2C_Read(MPU9250_AD, ACCEL_XOUT_H_AD, &data[6], 6);
    I2C_Read(MAG_AD, STATUS_1_AD, &status, 1);
    if ((status & DATA_READY) == DATA_READY) {
	I2C_Read(MAG_AD, HXL_AD, &data[12], 6);
	I2C_Read(MAG_AD, STATUS_2_AD, &status, 1);
    }
    
    for (uint8_t i = 0; i < 6; i++) values[i] = (data[2*i] << 8) | data[2*i+1];
    for (uint8_t i = 0; i < 3; i++) values[6+i] = (data[7+2*i] << 8) | data[6+2*i];
    for (uint8_t i = 0; i < 3; i++) {
	gyro[i] = (float)values[i] * M_PI / (GYRO_SENS * 180.0);
	acc[i] = (float)values[3+i] / ACCEL_SENS;
	mag[i] = (float)values[6+i] * asa[i] * SCALE;
    }
}
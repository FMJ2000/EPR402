#include "imu.h"

void IMU_Init(uint8_t whoami[2]) {
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
    
    // test everything is working
    I2C_Read(MPU9250_AD, WHO_AM_I_AD, whoami, 1);
}

void IMU_Read(float imu[3]) {
    // Get IMU sensor data
    uint8_t data[6];		// 9 sensor readings
    int16_t values[3];
    I2C_Read(MPU9250_AD, ACCEL_XOUT_H_AD, data, 4);
    I2C_Read(MPU9250_AD, GYRO_XOUT_H_AD, &data[4], 2);
    
    for (uint8_t i = 0; i < 3; i++) values[i] = (data[2*i] << 8) | data[2*i+1];
    imu[0] = (float)values[0] / ACCEL_SENS;
    imu[1] = (float)values[1] / ACCEL_SENS;
    imu[2] = (float)values[2] * M_PI / (GYRO_SENS * 180.0);
}
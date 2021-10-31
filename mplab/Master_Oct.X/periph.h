#ifndef _PERIPH_H
#define _PERIPH_H

#include "master.h"
#include "bot.h"

#define I2C_BAUD 400000l
#define UART_BAUD 9600l
#define UART_BRGH (int)(PBCLK / (4 * UART_BAUD) - 1)
#define I2C_W 0x0
#define I2C_R 0x1

/* peripheral defintions */
void Init();
void UART_Write(char data);
void I2C_Master_Init();
void I2C_Master_Start();
void I2C_Master_Stop();
void I2C_Master_Restart();
void I2C_Master_Idle();
void I2C_Master_Ack_Nack(int val);
int I2C_Master_Write(char byte);
char I2C_Master_Read();
int I2C_Write(char periphAdd, char regAdd, char data);
int I2C_Read(char periphAdd, char regAdd, char * data, int len);
void Ultrasonic_Trigger();
void Odometer_Read(uint8_t odo[2]);

#endif
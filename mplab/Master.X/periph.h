#ifndef _PERIPH_H
#define _PERIPH_H

#include "master.h"
#include "bot.h"

#define I2C_BAUD 400000l
#define UART_BAUD 38400l
#define I2C_W 0x0
#define I2C_R 0x1

/* peripheral defintions */
void Init(struct Bot * bot);
void UART_Write(char data);
void UART_Write_String(char * data, int len);
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
void Trigger_Ultrasonic(struct Bot * bot);

#endif
/* 
 * File:   slave.h
 * Author: martin
 *
 * Created on 06 October 2021, 8:06 AM
 */

#ifndef SLAVE_H
#define	SLAVE_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <xc.h>
#include <proc/p32mx220f032b.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
    
#define SYSCLK 64000000l
#define PBCLK 32000000l
#define I2C_BAUD 100000l
#define UART_BAUD 38400l
#define US_TRIG_T 640           // ultrasonic trigger
#define I2C_W 0x0
#define I2C_R 0x1
#define SLAVE_ADD 0x40		// I2C slave address
#define MSG_LEN 12              // length of I2C message (ultrasonic: 8, odometer: 2, adc: 1, user: 1)
#define F_SAMPLE 61		// sensor sample rate
#define US_BUF_LEN 1
#define US_SENSORS 4

    typedef struct Slave {
        unsigned short i2cIndex;           // current byte read out to I2C master
        unsigned short usState;            // ultrasonic sensor
        unsigned short usBufIndex;          // ultrasonic buffer index
        unsigned short adcCounter;         // counter until next ADC reading
        unsigned char usData[US_BUF_LEN][US_SENSORS*2]; // take mean of last 10 ultrasonic measurements for greater stability
        unsigned char data[MSG_LEN];  // ultrasonic[0:8], odometer[8:10], battery[10], user[11] readings
    } Slave;

    void Read_Ultrasonic(Slave * slave);
    void US_Min(Slave * slave);

#ifdef	__cplusplus
}
#endif

#endif	/* SLAVE_H */


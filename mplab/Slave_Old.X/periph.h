/* 
 * File:   periph.h
 * Author: martin
 *
 * Created on 06 October 2021, 9:17 AM
 */

#ifndef PERIPH_H
#define	PERIPH_H

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

    void PORT_Init();
    void INT_Init();
    void TMR_Init();
    void ADC_Init();
    void I2C_Init();

#ifdef	__cplusplus
}
#endif

#endif	/* PERIPH_H */


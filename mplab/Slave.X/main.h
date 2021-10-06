/* 
 * File:   main.h
 * Author: martin
 *
 * Created on 06 October 2021, 7:52 AM
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include <proc/p32mx220f032b.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
    
#include "periph.h"
#include "slave.h"

#define SYSCLK 64000000l
#define PBCLK 32000000l
#define I2C_BAUD 100000l
#define UART_BAUD 38400l
#define US_TRIG_T 640           // ultrasonic trigger
#define I2C_W 0x0
#define I2C_R 0x1
#define SLAVE_ADD 0x40		// I2C slave address
#define MSG_LEN 12              // length of I2C message (ultrasonic: 8, odometer: 2, adc: 1, user: 1)
#define N_STATES 4
#define F_SAMPLE 61		// sensor sample rate

    int I2C_Slave_Write();
    
#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */


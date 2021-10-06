/* 
 * File:   main.h
 * Author: martin
 *
 * Created on 04 October 2021, 1:55 PM
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
#include "bot.h"

#define SYSCLK 64000000l
#define PBCLK 32000000l
#define I2C_BAUD 100000l
#define UART_BAUD 38400l
#define I2C_W 0x0
#define I2C_R 0x1
#define IMU_ADD 0x68
#define SLAVE_ADD 0x40
#define IMU_MSG_LEN 7
#define SLAVE_MSG_LEN 12
#define SOUND_SPEED 330000l		// speed of sound (mm/s)


#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */


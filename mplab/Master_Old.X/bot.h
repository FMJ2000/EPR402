/* 
 * File:   bot.h
 * Author: martin
 *
 * Created on 04 October 2021, 1:55 PM
 */

#ifndef BOT_H
#define	BOT_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
    
#include "periph.h"
    
#define X 0
#define Y 1
#define Z 2
#define F_UPDATE 15.25878
#define DT 1.0 / F_UPDATE
#define WHEEL_CIRC 22.0
#define N_ODO 20

    /* Bot object that contains state */
    typedef struct Bot {
        float pos[3];           // position [x, y, rot]
        float desiredPos[3];    // desired position [x, y, rot]
        float speed[2];         // current speed of wheels
        float view[4];          // ultrasonic distance reading
        int odo[2];             // odometer reading
        float temp;             // temperature reading
        float acc[3];           // IMU acceleration reading
        float gyro[3];          // IMU gyroscope reading
        char fall;              // bot fall hazard warning flags
        float bat;              // battery percentage remaining
        float duty[2];          // PWM duty cycle for wheels
        char msg[256];          // UART output message
        char ** map;            // probabilistic map of world
    } Bot;
    
    /* Bot helper functions */
    void Bot_Periph(Bot * bot, int * imu_data, char * slave_data);
    void Bot_Navigate(Bot * bot);
    
    /* Bot print to UART functions */
    void Bot_Print(Bot * bot);
    void Bot_Print_Ultrasonic(Bot * bot);
    void Bot_Print_String(Bot * bot, const char *msg, ...);

#ifdef	__cplusplus
}
#endif

#endif	/* BOT_H */


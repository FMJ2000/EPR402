#ifndef MASTER_H
#define MASTER_H

#include <xc.h>
#include <proc/p32mx270f256b.h>
#include <sys/attribs.h>
#include <sys/kmem.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "periph.h"
#include "imu.h"
#include "oled.h"
#include "bot.h"
#include "aux.h"

/* definitions */
#define SYSCLK 64000000l
#define PBCLK 32000000l
#define TMR1_PR 16666
#define FREQ PBCLK / (64.0 * TMR1_PR)
#define DT 1.0 / FREQ    
#define US_TRIG_T 640           // ultrasonic trigger
#define US_SENSORS 3
#define SENSOR_OFFSET 40. * M_PI / 180.  // angle offset between two sensors
#define SENSOR_ANGLE 15. * M_PI / 180.

/* static global variables/constants */
static struct Bot * bot;
static char posModifier[8][2] = {
    {-1, 1},
    {0, 1},
    {1, 1},
    {1, 0},
    {1, -1},
    {0, -1},
    {-1, -1},
    {-1, 0}
};
static float sensorOffsets[US_SENSORS + 1] = { SENSOR_OFFSET, 0.0, -SENSOR_OFFSET, M_PI };
static float sensorModifier[US_SENSORS][US_SENSORS] = { 
    {3, 1, -3},
    {0.5, 1, 0.5},
    {-3, 1, 3}
};

void Master_Init();
void SYS_Unlock();
void SYS_Lock();

#endif
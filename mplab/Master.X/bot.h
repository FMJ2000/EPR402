#ifndef _BOT_H
#define _BOT_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "map.h"
#include "aux.h"
#include "periph.h"
#include "imu.h"

#define DT_IMU 0.025		// 40 Hz
#define DT_ODO 0.25			// 4 Hz
#define F_IMU 40.0
#define F_ODO 40.0

// ukf
#define UKF_A 1e-3
#define UKF_B 2
#define UKF_K 0
#define UKF_N 6
#define UKF_T 13
#define UKF_L -5.994
#define SIGMA_Q 0.05
#define SIGMA_R 0.05

// controller
#define PWM_T 0xFFF
#define PWM_V 48
#define WHEEL_R 0.032
#define WHEEL_HOLES 20.0
#define CHASSIS_L 0.15

// auxiliary
#define BUF_LEN 1024
#define OLED_LINE_LEN 24

// state
#define INIT 0
#define IDLE 1
#define NAVIGATE 2
#define BATTERY 3
#define STATE_MASK 0x3

// battery
#define ADC_MAX 192
#define ADC_MIN 120
#define ADC_SCALE 100.0 / (ADC_MAX - ADC_MIN)

static uint8_t posMod[8][2] = {
		{-1, 1},
		{0, 1},
		{1, 1},
		{1, 0},
		{1, -1},
		{0, -1},
		{-1, -1},
		{-1, 0}
	};

// bot struct
struct Bot {
	// state
	float pos[UKF_N];				// x, y, rot, vx, vy, w		
	float xp[UKF_N];				// mean of estimate
	float zp[UKF_N];				// mean of measurement
	float v[UKF_N][1];			// innovation


	// ukf
	float P[UKF_N][UKF_N];			// covariance P
	float Pp[UKF_N][UKF_N];			// covariance estimate
	float Pzz[UKF_N][UKF_N];		// covariance measurement
	float Pvv[UKF_N][UKF_N];		// with noise
	float Pxz[UKF_N][UKF_N];		// cross correlation
	float Q[UKF_N][UKF_N];			// process noise
	float R[UKF_N][UKF_N];			// measurment noise
	float X[UKF_T][UKF_N];			// sigma points
	float Y[UKF_T][UKF_N];			// processed sigma points
	float Z[UKF_T][UKF_N];			// measured sigma points
	float Wm[UKF_T];						// mean weights
	float Wp[UKF_T];						// covariance weights

	// sensors
	float dist[3];			// latest distance readings
	float imu[3];				// imu reading: ax, ay, gz
	float odo[2];				// odometry reading
    float asa[3];
    uint8_t usState;

	// controller
    char state;
	float duty[2];			// duty cycle
	
	// map
	struct Map * map;		// current submap
    
    // battery
    float battery;
    
    // auxiliary
    int portCN;
    long long time;
    uint8_t count;
    uint8_t dblClickCount;
    unsigned char buf[BUF_LEN];
};

void Bot_Init(struct Bot ** bot, float pos[3]);
void Bot_Map_Required(struct Bot * bot);
void Bot_Map_Update(struct Bot * bot);
void Bot_Pos_IMU(struct Bot * bot);
void Bot_Pos_Odo(struct Bot * bot);
void Bot_UKF_Sigma(struct Bot * bot);
void Bot_UKF_Mean(uint8_t rows, float result[rows], float w[UKF_T], float X[UKF_T][rows]);
void Bot_UKF_Cov(uint8_t rows, float result[rows][rows], float w[UKF_T], float X[UKF_T][rows], float m[rows]);
void Bot_UKF_Update(struct Bot * bot);
void Bot_Display_Status(struct Bot * bot);
void Bot_UART_Send_Status(struct Bot * bot);
void Bot_UART_Write(struct Bot * bot, char * format, ...);

#endif
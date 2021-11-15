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
#define GOAL_LEN 256
#define K_DO 0.4
#define K_DA 0.08
#define K_RO M_PI
#define K_RA 0.4
#define K_DP 0.8
#define K_DI 0.002
#define K_DD 0.04
#define K_RP 0.4
#define K_RI 0.0002
#define K_RD 0.02
#define MIN_GOAL_DIST 0.04

// sensors
#define SOUND_SPEED 1.65e-4              // TMR5*SOUND_SPEED for distance

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
    float bias[3];
    long long numBias;

	// sensors
	float dist[3];			// latest distance readings
	float imu[3];				// imu reading: ax, ay, gz
	float odo[2];				// odometry reading
	uint8_t usState;

	// controller
	char state;
	float goal[GOAL_LEN][2];
	unsigned int goalIndex;
	float ePosInt[2];
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

void Bot_Bias(struct Bot * bot);
void Bot_Pos_IMU(struct Bot * bot);
void Bot_Pos_Odo(struct Bot * bot);
void Bot_UKF_Init(struct Bot * bot);
void Bot_UKF_Sigma(struct Bot * bot);
void Bot_UKF_Mean(uint8_t rows, float result[rows], float X[UKF_T][rows]);
void Bot_UKF_Cov(uint8_t rows, float result[rows][rows], float X[UKF_T][rows], float m[rows]);
void Bot_UKF_Update(struct Bot * bot);

void Bot_Control(struct Bot * bot);
void Bot_Trajectory(struct Bot * bot);
void Bot_Explore(struct Bot * bot);

void Bot_Display_Status(struct Bot * bot);
void Bot_UART_Send_Status(struct Bot * bot);
void Bot_UART_Write(struct Bot * bot, char * format, ...);
void Mat_Print(struct Bot * bot, uint8_t rows, uint8_t cols, float mat[rows][cols], char * title);
void Vec_Print(struct Bot * bot, uint8_t cols, float vec[cols], char * title);

#endif
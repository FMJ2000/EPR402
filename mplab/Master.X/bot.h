#ifndef _BOT_H
#define _BOT_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "node.h"
#include "map.h"
#include "aux.h"
#include "periph.h"
#include "imu.h"

#define DT 0.05
#define FREQ 20

// ukf
#define UKF_A 1e-1
#define UKF_B 2
#define UKF_K 0
#define UKF_N 5
#define UKF_T 2*UKF_N+1
#define UKF_L (float)UKF_A*UKF_A * (UKF_N + UKF_K) - UKF_N
#define SIGMA_Q 0.9
#define SIGMA_R 0.05
#define K_OLD 0.5
#define K_ODO 0.4
#define W_ODO 0.769230769
#define ODO_LEN 5

// controller
#define PWM_T 0xFFF
#define PWM_V 32
#define WHEEL_R 0.036
#define WHEEL_HOLES 20.0
#define CHASSIS_L 0.15
#define K_DO 0.36
#define K_DA 0.02
#define K_RO 0.7
#define K_RA 0.5
#define K_DP 0.8
#define K_DI 0.002
#define K_DD 0.002
#define K_RP 0.8
#define K_RI 0.0002
#define K_RD 0.001
#define K_UV 0.2
#define K_UW 0.2
#define MIN_GOAL_DIST 0.08
#define MIN_DC 0.08
#define INTEGRAL_LEN 40
#define TURN_REF 0.6981318
#define TURN_CONST 0.13
#define FORWARD_CONST 0.18

// path planning
#define MAX_COL_COUNT 5
#define V_REF 0.1
#define W_REF 0.06
#define NAV_STEP 0.15
#define NAV_SQRT 0.21213203
#define MIN_SEARCH_GOAL 0.11
#define GOAL_LEN 256
#define MAX_SEARCH_ITER 128
#define MIN_OBST_DIST 0.05
#define VISIT_COST 0.2
#define REVERSE_SPEED 0.84
#define VACUUM_SPEED 1
#define ERR_REF 0.008
#define EXPOS_LEN 5

// sensors
#define SOUND_SPEED 3.3e-4              // TMR5*SOUND_SPEED for distance
#define K_MAG 1

// auxiliary
#define BUF_LEN 2700
#define OLED_LINE_LEN 24

// state
#define INIT 0
#define IDLE 1
#define FINISH 2
#define REVERSE 3
#define NAVIGATE 4
#define VACUUM 5

// battery
#define ADC_MAX 192
#define ADC_MIN 120
#define ADC_SCALE 100.0 / (ADC_MAX - ADC_MIN)

static const int bPosMod[8][2] = {
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
	float pos[UKF_N];						// x, y, rot, v, w
	float opos[UKF_N];
	float ePos[2];

	// sensors
	float dist[3];							// latest distance readings
	float imu[4];								// imu reading: ax, ay, gz
	float odoArr[ODO_LEN][2];								// odometry reading
	float odo[2];
	uint8_t odoIndex;
	float wOdo[ODO_LEN];
	uint8_t usState;
	unsigned int usCount;
	float asa[2];
	float bias[4];
	long long numBias;

	// controller
	char state;
	float goal[GOAL_LEN][2];
	unsigned int goalIndex;
	float ePosInt[INTEGRAL_LEN][2];
	uint8_t ePosIndex;
	float duty[2];							// duty cycle
	float uGoal[2];							// desired velocity input to motor control
	float terrainMod;
    char inReverse;
    unsigned int reverseCount;

	// planning
	uint8_t collideCount;
	float explorePos[2];				// desired position for information gain
	struct Node * goalNode;
	
	// map
	struct Map * map;						// current submap
    long numMaps;                           // amount of maps
    
	// battery
	float battery;
    
	// auxiliary
	int portCN;
	long long time;
	uint8_t count;
	uint8_t dblClickCount;
    uint8_t noPathCount;
    uint8_t exploreCount;
	unsigned char buf[BUF_LEN];
};

void Bot_Init(struct Bot ** bot, float pos[3]);
void Bot_Map_Required(struct Bot * bot);
void Bot_Map_Update(struct Bot * bot);
uint8_t Bot_Map_View(struct Bot * bot);

void Bot_Bias(struct Bot * bot);
void Bot_Pos_Update(struct Bot * bot);
void Bot_Motion_Model(float x[5], float dt);
void Bot_Motor_Control(struct Bot * bot);
void Bot_Pos_Control(struct Bot * bot);

void Bot_Goal_Queue(struct Bot * bot, uint8_t len, float queue[len][2]);
char Bot_Navigate(struct Bot * bot);
void Bot_Backtrack(struct Bot * bot, struct Node * node);
void Bot_Explore(struct Bot * bot);
void Bot_Sweep(struct Bot * bot);

char Bot_Detect_Collision(struct Bot * bot);
void Bot_Quick_Reverse(struct Bot * bot);
void Bot_Stop_Reverse(struct Bot * bot);
void Bot_Vacuum(char turnOn);

void Bot_Display_Status(struct Bot * bot);
void Bot_Display_Map(struct Bot * bot);
void Bot_UART_Status(struct Bot * bot);
void Bot_UART_NodeQueue(struct Bot * bot, struct NodeQueue * queue);
void Bot_UART_Node(struct Bot * bot, struct Node * node);
void Bot_UART_Map(struct Bot * bot);
void Bot_UART_Write(struct Bot * bot, char * format, ...);
void Mat_Print(struct Bot * bot, uint8_t rows, uint8_t cols, float mat[rows][cols], char * title);
void Mat_Print_Char(struct Bot * bot, uint8_t rows, uint8_t cols, char mat[rows][cols], char * title);
void Map_Print_String(struct Bot * bot, int index, struct Map * map);
void Vec_Print(struct Bot * bot, uint8_t cols, float vec[cols], char * title);

#endif

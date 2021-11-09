#ifndef _BOT_H
#define _BOT_H

#include <stdarg.h>
#include <math.h>

#include "master.h"
#include "periph.h"

#define PWM_T 0xFFF
#define VEL_MAX 48                     // maximum wheel spin speed (rad) if duty 100%
#define WHEEL_RADIUS 0.032
#define WHEEL_HOLES 20.0
#define ODO_SCALE M_PI / WHEEL_HOLES
#define CHASSIS_LENGTH 0.15
#define ALPHA 0.1
#define BETA M_PI / 2.0
#define GAMMA 0.1

// sensor fusion
#define K_ODO 0.8
#define K_ACC 0.2
#define K_GYRO 0.8
#define K_MAG 0.02
#define K_VEL 1.5

// pi controller
#define K_OFFSET 0.18
#define K_TURN 4
#define K_DIST 0.005

// FIR filter
#define FIR_F 10.0
#define FIR_FP 0.2
#define FIR_FS 2.0
#define FIR_N 18

#define ERROR_COLLIDE 0.2618            // 15 deg
#define ERROR_MIN 0.3491                // 20 deg
#define ERROR_MAX 2.7925                // 160 deg
#define MIN_DIST 0.04
#define MIN_OBST_DIST 0.1
#define MIN_GOAL_DIST 0.1
#define MAX_GOAL_DIST 0.3
#define MIN_US_DIST 0.03                // minimum distance visible
#define MAX_US_DIST 0.9
#define MIN_PWM 0.1

#define Q_VAL 0.5
#define R_VAL 0.5
#define INPUTQ_SIZE 50
#define ADC_MAX 192
#define ADC_MIN 120
#define ADC_SCALE 100.0 / (ADC_MAX - ADC_MIN)
#define DIST_CORR_OFFSET 0.03
#define ODO_WEIGHT 0.8
 
#define US_SENSORS 3
#define SENSOR_OFFSET 40. * M_PI / 180.  // angle offset between two sensors
#define SENSOR_ANGLE 15. * M_PI / 180.
#define SOUND_SPEED 1.65e-4              // TMR5*SOUND_SPEED for distance
#define MAP_SIZE 1.28
#define MAP_RES 0.08
#define MAP_UNITS 16
#define MAP_UNITS_BIT 4
#define DEFAULT_VAL 0x07
#define US_SIGMA 0.03
#define NAV_SIGMA 0.2618 / 2
#define N_RANDOM_TRIES 100
#define BIAS_MAX 1000

#define BUF_LEN 1024
#define OLED_LINE_LEN 24
#define INPUT_LEN 3

// state
#define INIT 0
#define IDLE 1
#define NAVIGATE 2
#define BATTERY 3
#define STATE_MASK 0x3
#define STATE_MASK0 0x1
#define STATE_MASK1 0x2
#define STATE_UART_MASK 0x4
#define STATE_TX_MASK 0x8

/* bot data structures */
struct ProbMap {
    float pos[2];
    char grid[MAP_UNITS][MAP_UNITS];
};

struct BitMap {
    float pos[2];
    char grid[MAP_UNITS][MAP_UNITS_BIT];
    struct BitMap * neighbors[8];
};

struct Bot {
    /* positioning */
    unsigned long time;
    float pos[3];               // x, y, rot
    float dPos[3];              // pos at last desired pos, angle at last map
    float goal[2];       
    uint8_t odo[2];
    unsigned int execIndex;
    unsigned int addIndex;
    float ePos[2];              // pid error r, theta
    float duty[2];
    float gyro[3];
    float acc[3];
    float vel[3];              // prior velocity
    float vEst[3];
    float mag[3];
    float asa[3];               // magnetometer adjustment
    float bias[6];
    unsigned int numBias;
    
    /* mapping */
    char usState;
    float distances[3];
    uint8_t obstructed;                     // bitwise obstacle flags
    char changeLock;                        // do not use maps while recreating them
    struct BitMap * currentMaps[4];             // current bitmap of bot
    char viewMaps[4][MAP_UNITS][MAP_UNITS_BIT];
    uint8_t mapsDir;
    unsigned int numMaps;
    float battery;
    
    /* navigation */
    float randomAngle;
    
    /* auxiliary */
    char count;
    char state;
    char uartState;
    int portCN;
    unsigned char buf[BUF_LEN];
    float * angleModifier;
    float sensorOffsets[US_SENSORS];
    float obsModifier[8];
    float obsAngles[2];
    uint8_t collisionCount[2];
    uint8_t posIndex;
    
    /* filter */
    float firW[FIR_N / 2];
    float firHd[FIR_N / 2];
    float firH[FIR_N];
    float firX[FIR_N];
    
    uint8_t posModifier[8][2];
};

/* bot functions */
void Bot_Pos_Update(struct Bot * bot, uint8_t startIndex);
float Bot_InputPos_Update(struct Bot * bot);
void Bot_Map_Required(struct Bot * bot);
void Bot_Reinforce_Neighbors(struct BitMap * map);
void Bot_Map_Update(struct Bot * bot);
void Bot_Controller(struct Bot * bot);
void Bot_Vector_Angles(struct Bot * bot, float angles[US_SENSORS]);
void Bot_Display_Status(struct Bot * bot);
void Bot_UART_Send_Status(struct Bot * bot);
void Bot_UART_Write(struct Bot * bot, char * format, ...);
void Bot_Display_BitMap(struct Bot * bot);
void Bot_Navigate(struct Bot * bot, uint8_t index) ;
uint8_t Bot_Odometer_Read(struct Bot * bot, uint8_t times);

/* map functions */
void BitMap_Initialize(struct Bot * bot, struct BitMap ** ptr, float pos[2]);
char BitMap_Contains(struct BitMap * map, uint8_t index[2], float pos[2]);
char Bitmap_At(struct BitMap * map, uint8_t index[2]);
void BitMap_Set(struct BitMap * map, uint8_t index[2], char val);
char BitMap_IndexToPos(struct BitMap * map, float pos[2], uint8_t index[2]);

/* filter functions */
void Bot_FIR_Init(struct Bot * bot);
float Bot_FIR_Filter(struct Bot * bot, uint8_t n); 

#endif
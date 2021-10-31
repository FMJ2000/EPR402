#ifndef _BOT_H
#define _BOT_H

#include <stdarg.h>

#include "master.h"
#include "periph.h"

#define PWM_T 0xFFF
#define VEL_MAX 0.55                     // maximum speed (m/s) if duty 100%
#define WHEEL_DIAMETER 0.22
#define WHEEL_RADIUS 0.03
#define CHASSIS_LENGTH 0.15
#define ALPHA 0.1
#define BETA M_PI / 2.0
#define GAMMA 0.1   
#define K_EST 0.8
#define K_OFFSET 0.4
#define K_ALPHA 0.024
#define K_TURN 2.5
#define ERROR_MAX 5. * M_PI / 180.
#define MIN_DIST 0.04
#define MIN_OBST_DIST 0.08
#define MIN_INPUT_DIST 0.3
#define MIN_US_DIST 0.03                // minimum distance visible
#define MAX_US_DIST 0.9
#define MIN_PWM 0.1
#define Q_VAL 0.5
#define R_VAL 0.5
#define INPUTQ_SIZE 50
#define ADC_SCALE 4.54
#define ADC_OFFSET 170
#define DIST_CORR_OFFSET 0.03
 
#define US_SENSORS 3
#define SENSOR_OFFSET 40. * M_PI / 180.  // angle offset between two sensors
#define SENSOR_ANGLE 15. * M_PI / 180.
#define SOUND_SPEED 1.65e-4              // TMR2*SOUND_SPEED for distance
#define MAP_SIZE 0.64
#define MAP_RES 0.04
#define MAP_UNITS 16
#define MAP_UNITS_BIT 2
#define DEFAULT_VAL 0x07
#define US_SIGMA 0.03
#define NAV_SIGMA 0.2618 / 2
#define N_RANDOM_TRIES 100
#define BIAS_MAX 1000
#define IDLE 0
#define NAVIGATE 1
#define BUF_LEN 1024

#define STATE_MASK 0x1
#define STATE_UART_MASK 0x2
#define STATE_TX_MASK 0x3

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
    float inputPos[2];
    uint8_t odo[2];
    unsigned int execIndex;
    unsigned int addIndex;
    float ePos[3];              // pid error
    float duty[2];
    float gyro[3];
    float acc[3];
    float mag[3];
    float asa[3];               // magnetometer adjustment
    float bias[3];
    unsigned int numBias;
    
    /* mapping */
    char usState;
    float distances[3];
    char changeLock;                        // do not use maps while recreating them
    struct BitMap * currentMap;             // current bitmap of bot
    struct ProbMap * localMaps[4];              // modifiable copies in between samples (0 is current map, 1-3 viewed maps)
    char localViewMaps[4][MAP_UNITS][MAP_UNITS_BIT];
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
};

/* bot functions */
void Bot_Pos_Update(struct Bot * bot);
float Bot_InputPos_Update(struct Bot * bot);
char Bot_Frontier(struct Bot * bot, float pos[2]);
void Bot_Map_Required(struct Bot * bot);
void Bot_Reinforce_Neighbors(struct BitMap * map);
void Bot_Map_Update(struct Bot * bot);
void Bot_Optimise_Local(struct Bot * bot);
void Bot_Controller(struct Bot * bot);
void Bot_Vector_Angles(struct Bot * bot, float angles[US_SENSORS]);
void Bot_Display_Status(struct Bot * bot);
void Bot_UART_Write(struct Bot * bot, char * format, ...);
void Bot_Display_BitMap(struct BitMap * map);
void Bot_Display_ProbMap(struct ProbMap * map);

/* map functions */
void BitMap_Initialize(struct Bot * bot, struct BitMap ** ptr, float pos[2]);
void ProbMap_Initialize(struct ProbMap ** ptr, float pos[2], char fillVal);
char BitMap_Contains(struct BitMap * map, float pos[2]);
char ProbMap_Contains(struct ProbMap * map, float pos[2]);

#endif
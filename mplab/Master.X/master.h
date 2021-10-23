#ifndef MASTER_H
#define MASTER_H

#include <xc.h>
#include <proc/p32mx270f256b.h>
#include <sys/attribs.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

/* definitions */
#define SYSCLK 64000000l
#define PBCLK 32000000l
#define I2C_BAUD 100000l
#define UART_BAUD 38400l
#define I2C_W 0x0
#define I2C_R 0x1
#define FREQ 30.0003
#define DT 1.0 / FREQ                    

/* MPU registers */
#define XG_OFFS_TC         0x00
#define YG_OFFS_TC         0x01
#define ZG_OFFS_TC         0x02
#define X_FINE_GAIN        0x03
#define Y_FINE_GAIN        0x04
#define Z_FINE_GAIN        0x05
#define XA_OFFS_H          0x06
#define XA_OFFS_L_TC       0x07
#define YA_OFFS_H          0x08
#define YA_OFFS_L_TC       0x09
#define ZA_OFFS_H          0x0A
#define ZA_OFFS_L_TC       0x0B
#define XG_OFFS_USRH       0x13
#define XG_OFFS_USRL       0x14
#define YG_OFFS_USRH       0x15
#define YG_OFFS_USRL       0x16
#define ZG_OFFS_USRH       0x17
#define ZG_OFFS_USRL       0x18
#define SMPLRT_DIV         0x19
#define CONFIG             0x1A
#define GYRO_CONFIG        0x1B
#define ACCEL_CONFIG       0x1C
#define FF_THR             0x1D
#define FF_DUR             0x1E
#define MOT_THR            0x1F
#define MOT_DUR            0x20
#define ZRMOT_THR          0x21
#define ZRMOT_DUR          0x22
#define FIFO_EN            0x23
#define I2C_MST_CTRL       0x24
#define I2C_SLV0_ADDR      0x25
#define I2C_SLV0_REG       0x26
#define I2C_SLV0_CTRL      0x27
#define I2C_SLV1_ADDR      0x28
#define I2C_SLV1_REG       0x29
#define I2C_SLV1_CTRL      0x2A
#define I2C_SLV2_ADDR      0x2B
#define I2C_SLV2_REG       0x2C
#define I2C_SLV2_CTRL      0x2D
#define I2C_SLV3_ADDR      0x2E
#define I2C_SLV3_REG       0x2F
#define I2C_SLV3_CTRL      0x30
#define I2C_SLV4_ADDR      0x31
#define I2C_SLV4_REG       0x32
#define I2C_SLV4_DO        0x33
#define I2C_SLV4_CTRL      0x34
#define I2C_SLV4_DI        0x35
#define I2C_MST_STATUS     0x36
#define INT_PIN_CFG        0x37
#define INT_ENABLE         0x38
#define DMP_INT_STATUS     0x39
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E
#define EXT_SENS_DATA_06   0x4F
#define EXT_SENS_DATA_07   0x50
#define EXT_SENS_DATA_08   0x51
#define EXT_SENS_DATA_09   0x52
#define EXT_SENS_DATA_10   0x53
#define EXT_SENS_DATA_11   0x54
#define EXT_SENS_DATA_12   0x55
#define EXT_SENS_DATA_13   0x56
#define EXT_SENS_DATA_14   0x57
#define EXT_SENS_DATA_15   0x58
#define EXT_SENS_DATA_16   0x59
#define EXT_SENS_DATA_17   0x5A
#define EXT_SENS_DATA_18   0x5B
#define EXT_SENS_DATA_19   0x5C
#define EXT_SENS_DATA_20   0x5D
#define EXT_SENS_DATA_21   0x5E
#define EXT_SENS_DATA_22   0x5F
#define EXT_SENS_DATA_23   0x60
#define MOT_DETECT_STATUS  0x61
#define I2C_SLV0_DO        0x63
#define I2C_SLV1_DO        0x64
#define I2C_SLV2_DO        0x65
#define I2C_SLV3_DO        0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL    0x69
#define USER_CTRL          0x6A
#define PWR_MGMT_1         0x6B
#define PWR_MGMT_2         0x6C
#define BANK_SEL           0x6D
#define MEM_START_ADDR     0x6E
#define MEM_R_W            0x6F
#define DMP_CFG_1          0x70
#define DMP_CFG_2          0x71
#define FIFO_COUNTH        0x72
#define FIFO_COUNTL        0x73
#define FIFO_R_W           0x74
#define WHO_AM_I           0x75

#define IMU_ADD 0x68
#define GYRO_SCALE 500.0
#define ACC_SCALE 4.0
#define TEMP_SCALE 16.0
#define G_CONSTANT 9.80665
#define TEMP_OFFSET 25
#define IMU_RES 32768.0
#define BIAS_MAX 1000
#define IDLE 0
#define NAVIGATE 1

#define PWM_T 0xFFF
#define PWM_H 0x7FF                     // normal full speed
#define VEL_MAX 0.55                     // maximum speed (m/s) if duty 100%
#define WHEEL_DIAMETER 0.22
#define WHEEL_RADIUS 0.03
#define CHASSIS_LENGTH 0.15
#define ALPHA 0.1
#define BETA M_PI / 2.0
#define GAMMA 0.1   
#define K_EST 0.8
#define K_TURN 1.5
#define ERROR_MAX 30. * M_PI / 180.
#define MIN_DIST 0.08
#define MIN_OBST_DIST 0.08
#define MIN_US_DIST 0.03                // minimum distance visible
#define MAX_US_DIST 0.6
#define MIN_PWM 0.1
#define Q_VAL 0.5
#define R_VAL 0.5
#define INPUTQ_SIZE 50
#define ADC_SCALE 4.54
#define ADC_OFFSET 170
#define DIST_CORR_OFFSET 0.03

#define US_TRIG_T 640                   // ultrasonic trigger
#define US_SENSORS 3
#define SENSOR_OFFSET 40. * M_PI / 180.  // angle offset between two sensors
#define SENSOR_ANGLE 15. * M_PI / 180.
#define SOUND_SPEED 1.65e-4              // TMR2*SOUND_SPEED for distance
#define MAP_SIZE 0.6
#define MAP_RES 0.03
#define MAP_UNITS 20
#define DEFAULT_VAL 0x07
#define US_SIGMA 0.03

/* bot data structure */
struct Map {
    float pos[2];
    char grid[MAP_UNITS][MAP_UNITS];
    struct Map * neighbors[8];
};

struct Bot {
    /* positioning */
    unsigned long time;
    float pos[3];               // x, y, rot
    float dPos[3];              // pos at last map update
    float inputPos[2];
    unsigned int execIndex;
    unsigned int addIndex;
    float ePos[3];              // pid error
    float duty[2];
    float imuData[3];           // accX, accY, gyroZ
    float bias[3];
    unsigned int numBias;
    char isTurning;
    
    /* mapping */
    char usState;
    char usCount;
    float distances[4];
    char mapChangeLock;
    struct Map * currentMaps[4];            // current map bot is in [0] and directional adjunctions [1-3]
    struct Map * localMaps[4];              // modifiable copies in between samples
    char globalViewMaps[4][MAP_UNITS][MAP_UNITS];
    char localViewMaps[4][MAP_UNITS][MAP_UNITS];
    unsigned int numMaps;
    float battery;
    
    /* auxiliary */
    char count;
    char state;
    int portCN;
    unsigned char buf[100];
};

/* static global variables/constants */
static struct Bot bot = { .state=0 };
static char posModifier[8][2] = {
    {-1, -1},
    {0, -1},
    {1, -1},
    {1, 0},
    {1, 1},
    {0, 1},
    {-1, 1},
    {-1, 0}
};
static float sensorOffsets[US_SENSORS + 1] = { SENSOR_OFFSET, 0.0, -SENSOR_OFFSET, M_PI };
static float sensorModifier[US_SENSORS][US_SENSORS] = { 
    {3, 1, -3},
    {0.5, 1, 0.5},
    {-3, 1, 3}
};

/* peripheral defintions */
void Init();
char IMU_Init();
void UART_Write(char data);
void UART_Write_String(char * data, int len);
void I2C_Master_Init();
void I2C_Master_Start();
void I2C_Master_Stop();
void I2C_Master_Restart();
void I2C_Master_Idle();
void I2C_Master_Ack_Nack(int val);
int I2C_Master_Write(char byte);
char I2C_Master_Read();
int I2C_Write(char periphAdd, char regAdd, char data);
int I2C_Read(char periphAdd, char regAdd, char * data, int len);

/* bot functions */
void Bot_Pos_Update();
float Bot_InputPos_Update();
void Bot_Map_Required();
void Bot_Reinforce_Neighbors(struct Map * map);
void Bot_Map_Update();
void Bot_Optimise_Local();
void Bot_Trigger_Ultrasonic();
void Bot_Controller();
//void Bot_Add_Instruction(float x, float y);
void Bot_Display_Status();
void Bot_Display_Map(struct Map * map);

/* map functions */
struct Map * Map_Initialize(float pos[2], char fillVal);
void Map_Destroy(struct Map * map);
char Map_Contains(struct Map * map, float * pos);

/* helper functions */
float getAngle(float x1, float y1, float x2, float y2);
float getDistance(float x1, float y1, float x2, float y2);
char distanceToPos(float pos[][2], float * valid, float * distances);
char Multivariate_Gaussian(float meanX, float meanY, float posX, float posY);
float Box_Muller(float mu, float sigma);

void matrix_mul( int nRows,  int nCols, int nAdd, float result[][nCols], float mat1[][nAdd], float mat2[][nCols]);
/*
void matrix_plus(int len, float result[][len], float mat1[][len], float mat2[][len]);

void matrix_inv(float result[3][3], float mat[3][3]);
*/

#endif
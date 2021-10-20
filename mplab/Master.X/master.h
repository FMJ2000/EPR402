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
#define FREQ 4.0
#define DT 1 / 4.0                     // 1 / (1000000 / 2^16)

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

#define PWM_T 0xFFF
#define PWM_H 0x8FF                     // normal full speed
#define VEL_MAX 0.5                     // maximum speed (m/s) if duty 100%
#define WHEEL_DIAMETER 0.22
#define WHEEL_RADIUS 0.03
#define ALPHA 0.1
#define BETA M_PI / 2.0
#define GAMMA 0.1   
#define ERROR_MAX 0.7854                // 45 degrees
#define MIN_DIST 0.05
#define MIN_PWM 0.1
#define Q_VAL 0.5
#define R_VAL 0.5

/* bot data structure */
typedef struct Bot {
    float pos[3];
    float sigma[3][3];
    float inputPos[2];
    float vel[2];
    float duty[2];
    float gyro;
    float acc[2];
    const float Q[3][3];
    const float R[3][3];
    char count;
    unsigned char buf[100];
    
    /* temporary */
    float theta;
    float distance;
    float maxSpeed;
    float maxTurn;
    float rotSign;
} Bot;

/* static global variables/constants */
static Bot bot = { 
    .sigma = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
    .Q = {{Q_VAL, 0, 0}, {0, Q_VAL, 0}, {0, 0, Q_VAL}},
    .R = {{R_VAL, 0, 0}, {0, R_VAL, 0}, {0, 0, R_VAL}}
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
void Bot_Controller();
void Bot_Display_Status();

/* helper functions */
void matrix_plus(float ** result, float ** mat1, float ** mat2, int len);
void matrix_mul(float ** result, float ** mat1, float ** mat2, int len);
void matrix_inv(float ** result, float ** mat);

#endif
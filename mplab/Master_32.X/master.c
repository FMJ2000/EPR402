/* 
 * File:   main.c in master
 * Author: martin
 *
 * Created on 23 September 2021, 2:51 PM
 */

// PIC32MX220F032B Configuration Bit Settings

// DEVCFG3
#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_16         // PLL Multiplier (16x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF              // JTAG Enable (JTAG Port Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#include <xc.h>
#include <proc/p32mx220f032b.h>
#include <sys/attribs.h>
#include <stdlib.h>
#include <math.h>

/* definitions */
#define SYSCLK 64000000l
#define PBCLK 32000000l
#define I2C_BAUD 100000l
#define UART_BAUD 38400l
#define IMU_ADD 0x68
#define SLAVE_ADD 0x40
#define PWM_T 0xFFF
#define PWM_H 0x5FF                     // normal full speed
#define IMU_RES 32768.0                 // IMU data resolution
#define ACC_SCALE 4.0
#define GYRO_SCALE 500.0
#define ACC_RES 1.220703e-4             // ACC_SCALE / IMU_RES
#define GYRO_RES 1.525879e-2            // GYRO_SCALE / IMU_RES
#define TEMP_OFFSET 36.53
#define TEMP_SCALE 2.941176e-3
#define IMU_MSG_LEN 14
#define SLAVE_MSG_LEN 12
#define I2C_W 0x0
#define I2C_R 0x1
#define DT 0.065536                     // 1 / (1000000 / 2^16)
#define INV_N_ODO 0.05                  // inverse of amount of odometer holes
#define ODO_SPEED 1.1                   // wheel circumference / #odometer holes = 22 / 20
#define SOUND_SPEED 0.033
#define _2_PI_INV 0.1591549		// inverse of 2 pi
#define PI_2 1.570796			// half pi
#define MIN_DIST 3.0                   // minimum distance before new RRT node chosen
#define MAX_DIST 100.0                  // maximum sphere of new RRT node
#define K_P 0.2                         // Proportional controller
#define K_I 0.001                       // Integral controller
#define Q 0.05                          // process covariance noise
#define R 0.05                          // measurement covariance noise
#define F_UART 0
#define F_PWM 1
#define F_SAMPLE 15
#define MOVE_MASK 0x2
#define DIR_MASK 0x1

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

/* Bot object that contains state */
typedef struct Bot {
    float pos[2];           // position [x, y]
    float rotMu;            // mean of rotation estimation
    float rotSigma;         // covariance of rotation estimation
    float rPos[2];          // desired position [x, y]
    float vel[2];           // current speed of wheels
    float eRotInt;          // integral of rotation error
    float duty[2];          // PWM duty cycle for wheels
    float acc[3];           // IMU acceleration reading
    float gyro[3];          // IMU gyroscope reading    
    float bat;		    // battery percentage remaining
    float view[4];          // ultrasonic distance reading
    unsigned char count;    // sample count
    char state;		    // { direction<0>, move<1> }
    /*unsigned char odo[2];           // odometer reading
    float temp;             // temperature reading
    unsigned char fall;              // bot fall hazard warning flags
    char msg[256];          // UART output message
    char ** map;            // probabilistic map of world*/
} Bot;

static Bot bot = { .pos = {0} };

/* Peripheral functions */
void Init();
#if F_UART
void UART_Write(unsigned char data);
void UART_Write_String(unsigned char * data, int len);
#endif
void I2C_Master_Init();
void I2C_Master_Start();
void I2C_Master_Stop();
void I2C_Master_Restart();
void I2C_Master_Idle();
void I2C_Master_Ack_Nack(int val);
int I2C_Master_Write(unsigned char byte);
unsigned char I2C_Master_Read();
void IMU_Init();

/* Bot functions */
void Bot_Update();
void Bot_Controller();

/* Helper functions */
float charToFloat(char val1, char val2, float weight);
float unsignedCharToFloat(char val1, char val2, float weight);
void rotationMatrix(float * x, float * y);

int main() {
    /* delay startup to allow peripherals to initialize */
    long delay = 640000l;
    while (delay--);
    
    bot.state |= MOVE_MASK;	    // start engines
    
    Init();
    IMU_Init();
    
    for (;;);    
    return EXIT_SUCCESS;
}

/* Bot functions */
void Bot_Update() {
    unsigned char data[IMU_MSG_LEN + SLAVE_MSG_LEN] = {0};
    
    /* get readings from IMU: { axh, axl, ayh, ayl, azh, azl, th, tl, gxh, gxl, gyh, gyl, gzh, gzl } */
    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_W);
    I2C_Master_Write(ACCEL_XOUT_H);
    I2C_Master_Stop();
    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_R);
    for (int i = 0; i < IMU_MSG_LEN; i++) {
	data[i] = I2C_Master_Read();
        if (i != IMU_MSG_LEN - 1) I2C_Master_Ack_Nack(0);
    }
    I2C_Master_Ack_Nack(1);
    I2C_Master_Stop();
    
    /* get readings from slave: { v1h, v1l, v2h, v2l, v3h, v3l, v4h, v4l, odl, odr, bat, usr } */
    I2C_Master_Start();
    I2C_Master_Write((SLAVE_ADD << 1) | I2C_R);
    for (int i = 0; i < SLAVE_MSG_LEN; i++) {
	data[IMU_MSG_LEN + i] = I2C_Master_Read();
	if (i != SLAVE_MSG_LEN - 1) I2C_Master_Ack_Nack(0);
    }
    I2C_Master_Ack_Nack(1);
    I2C_Master_Stop();
    
#if F_UART
    if (bot.count++ == 1/*F_SAMPLE*/) {
	UART_Write_String(data, IMU_MSG_LEN + SLAVE_MSG_LEN);
	bot.count = 0;
    }
#endif
    
    /* save in bot */
    for (int i = 0; i < 3; i++) {
        bot.acc[i] = charToFloat(data[2*i], data[2*i+1], ACC_RES);
	bot.gyro[i] = charToFloat(data[2*i+8], data[2*i+9], GYRO_RES);
        bot.view[i] = unsignedCharToFloat(data[2*i+14], data[2*i+15], SOUND_SPEED);
    }
    /*bot.view[3] = charToFloat(data[20], data[21], SOUND_SPEED / 1000000.0);
	bot.temp = charToFloat(data[6], data[7], TEMP_SCALE) + TEMP_OFFSET;
    bot.odo[0] = data[22]; //charToFloat(0, data[22], 1);
    bot.odo[1] = data[23]; //charToFloat(0, data[23], 1);*/
    bot.bat = unsignedCharToFloat(0, data[24], 1);

    /* Kalman filter for orientation estimation */
    float eRotMu = bot.rotMu + bot.gyro[2] * DT;
    float eRotSigma = bot.rotSigma + Q;
    float z = atan2(sqrt(bot.acc[0]*bot.acc[0] + bot.acc[1]*bot.acc[1]), bot.acc[2]);
    float K = eRotSigma / (eRotSigma + R);
    bot.rotMu = eRotMu + K * (z - eRotMu);
    bot.rotSigma = eRotSigma - K * eRotSigma;
    
    /* update position */
    rotationMatrix(bot.vel, bot.acc);
    rotationMatrix(bot.pos, bot.vel);
    
    /* PI controller */
    float rRotMu = atan2(bot.rPos[1] - bot.pos[1], bot.rPos[0] - bot.pos[0]);
    eRotMu = rRotMu - bot.rotMu;
    float halfPWM = 0.5 - _2_PI_INV * fabs(eRotMu);
    bot.eRotInt += eRotMu * DT;
    bot.duty[0] = halfPWM - K_P * eRotMu - K_I * bot.eRotInt;
    bot.duty[1] = halfPWM + K_P * eRotMu + K_I * bot.eRotInt;
    
    if (bot.view[0] < MIN_DIST/* || bot.view[1] < MIN_DIST || bot.view[2] < MIN_DIST*/) {
	bot.state &= ~MOVE_MASK;
	//bot.state |= DIR_MASK;
	bot.count = 0;
	T4CONSET = _T4CON_ON_MASK;
    }
    
    Bot_Controller();
    
    /* check if RRT node still valid, update otherwise */	
    if (fabs(bot.rPos[0] - bot.pos[0]) + fabs(bot.rPos[1] - bot.pos[1]) < MIN_DIST) {
        bot.rPos[0] = bot.pos[0] + MAX_DIST * (rand() - 0.5);
        bot.rPos[1] = bot.pos[1] + MAX_DIST * (rand() - 0.5);
    }
}

void Bot_Controller() {
    float oc4 = bot.duty[0];
    float oc5 = bot.duty[1];
    if (!(bot.state & MOVE_MASK)) {
	oc4 = 0;
	oc5 = 0;
    } else if (bot.state & DIR_MASK) {
	oc4 = 1 - oc4;
	oc5 = 1 - oc5;
	LATBSET = _LATB_LATB4_MASK;
	LATASET = _LATA_LATA3_MASK;
    } else {
	LATBCLR = _LATB_LATB4_MASK;
	LATACLR = _LATA_LATA3_MASK;
    }
    
    OC4RS = (int)(oc4 * PWM_H);
    OC5RS = (int)(oc5 * PWM_H);
    
    if (bot.rotMu > PI_2) LATBSET = _LATB_LATB10_MASK;
    else if (bot.rotMu < -PI_2) LATBSET = _LATB_LATB11_MASK;
    else {
	LATBCLR = _LATB_LATB10_MASK;
	LATBCLR = _LATB_LATB11_MASK;
    }
}

/* Peripheral functions */
void Init() {
    /* Random function */
    srand(PORTA);
    
    /* Ports */
    ANSELA = 0x0;
    TRISA = 0x0;
    LATA = 0x0;

    ANSELB = 0x0;
    TRISB = 0x38C;

    U1RXR = 0x3;		// RB13 = RX
    RPA2R = 0x5;		// RA2 = OC4
    RPA4R = 0x6;		// RA4 = OC5
    RPB15R = 0x1;		// RB15 = TX
    INT1R = 0x4;		// RB9 = INT1
    INT3R = 0x4;		// RB8 = INT3
    INT4R = 0x0;		// RA0 = INT4
    
    /* Interrupts */
    __builtin_disable_interrupts();
    INTCONSET = _INTCON_MVEC_MASK;
    IPC0 = 0xA000000;		// INT0
    IPC1 = 0xA000000;		// INT1
    IPC3 = 0xA00000A;		// INT3, TMR3
    IPC4 = 0xA00000A;		// INT4, TMR4
    IFS0 = 0x0;
    IFS1 = 0x0;
    IEC0 = 0x8C6108;       // INT0, INT1, INT2, TMR3, INT3, TMR4, INT4
    __builtin_enable_interrupts();
    
    /* Timers + PWM */
    TMR2 = 0x0;
    PR2 = PWM_T;
    OC4R = 0x0;
    OC4RS = 0x0;
    OC4CON = 0x6;		// PWM mode without fault protection
    OC5R = 0x0;
    OC5RS = 0x0;
    OC5CON = 0x6;
    T2CON = 0x8060;		// 1:64 prescaler = 500 kHz
    OC4CONSET = _OC4CON_ON_MASK;
    OC5CONSET = _OC5CON_ON_MASK;
    
    /* Sampling timer */
    TMR3 = 0x0;
    PR3 = 0xFFFF;		// f = 15 Hz
    T3CON = 0x8050;		// 1:32 prescaler = 1 MHz
    
    /* Long counter */
    TMR4 = 0x0;
    PR4 = 0xFFFF;		// 2 Hz
    T4CON = 0x70;		// 1:256 prescaler = 125 kHz
    
    /* I2C */
    I2C2BRG = (int)(PBCLK / (2 * I2C_BAUD) - 2);
    I2C2CONSET = _I2C2CON_ON_MASK;
    
#if F_UART
    /* UART */
    U1MODE = 0x0;
    U1STA = 0x0;
    U1BRG = 51;//(int)(PBCLK / (16 * UART_BAUD) - 1);
    U1STASET = (_U1STA_UTXISEL0_MASK | _U1STA_URXEN_MASK | _U1STA_UTXEN_MASK);
    U1MODESET = _U1MODE_ON_MASK;
#endif
}

#if F_UART
void UART_Write(unsigned char data) {
    while (U1STAbits.UTXBF);
    U1TXREG = data;
}

void UART_Write_String(unsigned char * data, int len) {
    while (len--) {
	UART_Write(*data);
	data++;
    }
}
#endif

void I2C_Master_Start() {
    while (I2C2CON & 0x1F & I2C2STATbits.TRSTAT);
    I2C2CONbits.SEN = 1;
    while (I2C2CONbits.SEN);
}

void I2C_Master_Stop() {
    while (I2C2CON & 0x1F & I2C2STATbits.TRSTAT);
    I2C2CONbits.PEN = 1;
    while (I2C2CONbits.PEN);
}

void I2C_Master_Restart() {
    while (I2C2CON & 0x1F & I2C2STATbits.TRSTAT);
    I2C2CONbits.RSEN = 1;
    while (I2C2CONbits.RSEN);
}

void I2C_Master_Ack_Nack(int val) {
    // ACK = 0 (slave should send another byte)
    // NACK = 1 (no more bytes requested by slave)
    while (I2C2CON & 0x1F & I2C2STATbits.TRSTAT);
    I2C2CONbits.ACKDT = val;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN);
}

int I2C_Master_Write(unsigned char byte) {
    I2C2TRN = byte;
    while (I2C2STATbits.TRSTAT);
    if (I2C2STATbits.ACKSTAT) return 0;
    return 1;
}

unsigned char I2C_Master_Read() {
    I2C2CONbits.RCEN = 1;
    while (!I2C2STATbits.RBF);
    return I2C2RCV;
}

void IMU_Init() {
    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_W);
    I2C_Master_Write(SMPLRT_DIV);
    I2C_Master_Write(0x07);
    I2C_Master_Stop();

    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_W);
    I2C_Master_Write(PWR_MGMT_1);
    I2C_Master_Write(0x01);
    I2C_Master_Stop();

    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_W);
    I2C_Master_Write(CONFIG);
    I2C_Master_Write(0x00);
    I2C_Master_Stop();

    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_W);
    I2C_Master_Write(ACCEL_CONFIG);
    I2C_Master_Write(0x08);			// 4g
    I2C_Master_Stop();

    I2C_Master_Write((IMU_ADD << 1) | I2C_W);
    I2C_Master_Write(GYRO_CONFIG);
    I2C_Master_Write(0x08);			// 500 dps
    I2C_Master_Stop();

    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_W);
    I2C_Master_Write(INT_ENABLE);
    I2C_Master_Write(0x01);
    I2C_Master_Stop();

    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_W);
    I2C_Master_Write(WHO_AM_I);
    I2C_Master_Stop();

    I2C_Master_Start();
    I2C_Master_Write((IMU_ADD << 1) | I2C_R);
    I2C_Master_Read();
    I2C_Master_Ack_Nack(1);
    I2C_Master_Stop();
}

/* helper functions

/* standardize double char conversion to float */
float charToFloat(char val1, char val2, float weight) {
    return (float)((val1 << 8) | val2) * weight;
}

float unsignedCharToFloat(char val1, char val2, float weight) {
    return (float)((unsigned int)((val1 << 8) | val2) * weight);
}

void rotationMatrix(float * x, float * y) {
    float cos_rot = cos(bot.rotMu);
    float sin_rot = sin(bot.rotMu);
    x[0] += (y[0] * cos_rot - y[1] * sin_rot) * DT;
    x[1] += (y[0] * sin_rot + y[1] * cos_rot) * DT;
}

/* Interrupt handlers */
void __ISR(_TIMER_3_VECTOR, IPL2SOFT) TMR3_IntHandler() {
    IFS0CLR = _IFS0_T3IF_MASK;
    Bot_Update();
}

void __ISR(_TIMER_4_VECTOR, IPL2SOFT) TMR4_IntHandler() {
    IFS0CLR = _IFS0_T4IF_MASK;
    if (bot.count++ == 4) {
	bot.count = 0;
	T4CONCLR = _T4CON_ON_MASK;	
	//bot.state &= ~DIR_MASK;
	bot.state |= MOVE_MASK;
    }
}

/*
void __ISR(_EXTERNAL_0_VECTOR, IPL2SOFT) Ext0_IntHandler() {
	IFS0CLR = _IFS0_INT0IF_MASK;
	bot.fall |= 0x1;
}

void __ISR(_EXTERNAL_1_VECTOR, IPL2SOFT) Ext1_IntHandler() {
	IFS0CLR = _IFS0_INT1IF_MASK;
	bot.fall |= 0x2;
}

void __ISR(_EXTERNAL_3_VECTOR, IPL2SOFT) Ext3_IntHandler() {
	IFS0CLR = _IFS0_INT3IF_MASK;
	bot.fall |= 0x4;
}

void __ISR(_EXTERNAL_4_VECTOR, IPL2SOFT) Ext4_IntHandler() {
	IFS0CLR = _IFS0_INT4IF_MASK;
	bot.fall |= 0x8;
}
 * */
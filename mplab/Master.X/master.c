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

#include "master.h"

int main() {
    long delay = 640000l;
    while (delay--);
    
    float mat1[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    float mat2[3][3] = {{8, 7, 8}, {6, 5, 6}, {4, 3, 4}};
    float result[3][3];
    matrix_mul((float **)result, (float **)mat1, (float **)mat2, 3);
    matrix_inv((float **)result, (float **)mat1);
    
    bot.inputPos[0] = 1.5;
    bot.inputPos[1] = 0.0;
    
    Init();
    UART_Write_String("Peripherals Initialized\r\n", 26);
    char whoami = IMU_Init();
    
    snprintf(bot.buf, 100, "IMU address: %d\r\n", whoami);
    UART_Write_String(bot.buf, strlen(bot.buf));
    
    for (;;);
    return EXIT_SUCCESS;
}

void Bot_Pos_Update() {
    /* Get IMU sensor data */
    uint8_t data[6];
    I2C_Read(IMU_ADD, GYRO_ZOUT_H, data, 2);
    I2C_Read(IMU_ADD, ACCEL_XOUT_H, &data[2], 4);
    
    int16_t val = (data[0] << 8) | data[1];
    bot.gyro = (float)val * GYRO_SCALE / IMU_RES;
    for (int i = 1; i < 3; i++) {
	val = (data[2*i] << 8) | data[2*i+1];
	bot.acc[i-1] = (float)val * ACC_SCALE / IMU_RES;
    }
    
    /* estimate next position with current trajectory */
    float vel[2] = { bot.duty[0] * VEL_MAX, bot.duty[1] * VEL_MAX };
    float dPos[3] = {0};
    float ePos[3], eSigma[3][3];
    
    memcpy(ePos, bot.pos, sizeof(float) * 3);
    matrix_plus((float **)eSigma, (float **)bot.sigma, (float **)bot.Q, 3);
    
    if (vel[0] == vel[1]) {
	// bot moving straight ahead
	dPos[0] = vel[0] * DT * cos(bot.pos[2]);
	dPos[1] = vel[1] * DT * sin(-bot.pos[2]);
    } else {
	float radius = 0.0;
	if (vel[0] > vel[1]) {
	    // turning right
	    radius = vel[1] / (vel[0] - vel[1]) * WHEEL_RADIUS;
	    dPos[2] = -(vel[0] * DT / (WHEEL_RADIUS + radius));
	} else {
	    // turning left
	    radius = vel[0] / (vel[1] - vel[0]) * WHEEL_RADIUS;
	    dPos[2] = vel[1] * DT / (WHEEL_RADIUS + radius);
	}
	float distance = 2 * (WHEEL_RADIUS + radius) * sin(fabs(dPos[2]) / 2.0);
	ePos[2] += dPos[2];
	dPos[0] = distance * cos(ePos[2]);
	dPos[1] = distance * sin(-ePos[2]);
    }
    ePos[0] += dPos[0];
    ePos[1] += dPos[1];
	    
    /* estimate sensor measurements */
    vel[0] = dPos[0] / DT;
    vel[1] = dPos[1] / DT;
    float eAcc[2] = { (vel[0] - bot.vel[0]) / DT, (vel[1] - bot.vel[1]) / DT };
    float eGyro = dPos[2] / 2.0;
    memcpy(bot.vel, vel, sizeof(float) * 2);
    
    /* fuse estimation with measurement */
    float K[3][3], inv_K[3][3];
    matrix_plus((float **)inv_K, (float **)eSigma, (float **)bot.R, 3);
    
    
    memcpy(bot.pos, ePos, sizeof(float) * 3);
}

void Bot_Controller() {
    // obtain error angle and distance
    bot.theta = 0.0;
    if (bot.inputPos[0] == bot.pos[0]) {
	bot.theta = (bot.pos[1] - bot.inputPos[1] < 0.0) ? -M_PI / 2 : M_PI / 2;
    }
    else bot.theta = atan2f(-(bot.inputPos[1] - bot.pos[1]), (bot.inputPos[0] - bot.pos[0]));
    if (bot.inputPos[0] - bot.pos[0] < 0) bot.theta = M_PI + bot.theta;
    bot.distance = sqrtf(powf(bot.inputPos[0] - bot.pos[0], 2) + powf(bot.inputPos[1] - bot.pos[1], 2));
    
    if (bot.distance > MIN_DIST) {
	float ePos[3] = {0};
	ePos[2] = bot.theta - bot.pos[2];
	bot.rotSign = (ePos[2] < 0.0) ? -1.0 : 1.0;
	bot.maxTurn = fabs(ePos[2]) / (3*(BETA + fabs(ePos[2])));
	float newDuty[2] = {0};

	if (fabs(ePos[2]) > ERROR_MAX) {
	    newDuty[0] = bot.rotSign * -bot.maxTurn;
	    newDuty[1] = bot.rotSign * bot.maxTurn;
	} else {
	    bot.maxSpeed = bot.distance / (ALPHA + bot.distance);
	    newDuty[0] = bot.maxSpeed / 2.0 + bot.rotSign * -bot.maxTurn;
	    newDuty[1] = bot.maxSpeed / 2.0 + bot.rotSign * bot.maxTurn;
	}

	for (int i = 0; i < 2; i++) {
	    /* set new duty cycles */
	    if (newDuty[i] < bot.duty[i] - GAMMA) bot.duty[i] -= GAMMA;
	    else if (newDuty[i] > bot.duty[i] + GAMMA) bot.duty[i] += GAMMA;
	    //else if (fabs(newDuty[i]) < MIN_PWM) bot.duty[i] = 0.0;
	    else bot.duty[i] = newDuty[i];
	}

    	
	/* negative duty cycles should reverse rotation */
	if (bot.duty[0] > 0) LATACLR = _LATA_LATA3_MASK;
	else LATASET = _LATA_LATA3_MASK;
	if (bot.duty[1] > 0) LATBCLR = _LATB_LATB4_MASK;
	else LATBSET = _LATB_LATB4_MASK;
    } else {
	bot.duty[0] = 0.0;
	bot.duty[1] = 0.0;
    }
    
    OC4RS = (int)(fabs(bot.duty[0]) * PWM_H);
    OC5RS = (int)(fabs(bot.duty[1]) * PWM_H);
}

void Bot_Display_Status() {
    snprintf(bot.buf, 100, "\33[2J\33[HBot status:\r\n");
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "pos: (%.2f, %.2f, %.2f)\r\n", bot.pos[0], bot.pos[1], bot.pos[2] * 180.0 / M_PI);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "input: (%.2f, %.2f)\r\n", bot.inputPos[0], bot.inputPos[1]);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "duty: (%.2f, %.2f)\r\n", bot.duty[0], bot.duty[1]);
    UART_Write_String(bot.buf, strlen(bot.buf));    
    snprintf(bot.buf, 100, "gyro: (%.2f), acc: (%.2f, %.2f)\r\n", bot.gyro, bot.acc[0], bot.acc[1]);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "theta: %.2f, distance: %.2f, maxTurn: %.2f, maxSpeed: %.2f, rotSign: %.2f\r\n", bot.theta * 180.0 / M_PI, bot.distance, bot.maxTurn, bot.maxSpeed, bot.rotSign);
    UART_Write_String(bot.buf, strlen(bot.buf));
}

/* Interrupt handlers */
void __ISR(_TIMER_3_VECTOR, IPL2SOFT) TMR3_IntHandler() {
    IFS0CLR = _IFS0_T3IF_MASK;
    Bot_Pos_Update();  
    Bot_Controller();
    Bot_Display_Status();
}

/* Helper function */
void matrix_plus(float ** result, float ** mat1, float ** mat2, int len) {
    for (int i = 0; i < len; i++)
	for (int j = 0; j < len; j++)
	    result[i][j] = mat1[i][j] + mat2[i][j];
}

void matrix_mul(float ** result, float ** mat1, float ** mat2, int len) {
    for (int i = 0; i < len; i++)
	for (int j = 0; j < len; j++) {
	    result[i][j] = 0.0;
	    for (int k = 0; k < len; k++) 
		result[i][j] += mat1[i][k] * mat2[k][j];
	}    
}

/* inverse of 3x3 matrix */
void matrix_inv(float ** result, float ** mat) {
    // calculate matrix of minors
    float minors[3][3], detMat[2][2];
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    // get determinant
	    int offsetK = 0;
	    for (int k = 0; k < 2; k++) {
		int offsetL = 0;
		for (int l = 0; l < 2; l++) {
		    if (k == i) offsetK = 1;
		    if (l == j) offsetL = 1;
		    detMat[k][l] = mat[i + offsetK][j + offsetL];
		}
	    }
	    minors[i][j] = detMat[0][0] * detMat[1][1] - detMat[0][1] * detMat[1][0];
	}
    }
    
    float det = mat[0][0] * minors[0][0] + mat[0][1] * minors[0][1] + mat[0][2] * minors[0][2];
    
    // cofactors and adjoint
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    if ((i + j) % 2 != 0) minors[i][j] *= -1;
	}
    }
    
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    result[i][j] = minors[j][i] / det;
	}
    }
}

/* Peripheral functions*/
void Init() {
    /* Random function */
    srand(PORTA);
    
    /* Ports */
    ANSELA = 0x0;
    TRISA = 0x0;
    LATA = 0x0;
    LATASET = _LATA_LATA0_MASK;

    ANSELB = 0x0;
    TRISB = 0x380;
    LATB = 0xC;
    LATBSET = _LATB_LATB10_MASK;

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
    T3CON = 0x8060;		// 1:32 prescaler = 1 MHz (reset to 0x8050)
    
    /* Long counter */
    TMR4 = 0x0;
    PR4 = 0xFFFF;		// 2 Hz
    T4CON = 0x70;		// 1:256 prescaler = 125 kHz
    
    /* I2C */
    I2C2BRG = (int)(PBCLK / (2 * I2C_BAUD) - 2);
    I2C2CON = _I2C2CON_ON_MASK;
    
    /* UART */
    U1MODE = 0x0;
    U1STA = 0x0;
    U1BRG = 51;//(int)(PBCLK / (16 * UART_BAUD) - 1);
    U1STASET = (_U1STA_UTXISEL0_MASK | _U1STA_URXEN_MASK | _U1STA_UTXEN_MASK);
    U1MODESET = _U1MODE_ON_MASK;
}

void UART_Write(char data) {
    while (U1STAbits.UTXBF);
    U1TXREG = data;
}

void UART_Write_String(char * data, int len) {
    while (len--) {
	UART_Write(*data);
	data++;
    }
}

void I2C_Master_Start() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.SEN = 1;
    while (I2C2CONbits.SEN);
    //snprintf(bot.buf, 100, "start: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Stop() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.PEN = 1;
    while (I2C2CONbits.PEN);
    //snprintf(bot.buf, 100, "stop: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Restart() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.RSEN = 1;
    while (I2C2CONbits.RSEN);
    //snprintf(bot.buf, 100, "restart: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Ack_Nack(int val) {
    // ACK = 0 (slave should send another byte)
    // NACK = 1 (no more bytes requested by slave)
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.ACKDT = val;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN);
    //snprintf(bot.buf, 100, "ack: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

int I2C_Master_Write(char byte) {
    I2C2TRN = byte;
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    //snprintf(bot.buf, 100, "write: %d (%d)\r\n", I2C2STAT, byte);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    if (I2C2STATbits.ACKSTAT) return 0;
    return 1;
}

char I2C_Master_Read() {
    I2C2CONbits.RCEN = 1;
    //snprintf(bot.buf, 100, "read: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    while (!I2C2STATbits.RBF);
    return I2C2RCV;
}

int I2C_Write(char periphAdd, char regAdd, char data) {
    int ret = 0;
    I2C_Master_Start();
    ret |= I2C_Master_Write((periphAdd << 1) | I2C_W);
    ret |= I2C_Master_Write(regAdd);
    ret |= I2C_Master_Write(data);
    I2C_Master_Stop();
    return ret;
}

int I2C_Read(char periphAdd, char regAdd, char * data, int len) {
    int ret = 0;
    I2C_Master_Start();
    ret |= I2C_Master_Write((periphAdd << 1) | I2C_W);
    ret |= I2C_Master_Write(regAdd);
    I2C_Master_Stop();
    I2C_Master_Start();
    I2C_Master_Write((periphAdd << 1) | I2C_R);
    
    for (int i = 0; i < len; i++) {
	data[i] = I2C_Master_Read();
	if (i != len - 1) I2C_Master_Ack_Nack(0);
    }
    I2C_Master_Ack_Nack(1);
    I2C_Master_Stop();
}

char IMU_Init() {
    int ret = 0;
    //snprintf(bot.buf, 100, "init: %d (%d)\r\n", I2C2STAT, PORTB);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    ret |= I2C_Write(IMU_ADD, SMPLRT_DIV, 0x07);
    ret |= I2C_Write(IMU_ADD, PWR_MGMT_1, 0x01);
    ret |= I2C_Write(IMU_ADD, CONFIG, 0x00);
    ret |= I2C_Write(IMU_ADD, ACCEL_CONFIG, 0x08);
    ret |= I2C_Write(IMU_ADD, GYRO_CONFIG, 0x08);
    ret |= I2C_Write(IMU_ADD, INT_ENABLE, 0x01);
    
    /* test everything is working */
    char whoami = 0;
    I2C_Read(IMU_ADD, WHO_AM_I, &whoami, 1);
    
    return whoami;
}
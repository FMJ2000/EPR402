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
#pragma config JTAGEN = ON              // JTAG Enable (JTAG Port Enabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#include <xc.h>
#include <proc/p32mx220f032b.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>

#define SYSCLK 64000000l
#define PBCLK 32000000l
#define I2C_BAUD 100000l
#define UART_BAUD 38400l
#define I2C_W 0x0
#define I2C_R 0x1
#define IMU_ADD 0x68

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

unsigned char letter = 'A';

void PORT_Init();
void INT_Init();
void TMR_Init();
void UART_Init();
void UART_Write(unsigned char data);
void UART_Write_String(unsigned char * data);
void I2C_Master_Init();
void I2C_Master_Start();
void I2C_Master_Stop();
void I2C_Master_Restart();
void I2C_Master_Idle();
void I2C_Master_Ack_Nack(int val);
int I2C_Master_Write(unsigned char byte);
unsigned char I2C_Master_Read();
int I2C_Read(unsigned char add, int len, unsigned char * data);
int I2C_Write(unsigned char add, int len, unsigned char * data);
int IMU_Init();

int main() {
	char msg[20];
	
	PORT_Init();
	INT_Init();
	TMR_Init();
	UART_Init();
	I2C_Master_Init();
	char ret = IMU_Init();
	snprintf(msg, 20, "Hello world: %d\r\n", ret);	
	UART_Write_String(msg);
	
	T3CONSET = _T3CON_ON_MASK;
	
	for(;;);
	return (EXIT_SUCCESS);
}

void PORT_Init() {
	ANSELA = 0x0;
	TRISA = 0x0;
	LATA = 0x0;	
	
	ANSELB = 0x0;
	TRISB = 0xC;
	//LATB = 0x0;
	
	/* I/O Port assignments */
	U1RXR = 0x3;		// RB13 = RX
	RPA2R = 0x5;		// RA2 = OC4
	RPA4R = 0x6;		// RA4 = OC5
	RPB15R = 0x1;		// RB15 = TX
}

/* Interrupt function definitions */
void INT_Init() {
	__builtin_disable_interrupts();
	INTCONSET = _INTCON_MVEC_MASK;
	
	/* Priority */
	//IPC2 = 0xA;			// TMR2
	IPC3 = 0xA;			// TMR3
	//IPC4 = 0xA0000;		// OC4
	//IPC5 = 0xA000A;		// OC5
	
	/* Status reset */
	IFS0 = 0x0;
	IFS1 = 0x0;
	
	/* Control */
	//IEC0 = 0x8404200;		// OC5, OC4, TMR2
	IEC0 = 0x4000;			// TMR3
	
	__builtin_enable_interrupts();
}

void TMR_Init() {
	/* Timer 2 */
	/*T2CON = 0x60;		// 1:64 prescaler = 500 kHz
	TMR2 = 0x0;
	PR2 = PWM_T;*/
	
	/* Timer 3 */
	T3CON = 0x70;		// 1:256 prescaler = 125 kHz
	TMR3 = 0x0;
	PR3 = 0xFFFF;
}

void UART_Init() {
	U1MODE = 0x0;
	U1STA = 0x0;
	U1BRG = (int)(PBCLK / (16 * UART_BAUD) - 1);
	U1STASET = (_U1STA_UTXISEL0_MASK | _U1STA_URXEN_MASK | _U1STA_UTXEN_MASK);
	U1MODESET = _U1MODE_ON_MASK;	
}

void UART_Write(unsigned char data) {
	while (U1STAbits.UTXBF);
	U1TXREG = data;
}

void UART_Write_String(unsigned char * data) {
	while (*data) UART_Write(*data++);
}

void I2C_Master_Init() {
	/*I2C2CON = 0x0;
	I2C2STAT = 0x0;*/
	I2C2BRG = (int)(PBCLK / (2 * I2C_BAUD) - 2);
	I2C2CONSET = _I2C2CON_ON_MASK;
}

void I2C_Master_Start() {
	I2C_Master_Idle();
	I2C2CONbits.SEN = 1;
	while (I2C2CONbits.SEN);
}

void I2C_Master_Stop() {
	I2C_Master_Idle();
	I2C2CONbits.PEN = 1;
	while (I2C2CONbits.PEN);
}

void I2C_Master_Restart() {
	I2C_Master_Idle();
	I2C2CONbits.RSEN = 1;
	while (I2C2CONbits.RSEN);
}

void I2C_Master_Idle() {
	while (I2C2CON & 0x1F);
	while (I2C2STATbits.TRSTAT);
}

void I2C_Master_Ack_Nack(int val) {
	// ACK = 0 (slave should send another byte)
	// NACK = 1 (no more bytes requested by slave)
	I2C_Master_Idle();
	I2C2CONbits.ACKDT = val;
	I2C2CONbits.ACKEN = 1;
	while (I2C2CONbits.ACKEN);
}

int I2C_Master_Write(unsigned char byte) {
	char msg[20];
	I2C2TRN = byte;
	while (I2C2STATbits.TRSTAT);
	snprintf(msg, 20, "ACKSTAT: %d (%d)\r\n", I2C2STATbits.ACKSTAT, I2C2TRN);
	//UART_Write_String(msg);
	if (I2C2STATbits.ACKSTAT) return 0;
	return 1;
}

unsigned char I2C_Master_Read() {
	char msg[20];
	I2C2CONbits.RCEN = 1;
	while (!I2C2STATbits.RBF);
	snprintf(msg, 20, "READ: %d\r\n", I2C2RCV);
	//UART_Write_String(msg);
	return I2C2RCV;
}

int I2C_Read(unsigned char add, int len, unsigned char * data) {
	I2C_Master_Start();
	I2C_Master_Write((add << 1) | I2C_R);
	for (int i = 0; i < len; i++) {
		if (!I2C_Master_Read(*data++)) return 0;
		I2C_Master_Ack_Nack(i == len - 1);
	}
	I2C_Master_Stop();
	return 1;
}

int I2C_Write(unsigned char add, int len, unsigned char * data) {
	I2C_Master_Start();
	if (!I2C_Master_Write((add << 1) | I2C_W)) return 0;
	for (int i = 0; i < len; i++) {
		if (!I2C_Master_Write(*data++)) return 0;
	}
	I2C_Master_Stop();
	return 1;
}

int IMU_Init() {
	int ret = 0;
	I2C_Master_Start();
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	ret = I2C_Master_Write(SMPLRT_DIV);
	ret = I2C_Master_Write(0x07);
	I2C_Master_Stop();
	
	I2C_Master_Start();
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	ret = I2C_Master_Write(PWR_MGMT_1);
	ret = I2C_Master_Write(0x01);
	I2C_Master_Stop();
	
	I2C_Master_Start();
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	ret = I2C_Master_Write(CONFIG);
	ret = I2C_Master_Write(0x00);
	I2C_Master_Stop();
	
	I2C_Master_Start();
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	ret = I2C_Master_Write(ACCEL_CONFIG);
	ret = I2C_Master_Write(0x08);
	I2C_Master_Stop();
	
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	ret = I2C_Master_Write(GYRO_CONFIG);
	ret = I2C_Master_Write(0x08);
	I2C_Master_Stop();
	
	I2C_Master_Start();
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	ret = I2C_Master_Write(INT_ENABLE);
	ret = I2C_Master_Write(0x01);
	I2C_Master_Stop();
	
	I2C_Master_Start();
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	ret = I2C_Master_Write(WHO_AM_I);
	I2C_Master_Stop();
	
	I2C_Master_Start();
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_R);
	ret = I2C_Master_Read();
	I2C_Master_Ack_Nack(1);
	I2C_Master_Stop();
	
	return ret;
}

int IMU_Read(int data[7]) {
	I2C_Master_Start();
	I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	I2C_Master_Write(ACCEL_XOUT_H);
	I2C_Master_Stop();
	
	I2C_Master_Start();
	I2C_Master_Write((IMU_ADD << 1) | I2C_R);
	for (int i = 0; i < 7; i++) {
		char high = I2C_Master_Read();
		I2C_Master_Ack_Nack(0);
		char low = I2C_Master_Read();
		if (i != 6) I2C_Master_Ack_Nack(0);
		else I2C_Master_Ack_Nack(1);
		data[i] = ((int)high << 8) | (int)low;
	}
	I2C_Master_Stop();
	
	return 1;
}

/*
int I2C_Master_Write(unsigned char add, unsigned char * data) {
	while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
	I2C2CONSET = _I2C2CON_SEN_MASK;
	while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
	I2C2TRN = (add << 1) | I2C_W;
	while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
	while (*data) {
		if (I2C2STATbits.ACKSTAT) return 1;
		while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
		I2C2TRN = *data++;
	}
	while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
	if (I2C2STATbits.ACKSTAT) return 1;
	I2C2CONSET = _I2C2CON_PEN_MASK;
	//LATAINV = 0x1;
	return 0;
}

int I2C_Master_Read(unsigned char add, unsigned char * data, int len) {
	while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
	I2C2CONSET = _I2C2CON_SEN_MASK;
	while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
	I2C2TRN = (add << 1) | I2C_R;
	while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
	if (I2C2STATbits.ACKSTAT) return 1;
	for (int i = 0; i < len; i++) {
		I2C2CONSET = _I2C2CON_RCEN_MASK;
		while (!I2C2STATbits.RBF);
		//IFS1CLR = _IFS1_I2C2MIF_MASK;
		data[i] = I2C2RCV;
		while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
		if (i != len - 1) I2C2CONCLR = _I2C2CON_ACKDT_MASK;
		else I2C2CONSET = _I2C2CON_ACKDT_MASK;
		I2C2CONSET = _I2C2CON_ACKEN_MASK;
	}
	while (I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
	I2C2CONSET = _I2C2CON_PEN_MASK;
	return 0;
}
*/

void __ISR (_TIMER_3_VECTOR, IPL2SOFT) TMR3_IntHandler() {
	LATAINV = 0x1;
	IFS0CLR = _IFS0_T3IF_MASK;
	int data[7];
	char msg[80];
	int ret = IMU_Read(data);
	snprintf(msg, 80, "ax: %d, ay: %d, az: %d, t: %d, gx: %d, gy: %d, gz: %d \r\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
	UART_Write_String(msg);
	//letter = (letter == 'Z') ? 'A' : letter + 1;
} 
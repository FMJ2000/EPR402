#include "periph.h"
#include "main.h"

void PORT_Init() {
	ANSELA = 0x0;
	TRISA = 0x1;
	LATA = 0x0;	
	
	ANSELB = 0x0;
	TRISB = 0x38C;
	LATB = 0x0;
	LATBSET = _LATB_LATB10_MASK;		// on button
	
	/* I/O Port assignments */
	U1RXR = 0x3;		// RB13 = RX
	RPA2R = 0x5;		// RA2 = OC4
	RPA4R = 0x6;		// RA4 = OC5
	RPB15R = 0x1;		// RB15 = TX
	INT1R = 0x4;		// RB9 = INT1
	INT3R = 0x4;		// RB8 = INT3
	INT4R = 0x0;		// RA0 = INT4
}

/* Interrupt function definitions */
void INT_Init() {
	__builtin_disable_interrupts();
	INTCONSET = _INTCON_MVEC_MASK;
	
	/* Priority */
	IPC0 = 0xA000000;		// INT0
	IPC1 = 0xA000000;		// INT1
	IPC3 = 0xA00000A;		// INT3, TMR3
	IPC4 = 0xA000000;		// INT4
	//IPC2 = 0xA;			// TMR2
	IPC4 = 0xA0000;		// OC4
	IPC5 = 0xA000A;		// OC5
	
	/* Status reset */
	IFS0 = 0x0;
	IFS1 = 0x0;
	
	/* Control */
	//IEC0 = 0x8404200;		// OC5, OC4, TMR2
	IEC0 = (_IEC0_T3IE_MASK | _IEC0_INT0IE_MASK | _IEC0_INT1IE_MASK | _IEC0_INT3IE_MASK | _IEC0_INT4IE_MASK | _IEC0_OC4IE_MASK | _IEC0_OC5IE_MASK);			// TMR3
	__builtin_enable_interrupts();
}

void TMR_Init() {
	/* Timer 2 */
	/*T2CON = 0x60;		// 1:64 prescaler = 500 kHz
	TMR2 = 0x0;
	PR2 = PWM_T;*/
	
	/* Timer 3 */
	T3CON = 0x50;		// 1:32 prescaler = 1 MHz
	TMR3 = 0x0;
	PR3 = 0xFFFF;		// f = 15 Hz
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
	ret = I2C_Master_Write(0x08);			// 4g
	I2C_Master_Stop();
	
	ret = I2C_Master_Write((IMU_ADD << 1) | I2C_W);
	ret = I2C_Master_Write(GYRO_CONFIG);
	ret = I2C_Master_Write(0x08);			// 500 dps
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

int IMU_Read(int * data) {
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

int Slave_Read(char * data) {
	/* first 8 bytes = 4 ultrasonic measurements */
	I2C_Master_Start();
	I2C_Master_Write((SLAVE_ADD << 1) | I2C_R);
	for (int i = 0; i < SLAVE_MSG_LEN; i++) {
		data[i] = I2C_Master_Read();
		if (i != SLAVE_MSG_LEN - 1) I2C_Master_Ack_Nack(0);
		else I2C_Master_Ack_Nack(1);
	}
	I2C_Master_Stop();
	return 1;
}

void Slave_Interpret(unsigned char * input, float * output) {
	unsigned int val[4];
	for (int i = 0; i < 4; i++) {
		val[i] = (unsigned int)((input[2*i] << 8) | input[2*i+1]);
		output[i] = (float)(val[i] * SOUND_SPEED / 1000000.0);
	}
}
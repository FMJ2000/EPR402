#include "periph.h"

void PORT_Init() {	
	ANSELA = 0x1;
	TRISA = 0x11;
	LATA = 0x0;	
	
	ANSELB = 0x0;
	TRISB = 0x89C;
	LATB = 0x0;
	
	/* I/O Port assignments */
	T2CKR = 0x2;			// RB4 = T2CK
	T3CKR = 0x3;			// RB11 = T3CK
	T4CKR = 0x2;			// RA4 = T4CK
}

void INT_Init() {
	__builtin_disable_interrupts();
	INTCONSET = _INTCON_MVEC_MASK;
	
	/* Priority */
	IPC0 = 0xA000000;		// INT0
	IPC2 = 0xA;				// TMR2
	IPC5 = 0xA00000A;		// TMR5, ADC1
	IPC9 = 0xA0000;			// I2C2
	
	/* Status reset */
	IFS0 = 0x0;
	IFS1 = 0x0;
	
	/* Control */
	IEC0 = (_IEC0_T2IE_MASK | _IEC0_T5IE_MASK | _IEC0_AD1IE_MASK | _IEC0_INT0IE_MASK);
	IEC1 = (_IEC1_I2C2BIE_MASK | _IEC1_I2C2MIE_MASK | _IEC1_I2C2SIE_MASK);
	
	__builtin_enable_interrupts();
}

void TMR_Init() {
	/* Ultrasonic echo timer */
	T2CON = 0xD0;				// gated time, 1:32 prescaler, TMR2 f = 1 MHz
	TMR2 = 0x0;
	PR2 = 0xFFFF;
	
	/* Odometer count timers */
	TMR3 = 0x0;
	T3CON = 0x2;				// external clock source, 1:1 prescaler
	TMR4 = 0x0;
	T4CON = 0x2;				// external clock source, 1:1 prescaler
	
	/* Sample rate */
	T5CON = 0x30;				// 1:8 prescaler, TMR2 f = 4 MHz
	PR5 = 0xFFFF;				// int f = 61 Hz
}

void ADC_Init() {
	AD1CON1 = 0x2E0;			// fractional output, manual sampling
	AD1CON2 = 0x0;				// no scan, single conversion per interrupt
	AD1CON3 = 0xF03;			// 15 Tad sample, 250ns conversion
	AD1CHS = 0x0;				// AN0 - Vrefl
	AD1CSSL = 0x0;				// no inputs scanned
	
	AD1CON1SET = _AD1CON1_ADON_MASK;
	AD1CON1SET = _AD1CON1_SAMP_MASK;
}

void I2C_Init() {
	I2C1CON = 0x0;
	I2C2CON = 0x0;
	I2C2CONSET = _I2C2CON_SCLREL_MASK;
	I2C2STAT = 0x0;
	I2C2ADD = SLAVE_ADD;
	I2C2MSK = 0x0;
	I2C2BRG = (int)(PBCLK / (2 * I2C_BAUD) - 2);
	I2C2CONSET = _I2C2CON_ON_MASK;
}


/* 
 * File:   main.c in slave
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

#include "main.h"
#include "slave.h"

static Slave slave = { .usState = 0 };

int main() {
	PORT_Init();
	INT_Init();
	TMR_Init();
	ADC_Init();
	I2C_Init();
	
	T2CONSET = _T2CON_ON_MASK;
	T3CONSET = _T3CON_ON_MASK;
	T4CONSET = _T4CON_ON_MASK;
	T5CONSET = _T5CON_ON_MASK;
	
	for (;;);
	return (EXIT_SUCCESS);
}

int I2C_Slave_Write() {
	slave.data[8] = (unsigned char) TMR3;
	slave.data[9] = (unsigned char) TMR4;
	I2C2TRN = slave.data[slave.i2cIndex];
	I2C2CONbits.SCLREL = 1;
	while (I2C2STATbits.TBF);
	if (slave.i2cIndex == 11) slave.data[slave.i2cIndex] = 0x0;
	if (I2C2STATbits.ACKSTAT) {
		slave.i2cIndex = 0;
		return 0;
	}
	slave.i2cIndex++;
	return 1;
}

void __ISR(_I2C_2_VECTOR, IPL2SOFT) I2C_IntHandler() {
	unsigned char recv;
	if (IFS1bits.I2C2MIF) {
		IFS1CLR = _IFS1_I2C2MIF_MASK;
	}
	if (IFS1bits.I2C2BIF) {
		IFS1CLR = _IFS1_I2C2BIF_MASK;
	}
	if (IFS1bits.I2C2SIF) {
		IFS1CLR = _IFS1_I2C2SIF_MASK;
		if (I2C2STATbits.R_W == 0 && I2C2STATbits.D_A == 0) {
			/* address + write */
			I2C2CONbits.SCLREL = 1;
		} else if (I2C2STATbits.R_W == 0 && I2C2STATbits.D_A == 1) {
			/* data + write */
			recv = I2C2RCV;
			I2C2CONbits.SCLREL = 1;
		} else if (I2C2STATbits.R_W == 1 && I2C2STATbits.D_A == 0) {
			/* address + read */
			recv = I2C2RCV;
			I2C_Slave_Write();
		} else if (I2C2STATbits.R_W == 1 && I2C2STATbits.D_A == 1) {
			/* data + read */
			recv = I2C2RCV;
			I2C_Slave_Write();
		} 
	}
}

void __ISR(_TIMER_2_VECTOR, IPL2SOFT) TMR2_IntHandler() {
	IFS0CLR = _IFS0_T2IF_MASK;
	unsigned int duration = TMR2;
	slave.data[2*slave.usState+1] = duration & 0xFF;
	slave.data[2*slave.usState] = (duration >> 8) & 0xFF;
}

void __ISR(_TIMER_5_VECTOR, IPL2SOFT) TMR5_IntHandler() {
	LATBINV = _LATB_LATB8_MASK;
	IFS0CLR = _IFS0_T5IF_MASK;
	slave.usState = (slave.usState + 1) % US_SENSORS;
	Read_Ultrasonic(&slave);
	slave.adcCounter++;
	if (slave.adcCounter == F_SAMPLE) {
		TMR3 = 0x0;
		TMR4 = 0x0;
		AD1CON1SET = _AD1CON1_SAMP_MASK;	// start battery sample
		slave.adcCounter = 0;
	}
}

void __ISR(_ADC_VECTOR, IPL2SOFT) ADC_IntHandler() {
	if (IFS0bits.AD1IF) {
		IFS0CLR = _IFS0_AD1IF_MASK;
		slave.data[10] = (unsigned char)((ADC1BUF0 >> 8) & 0xFF);
	}
}

void __ISR(_EXTERNAL_0_VECTOR, IPL2SOFT) Ext0_IntHandler() {
	IFS0CLR = _IFS0_INT0IF_MASK;
	slave.data[11] = 0x1;
}
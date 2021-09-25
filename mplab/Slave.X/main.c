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
#define I2C_ADD 0x78

void PORT_Init();
void INT_Init();
void UART_Init();
void UART_Write(unsigned char data);
void UART_Write_String(unsigned char * data);
void I2C_Init();
int I2C_Slave_Write(unsigned char add, unsigned char * data);
int I2C_Slave_Read(unsigned char add, unsigned char * data, int len);

int main() {
	PORT_Init();
	INT_Init();
	UART_Init();
	I2C_Init();
	
	UART_Write_String("Initialize Slave\r\n");
	
	for (;;) {
		
	}
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

void INT_Init() {
	__builtin_disable_interrupts();
	INTCONSET = _INTCON_MVEC_MASK;
	
	/* Priority */
	IPC9 = 0xA00;		// I2C2
	
	/* Status reset */
	IFS1 = 0x0;
	
	/* Control */
	IEC1 = 0x7000000;		// I2CM, I2CS, I2CB
	
	__builtin_enable_interrupts();
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

void I2C_Init() {
	I2C2CON = 0x0;
	I2C2CONSET = (_I2C2CON_SCLREL_MASK | _I2C2CON_STRICT_MASK);
	I2C2STAT = 0x0;
	I2C2ADD = 0x10;
	I2C2BRG = (int)(PBCLK / (2 * I2C_BAUD) - 2);
	I2C2CONSET = _I2C2CON_ON_MASK;
}

void __ISR(_I2C_2_VECTOR, IPL2SOFT) I2C_IntHandler() {
	UART_Write_String("Interrupt\r\n");
	unsigned char data;
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
			data = I2C2RCV;
			I2C2CONbits.SCLREL = 1;
			LATAINV = 0x1;
			UART_Write(data);
		}
	}
}
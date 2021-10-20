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
#define SLAVE_MSG_LEN 12                // length of I2C message (ultrasonic: 8, odometer: 2, adc: 1, user: 1)
#define F_SAMPLE 61
#define US_SENSORS 3
#define US_TRIG_T 640                   // ultrasonic trigger

/* Bot object that contains state */
typedef struct Bot {
    float pos[2];
    float rotMu;
    
    unsigned short i2cIndex;            // current byte read out to I2C master
    unsigned short usState;             // ultrasonic sensor
    unsigned short prevState;		// previous ultrasonic sensor state
    unsigned short usBufIndex;          // ultrasonic buffer index
    unsigned short adcCounter;          // counter until next ADC reading
    unsigned char data[SLAVE_MSG_LEN];  // ultrasonic[0:8], odometer[8:10], battery[10], user[11] readings
} Bot;

static Bot bot = { .prevState = 3 };

/* Peripheral functions */
void Init();

/* Bot functions */
void Bot_Trigger_Ultrasonic();
void Bot_I2C_Write();

int main() {
    Init();
    
    Bot_Trigger_Ultrasonic();
    for (;;);
    return EXIT_SUCCESS;
}

void Bot_Trigger_Ultrasonic() {    
    TMR2 = 0x0;
    /* enable HC-SR04 trigger on current state for 10us */
    long delay = SYSCLK / (2 * US_TRIG_T);
    switch (bot.usState) {
	case 0: 
	    LATASET = _LATA_LATA2_MASK;
	    while (delay--);
	    LATACLR = _LATA_LATA2_MASK;
	    break;
	case 1: 
	    LATASET = _LATA_LATA3_MASK;
	    while (delay--);
	    LATACLR = _LATA_LATA3_MASK;
	    break;
	case 2:
	    LATBSET = _LATB_LATB14_MASK;
	    while (delay--);
	    LATBCLR = _LATB_LATB14_MASK;
	    break;
	case 3:
	    LATBSET = _LATB_LATB10_MASK;
	    while (delay--);
	    LATBCLR = _LATB_LATB10_MASK;
	    break;
    }
}

void Bot_I2C_Write() {
    bot.data[8] = (unsigned char) TMR3;
    bot.data[9] = (unsigned char) TMR4;
    I2C2TRN = bot.data[bot.i2cIndex];
    I2C2CONbits.SCLREL = 1;
    if (bot.i2cIndex == SLAVE_MSG_LEN - 1) bot.data[bot.i2cIndex] = 0x0;
    while (I2C2STATbits.TBF);
    if (I2C2STATbits.ACKSTAT) bot.i2cIndex = 0;
    else bot.i2cIndex++;
}

void Init() {
    /* Ports */
    ANSELA = 0x1;
    TRISA = 0x11;
    LATA = 0x0;	

    ANSELB = 0x0;
    TRISB = 0x89C;
    LATB = 0x0;

    T2CKR = 0x2;			// RB4 = T2CK
    T3CKR = 0x3;			// RB11 = T3CK
    T4CKR = 0x2;			// RA4 = T4CK

    /* Interrupts */
    __builtin_disable_interrupts();
    INTCONSET = _INTCON_MVEC_MASK;
    IPC0 = 0xA000000;		// INT0
    IPC2 = 0xA;				// TMR2
    IPC5 = 0xA00000A;		// TMR5, ADC1
    IPC9 = 0xA0000;			// I2C2
    IFS0 = 0x0;
    IFS1 = 0x0;
    IEC0 = 0x11000208;      // INT0, TMR2, TMR5, ADC1
    IEC1 = 0x7000000;       // I2C2B, I2C2M, I2C2S
    __builtin_enable_interrupts();
    
    /* Ultrasonic */
    TMR2 = 0x0;
    PR2 = 0xFFFF;           // f = 15 Hz
    T2CON = 0x80F0;         // gated time, 1:256 prescaler, TMR2 f = 125 kHz
    
    /* Odometers */
    TMR3 = 0x0;
    T3CON = 0x8020;         // external clock source, 1:1 prescaler
    TMR4 = 0x0;
    T4CON = 0x8020;         // external clock source, 1:1 prescaler
    
    /* Sample timer */
    TMR5 = 0x0;
    PR5 = 0xFFFF;           // int f = 15 Hz
    T5CON = 0x8050;         // 1:32 prescaler, TMR5 f = 1 MHz
    
    /* ADC */
    AD1CON1 = 0x2E0;			// fractional output, manual sampling
    AD1CON2 = 0x0;				// no scan, single conversion per interrupt
    AD1CON3 = 0xF03;			// 15 Tad sample, 250ns conversion
    AD1CHS = 0x0;				// AN0 - Vrefl
    AD1CSSL = 0x0;				// no inputs scanned
    AD1CON1SET = _AD1CON1_ADON_MASK;
    AD1CON1SET = _AD1CON1_SAMP_MASK;
    
    /* I2C */	
    I2C1CON = 0x0;
    I2C2CON = 0x0;
    I2C2CONSET = _I2C2CON_SCLREL_MASK;
    I2C2STAT = 0x0;
    I2C2ADD = SLAVE_ADD;
    I2C2MSK = 0x0;
    I2C2BRG = (int)(PBCLK / (2 * I2C_BAUD) - 2);
    I2C2CONSET = _I2C2CON_ON_MASK;
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
	    Bot_I2C_Write();
	} else if (I2C2STATbits.R_W == 1 && I2C2STATbits.D_A == 1) {
	    /* data + read */
	    recv = I2C2RCV;
	    Bot_I2C_Write();
	} 
    }
}

void __ISR(_TIMER_2_VECTOR, IPL2SOFT) TMR2_IntHandler() {
    IFS0CLR = _IFS0_T2IF_MASK;
    unsigned int duration = TMR2;
    bot.data[2*bot.usState] = (duration >> 8) & 0xFF;
    bot.data[2*bot.usState+1] = duration & 0xFF;
    bot.usState = (bot.usState + 1) % US_SENSORS;
    Bot_Trigger_Ultrasonic();
}

void __ISR(_TIMER_5_VECTOR, IPL2SOFT) TMR5_IntHandler() {
    LATBINV = _LATB_LATB8_MASK;
    IFS0CLR = _IFS0_T5IF_MASK;

    bot.adcCounter++;
    if (bot.adcCounter == F_SAMPLE) {
	TMR3 = 0x0;
	TMR4 = 0x0;
	AD1CON1SET = _AD1CON1_SAMP_MASK;	// start battery sample
	bot.adcCounter = 0;
    }
}

void __ISR(_ADC_VECTOR, IPL2SOFT) ADC_IntHandler() {
    if (IFS0bits.AD1IF) {
	IFS0CLR = _IFS0_AD1IF_MASK;
	bot.data[10] = (unsigned char)((ADC1BUF0 >> 8) & 0xFF);
    }
}

void __ISR(_EXTERNAL_0_VECTOR, IPL2SOFT) Ext0_IntHandler() {
    IFS0CLR = _IFS0_INT0IF_MASK;
    bot.data[11] = 0x1;
}

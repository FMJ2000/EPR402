/* 
 * File:   main.c in master
 * Author: martin
 *
 * Created on 23 September 2021, 2:51 PM
 */

/* Bluetooth setup code:
 * sudo killall rfcomm
 * sudo rfcomm connect /dev/rfcomm0 98:DA:D0:00:5E:C4 1 &
 * sudo minicom -D /dev/rfcomm0
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
#pragma config UPLLIDIV = DIV_3        // USB PLL Input Divider (3 Divider)
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
	delay(100000l);
	
	// bot init
	float startPos[3] = {0};
	Bot_Init(&bot, startPos);
	
	// peripheral init
	uint8_t whoami[2] = {0};
	Init(bot->buf);
	IMU_Init(whoami);
	OLED_Init();
	
	OLED_ClearDisplay();
	OLED_Write_Text(0, 0, "imu %d, mag %d", whoami[0], whoami[1]);
	OLED_Update();
	
	Bot_UART_Write(bot, "Bot init...\r\n\n");

	for (;;);
	return EXIT_SUCCESS;
}

void SYS_Unlock() {
    SYSKEY = 0x0; // write invalid key to force lock
    SYSKEY = 0xAA996655; // Write Key1 to SYSKEY
    SYSKEY = 0x556699AA; // Write Key2 to SYSKEY
}

void SYS_Lock() {
    SYSKEY = 0x0; 
}

/* Interrupt functions */

/* Sample timer */
void __ISR(_TIMER_1_VECTOR, IPL2SOFT) TMR1_IntHandler() {
	IFS0CLR = _IFS0_T1IF_MASK;
	bot->count++;
	
	Bot_Pos_IMU(bot);		// imu pos update @ 40 Hz
	Bot_Pos_Control(bot);
	
	//Bot_UART_Status(bot);
	
	if (bot->count % 5 == 0) {
		Bot_Pos_Odo(bot);		// odo pos update @ 4 Hz
		Bot_Display_Status(bot);
		
		//Bot_Display_Map(bot);
	}
	
	if (bot->count % 10 == 0) {
		Ultrasonic_Trigger();	// distance reading @ 2 Hz
		
	}

	if (bot-> count % FREQ == 0) {
		bot->time++;
		bot->count = 0;
		bot->dblClickCount = (bot->time % 2) ? 0 : bot->dblClickCount;
		AD1CON1SET = _AD1CON1_SAMP_MASK;
		

		if (bot->time == 3) {
			for (uint8_t i = 0; i < 3; i++) bot->bias[i] /= bot->numBias;
			bot->bias[2] *= M_PI / 180.0;
			bot->state = (bot->state & ~STATE_MASK) | NAVIGATE;
		}
	}
}

/* Ultrasonic echo timer */
void __ISR(_TIMER_5_VECTOR, IPL3SOFT) TMR5_IntHandler() {
	IFS0CLR = _IFS0_T5IF_MASK;
	bot->dist[bot->usState++] = TMR5 * SOUND_SPEED;
	bot->usCount++;
	TMR5 = 0;

	// check if all readings taken, update map 
	if (bot->usState == US_SENSORS) {
		bot->usState = 0;
		Bot_Map_Update(bot);
	}
}

/* Battery sampler */
void __ISR(_ADC_VECTOR, IPL2SOFT) ADC_IntHandler() {
	if (IFS0bits.AD1IF) {
		IFS0CLR = _IFS0_AD1IF_MASK;
		bot->battery = (((ADC1BUF0 >> 8) & 0xFF) - ADC_MIN) * ADC_SCALE;
	}
}

/* User */
void __ISR(_CHANGE_NOTICE_VECTOR, IPL2SOFT) CNB_IntHandler() {
	IFS1CLR = _IFS1_CNBIF_MASK;
	if (bot->portCN && !(PORTB & 0x20)) {
		/* button press occured */
	    //if (bot->dblClickCount == 0) {
		//bot->dblClickCount++;
		char newState = ((bot->state & STATE_MASK) == IDLE) ? NAVIGATE : IDLE;
		bot->state = (bot->state & ~STATE_MASK) | newState;
		bot->portCN = 1;
		return;
	    //}
	    
	    free(bot);
	    SYS_Unlock();
	    RSWRSTSET = 1;
	    volatile int * p = &RSWRST;
	    *p;
	    while (1);
	}
}

void __ISR(_DMA1_VECTOR, IPL2SOFT) DMA_IntHandler() {
	if (DCH1INT & _DCH1INT_CHBCIF_MASK) DCH1INTCLR = _DCH1INT_CHBCIF_MASK;
	IFS1CLR = _IFS1_DMA1IF_MASK;
}

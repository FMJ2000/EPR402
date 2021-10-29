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

#include <math.h>

#include "master.h"

int main() {
    srand(PORTB);
    delay(100000);
    
    Master_Init();
    Init(bot);
    UART_Write_String("Bot Initialized...\r\n", 21);
    UART_Write_String("Peripherals Initialized\r\n", 26);
    uint8_t whoami[2] = {0};
    IMU_Init(bot, whoami);
    
    /*OLED_Init();
    OLED_ClearDisplay();
    OLED_Write_Text(0, 0, "Hello world 2!");
    OLED_Update();*/
     
    snprintf(bot->buf, 100, "IMU_ADD: %d, MAG_ADD: %d\r\n", whoami[0], whoami[1]);
    UART_Write_String(bot->buf, strlen(bot->buf));
    
    for (;;);
    return EXIT_SUCCESS;
}

void Master_Init() {
    bot = malloc(sizeof(struct Bot));
    bot->state = 0;
    bot->uartState = 1;		// .uartState=0 in prod
    float startMap[2] = { -MAP_SIZE / 2, MAP_SIZE / 2 };
    BitMap_Initialize(&bot->currentMap, startMap);
    Bot_Map_Required(bot);
}

/* Interrupt functions */

/* Ultrasonic echo timer */
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) TMR2_IntHandler() {
    IFS0CLR = _IFS0_T2IF_MASK;
}

/* Sample timer */
void __ISR(_TIMER_4_VECTOR, IPL2SOFT) TMR4_IntHandler() {
    IFS0CLR = _IFS0_T4IF_MASK;
    
    Bot_Pos_Update(bot);
    if (bot->count % 5) Trigger_Ultrasonic(bot);
    //Bot_Controller();
    
    bot->count = (bot->count + 1) % 10;//(int)FREQ;
    if (bot->count == 0) {
	bot->time++;
	AD1CON1SET = _AD1CON1_SAMP_MASK;
	if (bot->uartState) Bot_Display_Status(bot);
	/*Bot_Display_BitMap(bot.currentMap); */
	/*for (char i = 0; i < 4; i++)
	    if (ProbMap_Contains(bot.localMaps[i], bot.pos))
		Bot_Display_ProbMap(bot.localMaps[i]);*/
    }
}

void __ISR(_TIMER_5_VECTOR, IPL2SOFT) TMR5_IntHandler() {
    IFS0CLR = _IFS0_T5IF_MASK;
    bot->distances[bot->usState] = (float)TMR5 * SOUND_SPEED;
    TMR5 = 0;
    if (bot->distances[bot->usState] > MAX_US_DIST || bot->distances[bot->usState] < MIN_US_DIST) bot->distances[bot->usState] = MAX_US_DIST;
    
    /* check if all readings taken, update map */
    bot->usState = (bot->usState + 1) % US_SENSORS;
    //if (bot.usState == 0) Bot_Map_Update();
}

/* Battery sampler */
void __ISR(_ADC_VECTOR, IPL2SOFT) ADC_IntHandler() {
    if (IFS0bits.AD1IF) {
	IFS0CLR = _IFS0_AD1IF_MASK;
	bot->battery = (((ADC1BUF0 >> 8) & 0xFF) - ADC_OFFSET) * ADC_SCALE;
    }
}

/* Fall sensor */
void __ISR(_EXTERNAL_3_VECTOR, IPL2SOFT) Ext3_IntHandler() {
    IFS0CLR = _IFS0_INT3IF_MASK;
}

/* User */
void __ISR(_CHANGE_NOTICE_VECTOR, IPL2SOFT) CNB_IntHandler() {
    if ((bot->portCN & 0x20) && !(PORTB & 0x20)) {
	/* button press occured */
	bot->uartState ^= 1;
	
	switch (bot->uartState) {
	    case 0: {
		U1MODECLR = _U1MODE_ON_MASK;
		TRISASET = 0x10;
		TRISBSET = 0x10;
		U1RXR = 0x0;		// RA4 = RX
		RPA2R = 0x5;		// RA2 = OC4
		RPA4R = 0x6;		// RA4 = OC5
		RPB4R = 0x0;		// RB4 = TX
		OC4CONSET = _OC4CON_ON_MASK;
		OC5CONSET = _OC5CON_ON_MASK;
		break;
	    }
	    
	    case 1: {
		OC4CONCLR = _OC4CON_ON_MASK;
		OC5CONCLR = _OC5CON_ON_MASK;
		U1RXR = 0x2;		// RA4 = RX
		RPA2R = 0x0;		// RA2 = OC4
		RPA4R = 0x0;		// RA4 = OC5
		RPB4R = 0x1;		// RB4 = TX
		U1MODESET = _U1MODE_ON_MASK;
		break;
	    }
	}
	
	/*
	switch (bot.state) {
	    case IDLE: {
		bot.numBias = 0;
		for (int i = 0; i < 0; i++) bot.bias[i] = 0;
		break;
	    }
	    
	    case NAVIGATE: {
		bot.bias[0] /= bot.numBias;
		bot.bias[1] /= bot.numBias;
		bot.bias[2] *= M_PI / (180.0 * bot.numBias);
		break;
	    }
	} */
	
    }
    bot->portCN = PORTB;
    IFS1CLR = _IFS1_CNBIF_MASK;
}
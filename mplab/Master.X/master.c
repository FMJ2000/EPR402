/* 
 * File:   main.c in master
 * Author: martin
 *
 * Created on 23 September 2021, 2:51 PM
 */

/* Bluetooth setup code:
 * sudo killall rfcomm & sudo rfcomm connect /dev/rfcomm0 98:DA:D0:00:5E:C4 1 &
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
    
    Master_Init();
    Init(bot->buf);
    bot->portCN = PORTB;
    Bot_UART_Write(bot, "Bot Initialized...\r\n");
    uint8_t whoami[2] = {0};
    IMU_Init(bot->asa, &bot->pos[2], whoami);
    Bot_UART_Write(bot, "Peripherals Initialized...\r\n");
    
    OLED_Init();
    OLED_ClearDisplay();
    snprintf(bot->buf, 100, "imu %d, mag %d", whoami[0], whoami[1]);
    OLED_Write_Text(0, 0, bot->buf);
    OLED_Update();
    
    for (;;);
    return EXIT_SUCCESS;
}

void Master_Init() {
    bot = malloc(sizeof(struct Bot));
    bot->state = IDLE;//STATE_UART_MASK;		// .uartState=0 in prod
    float startMap[2] = { -MAP_SIZE / 2, MAP_SIZE / 2 };
    BitMap_Initialize(bot, &bot->currentMap, startMap);
    Bot_Map_Required(bot);
    bot->inputPos[0] = 2.0;
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
    Bot_Pos_Update(bot);
    if (bot->count % 9) {
	bot->usState = 0;
	Ultrasonic_Trigger();
    }
    Bot_Controller(bot);
    if (bot->count > FREQ) {
	// 1 Hz
	bot->time++;
	Odometer_Read(bot->odo);
	Bot_Display_Status(bot);
	AD1CON1SET = _AD1CON1_SAMP_MASK;
	/*if (bot->time++ == 3) {
	    bot->state = NAVIGATE;
	    bot->bias[0] /= bot->numBias;
	    bot->bias[1] /= bot->numBias;
	    bot->bias[2] *= M_PI / (180.0 * bot->numBias);
	}*/
	
	
	if (bot->state & STATE_UART_MASK) Bot_UART_Send_Status(bot);
	//Bot_Display_BitMap(bot.currentMap); 
	/*for (char i = 0; i < 4; i++)
	    if (ProbMap_Contains(bot.localMaps[i], bot.pos))
		Bot_Display_ProbMap(bot.localMaps[i]);*/
	bot->count = 0;
    }
}

/* Ultrasonic echo timer */
void __ISR(_TIMER_5_VECTOR, IPL2SOFT) TMR5_IntHandler() {
    IFS0CLR = _IFS0_T5IF_MASK;
    if (bot->usState < US_SENSORS) {
	bot->distances[bot->usState] = TMR5 * SOUND_SPEED;
	TMR5 = 0;
	//if (bot->distances[bot->usState] > MAX_US_DIST || bot->distances[bot->usState] < MIN_US_DIST) bot->distances[bot->usState] = MAX_US_DIST;

	/* check if all readings taken, update map */
	bot->usState++;// = bot->usState + 1) % US_SENSORS;
	//if (bot->usState == 0) Bot_Map_Update(bot);
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
    if ((bot->portCN & 0x20) && !(PORTB & 0x20)) {
	/* button press occured */
	/*free(bot);
	SYS_Unlock();
	RSWRSTSET = 1;
	volatile int * p = &RSWRST;
	*p;
	while (1); */
	bot->state ^= STATE_MASK0;
	switch (bot->state & STATE_MASK) {
	    case IDLE: {
		bot->numBias = 0;
		for (int i = 0; i < 6; i++) bot->bias[i] = 0;
		break;
	    }
	    
	    case NAVIGATE: {
		for (uint8_t i = 0; i < 6; i++) bot->bias[i] /= bot->numBias;
		bot->bias[2] *= M_PI / 180.0;
		break;
	    }
	}
    }
    bot->portCN = PORTB;
    IFS1CLR = _IFS1_CNBIF_MASK;
}

void __ISR(_DMA1_VECTOR, IPL2SOFT) DMA_IntHandler() {
    if (DCH1INT & _DCH1INT_CHBCIF_MASK) {
	DCH1INTCLR = _DCH1INT_CHBCIF_MASK;
    }
    IFS1CLR = _IFS1_DMA1IF_MASK;
}
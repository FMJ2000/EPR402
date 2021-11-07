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
    bot->portCN = 1;
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
    bot->state = INIT;//STATE_UART_MASK;		// .uartState=0 in prod
    float sensorOffsets[3] = { SENSOR_OFFSET, 0.0, -SENSOR_OFFSET };
    memcpy(bot->sensorOffsets, sensorOffsets, sizeof(bot->sensorOffsets));
    float obsModifier[8] = { 0, -0.8727, 1.2217, -1.2217, 0.8727, 1.571, 1.2217, 1.571 };
    memcpy(bot->obsModifier, obsModifier, sizeof(bot->obsModifier));
    /*bot->angleModifier = (float [8]){3*M_PI/4, M_PI/2, M_PI/4, 0, -M_PI/4, M_PI/2, -3*M_PI/4, M_PI};
    const uint8_t posModifier[8][2] = {
        {-1, 1},
        {0, 1},
        {1, 1},
        {1, 0},
        {1, -1},
        {0, -1},
        {-1, -1},
        {-1, 0}
    };
    memcpy(&bot->posModifier, &posModifier, sizeof(bot->posModifier));
    Bot_FIR_Init(bot);
    float startMap[2] = { -MAP_SIZE / 2, MAP_SIZE / 2 };
    BitMap_Initialize(bot, &bot->currentMaps[0], startMap);
    Bot_Map_Required(bot);*/
    //bot->inputPos[0][0] = -0.6;
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
    
    // bot us update at 4 Hz
    if (bot->count % (FREQ / 4) == 0) {
	bot->usState = 0;
	Ultrasonic_Trigger();
    }
    
    // bot pos update and controller at 20 Hz
    Bot_Pos_Update(bot, bot->count, 0);
    Bot_Controller(bot, 0);
    
    // bot odo update and status display at 5 Hz
    if (bot->count % (FREQ / 5) == 0) {
	Odometer_Read(5, bot->odo);
	Bot_Display_Status(bot);
	//Bot_Display_BitMap(bot);
    }
    
    // bot battery read at 1 Hz
    if (bot->count % FREQ == 0) {
	bot->time++;
	if (bot->time == 3) {
	    for (uint8_t i = 0; i < 6; i++) {
		bot->bias[i] /= bot->numBias;
		if (i < 3) bot->bias[i] *= M_PI / 180.0;
	    }
	    bot->state = (bot->state & ~STATE_MASK) | NAVIGATE;
	}
	AD1CON1SET = _AD1CON1_SAMP_MASK;
	//Bot_Optimise_Local(bot);
	if (bot->state & STATE_UART_MASK) Bot_UART_Send_Status(bot);
	bot->count = 0;
	bot->portCN = 1;
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
	if ((bot->state & STATE_MASK) == NAVIGATE) Bot_Navigate(bot, bot->usState);
	bot->usState = (bot->usState + 1) % US_SENSORS;
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
    if (bot->portCN && !(PORTB & 0x20)) {
	/* button press occured */
	/*free(bot);
	SYS_Unlock();
	RSWRSTSET = 1;
	volatile int * p = &RSWRST;
	*p;
	while (1); */
	char newState = ((bot->state & STATE_MASK) == IDLE) ? NAVIGATE : IDLE;
	bot->state = (bot->state & ~STATE_MASK) | newState;
	bot->portCN = 0;
    }
    IFS1CLR = _IFS1_CNBIF_MASK;
}

void __ISR(_DMA1_VECTOR, IPL2SOFT) DMA_IntHandler() {
    if (DCH1INT & _DCH1INT_CHBCIF_MASK) {
	DCH1INTCLR = _DCH1INT_CHBCIF_MASK;
    }
    IFS1CLR = _IFS1_DMA1IF_MASK;
}
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

#include "main.h"

/* Bot object that contains state */
static Bot bot = { .pos = {0} };

int main() {
	/* delay startup to allow peripherals to initialize */
	long delay = 6400000l;
	while (delay--);
	
	/* Initialize peripherals */
	PORT_Init();
	INT_Init();
	TMR_Init();
	UART_Init();
	I2C_Master_Init();
	char ret = IMU_Init();
	
	snprintf(bot.msg, 40, "Peripheral initialization complete\r\n");
	UART_Write_String(bot.msg);
	snprintf(bot.msg, 40, "return code: %d\r\n", ret);	
	UART_Write_String(bot.msg);
	
	T3CONSET = _T3CON_ON_MASK;
	
	/*Bot_Print_String(&bot, "slave: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n",
		data[IMU_MSG_LEN + 0], data[IMU_MSG_LEN + 1], data[IMU_MSG_LEN + 2], data[IMU_MSG_LEN + 3], data[IMU_MSG_LEN + 4], data[IMU_MSG_LEN + 5],
		data[IMU_MSG_LEN + 6], data[IMU_MSG_LEN + 7], data[IMU_MSG_LEN + 8], data[IMU_MSG_LEN + 9], data[IMU_MSG_LEN + 10], data[IMU_MSG_LEN + 11]
		);*/
	
	for(;;);
	return (EXIT_SUCCESS);
}


void __ISR(_EXTERNAL_0_VECTOR, IPL2SOFT) Ext0_IntHandler() {
	IFS0CLR = _IFS0_INT0IF_MASK;
	bot.fall |= 0x1;
}

void __ISR(_EXTERNAL_1_VECTOR, IPL2SOFT) Ext1_IntHandler() {
	IFS0CLR = _IFS0_INT1IF_MASK;
	bot.fall |= 0x2;
}

void __ISR(_EXTERNAL_3_VECTOR, IPL2SOFT) Ext3_IntHandler() {
	IFS0CLR = _IFS0_INT3IF_MASK;
	bot.fall |= 0x4;
}

void __ISR(_EXTERNAL_4_VECTOR, IPL2SOFT) Ext4_IntHandler() {
	IFS0CLR = _IFS0_INT4IF_MASK;
	bot.fall |= 0x8;
}

void __ISR (_TIMER_3_VECTOR, IPL2SOFT) TMR3_IntHandler() {
	LATBINV = _LATB_LATB11_MASK;
	IFS0CLR = _IFS0_T3IF_MASK;
	
	int imu_data[IMU_MSG_LEN] = {0};
	char slave_data[SLAVE_MSG_LEN] = {0};
	int ret = IMU_Read(imu_data);
	Slave_Read(slave_data);
	Bot_Periph(&bot, imu_data, slave_data);
	Bot_Print(&bot);
	
	/*
	 * unsigned char slave[MSG_LEN];
	float ultrasonic[4];
	
	Slave_Interpret(slave, ultrasonic);
	snprintf(msg, 80, "slave: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", slave[0], slave[1], slave[2], slave[3], slave[4], slave[5], slave[6], slave[7], slave[8], slave[9], slave[10], slave[11]);
	UART_Write_String(msg);
	snprintf(msg, 80, "ultrasonic: %.2fmm, %.2fmm, %.2fmm, %.2fmm\r\n", ultrasonic[0], ultrasonic[1], ultrasonic[2], ultrasonic[3]);
	UART_Write_String(msg);
	snprintf(msg, 80, "odometer: %d, %d | ADC: %d | user: %d\r\n", slave[8], slave[9], slave[10], slave[11]);
	UART_Write_String(msg);
	snprintf(msg, 80, "fall: %d, %d, %d, %d\r\n", fall[0], fall[1], fall[2], fall[3]);
	UART_Write_String(msg);
	 * */
	
} 
#include "bot.h"
#include "main.h"

void Bot_Periph(Bot * bot, int * imu_data, char * slave_data) {
	/* read IMU data into bot {ax, ay, az, t, gx, gy, gz} */
	for (int i = 0; i < 3; i++) {
		bot->acc[i] = (float)(imu_data[i] * ACC_RES);
		bot->gyro[i] = (float)(imu_data[4+i] * GYRO_RES);
	}
	bot->temp = (float)(imu_data[3] / 340 + TEMP_OFFSET);
	
	/* read Slave data {us1[2], us2[2], us3[2], us4[2], odl, odr, bat, usr */
	unsigned int val[4];
	for (int i = 0; i < 4; i++) {
		val[i] = (unsigned int)((slave_data[2*i] << 8) | slave_data[2*i+1]);
		bot->view[i] = (float)(val[i] * SOUND_SPEED / 1000000.0);
	}
	bot->odo[0] = (int)slave_data[8];
	bot->odo[1] = (int)slave_data[9];
	bot->bat = (float)slave_data[10];
}

/* Navigate Bot to desired position based on current position and input */
void Bot_Navigate(Bot * bot) {
    /* update Bot position */
    bot->speed[0] = WHEEL_CIRC * bot->odo[0] / N_ODO;
    bot->speed[1] = WHEEL_CIRC * bot->odo[1] / N_ODO;
    bot->pos[X] += 0.5 * (bot->speed[0] + bot->speed[1]) * cos(bot->pos[Z]) * DT;
    bot->pos[Y] += 0.5 * (bot->speed[0] + bot->speed[1]) * sin(bot->pos[Z]) * DT;
}


/* Print status of Bot to UART 
void Bot_Print(Bot * bot) {
	snprintf(bot->msg, sizeof(bot->msg), "\x1b[2J\x1b[HBot status:\r\n");
	UART_Write_String(bot->msg);
	snprintf(bot->msg, sizeof(bot->msg), "position:\t\t(%.2f, %.2f, %.2f)\r\n", bot->pos[X], bot->pos[Y], bot->pos[Z]);
	UART_Write_String(bot->msg);
    snprintf(bot->msg, sizeof(bot->msg), "desired:\t\t(%.2f, %.2f, %.2f)\r\n", bot->desiredPos[X], bot->desiredPos[Y], bot->desiredPos[Z]);
	UART_Write_String(bot->msg);
    snprintf(bot->msg, sizeof(bot->msg), "speed:\t\t(%.2f, %.2f)\r\n", bot->speed[0], bot->speed[1]);
	UART_Write_String(bot->msg);
	snprintf(bot->msg, sizeof(bot->msg), "view:\t\t\t(%.2f, %.2f, %.2f, %.2f)\r\n", bot->view[0], bot->view[1], bot->view[2], bot->view[3]);
	UART_Write_String(bot->msg);
	snprintf(bot->msg, sizeof(bot->msg), "accelerometer:\t\t(%.2f, %.2f, %.2f) g\r\n",  bot->acc[X], bot->acc[Y], bot->acc[Z]);
	UART_Write_String(bot->msg);
	snprintf(bot->msg, sizeof(bot->msg), "gyroscope:\t\t(%.2f, %.2f, %.2f) dps\r\n",  bot->gyro[X], bot->gyro[Y], bot->gyro[Z]);
	UART_Write_String(bot->msg);
	snprintf(bot->msg, sizeof(bot->msg), "odometer:\t\t(%d, %d)\r\n",  bot->odo[0], bot->odo[1]);
	UART_Write_String(bot->msg);
	snprintf(bot->msg, sizeof(bot->msg), "battery:\t\t%.2f\r\n",  bot->bat);
	UART_Write_String(bot->msg);
	snprintf(bot->msg, sizeof(bot->msg), "temperature:\t\t%.2f\r\n",  bot->temp);
	UART_Write_String(bot->msg);
}	

/*
void Bot_Print_Ultrasonic(Bot * bot) {
	snprintf(bot->msg, sizeof(bot->msg), "%.2fmm, %.2fmm, %.2fmm, %.2fmm\r\n", bot->view[0], bot->view[1], bot->view[2], bot->view[3]);
	UART_Write_String(bot->msg);
}
 * */

/* Print given string to UART */
void Bot_Print_String(Bot * bot, const char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(bot->msg, sizeof(bot->msg), fmt, ap);
	va_end(ap);
	UART_Write_String(bot->msg);
}
#include "bot.h"

// bot init
void Bot_Init(struct Bot ** bot, float pos[3]) {
	*bot = malloc(sizeof(struct Bot));
	memcpy((*bot)->pos, pos, sizeof(float) * 3);
	float mapPos[2] = { pos[0] - MAP_SIZE / 2, pos[1] + MAP_SIZE / 2 };
	Map_Init(&((*bot)->map), mapPos);
	for (uint8_t i = 0; i < 6; i++) {
		(*bot)->P[i][i] = 1;
		(*bot)->Q[i][i] = SIGMA_Q;
		(*bot)->R[i][i] = SIGMA_R;
	}
	Bot_UKF_Sigma(*bot);
}

// determine maps that form part of local map and create if necessary
void Bot_Map_Required(struct Bot * bot) {
	if (!Map_Contains(bot->map, bot->pos))
		for (uint8_t i = 0; i < 8; i++) {
			if (bot->map->neighbors[i] && Map_Contains(bot->map->neighbors[i], bot->pos)) {
				bot->map = bot->map->neighbors[i];
				break;
			}
		}

	for (uint8_t i = 0; i < 8; i++) {
		if (!bot->map->neighbors[i]) {
			float pos[2] = { bot->map->pos[0] + posMod[i][0] * MAP_SIZE, bot->map->pos[1] + posMod[i][1] * MAP_SIZE };
			Map_Init(&(bot->map->neighbors[i]), pos);
			Map_Reinforce(bot->map->neighbors[i]);
		}
	}
}

void Bot_Map_Update(struct Bot * bot) {
	
}

// localization steps, IMU updates at 40 Hz
void Bot_Pos_IMU(struct Bot * bot) {
	IMU_Read(bot->imu);

	for (uint8_t i = 0; i < UKF_T; i++) {
		// measure
		bot->Z[i][3] = bot->Y[i][3] + bot->imu[0]*DT_IMU*cos(bot->Y[i][2]) - bot->imu[1]*DT_IMU*sin(bot->Y[i][2]);
		bot->Z[i][4] = bot->Y[i][4] + bot->imu[0]*DT_IMU*sin(bot->Y[i][2]) + bot->imu[1]*DT_IMU*cos(bot->Y[i][2]);
		bot->Z[i][5] = bot->imu[2];
		
		// process
		bot->Y[i][3] = 0.5*WHEEL_R*PWM_V*cos(bot->Y[i][2])*(bot->duty[0] + bot->duty[1]);
		bot->Y[i][4] = 0.5*WHEEL_R*PWM_V*sin(bot->Y[i][2])*(bot->duty[0] + bot->duty[1]);
		bot->Y[i][5] = WHEEL_R*(bot->duty[0] - bot->duty[1]) / CHASSIS_L;
		for (uint8_t j = 0; j < 3; j++) {
			bot->Y[i][j] += bot->Y[i][3+j]*DT_IMU;
			bot->Z[i][j] = bot->Y[i][j] + bot->Z[i][3+j]*DT_IMU;
		}	
	}

	// update
	Bot_UKF_Update(bot);
}

// odometer updates at 4 Hz
void Bot_Pos_Odo(struct Bot * bot) {
	Odometer_Read(bot->odo, F_ODO);

	float oVel[2] = { bot->odo[0]*WHEEL_R, bot->odo[1]*WHEEL_R };
	float w = (oVel[1] - oVel[0]) / 2;
	float r = (oVel[1] + oVel[0]) * CHASSIS_L / (2*(oVel[1] - oVel[0]));
	for (uint8_t i = 0; i < UKF_T; i++) {
		// measure
		float xIcc[2] = { bot->X[i][0] - r*sin(bot->X[i][2]), bot->X[i][1] + r*cos(bot->X[i][2]) };
		bot->Z[i][0] = (bot->X[i][0] - xIcc[0])*cos(w*DT_ODO) - (bot->X[i][1] - xIcc[1])*sin(w*DT_ODO) + xIcc[0];
		bot->Z[i][1] = (bot->X[i][0] - xIcc[0])*sin(w*DT_ODO) + (bot->X[i][1] - xIcc[1])*cos(w*DT_ODO) + xIcc[1];
		bot->Z[i][2] = bot->X[i][2] + w*DT_ODO;
		bot->Z[i][3] = bot->X[i][3]*cos(bot->Z[i][2]);
		bot->Z[i][4] = bot->X[i][4]*cos(bot->Z[i][2]);
		bot->Z[i][5] = w;
	}
	
	// update and new sigma points
	Bot_UKF_Update(bot);
	Bot_UKF_Sigma(bot);
}

// obtain sigma points
void Bot_UKF_Sigma(struct Bot * bot) {
	float S[UKF_N][UKF_N];
	float PQ[UKF_N][UKF_N];
	Mat_Add(UKF_N, UKF_N, PQ, bot->P, bot->Q);
	cholesky(UKF_N, S, PQ);

	float scale = sqrt(UKF_N + UKF_L);
	for (uint8_t i = 0; i < UKF_N; i++) {
		bot->X[0][i] = bot->pos[i];
		for (uint8_t j = 0; j < UKF_N; j++) {
			bot->X[j+1][i] = bot->pos[i] + scale * S[j][i];
			bot->X[j+5][i] = bot->pos[i] - scale * S[j][i];
		}
	}
	memcpy(bot->Y, bot->X, sizeof(bot->Y));
}

void Bot_UKF_Mean(uint8_t rows, float result[rows], float w[UKF_T], float X[UKF_T][rows]) {
	for (uint8_t j = 0; j < rows; j++) result[j] = 0;
	for (uint8_t i = 0; i < UKF_T; i++)
		for (uint8_t j = 0; j < rows; j++)
			result[j] += w[i] * X[i][j];
}

void Bot_UKF_Cov(uint8_t rows, float result[rows][rows], float w[UKF_T], float X[UKF_T][rows], float m[rows]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < rows; j++)
			result[i][j] = 0;

	float Xm[rows][1];
	float Xm_T[1][rows];
	float XmX[rows][rows];
	for (uint8_t i = 0; i < UKF_T; i++) {
		for (uint8_t j = 0; j < rows; j++)
			Xm[j][0] = X[i][j] - m[j];
		Mat_T(rows, 1, Xm_T, Xm);
		Mat_Mul(rows, rows, 1, XmX, Xm, Xm_T);
		for (uint8_t j = 0; j < rows; j++)
			for (uint8_t k = 0; k < rows; k++)
				result[j][k] += w[i] * XmX[j][k];
	}
}

void Bot_UKF_Update(struct Bot * bot) {
	// mean and covariance
	Bot_UKF_Mean(UKF_N, bot->xp, bot->Wm, bot->Y);
	Bot_UKF_Mean(UKF_N, bot->zp, bot->Wm, bot->Z);
	Bot_UKF_Cov(UKF_N, bot->Pp, bot->Wp, bot->Y, bot->xp);
	Bot_UKF_Cov(UKF_N, bot->Pzz, bot->Wp, bot->Z, bot->zp);
	Mat_Add(UKF_N, UKF_N, bot->Pvv, bot->Pzz, bot->R);

	for (uint8_t i = 0; i < UKF_N; i++) bot->v[i][0] = bot->zp[i] - bot->xp[i];
	for (uint8_t i = 0; i < UKF_N; i++)
		for (uint8_t j = 0; j < UKF_N; j++)
			bot->Pxz[i][j] = 0;
	
	float Xm[UKF_N][1];
	float Zm_T[1][UKF_N];
	float XmZ[UKF_N][UKF_N];
	for (uint8_t i = 0; i < UKF_T; i++) {
		for (uint8_t j = 0; j < UKF_N; j++) {
			Xm[j][0] = bot->Y[i][j] - bot->xp[j];
			Zm_T[0][j] = bot->Z[i][j] - bot->zp[j];
		}
		Mat_Mul(UKF_N, UKF_N, 1, XmZ, Xm, Zm_T);
		for (uint8_t j = 0; j < UKF_N; j++)
			for (uint8_t k = 0; k < UKF_N; k++)
				bot->Pxz[j][k] += bot->Wp[i] * XmZ[j][k];
	}

	float K[UKF_N][UKF_N];
	float Pvv_inv[UKF_N][UKF_N];
	float K_zp[UKF_N][1];
	Mat_Inv(UKF_N, Pvv_inv, bot->Pvv);
	Mat_Mul(UKF_N, UKF_N, UKF_N, K, bot->Pxz, Pvv_inv);
	Mat_Mul(UKF_N, 1, UKF_N, K_zp, K, bot->v);
	for (uint8_t i = 0; i < UKF_N; i++) bot->pos[i] = bot->xp[i] + K_zp[i][0];
	normAngle(bot->pos[2]);

	float K_T[UKF_N][UKF_N];
	float KP[UKF_N][UKF_N];
	float KKP[UKF_N][UKF_N];
	Mat_T(UKF_N, UKF_N, K_T, K);
	Mat_Mul(UKF_N, UKF_N, UKF_N, KP, bot->Pvv, K_T);
	Mat_Mul(UKF_N, UKF_N, UKF_N, KKP, K, KP);
	Mat_Sub(UKF_N, UKF_N, bot->P, bot->Pp, KKP);
}

// print
void Bot_Display_Status(struct Bot * bot) {
	char line[OLED_LINE_LEN];
	char status[4];
	switch (bot->state & STATE_MASK) {
		case INIT: strcpy(status, "INT"); break;
		case IDLE: strcpy(status, "IDL"); break;
		case NAVIGATE: strcpy(status, "NAV"); break;
		case BATTERY: strcpy(status, "BAT"); break;
	}
	OLED_ClearDisplay();
	snprintf(line, OLED_LINE_LEN, "%s %ds %ds %.f%%", status, bot->time, bot->dblClickCount, bot->battery);
	OLED_Write_Text(0, 0, line);
	snprintf(line, OLED_LINE_LEN, "p %.2f %.2f %.f", bot->pos[0], bot->pos[1], bot->pos[2] * 180.0 / M_PI);
	OLED_Write_Text(0, 10, line);
	//snprintf(line, OLED_LINE_LEN, "g %.2f %.2f %.f", pos[0], pos[1], pos[1] * 180.0 / M_PI);
	//OLED_Write_Text(0, 20, line);
	snprintf(line, OLED_LINE_LEN, "dc %.2f %.2f o %d %d", bot->duty[0], bot->duty[1], bot->odo[0], bot->odo[1]);
	OLED_Write_Text(0, 30, line);
	snprintf(line, OLED_LINE_LEN, "us %.2f %.2f %.2f", bot->dist[0], bot->dist[1], bot->dist[2]);
	OLED_Write_Text(0, 40, line);
	//snprintf(line, OLED_LINE_LEN, "n %.f %.2f %.2f %.2f", bot->randomAngle * 180. / M_PI, bot->randomRadius, bot->dPos[0], bot->dPos[1]);
	//OLED_Write_Text(0, 50, line);
	OLED_Update();
}

void Bot_UART_Send_Status(struct Bot * bot) {
    Bot_UART_Write(bot, 
	    "\33[2J\33[H(%ds) Bot state: %d, battery: %.2f%%\r\n"
	    "pos: (%.2f, %.2f, %.2f)\r\n"
	    "duty: (%.2f, %.2f), odo: (%d, %d)\r\n"
	    "imu: (%.2f, %.2f. %.2f)\r\n"
	    "dist: (%.2f, %.2f, %.2f)\r\n"
	    "bitMap: (%.2f, %.2f)\r\n",
	    bot->time, bot->state, bot->battery,
	    bot->pos[0], bot->pos[1], bot->pos[2] * 180.0 / M_PI,
	    bot->duty[0], bot->duty[1], bot->odo[0], bot->odo[1],
	    bot->imu[0], bot->imu[1], bot->imu[2],
	    bot->dist[0], bot->dist[1], bot->dist[2],
	    bot->map->pos[0], bot->map->pos[1]
    );
}

void Bot_UART_Write(struct Bot * bot, char * format, ...) {
    va_list args;
    va_start(args, format);
    while (!U2STAbits.TRMT);
    DCH0SSIZ = vsnprintf(bot->buf, BUF_LEN, format, args);  
    DCH0INTCLR = 0x00FF00FF;
    DCH0CONSET = _DCH0CON_CHEN_MASK;
    va_end(args);
}
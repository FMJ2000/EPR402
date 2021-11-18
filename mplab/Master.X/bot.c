#include "bot.h"

// bot init
void Bot_Init(struct Bot ** bot, float pos[3]) {
	struct Bot temp = {0};
	*bot = malloc(sizeof(struct Bot));
	**bot = temp;
	memcpy((*bot)->pos, pos, sizeof(float) * 3);
	float mapPos[2] = { pos[0] - MAP_SIZE / 2, pos[1] + MAP_SIZE / 2 };
	Map_Init(&((*bot)->map), mapPos);
	Bot_Map_Required(*bot);
	(*bot)->portCN = 1;
	Bot_UKF_Init(*bot);
	(*bot)->goalIndex = GOAL_LEN - 1;
	
	// temporary configurations
	(*bot)->goalIndex = GOAL_LEN - 3;
	(*bot)->goal[(*bot)->goalIndex][0] = 0.8;
	(*bot)->goal[(*bot)->goalIndex][1] = -0.5;
	(*bot)->goal[(*bot)->goalIndex+1][0] = 0.8;
	(*bot)->goal[(*bot)->goalIndex+1][1] = 0.3;
	(*bot)->goal[(*bot)->goalIndex+2][0] = 0.8;
	(*bot)->goal[(*bot)->goalIndex+2][1] = 0.7;
	//(*bot)->explorePos[0] = 2.0;
}

// determine maps that form part of local map and create if necessary
void Bot_Map_Required(struct Bot * bot) {
	if (!Map_Contains(bot->map, bot->pos)) {
		for (uint8_t i = 0; i < 8; i++) {
			if (bot->map->neighbors[i] && Map_Contains(bot->map->neighbors[i], bot->pos)) {
				bot->map = bot->map->neighbors[i];
				break;
			}
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
	Bot_Map_Required(bot);
	Map_Update(bot->map, bot->pos, bot->dist);
	for (uint8_t i = 0; i < 8; i++) Map_Update(bot->map->neighbors[i], bot->pos, bot->dist);
}

void Bot_Bias(struct Bot * bot) {
	switch (bot->state & STATE_MASK) {
		case INIT:
		case IDLE: {
			for (uint8_t i = 0; i < 3; i++) bot->bias[i] += bot->imu[i];
			bot->numBias++;
			break;
		}
		case NAVIGATE: {
			for (uint8_t i = 0; i < 3; i++) bot->imu[i] -= bot->bias[i];
			break;
		}
	}
}

// localization steps, IMU updates at 40 Hz
void Bot_Pos_IMU(struct Bot * bot) {
	IMU_Read(bot->imu);
	Bot_Bias(bot);
	if ((bot->state & STATE_MASK) != NAVIGATE) return;

	for (uint8_t i = 0; i < UKF_T; i++) {
		// measure
		bot->Z[i][3] += bot->imu[0]*DT_IMU;
		bot->Z[i][4] = K_OLD*bot->Z[i][4] + (1-K_OLD)*bot->imu[2];
		Bot_Motion_Model(&(bot->Z[i]), DT_IMU);
		
		// process
		bot->Y[i][3] = K_OLD*bot->Y[i][3] + (1-K_OLD)*0.5*WHEEL_R*PWM_V*(bot->duty[0] + bot->duty[1]);
		bot->Y[i][4] = K_OLD*bot->Y[i][4] + (1-K_OLD)*WHEEL_R*PWM_V*(bot->duty[1] - bot->duty[0]) / CHASSIS_L;
		Bot_Motion_Model(&(bot->Y[i]), DT_IMU);
	}
	// update
	Bot_UKF_Update(bot, 0);
}

// odometer updates at 4 Hz
void Bot_Pos_Odo(struct Bot * bot) {
	Odometer_Read(&bot->odo, F_ODO);
	//Vec_Print(bot, 2, bot->odo, "odo");
	//delay(100000l);
	if ((bot->state & STATE_MASK) != NAVIGATE) return;
	
	for (uint8_t i = 0; i < UKF_T; i++) {
		bot->Z[i][3] = K_OLD*bot->Z[i][3] + (1-K_OLD)*0.5*(bot->odo[1] + bot->odo[0]);
		bot->Z[i][4] = K_OLD*bot->Z[i][4] + (1-K_OLD)*(bot->odo[0] - bot->odo[1]) / CHASSIS_L;

		memcpy(bot->Z[i], bot->X[i], sizeof(float) * 3);
		Bot_Motion_Model(&bot->Z[i], DT_ODO);
	}

	//Mat_Print(bot, UKF_T, UKF_N, bot->Z, "Z");
	// update and new sigma points
	Bot_UKF_Update(bot, 0.8);
	Bot_UKF_Sigma(bot);
}

void Bot_UKF_Init(struct Bot * bot) {
	for (uint8_t i = 0; i < 6; i++) {
		bot->P[i][i] = 1;
		bot->Q[i][i] = SIGMA_Q;
		bot->R[i][i] = SIGMA_R;
	}
	
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
			bot->X[j+1+UKF_N][i] = bot->pos[i] - scale * S[j][i];
		}
	}
	memcpy(bot->Y, bot->X, sizeof(bot->Y));
	memcpy(bot->Z, bot->X, sizeof(bot->Z));
}

void Bot_UKF_Mean(uint8_t rows, float result[rows], float X[UKF_T][rows]) {
	for (uint8_t j = 0; j < rows; j++) result[j] = 0;
	for (uint8_t i = 0; i < UKF_T; i++)
		for (uint8_t j = 0; j < rows; j++)
			result[j] += X[i][j];
	for (uint8_t i = 0; i < rows; i++) result[i] /= UKF_T;
}

void Bot_UKF_Cov(uint8_t rows, float result[rows][rows], float X[UKF_T][rows], float m[rows]) {
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
				result[j][k] += XmX[j][k];
	}
	
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < rows; j++)
			result[i][j] /= UKF_T;
}

void Bot_UKF_Update(struct Bot * bot, float weight) {
	// mean and covariance
	Bot_UKF_Mean(UKF_N, bot->xp, bot->Y);
	Bot_UKF_Mean(UKF_N, bot->zp, bot->Z);
	Bot_UKF_Cov(UKF_N, bot->Pp, bot->Y, bot->xp);
	Bot_UKF_Cov(UKF_N, bot->Pzz, bot->Z, bot->zp);
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
				bot->Pxz[j][k] += XmZ[j][k];
	}
	
	for (uint8_t i = 0; i < UKF_N; i++)
		for (uint8_t j = 0; j < UKF_N; j++)
			bot->Pxz[i][j] /= UKF_T;

	float K[UKF_N][UKF_N];
	float Pvv_inv[UKF_N][UKF_N];
	float K_zp[UKF_N][1];
	Mat_Inv(UKF_N, Pvv_inv, bot->Pvv);
	Mat_Mul(UKF_N, UKF_N, UKF_N, K, bot->Pxz, Pvv_inv);
	Mat_Mul(UKF_N, 1, UKF_N, K_zp, K, bot->v);
	//Mat_Print(bot, UKF_N, UKF_N, bot->Z, "Z");
	for (uint8_t i = 0; i < UKF_N; i++) bot->pos[i] = (1 - weight)*bot->xp[i] + weight*bot->zp[i]; //K_zp[i][0];
	bot->pos[2] = normAngle(bot->pos[2]);

	float K_T[UKF_N][UKF_N];
	float KP[UKF_N][UKF_N];
	float KKP[UKF_N][UKF_N];
	Mat_T(UKF_N, UKF_N, K_T, K);
	Mat_Mul(UKF_N, UKF_N, UKF_N, KP, bot->Pvv, K_T);
	Mat_Mul(UKF_N, UKF_N, UKF_N, KKP, K, KP);
	Mat_Sub(UKF_N, UKF_N, bot->P, bot->Pp, KKP);
}

void Bot_Motion_Model(float (*x)[UKF_N], float dt) {
	(*x)[2] = normAngle((*x)[2] + (*x)[4]*dt);
	(*x)[0] += (*x)[3]*cos((*x)[2])*dt;
	(*x)[1] += (*x)[3]*sin((*x)[2])*dt;
}

// motor controller at 40 Hz
void Bot_Motor_Control(struct Bot * bot) {
	if ((bot->state & STATE_MASK) == NAVIGATE) {
		/*float e[2] = { bot->uGoal[0] - 0.5*WHEEL_R*PWM_V*(bot->odo[0] + bot->odo[1]), bot->uGoal[1] - WHEEL_R*PWM_V*(bot->odo[0] - bot->odo[1]) / CHASSIS_L };
		bot->duty[0] = K_UV * e[0] + K_UW * e[1];
		bot->duty[1] = K_UV * e[0] - K_UW * e[1];*/
		bot->duty[0] = bot->uGoal[0] - bot->uGoal[1];
		bot->duty[1] = bot->uGoal[0] + bot->uGoal[1];
		if (fabs(bot->duty[0]) < MIN_DC) bot->duty[0] = 0;
		if (fabs(bot->duty[1]) < MIN_DC) bot->duty[1] = 0;
	} else {
		bot->duty[0] = 0;
		bot->duty[1] = 0;
		LATBCLR = _LATB_LATB4_MASK;
		LATACLR = _LATA_LATA3_MASK;
	}

	float convDuty[2] = {0};
	if (bot->duty[0] > 0) {
		LATACLR = _LATA_LATA3_MASK;
		convDuty[0] = bot->duty[0];
	} else {
		LATASET = _LATA_LATA3_MASK;
		convDuty[0] = 1.0 + bot->duty[0];
	}
	if (bot->duty[1] > 0) {
		LATBCLR = _LATB_LATB4_MASK;
		convDuty[1] = bot->duty[1];
	} else {
		LATBSET = _LATB_LATB4_MASK;
		convDuty[1] = 1.0 + bot->duty[1];
	}

	OC4RS = (int)(convDuty[0] * PWM_T);
	OC5RS = (int)(convDuty[1] * PWM_T);
}

// position controller at 4 Hz
/*
void Bot_Pos_Control(struct Bot * bot) {
	if ((bot->state & STATE_MASK) != NAVIGATE) return;

	// check if new path needed
	if (Bot_Detect_Collision(bot)) Bot_Navigate(bot);
	if (getDistance(bot->goal[bot->goalIndex], bot->pos) < MIN_GOAL_DIST) {
		//bot->goalIndex++;
		//if (bot->goalIndex == GOAL_LEN) Bot_Navigate(bot);
		bot->state = (bot->state & ~STATE_MASK) | IDLE;
	}

	// obtain current and integral errors
	float e[2] = { getDistance(bot->pos, bot->goal[bot->goalIndex]), normAngle(getAngle(bot->pos, bot->goal[bot->goalIndex]) - bot->pos[2]) };
	memcpy(bot->ePosInt[bot->ePosIndex], e, sizeof(float) * 2);
	uint8_t prevIndex = (bot->ePosIndex > 0) ? bot->ePosIndex - 1 : INTEGRAL_LEN - 1;
	float eD[2] = { (e[0] - bot->ePosInt[prevIndex][0]) / DT_IMU, e[1] - bot->ePosInt[prevIndex][1] / DT_IMU };
	//float eD[2] = { K_DO * e[0] / (K_DA + e[0]) - bot->pos[3], K_RO * fabs(e[1]) / (K_RA + fabs(e[1])) - bot->pos[4] }
	float eI[2] = { 0 };
	for (uint8_t i = 0; i < INTEGRAL_LEN; i++) {
		eI[0] += bot->ePosInt[i][0] * DT_IMU;
		eI[0] += bot->ePosInt[i][1] * DT_IMU;
	}
	bot->ePosIndex = (bot->ePosIndex + 1) % INTEGRAL_LEN;

	// implement PID controller
	float u[2] = {
		K_DP*e[0] + K_DI*eI[0] + K_DD*eD[0],
		K_RP*e[1] + K_RI*eI[1] + K_RD*eD[1]
	};

	bot->uGoal[0] = K_DO * u[0] / (K_DA + u[0]);
	bot->uGoal[1] = K_RO * u[1] / (K_RA + u[1]);
	
	Bot_Motor_Control(bot);
}
 */

void Bot_Pos_Control(struct Bot * bot) {
	if ((bot->state & STATE_MASK) != NAVIGATE) {
		Bot_Motor_Control(bot);
		return;
	}

	// check if new path needed
	//if (Bot_Detect_Collision(bot)) Bot_Navigate(bot);
	if (getDistance(bot->goal[bot->goalIndex], bot->pos) < MIN_GOAL_DIST) {
		//bot->goalIndex++;
		//if (bot->goalIndex == GOAL_LEN) //Bot_Navigate(bot);
		bot->state = (bot->state & ~STATE_MASK) | IDLE;
	}

	// obtain current and integral errors
	bot->ePos[0] = getDistance(bot->pos, bot->goal[bot->goalIndex]);
	bot->ePos[1] = normAngle(getAngle(bot->pos, bot->goal[bot->goalIndex]) - bot->pos[2]);
	
	memcpy(bot->ePosInt[bot->ePosIndex], bot->ePos, sizeof(float) * 2);
	uint8_t prevIndex = (bot->ePosIndex > 0) ? bot->ePosIndex - 1 : INTEGRAL_LEN - 1;
	float eD[2] = { (bot->ePos[0] - bot->ePosInt[prevIndex][0]) / DT_IMU, bot->ePos[1] - bot->ePosInt[prevIndex][1] / DT_IMU };
	//float eD[2] = { K_DO * e[0] / (K_DA + e[0]) - bot->pos[3], K_RO * fabs(e[1]) / (K_RA + fabs(e[1])) - bot->pos[4] }
	float eI[2] = { 0 };
	for (uint8_t i = 0; i < INTEGRAL_LEN; i++) {
		eI[0] += bot->ePosInt[i][0] * DT_IMU;
		eI[0] += bot->ePosInt[i][1] * DT_IMU;
	}
	bot->ePosIndex = (bot->ePosIndex + 1) % INTEGRAL_LEN;
	
	float u[2] = {
		K_DP*e[0] + K_DI*eI[0] + K_DD*eD[0],
		K_RP*e[1] + K_RI*eI[1] + K_RD*eD[1]
	};
	
	bot->uGoal[0] = K_DO * bot->ePos[0] / (K_DA + bot->ePos[0]);
	bot->uGoal[1] = K_RO * bot->ePos[1] / (K_RA + fabs(bot->ePos[1]));
	
	Bot_UART_Write(bot, "e: %.2f, %.2f, u: %.2f, %.2f", bot->ePos[0], bot->ePos[1], bot->uGoal[0], bot->uGoal[1]);
	
	Bot_Motor_Control(bot);
}

// A* search algorithm
void Bot_Navigate(struct Bot * bot) {
	// check if new exploration point needed
	if (getDistance(bot->pos, bot->explorePos) < MIN_GOAL_DIST) Bot_Explore(bot); 
	
	// build queues of nodes
	struct NodeQueue openQueue = {0};
	struct NodeQueue closedQueue = {0};
	struct Node initNode = { .h=getDistance(bot->explorePos, bot->pos) };
	NodeQueue_Add(&openQueue, &initNode);
	
	struct Node * node = NULL;
	int iter = 0;
	while (!NodeQueue_Empty(&openQueue) && iter++ < MAX_SEARCH_ITER) {
		unsigned int bestIndex = NodeQueue_Best(&openQueue);
		NodeQueue_Remove(&openQueue, &node, bestIndex);
		
		if (node) {
			NodeQueue_Add(&closedQueue, node);
			
			// check if goalNode
			if (node->h < MIN_SEARCH_GOAL) {
				Bot_Backtrack(bot, node);
				NodeQueue_Destroy(&openQueue);
				NodeQueue_Destroy(&closedQueue);
				return;
			}

			// expand neighbors
			for (uint8_t i = 0; i < 8; i++) {
				int newPosI[2] = { node->pos[0] + bPosMod[i][0], node->pos[1] + bPosMod[i][1] };
				char flag = NodeQueue_Contains(&closedQueue, newPosI);
				if (NodeQueue_Contains(&closedQueue, newPosI) == -1) {
					float newPosF[2] = { bot->pos[0] + NAV_STEP*newPosI[0], bot->pos[1] + NAV_STEP*newPosI[1] };
					char fObs = Map_Pos_Collide(bot->map, newPosF);
					uint8_t nIndex = 0;
					while (fObs == -1 && nIndex < 8) fObs = Map_Pos_Collide(bot->map->neighbors[nIndex++], newPosF);
					if (!fObs) {
						int openIndex = NodeQueue_Contains(&openQueue, newPosI);
						float cost = (i % 2 == 0) ? NAV_SQRT : NAV_STEP;
						if (openIndex == -1) {
							// not in open queue yet	
							struct Node newNode = { 
								.pos={newPosI[0], newPosI[1]},
								.posf={newPosF[0], newPosF[1]},
								.g=(node->g + cost),
								.h=getDistance(bot->explorePos, newPosF),
								.parent=node
							};
							NodeQueue_Add(&openQueue, &newNode);
						} else if (node->g + cost < openQueue.queue[openIndex]->g) {
							openQueue.queue[openIndex]->g = node->g + cost;
							openQueue.queue[openIndex]->parent = node;
						}
					}
				}	
			}
		}
	}
	NodeQueue_Destroy(&openQueue);
	NodeQueue_Destroy(&closedQueue);
	
	// path not found, calculate new explore pos then recursive
	Bot_Explore(bot);
	Bot_Navigate(bot);
}

void Bot_Backtrack(struct Bot * bot, struct Node * node) {
	Bot_UART_Node(bot, node);
	delay(1000000l);
	if (node == NULL) return;
	bot->goalIndex = GOAL_LEN;
	int count = 0;
	while (node != NULL && bot->goalIndex >= 0) {
		if (++count % 2) memcpy(bot->goal[--bot->goalIndex], node->posf, sizeof(float) * 2);
		node = node->parent;
	}
}

// choose a new exploration position to maximize information gain
void Bot_Explore(struct Bot * bot) {
	// build edge map of current map first, then neighbors
	uint8_t fIndex[2] = {0};
	float fPos[2] = {0};
	if (Map_Frontier(bot->map, bot->pos, fIndex))	memcpy(fPos, bot->map->pos, sizeof(float) * 2);
	else {
		uint8_t mIndex = 0;
		while (mIndex < 8 && !Map_Frontier(bot->map->neighbors[mIndex++], bot->pos, fIndex));
		if (bot->map->neighbors[mIndex-1]) memcpy(fPos, bot->map->neighbors[mIndex-1]->pos, sizeof(float) * 2);
	}
	bot->explorePos[0] = fPos[0] + fIndex[0]*MAP_RES;
	bot->explorePos[1] = fPos[1] - fIndex[1]*MAP_RES;
}

char Bot_Detect_Collision(struct Bot * bot) {
	// check for collision
	if ((bot->uGoal[0] > V_REF && (bot->odo[0] + bot->odo[1]) == 0) || (fabs(bot->uGoal[1]) > W_REF && fabs(bot->imu[2] < W_REF / 2))) bot->collideCount++;
	else bot->collideCount = 0;
	if (bot->collideCount >= MAX_COL_COUNT) {
		bot->collideCount = 0;
		return 1;
	}

	// check for collision on trajectory - assuming constant velocity
	float xt[3] = { bot->pos[0], bot->pos[1], bot->pos[2] };
	for (uint8_t i = 0; i < (uint8_t)F_IMU; i++) {
		xt[2] += xt[4];
		xt[0] += xt[3]*cos(xt[2]);
		xt[1] += xt[3]*sin(xt[2]);
		char fObs = Map_Pos_Collide(bot->map, xt);
		uint8_t nIndex = 0;
		while (fObs == -1 && nIndex < 8) fObs = Map_Pos_Collide(bot->map->neighbors[nIndex++], xt);
		if (fObs == 1) return 1;
	}

	return 0;
}

// print
void Bot_Display_Status(struct Bot * bot) {
	char status[4];
	switch (bot->state & STATE_MASK) {
		case INIT: strcpy(status, "INT"); break;
		case IDLE: strcpy(status, "IDL"); break;
		case NAVIGATE: strcpy(status, "NAV"); break;
		case BATTERY: strcpy(status, "BAT"); break;
	}
	OLED_ClearDisplay();
	OLED_Write_Text(0, 0, "%s %ds %ds %.f%%", status, bot->time, bot->dblClickCount, bot->battery);
	OLED_Write_Text(0, 10, "p %.2f %.2f %.f", bot->pos[0], bot->pos[1], bot->pos[2] * 180.0 / M_PI);
	OLED_Write_Text(0, 20, "e %.2f %.f", bot->ePos[0], bot->ePos[1]*180./M_PI);
	OLED_Write_Text(0, 30, "dc %.2f %.2f o %.2f %.2f", bot->duty[0], bot->duty[1], bot->odo[0], bot->odo[1]);
	OLED_Write_Text(0, 40, "us %.2f %.2f %.2f", bot->dist[0], bot->dist[1], bot->dist[2]);
	OLED_Write_Text(0, 50, "u %.2f %.2f", bot->uGoal[0], bot->uGoal[1]);// * 180. / M_PI);
	OLED_Update();
}

void Bot_Display_Map(struct Bot * bot) {
	OLED_ClearDisplay();
	OLED_FillRectangle(0, 0, 4*MAP_UNITS, 4*MAP_UNITS, WHITE);
	for (uint8_t i = 0; i < MAP_UNITS; i++) {
		for (uint8_t j = 0; j < MAP_UNITS; j++) {
			if (bot->map->grid[i][j] < LOG_PRIOR) OLED_FillRectangle(4*i, 4*j, 4*i+4, 4*j+4, BLACK); 
		}
	}
	OLED_Write_Text(70, 0, "%.2f", bot->dist[0]);
	OLED_Write_Text(70, 10, "%.2f", bot->dist[1]);
	OLED_Write_Text(70 , 20, "%.2f", bot->dist[2]);
	OLED_Write_Text(70, 30, "%d %.2f", bot->usCount, LOG_PRIOR);
	OLED_Update();
}

void Bot_UART_Status(struct Bot * bot) {
    // clear screen: "\33[2J\33[H"
	Bot_UART_Write(bot,
			"t: %d\r\n"
			"p: (%.2f, %.2f, %.f, %.2f, %.f)\r\n"
			"i: (%.2f, %.2f, %.f)\r\n"
			"o: (%.2f, %.2f)\r\n"
			"u: (%.2f, %.2f)\r\n"
			"dc: (%.2f, %.2f)\r\n"
			"xp: (%.2f, %.2f, %.f, %.2f, %.f)\r\n"
			"zp: (%.2f, %.2f, %.f, %.2f, %.f)\r\n"
			"L: (%.2f)\r\n\n",
			bot->time,
			bot->pos[0], bot->pos[1], bot->pos[2]*180./M_PI, bot->pos[3], bot->pos[4]*180./M_PI,
			bot->imu[0], bot->imu[1], bot->imu[2]*180./M_PI,
			bot->odo[0], bot->odo[1],
			bot->uGoal[0], bot->uGoal[1],
			bot->duty[0], bot->duty[1],
			bot->xp[0], bot->xp[1], bot->xp[2]*180./M_PI, bot->xp[3], bot->xp[4]*180./M_PI,
			bot->zp[0], bot->zp[1], bot->zp[2]*180./M_PI, bot->zp[3], bot->zp[4]*180./M_PI,
			UKF_L
    );
}

void Bot_UART_NodeQueue(struct Bot * bot, struct NodeQueue * queue) {
	char msg[BUF_LEN];
	snprintf(msg, BUF_LEN, "\r\nnode:\r\n");
	unsigned int iter = queue->startIndex;
	while (iter != queue->endIndex) {
		if (queue->queue[iter]) snprintf(msg, BUF_LEN, "%s(%.2f, %.2f)\r\n", msg, queue->queue[iter]->posf[0], queue->queue[iter]->posf[1]);
		iter = (iter + 1) % Q_LEN;
	}
	Bot_UART_Write(bot, msg);
}

void Bot_UART_Node(struct Bot * bot, struct Node * node) {
	if (!node) Bot_UART_Write(bot, "null node\r\n");
	else Bot_UART_Write(bot, "n: pi: (%d, %d), pf: (%.2f, %.2f), g: %.2f, h: %.2f\r\n", node->pos[0], node->pos[1], node->posf[0], node->posf[1], node->g, node->h);
}

void Bot_UART_Map(struct Bot * bot) {
	char msg[BUF_LEN];
	snprintf(msg, BUF_LEN, "\r\nmap: (%.2f, %.2f)\r\n", bot->map->pos[0], bot->map->pos[1]);
	for (uint8_t i = 0; i < 8; i++)
		if (bot->map->neighbors[i])
			snprintf(msg, BUF_LEN, "%sn(%d): (%.2f, %.2f)\r\n", msg, i, bot->map->neighbors[i]->pos[0], bot->map->neighbors[i]->pos[1]);
	Bot_UART_Write(bot, msg);
}

void Bot_UART_Write(struct Bot * bot, char * format, ...) {
    va_list args;
    va_start(args, format);
    while (U2STAbits.UTXBF);
    DCH0SSIZ = vsnprintf(bot->buf, BUF_LEN, format, args);  
    DCH0INTCLR = 0x00FF00FF;
    DCH0CONSET = _DCH0CON_CHEN_MASK;
    va_end(args);
}

void Mat_Print(struct Bot * bot, uint8_t rows, uint8_t cols, float mat[rows][cols], char * title) {
	char msg[BUF_LEN];
	snprintf(msg, BUF_LEN, "%s: {", title);
	for (uint8_t i = 0; i < rows; i++) {
		snprintf(msg, BUF_LEN, "%s{", msg);
		for (uint8_t j = 0; j < cols; j++) {
			snprintf(msg, BUF_LEN, "%s%.1f, ", msg, mat[i][j]);
		}
		snprintf(msg, BUF_LEN, "%s},\r\n", msg);
	}
	snprintf(msg, BUF_LEN, "%s}\r\n", msg);
	Bot_UART_Write(bot, msg);
}

void Vec_Print(struct Bot * bot, uint8_t cols, float vec[cols], char * title) {
	char msg[BUF_LEN];
	snprintf(msg, BUF_LEN, "%s: {", title);
	for (uint8_t i = 0; i < cols; i++)
		snprintf(msg, BUF_LEN, "%s%.1f, ", msg, vec[i]);
	snprintf(msg, BUF_LEN, "%s}\r\n", msg);
	Bot_UART_Write(bot, msg);
}

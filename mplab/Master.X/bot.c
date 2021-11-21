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
	(*bot)->terrainMod = 0.5;
	
	for (uint8_t i = 0; i < F_ODO; i++)
		(*bot)->wOdo[i] = W_ODO / ((i+1)*(i+1));
	
	// temporary configurations
	//float queue[2][2] = { {0.5, 0.0}, {1, 0.0} };//{0.3, 0.0}, {0.3, 0.3}, {0., 0.3}, {0.0, 0.0} };
	//Bot_Goal_Queue(*bot, 2, queue);
	//(*bot)->explorePos[0] = 1.2;
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
	
	// only map this map and three maps in view
	uint8_t viewIndex = Bot_Map_View(bot);
	Map_Update(bot->map->neighbors[viewIndex], bot->pos, bot->dist);
	Map_Update(bot->map->neighbors[(viewIndex + 1) % 8], bot->pos, bot->dist);
	Map_Update(bot->map->neighbors[(viewIndex + 7) % 8], bot->pos, bot->dist);
	//Bot_UART_Write(bot, "map: (%d, %d, %d)\r\n", (viewIndex + 7) % 8, viewIndex, (viewIndex + 1) % 8);
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

uint8_t Bot_Map_View(struct Bot * bot) {
	float minAngle = M_PI;
	uint8_t minIndex = -1;
	for (uint8_t i = 0; i < 8; i++) {
		if (bot->map->neighbors[i]) {
			float midPos[2] = { bot->map->neighbors[i]->pos[0] + MAP_UNITS*MAP_RES / 2, bot->map->neighbors[i]->pos[1] + MAP_UNITS*MAP_RES / 2 };
			float angle = getAngle(bot->pos, midPos);
			if (fabs(angle) < fabs(minAngle)) {
				minAngle = angle;
				minIndex = i;
			}
		}
	}
	return minIndex;
}

// localization steps, IMU updates at 40 Hz
void Bot_Pos_IMU(struct Bot * bot) {
	//if ((bot->state & STATE_MASK) == INIT) return;
	IMU_Read(bot->imu);
	Bot_Bias(bot);
	
	Odometer_Read(&bot->odo, F_IMU);//odoArr[bot->odoIndex], F_IMU);	
	/*bot->odo[0] = 0;
	bot->odo[1] = 0;
	for (uint8_t i = 0; i < F_ODO; i++) {
		uint8_t index = (bot->odoIndex + i) % F_ODO;
		bot->odo[0] += bot->odoArr[index][0] * bot->wOdo[i];
		bot->odo[1] += bot->odoArr[index][1] * bot->wOdo[i];
	}
	bot->odoIndex = (bot->odoIndex + 1) % F_ODO;
	bot->odo[0] *= copysign(1.0, bot->duty[0]);
	bot->odo[1] *= copysign(1.0, bot->duty[1]);*/
	
	if ((bot->state & STATE_MASK) != NAVIGATE) return;
	
	// speedy solution
	/*bot->pos[3] = 0.5*WHEEL_R*PWM_V*(bot->duty[0] + bot->duty[1]);
	bot->pos[4] = bot->imu[2];//WHEEL_R*PWM_V*(bot->duty[0] - bot->duty[1]) / CHASSIS_L;
	Bot_Motion_Model(&bot->pos, DT_IMU);
	bot->opos[3] = 0.5*(bot->odo[1] + bot->odo[0]);
	bot->opos[4] = (bot->odo[0] - bot->odo[1]) / CHASSIS_L;
	Bot_Motion_Model(&bot->opos, DT_ODO);
	for (uint8_t i = 0; i < 5; i++) {
		bot->pos[i] = (1 - K_OLD) * bot->opos[i] + K_OLD * bot->pos[i];
		bot->opos[i] = bot->pos[i];
	}*/
	
	// speedier solution
	bot->pos[3] =  0.5*(bot->odo[1] + bot->odo[0]);
	bot->pos[4] = bot->imu[2];
	Bot_Motion_Model(&bot->pos, DT_IMU);
	Map_Update_Visit(bot->map, bot->pos);
	
	/*
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
	 */
}

// odometer updates at 4 Hz
void Bot_Pos_Odo(struct Bot * bot) {
	Odometer_Read(&bot->odo, F_ODO);
	//Vec_Print(bot, 2, bot->odo, "odo");
	//delay(100000l);
	if ((bot->state & STATE_MASK) != NAVIGATE) return;
	
	bot->opos[3] = 0.5*(bot->odo[1] + bot->odo[0]);
	bot->opos[4] = (bot->odo[1] - bot->odo[0]) / CHASSIS_L;
	Bot_Motion_Model(&bot->opos, DT_ODO);
	for (uint8_t i = 0; i < 5; i++) {
		bot->pos[i] = (1 - K_OLD) * bot->opos[i] + K_OLD * bot->pos[i];
		bot->opos[i] = bot->pos[i];
	}
	
	/*
	for (uint8_t i = 0; i < UKF_T; i++) {
		bot->Z[i][3] = K_OLD*bot->Z[i][3] + (1-K_OLD)*0.5*(bot->odo[1] + bot->odo[0]);
		bot->Z[i][4] = K_OLD*bot->Z[i][4] + (1-K_OLD)*(bot->odo[0] - bot->odo[1]) / CHASSIS_L;

		memcpy(bot->Z[i], bot->X[i], sizeof(float) * 3);
		Bot_Motion_Model(&bot->Z[i], DT_ODO);
	}

	//Mat_Print(bot, UKF_T, UKF_N, bot->Z, "Z");
	// update and new sigma points
	Bot_UKF_Update(bot, 0.5);
	Bot_UKF_Sigma(bot);
	 */
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

// set goal queue manually
void Bot_Goal_Queue(struct Bot * bot, uint8_t len, float queue[len][2]) {
	if (len > GOAL_LEN) return;
	bot->goalIndex = GOAL_LEN - len;
	for (uint8_t i = 0; i < len; i++) {
		bot->goal[bot->goalIndex + i][0] = queue[i][0];
		bot->goal[bot->goalIndex + i][1] = queue[i][1];
	}
}

void Bot_Motion_Model(float (*x)[UKF_N], float dt) {
	(*x)[2] = normAngle((*x)[2] + (*x)[4]*dt);
	(*x)[0] += (*x)[3]*cos((*x)[2])*dt;
	(*x)[1] += (*x)[3]*sin((*x)[2])*dt;
}

// motor controller at 40 Hz
void Bot_Motor_Control(struct Bot * bot) {
	if ((bot->state & STATE_MASK) == NAVIGATE) {
		float e[2] = { bot->uGoal[0] - bot->uGoal[1] - bot->odo[0], bot->uGoal[0] + bot->uGoal[1] - bot->odo[1] };
		bot->duty[0] = bot->uGoal[0] + bot->uGoal[1];// + K_UV*e[0];// - K_UW*e[1];
		bot->duty[1] = bot->uGoal[0] - bot->uGoal[1];// + K_UV*e[1];// + K_UW*e[1]; 
		//if (fabs(bot->duty[0]) < MIN_DC) bot->duty[0] = 0;
		//if (fabs(bot->duty[1]) < MIN_DC) bot->duty[1] = 0;
	} else {
		bot->duty[0] = 0;
		bot->duty[1] = 0;
		LATBCLR = _LATB_LATB4_MASK;
		LATACLR = _LATA_LATA3_MASK;
	}

	float convDuty[2] = {0};
	if (bot->duty[0] > 0) {
		LATBCLR = _LATB_LATB4_MASK;
		convDuty[0] = bot->duty[0];
	} else {
		LATBSET = _LATB_LATB4_MASK;
		convDuty[0] = 1.0 + bot->duty[0];
	}
	if (bot->duty[1] > 0) {
		LATACLR = _LATA_LATA3_MASK;
		convDuty[1] = bot->duty[1];
	} else {
		LATASET = _LATA_LATA3_MASK;
		convDuty[1] = 1.0 + bot->duty[1];
	}

	OC5RS = (int)(convDuty[0] * PWM_T);
	OC4RS = (int)(convDuty[1] * PWM_T);
}

// position controller at 4 Hz
void Bot_Pos_Control(struct Bot * bot) {
	if ((bot->state & STATE_MASK) != NAVIGATE) {
		Bot_Motor_Control(bot);
		return;
	}

	// check if new path needed
	if (getDistance(bot->goal[bot->goalIndex], bot->pos) < MIN_GOAL_DIST) {
		bot->goalIndex++;
		if (bot->goalIndex == GOAL_LEN) {
			//if (getDistance(bot->explorePos, bot->pos) < MIN_GOAL_DIST) bot->state = (bot->state & ~STATE_MASK) | FINISH;
			//else
			Bot_Navigate(bot);
		}
		else if (Bot_Path_Collision(bot)) {
			Bot_Explore(bot);
			Bot_Navigate(bot);
		}
		else Bot_UART_Write(bot, "new pos: (%.2f, %.2f)\r\n", bot->goal[bot->goalIndex][0], bot->goal[bot->goalIndex][1]);
		char oldState = bot->state & STATE_MASK;
		//bot->state = (bot->state & ~STATE_MASK) | IDLE;
		Bot_UART_Write(bot, "bpos = [%.2f, %.2f]\r\n", bot->pos[0], bot->pos[1]);
		//delay(20000l);
		Bot_Motor_Control(bot);
		//Bot_UART_Map(bot);
		//bot->state = (bot->state & ~STATE_MASK) | oldState;
		//if (bot->goalIndex == GOAL_LEN || Bot_Path_Collision(bot)) Bot_Navigate(bot);
		return;
	}

	// obtain current and integral errors
	float ePos[2] = { getDistance(bot->pos, bot->goal[bot->goalIndex]), bot->ePos[1] = getAngle(bot->pos, bot->goal[bot->goalIndex]) };
	float dPos[2] = { ePos[0] - bot->ePos[0], normAngle(ePos[1] - bot->ePos[1]) };
	/*
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
		K_DP*bot->ePos[0] + K_DI*eI[0] + K_DD*eD[0],
		K_RP*bot->ePos[1] + K_RI*eI[1] + K_RD*eD[1]
	};
	 * */

	//bot->uGoal[1] = K_RO * bot->ePos[1];// / (K_RA + fabs(bot->ePos[1]));
	
	bot->uGoal[1] = K_RO*ePos[1]*ePos[1] - K_RA*dPos[1]*dPos[1];
	if (bot->uGoal[1] > FORWARD_CONST) bot->uGoal[1] = copysign(1.0, bot->ePos[1]) * FORWARD_CONST;
	else bot->uGoal[1] *= copysign(1.0, bot->ePos[1]);
	bot->uGoal[0] = FORWARD_CONST - fabs(bot->uGoal[1]);
	memcpy(bot->ePos, ePos, sizeof(float) * 2);

	Bot_Motor_Control(bot);
}

// A* search algorithm
void Bot_Navigate(struct Bot * bot) {
	// stop first as this might take a while
	LATACLR = _LATA_LATA3_MASK;
	LATBCLR = _LATB_LATB4_MASK;
	OC5RS = 0x0;
	OC4RS = 0x0;
	
	// check if new exploration point needed
	if (getDistance(bot->pos, bot->explorePos) < MIN_GOAL_DIST) Bot_Explore(bot); 
	
	// build queues of nodes
	struct NodeQueue openQueue = {0};
	struct NodeQueue closedQueue = {0};
	struct Node initNode = { .h=getDistance(bot->explorePos, bot->pos), .posf={bot->pos[0], bot->pos[1]} };
	NodeQueue_Add(&openQueue, &initNode);
	struct Node * bestNode = &initNode;
	
	struct Node * node = NULL;
	int iter = 0;
	while (!NodeQueue_Empty(&openQueue) && iter++ < MAX_SEARCH_ITER) {
		unsigned int bestIndex = NodeQueue_Best(&openQueue);
		NodeQueue_Remove(&openQueue, &node, bestIndex);
		
		if (node) {
			NodeQueue_Add(&closedQueue, node);
			//Bot_UART_Node(bot, node);
			//delay(200000l);
			
			// check if goalNode
			if (node->h < MIN_SEARCH_GOAL) {
				Bot_Backtrack(bot, node);
				NodeQueue_Destroy(&openQueue);
				NodeQueue_Destroy(&closedQueue);
				return;
			} else if ((node->h + node->g) < (bestNode->h + bestNode->h)) bestNode = node;

			// expand neighbors
			for (uint8_t i = 0; i < 8; i++) {
				int newPosI[2] = { node->pos[0] + bPosMod[i][0], node->pos[1] + bPosMod[i][1] };
				char flag = NodeQueue_Contains(&closedQueue, newPosI);
				if (NodeQueue_Contains(&closedQueue, newPosI) == -1) {
					float newPosF[2] = { bot->pos[0] + NAV_STEP*newPosI[0], bot->pos[1] + NAV_STEP*newPosI[1] };
					float cost = (i % 2 == 0) ? NAV_SQRT : NAV_STEP;
					//cost += VISIT_COST * Map_Check_Visited()
					char fObs = 0;
					//if (node->g < 0.) {
						fObs = Map_Pos_Collide(bot->map, newPosF);
						uint8_t nIndex = 0;
						while (fObs == -1 && nIndex < 8) fObs = Map_Pos_Collide(bot->map->neighbors[nIndex++], newPosF);
						//Bot_UART_Write(bot, "fObs: %d\r\n", fObs);
						//delay(10000l);
					//}
					if (fObs < 1) {
						int openIndex = NodeQueue_Contains(&openQueue, newPosI);
						
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
	
	// if not found take best route available
	Bot_Backtrack(bot, bestNode);
	NodeQueue_Destroy(&openQueue);
	NodeQueue_Destroy(&closedQueue);
	
	// path not found, calculate new explore pos then recursive
	//Bot_UART_Write(bot, "path not found\r\n");
	//bot->state = (bot->state & ~STATE_MASK) | FINISH;
	//Bot_Explore(bot);
	//Bot_Navigate(bot);
}

void Bot_Backtrack(struct Bot * bot, struct Node * node) {
	//Bot_UART_Node(bot, node);
	//delay(1000000l);
	//Bot_UART_Map(bot);
	//Bot_UART_Write(bot, "bPos: (%.2f, %2f)\r\n");
	//delay(20000l);
	
	if (node == NULL) return;
	bot->goalIndex = GOAL_LEN;
	int count = 0;
	while (node->parent != NULL && bot->goalIndex >= 0) {
		memcpy(bot->goal[--bot->goalIndex], node->posf, sizeof(float) * 2);
		node = node->parent;
		//Bot_UART_Node(bot, node);
		//delay(100000l);
	}
	char msg[BUF_LEN] = "[";
	for (int i = bot->goalIndex; i < GOAL_LEN; i++) snprintf(msg, BUF_LEN, "%s[%.2f, %.2f],\r\n", msg, bot->goal[i][0], bot->goal[i][1]);
	snprintf(msg, BUF_LEN, "%s]\r\n", msg);
	Bot_UART_Write(bot, msg);
	//bot->state = (bot->state & ~STATE_MASK) | IDLE;
	delay(300000l);
}


// choose a new exploration position to maximize information gain
void Bot_Explore(struct Bot * bot) {
	// build edge map of map in view first, then neighbors
	LATASET = _LATA_LATA1_MASK;
	uint8_t fIndex[2] = {0};
	float fPos[2] = {0};
	
	//uint8_t minIndex = Bot_Map_View(bot);
	char fObs = Map_Frontier(bot->map, bot->pos, bot->explorePos, fIndex);
	if (fObs) memcpy(fPos, bot->map->pos, sizeof(float) * 2);
	else {
		uint8_t iter = 0;
		while (!fObs && iter < 8) fObs = Map_Frontier(bot->map->neighbors[iter++], bot->pos, bot->explorePos, fIndex);
		if (fObs) memcpy(fPos, bot->map->neighbors[iter-1]->pos, sizeof(float) * 2);
	}
		
	if (fObs) {
		bot->explorePos[0] = fPos[0] + fIndex[0]*MAP_RES;
		bot->explorePos[1] = fPos[1] - fIndex[1]*MAP_RES;
	} else {
		Bot_UART_Write(bot, "path not found\r\n");
		bot->state = (bot->state & ~STATE_MASK) | FINISH;
	}
	Bot_UART_Write(bot, "exPos: %.2f, %.2f\r\n", bot->explorePos[0], bot->explorePos[1]);
	delay(20000l);
	LATACLR = _LATA_LATA1_MASK;
}

/*
void Bot_Explore(struct Bot * bot) {
	for (uint8_t i = 0; i < MAP_UNITS; i++) {
		for (uint8_t j = 0; j < MAP_UNITS; j++) {
			if (Map_Check_Visited(bot->map, (uint8_t [2]){i, j})) {
				bot->explorePos[0] = bot->map->pos[0] + MAP_RES*i;
				bot->explorePos[1] = bot->map->pos[1] - MAP_RES*j;
				return;
			}
		}
	}
	Bot_UART_Write(bot, "path not found\r\n");
	bot->state = (bot->state & ~STATE_MASK) | FINISH;
}
 */

char Bot_Detect_Collision(struct Bot * bot) {
	if ((bot->state & STATE_MASK) != NAVIGATE) return 0;
	// check for collision
	if ((bot->uGoal[0] > V_REF && (bot->odo[0] + bot->odo[1]) < V_REF / 2) || (fabs(bot->uGoal[1]) > W_REF && fabs(bot->imu[2] < W_REF / 2))) bot->collideCount++;
	else bot->collideCount = 0;
	if (bot->collideCount >= MAX_COL_COUNT) {
		bot->collideCount = 0;
		Bot_Quick_Reverse(bot);
		return 1;
	} 
	
	/*
	// check if ultrasonic readings too close 
	if (bot->pos[3] > 0.1) {
		for (uint8_t i = 0; i < 3; i++)
			if (bot->dist[i] < MIN_OBST_DIST) return 1;
	} 

	// check for collision on trajectory - assuming constant velocity
	float xt[3] = { bot->pos[0], bot->pos[1], bot->pos[2] };
	for (uint8_t i = 0; i < 10; i++) {
		xt[2] += bot->pos[4]*DT_IMU;
		xt[0] += bot->pos[3]*cos(xt[2])*DT_IMU;
		xt[1] += bot->pos[3]*sin(xt[2])*DT_IMU;
		char fObs = Map_Pos_Collide(bot->map, xt);
		uint8_t nIndex = 0;
		//while (fObs == -1 && nIndex < 8) fObs = Map_Pos_Collide(bot->map->neighbors[nIndex++], xt);
		if (fObs == 1) return 1;
	}
	 * */

	return 0;
}

// check if next node is occupied with new information
char Bot_Path_Collision(struct Bot * bot) {
	uint8_t fObs = Map_Pos_Collide(bot->map, bot->goal[bot->goalIndex]);
	uint8_t fIndex = 0;
	while (fObs == -1 && fIndex < 8) fObs = Map_Pos_Collide(bot->map->neighbors[fIndex++], bot->goal[bot->goalIndex]);
	return (fObs != 0);
}

void Bot_Quick_Reverse(struct Bot * bot) {
	OC5RS = 0.88;
	OC4RS = 0.88;
	LATASET = _LATA_LATA3_MASK;
	LATBSET = _LATB_LATB4_MASK;
	
	delay(200000l);
	
	OC5RS = 0.0;
	OC4RS = 0.0;
	LATACLR = _LATA_LATA3_MASK;
	LATBCLR = _LATB_LATB4_MASK;
}

// print
void Bot_Display_Status(struct Bot * bot) {
	char status[4];
	switch (bot->state & STATE_MASK) {
		case INIT: strcpy(status, "INT"); break;
		case IDLE: strcpy(status, "IDL"); break;
		case NAVIGATE: strcpy(status, "NAV"); break;
		case FINISH: strcpy(status, "FIN"); break;
	}
	/*float pos[2] = {0};
	if (bot->goalIndex >= 0 && bot->goalIndex < GOAL_LEN) {
		pos[0] = bot->goal[bot->goalIndex][0];
		pos[1] = bot->goal[bot->goalIndex][1];
	}*/
	OLED_ClearDisplay();
	OLED_Write_Text(0, 0, "%s %ds %2.f%% %.2f %.2f", status, bot->time, bot->battery, bot->explorePos[0], bot->explorePos[1]);
	OLED_Write_Text(0, 10, "p %.2f %.2f %.f", bot->pos[0], bot->pos[1], bot->pos[2] * 180.0 / M_PI);
	OLED_Write_Text(0, 20, "e %.2f %.f %d", bot->ePos[0], bot->ePos[1]*180./M_PI, bot->goalIndex);
	OLED_Write_Text(0, 30, "d %0.2f %0.2f o %.1f %.1f", bot->duty[0], bot->duty[1], bot->odo[0], bot->odo[1]);
	OLED_Write_Text(0, 40, "us %.2f %.2f %.2f", bot->dist[0], bot->dist[1], bot->dist[2]);
	OLED_Write_Text(0, 50, "u %.2f %.2f", bot->uGoal[0], bot->uGoal[1]);// * 180. / M_PI);
	OLED_Update();
}

void Bot_Display_Map(struct Bot * bot) {
	char status[4];
	switch (bot->state & STATE_MASK) {
		case INIT: strcpy(status, "INT"); break;
		case IDLE: strcpy(status, "IDL"); break;
		case NAVIGATE: strcpy(status, "NAV"); break;
		case FINISH: strcpy(status, "FIN"); break;
	}
	
	OLED_ClearDisplay();
	OLED_FillRectangle(0, 0, 4*MAP_UNITS, 4*MAP_UNITS, WHITE);
	for (uint8_t i = 0; i < MAP_UNITS; i++) {
		for (uint8_t j = 0; j < MAP_UNITS; j++) {
			if (bot->map->grid[i][j] < 0) {
				OLED_FillRectangle(4*i, 4*j, 4*i+4, 4*j+4, BLACK);
				if (bot->map->grid[i][j] > -0.8) {
					OLED_FillRectangle(4*i+1, 4*j+1, 4*i+3, 4*j+3, WHITE); 
					OLED_DrawPixel(4*i+1, 4*j+1, BLACK);
				}
			} else if (bot->map->grid[i][j] < 0.8) {
				OLED_FillRectangle(4*i+1, 4*j+1, 4*i+3, 4*j+3, BLACK); 
				OLED_DrawPixel(4*i+1, 4*j+1, WHITE);
			}
		}
	}
	float pos[2] = {0};
	if (bot->goalIndex >= 0 && bot->goalIndex < GOAL_LEN) {
		pos[0] = bot->goal[bot->goalIndex][0];
		pos[1] = bot->goal[bot->goalIndex][1];
	}
	OLED_Write_Text(70, 0, "%.2f %.2f", bot->pos[0], bot->pos[1]);
	OLED_Write_Text(70, 10, "%.2f %.2f", bot->explorePos[0], bot->explorePos[1]);
	OLED_Write_Text(70 , 20, "%.2f %.2f", pos[0], pos[1]);
	OLED_Write_Text(70, 30, "%.2f, %.2f", bot->dist[0], bot->dist[1]);
	OLED_Write_Text(70, 40, "%.2f %s", bot->dist[2], status);
	OLED_Update();
}

void Bot_UART_Status(struct Bot * bot) {
    // clear screen: "\33[2J\33[H"
	Bot_UART_Write(bot,
			"t: %d\r\n"
			"p: (%.2f, %.2f, %.f, %.2f, %.f)\r\n"
			"e: (%.2f, %.f)\r\n"
			"o: (%.2f, %.2f)\r\n"
			"i: (%.2f, %.2f, %.f)\r\n"
			"u: (%.2f, %.2f)\r\n"
			"dc: (%.2f, %.2f)\r\n\n",
			bot->time,
			bot->pos[0], bot->pos[1], bot->pos[2]*180./M_PI, bot->pos[3], bot->pos[4]*180./M_PI,
			bot->ePos[0], bot->ePos[1]*180./M_PI,
			bot->odo[0], bot->odo[1],
			bot->imu[0], bot->imu[1], bot->imu[2]*180./M_PI,
			bot->uGoal[0], bot->uGoal[1],
			bot->duty[0], bot->duty[1]
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
	/*char msg[BUF_LEN];
	snprintf(msg, BUF_LEN, "\r\nmap: (%.2f, %.2f)\r\n", bot->map->pos[0], bot->map->pos[1]);
	
	for (uint8_t i = 0; i < 8; i++)
		if (bot->map->neighbors[i])
			snprintf(msg, BUF_LEN, "%sn(%d): (%.2f, %.2f)\r\n", msg, i, bot->map->neighbors[i]->pos[0], bot->map->neighbors[i]->pos[1]);
	Bot_UART_Write(bot, msg);*/

	//Bot_UART_Write(bot, "\r\nm(%d): (%.2f, %.2f)\r\n", -1, bot->map->pos[0], bot->map->pos[1]);
	//delay(50000l);
	Mat_Print(bot, MAP_UNITS, MAP_UNITS, bot->map->grid, "");
	delay(4000000l);
	//Mat_Print(bot, MAP_UNITS, MAP_UNITS, bot->map->neighbors[3]->grid, "");
	//delay(4000000l);
	/*for (uint8_t i = 0; i < 8; i++) {
		if (bot->map->neighbors[i]) {
			Bot_UART_Write(bot, "\r\nm(%d):(%.2f, %.2f)\r\n", i, bot->map->neighbors[i]->pos[0], bot->map->neighbors[i]->pos[1]);
			delay(50000l);
			Mat_Print(bot, MAP_UNITS, MAP_UNITS, bot->map->neighbors[i]->grid, "");
			delay(4000000l);
		}
	}*/
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
	snprintf(msg, BUF_LEN, "%s: [", title);
	for (uint8_t i = 0; i < rows; i++) {
		snprintf(msg, BUF_LEN, "%s[", msg);
		for (uint8_t j = 0; j < cols; j++) {
			snprintf(msg, BUF_LEN, "%s%.1f, ", msg, mat[i][j]);
		}
		snprintf(msg, BUF_LEN, "%s],\r\n", msg);
	}
	snprintf(msg, BUF_LEN, "%s]\r\n", msg);
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

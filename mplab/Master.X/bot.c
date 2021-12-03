 #include "bot.h"

// bot init
void Bot_Init(struct Bot ** bot, float pos[3]) {
	struct Bot temp = {0};
	*bot = malloc(sizeof(struct Bot));
	**bot = temp;
	memcpy((*bot)->pos, pos, sizeof(float) * 3);
	float mapPos[2] = { pos[0] - MAP_SIZE / 2, pos[1] + MAP_SIZE / 2 };
	Map_Init(&((*bot)->map), mapPos);
	(*bot)->numMaps++;
	Bot_Map_Required(*bot);
	(*bot)->goalIndex = GOAL_LEN - 1;
	(*bot)->terrainMod = 0.5;
	
	for (uint8_t i = 0; i < ODO_LEN; i++)
		(*bot)->wOdo[i] = W_ODO / ((i+1)*(i+1));
	
	// temporary configurations
	//float queue[2][2] = { {, 0.0}, {1, 0.0} };//{0.3, 0.0}, {0.3, 0.3}, {0., 0.3}, {0.0, 0.0} };
	//Bot_Goal_Queue(*bot, 2, queue);
	//(*bot)->explorePos[0] = 0.8;
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
	
	uint8_t viewIndex = Bot_Map_View(bot);
	if (viewIndex != -1) {
		viewIndex = (viewIndex + 7) % 8;
		for (uint8_t i = 0; i < 3; i++) {
			if (!bot->map->neighbors[viewIndex]) {
				float pos[2] = { bot->map->pos[0] + posMod[viewIndex][0] * MAP_SIZE, bot->map->pos[1] + posMod[viewIndex][1] * MAP_SIZE };
				Map_Init(&(bot->map->neighbors[viewIndex]), pos);
				bot->numMaps++;
				Map_Reinforce(bot->map->neighbors[viewIndex]);
			}
			viewIndex = (viewIndex + 1) % 8;
		}
	}
}

void Bot_Map_Update(struct Bot * bot) {
	Map_Update(bot->map, bot->pos, bot->dist);
	
	// only map this map and three maps in view
	if (!bot->map) return;
	uint8_t viewIndex = Bot_Map_View(bot);
	if (viewIndex != -1) {
		Map_Update(bot->map->neighbors[viewIndex], bot->pos, bot->dist);
		Map_Update(bot->map->neighbors[(viewIndex + 1) % 8], bot->pos, bot->dist);
		Map_Update(bot->map->neighbors[(viewIndex + 7) % 8], bot->pos, bot->dist);
	}
}

void Bot_Bias(struct Bot * bot) {
	switch (bot->state) {
		case INIT:
		case IDLE: {
			for (uint8_t i = 0; i < 4; i++) bot->bias[i] += bot->imu[i];
			bot->numBias++;
			break;
		}
		case NAVIGATE:
		case REVERSE: {
			for (uint8_t i = 0; i < 4; i++) bot->imu[i] -= bot->bias[i];
			break;
		}
	}
}

// obtain map index in direction of bot's view
uint8_t Bot_Map_View(struct Bot * bot) {
	if (!bot->map) return -1;
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
void Bot_Pos_Update(struct Bot * bot) {
	IMU_Read(bot->imu, bot->asa);
	Bot_Bias(bot);
	//Odometer_Read(&bot->odo, FREQ);
	
	Odometer_Read(&bot->odoArr[bot->odoIndex], FREQ);	
	bot->odo[0] = 0;
	bot->odo[1] = 0;
	for (uint8_t i = 0; i < ODO_LEN; i++) {
		uint8_t index = (bot->odoIndex + i) % ODO_LEN;
		bot->odo[0] += bot->odoArr[index][0] * bot->wOdo[i];
		bot->odo[1] += bot->odoArr[index][1] * bot->wOdo[i];
	}
	bot->odoIndex = (bot->odoIndex + 1) % ODO_LEN;
	bot->odo[0] *= copysign(1.0, bot->duty[0]);
	bot->odo[1] *= copysign(1.0, bot->duty[1]);
	
	if (bot->state != NAVIGATE && bot->state != REVERSE) return;

	// speedier solution
	bot->pos[3] =  0.5*(bot->odo[1] + bot->odo[0]);
	bot->pos[4] = 0.5*((bot->odo[0] - bot->odo[1]) / CHASSIS_L + bot->imu[2]);
	Bot_Motion_Model(bot->pos, DT);
	//bot->pos[2] = (1 - K_MAG) * bot->pos[2] + K_MAG * bot->imu[2];
	Bot_Map_Required(bot);
	Map_Update_Visit(bot->map, bot->pos);
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

void Bot_Motion_Model(float x[UKF_N], float dt) {
	x[2] = normAngle(x[2] + x[4]*dt);
	x[0] += x[3]*cos(x[2])*dt;
	x[1] += x[3]*sin(x[2])*dt;
}

// motor controller at 40 Hz
void Bot_Motor_Control(struct Bot * bot) {
	if (bot->state == REVERSE) return;
	if (bot->state == NAVIGATE) {
		//float e[2] = { bot->uGoal[0] - bot->uGoal[1] - bot->odo[0], bot->uGoal[0] + bot->uGoal[1] - bot->odo[1] };
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
	if (bot->state != NAVIGATE) {
		Bot_Motor_Control(bot);
		return;
	}

	// check if new path needed
	if (getDistance(bot->goal[bot->goalIndex], bot->pos) < MIN_GOAL_DIST) {
		bot->goalIndex++;
		if (Bot_Path_Collision(bot)) Bot_Explore(bot);
		else Bot_UART_Write(bot, "new pos: (%.2f, %.2f)\r\n", bot->goal[bot->goalIndex][0], bot->goal[bot->goalIndex][1]);
		return;
	}

	// obtain current and integral errors
	float ePos[2] = { getDistance(bot->pos, bot->goal[bot->goalIndex]), bot->ePos[1] = getAngle(bot->pos, bot->goal[bot->goalIndex]) };
	float dPos[2] = { ePos[0] - bot->ePos[0], normAngle(ePos[1] - bot->ePos[1]) };
	
	bot->uGoal[1] = K_RO*ePos[1]*ePos[1] - K_RA*dPos[1]*dPos[1];
	if (bot->uGoal[1] > FORWARD_CONST) bot->uGoal[1] = copysign(1.0, bot->ePos[1]) * FORWARD_CONST;
	else bot->uGoal[1] *= copysign(1.0, bot->ePos[1]);
	bot->uGoal[0] = FORWARD_CONST - fabs(bot->uGoal[1]);
	memcpy(bot->ePos, ePos, sizeof(float) * 2);

	Bot_Motor_Control(bot);
}

// A* search algorithm
char Bot_Navigate(struct Bot * bot) {
	//bot->goalIndex = GOAL_LEN - 1;
	//memcpy(bot->goal[bot->goalIndex], bot->explorePos, sizeof(float) *2);

	// build queues of nodes
	struct NodeQueue openQueue = {0};
	struct NodeQueue closedQueue = {0};
	struct Node initNode = { .h=getDistance(bot->explorePos, bot->pos), .posf={bot->pos[0], bot->pos[1]} };
	NodeQueue_Add(&openQueue, &initNode);
	struct Node * bestNode = &initNode;
	
	struct Node * node = NULL;
	int iter = 0;
	while (!NodeQueue_Empty(&openQueue) && iter++ <= MAX_SEARCH_ITER) {
		unsigned int bestIndex = NodeQueue_Best(&openQueue);
		NodeQueue_Remove(&openQueue, &node, bestIndex);
		
		if (node) {
			if (!NodeQueue_Add(&closedQueue, node)) {
				iter = MAX_SEARCH_ITER;
				break;
			}
			//Bot_UART_Node(bot, node);
			//delay(200000l);
			
			// check if goalNode
			if (node->h < MIN_SEARCH_GOAL) {
				struct Node endNode = {
					.g=(node->g + getDistance(bot->explorePos, node->posf)),
					.posf={bot->explorePos[0], bot->explorePos[1]},
					.parent=node
				};
				bot->noPathCount = 0;
				Bot_Backtrack(bot, &endNode);
				NodeQueue_Destroy(&openQueue);
				NodeQueue_Destroy(&closedQueue);
				return 1;
			} else if ((node->g + node->h) < (bestNode->g + bestNode->h)) bestNode = node;

			// expand neighbors
			for (uint8_t i = 0; i < 8; i++) {
				int newPosI[2] = { node->pos[0] + bPosMod[i][0], node->pos[1] + bPosMod[i][1] };
				char flag = NodeQueue_Contains(&closedQueue, newPosI);
				if (NodeQueue_Contains(&closedQueue, newPosI) == -1) {
					float newPosF[2] = { initNode.posf[0] + NAV_STEP*newPosI[0], initNode.posf[1] + NAV_STEP*newPosI[1] };
					char fObs = Map_Pos_Collide(bot->map, newPosF);
					uint8_t nIndex = -1;
					while (fObs == -1 && nIndex < 8) fObs = Map_Pos_Collide(bot->map->neighbors[nIndex++], newPosF);
					if (fObs < 1) {
						char visited = (nIndex == -1) ? Map_Check_Visited(bot->map, newPosF) : ((nIndex < 8) ? Map_Check_Visited(bot->map->neighbors[nIndex], newPosF) : 0);
						float cost = (i % 2 == 0) ? NAV_SQRT : NAV_STEP;
						cost += visited*VISIT_COST;
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
							if (!NodeQueue_Add(&openQueue, &newNode)) {
								iter = MAX_SEARCH_ITER;
								break;
							}
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
	bot->noPathCount++;
	Bot_Backtrack(bot, bestNode);
	NodeQueue_Destroy(&openQueue);
	NodeQueue_Destroy(&closedQueue);
	return 0;
	
	// path not found, calculate new explore pos then recursive
	//Bot_UART_Write(bot, "path not found\r\n");
	//bot->state = (bot->state & ~STATE_MASK) | FINISH;
	//Bot_Explore(bot);
	//Bot_Navigate(bot);
}

void Bot_Backtrack(struct Bot * bot, struct Node * node) {
	if (node == NULL) return;
	bot->goalIndex = GOAL_LEN;
	int count = 0;
	while (node->parent != NULL && bot->goalIndex >= 0) {
		memcpy(bot->goal[--bot->goalIndex], node->posf, sizeof(float) * 2);
		node = node->parent;
	}
	/*char msg[BUF_LEN] = "[";
	for (int i = bot->goalIndex; i < GOAL_LEN; i++) snprintf(msg, BUF_LEN, "%s[%.2f, %.2f],\r\n", msg, bot->goal[i][0], bot->goal[i][1]);
	snprintf(msg, BUF_LEN, "%s]\r\n", msg);
	Bot_UART_Write(bot, msg);*/
}


// choose a new exploration position to maximize information gain
void Bot_Explore(struct Bot * bot) {
	if (bot->noPathCount > 20) {
		bot->state = FINISH;
		return;
	}
	bot->state = IDLE;
	Bot_Motor_Control(bot);
	if (bot->exploreCount++ > EXPOS_LEN) {
		bot->exploreCount = 0;
		Bot_Sweep(bot);
	} else if (!Map_Frontier(bot->map, bot->pos, bot->explorePos)) {
		Bot_UART_Write(bot, "path not found\r\n");
		//bot->state = FINISH;
		Bot_Sweep(bot);
	}
	//bot->explorePos[0] = (bot->explorePos[0]) ? 0.0 : 0.6;
	Bot_Navigate(bot);//) bot->state = IDLE;
	bot->state = NAVIGATE;
}

// sweep complete area
void Bot_Sweep(struct Bot * bot) {
	if (!bot->map) return;
	for (uint8_t i = 0; i < 30; i++) {
		uint8_t index[3] = { rand() % 9, rand() % MAP_UNITS, rand() % MAP_UNITS };
		struct Map * sweepMap;
		if (index[0]) sweepMap = bot->map->neighbors[index[0] - 1];
		else sweepMap = bot->map;
		Map_IndexToPos(sweepMap, bot->explorePos, &index[1]);
		if (!Map_Check_Visited(sweepMap, bot->explorePos)) return;
	}
}

char Bot_Detect_Collision(struct Bot * bot) {
	if (bot->state != NAVIGATE) return 0;
	// check for collision
	//float e[2] = { fabs(bot->uGoal[0] - bot->uGoal[1] - bot->odo[0]), fabs(bot->uGoal[0] + bot->uGoal[1] - bot->odo[1]) };
	if (fabs(bot->odo[0]) < ERR_REF && fabs(bot->odo[1]) < ERR_REF) {
		bot->collideCount++;
		LATASET = _LATA_LATA1_MASK;
	} else {
		bot->inReverse = 1;
		bot->collideCount = 0;
		LATACLR = _LATA_LATA1_MASK;
	}
	if (bot->collideCount >= MAX_COL_COUNT) {
		bot->collideCount = 0;
		return 1;
	} 
	
	
	// check if ultrasonic readings too close 
	/*if (bot->pos[3] > 0.1 && bot->count % FREQ == 0) {
		for (uint8_t i = 0; i < 3; i++)
			if (bot->dist[i] < MIN_OBST_DIST)  return 1;
	}*/

	/*
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
	if (bot->state == NAVIGATE && bot->goalIndex >= GOAL_LEN) return 1;
	uint8_t fObs = Map_Pos_Collide(bot->map, bot->goal[bot->goalIndex]);
	//uint8_t fIndex = 0;
	//while (fObs == -1 && fIndex < 8) fObs = Map_Pos_Collide(bot->map->neighbors[fIndex++], bot->goal[bot->goalIndex]);
	return (fObs == 1);
}

void Bot_Quick_Reverse(struct Bot * bot) {
	bot->state = REVERSE;
	if (bot->inReverse) {
		bot->duty[0] = -0.05;
		bot->duty[1] = -0.05;
		OC5RS = (int)(REVERSE_SPEED * PWM_T);
		OC4RS = (int)(REVERSE_SPEED * PWM_T);
		LATASET = _LATA_LATA3_MASK;
		LATBSET = _LATB_LATB4_MASK;
	} else {
		bot->duty[0] = 0.05;
		bot->duty[1] = 0.05;
		OC5RS = (int)((1 - REVERSE_SPEED) * PWM_T);
		OC4RS = (int)((1 - REVERSE_SPEED) * PWM_T);
		LATACLR = _LATA_LATA3_MASK;
		LATBCLR = _LATB_LATB4_MASK;
	}
	bot->reverseCount = 5;
}

void Bot_Stop_Reverse(struct Bot * bot) {
	OC5RS = 0;
	OC4RS = 0;
	LATACLR = _LATA_LATA3_MASK;
	LATBCLR = _LATB_LATB4_MASK;
	bot->inReverse ^= 1;
	bot->reverseCount = 0;
	
	Bot_Explore(bot);
	bot->state = NAVIGATE;
}


void Bot_Vacuum(struct Bot * bot, char turnOn) {
	if (turnOn) OC2RS = (int)(VACUUM_SPEED * PWM_T);
	else OC2RS = 0x0;
}

// print
void Bot_Display_Status(struct Bot * bot) {
	char status[4];
	switch (bot->state) {
		case INIT: strcpy(status, "INT"); break;
		case IDLE: strcpy(status, "IDL"); break;
		case NAVIGATE: strcpy(status, "NAV"); break;
		case REVERSE: strcpy(status, "REV"); break;
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
	switch (bot->state) {
		case INIT: strcpy(status, "INT"); break;
		case IDLE: strcpy(status, "IDL"); break;
		case NAVIGATE: strcpy(status, "NAV"); break;
		case REVERSE: strcpy(status, "REV"); break;
		case FINISH: strcpy(status, "FIN"); break;
	}
	
	// obtain 4 view maps
	char val = Bot_Map_View(bot);
	if (val != -1) {
		uint8_t viewIndex = 2*(val / 2);
		uint8_t mapIndex = (viewIndex/2 + 2) % 4;
		struct Map * mapArr[4];
		mapArr[mapIndex] = bot->map;
		mapArr[(mapIndex + 1) % 4] = bot->map->neighbors[(viewIndex+7)%8];
		mapArr[(mapIndex + 2) % 4] = bot->map->neighbors[viewIndex];
		mapArr[(mapIndex + 3) % 4] = bot->map->neighbors[(viewIndex+1)%8];

		uint8_t goalIndex[2] = {-1};
		if (bot->goalIndex < GOAL_LEN) Map_PosToIndex(bot->map, goalIndex, bot->explorePos);

		OLED_ClearDisplay();
		OLED_FillRectangle(0, 0, 4*MAP_UNITS, 4*MAP_UNITS, WHITE);
		for (uint8_t i = 0; i < 4; i++) {
			// find starting position
			uint8_t y = i / 2;
			uint8_t x = ((i - y) % 2) * 2*MAP_UNITS;
			y *= 2*MAP_UNITS;

			// colour submap
			if (mapArr[i] && mapArr[i]->grid) {
				for (uint8_t j = 0; j < MAP_UNITS; j++) {
					for (uint8_t k = 0; k < MAP_UNITS; k++) {
						if (mapArr[i]->grid[j][k] < 1.0) OLED_DrawPixel(x + 2*j, y + 2*k, BLACK);
						if (mapArr[i]->grid[j][k] < 0.3) OLED_DrawPixel(x + 2*j + 1, y + 2*k + 1, BLACK);
						if (mapArr[i]->grid[j][k] < -0.3) OLED_DrawPixel(x + 2*j + 1, y + 2*k, BLACK);
						if (mapArr[i]->grid[j][k] < -1.0) OLED_DrawPixel(x + 2*j, y + 2*k + 1, BLACK);
					}
				}
			}

			// plot bot and goal position
			if (i == mapIndex) {
				if (goalIndex[0] != -1) {
					OLED_Rectangle(x + 2*goalIndex[0]-2, y + 2*goalIndex[1]-2, x + 2*goalIndex[0]+2, y + 2*goalIndex[1]+2, WHITE);
					OLED_Rectangle(x + 2*goalIndex[0]-1, y + 2*goalIndex[1]-1, x + 2*goalIndex[0]+1, y + 2*goalIndex[1]+1, BLACK);
				}
				uint8_t botIndex[2];
				Map_PosToIndex(bot->map, botIndex, bot->pos);
				OLED_Circle(x + 2*botIndex[0], y + 2*botIndex[1], 2, WHITE);
				OLED_Circle(x + 2*botIndex[0], y + 2*botIndex[1], 1, BLACK);
			}
		}
	}
	
	float pos[2] = {0};
	if (bot->goalIndex >= 0 && bot->goalIndex < GOAL_LEN) {
		pos[0] = bot->goal[bot->goalIndex][0];
		pos[1] = bot->goal[bot->goalIndex][1];
	}
	OLED_Write_Text(66, 0, "%s %d %.f%%", status, bot->time, bot->battery);
	OLED_Write_Text(66, 10, "%.2f %.2f", bot->pos[0], bot->pos[1]);
	//OLED_Write_Text(66, 20, "%.f %d", bot->pos[2]*180.0/M_PI, bot->numMaps);
	OLED_Write_Text(66, 20, "%.2f %.2f", bot->explorePos[0], bot->explorePos[1]);
	OLED_Write_Text(66 , 30, "%.2f %.2f", pos[0], pos[1]);
	//OLED_Write_Text(66, 40, "%.2f %.2f", bot->dist[0], bot->dist[1]);
	//OLED_Write_Text(66, 50, "%.2f %d", bot->dist[2], bot->numMaps);
	OLED_Write_Text(66, 40, "%.2f %.2f", bot->duty[0], bot->duty[1]);
	//OLED_Write_Text(66, 40, "%.2f %.2f", bot->odo[0], bot->odo[1]);
	OLED_Write_Text(66, 50, "%.2f %.2f", bot->uGoal[0], bot->uGoal[1]);
	
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
	char oldState = bot->state;
	bot->state = IDLE;
	Bot_Motor_Control(bot);
	Bot_UART_Write(bot, "bPos = [%.2f, %.2f], exPos = [%.2f, %.2f]\r\n", bot->pos[0], bot->pos[1], bot->explorePos[0], bot->explorePos[1]);
	delay(50000l);
	Mat_Print(bot, MAP_UNITS, MAP_UNITS, bot->map->grid, "g");
	delay(4000000l);
	Mat_Print_Char(bot, MAP_UNITS, MAP_UNITS_BIT, bot->map->visited, "v");
	delay(1000000l);
	bot->state = oldState;
	if (bot->goalIndex == GOAL_LEN || Bot_Path_Collision(bot)) Bot_Navigate(bot);
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


void Mat_Print_Char(struct Bot * bot, uint8_t rows, uint8_t cols, char mat[rows][cols], char * title) {
	char msg[BUF_LEN];
	snprintf(msg, BUF_LEN, "%s: [", title);
	for (uint8_t i = 0; i < rows; i++) {
		snprintf(msg, BUF_LEN, "%s[", msg);
		for (uint8_t j = 0; j < cols; j++) {
			snprintf(msg, BUF_LEN, "%s%d, ", msg, mat[i][j]);
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

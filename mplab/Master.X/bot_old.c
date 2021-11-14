#include "bot.h"

void Bot_Pos_Update(struct Bot * bot, uint8_t startIndex) {
	IMU_Read(bot->gyro, bot->acc, bot->mag, bot->asa);

	// estimate bias in initialization
	if ((bot->state & STATE_MASK) == INIT) {
	for (uint8_t i = 0; i < 3; i++) {
		bot->bias[i] += bot->gyro[i];
		bot->bias[3+i] += bot->acc[i];
	}	//float distance = Bot_InputPos_Update(bot);
	float minDist = MAX_US_DIST;
	bot->numBias++;
	return;
	} else if ((bot->state & STATE_MASK) == IDLE) return;

	 // remove bias and filter magnetometer - assumed to be static
	for (uint8_t i = 0; i < 3; i++) {
		bot->gyro[i] -= bot->bias[i];
		bot->acc[i] -= bot->bias[3+i];
	}    
	uint8_t n = (startIndex + 1 < FIR_N) ? startIndex + 1: FIR_N;
	for (uint8_t i = 0; i < n - 1; i++) bot->firX[i+1] = bot->firX[i];
	bot->firX[0] = atan2(bot->mag[1], bot->mag[0]);
	float wMag = Bot_FIR_Filter(bot, n);

	// fuse wheel angular rate from estimate and odometer
	float wDc[2] = { bot->duty[0]*VEL_MAX, bot->duty[1]*VEL_MAX };
	float wOdo[2] = { copysign(1.0, wDc[0]) * bot->odo[0]*ODO_SCALE, copysign(1.0, wDc[1]) * bot->odo[1]*ODO_SCALE };
	float wVel[2] = {
		(K_ODO*wOdo[0] + (1 - K_ODO)*wDc[0])*WHEEL_RADIUS,
		(K_ODO*wOdo[1] + (1 - K_ODO)*wDc[1])*WHEEL_RADIUS
	};

	// fuse velocity from estimate and accelerometer
	bot->vEst[0] = ODO_WEIGHT * 0.5 * (cos(bot->pos[2])*wVel[0] + cos(bot->pos[2])*wVel[1]) + (1 - ODO_WEIGHT) * bot->vEst[0];
	bot->vEst[1] = ODO_WEIGHT * 0.5 * (sin(bot->pos[2])*wVel[0] + sin(bot->pos[2])*wVel[1]) + (1 - ODO_WEIGHT) * bot->vEst[0];
	bot->vEst[2] = ODO_WEIGHT * (wVel[1] - wVel[0]) / CHASSIS_LENGTH + (1 - ODO_WEIGHT) * bot->vEst[0];
	if (fabs(bot->gyro[2]) < ERROR_COLLIDE && fabs(bot->vEst[2]) > 2*ERROR_COLLIDE) bot->collisionCount[0]++;
	else bot->collisionCount[0] = 0;

	float vAcc[2] = { 
		bot->vel[0] + bot->acc[0]*cos(bot->pos[2])*DT - bot->acc[1]*sin(bot->pos[2])*DT,
		bot->vel[1] + bot->acc[1]*cos(bot->pos[2])*DT + bot->acc[0]*sin(bot->pos[2])*DT
	};
	bot->vel[0] = K_ACC*vAcc[0] + (1 - K_ACC)*bot->vEst[0];
	bot->vel[1] = K_ACC*vAcc[1] + (1 - K_ACC)*bot->vEst[1];

	// fuse angular rate from estimate, gyroscope and magnetometer

	bot->vel[2] = K_GYRO*bot->gyro[2] + (1 - K_GYRO)*bot->vEst[2];

	// estimate new position
	for (uint8_t i = 0; i < 3; i++) bot->pos[i] += bot->vel[i] * DT;
	//bot->pos[2] = K_MAG*wMag + (1 - K_MAG)*bot->pos[2];
	bot->pos[2] = normAngle(bot->pos[2]);
}

void Bot_Controller(struct Bot * bot) { 
    if (!bot->qCurr) return;
    float convDuty[2] = {0};		// convert duty cycle to correct value
    // obtain error angle and distance
    float goalVec[2] = { bot->qCurr->pos[0] - bot->pos[0], bot->qCurr->pos[1] - bot->pos[1] };
    if (goalVec[0] == 0) goalVec[0] += 0.001;
    float ePos[2] = {
		getDistance(bot->qCurr->pos, bot->pos),
		normAngle(atan2(goalVec[1], goalVec[0]) - bot->pos[2])
    };
        
    if ((bot->state & STATE_MASK) == NAVIGATE) {
	// NAVIGATE state: determine duty cycle 

	// determine minimum and maximum distance
	bot->ePosInt[0] += ePos[0] * DT;
	bot->ePosInt[1] += ePos[1] * DT;

	    if (ePos[0] > MIN_DIST) {
		    // pi controller   
		    float rotSign = copysign(1.0, ePos[1]);
		    if (fabs(ePos[1]) < ERROR_MIN) {
			    float p = ePos[1] * (K_OFFSET / ERROR_MIN);
			    float i = 0;//bot->ePosInt[1] * (K_INT / ERROR_MIN);
			    bot->duty[0] = K_OFFSET - p - i;// - (K_DIST * ePos[0]) / (ePos[0] + 0.005);
			    bot->duty[1] = K_OFFSET + p + i;// - (K_DIST * ePos[0]) / (ePos[0] + 0.005);
		    } else if (fabs(ePos[1]) < ERROR_MAX) {
			    bot->duty[0] = -rotSign * K_OFFSET;
			    bot->duty[1] = rotSign * K_OFFSET;
		    } else {
			    float p = normAngle(M_PI - ePos[1]) * (K_OFFSET / ERROR_MIN);
			    float i = 0;//normAngle(M_PI - bot->ePosInt[1]) * (K_INT / ERROR_MIN);
			    bot->duty[0] = -K_OFFSET + p + i;
			    bot->duty[1] = -K_OFFSET - p - i;
		    }
	    }

	    /* negative duty cycles should reverse rotation */
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
    } else {
	/* IDLE state: do not move */
	for (int i = 0; i < 2; i++) {
	bot->duty[i] = 0.0;
	convDuty[i] = 0.0;
	}
	LATBCLR = _LATB_LATB4_MASK;
	LATACLR = _LATA_LATA3_MASK;
    }
    
    bot->ePos[0] = ePos[0];
    bot->ePos[1] = ePos[1];
    OC4RS = (int)(convDuty[0] * PWM_T);
    OC5RS = (int)(convDuty[1] * PWM_T);
}

/* add areas cleaned to currentMaps */
void Bot_Map_Update(struct Bot * bot, uint8_t sensorIndex) {
	// check if out of map bounds
	uint8_t index[2];
	while (!BitMap_Contains(bot->currentMaps[0], index, bot->pos)) Bot_Map_Required(bot);

	// add new obstruction in sensor reading direction
	float vel = 0.5*(bot->vel[0] + bot->vel[1]);
    if (bot->dist[sensorIndex] < (MIN_OBST_DIST + K_VEL * vel)) {
    	uint8_t dirIndex = ((int)(8.5 - (bot->pos[2] + DIR_OFFSET) / DIR_DIV) + sensorIndex) % 8;
    	BitMap_SetRelative(bot, dirIndex, 0x11);
    }
    
	// set area cleaned
	if (bot->posIndex[0] != index[0] || bot->posIndex[1] != index[1]) {
		memcpy(bot->posIndex, index, sizeof(uint8_t) * 2);
		BitMap_Set(bot->currentMaps[0], bot->posIndex, 0x10);
	}
}

/* determine which maps will form part of supermap 
 * create maps if necessary */
void Bot_Map_Required(struct Bot * bot) {
    // update the current map based on the position of the bot
	if (!BitMap_Contains(bot->currentMaps[0], bot->posIndex, bot->pos)) {
		for (uint8_t i = 0; i < 8; i++) {
		    if (bot->currentMaps[0]->neighbors[i] && BitMap_Contains(bot->currentMaps[0]->neighbors[i], bot->posIndex, bot->pos)) {
			bot->currentMaps[0] = bot->currentMaps[0]->neighbors[i];
			break;
		    }
		}
	}
	
	// create surrounding currentmaps[0] and add to localmap
	for (uint8_t i = 0; i < 8; i++) {
		if (!bot->currentMaps[0]->neighbors[i]) {
			float pos[2] = { bot->currentMaps[0]->pos[0] + bot->posModifier[i][0] * MAP_SIZE, bot->currentMaps[0]->pos[1] + bot->posModifier[i][1] * MAP_SIZE };
			BitMap_Initialize(bot, &bot->currentMaps[0]->neighbors[i], pos);
			Bot_Reinforce_Neighbors(bot->currentMaps[0]->neighbors[i]);
		}
		bot->currentMaps[i+1] = bot->currentMaps[0]->neighbors[i];
	}
}

void Bot_Reinforce_Neighbors(struct BitMap * map) {    
	/* check if existing neighbor(s) has additional neighbors */
	for (char i = 0; i < 8; i++) {
		if (map->neighbors[i]) {
			char mIndex = (i + 4) % 8;	// my index relative to neighbor
			if (i % 2) {
				if (map->neighbors[i]->neighbors[(mIndex + 7) % 8]) {
					map->neighbors[(i + 2) % 8] = map->neighbors[i]->neighbors[(mIndex + 7) % 8];
					map->neighbors[i]->neighbors[(mIndex + 7) % 8]->neighbors[(i + 6) % 8] = map;
				}
				if (map->neighbors[i]->neighbors[(mIndex + 1) % 8]) {
					map->neighbors[(i + 6) % 8] = map->neighbors[i]->neighbors[(mIndex + 1) % 8];
					map->neighbors[i]->neighbors[(mIndex + 1) % 8]->neighbors[(i + 2) % 8] = map;
				}
				if (map->neighbors[i]->neighbors[(mIndex + 6) % 8]) {   
					map->neighbors[(i + 1) % 8] = map->neighbors[i]->neighbors[(mIndex + 6) % 8];
					map->neighbors[i]->neighbors[(mIndex + 6) % 8]->neighbors[(i + 5) % 8] = map;
				}
				if (map->neighbors[i]->neighbors[(mIndex + 2) % 8]) {
					map->neighbors[(i + 7) % 8] = map->neighbors[i]->neighbors[(mIndex + 2) % 8];
					map->neighbors[i]->neighbors[(mIndex + 2) % 8]->neighbors[(i + 3) % 8] = map;
				}
				} else {
				if (map->neighbors[i]->neighbors[(mIndex + 7) % 8]) {
					map->neighbors[(i + 1) % 8] = map->neighbors[i]->neighbors[(mIndex + 7) % 8];
					map->neighbors[i]->neighbors[(mIndex + 7) % 8]->neighbors[(i + 5) % 8] = map;
				}
				if (map->neighbors[i]->neighbors[(mIndex + 1) % 8]) {
					map->neighbors[(i + 7) % 8] = map->neighbors[i]->neighbors[(mIndex + 1) % 8];
					map->neighbors[i]->neighbors[(mIndex + 1) % 8]->neighbors[(i + 3) % 8] = map;
				}
			}
		}
	}
}

/* plan paths based on environment information 
void Bot_Navigate(struct Bot * bot) {
	// evaluate if new goal needed
	
	
	// determine if obstacle in goal's way
	float goalVec[2] = { bot->goal[0][0] - bot->pos[0], bot->goal[0][1] - bot->pos[1] };
	float errAngles[8];
	char obsFlag = 0;
	for (uint8_t i = 0; i < 8; i++) {
		errAngles[i] = getAngles(goalVec[0], goalVec[1], bot->posModifier[i][0], bot->posModifier[i][1]);
		if (BitMap_At(bot-> errAngles[i] < ERROR_COLLIDE) obsFlag = 1;
	}
    
    bot->obsAngles[0] = 0;
    bot->obsAngles[1] = 0;
    float sign = 1.0;
    for (uint8_t i = 0; i < US_SENSORS; i++) {
	if ((bot->obstructed >> i) & 0x1) bot->obsAngles[0] = bot->sensorOffsets[i] - 2*SENSOR_ANGLE;
	if ((bot->obstructed >> (2 - i)) & 0x1) bot->obsAngles[1] = bot->sensorOffsets[2 - i] + 2*SENSOR_ANGLE;
    }
    if ((bot->obstructed & 0x5) == 0x0 || (bot->obstructed & 0x5) == 0x5) sign = copysign(1.0, rand() - RAND_MAX);
    if (bot->ePos[0] < MIN_GOAL_DIST || (bot->ePos[1] > (bot->obsAngles[0] - 9) && bot->ePos[1] < (bot->obsAngles[1] - change))) {
	// determine new goal
	float offsetAngle = bot->obsModifier[bot->obstructed];
	bot->goal[0][0] = bot->pos[0] + MAX_GOAL_DIST * cos(bot->pos[2] + sign * offsetAngle);
	bot->goal[0][1] = bot->pos[1] + MAX_GOAL_DIST * sin(bot->pos[2] + sign * offsetAngle);
	//bot->state = (bot->state & ~STATE_MASK) | IDLE;
    }
    
    // check for collisions
    if (bot->collisionCount[0] + bot->collisionCount[1] > 2) {
	float modifier = (bot->ePos[1] > M_PI / 2) ? 0.0 : M_PI;
	float rotSign = copysign(1.0, bot->ePos[1]);
	bot->goal[0][0] = bot->pos[0] + 2*MIN_GOAL_DIST*cos(bot->pos[2] + modifier + rotSign*ERROR_COLLIDE);
	bot->goal[0][1] = bot->pos[1] + 2*MIN_GOAL_DIST*sin(bot->pos[2] + modifier - rotSign*ERROR_COLLIDE);
	bot->collisionCount[0] = 0;
	bot->collisionCount[1] = 0;
    }
}
*/


void Bot_Navigate(struct Bot * bot) {
    // check if obstacle in goal's way
    //if (!bot->qCurr) Node_Initialize(&bot->qCurr, NULL, bot->pos);
    float goalVec[2] = { bot->qCurr->pos[0] - bot->pos[0], bot->qCurr->pos[1] - bot->pos[1] };
    if (goalVec[0] == 0) goalVec[0] += 0.0001;
    float goalAngle = atan2(goalVec[1], goalVec[0]);
    char obsFlag = 0;
    for (uint8_t i = 0; i < 3; i++) {
	if (fabs(bot->pos[2] + bot->sensorOffsets[i] - goalAngle) < 1.5 * SENSOR_ANGLE)
	    if (bot->dist[i] < MIN_US_DIST) obsFlag = 1;
    }
    
	if (obsFlag || getDistance(bot->qCurr->pos, bot->pos) > G_MIN) return;
	bot->qCurr->pos[2] = bot->pos[2];
	memcpy(bot->qCurr->dist, bot->dist, sizeof(float) * 3);
	Node_Add(bot, bot->qCurr);
	
	// choose new orientation
	bot->qCand = NULL;
	bot->dirIndex = 0;
	while (!bot->qCand && bot->dirIndex < I_MAX) {
	    bot->randomAngle = random(-(SENSOR_M+SENSOR_A*bot->dirIndex), SENSOR_M+SENSOR_A*bot->dirIndex);
	    bot->randomRadius = 0;
	    if (bot->randomAngle < -SENSOR_M) bot->randomRadius = bot->dist[0];
	    else if (bot->randomAngle > SENSOR_M) bot->randomRadius = bot->dist[2];
	    else bot->randomRadius = bot->dist[1];
	    bot->dPos[0] = bot->pos[0] + bot->randomRadius*ALPHA*cos(bot->pos[2] + bot->randomAngle);
	    bot->dPos[1] = bot->pos[1] + bot->randomRadius*ALPHA*sin(bot->pos[2] + bot->randomAngle);
	    if (getDistance(bot->qCurr->pos, bot->dPos) >= D_MIN && Node_Valid(bot, bot->qRoot, bot->dPos))
		Node_Initialize(&bot->qCand, bot->qCurr, bot->dPos);
	    bot->dirIndex++;
	}
	Bot_UART_Write(bot, "dist: %.2f, %.2f, %.2f\r\n"
		"rand: %.1f, %.2f, %d\r\n"
		"qcand: %.2f, %.2f\r\n\n",
		bot->dist[0], bot->dist[1], bot->dist[2], bot->randomAngle * 180.0 / M_PI, bot->randomRadius, bot->dirIndex, bot->qCand->pos[0], bot->qCand->pos[1]);
	if (bot->qCand) bot->qCurr = bot->qCand;
	else if (bot->qCurr->parent) bot->qCurr = bot->qCurr->parent;
	else bot->state = (bot->state & ~STATE_MASK) | BATTERY;
	//bot->state = (bot->state & ~STATE_MASK) | IDLE;
}

uint8_t Bot_Odometer_Read(struct Bot * bot, uint8_t times) {
    bot->odo[0] = ODO_WEIGHT * TMR2 * times + (1 - ODO_WEIGHT) * bot->odo[0];
    bot->odo[1] = ODO_WEIGHT * TMR4 * times + (1 - ODO_WEIGHT) * bot->odo[1];
    TMR2 = 0x0;
    TMR4 = 0x0;
    if ((bot->odo[0] + bot->odo[1]) < 2 && (bot->vEst[0] + bot->vEst[1]) > 0.03) bot->collisionCount[1]++;
    else bot->collisionCount[1] = 0;
}

void Bot_Vector_Angles(struct Bot * bot, float angles[US_SENSORS]) {
	float leftVec[2] = { bot->pos[0] + cos(bot->pos[2] + SENSOR_OFFSET), bot->pos[1] + sin(bot->pos[2] + SENSOR_OFFSET) };
	float centerVec[2] = { bot->pos[0] + cos(bot->pos[2]), bot->pos[1] + sin(bot->pos[2]) };
	float rightVec[2] = { bot->pos[0] + cos(bot->pos[2] - SENSOR_OFFSET), bot->pos[1] + sin(bot->pos[2] - SENSOR_OFFSET) };
	angles[0] = atan2(leftVec[1], leftVec[0]);
	angles[1] = atan2(centerVec[1], centerVec[0]);
	angles[2] = atan2(rightVec[1], rightVec[0]);
}

void Bot_Display_Status(struct Bot * bot) {
	char line[OLED_LINE_LEN];
	char status[4];
	switch (bot->state & STATE_MASK) {
		case INIT: strcpy(status, "INT"); break;
		case IDLE: strcpy(status, "IDL"); break;
		case NAVIGATE: strcpy(status, "NAV"); break;
		case BATTERY: strcpy(status, "BAT"); break;
	}
	float pos[3] = {0};
	if (bot->qCurr && bot->qCurr->pos) memcpy(pos, bot->qCurr->pos, sizeof(float) * 3);
	OLED_ClearDisplay();
	snprintf(line, OLED_LINE_LEN, "%s %ds %ds %.f%% %d", status, bot->time, bot->dblClickCount, bot->battery, Tree_Size(bot->qRoot));
	OLED_Write_Text(0, 0, line);
	snprintf(line, OLED_LINE_LEN, "p %.2f %.2f %.f", bot->pos[0], bot->pos[1], bot->pos[2] * 180.0 / M_PI);
	OLED_Write_Text(0, 10, line);
	snprintf(line, OLED_LINE_LEN, "g %.2f %.2f %.f %d", pos[0], pos[1], pos[1] * 180.0 / M_PI, bot->qRoot);
	OLED_Write_Text(0, 20, line);
	snprintf(line, OLED_LINE_LEN, "dc %.2f %.2f o %d %d", bot->duty[0], bot->duty[1], bot->odo[0], bot->odo[1]);
	OLED_Write_Text(0, 30, line);
	snprintf(line, OLED_LINE_LEN, "us %.2f %.2f %.2f", bot->dist[0], bot->dist[1], bot->dist[2]);
	OLED_Write_Text(0, 40, line);
	snprintf(line, OLED_LINE_LEN, "n %.f %.2f %.2f %.2f", bot->randomAngle * 180. / M_PI, bot->randomRadius, bot->dPos[0], bot->dPos[1]);
	OLED_Write_Text(0, 50, line);
	OLED_Update();
}

void Bot_UART_Send_Status(struct Bot * bot) {
    Bot_UART_Write(bot, 
	    "\33[2J\33[H(%ds) Bot state: %d, battery: %.2f%%\r\n"
	    "pos: (%.2f, %.2f, %.2f), input: (%.2f, %.2f)\r\n"
	    "error: (%.2f, %.2f),\r\n"
	    "duty: (%.2f, %.2f), odo: (%d, %d)\r\n"
	    "gyro: (%.2f), acc: (%.2f. %.2f), mag: (%.2f, %.2f)\r\n"
	    "dist: (%.2f, %.2f, %.2f)\r\n"
	    "bitMap: (%.2f, %.2f)\r\n",
	    bot->time, bot->state, bot->battery,
	    bot->pos[0], bot->pos[1], bot->pos[2] * 180.0 / M_PI, bot->qCurr->pos[0], bot->qCurr->pos[1],
	    bot->ePos[0], bot->ePos[1] * 180.0 / M_PI,
	    bot->duty[0], bot->duty[1], bot->odo[0], bot->odo[1],
	    bot->gyro[2], bot->acc[0], bot->acc[1], bot->mag[0], bot->mag[1],
	    bot->dist[0], bot->dist[1], bot->dist[2],
	    bot->currentMaps[0]->pos[0], bot->currentMaps[0]->pos[1]
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

void Bot_Display_BitMap(struct Bot * bot) {
    OLED_FillDisplay();
    uint8_t index[2];
    for (uint8_t i = 0; i < 4; i++) {
	if (bot->currentMaps[i]) {
	    for (int16_t j = 0; j < MAP_UNITS; j++) {
		index[0] = j;
		for (int16_t k = 0; k < MAP_UNITS; k++) {
		    index[1] = k;
		    char val = Bitmap_At(bot->currentMaps[i], index);
		    if (val % 2 == 0) {
			// cleaned (0x10) or open (0x00)
			int16_t startPos[2] = {2*(MAP_UNITS*(i%2) + j), 2*(MAP_UNITS*(i/2) + k)};
			OLED_FillRectangle(startPos[0], startPos[1], startPos[0]+2, startPos[1]+2, BLACK);
		    }
		}
	    }
	}
    }
    
    OLED_Update();
}

/* map functions */
void BitMap_Initialize(struct Bot * bot, struct BitMap ** ptr, float pos[2]) {
    *ptr = malloc(sizeof(struct BitMap));
    memcpy((*ptr)->pos, pos, sizeof(float) * 2);
    for (char i = 0; i < 8; i++) (*ptr)->neighbors[i] = NULL;
    for (char i = 0; i < MAP_UNITS; i++)
	for (char j = 0; j < MAP_UNITS_BIT; j++)
	    (*ptr)->grid[i][j] = 0xFF;
    bot->numMaps++;
}

/* index returns the index position of the position */
char BitMap_Contains(struct BitMap * map, uint8_t index[2], float pos[2]) {
    float xRange[2] = { map->pos[0], map->pos[0] + MAP_SIZE };
    float yRange[2] = { map->pos[1] - MAP_SIZE, map->pos[1] };
    if ((pos[0] > xRange[0]) && (pos[0] < xRange[1]) && (pos[1] > yRange[0]) && (pos[1] < yRange[1])) {
	index[0] = (pos[0] - map->pos[0]) / MAP_RES;
	index[1] = (map->pos[1] - pos[1]) / MAP_RES;
	return 1;
    }
    return 0;
}

char Bitmap_At(struct BitMap * map, uint8_t index[2]) {
    char byte = map->grid[index[0]][index[1] / 4];
    return (byte & (0x3 << 2*(index[1] % 4))) >> 2*(index[1] % 4);
}

// get the bit location relative to current position
char BitMap_AtRelative(struct Bot * bot, uint8_t dirIndex) {
	uint8_t newIndex[3] = { 0, bot->posIndex[0] + bot->posModifier[dirIndex][0], bot->posIndex[1] + bot->posModifier[dirIndex][1] };
	if (newIndex[1] < 0 || newIndex[1] >= MAP_UNITS || newIndex[2] < 0 || newIndex[2] >= MAP_UNITS) {
		newIndex[0] = dirIndex + 1;
		newIndex[1] = (newIndex[1] < 0) ? MAP_UNITS : (newIndex[1] >= MAP_UNITS) ? MAP_UNITS - 1 : newIndex[1];
		newIndex[2] = (newIndex[2] < 0) ? MAP_UNITS : (newIndex[2] >= MAP_UNITS) ? MAP_UNITS - 1 : newIndex[2];
	}
	return Bitmap_At(bot->currentMaps[newIndex[0]], &newIndex[1]);
}

void BitMap_Set(struct BitMap * map, uint8_t index[2], char val) {
    char mask = 0x3 << 2*(index[1] % 4);
    map->grid[index[0]][index[1] / 4] = (map->grid[index[0]][index[1] / 4] & ~mask) | (val << 2*(index[1] % 4));
}

// set the bit location relative to current position
void BitMap_SetRelative(struct Bot * bot, uint8_t dirIndex, char val) {
	uint8_t newIndex[3] = { 0, bot->posIndex[0] + bot->posModifier[dirIndex][0], bot->posIndex[1] + bot->posModifier[dirIndex][1] };
	if (newIndex[1] < 0 || newIndex[1] >= MAP_UNITS || newIndex[2] < 0 || newIndex[2] >= MAP_UNITS) {
		newIndex[0] = dirIndex + 1;
		newIndex[1] = (newIndex[1] < 0) ? MAP_UNITS : (newIndex[1] >= MAP_UNITS) ? MAP_UNITS - 1 : newIndex[1];
		newIndex[2] = (newIndex[2] < 0) ? MAP_UNITS : (newIndex[2] >= MAP_UNITS) ? MAP_UNITS - 1 : newIndex[2];
	}
	BitMap_Set(bot->currentMaps[newIndex[0]], &newIndex[1], val);
}

char BitMap_IndexToPos(struct BitMap * map, float pos[2], uint8_t index[2]) {
    if (index[0] < 0 || index[0] >= MAP_UNITS || index[1] < 0 || index[1] >= MAP_UNITS) return 0;
    pos[0] = map->pos[0] + index[0] * MAP_RES;
    pos[1] = map->pos[1] - index[1] * MAP_RES;
    return 1;
}

/* node functions */
void Node_Initialize(struct Node ** ptr, struct Node * parent, float pos[3]) {
	*ptr = malloc(sizeof(struct Node));
	(*ptr)->parent = parent;
	memcpy((*ptr)->pos, pos, sizeof(float) * 3);
	(*ptr)->numChildren = 0;
}

void Node_Add(struct Bot * bot, struct Node * node) {
    if (!node) return;
	if (!node->parent) {
	    bot->qRoot = node;
	} else if (node->parent->numChildren++ == 1) {
		node->parent->children = malloc(sizeof(struct Node *));
		node->parent->children[0] = node;
	} else {
		node->parent->children = realloc(node->parent->children, sizeof(struct Node *) * node->parent->numChildren);
		node->parent->children[node->parent->numChildren-1] = node;
	}
}

// check if node's safe region contains position, else valid
char Node_Valid(struct Bot * bot, struct Node * node, float pos[2]) {
    if (!node) return 1;
    if (getDistance(node->pos, pos) < D_MIN) return 0;
    float posVec[2] = { pos[0] - node->pos[0], pos[1] - node->pos[1] };
    if (posVec[0] == 0) posVec[0] += 0.0001;
    float posAngle = atan2(posVec[1], posVec[0]);
    if (node != bot->qCurr) {
	for (uint8_t i = 0; i < 3; i++) {
	    if (fabs(node->pos[2] + bot->sensorOffsets[i] - posAngle) < SENSOR_ANGLE)
		if (getDistance(node->pos, pos) < node->dist[i]) return 0;
	}
    }
    for (uint8_t i = 0; i < node->numChildren; i++)
	if (!Node_Valid(bot, node->children[i], pos)) return 0;
    return 1;
}

unsigned int Tree_Size(struct Node * node) {
    if (!node) return -1;
    unsigned int num = node->numChildren;
    for (uint8_t i = 0; i < node->numChildren; i++) num += Tree_Size(node->children[i]);
    return num + 1;
}

void Bot_FIR_Init(struct Bot * bot) {
    float fcNew = (FIR_FP + (FIR_FS - FIR_FP) / 2.0) / FIR_F;
    
    for (uint8_t i = 0; i < FIR_N / 2; i++) {
	bot->firHd[i] = ideal_lowpass(fcNew, i);
	bot->firW[i] = hamming(i);
	bot->firH[FIR_N / 2 + i] = bot->firHd[i] * bot->firW[i];
	bot->firH[FIR_N / 2 - 1 - i] = bot->firHd[i] * bot->firW[i];
    }
}

float Bot_FIR_Filter(struct Bot * bot, uint8_t n) {
    float sum = 0;
    for (uint8_t i = 0; i < n; i++) {
	sum += bot->firH[i] * bot->firX[n - 1 - i];
    }
    return sum;
}

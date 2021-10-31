#include "bot.h"

void Bot_Pos_Update(struct Bot * bot) {
    IMU_Read(bot->gyro, bot->acc, bot->mag, bot->asa);
    
    switch (bot->state) {
	case IDLE: {
	    /* IDLE state: estimate bias */
	    if (bot->numBias++ < BIAS_MAX) {
		for (int i = 0; i < 3; i++) bot->bias[i] += bot->gyro[i];
	    } else {
		bot->numBias = 0;
		for (int i = 0; i < 3; i++) bot->bias[i] = 0.0;
	    }
	    break;
	}
	
	case NAVIGATE: {
	    /* NAVIGATE state: estimate next position with current trajectory */	    
	    float vel[2] = { bot->duty[0] * VEL_MAX, bot->duty[1] * VEL_MAX };
	    float dPos[3] = {0}, ePos[3] = {0};
	    dPos[0] = 0.5 * (cos(bot->pos[2]) * vel[0] + cos(bot->pos[2]) * vel[1]);
	    dPos[1] = 0.5 * (sin(bot->pos[2]) * vel[0] + sin(bot->pos[2]) * vel[1]);
	    dPos[2] = 3.0 * (vel[1] - vel[0]) / CHASSIS_LENGTH;
	    ePos[0] = bot->pos[0] + dPos[0] * DT;
	    ePos[1] = bot->pos[1] + dPos[1] * DT;
	    ePos[2] = bot->pos[2] + dPos[2] * DT;
	    
	    /* estimate rotation with IMU */
	    
	    float mPos[3] = {0.0, 0.0, bot->pos[2] + bot->gyro[2] * DT};
	    float dc = 1;//(fabs(bot.duty[0]) + fabs(bot.duty[1])) / 2.0;

	    bot->pos[0] = ePos[0];// + mMes[0][0];
	    bot->pos[1] = ePos[1];// + mMes[0][1];
	    bot->pos[2] = normAngle((1 - dc * K_EST) * ePos[2] + dc * K_EST * mPos[2]);
	    
	    /* check if out of map bounds and turned too much */
	    if ((fabs(bot->pos[2] - bot->dPos[2]) > SENSOR_OFFSET) || !BitMap_Contains(bot->currentMap, bot->pos))
		Bot_Map_Required(bot);
	    break;
	}
    }; 
}

float Bot_InputPos_Update(struct Bot * bot) {
    /* check if new position necessary */
    /* return smallest distance to obstacle */
    float minDist = getDistance(bot->inputPos[0], bot->inputPos[1], bot->pos[0], bot->pos[1]);
    float maxDist = 0.0;
    char maxIndex = -1;
    for (char i = 0; i < 3; i++) {
	if (bot->distances[i] < minDist) minDist = bot->distances[i];
	if (bot->distances[i] > maxDist) {
	    maxDist = bot->distances[i];
	    maxIndex = i;
	}
    }
    float dcAvg = (bot->duty[0] + bot->duty[1]) / 10.0;
    /*if (minDist < MIN_OBST_DIST + dcAvg) {
	LATAINV = _LATA_LATA1_MASK;
	float newAngle = Box_Muller(bot->pos[2] + sensorOffsets[maxIndex], NAV_SIGMA);
	float newDist = (bot->distances[maxIndex] - MIN_OBST_DIST > MIN_INPUT_DIST) ? MIN_INPUT_DIST : bot->distances[maxIndex];
	float pos[2] = { bot->pos[0] + newDist * cos(newAngle), bot->pos[1] + newDist * sin(newAngle) };
	memcpy(bot->inputPos, pos, sizeof(float) * 2);
	/* too close to obstacle or desired location - recalculate SRT */
	/*float sensorAngles[3];
	Bot_Vector_Angles(sensorAngles);
	char found = 0;
	char i = 0;
	while (i < N_RANDOM_TRIES && !found) {
	    bot.randomAngle = Box_Muller(bot.pos[2], NAV_SIGMA);
	    char numRange = -1;
	    if (fabs(bot.randomAngle - sensorAngles[0]) <= SENSOR_ANGLE) numRange = 0;
	    else if (fabs(bot.randomAngle - sensorAngles[1]) <= SENSOR_ANGLE) numRange = 1;
	    else if (fabs(bot.randomAngle - sensorAngles[2]) <= SENSOR_ANGLE) numRange = 2;
	    snprintf(bot.buf, 100, "angle: %.2f, numRange: %d\r\n", bot.randomAngle, numRange);
	    UART_Write_String(bot.buf, strlen(bot.buf));
	    if (numRange >= 0) {
		float pos[2] = { bot.pos[0] + bot.distances[numRange] * cos(bot.randomAngle), bot.pos[1] + bot.distances[numRange] * sin(bot.randomAngle) };
		char frontier = Bot_Frontier(pos);
		snprintf(bot.buf, 100, "frontier: %d, pos: (%.2f, %.2f), angle: %.2f, numRange: %d\r\n", frontier, pos[0], pos[1], bot.randomAngle, numRange);
		UART_Write_String(bot.buf, strlen(bot.buf));
		if (frontier < 0x9) {
		    memcpy(bot.inputPos, pos, sizeof(float) * 2);
		    found = 1;
		}
	    }
	    i++;
	}
	/* if no open spot go backwards 
	if (!found) {
	    bot.randomAngle = Box_Muller(bot.pos[2], NAV_SIGMA) + M_PI;
	    bot.inputPos[0] = bot.pos[0] + MIN_INPUT_DIST * cos(bot.randomAngle);
	    bot.inputPos[1] = bot.pos[1] + MIN_INPUT_DIST * sin(bot.randomAngle);
	}
	memcpy(bot.dPos, bot.pos, sizeof(float) * 2);*/
    //}*/
    
    return minDist;
}

char Bot_Frontier(struct Bot * bot, float pos[2]) {
    char max = 0xF;
    for (char i = 0; i < 4; i++) {
	if (bot->localMaps[i] && ProbMap_Contains(bot->localMaps[i], pos)) {
	    char mapIndex[2] = { (char)((pos[0] - bot->localMaps[i]->pos[0]) / MAP_RES), (char)((bot->localMaps[i]->pos[1] - pos[1]) / MAP_RES) };
	    max = bot->localMaps[i]->grid[mapIndex[0]][mapIndex[1]];
	    for (char j = 0; j < 8; j++) {
		char val = bot->localMaps[i]->grid[mapIndex[0] + posModifier[j][0]][mapIndex[1] + posModifier[j][1]];
		if (max < val) max = val;
	    }
	    break;
	}
    }
    return max;
}

/* determine which maps will form part of supermap 
 * create maps if necessary */
void Bot_Map_Required(struct Bot * bot) {
    /* save old maps and optimize */
    bot->changeLock = 1;
    Bot_Optimise_Local(bot);

    /* update the current map based on the position of the bot */
    if (!BitMap_Contains(bot->currentMap, bot->pos)) {
	for (char i = 0; i < 8; i++) {
	    if (bot->currentMap->neighbors[i] && BitMap_Contains(bot->currentMap->neighbors[i], bot->pos)) {
		bot->currentMap = bot->currentMap->neighbors[i];
		break;
	    }
	}
    }

    /* determine current map corner with smallest view angle */
    bot->dPos[2] = bot->pos[2];
    float viewVec[2] = { bot->pos[0] + cos(bot->pos[2]), bot->pos[1] + sin(bot->pos[2]) };
    float rect[4][2] = {
	{bot->currentMap->pos[0], bot->currentMap->pos[1]},
	{bot->currentMap->pos[0] + MAP_SIZE, bot->currentMap->pos[1]},
	{bot->currentMap->pos[0] + MAP_SIZE, bot->currentMap->pos[1] - MAP_SIZE},
	{bot->currentMap->pos[0], bot->currentMap->pos[1] - MAP_SIZE}
    };
    float minAngle = M_PI;
    char minIndex = 0;
    for (char i = 0; i < 4; i++) {
	float angle = getAngle(viewVec[0], viewVec[1], rect[i][0], rect[i][1]);
	if (angle < minAngle) {
	    minAngle = angle;
	    minIndex = i;
	}
    }

    /* identify/create bit and prob maps in corresponding direction */
    ProbMap_Initialize(&bot->localMaps[0], bot->currentMap->pos, DEFAULT_VAL);
    char req[3] = {(2*minIndex + 7) % 8, 2*minIndex, (2*minIndex + 1) % 8};
    for (char i = 0; i < 3; i++) {
	if (!bot->currentMap->neighbors[req[i]]) {
	    float pos[2] = { bot->currentMap->pos[0] + posModifier[req[i]][0] * MAP_SIZE, bot->currentMap->pos[1] + posModifier[req[i]][1] * MAP_SIZE };
	    BitMap_Initialize(bot, &bot->currentMap->neighbors[req[i]], pos);
	    bot->currentMap->neighbors[req[i]]->neighbors[(req[i] + 4) % 8] = bot->currentMap;
	    Bot_Reinforce_Neighbors(bot->currentMap->neighbors[req[i]]);
	}
	ProbMap_Initialize(&bot->localMaps[i+1], bot->currentMap->neighbors[req[i]]->pos, DEFAULT_VAL);
    }
    bot->changeLock = 0;
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

void Bot_Map_Update(struct Bot * bot) {
    float obstaclePos[US_SENSORS][2];
    float valid[US_SENSORS] = {0};
    char numPos = distanceToPos(obstaclePos, bot->pos, valid, bot->distances);
    
    if (numPos > 0) {
	/* loop each local cell, check validity and modify */
	float max = 0;
	float probMap[4][MAP_UNITS][MAP_UNITS] = {{{0}}};
	/* obtain view and sensor vectors */
	float sensorAngles[3];
	Bot_Vector_Angles(bot, sensorAngles);
	
	float posAngle, distance;
	for (char i = 0; i < 4; i++) {
	    for (char j = 0; j < MAP_UNITS; j++) {
		for (char k = 0; k < MAP_UNITS; k++) {
		    float pos[2] = { bot->localMaps[i]->pos[0] + MAP_RES * j, bot->localMaps[i]->pos[1] - MAP_RES * k };
		    float posVec[2] = { pos[0] - bot->pos[0], pos[1] - bot->pos[1] };
		    posAngle = atan2(posVec[1], posVec[0]);
		    distance = getDistance(pos[0], pos[1], bot->pos[0], bot->pos[1]);
		    /* make sure position is not behind reading */
		    if (distance > MIN_US_DIST) {
			char numRange = -1;
			if (fabs(posAngle - sensorAngles[0]) <= SENSOR_ANGLE) numRange = 0;
			else if (fabs(posAngle - sensorAngles[1]) <= SENSOR_ANGLE) numRange = 1;
			else if (fabs(posAngle - sensorAngles[2]) <= SENSOR_ANGLE) numRange = 2;		    
			if (numRange >= 0) {
			    bot->localViewMaps[i][j][k / 8] |= 0x1 << (k % 8);
			    for (char l = 0; l < US_SENSORS; l++) {
				if (valid[l]) probMap[i][j][k] += Multivariate_Gaussian(obstaclePos[l][0], obstaclePos[l][1], pos[0], pos[1]);
			    }
			    if (max < probMap[i][j][k]) max = probMap[i][j][k];
			}
		    }
		}
	    }
	}
	
	max *= 1.01;
	for (char i = 0; i < 4; i++) {
	    for (char j = 0; j < MAP_UNITS; j++) {
		for (char k = 0; k < MAP_UNITS; k++) {
		    if (bot->localViewMaps[i][j][k / 8] & (0x1 << (k % 8))) {
			char nibbleProb = (char)(probMap[i][j][k] * 16.0 / max);
			bot->localMaps[i]->grid[j][k] = ((bot->localMaps[i]->grid[j][k] + nibbleProb) >> 1) & 0xF;
		    }
		}
	    }
	}
    }
}

void Bot_Optimise_Local(struct Bot * bot) {
    /* save local map back to global */
    if (bot->localMaps && bot->localMaps[0] && bot->currentMap) {
	for (char i = 0; i < MAP_UNITS; i++) {
	    for (char j = 0; j < MAP_UNITS; j++) {
		bot->currentMap->grid[i][j / 8] |= ((bot->localMaps[0]->grid[i][j] >> 3) & 0x1) << (j % 8);
	    }
	}
    }
}

void Bot_Controller(struct Bot * bot) { 
    float convDuty[2] = {0};		// convert duty cycle to correct value
        
    switch (bot->state) {
	case IDLE: {
	    /* IDLE state: do not move */
	    for (int i = 0; i < 2; i++) {
		bot->duty[i] = 0.0;
		convDuty[i] = 0.0;
	    }
	    LATBCLR = _LATB_LATB4_MASK;
	    LATACLR = _LATA_LATA3_MASK;
	    break;
	}
	
	case NAVIGATE: {
	    /* NAVIGATE state: determine duty cycle */
	    float distance = Bot_InputPos_Update(bot);
	    
	    /* navigate: obtain error angle and distance */
	    //float viewVec[2] = { bot->pos[0] + cos(bot->pos[2]), bot->pos[1] + sin(bot->pos[2]) };
	    float inputVec[2] = { bot->inputPos[0] - bot->pos[0], bot->inputPos[1] - bot->pos[1] };
	    bot->ePos[2] = normAngle(atan2(inputVec[1], inputVec[0]) - bot->pos[2]);//atan2(viewVec[1], viewVec[0]);
    
	    float rotSign = (bot->ePos[2] < 0.0) ? -1.0 : 1.0;
	    float maxTurn = fabs(bot->ePos[2]) / (K_TURN * (BETA + fabs(bot->ePos[2]))) + 0.1;

	    if (fabs(bot->ePos[2]) > ERROR_MAX) {
		bot->duty[0] = rotSign * -maxTurn;
		bot->duty[1] = rotSign * maxTurn;
	    } else {
		float maxSpeed = K_OFFSET - K_ALPHA / distance;
		bot->duty[0] = maxSpeed + rotSign * -maxTurn;
		bot->duty[1] = maxSpeed + rotSign * maxTurn;
	    }

	    /*for (int i = 0; i < 2; i++) {
		/* set new duty cycles 
		if (newDuty[i] < bot.duty[i] - GAMMA) bot.duty[i] -= GAMMA;
		else if (newDuty[i] > bot.duty[i] + GAMMA) bot.duty[i] += GAMMA;
		//else if (fabs(newDuty[i]) < MIN_PWM) bot.duty[i] = 0.0;
		else bot.duty[i] = newDuty[i];
	    }*/


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
	    break;
	}
    };
    
    OC4RS = (int)(convDuty[0] * PWM_T);
    OC5RS = (int)(convDuty[1] * PWM_T);
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
    float mapPos[4][2] = {{0}};
    for (char i = 0; i < 4; i++) {
	if (bot->localMaps[i]) memcpy(mapPos[i], bot->localMaps[i], sizeof(float) * 2);
    }
    
    Bot_UART_Write(bot, "\33[2J\33[H(%ds) Bot state: %d, battery: %.2f%%\r\n", bot->time, bot->state, bot->battery);
    Bot_UART_Write(bot, "pos: (%.2f, %.2f, %.2f)\r\n", bot->pos[0], bot->pos[1], bot->pos[2] * 180.0 / M_PI);
    Bot_UART_Write(bot, "input: (%.2f, %.2f), random: %.2f\r\n", bot->inputPos[0], bot->inputPos[1], bot->randomAngle);
    Bot_UART_Write(bot, "error: (%.2f, %.2f, %.2f)\r\n", bot->ePos[0], bot->ePos[1], bot->ePos[2] * 180.0 / M_PI);
    Bot_UART_Write(bot, "duty: (%.2f, %.2f)\r\n", bot->duty[0], bot->duty[1]);  
    Bot_UART_Write(bot, "gyro: (%.2f, %.2f, %.2f), acc: (%.2f. %.2f, %.2f), mag: (%.2f, %.2f, %.2f)\r\n",
	bot->gyro[0], bot->gyro[1], bot->gyro[2], bot->acc[0], bot->acc[1], bot->acc[2], bot->mag[0], bot->mag[1], bot->mag[2]);
    Bot_UART_Write(bot, "bias: (%.2f, %.2f, %.2f), asa: (%.2f, %.2f, %.2f)\r\n", bot->bias[0], bot->bias[1], bot->bias[2], bot->asa[0], bot->asa[1], bot->asa[2]);
    Bot_UART_Write(bot, "dist: (%.2f, %.2f, %.2f)\r\n", bot->distances[0], bot->distances[1], bot->distances[2]);
    Bot_UART_Write(bot, "numMaps: %d, local: (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)\r\n", bot->numMaps,
	mapPos[0][0], mapPos[0][1], mapPos[1][0], mapPos[1][1], mapPos[2][0], mapPos[2][1], mapPos[3][0], mapPos[3][1]);
    Bot_UART_Write(bot, "bitMap: (%.2f, %.2f)\r\n", bot->currentMap->pos[0], bot->currentMap->pos[1]);
}

void Bot_UART_Write(struct Bot * bot, char * format, ...) {
    va_list args;
    va_start(args, format);
    DCH1SSIZ = vsnprintf(bot->buf, BUF_LEN, format, args);
    /*bot->bufSaveIndex += len;
    if (bot->bufSaveIndex > BUF_LEN) {
	len = vsnprintf(bot->buf, BUF_LEN, format, args);
	bot->bufSaveIndex = len;
	
    }
    if (bot->buf[bot->bufPrintIndex]) {
	UART_Write(bot->buf[bot->bufPrintIndex]);
	bot->bufPrintIndex = (bot->bufPrintIndex + 1) % (BUF_LEN - 1);
    }*/
    DCH1INTCLR = 0x00FF00FF;
    DCH1CONSET = _DCH1CON_CHEN_MASK;
    va_end(args);
}

void Bot_Display_BitMap(struct BitMap * map) {
    /*snprintf(bot->buf, 100, "\r\nBitMap pos: (%.2f, %.2f)\r\n", map->pos[0], map->pos[1]);
    UART_Write_String();
    for (char i = 0; i < MAP_UNITS; i++) {
	snprintf(bot->buf, 100, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
	    map->grid[0][i] & 1, (map->grid[0][i] >> 1) & 1, (map->grid[0][i] >> 2) & 1, (map->grid[0][i] >> 3) & 1,
	    (map->grid[0][i] >> 4) & 1, (map->grid[0][i] >> 5) & 1, (map->grid[0][i] >> 6) & 1, (map->grid[0][i] >> 7) & 1,
	    map->grid[1][i] & 1, (map->grid[1][i] >> 1) & 1, (map->grid[1][i] >> 2) & 1, (map->grid[1][i] >> 3) & 1,
	    (map->grid[1][i] >> 4) & 1, (map->grid[1][i] >> 5) & 1, (map->grid[1][i] >> 6) & 1, (map->grid[1][i] >> 7) & 1);
	UART_Write_String();
    }*/
}

void Bot_Display_ProbMap(struct ProbMap * map) {
    /*snprintf(bot->buf, 100, "\r\nProbMap pos: (%.2f, %.2f)\r\n", map->pos[0], map->pos[1]);
    UART_Write_String();
    for (char i = 0; i < MAP_UNITS; i++) {
	snprintf(bot->buf, 100, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
	    map->grid[0][i], map->grid[1][i], map->grid[2][i], map->grid[3][i],
	    map->grid[4][i], map->grid[5][i], map->grid[6][i], map->grid[7][i],
	    map->grid[8][i], map->grid[9][i], map->grid[10][i], map->grid[11][i],
	    map->grid[12][i], map->grid[13][i], map->grid[14][i], map->grid[15][i]);
	UART_Write_String();
    }*/
}

/* map functions */
void BitMap_Initialize(struct Bot * bot, struct BitMap ** ptr, float pos[2]) {
    *ptr = malloc(sizeof(struct BitMap));
    memcpy((*ptr)->pos, pos, sizeof(float) * 2);
    for (char i = 0; i < 8; i++) (*ptr)->neighbors[i] = NULL;
    for (char i = 0; i < MAP_UNITS; i++)
	for (char j = 0; j < MAP_UNITS_BIT; j++)
	    (*ptr)->grid[i][j] = 0xF;
    bot->numMaps++;
}

void ProbMap_Initialize(struct ProbMap ** ptr, float pos[2], char fillVal) {
    *ptr = malloc(sizeof(struct ProbMap));
    memcpy((*ptr)->pos, pos, sizeof(float) * 2);
    for (char i = 0; i < MAP_UNITS; i++)
	for (char j = 0; j < MAP_UNITS; j++)
	    (*ptr)->grid[i][j] = fillVal;
}

char BitMap_Contains(struct BitMap * map, float pos[2]) {
    float xRange[2] = { map->pos[0], map->pos[0] + MAP_SIZE };
    float yRange[2] = { map->pos[1] - MAP_SIZE, map->pos[1] };
    return ((pos[0] > xRange[0]) && (pos[0] < xRange[1]) && (pos[1] > yRange[0]) && (pos[1] < yRange[1]));
}

char ProbMap_Contains(struct ProbMap * map, float pos[2]) {
    float xRange[2] = { map->pos[0], map->pos[0] + MAP_SIZE };
    float yRange[2] = { map->pos[1] - MAP_SIZE, map->pos[1] };
    return ((pos[0] > xRange[0]) && (pos[0] < xRange[1]) && (pos[1] > yRange[0]) && (pos[1] < yRange[1]));
}
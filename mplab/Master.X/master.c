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

#include <math.h>

#include "master.h"

int main() {
    srand(PORTB);
    
    long delay = 640000l;
    while (delay--);
    
    Init();
    UART_Write_String("Peripherals Initialized\r\n", 26);
    char whoami = IMU_Init();
    bot.inputPos[0] = 0.0;
    bot.inputPos[1] = 0.0;
    bot.distances[3] = MAX_US_DIST;
    
    //float startMap[2] = { -MAP_SIZE / 2, -MAP_SIZE / 2 };
    //bot.currentMaps[0] = Map_Initialize(startMap, DEFAULT_VAL);
    //Bot_Map_Required();
    //Bot_Add_Instruction(1.0, 0.0);
    
    snprintf(bot.buf, 100, "IMU address: %d\r\n", whoami);
    UART_Write_String(bot.buf, strlen(bot.buf));
    
    for (;;);
    return EXIT_SUCCESS;
}

void Bot_Pos_Update() {
    /* Get IMU sensor data */
    uint8_t data[6];
    int16_t values[3];
    I2C_Read(IMU_ADD, ACCEL_XOUT_H, data, 4);
    I2C_Read(IMU_ADD, GYRO_ZOUT_H, &data[4], 2);
    
    for (int i = 0; i < 3; i++) values[i] = (data[2*i] << 8) | data[2*i+1];
    bot.imuData[0] = (float)values[1] * ACC_SCALE / IMU_RES;
    bot.imuData[1] = (float)values[0] * ACC_SCALE / IMU_RES;
    bot.imuData[2] = (float)values[2] * GYRO_SCALE * M_PI / (IMU_RES * 180.0);
    
    switch (bot.state) {
	case IDLE: {
	    /* IDLE state: estimate bias */
	    if (bot.numBias++ < BIAS_MAX) {
		for (int i = 0; i < 3; i++) bot.bias[i] += bot.imuData[i];
	    } else {
		bot.numBias = 0;
		for (int i = 0; i < 3; i++) bot.bias[i] = 0.0;
	    }
	    break;
	}
	
	case NAVIGATE: {
	    /* NAVIGATE state: estimate next position with current trajectory 
	    float vel[2] = { bot.duty[0] * VEL_MAX, bot.duty[1] * VEL_MAX };
	    float dPos[3] = {0};
	    float ePos[3];//, eSigma[3][3];

	    memcpy(ePos, bot.pos, sizeof(float) * 3);
	    //matrix_plus(3, eSigma, bot.sigma, bot.Q);

	    if (vel[0] == vel[1]) {
		// bot moving straight ahead
		dPos[0] = vel[0] * DT * cos(bot.pos[2]);
		dPos[1] = vel[1] * DT * sin(-bot.pos[2]);
	    } else {
		float radius = 0.0;
		if (vel[0] > vel[1]) {
		    // turning right
		    radius = vel[1] / (vel[0] - vel[1]) * WHEEL_RADIUS;
		    dPos[2] = (vel[0] * DT / (WHEEL_RADIUS + radius));
		} else {
		    // turning left
		    radius = vel[0] / (vel[1] - vel[0]) * WHEEL_RADIUS;
		    dPos[2] = -(vel[1] * DT / (WHEEL_RADIUS + radius));
		}
		float distance = 2 * (WHEEL_RADIUS + radius) * sin(fabs(dPos[2]) / 2.0);
		ePos[2] += dPos[2];
		dPos[0] = distance * cos(ePos[2]);
		dPos[1] = distance * sin(-ePos[2]);
	    }
	    ePos[0] += dPos[0];
	    ePos[1] += dPos[1];
	     * */
	    
	    float vel[2] = { bot.duty[0] * VEL_MAX, bot.duty[1] * VEL_MAX };
	    float dPos[3] = {0}, ePos[3] = {0};
	    dPos[0] = 0.5 * (cos(bot.pos[2]) * vel[0] + cos(bot.pos[2]) * vel[1]);
	    dPos[1] = 0.5 * (sin(bot.pos[2]) * vel[0] + sin(bot.pos[2]) * vel[1]);
	    dPos[2] = vel[1] / CHASSIS_LENGTH - vel[0] / CHASSIS_LENGTH;
	    ePos[0] = bot.pos[0] + dPos[0] * DT;
	    ePos[1] = bot.pos[1] + dPos[1] * DT;
	    ePos[2] = bot.pos[2] + dPos[2] * DT;
	    
	    /* estimate roation with gyroscope */
	    float mPos[3] = {0.0, 0.0, bot.pos[2] + bot.imuData[2] * DT};
	    float dc = (fabs(bot.duty[0]) + fabs(bot.duty[1])) / 2.0;
	    
	    /* estimate sensor measurements */
	    //vel[0] = dPos[0] / DT;
	    //vel[1] = dPos[1] / DT;
	    //float eImuData[3] = { (vel[0] - bot.vel[0]) / DT, (vel[1] - bot.vel[1]) / DT, dPos[2] / 2.0 };
	    //memcpy(bot.vel, vel, sizeof(float) * 2);

	    /* fuse estimation with measurement 
	    float K[3][3], inv_K[3][3], mMes[3][1];
	    float dMes[3][1] = {
		{bot.imuData[0] - eImuData[0] - bot.bias[0]},
		{bot.imuData[1] - eImuData[1] - bot.bias[1]},
		{bot.imuData[2] - eImuData[2] - bot.bias[2]}
	    };
	    matrix_plus(3, K, eSigma, bot.R);
	    matrix_inv(inv_K, K);
	    matrix_mul(3, 3, 3, K, eSigma, inv_K);
	    matrix_mul(3, 1, 3, mMes, K, dMes); */

	    bot.pos[0] = ePos[0];// + mMes[0][0];
	    bot.pos[1] = ePos[1];// + mMes[0][1];
	    bot.pos[2] = ePos[2];//(1 - dc * K_EST) * ePos[2] + dc * K_EST * mPos[2];
	    
	    /* check if out of map bounds and turned too much 
	    if ((fabs(bot.pos[2] - bot.dPos[2]) > SENSOR_OFFSET) || !Map_Contains(bot.currentMaps[0], bot.pos))
		Bot_Map_Required();
	    break;*/
	}
    };
}

float Bot_InputPos_Update() {
    /* check if new position necessary */
    float distance = getDistance(bot.inputPos[0], bot.inputPos[1], bot.pos[0], bot.pos[1]);
    float minDist = bot.distances[0];
    if (distance < MIN_DIST || bot.distances[0] < MIN_OBST_DIST || bot.distances[1] < MIN_OBST_DIST || bot.distances[2] < MIN_OBST_DIST) {
	float distances[3][1] = {{bot.distances[0]}, {bot.distances[1]}, {bot.distances[2]}};
	float cmu[4][1];
	matrix_mul(3, 1, 3, cmu, sensorModifier, distances);
	cmu[3][0] = cmu[0][0];
	char mu = 0;
	float muMax = powf(cmu[0][0], 2);
	for (char i = 1; i < 4; i++) {
	    if (i < 3) cmu[3][0] += powf(cmu[i][0], 2);
	    else cmu[3][0] = 2.0 / cmu[3][0];
	    if (cmu[i][0] > muMax) {
		muMax = cmu[i][0];
		mu = i;
	    }
	    if (bot.distances[i] < minDist) minDist = bot.distances[i];
	}
	float angle = Box_Muller(sensorOffsets[mu], 1.0 / (1.0 + muMax));
	bot.inputPos[0] = bot.pos[0] + 2 * bot.distances[mu] * cos(bot.pos[2] + angle);
	bot.inputPos[1] = bot.pos[1] + 2 * bot.distances[mu] * sin(bot.pos[2] + angle);
    }
    return minDist;
}

void Bot_Trigger_Ultrasonic() { 
    bot.usState = (bot.usState + 1) % US_SENSORS;
    TMR2 = 0x0;
    /* enable HC-SR04 trigger on current state for 10us */
    long delay = SYSCLK / (2 * US_TRIG_T);
    switch (bot.usState) {
	case 0:
	    LATBSET = _LATB_LATB9_MASK;
	    while (delay--);
	    LATBCLR = _LATB_LATB9_MASK;
	    break;
	case 1: 
	    LATBSET = _LATB_LATB10_MASK;
	    while (delay--);
	    LATBCLR = _LATB_LATB10_MASK;
	    break;
	case 2:
	    LATBSET = _LATB_LATB11_MASK;
	    while (delay--);
	    LATBCLR = _LATB_LATB11_MASK;
	    break;
    }
}

/* determine which maps will form part of supermap 
 * create maps if necessary */
void Bot_Map_Required() {
    if (!bot.mapChangeLock) {
	/* save old maps and optimize */
	Bot_Optimise_Local();

	/* update the current map based on the position of the bot */
	if (!Map_Contains(bot.currentMaps[0], bot.pos)) {
	    bot.mapChangeLock = 1;
	    for (char i = 0; i < 8; i++) {
		if (bot.currentMaps[0]->neighbors[i] && Map_Contains(bot.currentMaps[0]->neighbors[i], bot.pos)) {
		    bot.currentMaps[0] = bot.currentMaps[0]->neighbors[i];
		    for (char j = 1; j < 4; j++) bot.currentMaps[j] = NULL;
		    break;
		}
	    }
	}
	
	/* determine current map corner with smallest view angle */
	memcpy(bot.dPos, bot.pos, sizeof(float) * 3);
	float viewVec[2] = { bot.pos[0] + cos(bot.pos[2]), bot.pos[1] - sin(bot.pos[2]) };
	float rect[4][2] = {
	    {bot.currentMaps[0]->pos[0], bot.currentMaps[0]->pos[1]},
	    {bot.currentMaps[0]->pos[0] + MAP_SIZE, bot.currentMaps[0]->pos[1]},
	    {bot.currentMaps[0]->pos[0] + MAP_SIZE, bot.currentMaps[0]->pos[1] + MAP_SIZE},
	    {bot.currentMaps[0]->pos[0], bot.currentMaps[0]->pos[1] + MAP_SIZE}
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
	

	/* identify/create maps in corresponding direction */
	char req[3] = {(2*minIndex + 7) % 8, 2*minIndex, (2*minIndex + 1) % 8};
	for (char i = 0; i < 3; i++) {
	    if (!bot.currentMaps[0]->neighbors[req[i]]) {
		float pos[2] = { bot.currentMaps[0]->pos[0] + posModifier[req[i]][0] * MAP_SIZE, bot.currentMaps[0]->pos[1] + posModifier[req[i]][1] * MAP_SIZE };
		struct Map * newMap = Map_Initialize(pos, DEFAULT_VAL);
		bot.currentMaps[0]->neighbors[req[i]] = newMap;
		newMap->neighbors[(req[i] + 4) % 8] = bot.currentMaps[0];
		
		Bot_Reinforce_Neighbors(newMap);
	    }
	    bot.currentMaps[i+1] = bot.currentMaps[0]->neighbors[req[i]];
	}

	/* copy current map to local map */
	for (char i = 0; i < 4; i++) {
	    if (!bot.localMaps[i]) bot.localMaps[i] = Map_Initialize(bot.currentMaps[i]->pos, DEFAULT_VAL);
	    else {
		memcpy(bot.localMaps[i]->pos, bot.currentMaps[i]->pos, sizeof(float) * 2);
		/* nibble multiply back in global map */
		for (char j = 0; j < MAP_UNITS; j++)
		    for (char k = 0; k < MAP_UNITS; k++) {
			bot.localMaps[i]->grid[j][k] = bot.currentMaps[i]->grid[j][k];
			bot.localViewMaps[i][j][k] = 0;
		    }
	    }
	}

	bot.mapChangeLock = 0;
    }
}

void Bot_Reinforce_Neighbors(struct Map * map) {    
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

void Bot_Map_Update() {
    if (bot.usCount == 0) Bot_Map_Required();
    float obstaclePos[US_SENSORS][2];
    float valid[US_SENSORS] = {0};
    char numPos = distanceToPos(obstaclePos, valid, bot.distances);
    
    if (numPos > 0) {
	/* loop each local cell, check validity and modify */
	float max = 0;
	float probMap[4][MAP_UNITS][MAP_UNITS] = {{{0}}};
	float viewVec[2] = { bot.pos[0] + cos(bot.pos[2]), bot.pos[1] - sin(bot.pos[2]) };
	for (char i = 0; i < 4; i++) {
	    for (char j = 0; j < MAP_UNITS; j++) {
		for (char k = 0; k < MAP_UNITS; k++) {
		    float pos[2] = { bot.localMaps[i]->pos[0] + MAP_RES * j, bot.localMaps[i]->pos[1] + MAP_RES * k };
		    float posVec[2] = { pos[0] - bot.pos[0], pos[1] - bot.pos[1] };
		    float angle = getAngle(viewVec[0], viewVec[1], posVec[0], posVec[1]);
		    float distance = getDistance(pos[0], pos[1], bot.pos[0], bot.pos[1]);
		    
		    /* make sure position is not behind reading */
		    if (angle > -SENSOR_OFFSET && angle < SENSOR_OFFSET && distance > MIN_US_DIST) {
			char flag = 0;
			for (char l = 0; l < US_SENSORS; l++) {
			    if (angle > (sensorOffsets[l] - SENSOR_ANGLE) && angle < (sensorOffsets[l] + SENSOR_ANGLE)) {
				if ((!valid[l] && distance < MAX_US_DIST) || (valid[l] && distance < bot.distances[l])) flag = 1;
				break;
			    }
			}
		    
			if (flag) {
			    bot.localViewMaps[i][j][k] = bot.usCount + 1;
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
		    if (bot.localViewMaps[i][j][k] == bot.usCount + 1) {
			char nibbleProb = (char)(probMap[i][j][k] * 16.0 / max);
			bot.localMaps[i]->grid[j][k] = ((bot.localMaps[i]->grid[j][k] + nibbleProb) >> 1) & 0xF;
		    }
		}
	    }
	}
    }
    
    
    bot.usCount = (bot.usCount + 1) % 10;
}

void Bot_Optimise_Local() {
    /* save local map back to global */
    for (char i = 0; i < 4; i++) {
	if (bot.localMaps[i] && bot.currentMaps[i]) {
	    for (char j = 0; j < MAP_UNITS; j++) {
		for (char k = 0; k < MAP_UNITS; k++) {
		    bot.currentMaps[i]->grid[j][k] = ((bot.currentMaps[i]->grid[j][k] + bot.localMaps[i]->grid[j][k]) >> 1) & 0xF;
		}
	    }
	}
    }
}

void Bot_Controller() { 
    float convDuty[2] = {0};		// convert duty cycle to correct value
    
    switch (bot.state) {
	case IDLE: {
	    /* IDLE state: do not move */
	    for (int i = 0; i < 2; i++) {
		bot.duty[i] = 0.0;
		convDuty[i] = 0.0;
	    }
	    LATBCLR = _LATB_LATB4_MASK;
	    LATACLR = _LATA_LATA3_MASK;
	    break;
	}
	
	case NAVIGATE: {
	    /* NAVIGATE state: determine duty cycle */
	    float minDist = Bot_InputPos_Update();
	    
	    /* navigate: obtain error angle and distance */
	    float viewVec[2] = { bot.pos[0] + cos(bot.pos[2]), bot.pos[1] + sin(bot.pos[2]) };
	    float inputVec[2] = { bot.inputPos[0] - bot.pos[0], bot.inputPos[1] - bot.pos[1] };
	    bot.ePos[2] = atan2(inputVec[1], inputVec[0]) - atan2(viewVec[1], viewVec[0]);
	    float distance = getDistance(bot.inputPos[0], bot.inputPos[1], bot.pos[0], bot.pos[1]);
	    if (minDist < distance) distance = minDist;
    
	    float rotSign = (bot.ePos[2] < 0.0) ? -1.0 : 1.0;
	    float maxTurn = fabs(bot.ePos[2]) / (BETA + fabs(bot.ePos[2]));
	    float newDuty[2] = {0};

	    if (fabs(bot.ePos[2]) > ERROR_MAX) {
		newDuty[0] = rotSign * -maxTurn / K_TURN;
		newDuty[1] = rotSign * maxTurn / K_TURN;
	    } else {
		float maxSpeed = distance / (ALPHA + distance);
		newDuty[0] = maxSpeed / 2.0 + rotSign * -maxTurn;
		newDuty[1] = maxSpeed / 2.0 + rotSign * maxTurn;
	    }

	    for (int i = 0; i < 2; i++) {
		/* set new duty cycles */
		if (newDuty[i] < bot.duty[i] - GAMMA) bot.duty[i] -= GAMMA;
		else if (newDuty[i] > bot.duty[i] + GAMMA) bot.duty[i] += GAMMA;
		//else if (fabs(newDuty[i]) < MIN_PWM) bot.duty[i] = 0.0;
		else bot.duty[i] = newDuty[i];
	    }


	    /* negative duty cycles should reverse rotation */
	    if (bot.duty[0] > 0) {
		LATACLR = _LATA_LATA3_MASK;
		convDuty[0] = bot.duty[0];
	    } else {
		LATASET = _LATA_LATA3_MASK;
		convDuty[0] = 1 - bot.duty[0];
	    }
	    if (bot.duty[1] > 0) {
		LATBCLR = _LATB_LATB4_MASK;
		convDuty[1] = bot.duty[1];
	    } else {
		LATBSET = _LATB_LATB4_MASK;
		convDuty[1] = 1 - bot.duty[1];
	    }
	    break;
	}
    };
    
    OC4RS = (int)(convDuty[0] * PWM_H);
    OC5RS = (int)(convDuty[1] * PWM_H);
}

/*
void Bot_Add_Instruction(float x, float y) {
    if ((bot.execIndex + INPUTQ_SIZE - 1) % INPUTQ_SIZE != bot.addIndex) {
	bot.inputPosQueue[bot.addIndex][0] = x;
	bot.inputPosQueue[bot.addIndex][1] = y;
	bot.addIndex = (bot.addIndex + 1) % INPUTQ_SIZE;
    }
}
*/

void Bot_Display_Status() {
    float mapPos[4][2] = {{0}};
    for (char i = 0; i < 4; i++) {
	if (bot.currentMaps[i]) memcpy(mapPos[i], bot.currentMaps[i], sizeof(float) * 2);
    }
    
    snprintf(bot.buf, 100, "\33[2J\33[H(%ds) Bot state: %d, battery: %.2f%%\r\n", bot.time, bot.state, bot.battery);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "pos: (%.2f, %.2f, %.2f)\r\n", bot.pos[0], bot.pos[1], bot.pos[2] * 180.0 / M_PI);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "input: (%.2f, %.2f)\r\n", bot.inputPos[0], bot.inputPos[1]);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "error: (%.2f, %.2f, %.2f)\r\n", bot.ePos[0], bot.ePos[1], bot.ePos[2] * 180.0 / M_PI);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "duty: (%.2f, %.2f)\r\n", bot.duty[0], bot.duty[1]);
    UART_Write_String(bot.buf, strlen(bot.buf));    
    snprintf(bot.buf, 100, "acc: (%.2f, %.2f), gyro: (%.2f), bias: (%.2f, %.2f, %.2f)\r\n", bot.imuData[0], bot.imuData[1], bot.imuData[2], bot.bias[0], bot.bias[1], bot.bias[2]);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "dist: (%.2f, %.2f, %.2f)\r\n", bot.distances[0], bot.distances[1], bot.distances[2]);
    UART_Write_String(bot.buf, strlen(bot.buf));
    snprintf(bot.buf, 100, "numMaps: %d, current: (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)\r\n", bot.numMaps,
	mapPos[0][0], mapPos[0][1], mapPos[1][0], mapPos[1][1], mapPos[2][0], mapPos[2][1], mapPos[3][0], mapPos[3][1]);
    UART_Write_String(bot.buf, strlen(bot.buf));
    for (char i = 0; i < 4; i++) {
	if (bot.currentMaps[i]) {
	    snprintf(bot.buf, 100, "%d neighbors: %d, %d, %d, %d, %d, %d, %d, %d\r\n", i, bot.currentMaps[i]->neighbors[0], bot.currentMaps[i]->neighbors[1], bot.currentMaps[i]->neighbors[2], bot.currentMaps[i]->neighbors[3],
		bot.currentMaps[i]->neighbors[4], bot.currentMaps[i]->neighbors[5], bot.currentMaps[i]->neighbors[6], bot.currentMaps[i]->neighbors[7]);
	    UART_Write_String(bot.buf, strlen(bot.buf));
	}
    }
}

void Bot_Display_Map(struct Map * map) {
    snprintf(bot.buf, 100, "\r\nMap pos: (%.2f, %.2f)\r\n", map->pos[0], map->pos[1]);
    UART_Write_String(bot.buf, strlen(bot.buf));
    for (char i = 0; i < MAP_UNITS; i++) {
	snprintf(bot.buf, 100, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
	    map->grid[0][i], map->grid[1][i], map->grid[2][i], map->grid[3][i], map->grid[4][i],
	    map->grid[5][i], map->grid[6][i], map->grid[7][i], map->grid[8][i], map->grid[9][i],
	    map->grid[10][i], map->grid[11][i], map->grid[12][i], map->grid[13][i], map->grid[14][i],
	    map->grid[15][i], map->grid[16][i], map->grid[17][i], map->grid[18][i], map->grid[19][i]);
	UART_Write_String(bot.buf, strlen(bot.buf));
    }
}

/* map functions */
struct Map * Map_Initialize(float pos[2], char fillVal) {
    struct Map * map = malloc(sizeof(struct Map));
    memcpy(map->pos, pos, sizeof(float) * 2);
    for (char i = 0; i < 8; i++) map->neighbors[i] = NULL;
    for (char i = 0; i < MAP_UNITS; i++)
	for (char j = 0; j < MAP_UNITS; j++)
	    map->grid[i][j] = fillVal;
    return map;
}

void Map_Destroy(struct Map * map) {
    if (map != NULL) {
	//free(map->neighbors);
	free(map);
	map = NULL;
    }
    bot.numMaps--;
}

char Map_Contains(struct Map * map, float * pos) {
    float xRange[2] = { map->pos[0], map->pos[0] + MAP_SIZE };
    float yRange[2] = { map->pos[1], map->pos[1] + MAP_SIZE };
    return ((pos[0] > xRange[0]) && (pos[0] < xRange[1]) && (pos[1] > yRange[0]) && (pos[1] < yRange[1]));
}

/* Interrupt functions */

/* Ultrasonic echo timer */
void __ISR(_TIMER_2_VECTOR, IPL2SOFT) TMR2_IntHandler() {
    IFS0CLR = _IFS0_T2IF_MASK;
    bot.distances[bot.usState] = (float)TMR2 * SOUND_SPEED;
    if (bot.distances[bot.usState] > MAX_US_DIST || bot.distances[bot.usState] < MIN_US_DIST) bot.distances[bot.usState] = MAX_US_DIST;
    
    /* check if all readings taken, update map */
    if (bot.usState == US_SENSORS - 1) {
	//Bot_Map_Update();
    }
}

/* Sample timer */
void __ISR(_TIMER_4_VECTOR, IPL2SOFT) TMR4_IntHandler() {
    IFS0CLR = _IFS0_T4IF_MASK;
    
    Bot_Pos_Update();
    if (bot.count % 2) {
	Bot_Trigger_Ultrasonic();
    }
    Bot_Controller();
    
    bot.count = (bot.count + 1) % 10;//(int)FREQ;
    if (bot.count == 0) {
	bot.time++;
	AD1CON1SET = _AD1CON1_SAMP_MASK;
	Bot_Display_Status();
	//Bot_Display_Map(bot.currentMaps[0]);
    }
}

void __ISR(_TIMER_5_VECTOR, IPL2SOFT) TMR5_IntHandler() {
    IFS0CLR = _IFS0_T5IF_MASK;
}

/* Battery sampler */
void __ISR(_ADC_VECTOR, IPL2SOFT) ADC_IntHandler() {
    if (IFS0bits.AD1IF) {
	IFS0CLR = _IFS0_AD1IF_MASK;
	bot.battery = (((ADC1BUF0 >> 8) & 0xFF) - ADC_OFFSET) * ADC_SCALE;
    }
}

/* Fall sensor */
void __ISR(_EXTERNAL_3_VECTOR, IPL2SOFT) Ext3_IntHandler() {
    IFS0CLR = _IFS0_INT3IF_MASK;
}

/* User */
void __ISR(_CHANGE_NOTICE_VECTOR, IPL2SOFT) CNB_IntHandler() {
    if ((bot.portCN & 0x20) && !(PORTB & 0x20)) {
	/* button press occured */
	bot.state ^= 1;
	
	switch (bot.state) {
	    case IDLE: {
		bot.numBias = 0;
		for (int i = 0; i < 0; i++) bot.bias[i] = 0;
		break;
	    }
	    
	    case NAVIGATE: {
		bot.bias[0] /= bot.numBias;
		bot.bias[1] /= bot.numBias;
		bot.bias[2] *= M_PI / (180.0 * bot.numBias);
		break;
	    }
	}
	
    }
    bot.portCN = PORTB;
    IFS1CLR = _IFS1_CNBIF_MASK;
}


/* Peripheral functions*/
void Init() {
    /* Random function */
    srand(PORTA);
    
    /* Ports */
    ANSELA = 0x1;
    TRISA = 0x1;
    LATA = 0x0;
    LATASET = _LATA_LATA1_MASK;

    ANSELB = 0x0;
    TRISB = 0x1A0;
    LATB = 0xC;
    LATBSET = _LATB_LATB10_MASK;

    T2CKR = 0x4;		// RB7 = T2CK
    U1RXR = 0x3;		// RB13 = RX
    RPA2R = 0x5;		// RA2 = OC4
    RPA4R = 0x6;		// RA4 = OC5
    RPB15R = 0x1;		// RB15 = TX
    INT3R = 0x4;		// RB8 = INT3
    CNCONB = 0x8000;
    CNENB = 0x20;		// CNB5 
    bot.portCN = PORTB;
    
    /* Interrupts */
    __builtin_disable_interrupts();
    INTCONSET = _INTCON_MVEC_MASK;
    IPC2 = 0xA;			// TMR2
    IPC3 = 0xA00000A;		// INT3, TMR3
    IPC4 = 0xA;			// TMR4
    IPC5 = 0xA00000A;		// TMR5, ADC1
    IPC8 = 0xA0000;			// CN
    IFS0 = 0x0;
    IFS1 = 0x0;
    IEC0 = 0x110C0200;		// TMR2, INT3, TMR4, TMR5, ADC1
    IEC1 = 0x4000;		// CNB
    __builtin_enable_interrupts();
    
    /* Ultrasonic */
    TMR2 = 0x0;
    PR2 = 0xFFFF;           // f = 15 Hz
    T2CON = 0x80D0;         // gated time, 1:32 prescaler, TMR2 f = 1 MHz
    
    /* PWM */
    TMR3 = 0x0;
    PR3 = PWM_T;
    OC4R = 0x0;
    OC4RS = 0x0;
    OC4CON = 0xE;		// PWM mode without fault protection, timer 3
    OC5R = 0x0;
    OC5RS = 0x0;
    OC5CON = 0xE;
    T3CON = 0x8060;		// 1:64 prescaler = 500 kHz
    OC4CONSET = _OC4CON_ON_MASK;
    OC5CONSET = _OC5CON_ON_MASK;
    
    /* Sampling timer */
    TMR4 = 0x0;
    PR4 = 0x8235;		// f = 30.0003 Hz
    T4CON = 0x8050;		// 1:32 prescaler = 2 MHz (reset to 0x8050)
    
    /* Long counter */
    TMR5 = 0x0;
    PR5 = 0xFFFF;		// 2 Hz
    T5CON = 0x70;		// 1:256 prescaler = 125 kHz    
    
    /* I2C */
    I2C2BRG = (int)(PBCLK / (2 * I2C_BAUD) - 2);
    I2C2CON = _I2C2CON_ON_MASK;
    
     /* ADC */
    AD1CON1 = 0x2E0;			// fractional output, manual sampling
    AD1CON2 = 0x0;				// no scan, single conversion per interrupt
    AD1CON3 = 0xF03;			// 15 Tad sample, 250ns conversion
    AD1CHS = 0x0;				// AN0 - Vrefl
    AD1CSSL = 0x0;				// no inputs scanned
    AD1CON1SET = _AD1CON1_ADON_MASK;
    AD1CON1SET = _AD1CON1_SAMP_MASK;
    
    /* UART */
    U1MODE = 0x0;
    U1STA = 0x0;
    U1BRG = (int)(PBCLK / (16 * UART_BAUD) - 1);
    U1STASET = (_U1STA_UTXISEL0_MASK | _U1STA_URXEN_MASK | _U1STA_UTXEN_MASK);
    U1MODESET = _U1MODE_ON_MASK;
}

void UART_Write(char data) {
    while (U1STAbits.UTXBF);
    U1TXREG = data;
}

void UART_Write_String(char * data, int len) {
    while (len--) {
	UART_Write(*data);
	data++;
    }
}

void I2C_Master_Start() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.SEN = 1;
    while (I2C2CONbits.SEN);
    //snprintf(bot.buf, 100, "start: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Stop() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.PEN = 1;
    while (I2C2CONbits.PEN);
    //snprintf(bot.buf, 100, "stop: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Restart() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.RSEN = 1;
    while (I2C2CONbits.RSEN);
    //snprintf(bot.buf, 100, "restart: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Ack_Nack(int val) {
    // ACK = 0 (slave should send another byte)
    // NACK = 1 (no more bytes requested by slave)
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.ACKDT = val;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN);
    //snprintf(bot.buf, 100, "ack: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

int I2C_Master_Write(char byte) {
    I2C2TRN = byte;
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    //snprintf(bot.buf, 100, "write: %d (%d)\r\n", I2C2STAT, byte);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    if (I2C2STATbits.ACKSTAT) return 0;
    return 1;
}

char I2C_Master_Read() {
    I2C2CONbits.RCEN = 1;
    //snprintf(bot.buf, 100, "read: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    while (!I2C2STATbits.RBF);
    return I2C2RCV;
}

int I2C_Write(char periphAdd, char regAdd, char data) {
    int ret = 0;
    I2C_Master_Start();
    ret |= I2C_Master_Write((periphAdd << 1) | I2C_W);
    ret |= I2C_Master_Write(regAdd);
    ret |= I2C_Master_Write(data);
    I2C_Master_Stop();
    return ret;
}

int I2C_Read(char periphAdd, char regAdd, char * data, int len) {
    int ret = 0;
    I2C_Master_Start();
    ret |= I2C_Master_Write((periphAdd << 1) | I2C_W);
    ret |= I2C_Master_Write(regAdd);
    I2C_Master_Stop();
    I2C_Master_Start();
    I2C_Master_Write((periphAdd << 1) | I2C_R);
    
    for (int i = 0; i < len; i++) {
	data[i] = I2C_Master_Read();
	if (i != len - 1) I2C_Master_Ack_Nack(0);
    }
    I2C_Master_Ack_Nack(1);
    I2C_Master_Stop();
}

char IMU_Init() {
    int ret = 0;
    //snprintf(bot.buf, 100, "init: %d (%d)\r\n", I2C2STAT, PORTB);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    ret |= I2C_Write(IMU_ADD, SMPLRT_DIV, 0x07);
    ret |= I2C_Write(IMU_ADD, PWR_MGMT_1, 0x01);
    ret |= I2C_Write(IMU_ADD, CONFIG, 0x00);
    ret |= I2C_Write(IMU_ADD, ACCEL_CONFIG, 0x08);
    ret |= I2C_Write(IMU_ADD, GYRO_CONFIG, 0x08);
    ret |= I2C_Write(IMU_ADD, INT_ENABLE, 0x01);
    
    /* test everything is working */
    char whoami = 0;
    I2C_Read(IMU_ADD, WHO_AM_I, &whoami, 1);
    
    return whoami;
}


/* Helper functions */
float getAngle(float x1, float y1, float x2, float y2) {
    float angle = acos((x1*x2 + y1*y2) / (sqrt(x1*x1 + y1*y1) * sqrt(x2*x2 + y2*y2)));
    if (isnanf(angle)) return 0.0;
    return angle;	    
}

float getDistance(float x1, float y1, float x2, float y2) {
    return sqrtf(powf(x1 - x2, 2) + powf(y1 - y2, 2));
}

char distanceToPos(float pos[][2], float * valid, float * distances) {
    char index = 0;
    for (char i = 0; i < US_SENSORS; i++) {
	if (distances[i] >= MIN_US_DIST && distances[i] <= MAX_US_DIST) {
	    pos[i][0] = bot.pos[0] + distances[i] * cos(sensorOffsets[i] - bot.pos[2]);
	    pos[i][1] = bot.pos[1] + distances[i] * sin(sensorOffsets[i] - bot.pos[2]);
	    valid[i] = 1;
	    index++;
	}
    }
    return index;
}

char Multivariate_Gaussian(float meanX, float meanY, float posX, float posY) {
    float devX = (posX - meanX) / US_SIGMA;
    float devY = (posY - meanY) / US_SIGMA;
    return 1 / (2*M_PI * US_SIGMA * US_SIGMA) * exp(-0.5 * (powf(devX, 2) + powf(devY, 2)));
}

float Box_Muller(float mu, float sigma) {
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    float z0 = sqrt(-2*log(u1)) * cos(2*M_PI*u2);
    return z0*sigma + mu;
}

/*
void matrix_plus(int len, float result[][len], float mat1[][len], float ma6t2[][len]) {
    for (int i = 0; i < len; i++) {
	for (int j = 0; j < len; j++) {
	    result[i][j] = mat1[i][j] + mat2[i][j];
	}
    }
} */

void matrix_mul( int nRows,  int nCols, int nAdd, float result[][nCols], float mat1[][nAdd], float mat2[][nCols]) {
    for (int i = 0; i < nRows; i++) {
	for (int j = 0; j < nCols; j++) {
	    result[i][j] = 0.0;
	    for (int k = 0; k < nAdd; k++) 
		result[i][j] += mat1[i][k] * mat2[k][j];
	}
    }
}

/* inverse of 3x3 matrix /
void matrix_inv(float result[3][3], float mat[3][3]) {
    // calculate matrix of minors
    float minors[3][3] = {{0}};
    float detMat[2][2] = {{0}};
    int offsetK = 0;
    int offsetL = 0;
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    // get determinant
	    offsetK = 0;
	    for (int k = 0; k < 2; k++) {
		offsetL = 0;
		for (int l = 0; l < 2; l++) {
		    if (k == i) offsetK = 1;
		    if (l == j) offsetL = 1;
		    detMat[k][l] = mat[k + offsetK][l + offsetL];
		}
	    }
	    minors[i][j] = detMat[0][0] * detMat[1][1] - detMat[0][1] * detMat[1][0];
	}
    }
    
    float det = mat[0][0] * minors[0][0] + mat[0][1] * minors[0][1] + mat[0][2] * minors[0][2];
    
    // cofactors and adjoint
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    if ((i + j) % 2 != 0) minors[i][j] *= -1;
	}
    }
    
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    result[i][j] = minors[j][i] / det;
	}
    }
}

 */
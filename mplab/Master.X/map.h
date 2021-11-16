#ifndef _MAP_H
#define _MAP_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include "aux.h"

// map properties
#define MAP_RES 0.08
#define MAP_UNITS 16
#define MAP_SIZE 1.28
#define MAP_PRIOR 0.2
#define LOG_PRIOR log(MAP_PRIOR / (1 - MAP_PRIOR))
#define US_SENSORS 3
#define SENSOR_O 0.6981317				// 40 deg
#define SENSOR_A 0.3490659				// 20 deg

// log odds update
#define D_1 0.04
#define D_2 0.14
#define D_3 0.18
#define L_LOW 0.3
#define L_MED 0.5
#define L_HIGH 0.8

static const uint8_t posMod[8][2] = {
	{-1, 1},
	{0, 1},
	{1, 1},
	{1, 0},
	{1, -1},
	{0, -1},
	{-1, -1},
	{-1, 0}
};

// map struct
struct Map {
	float pos[2];
	float grid[MAP_UNITS][MAP_UNITS];
	struct Map * neighbors[8];
};

void Map_Init(struct Map ** map, float pos[2]);
void Map_Reinforce(struct Map * map);
void Map_Update(struct Map * map, float pos[3], float dist[US_SENSORS]);
void Map_Cell_Update(float * cell, float dist, float z);
char Map_Contains(struct Map * map, float pos[2]);
char Map_Pos_Collide(struct Map * map, float pos[2]);

#endif
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
#define MAP_PRIOR 0.5
#define LOG_PRIOR log(MAP_PRIOR / (1 - MAP_PRIOR))
#define US_SENSORS 3
#define SENSOR_O 0.6981317				// 40 deg
#define SENSOR_A 0.3490659				// 20 deg

// log odds update
#define D_1 0.04
#define D_2 0.2
#define D_3 0.24
#define L_LOW -0.847298
#define L_MED 0.0
#define L_HIGH 1.386294

// frontier detection
#define PH 0.99
#define PL 0.01
#define LH log(PH / (1 - PH))
#define LL log(PL / (1 - PL))
#define BLOCK_SIZE 5
#define EDGE_BAR 2.0
#define EDGE_H 4.0
#define EDGE_L -1.0
#define FRONTIER_L 1.38

static const int posMod[8][2] = {
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
void Map_Edge(struct Map * map, float edgeGrid[MAP_UNITS][MAP_UNITS]);
void Map_Blur(float blurGrid[MAP_UNITS][MAP_UNITS], float edgeGrid[MAP_UNITS][MAP_UNITS]);
char Map_Frontier(struct Map * map, float pos[3], uint8_t frontier[2]);

#endif
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
#define MAP_UNITS_BIT 2
#define MAP_SIZE 1.28
#define MAP_PRIOR 0.5
#define LOG_PRIOR log(MAP_PRIOR / (1 - MAP_PRIOR))
#define US_SENSORS 3
#define SENSOR_O 30. * M_PI/180.
#define SENSOR_A 12. * M_PI/180.
#define MAX_US_DIST 0.9

// log odds update
#define D_1 0.06
#define D_2 0.22
#define D_3 0.32
#define L_LOW -0.847298
#define L_MED 0.0
#define L_MEDHIGH 0.619039
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
#define FRONTIER_DIST 0.12
#define FRONTIER_TRIES 64
#define FRONTIER_MIN 0.05

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

static const int safePosMod[9][2] = {
    {0,0},
    {-1, -1},
    {0, -1},
    {1, -1},
    {1, 0},
    {1, 1},
    {0, 1},
    {-1, 1},
    {-1, 0}
};

// map struct
struct Map {
	float pos[2];
	float grid[MAP_UNITS][MAP_UNITS];
    char visited[MAP_UNITS][MAP_UNITS_BIT];
	struct Map * neighbors[8];
};

void Map_Init(struct Map ** map, float pos[2]);
char Map_PosToIndex(struct Map * map, uint8_t index[2], float pos[2]);
char Map_IndexToPos(struct Map * map, float pos[2], uint8_t index[2]);

void Map_Reinforce(struct Map * map);
void Map_Update(struct Map * map, float pos[3], float dist[US_SENSORS]);
void Map_Cell_Update(float * cell, float dist, float z);
void Map_Update_Visit(struct Map * map, float pos[2]);
char Map_Check_Visited(struct Map * map, float pos[2]);
char Map_Contains(struct Map * map, float pos[2]);
char Map_Pos_Collide(struct Map * map, float pos[2]);

void Map_Edge(struct Map * map, float edgeGrid[MAP_UNITS][MAP_UNITS]);
void Map_Blur(float blurGrid[MAP_UNITS][MAP_UNITS], float edgeGrid[MAP_UNITS][MAP_UNITS]);
void Map_SafeZone(struct Map * map, char safeGrid[3*MAP_UNITS][3*MAP_UNITS]);
char Map_Frontier(struct Map * map, float pos[3], float exPos[2]);

#endif
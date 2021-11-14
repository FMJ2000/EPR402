#ifndef _MAP_H
#define _MAP_H

#include <stdlib.h>
#include <stdint-gcc.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

// map properties
#define MAP_RES 0.08
#define MAP_UNITS 16
#define MAP_SIZE 1.28
#define MAP_PRIOR 0.5

// map struct
struct Map {
	float pos[2];
	float grid[MAP_UNITS][MAP_UNITS];
	struct Map * neighbors[8];
};

void Map_Init(struct Map ** map, float pos[2]);
void Map_Reinforce(struct Map * map);
char Map_Contains(struct Map * map, float pos[2]);

#endif
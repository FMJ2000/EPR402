#ifndef WORLD_H
#define WORLD_H

#include <stdlib.h>
#include <stdio.h>
#include "bot.h"

/* real bot x, y, r positions */
typedef struct {
	int x;
	int y;
	int r;
	int width;
	int height;
	int8_t ** map;
} World;

World * worldConstructor(int x, int y, int r, int width, int height);
int worldDestructor(World * world);
int8_t ** worldInitMap(int width, int height);
void worldPrint(World * world);
int8_t * worldView(World * world);
void worldMove(World * world, int forward, int turn);

#endif
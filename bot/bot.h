#ifndef BOT_H
#define BOT_H

#include <stdlib.h>
#include <stdio.h>
#include "node.h"

/* relative bot x, y, r positions to start, nodes are viewpoints on map */
typedef struct {
	int x;
	int y;
	int r;
	int width;
	int height;
	void * rootNode;
	int8_t ** map;
} Bot;

Bot * botConstructor(int width, int height);
int botDestructor(Bot * bot);
void botPrint(Bot * bot);
void botAddNode(Bot * bot, Node * node);

#endif
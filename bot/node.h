#ifndef NODE_H
#define NODE_H

#include <stdlib.h>
#include <stdio.h>

/* single view snapshot in position and perception */
typedef struct {
	int x;
	int y;
	int r;
	int8_t * view;
	void * next;
} Node;

Node * nodeConstructor(int x, int y, int r, int8_t * view);

#endif
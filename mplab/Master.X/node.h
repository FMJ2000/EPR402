#ifndef _NODE_H
#define _NODE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#define Q_LEN 512
#define NAV_STEP 0.24

// path planning node
struct Node {
	int pos[2];						// relative to start pos
	float posf[2];				// absolute
	float g;
	float h;
	struct Node * parent;
};

struct NodeQueue {
	struct Node * queue[Q_LEN];
	unsigned int startIndex;
	unsigned int endIndex;
};

char NodeQueue_Add(struct NodeQueue * queue, struct Node * node);
struct Node * NodeQueue_Remove(struct NodeQueue * queue, unsigned int index);
char NodeQueue_Empty(struct NodeQueue * queue);
int NodeQueue_Contains(struct NodeQueue * queue, int pos[2]);
unsigned int NodeQueue_Best(struct NodeQueue * queue);

#endif
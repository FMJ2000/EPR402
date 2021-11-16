#ifndef _NODE_H
#define _NODE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#define Q_LEN 512

// path planning node
struct Node {
	float pos[2];
	float g;
	float h;
};

struct NodeQueue {
	struct Node * queue[Q_LEN];
	unsigned int startIndex;
	unsigned int endIndex;
};

char NodeQueue_Add(struct NodeQueue * queue, float pos[2], float g, float h);
char NodeQueue_Remove(struct NodeQueue * queue, unsigned int index);
char NodeQueue_Empty(struct NodeQueue * queue);
char NodeQueue_Contains(struct NodeQueue * queue, struct Node * node);

#endif
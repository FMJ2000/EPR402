#ifndef _NODE_H
#define _NODE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#define Q_LEN 512
#define NAV_STEP 0.15
#define NAV_SQRT 0.21213203
#define MIN_SEARCH_GOAL 0.11

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

void NodeQueue_Destroy(struct NodeQueue * queue);
char NodeQueue_Add(struct NodeQueue * queue, struct Node * node);
void NodeQueue_Remove(struct NodeQueue * queue, struct Node ** node, unsigned int index);
char NodeQueue_Empty(struct NodeQueue * queue);
int NodeQueue_Contains(struct NodeQueue * queue, int pos[2]);
unsigned int NodeQueue_Best(struct NodeQueue * queue);


#endif
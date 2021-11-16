#include "node.h"

// add node to end of queue
char NodeQueue_Add(struct NodeQueue * queue, struct Node * node) {
	if (!queue) return 0;
	// check if queue is full
	if ((queue->endIndex + 1) % Q_LEN == queue->startIndex) return 0;
	queue->queue[queue->endIndex] = node;
	queue->endIndex = (queue->endIndex + 1) % Q_LEN;
	return 1;
}

// remove node at position and shift nodes up
struct Node * NodeQueue_Remove(struct NodeQueue * queue, unsigned int index) {
	if (!queue || index < 0 || index >= Q_LEN || !queue->queue[index]) return NULL;
	struct Node * returnNode = queue->queue[index];
	unsigned int amount = (index > queue->startIndex) ? index - queue->startIndex : Q_LEN - (queue->startIndex - index);
	for (unsigned int i = 0; i < amount; i++) {
		unsigned int newIndex = (index > 0) ? index - 1 : Q_LEN - 1;
		queue->queue[index] = queue->queue[newIndex];
	}
	queue->startIndex = (queue->startIndex + 1) % Q_LEN;
	return returnNode;
}

char NodeQueue_Empty(struct NodeQueue * queue) {
	if (!queue) return 0;
	return (queue->startIndex == queue->endIndex);
}

// return index if found, -1 otherwise
int NodeQueue_Contains(struct NodeQueue * queue, int pos[2]) {
	if (!queue) return -1;
	for (unsigned int i = 0; i < Q_LEN; i++) {
		if (queue->queue[i] && queue->queue[i]->pos[0] == pos[0] && queue->queue[i]->pos[1] == pos[1]) 
			return i;
	}
	return -1;
}

unsigned int NodeQueue_Best(struct NodeQueue * queue) {
	if (!queue) return 0;
	unsigned int bestIndex = 0;
	float bestf = 10000;

	for (unsigned int i = 0; i < Q_LEN; i++) {
		float f = queue->queue[i]->g + queue->queue[i]->h;
		if (f < bestf) {
			bestf = f;
			bestIndex = i;
		}
	}

	return bestIndex;
}
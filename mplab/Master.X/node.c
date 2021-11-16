#include "node.h"

char NodeQueue_Init(struct NodeQueue ** queue) {
	*queue = malloc(sizeof(struct NodeQueue *));
	(*queue)->startIndex = 0;
	(*queue)->endIndex = 0;
	for (unsigned int i = 0; i < Q_LEN; i++)	(*queue)->queue[i] = NULL;
}

void NodeQueue_Destroy(struct NodeQueue * queue) {
	if (!queue) return;
	for (unsigned int i = 0; i < Q_LEN; i++)
		if (queue->queue[i]) free(queue->queue[i]);
	free(queue);
	queue = NULL;
}

// add node to end of queue
char NodeQueue_Add(struct NodeQueue * queue, float pos[2], float g, float h) {
	if (!queue) return 0;
	// check if queue is full
	if ((queue->endIndex + 1) % Q_LEN == queue->startIndex) return 0;
	struct Node node = { .pos={pos[0], pos[1]}, .g=g, .h=h };
	queue->queue[queue->endIndex] = malloc(sizeof(struct Node *));
	*queue->queue[queue->endIndex] = node;
	queue->endIndex = (queue->endIndex + 1) % Q_LEN;
	return 1;
}

// remove node at position and shift nodes up
char NodeQueue_Remove(struct NodeQueue * queue, unsigned int index) {
	if (!queue || !queue->queue[index]) return 0;
	free(queue->queue[index]);
	unsigned int amount = (index > queue->startIndex) ? index - queue->startIndex : Q_LEN - (queue->startIndex - index);
	for (unsigned int i = 0; i < amount; i++) {
		unsigned int newIndex = (index > 0) ? index - 1 : Q_LEN - 1;
		queue->queue[index] = queue->queue[newIndex];
	}
	queue->startIndex = (queue->startIndex + 1) % Q_LEN;
}

char NodeQueue_Empty(struct NodeQueue * queue) {
	if (!queue) return 0;
	return (queue->startIndex == queue->endIndex);
}

// return index if found, -1 otherwise
char NodeQueue_Contains(struct NodeQueue * queue, struct Node * node) {
	if (!queue) return 0;
	for (unsigned int i = 0; i < Q_LEN; i++) if (queue->queue[i] == node) return i;
	return -1;
}
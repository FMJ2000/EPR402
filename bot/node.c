#include "node.h"

Node * nodeConstructor(int x, int y, int r, int8_t * view) {
	Node * node = (Node *) malloc(sizeof(Node));
	node->x = x;
	node->y = y;
	node->r = r;
	node->view = view;
	node->next = NULL;
	return node;
}
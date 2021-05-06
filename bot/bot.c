#include "bot.h"

Bot * botConstructor(int width, int height) {
	Bot * bot = (Bot *) malloc(sizeof(Bot));
	bot->x = width / 2;
	bot->y = height / 2;
	bot->r = 0;
	bot->width = width;
	bot->height = height;
	bot->map = (int8_t **) malloc(height * sizeof(int8_t *));
	for (int i = 0; i < height; i++) {
		bot->map[i] = (int8_t *) malloc(width * sizeof(int8_t));
		for (int j = 0; j < width; j++)
			bot->map[i][j] = ' ';
	}
	bot->map[bot->y][bot->x] = '^';
	bot->rootNode = NULL;
	return bot;
}

int botDestructor(Bot * bot) {
	for (int i = 0; i < bot->height; i++)
		free(bot->map[i]);
	free(bot->map);
	free(bot);
	return 0;
}

void botPrint(Bot * bot) {
	int yMin = (bot->y - 5 >= 0) ? bot->y - 5 : 0;
	int yMax = (bot->y + 5 < bot->height) ? bot->y + 5 : bot->height;
	int xMin = (bot->x - 5 >= 0) ? bot->x - 5 : 0;
	int xMax = (bot->x + 5 < bot->width) ? bot->x + 5: bot->width;
	printf("Bot Map:\n");
	for (int i = yMin; i < yMax; i++) {
		for (int j = xMin; j < xMax; j++) {
			putchar(bot->map[i][j]);
			putchar(' ');
		}
		putchar('\n');
	}
	putchar('\n');
	Node * node = bot->rootNode;
	int index = 0;
	while (node != NULL) {
		printf("%d: %c, ", index, node->view[0]);
		index++;
		node = node->next;
	}
}

void botAddNode(Bot * bot, Node * node) {
	if (bot->rootNode == NULL) {
		bot->rootNode = node;
	} else {
		Node * nodePtr = bot->rootNode;
		while (nodePtr->next != NULL) {
			nodePtr = nodePtr->next;
		}
		nodePtr->next = node;
	}
}
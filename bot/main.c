#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "bot.h"
#include "world.h"

void print(World * world, Bot * bot);

int main() {
	time_t t;
	srand((unsigned) time(&t));

	char output[250];
	int x = rand() % 10 + 15;
	int y = rand() % 10 + 5;
	int r = 2;//rand() % 4;
	int width = 30;
	int height = 20;
	World * world = worldConstructor(x, y, r, width, height);
	Bot * bot = botConstructor(width * 2, height * 2);
	
	for (int i = 0; i < 30; i++) {
		print(world, bot);
		worldMove(world, 1, 0);
		int8_t * view = worldView(world);
		Node * node = nodeConstructor(bot->x, bot->y, bot->r, view);
		botAddNode(bot, node);
		sleep(1);
	}
	
	
	botDestructor(bot);
	worldDestructor(world);
}

void print(World * world, Bot * bot) {
	printf("\e[1;1H\e[2J");
	worldPrint(world);
	putchar('\n');
	botPrint(bot);
	putchar('\n');
}
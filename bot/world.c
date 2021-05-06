#include "world.h"

World * worldConstructor(int x, int y, int r, int width, int height) {
	World * world = (World *) malloc(sizeof(World));
	world->x = x;
	world->y = y;
	world->r = r;
	world->width = width;
	world->height = height;
	world->map = worldInitMap(width, height);
	return world;
}

int worldDestructor(World * world) {
	for (int i = 0; i < world->height; i++)
		free(world->map[i]);
	free(world->map);
	free(world);
	return 0;
}

int8_t ** worldInitMap(int width, int height) {	
	/* initialize map */
	int8_t ** map = (int8_t **) malloc(height * sizeof(int8_t *));
	for (int i = 0; i < height; i++) {
		map[i] = (int8_t *) malloc(width * sizeof(int8_t));
		for (int j = 0; j < width; j++)
			map[i][j] = ' ';
	}

	/* walls */
	for (int i = 0; i < height; i++) {
		map[i][0] = '#';
		map[i][width - 1] = '#';
	}
	for (int i = 0; i < width; i++) {
		map[0][i] = '#';
		map[height - 1][i] = '#';
	}

	/* furniture */
	for (int i = 1; i < 6; i++) {
		map[height - 5][i] = '@';
		map[i][width - 6] = '@';
	}

	return map;
}

void worldPrint(World * world) {
	char ROTATIONS[4] = {'^', '>', 'v', '<'};

	printf("World Map:\n");
	for (int i = 0; i < world->height; i++) {
		for (int j = 0; j < world->width; j++) {
			if (world->x == j && world->y == i) putchar(ROTATIONS[world->r]);
			else putchar(world->map[i][j]);
			putchar(' ');
		}
		putchar('\n');
	}
}

/*
* world is seen as 3 blocks directly in front and 
* 5 blocks after that - if not blocked by walls
*/
int8_t * worldView(World * world) {
	/* determine 8 facing blocks */
	int xArr[8];
	int yArr[8];
	int8_t * view = (int8_t *) malloc(8 * sizeof(int8_t));
	int frontFlags[3] = { 0, 0, 0 };

	if (world->r == 0 || world->r == 2) {
		if (world->r == 0) {
			for (int i = 0; i < 3; i++) yArr[i] = world->y - 1;
			for (int i = 3; i < 8; i++) yArr[i] = world->y - 2;
		} else if (world->r == 2) {
			printf("x: %d\n", world->x);
			printf("y: %d\n", world->y);
			printf("r: %d\n", world->r);
			
			for (int i = 0; i < 3; i++) {
				printf(", %d\n", yArr[i]);
				yArr[i] = world->y + 1;
			}
			for (int i = 3; i < 8; i++) {
				printf(". %d\n", i);
				yArr[i] = world->y + 2;
			}
		}
		printf("i %d ", yArr[0]);
		xArr[0] = world->x - 1;
		xArr[1] = world->x;
		xArr[2] = world->x + 1;
		xArr[3] = world->x - 2;
		xArr[4] = world->x - 1;
		xArr[5] = world->x;
		xArr[6] = world->x + 1;
		xArr[7] = world->x + 2;
	} else if (world->r == 1 || world->r == 3) {
		if (world->r == 1) {
			for (int i = 0; i < 3; i++) xArr[i] = world->x + 1;
			for (int i = 3; i < 8; i++) xArr[i] = world->x + 2;
		} else if (world->r == 3) {
			for (int i = 0; i < 3; i++) xArr[i] = world->x - 1;
			for (int i = 3; i < 8; i++) xArr[i] = world->x - 2;
		}
		yArr[0] = world->y - 1;
		yArr[1] = world->y;
		yArr[2] = world->y + 1;
		yArr[3] = world->y - 2;
		yArr[4] = world->y - 1;
		yArr[5] = world->y;
		yArr[6] = world->y + 1;
		yArr[7] = world->y + 2;
	}

	/* determine contents and legality of blocks */
	for (int i = 0; i < 8; i++) {
		printf(". %d", yArr[i]);
		view[i] = ' ';
		if (world->map[yArr[i]][xArr[i]] != ' ') {
			/* if in first row, prohibit light from passing through */
			if (i < 3) {
				frontFlags[i] = 1;
				view[i] = world->map[yArr[i]][xArr[i]];
			} else if ((i == 3 || i == 4) && !frontFlags[0]) {
				view[i] = world->map[yArr[i]][xArr[i]];
			} else if (i == 5 && !frontFlags[1]) {
				view[i] = world->map[yArr[i]][xArr[i]];
			} else if ((i == 6 || i == 7) && !frontFlags[2]) {
				view[i] = world->map[yArr[i]][xArr[i]];
			}
		}
	}

	return view;
}

/* turn = {-1: left, 0: none, 1: right} */
void worldMove(World * world, int forward, int turn) {
	if (turn == 0) {
		if (world->r == 0 && world->map[world->y-1][world->x] == ' ') world->y -= forward;
		else if (world->r == 1 && world->map[world->y][world->x+1] == ' ') world->x += forward;
		else if (world->r == 2 && world->map[world->y+1][world->x] == ' ') world->y += forward;
		else if (world->r == 3 && world->map[world->y][world->x-1] == ' ') world->x -= forward;
	} else {
		world->r += turn;
	}
}
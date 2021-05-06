#include <stdio.h>
#include <stdlib.h>

struct MapNode
{
	/* data */
	float x;
	float y;
	float r;
	int view[8];
	/* 0 1 2
	 3 4 5 6 7 */
};

class Bot
{
private:
	/* data */
	float x;
	float y;
	float r;
	
public:
	Bot();
	~Bot();
	void sense(World * world);
};

Bot::Bot()
{
	x = 0;
	y = 0;
	r = 0;
}

Bot::~Bot()
{
}

void Bot::sense(World * world) {
	MapNode node{ x, y, r };
	node.view = world->sense();
}

class World
{
private:
	/* data */

public:
	World(/* args */);
	~World();
	int * sense();
};

World::World(/* args */)
{
}

World::~World()
{
}

int * World::sense() {
	
}

int main(int argc, char ** argv)
{

}
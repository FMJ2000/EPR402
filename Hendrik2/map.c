#include "map.h"

// map functions
void Map_Init(struct Map ** map, float pos[2]) {
	*map = malloc(sizeof(struct Map));
	memcpy((*map)->pos, pos, sizeof(float) * 2);
	for (uint8_t i = 0; i < MAP_UNITS; i++)
		for (uint8_t j = 0; j < MAP_UNITS; j++)
			(*map)->grid[i][j] = MAP_PRIOR;
}

// reinforce neighbor connections between maps
void Map_Reinforce(struct Map * map) {
	for (uint8_t i = 0; i < 8; i++) {
		if (map->neighbors[i]) {
			uint8_t mIndex = (i + 4) % 8;	// my index relative to neighbor
			if (i % 2) {
				if (map->neighbors[i]->neighbors[(mIndex + 7) % 8]) {
					map->neighbors[(i + 2) % 8] = map->neighbors[i]->neighbors[(mIndex + 7) % 8];
					map->neighbors[i]->neighbors[(mIndex + 7) % 8]->neighbors[(i + 6) % 8] = map;
				}
				if (map->neighbors[i]->neighbors[(mIndex + 1) % 8]) {
					map->neighbors[(i + 6) % 8] = map->neighbors[i]->neighbors[(mIndex + 1) % 8];
					map->neighbors[i]->neighbors[(mIndex + 1) % 8]->neighbors[(i + 2) % 8] = map;
				}
				if (map->neighbors[i]->neighbors[(mIndex + 6) % 8]) {   
					map->neighbors[(i + 1) % 8] = map->neighbors[i]->neighbors[(mIndex + 6) % 8];
					map->neighbors[i]->neighbors[(mIndex + 6) % 8]->neighbors[(i + 5) % 8] = map;
				}
				if (map->neighbors[i]->neighbors[(mIndex + 2) % 8]) {
					map->neighbors[(i + 7) % 8] = map->neighbors[i]->neighbors[(mIndex + 2) % 8];
					map->neighbors[i]->neighbors[(mIndex + 2) % 8]->neighbors[(i + 3) % 8] = map;
				}
				} else {
				if (map->neighbors[i]->neighbors[(mIndex + 7) % 8]) {
					map->neighbors[(i + 1) % 8] = map->neighbors[i]->neighbors[(mIndex + 7) % 8];
					map->neighbors[i]->neighbors[(mIndex + 7) % 8]->neighbors[(i + 5) % 8] = map;
				}
				if (map->neighbors[i]->neighbors[(mIndex + 1) % 8]) {
					map->neighbors[(i + 7) % 8] = map->neighbors[i]->neighbors[(mIndex + 1) % 8];
					map->neighbors[i]->neighbors[(mIndex + 1) % 8]->neighbors[(i + 3) % 8] = map;
				}
			}
		}
	}
}

// check if position is located in map
char Map_Contains(struct Map * map, float pos[2]) {
	float xRange[2] = { map->pos[0], map->pos[0] + MAP_SIZE };
	float yRange[2] = { map->pos[1], map->pos[1] + MAP_SIZE };
	return ((pos[0] >= xRange[0]) && (pos[0] < xRange[1]) && (pos[1] >= yRange[0]) && (pos[1] < yRange[1]));
}
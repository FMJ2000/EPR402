#include "map.h"

// map functions - log odds probability map
void Map_Init(struct Map ** map, float pos[2]) {
	*map = malloc(sizeof(struct Map));
	memcpy((*map)->pos, pos, sizeof(float) * 2);
	for (uint8_t i = 0; i < MAP_UNITS; i++)
		for (uint8_t j = 0; j < MAP_UNITS; j++)
			(*map)->grid[i][j] = LOG_PRIOR;
}

// reinforce neighbor connections between maps
void Map_Reinforce(struct Map * map) {
	if (!map) return;
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

// update log odds probability given distance measurement
void Map_Update(struct Map * map, float pos[3], float dist[US_SENSORS]) {
	if (!map) return;
	// calculate robot sensor angles
	float sensorAngles[US_SENSORS] = { normAngle(pos[2] + SENSOR_O), pos[2], normAngle(pos[2] - SENSOR_O) };

	// iterate through map and update if visible
	for (uint8_t i = 0; i < MAP_UNITS; i++) {
		for (uint8_t j = 0; j < MAP_UNITS; j++) {
			float cellPos[2] = { map->pos[0] + i*MAP_RES - pos[0], map->pos[1] - j*MAP_RES };
			float cellAngle = atan2(cellPos[1], cellPos[0]);
			for (uint8_t k = 0; k < US_SENSORS; k++) {
				if (fabs(sensorAngles[k] - cellAngle) <= SENSOR_A) {
					Map_Cell_Update(&map->grid[i][j], getDistance(cellPos, pos), dist[k]);
					break;
				}
			}
		}
	}
}

// update cell occupancy based on sensor data
void Map_Cell_Update(float * cell, float dist, float z) {
	float newL = L_MED;
	if (dist < z - D_1) newL = L_LOW;
	else if (dist < z + D_1) newL = L_LOW + (dist - z - D_1) * (L_HIGH - L_LOW) / (2*D_1);
	else if (dist < z + D_2) newL = L_HIGH;
	else if (dist < z + D_3) newL = L_HIGH - (dist - z + D_3) * (L_HIGH - L_MED) / (D_3 - D_2);
	*cell += newL - LOG_PRIOR;
}

// check if position is located in map
char Map_Contains(struct Map * map, float pos[2]) {
	if (!map) return 0;
	float xRange[2] = { map->pos[0], map->pos[0] + MAP_SIZE };
	float yRange[2] = { map->pos[1] - MAP_SIZE, map->pos[1] };
	return ((pos[0] >= xRange[0]) && (pos[0] < xRange[1]) && (pos[1] >= yRange[0]) && (pos[1] < yRange[1]));
}

// check if the location or surrounding is obstructed
char Map_Pos_Collide(struct Map * map, float pos[2]) {
	if (!map || !Map_Contains(map, pos)) return -1;
	uint8_t index[2] = {(uint8_t)((pos[0] - map->pos[0]) / MAP_RES), (uint8_t)((map->pos[1] - pos[1]) / MAP_RES) };
	if (map->grid[index[0]][index[1]] > L_HIGH) return 1;
	for (uint8_t i = 0; i < 8; i++) {
		uint8_t newIndex[2] = { index[0] + posMod[i][0], index[1] + posMod[i][1] };
		if (newIndex[0] >= 0 && newIndex[0] < MAP_UNITS && newIndex[1] >= 0 && newIndex[1] < MAP_UNITS) {
			if (map->grid[newIndex[0]][newIndex[1]] > L_HIGH) return 1;
		}
	}
	return 0;
}

// create edge map
void Map_Edge(struct Map * map, float edgeGrid[MAP_UNITS][MAP_UNITS]) {
	if (!map) return;
	for (uint8_t i = 0; i < MAP_UNITS; i++) {
		for (uint8_t j = 0; j < MAP_UNITS; j++) {
			float lmin = LH;
			for (uint8_t k = 0; k < 8; k++) {
				uint8_t index[2] = { i+posMod[k][0], j+posMod[k][1] };
				if (index[0] >= 0 && index[0] < MAP_UNITS && index[1] >= 0 && index[1] < MAP_UNITS) {
					float lm = map->grid[index[0]][index[1]];
					if (lm < lmin) lmin = lm;
				}
			}
			edgeGrid[i][j] = LL*fabs(map->grid[i][j]) - lmin;
			if (edgeGrid[i][j] >= EDGE_BAR) edgeGrid[i][j] = EDGE_H;
			else edgeGrid[i][j] = EDGE_L;
		}
	}
}

// 2D blur NxN parts of the map
void Map_Blur(float blurGrid[MAP_UNITS][MAP_UNITS], float edgeGrid[MAP_UNITS][MAP_UNITS]) {
	for (uint8_t i = 0; i < MAP_UNITS; i++) {
		for (uint8_t j = 0; j < MAP_UNITS; j++) {
			// blur at position
			blurGrid[i][j] = 0;
			for (int16_t k = -2; k < 3; k++) {
				if (i + k >= 0 && i + k < MAP_UNITS) {
					for (int16_t l = -2; l < 3; l++) {
						if (j + l >= 0 && j + l< MAP_UNITS) {
							blurGrid[i][j] += edgeGrid[i+k][j+l];
						}
					}
				}
			}
		}
	}
}

// use edge map to obtain frontiers, if any
char Map_Frontier(struct Map * map, float pos[3], uint8_t frontier[2]) {
	if (!map) return 0;
	float edgeGrid[MAP_UNITS][MAP_UNITS], blurGrid[MAP_UNITS][MAP_UNITS];
	Map_Edge(map, edgeGrid);
	Map_Blur(blurGrid, edgeGrid);

	// identify frontiers - choose at borders, in direction of bot
	float frontiers[MAP_UNITS*MAP_UNITS][2];
	int fIndex = 0;
	for (uint8_t i = 0; i < MAP_UNITS; i++)
		for (uint8_t j = 0; j < MAP_UNITS; j++)
			if (blurGrid[i][j] >= FRONTIER_L && fIndex < MAP_UNITS*MAP_UNITS) {
				frontiers[fIndex][0] = i;
				frontiers[fIndex][1] = j;
				fIndex++;
			}
	
	if (fIndex == 0) return 0;
	float minAngle = M_PI;
	for (int i = 0; i < fIndex; i++) {
		float fpos[2] = { map->pos[0] + frontiers[i][0]*MAP_RES, map->pos[1] - frontiers[i][1]*MAP_RES };
		float angle = atan2(fpos[1] - pos[1], fpos[0] - pos[0]) - pos[2];
		float distance = getDistance(fpos, pos);
		if (fabs(angle) < fabs(minAngle)) {
			minAngle = angle;
			frontier[0] = frontiers[i][0];
			frontier[1] = frontiers[i][1];
		}
	}
	return 1;
}
#include "map.h"

// map functions - log odds probability map
void Map_Init(struct Map ** map, float pos[2]) {
	*map = malloc(sizeof(struct Map));
	memcpy((*map)->pos, pos, sizeof(float) * 2);
	for (uint8_t i = 0; i < MAP_UNITS; i++)
		for (uint8_t j = 0; j < MAP_UNITS; j++)
			(*map)->grid[i][j] = LOG_PRIOR;
}

// auxiliary index and state space mapping functions
char Map_PosToIndex(struct Map * map, uint8_t index[2], float pos[2]) {
	if (!map || !Map_Contains(map, pos)) return 0;
	index[0] = (uint8_t)((pos[0] - map->pos[0]) / MAP_RES);
	index[1] = (uint8_t)((map->pos[1] - pos[1]) / MAP_RES);
	return 1;
}

char Map_IndexToPos(struct Map * map, float pos[2], uint8_t index[2]) {
	if (!map || index[0] < 0 || index[0] >= MAP_UNITS || index[1] < 0 || index[1] >= MAP_UNITS) return 0;
	pos[0] = map->pos[0] + index[0]*MAP_RES;
	pos[1] = map->pos[1] - index[1]*MAP_RES;
	return 1;
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
	float sensorAngles[US_SENSORS] = { SENSOR_O, 0, -SENSOR_O };//{ normAngle(pos[2] + SENSOR_O), pos[2], normAngle(pos[2] - SENSOR_O) };

	// iterate through map and update if visible
	for (uint8_t i = 0; i < MAP_UNITS; i++) {
		for (uint8_t j = 0; j < MAP_UNITS; j++) {
			float cellPos[2] = { map->pos[0] + i*MAP_RES, map->pos[1] - j*MAP_RES };
			float cellAngle = getAngle(pos, cellPos);
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
	if (dist > MAX_US_DIST) return;
	float newL = L_MED;
	if (dist < z - D_1) newL = L_LOW;
	else if (dist < z + D_1) newL = L_LOW + (dist - (z - D_1)) * (L_HIGH - L_LOW) / (2*D_1);
	else if (dist < z + D_2) newL = L_HIGH;
	else if (dist < z + D_3) newL = L_HIGH - ((z + D_3) - dist) * (L_HIGH - L_MED) / (D_3 - D_2);
	*cell += newL - LOG_PRIOR;
}

void Map_Update_Visit(struct Map * map, float pos[2]) {
	uint8_t botIndex[2];
	if (!map || !Map_PosToIndex(map, botIndex, pos)) return;
	map->visited[botIndex[0]][botIndex[1] / 8] |= 0x1 << (botIndex[1] % 8);
	for (uint8_t i = 0; i < 8; i++) {
		uint8_t index[2] = { botIndex[0] + bPosMod[i][0], botIndex[1] + bPosMod[i][1] };
		if (index[0] >= 0 && index[0] < MAP_UNITS && index[1] >= 0 && index[1] < MAP_UNITS) map->visited[index[0]][index[1] / 8] |= 0x1 << (index[1] % 8);
	}
}

char Map_Check_Visited(struct Map * map, float pos[2]) {
	uint8_t index[2];
	if (!map || !Map_PosToIndex(map, index, pos)) return 0;
	return (map->visited[index[0]][index[1] / 8] >> (index[1] % 8)) & 0x1;
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
	uint8_t index[2];
	if (!map || !Map_PosToIndex(map, index, pos)) return -1;
	if (map->grid[index[0]][index[1]] > L_HIGH) return 1;
	for (uint8_t i = 0; i < 4; i++) {
		uint8_t newIndex[2] = { index[0] + posMod[2*i+1][0], index[1] + posMod[2*i+1][1] };
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

void Map_SafeZone(struct Map * map, char safeGrid[3*MAP_UNITS][3*MAP_UNITS]) {
	if (!map) return;
	for (uint8_t i = 0; i < 9; i++) {
		struct Map * submap = (i) ? map->neighbors[i-1] : map;
		if (submap && submap->grid) {
			for (uint8_t j = 0; j < MAP_UNITS; j++) {
				for (uint8_t k = 0; k < MAP_UNITS; k++) {				
					// check for frontier if uncertain with known unobstructed neighbour
					char visited = (submap->visited[j][k / 8] >> (k % 8)) & 0x1;
					char frontier = 0;
					if (submap->grid[j][k] > -0.3 && submap->grid[j][k] < 0.3) {
						float lmin = LH;
						for (uint8_t l = 0; l < 8; l++) {	
							uint8_t nIndex[2] = { j+posMod[l][0], k+posMod[l][1] };
							if (nIndex[0] >= 0 && nIndex[0] < MAP_UNITS && nIndex[1] >= 0 && nIndex[1] < MAP_UNITS) {
								float lm = submap->grid[nIndex[0]][nIndex[1]];
								if (lm < lmin) lmin = lm;
							}
						}
						if (lmin < EDGE_L) frontier = 1;
					}
					safeGrid[MAP_UNITS + safePosMod[i][0]*MAP_UNITS + j][MAP_UNITS + safePosMod[i][1]*MAP_UNITS + k] = (frontier || (submap->grid[j][k] < EDGE_L && !visited));
				}
			}
		}
	}
}

// use edge map to obtain global safe region, if any
/*char Map_Frontier(struct Map * map, float pos[3], float oldPos[2], uint8_t frontier[2]) {
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
		float distance = getDistance(fpos, oldPos);
		if (fabs(angle) < fabs(minAngle) && distance > FRONTIER_DIST) {
			minAngle = angle;
			frontier[0] = frontiers[i][0];
			frontier[1] = frontiers[i][1];
		}
	}
	return (minAngle != M_PI);
}*/

char Map_Frontier(struct Map * map, float botPos[3], float exPos[2])  {	
	// get unoccupied and unvisited blocks in supermap
	char safeGrid[3*MAP_UNITS][3*MAP_UNITS] = {{0}};
	Map_SafeZone(map, safeGrid);
	float startPos[2] = { map->pos[0] - MAP_SIZE, map->pos[1] + MAP_SIZE };
	uint8_t oldIndex[2] = { (uint8_t)((exPos[0] - startPos[0]) / MAP_RES), (uint8_t)((startPos[1] - exPos[1]) / MAP_RES) };
	
	// determine most viable point first in current map then in global map
	uint8_t bestIndex[2] = {0};
	float minVal = 1000;
	for (uint8_t i = 0; i < 2; i++) {
		for (uint8_t j = 0; j < FRONTIER_TRIES; j++) {
			uint8_t randIndex[2] = { MAP_UNITS + rand() % MAP_UNITS, MAP_UNITS + rand() % MAP_UNITS };
			if (i) {
				randIndex[0] = rand() % (3*MAP_UNITS);
				randIndex[1] = rand() % (3*MAP_UNITS);
			}
			if (randIndex[0] != oldIndex[0] && randIndex[1] != oldIndex[1]) {
				if (safeGrid[randIndex[0]][randIndex[1]]) {
					float randPos[2] = { startPos[0] + randIndex[0] * MAP_RES, startPos[1] - randIndex[1] * MAP_RES };
					float val = powf(getAngle(botPos, randPos), 2) / powf(getDistance(botPos, randPos), 2);
					if (val < minVal) {
						minVal = val;
						memcpy(bestIndex, randIndex, sizeof(float) * 2);
						if (minVal < FRONTIER_MIN) break;
					}
				}
			}
		}
		if (minVal < FRONTIER_MIN) break;
	}
	if (minVal == 1000) return 0;
	
	exPos[0] = startPos[0] + bestIndex[0] * MAP_RES;
	exPos[1] = startPos[1] - bestIndex[1] * MAP_RES;
	return 1;
}
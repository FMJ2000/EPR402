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
	// calculate robot sensor angles
	float sensorAngles[US_SENSORS] = { normAngle(pos[2] + SENSOR_O), pos[2], normAngle(pos[2] - SENSOR_O) };

	// iterate through map and update if visible
	for (uint8_t i = 0; i < MAP_UNITS; i++) {
		for (uint8_t j = 0; j < MAP_UNITS; j++) {
			float cellPos[2] = { map->pos[0] + i*MAP_RES - pos[0], map->pos[1] - j*MAP_RES }
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
	*cell += log(newL) - LOG_PRIOR;
}

// check if position is located in map
char Map_Contains(struct Map * map, float pos[2]) {
	float xRange[2] = { map->pos[0], map->pos[0] + MAP_SIZE };
	float yRange[2] = { map->pos[1], map->pos[1] + MAP_SIZE };
	return ((pos[0] >= xRange[0]) && (pos[0] < xRange[1]) && (pos[1] >= yRange[0]) && (pos[1] < yRange[1]));
}

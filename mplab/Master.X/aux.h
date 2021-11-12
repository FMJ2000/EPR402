#ifndef _AUX_H
#define _AUX_H

#include "master.h"
#include "bot.h"

/* helper functions */
float getAngle(float pos1[2], float pos2[2]);
float normAngle(float x);
float getDistance(float pos1[2], float pos2[2]);
char distanceToPos(float result[][2], float botPos[3], float sensorOffsets[3], float * distances);
char Multivariate_Gaussian(float meanX, float meanY, float posX, float posY);
float Box_Muller(float mu, float sigma);
void delay(long us);
float ideal_lowpass(float fcNew, uint8_t i);
float hamming(uint8_t i);
void matrix_mul( int nRows,  int nCols, int nAdd, float result[][nCols], float mat1[][nAdd], float mat2[][nCols]);
float random(float a, float b);

/*
void matrix_plus(int len, float result[][len], float mat1[][len], float mat2[][len]);

void matrix_inv(float result[3][3], float mat[3][3]);
*/

#endif
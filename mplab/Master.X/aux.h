#ifndef _AUX_H
#define _AUX_H

#include "master.h"
#include "bot.h"

/* helper functions */
float getAngle(float x1, float y1, float x2, float y2);
float normAngle(float x);
float getDistance(float x1, float y1, float x2, float y2);
char distanceToPos(float result[][2], float botPos[3], float * valid, float * distances);
char Multivariate_Gaussian(float meanX, float meanY, float posX, float posY);
float Box_Muller(float mu, float sigma);
void delay(long us);
void matrix_mul( int nRows,  int nCols, int nAdd, float result[][nCols], float mat1[][nAdd], float mat2[][nCols]);
/*
void matrix_plus(int len, float result[][len], float mat1[][len], float mat2[][len]);

void matrix_inv(float result[3][3], float mat[3][3]);
*/

#endif
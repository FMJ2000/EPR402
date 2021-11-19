#include "aux.h"

/* Helper functions */
float getAngle(float pos1[3], float pos2[2]) {
    /*float angle = acos((pos1[0]*pos2[0] + pos1[1]*pos2[1]) / (sqrt(pos1[0]*pos1[0] + pos1[1]*pos1[1]) * sqrt(pos2[0]*pos2[0] + pos2[1]*pos2[1])));
    if (isnanf(angle)) return 0.0;
    return angle;	   */
	float denom = (pos2[0] == pos1[0]) ? 0.001 : pos2[0] - pos1[0];
	return normAngle(atan2(pos2[1] - pos1[1], denom) - pos1[2]);
}

float normAngle(float x) {
	x = fmod(x + M_PI, 2*M_PI);
	if (x <= 0) x += 2*M_PI;
	return x - M_PI;
}

float getDistance(float pos1[2], float pos2[2]) {
	return sqrtf(powf(pos1[0] - pos2[0], 2) + powf(pos1[1] - pos2[1], 2));
}

void distanceToPos(float result[][2], float botPos[3], float sensorOffsets[3], float * distances) {
	for (char i = 0; i < US_SENSORS; i++) {
		result[i][0] = botPos[0] + distances[i] * cos(sensorOffsets[i] - botPos[2]);
		result[i][1] = botPos[1] + distances[i] * sin(sensorOffsets[i] - botPos[2]);
	}
}

void delay(long long us) {
	long count = us * SYSCLK / 16000000;
	while (count--);
}

/* matrix manipulations */
void Mat_T(uint8_t rows, uint8_t cols, float result[cols][rows], float mat[rows][cols]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < cols; j++)
			result[j][i] = mat[i][j];
}

void Mat_Add(uint8_t rows, uint8_t cols, float result[rows][cols], float mat1[rows][cols], float mat2[rows][cols]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < cols; j++)
			result[i][j] = mat1[i][j] + mat2[i][j];
}

void Mat_Sub(uint8_t rows, uint8_t cols, float result[rows][cols], float mat1[rows][cols], float mat2[rows][cols]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < cols; j++)
			result[i][j] = mat1[i][j] - mat2[i][j];
}

void Mat_Mul(uint8_t rows, uint8_t cols, uint8_t add, float result[rows][cols], float mat1[rows][add], float mat2[add][cols]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < cols; j++) {
			result[i][j] = 0.0;
			for (uint8_t k = 0; k < add; k++)
				result[i][j] += mat1[i][k] * mat2[k][j];
		}
}

void Mat_Inv(uint8_t rows, float result[rows][rows], float mat[rows][rows]) {
	// calculate matrix of minors and determinant
	float minors[rows][rows];
	for (uint8_t i = 0; i < rows; i++) {
		for (uint8_t j = 0; j < rows; j++) {
			float submat[rows-1][rows-1];
			uint8_t offsetK = 0;
			for (uint8_t k = 0; k < rows-1; k++) {
				uint8_t offsetL = 0;
				for (uint8_t l = 0; l < rows-1; l++) {
					if (k == i) offsetK = 1;
					if (l == j) offsetL = 1;
					submat[k][l] = mat[k+offsetK][l+offsetL];
				}
			}
			minors[i][j] = Mat_Det(rows-1, submat);
		}
	}
	float det = 0;
	for (uint8_t i = 0; i < rows; i++)
		det = (i % 2) ? det - mat[0][i] * minors[0][i] : det + mat[0][i] * minors[0][i];

	// calculate cofactors and adjoint
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < rows; j++)
			if ((i + j) % 2) minors[i][j] *= -1;

	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < rows; j++)
			result[i][j] = minors[j][i] / det;
}

float Mat_Det(uint8_t rows, float mat[rows][rows]) {
	if (rows == 2) return mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0];
	float det = 0;
	for (uint8_t i = 0; i < rows; i++) {
		// determine submatrix and its determinant
		float submat[rows-1][rows-1];
		for (uint8_t j = 0; j < rows-1; j++) {
			uint8_t offsetK = 0;
			for (uint8_t k = 0; k < rows-1; k++) {
				if (k == i) offsetK = 1;
				submat[j][k] = mat[j+1][k+offsetK];
			}
		}
		float submat_det = Mat_Det(rows-1, submat);
		det = (i % 2) ? det - mat[0][i]*submat_det : det + mat[0][i]*submat_det;
	}
	return det;
}

void norm(uint8_t len, float x[len]) {
	float sum = 0;
	for (uint8_t i = 0; i < len; i++) sum += x[i]*x[i];
	sum = sqrt(sum);
	for (uint8_t i = 0; i < len; i++) x[i] /= sum;
}

void cholesky(uint8_t rows, float L[rows][rows], float A[rows][rows]) {
	for (uint8_t i = 0; i < rows; i++)
		for (uint8_t j = 0; j < rows; j++)
			L[i][j] = 0;

	for (uint8_t i = 0; i < rows; i++) {
		for (uint8_t j = 0; j < i+1; j++) {
			float sum = 0;
			for (uint8_t k = 0; k < j; k++) sum += L[i][k] * L[j][k];
			L[i][j] = (i == j) ? sqrt(A[i][j] - sum) : 1 / L[j][j] * (A[i][j] - sum);
		}
	}
}

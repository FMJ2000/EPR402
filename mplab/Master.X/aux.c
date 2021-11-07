#include "aux.h"

/* Helper functions */
float getAngle(float x1, float y1, float x2, float y2) {
    float angle = acos((x1*x2 + y1*y2) / (sqrt(x1*x1 + y1*y1) * sqrt(x2*x2 + y2*y2)));
    if (isnanf(angle)) return 0.0;
    return angle;	    
}

float normAngle(float x) {
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0) x += 2*M_PI;
    return x - M_PI;
}

float getDistance(float pos1[2], float pos2[2]) {
    return sqrtf(powf(pos1[0] - pos2[0], 2) + powf(pos1[1] - pos2[1], 2));
}

char distanceToPos(float result[][2], float botPos[3], float sensorOffsets[3], float * distances) {
    char index = 0;
    for (char i = 0; i < US_SENSORS; i++) {
	if (distances[i] >= MIN_US_DIST && distances[i] <= MAX_US_DIST) {
	    result[i][0] = botPos[0] + distances[i] * cos(sensorOffsets[i] - botPos[2]);
	    result[i][1] = botPos[1] + distances[i] * sin(sensorOffsets[i] - botPos[2]);
	    index++;
	}
    }
    return index;
}

char Multivariate_Gaussian(float meanX, float meanY, float posX, float posY) {
    float devX = (posX - meanX) / US_SIGMA;
    float devY = (posY - meanY) / US_SIGMA;
    return 1 / (2*M_PI * US_SIGMA * US_SIGMA) * exp(-0.5 * (powf(devX, 2) + powf(devY, 2)));
}

float Box_Muller(float mu, float sigma) {
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    float z0 = sqrt(-2*log(u1)) * cos(2*M_PI*u2);
    return z0*sigma + mu;
}

void delay(long us) {
    long count = us * SYSCLK / 16000000;
    while (count--);
}

float ideal_lowpass(float fcNew, uint8_t n) {
    if (n == 0) return 2*fcNew;
    return 2*fcNew*sin(2*M_PI*fcNew*n) / (2*M_PI*fcNew*n);
}

float hamming(uint8_t n) {
    return 0.53 + 0.46*cos(2*M_PI*n / FIR_N);
}

/*
void matrix_plus(int len, float result[][len], float mat1[][len], float ma6t2[][len]) {
    for (int i = 0; i < len; i++) {
	for (int j = 0; j < len; j++) {
	    result[i][j] = mat1[i][j] + mat2[i][j];
	}
    }
} */

void matrix_mul( int nRows,  int nCols, int nAdd, float result[][nCols], float mat1[][nAdd], float mat2[][nCols]) {
    for (int i = 0; i < nRows; i++) {
	for (int j = 0; j < nCols; j++) {
	    result[i][j] = 0.0;
	    for (int k = 0; k < nAdd; k++) 
		result[i][j] += mat1[i][k] * mat2[k][j];
	}
    }
}

/* inverse of 3x3 matrix /
void matrix_inv(float result[3][3], float mat[3][3]) {
    // calculate matrix of minors
    float minors[3][3] = {{0}};
    float detMat[2][2] = {{0}};
    int offsetK = 0;
    int offsetL = 0;
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    // get determinant
	    offsetK = 0;
	    for (int k = 0; k < 2; k++) {
		offsetL = 0;
		for (int l = 0; l < 2; l++) {
		    if (k == i) offsetK = 1;
		    if (l == j) offsetL = 1;
		    detMat[k][l] = mat[k + offsetK][l + offsetL];
		}
	    }
	    minors[i][j] = detMat[0][0] * detMat[1][1] - detMat[0][1] * detMat[1][0];
	}
    }
    
    float det = mat[0][0] * minors[0][0] + mat[0][1] * minors[0][1] + mat[0][2] * minors[0][2];
    
    // cofactors and adjoint
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    if ((i + j) % 2 != 0) minors[i][j] *= -1;
	}
    }
    
    for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++) {
	    result[i][j] = minors[j][i] / det;
	}
    }
}

 */
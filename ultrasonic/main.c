#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define d 0.05
#define v 1500.0
#define nReturns 3
#define nSensors 2
#define timesteps 2
#define maxVal 10000.0

float s1Time[nReturns * timesteps] = { 0.0533, 0.0893, 0.0947, 0.0307, 0.0693, 0.088 };
float s2Time[nReturns * timesteps] = { 0.0693, 0.0767, 0.0913, 0.0573, 0.0613, 0.0707 };
float xOffset[timesteps] = { 0, 0 };
float yOffset[timesteps] = { 0, 0.017};

void calcPos(float ** x, float ** y, const float * t1, const float * t2, int size, float xOff, float yOff);
float evalRow(float * x1, float * y1, float * x2, float * y2, int size);
void evalPos(float * xReal, float * yReal, float ** xPos, float ** yPos, int size);
void printPos(float ** x, float ** y, int rows, int cols	);
void freePos(float **x, float **y, int size);

void calcPos(float ** x, float ** y, const float * t1, const float * t2, int size, float xOff, float yOff) {
	float r1[size];
	float r2[size];
	
	for (int i = 0; i < size; i++) {
		r1[i] = v * t1[i];
		r2[i] = v * t2[i];
	}
	
	/* elliptic localization */	
	for (int i = 0; i < size; i++) {
		x[i] = malloc(size);
		y[i] = malloc(size);
		for (int j = 0; j < size; j++) {
			int k = (i + j) % size;
			x[i][j] = (r1[j] * r2[k] - pow(r2[k], 2)) / (2 * d) + xOff;
			y[i][j] = (sqrtf(pow(r2[k], 2) - pow(d, 2)) * sqrtf(pow(d, 2) - pow(r1[j] - r2[k], 2))) / (2 * d) + yOff;
		}
	}
}

float evalRow(float * x1, float * y1, float * x2, float * y2, int size) {
	float min = maxVal;
	for (int i = 0; i < size; i++) {
		float sum = 0.0;
		for (int j = 0; j < size; j++) {
			sum += fabsf(x1[j] - x2[(i + j) % size]) + fabsf(y1[j] - y2[(i + j) % size]);
			printf("%d, %d: %f\n", i, j, sum);
		}
		if (sum < min) min = sum;
	}
	return min;
}

void evalPos(float * xReal, float * yReal, float ** xPos, float ** yPos, int size) {
	int num1 = size, num2 = size;
	
	/* check for nan */
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			int flag = 0;
			if (isnan(xPos[i][j]) || isnan(yPos[i][j])) {
				num1--;
				flag = 1;
			}
			if (isnan(xPos[size + i][j]) || isnan(yPos[size + i][j])) {
				num2--;
				flag = 1;
			}
			if (flag) break;
		}
	}
	
	float ** x1 = malloc(sizeof(float *) * num1);
	float ** x2 = malloc(sizeof(float *) * num2);
	float ** y1 = malloc(sizeof(float *) * num1);
	float ** y2 = malloc(sizeof(float *) * num2);
	
	int index1 = 0, index2 = 0;
	for (int i = 0; i < size; i++) {
		int flag1 = 0, flag2 = 0;
		for (int j = 0; j < size; j++) {
			if (isnan(xPos[i][j]) || isnan(yPos[i][j])) flag1 = 1;
			if (isnan(xPos[size + i][j]) || isnan(yPos[size + i][j])) flag2 = 0;
		}
		if (!flag1) {
			x1[index1] = malloc(sizeof(float) * size);
			y1[index1] = malloc(sizeof(float) * size);
			memcpy(x1[index1], xPos[i], sizeof(float) * size);
			memcpy(y1[index1++], yPos[i], sizeof(float) * size);
		}
		if (!flag2) {
			x2[index2] = malloc(sizeof(float) * size);
			y2[index2] = malloc(sizeof(float) * size);
			memcpy(x2[index2], xPos[i + size], sizeof(float) * size);			
			memcpy(y2[index2++], yPos[i + size], sizeof(float) * size);
		}
	}
	
	printPos(x1, y1, num1, size);
	printPos(x2, y2, num2, size);

	/* evaluate each row */
	float min = maxVal;
	int index = 0;
	for (int i = 0; i < num1; i++) {
		for (int j = 0; j < num2; j++) {
			float val = evalRow(x1[i], y1[i], x2[j], y2[j], size);
			printf("num1: %d, num2: %d, val: %f\n", i, j, val);
			if (val < min) {
				min = val;
				index = i;
			}
		}
	}

	printf("min: %f, index: %d\n", min, index);
	
	
//	float val = evalRow(x1[1], y1[1], x2[1], y2[1], size);
		
	freePos(x1, y1, num1);
	freePos(x2, y2, num2);
	free(x1);
	free(x2);
	free(y1);
	free(y2);
}

void printPos(float ** x, float ** y, int rows, int cols) {
	for (int i = 0; i < rows; i++) {
		printf("%d. ", i+1);
		for (int j = 0; j < cols; j++) {
			printf("(%.4f, %.4f), ", x[i][j], y[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

void freePos(float ** x, float ** y, int size) {
	for (int i = 0; i < size; i++) {
		if (x[i]) free(x[i]);
		if (y[i]) free(y[i]);
	}
}

int main(int argc, char * argv[]) {
	for (int i = 0; i < nReturns * timesteps; i++) {
		s1Time[i] /= 1000;
		s2Time[i] /= 1000;
	}
	
	float * xPos[nReturns * timesteps];
	float * yPos[nReturns * timesteps];
	float xReal[nReturns];
	float yReal[nReturns];
	
	printf("t = 0");
	calcPos(xPos, yPos, s1Time, s2Time, nReturns, xOffset[0], yOffset[0]);
	printPos(xPos, yPos, nReturns, nReturns);
		
	for (int i = 1; i < timesteps; i++) {
		printf("t = %d\n", i);
		int n = nReturns * i;
		calcPos(&xPos[n], &yPos[n], &s1Time[n], &s2Time[n], nReturns, xOffset[i], yOffset[i]);
		printPos(&xPos[n], &yPos[n], nReturns, nReturns);
		evalPos(xReal, yReal, &xPos[nReturns * (i - 1)], &yPos[nReturns * (i - 1)], nReturns);
	}
	freePos(xPos, yPos, nReturns * timesteps);
	return 0;
}

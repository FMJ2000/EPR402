#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define d 0.05
#define v 1500.0
#define timesteps 2
#define maxVal 10000.0
#define maxDistance 0.007
#define maxLives 3

typedef struct tInput {
	int sensor;
	int timestep;
	int size;
	float * input;
} tInput;

typedef struct ledgeItem {
	float x;
	float y;
	int ack;
	int life;
	struct ledgeItem * next;
} ledgeItem;

float time1[timesteps][3] = {{ 0.0000533, 0.0000893, 0.0000947 }, { 0.0000307, 0.0000693, 0.000088 }};
float time2[timesteps][3] = {{ 0.0000693, 0.0000767, 0.0000913 }, { 0.0000573, 0.0000613, 0.0000707 }};
float xOffset[timesteps] = { 0, 0 };
float yOffset[timesteps] = { 0, 0.017};

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

void printLedger(ledgeItem * root) {
	ledgeItem * itemPtr = root;	
	printf("Ledger:\n");
	while (itemPtr != NULL) {
		printf("%.2f\t| %.2f\t| %d\t| %d\n", itemPtr->x * 100, itemPtr->y * 100, itemPtr->ack, itemPtr->life);
		itemPtr = itemPtr->next;
	}
}

void freePos(float ** x, float ** y, int size) {
	for (int i = 0; i < size; i++) {
		if (x[i]) free(x[i]);
		if (y[i]) free(y[i]);
	}
	if (x) free(x);
	if (y) free(y);
}

void freeLedger(ledgeItem * root) {
	ledgeItem * prevPtr;
	while (root != NULL) {
		prevPtr = root;
		root = root->next;
		free(prevPtr);
		prevPtr = NULL;
	}
}

ledgeItem * createLedgeItem(float x, float y) {
	ledgeItem * newItem = malloc(sizeof(ledgeItem));	
	newItem->ack = 0;
	newItem->x = x;
	newItem->y = y;
	newItem->life = maxLives;
	newItem->next = NULL;
	return newItem;
}

void appendLedger(ledgeItem ** root, float * x, float * y, int size) {
	ledgeItem * rootPtr = NULL;
	ledgeItem * newPtr;
	for (int i = 0; i < size; i++) {
		int placed = 0;
		ledgeItem * prevPtr;
		ledgeItem * itemPtr = *root;
		while (itemPtr != NULL && !placed) {
			float distance = sqrtf(pow(itemPtr->x - x[i], 2) + pow(itemPtr->y - y[i], 2));
			if (distance < maxDistance) {
				printf("distance: %f, x: %f, y: %f\n", distance, x[i], y[i]);
				itemPtr->ack++;
				itemPtr->x = (itemPtr->ack * itemPtr->x + x[i]) / (itemPtr->ack + 1);
				itemPtr->y = (itemPtr->ack * itemPtr->y + y[i]) / (itemPtr->ack + 1);
				placed = 1;
			}
			prevPtr = itemPtr;
			itemPtr = itemPtr->next;
		}
		if (!placed) {
			if (rootPtr == NULL) {
				rootPtr = createLedgeItem(x[i], y[i]);
				newPtr = rootPtr;
			} else {
				newPtr->next = createLedgeItem(x[i], y[i]);
				newPtr = newPtr->next;
			}
		}
	}
	if (*root == NULL) *root = rootPtr;
	else {
		newPtr = *root;
		while (newPtr->next != NULL) newPtr = newPtr->next;
		newPtr->next = rootPtr;
	}
}

void updateLedger(ledgeItem ** root) {
	ledgeItem * itemPtr = *root;
	while (itemPtr != NULL) {
		itemPtr->life--;
		//if (itemPtr->life == 0)
	}
}

float * flattenArr(float ** input, int rows, int cols) {
	float * output = malloc(sizeof(float) * rows * cols);
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++)
			output[cols * i + j] = input[i][j];
	return output;
}

int calcPos(float *** x, float *** y, const float * t1, const float * t2, int size, float xOff, float yOff) {
	float r1[size];
	float r2[size];
	int len = size;
	*x = malloc(sizeof(float *) * size);
	*y = malloc(sizeof(float *) * size);
	
	for (int i = 0; i < size; i++) {
		r1[i] = v * t1[i];
		r2[i] = v * t2[i];
	}
	
	/* elliptic localization */	
	int i = 0;
	while (i < len) {
		int flag = 0;
		(*x)[i] = malloc(sizeof(float) * size);
		(*y)[i] = malloc(sizeof(float) * size);
		for (int j = 0; j < size; j++) {
			int k = (i + j) % size;
			float newX = (r1[j] * r2[k] - pow(r2[k], 2)) / (2 * d) + xOff;
			float newY = (sqrtf(pow(r2[k], 2) - pow(d, 2)) * sqrtf(pow(d, 2) - pow(r1[j] - r2[k], 2))) / (2 * d) + yOff;
			if (isnan(newX) || isnan(newY)) {
				memmove((*x)[i], (*x)[i + 1], sizeof(float) * (size - i - 1));
				memmove((*y)[i], (*y)[i + 1], sizeof(float) * (size - i - 1));
				len--;
				flag = 1;
				break;
			} else {
				(*x)[i][j] = newX;
				(*y)[i][j] = newY;
 			}
		}
		if (!flag) i++;
	}
	return len;
}	

int main(int argc, char * argv[]) {
	ledgeItem * ledgeRoot = NULL;
	
	for (int i = 0; i < timesteps; i++) {
		int n = 3;
		tInput t1 = { 0, i, n, time1[i] };
		tInput t2 = { 1, i, n, time2[i] };
		float ** x;
		float ** y;
		
		printf("t = %d\n", i);
		int len = calcPos(&x, &y, t1.input, t2.input, n, xOffset[i], yOffset[i]);
		appendLedger(&ledgeRoot, flattenArr(x, len, n), flattenArr(y, len, n), n * len);
		printLedger(ledgeRoot);
		printPos(x, y, len, n);
		freePos(x, y, len);
	}
	freeLedger(ledgeRoot);
	return 0;
}

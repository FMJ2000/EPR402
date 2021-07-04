#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

typedef struct probDistItem {
	int value;
	int frequency;
	double probability;
	double normalProbability;
	double normalFrequency;
} probDistItem;

typedef struct probDist {
	probDistItem * items;
	int count;
	double totalProbability;
	double totalNormalProbability;
	double totalFrequency;
	double totalNormalFrequency;
} probDist;

probDist * normalDistributionCreate();
void normalDistributionCalculate(int * data, int size, probDist * pd);
void normalDistributionPrint(probDist * pd);
void normalDistributionFree(probDist * pd);

#include "normaldistribution.h"

#ifndef M_PI
#define M_PI 2.71828
#endif

#ifndef M_E
#define M_E 2.71828
#endif

static int indexof(int value, probDist * pd);
static int compareProbDistItem(const void * a, const void * b);

probDist * normalDistributionCreate() {
	probDist * pd = malloc(sizeof(probDist));
	pd->count = 0;
	pd->items = NULL;
	pd->totalProbability = 0;
	pd->totalNormalProbability = 0;
	pd->totalNormalFrequency = 0;
	return pd;
}

void normalDistributionCalculate(int * data, int size, probDist * pd) {
	int index;
	double total = 0;
	double sumofsquares = 0;
	double mean;
	double variance;
	double stddev;
	pd->totalFrequency = size;
	
	/* calculate frequencies */
	for (int i = 0; i < size; i++) {
		index = indexof(data[i], pd);
		if (index >= 0) pd->items[index].frequency++;
		else {
			pd->count++;
			if (pd->items == NULL) pd->items = malloc(sizeof(probDistItem));
			else pd->items = realloc(pd->items, sizeof(probDistItem) * pd->count);
			pd->items[pd->count - 1].value = data[i];
			pd->items[pd->count - 1].frequency = 1;
		}
		total += data[i];
		sumofsquares += pow(data[i], 2);
	}
	
	/* sort, calculate mean, variance and std deviation */
	qsort(pd->items, pd->count, sizeof(probDistItem), compareProbDistItem);
	mean = total / size;
	variance = (sumofsquares - (pow(total, 2) / size)) / size;
	stddev = sqrt(variance);
	
	
	/* calculate probabilities of unique values */
	for (int c = 0; c < pd->count; c++) {
		pd->items[c].probability = (double)pd->items[c].frequency / (double)size;
		pd->items[c].normalProbability = 1.0 / (stddev * sqrt(2.0 * M_PI)) * pow(M_E, -1.0 * pow(pd->items[c].value - mean, 2.0) / (variance * 2.0));
		pd->items[c].normalFrequency = pd->items[c].normalProbability * size;
		pd->totalProbability += pd->items[c].probability;
		pd->totalNormalProbability += pd->items[c].normalProbability;
		pd->totalNormalFrequency += pd->items[c].normalFrequency;
	}
}

void normalDistributionPrint(probDist * pd) {
	printf("Value | Probability | Normal Prob | Freq | Normal Frequency\n---------------------------------------------------------\n");
	for (int i = 0; i < pd->count; i++)
		printf("%5d | %12.6lf | %12.4lf | %5d | %12.4lf\n", pd->items[i].value, pd->items[i].probability, pd->items[i].normalProbability, pd->items[i].frequency, pd->items[i].normalFrequency);
	printf("---------------------------------------------------------");
	printf("      | %12.4lf | %12.6lf | %5.0lf | %12.4lf\n", pd->totalProbability, pd->totalNormalProbability, pd->totalFrequency, pd->totalNormalFrequency);
	printf("---------------------------------------------------------\n");
}

void normalDistributionFree(probDist * pd) {
	free(pd->items);
	free(pd);
}

static int indexof(int value, probDist * pd) {
	for (int i = 0; i < pd->count; i++)
		if (pd->items[i].value == value) return i;
	return -1;
}

static int compareProbDistItem(const void * a, const void * b) {
	if (((probDistItem *)a)->value < ((probDistItem *)b)->value) return -1;
	else if (((probDistItem *)a)->value > ((probDistItem *)b)->value) return 1;
	else return 0;
}

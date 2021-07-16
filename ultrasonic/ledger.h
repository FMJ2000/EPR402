#ifndef LEDGER_H
#define LEDGER_H

#include <stdlib.h>

#define MAX_LIVES 3
#define MAX_DISTANCE 0.007

typedef struct ledgeItem {
	float x;
	float y;
	int ack;
	int life;
	int pos;
} ledgeItem;

ledgeItem * createLedgeItem(float x, float y, int pos);
void appendLedgeItem(ledgeItem * ledger[], int size, float x, float y);
void removeLedgeItem(ledgeItem * ledger[], int pos);

#endif

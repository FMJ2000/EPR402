#include "ledger.h"

ledgeItem * createLedgeItem(float x, float y, int pos) {
	ledgeItem * newItem = malloc(sizeof(ledgeItem));	
	newItem->ack = 0;
	newItem->x = x;
	newItem->y = y;
	newItem->life = MAX_LIVES;
	newItem->pos = pos;
	return newItem;
}

void appendLedgeItem(ledgeItem * ledger[], int size, float x, float y) {
	int pos = -1;
	for (int i = 0; i < size; i++) {
		int placed = 0;
		ledgeItem * item = ledger[i];
		if (item != NULL) {
			float distance = sqrtf(pow(item->x - x, 2) + pow(item->y - y, 2));
			if (distance < MAX_DISTANCE) {
				item->ack++;
				item->x = (item->ack * item->x + x) / (item->ack + 1);			
				item->y = (item->ack * item->y + y) / (item->ack + 1);			
				placed = 1;
			}
		} else if (pos == -1) pos = i;
		if (!placed) {
			ledger[pos] = createLedgeItem(x, y, pos);
		}
	}
}

void removeLedgeItem(ledgeItem * ledger[], int pos) {
	free(ledger[pos]);
	ledger[pos] = NULL;
}
#include "slave.h"

void Read_Ultrasonic(Slave * slave) {
	TMR2 = 0x0;
	
	/* enable HC-SR04 trigger on current state for 10us */
	long delay = SYSCLK / (2 * US_TRIG_T);
	switch (slave->usState) {
		case 0: 
			LATASET = _LATA_LATA2_MASK;
			while (delay--);
			LATACLR = _LATA_LATA2_MASK;
			break;
		case 1: 
			LATASET = _LATA_LATA3_MASK;
			while (delay--);
			LATACLR = _LATA_LATA3_MASK;
			break;
		case 2:
			LATBSET = _LATB_LATB10_MASK;
			while (delay--);
			LATBCLR = _LATB_LATB10_MASK;
			break;
		case 3:
			LATBSET = _LATB_LATB14_MASK;
			while (delay--);
			LATBCLR = _LATB_LATB14_MASK;
			break;
	}
}

/* Get minimum value in buffer for each sensor */
void US_Min(Slave * slave) {
	unsigned int minVal, currentVal;
	for (int i = 0; i < US_SENSORS; i++) {
		/*minVal = 0xFFFF;
		for (int j = 0; j < US_BUF_LEN; j++) {
			currentVal = ((unsigned short)slave->usData[j][2*i]) << 8 | ((unsigned short)slave->usData[j][2*i+1]);
			if (currentVal < minVal) minVal = currentVal;
		}
		slave->data[2*i+1] = minVal & 0xFF;
		slave->data[2*i] = (minVal >> 8) & 0xFF;*/
		slave->data[2*i+1] = slave->usData[US_BUF_LEN-1][2*i+1];
		slave->data[2*i] = slave->usData[US_BUF_LEN-1][2*i];
	}
}

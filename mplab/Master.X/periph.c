#include "periph.h"

/* Peripheral functions*/
void Init(struct Bot * bot) {
    /* Random function */
    srand(PORTA);
    
    /* Ports */
    ANSELA = 0x1;
    TRISA = 0x1;
    LATA = 0x0;
    LATASET = _LATA_LATA1_MASK;

    ANSELB = 0x0;
    TRISB = 0xC3A0;
    LATB = 0xC;
    LATBSET = _LATB_LATB10_MASK;

    if (bot->uartState) {
	U1RXR = 0x2;		// RA4 = RX
	RPB4R = 0x1;		// RB4 = TX
    } else {
	RPA2R = 0x5;		// RA2 = OC4
	RPA4R = 0x6;		// RA4 = OC5
    }
    T5CKR = 0x4;		// RB9 = T5CK
    INT3R = 0x4;		// RB8 = INT3
    CNCONB = 0x8000;
    CNENB = 0x20;		// CNB5 
    bot->portCN = PORTB;
    CNPUB = 0x200;		// RB9 internal pull-up
    
    /* Interrupts */
    __builtin_disable_interrupts();
    INTCONSET = _INTCON_MVEC_MASK;
    IPC2 = 0xA;			// TMR2
    IPC3 = 0xA00000A;		// INT3, TMR3
    IPC4 = 0xA;			// TMR4
    IPC5 = 0xA00000A;		// TMR5, ADC1
    IPC8 = 0xA0000;			// CN
    IFS0 = 0x0;
    IFS1 = 0x0;
    IEC0 = 0x110C0200;		// TMR2, INT3, TMR4, TMR5, ADC1
    IEC1 = 0x4000;		// CNB
    __builtin_enable_interrupts();
    
    /* Ultrasonic */
    TMR5 = 0x0;
    PR5 = 0xFFFF;           // f = 15 Hz
    T5CON = 0x80D0;         // gated time, 1:32 prescaler, TMR2 f = 1 MHz
    
    /* PWM */
    TMR3 = 0x0;
    PR3 = PWM_T;
    OC4R = 0x0;
    OC4RS = 0x0;
    OC4CON = 0xE;		// PWM mode without fault protection, timer 3
    OC5R = 0x0;
    OC5RS = 0x0;
    OC5CON = 0xE;
    T3CON = 0x8060;		// 1:64 prescaler = 500 kHz
    if (!bot->uartState) {
	OC4CONSET = _OC4CON_ON_MASK;
	OC5CONSET = _OC5CON_ON_MASK;
    }
    
    /* Sampling timer */
    TMR4 = 0x0;
    PR4 = 0x8235;		// f = 30.0003 Hz
    T4CON = 0x8050;		// 1:32 prescaler = 2 MHz (reset to 0x8050)
    
    /* Long counter */
    TMR2 = 0x0;
    PR2 = 0xFFFF;		// 2 Hz
    T2CON = 0x70;		// 1:256 prescaler = 125 kHz    
    
    /* I2C */
    I2C2BRG = (int)(PBCLK / (2 * I2C_BAUD) - 2);
    I2C2CON = _I2C2CON_ON_MASK;
    
     /* ADC */
    AD1CON1 = 0x2E0;			// fractional output, manual sampling
    AD1CON2 = 0x0;				// no scan, single conversion per interrupt
    AD1CON3 = 0xF03;			// 15 Tad sample, 250ns conversion
    AD1CHS = 0x0;				// AN0 - Vrefl
    AD1CSSL = 0x0;				// no inputs scanned
    AD1CON1SET = _AD1CON1_ADON_MASK;
    AD1CON1SET = _AD1CON1_SAMP_MASK;
    
    /* UART */
    U1MODE = 0x0;
    U1STA = 0x0;
    U1BRG = (int)(PBCLK / (16 * UART_BAUD) - 1);
    U1STASET = (_U1STA_UTXISEL0_MASK | _U1STA_URXEN_MASK | _U1STA_UTXEN_MASK);
    if (bot->uartState) U1MODESET = _U1MODE_ON_MASK;
    //*/
}

void UART_Write(char data) {
    while (U1STAbits.UTXBF);
    U1TXREG = data;
}

void UART_Write_String(char * data, int len) {
    while (len--) {
	UART_Write(*data);
	data++;
    }
}

void I2C_Master_Start() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.SEN = 1;
    while (I2C2CONbits.SEN);
    //snprintf(bot.buf, 100, "start: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Stop() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.PEN = 1;
    while (I2C2CONbits.PEN);
    //snprintf(bot.buf, 100, "stop: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Restart() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.RSEN = 1;
    while (I2C2CONbits.RSEN);
    //snprintf(bot.buf, 100, "restart: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

void I2C_Master_Ack_Nack(int val) {
    // ACK = 0 (slave should send another byte)
    // NACK = 1 (no more bytes requested by slave)
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.ACKDT = val;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN);
    //snprintf(bot.buf, 100, "ack: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
}

int I2C_Master_Write(char byte) {
    I2C2TRN = byte;
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    //snprintf(bot.buf, 100, "write: %d (%d)\r\n", I2C2STAT, byte);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    if (I2C2STATbits.ACKSTAT) return 0;
    return 1;
}

char I2C_Master_Read() {
    I2C2CONbits.RCEN = 1;
    //snprintf(bot.buf, 100, "read: %d\r\n", I2C2STAT);
    //UART_Write_String(bot.buf, strlen(bot.buf));
    while (!I2C2STATbits.RBF);
    return I2C2RCV;
}

int I2C_Write(char periphAdd, char regAdd, char data) {
    int ret = 0;
    I2C_Master_Start();
    ret |= I2C_Master_Write((periphAdd << 1) | I2C_W);
    ret |= I2C_Master_Write(regAdd);
    ret |= I2C_Master_Write(data);
    I2C_Master_Stop();
    return ret;
}

int I2C_Read(char periphAdd, char regAdd, char * data, int len) {
    int ret = 0;
    I2C_Master_Start();
    ret |= I2C_Master_Write((periphAdd << 1) | I2C_W);
    ret |= I2C_Master_Write(regAdd);
    I2C_Master_Stop();
    I2C_Master_Start();
    I2C_Master_Write((periphAdd << 1) | I2C_R);
    
    for (int i = 0; i < len; i++) {
	data[i] = I2C_Master_Read();
	if (i != len - 1) I2C_Master_Ack_Nack(0);
    }
    I2C_Master_Ack_Nack(1);
    I2C_Master_Stop();
}

void Trigger_Ultrasonic(struct Bot * bot) { 
    bot->usState = 0;
    TMR5 = 0;
    /* enable HC-SR04 trigger on current state for 10us */
    LATBSET = _LATB_LATB10_MASK;
    delay(10);
    LATBCLR = _LATB_LATB10_MASK;
}
#include "periph.h"

/* Peripheral functions*/
void Init() {
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

    RPA2R = 0x5;		// RA2 = OC4
    RPA4R = 0x6;		// RA4 = OC5
    RPB14R = 0x2;		// RB14 = U2TX
    T5CKR = 0x4;		// RB9 = T5CK
    INT3R = 0x4;		// RB8 = INT3
    CNCONB = 0x8000;
    CNENB = 0x20;		// CNB5 
    CNPUB = 0x200;		// RB9 internal pull-up
    
    /* DMA */
    DMACONSET = _DMACON_ON_MASK;
    DCH1SSA = KVA_TO_PA(bot->buf);
    DCH1DSA = KVA_TO_PA(&U2TXREG);
    DCH1ECONbits.CHSIRQ = 55;
    DCH1ECONbits.SIRQEN = 1;
    DCH1SSIZ = BUF_LEN;
    DCH1DSIZ = 1;
    DCH1CSIZ = 1;
    
    /* Interrupts */
    __builtin_disable_interrupts();
    INTCONSET = _INTCON_MVEC_MASK;
    IPC1 = 0xA;			// TMR1
    IPC3 = 0xA00000A;		// INT3, TMR3
    IPC5 = 0xA00000A;		// TMR5, ADC1
    IPC8 = 0xA0000;		// CN
    IPC10 = 0xA00;		// DMA1
    
    IFS0 = 0x0;
    IFS1 = 0x0;
    IEC0 = 0x11040010;		// TMR1, INT3, TMR5, ADC1
    IEC1 = 0x20004000;		// CNB, DMA1
    __builtin_enable_interrupts();
    
    /* Sampling timer */
    TMR1 = 0x0;
    PR1 = 0x8235;		// f = 30.0003 Hz
    T1CON = 0x8020;		// 1:64 prescaler = 1 MHz (reset to 0x8050)
    
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
    OC4CONSET = _OC4CON_ON_MASK;
    OC5CONSET = _OC5CON_ON_MASK; 
    
    /* Odometers */
    TMR2 = 0x0;
    PR2 = 0xFFFF;
    T2CON = 0x8020;
    TMR4 = 0x0;
    PR4 = 0xFFFF;
    T4CON = 0x8020;
    
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
    U2MODE = 0x8;			// high speed mode
    U2STA = 0x400;			// TX enabled
    U2BRG = UART_BRGH;
    U2MODESET = _U2MODE_ON_MASK;
    //UART_Write_String("AT+BAUD8", 8);
    
    
}

void UART_Write(char data) {
    while (U2STAbits.UTXBF);
    U2TXREG = data;
}

void I2C_Master_Start() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.SEN = 1;
    while (I2C2CONbits.SEN);
}

void I2C_Master_Stop() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.PEN = 1;
    while (I2C2CONbits.PEN);
}

void I2C_Master_Restart() {
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.RSEN = 1;
    while (I2C2CONbits.RSEN);
}

void I2C_Master_Ack_Nack(int val) {
    // ACK = 0 (slave should send another byte)
    // NACK = 1 (no more bytes requested by slave)
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    I2C2CONbits.ACKDT = val;
    I2C2CONbits.ACKEN = 1;
    while (I2C2CONbits.ACKEN);
}

int I2C_Master_Write(char byte) {
    I2C2TRN = byte;
    while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2STATbits.TRSTAT);
    if (I2C2STATbits.ACKSTAT) return 0;
    return 1;
}

char I2C_Master_Read() {
    I2C2CONbits.RCEN = 1;
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

void Ultrasonic_Trigger(struct Bot * bot) { 
    TMR5 = 0;
    /* enable HC-SR04 trigger on current state for 10us */
    LATBSET = _LATB_LATB10_MASK;
    delay(10);
    LATBCLR = _LATB_LATB10_MASK;
}

void Odometer_Read(uint8_t odo[2]) {
    odo[0] = TMR2;
    odo[1] = TMR4;
    TMR2 = 0x0;
    TMR4 = 0x0;
}
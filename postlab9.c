/*
 * File:   prelab9.c
 * Author: Carlos
 *
 * Created on 27 de abril de 2022, 04:14 PM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


#include <xc.h>
#include <stdint.h>
//CONSTANTES
#define _XTAL_FREQ 500000
#define IN_MIN 0
#define IN_MAX 255
#define OUT_MIN 125
#define OUT_MAX 250
//VARIABLES
uint8_t POT_CONT = 0;
uint8_t CONT = 0;
unsigned short CCPR_UNO = 0;
unsigned short CCPR_DOS = 0;
//PROTOTIPO DE FUNCIONES
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, unsigned short out_min, unsigned short out_max);
void __interrupt() isr (void){
    PORTB ++;
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){
            CCPR_UNO = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            CCPR1L = (uint8_t)(CCPR_UNO>>2);
            CCP1CONbits.DC1B = CCPR_UNO & 0b11;
            
        }
        else if(ADCON0bits.CHS == 2){
            CCPR_DOS = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            CCPR2L = (uint8_t)(CCPR_DOS>>2);
            CCP2CONbits.DC2B1 = (CCPR_DOS & 0b10)>>1;
            CCP2CONbits.DC2B0 = CCPR_DOS & 0b01;
        }
        else if (ADCON0bits.CHS == 3){
            POT_CONT = ADRESH;
        }
        PIR1bits.ADIF = 0;
    }
    if (INTCONbits.T0IF ==1){
        CONT++;
        INTCONbits.T0IF = 0;
        TMR0 = 99;
        if (CONT < POT_CONT){
            PORTDbits.RD0 = 1;
        }
        else
            PORTDbits.RD0 = 0;
    }
    
}
void setup (void){
    ANSEL = 0b00001101;
    ANSELH = 0;
    TRISA = 0b00001101;
    TRISB = 0;
    TRISD = 0;
    PORTB = 0;
    PORTA = 0;
    PORTD = 0;
    //CONFIGURACIÓN DEL RELOJ INTERNO
    OSCCONbits.IRCF = 0b11;
    OSCCONbits.SCS = 1;
    //CONFIGURACIÓN ADC
    ADCON0bits.ADCS = 0b01;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.CHS = 0b0000;
    ADCON0bits.CHS = 0b0010;
    ADCON0bits.CHS = 0b0011;
    ADCON1bits.ADFM = 0;
    ADCON0bits.ADON = 1;
    __delay_us(40);
    //CONFIGURACIÓN PWM
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;
    PR2 = 156;
    //CONFIGURACIÓN CCP
    CCP1CON = 0;
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b1100;
    CCPR1L = 125 >> 2;
    CCP1CONbits.DC1B = 125 & 0b11;
    
    CCP2CON = 0;
    CCP2CONbits.CCP2M = 0b1100;
    CCPR2L = 125 >> 2;
    CCP2CONbits.DC2B0 = 125 & 0b01;
    CCP2CONbits.DC2B1 = 125 & 0b10;
    
    
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS = 0b01;
    T2CONbits.TMR2ON = 1;
    while (!PIR1bits.TMR2IF);
    PIR1bits.TMR2IF = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    //CONFIGURACIÓN DE TIMER 0
   //Timer0 Registers Prescaler= 16 - TMR0 Preset = 99 - Freq = 49.76 Hz - Period = 0.020096 seconds
    OPTION_REGbits.T0CS = 0;  // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;  // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;   // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the Timer0
    OPTION_REGbits.PS2 = 0;   // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 1;
    TMR0 = 99;   
    
   //CONFIGURACIÓN INTERRUPCIONES
    
    PIR1bits.ADIF = 0;
    INTCONbits.T0IF = 0;
    PIE1bits.ADIE = 1;
    INTCONbits.T0IE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    return;
}

void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            if(ADCON0bits.CHS == 0b0000)    
                ADCON0bits.CHS = 0b0010;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0010)
                ADCON0bits.CHS = 0b0011;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0011)
                ADCON0bits.CHS = 0b0000;    // Cambio de canal
            
            __delay_us(40);                 // Tiempo de adquisici n?
            
            ADCON0bits.GO = 1;             
    }
    }
return;
}
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
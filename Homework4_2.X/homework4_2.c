/*
 * File:   homework4_2.c
 * Author: partw
 *
 * Created on February 17, 2021, 9:10 PM
 */


#ifndef CONFIG_H
#define CONFIG_H
#include <xc.h>

#define _XTAL_FREQ 1000000

// PIC18F4331 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = IRCIO // Oscillator Selection bits (Internal oscillator block, port function on RA6 and port function on RA7)
#pragma config FCMEN = ON // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON // Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRTEN = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bits (Brown-out Reset disabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDTEN = OFF // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDPS = 32768 // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WINEN = OFF // Watchdog Timer Window Enable bit (WDT window disabled)

// CONFIG3L
#pragma config PWMPIN = OFF // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
#pragma config LPOL = HIGH // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
#pragma config HPOL = HIGH // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
#pragma config T1OSCMX = ON // Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)

// CONFIG3H
#pragma config FLTAMX = RC1 // FLTA MUX bit (FLTA input is multiplexed with RC1)
#pragma config SSPMX = RC7 // SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RC5 and RC4, respectively. SDO output is multiplexed with RC7.)
#pragma config PWM4MX = RB5 // PWM4 MUX bit (PWM4 output is multiplexed with RB5)
#pragma config EXCLKMX = RC3 // TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RC3)
#pragma config MCLRE = ON // MCLR Pin Enable bit (Enabled)

// CONFIG4L
#pragma config STVREN = OFF // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF // Low-Voltage ICSP Enable bit (Low-voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF // Code Protection bit (Block 1 (001000-001FFF) not code-protected)

// CONFIG5H
#pragma config CPB = OFF // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF // Write Protection bit (Block 1 (001000-001FFF) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#endif / XC_HEADER_TEMPLATE_H /

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#define _XTAL_FREQ 1000000 //sets frequency


void __interrupt() high_isr(void){ //caused by button at RC3
    INTCONbits.GIEH = 0;
    if(INTCONbits.INT0IF){
        LATBbits.LB0 = ~LATBbits.LB0; //toggle white led 0001
        LATBbits.LB2 = ~LATBbits.LB2; //toggle green led 0100
        INTCONbits.INT0IF = 0;
    }   
    INTCONbits.GIEH = 1; 
}
    
    
void __interrupt(low_priority) low_isr(void){ //caused by button at RC4
    INTCONbits.GIEH = 0;
    if(INTCON3bits.INT1IF){
        LATBbits.LB1 = ~LATBbits.LB1; //toggle yellow led 0010
        LATBbits.LB3 = ~LATBbits.LB3; //toggle blue led 1000
        INTCON3bits.INT1IF = 0;
    }
    INTCONbits.GIEH = 1;
}

void main(void){
    TRISCbits.RC3=1;        //Set RC3 as input INT0
    TRISCbits.RC4=1;        //Set RC4 as input INT1
    TRISBbits.RB0=0;        //Set RB0 as output
    TRISBbits.RB1=0;        //Set RB1 as output
    TRISBbits.RB2=0;        //Set RB1 as output
    TRISBbits.RB3=0;        //Set RB1 as output
    
    
    LATBbits.LB0 = 0;       //all leds start off
    LATBbits.LB1 = 0;
    LATBbits.LB2 = 0;
    LATBbits.LB3 = 0;
    
    RCONbits.IPEN = 1;      //Enable priority levels
    
    INTCON3bits.INT1E = 1;  //Enable INT0
    INTCON3bits.INT1P = 0;  //Low priority
    INTCON2bits.INTEDG1 = 1;//Enable High edge trigger
   
    INTCONbits.INT0IE = 1;  //Enable INT1 
    INTCON2bits.INTEDG0 = 1;//Enable high edge trigger
    
    INTCONbits.GIEH = 1;    //Enable high interrupts
    INTCONbits.GIEL = 1;    //Enable high interrupts
   
    while(1){ //loop to keep program running until interrupts occur
    } 
   
}

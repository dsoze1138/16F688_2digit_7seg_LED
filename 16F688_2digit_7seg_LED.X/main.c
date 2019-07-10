/*
 * File:   main.c
 * Author: dan1138
 * Target: PIC16F688
 * Compiler: XC8 v2.00
 *
 *                       PIC16F688
 *             +------------:_:------------+
 *    GND -> 1 : VDD                   VSS : 14 <- 5v0
 * SEG_An <> 2 : RA5/T1CKI     PGD/AN0/RA0 : 13 <> PGD DIGIT_1n
 *        <> 3 : RA4/AN3       PGC/AN1/RA1 : 12 <> PGC DIGIT_2n
 *    VPP -> 4 : RA3/VPP           AN2/RA2 : 11 <>
 * SEG_Bn <> 5 : RC5/RXD           AN4/RC0 : 10 <> SEG_Gn
 * SEG_Cn <> 6 : RC4/TXD           AN5/RC1 : 9  <> SEG_Fn
 * SEG_Dn <> 7 : RC3/AN7           AN6 RC2 : 8  <> SEG_En
 *             +---------------------------:
 *                        DIP-14
 *
 * Created on July 7, 2019, 6:56 PM
 */

#pragma config FOSC = INTOSCIO
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ (8000000ul)

const char LEDDigit[] = 
{ 
  /* abcdefg         _   */
  0b00000001,   /*  | |  */
                /*  |_|  */
                /*       */
  0b01001111,   /*    |  */
                /*    |  */
                /*   _   */
  0b00010010,   /*   _|  */
                /*  |_   */
                /*   _   */
  0b00000110,   /*   _|  */
                /*   _|  */
                /*       */
  0b01001100,   /*  |_|  */
                /*    |  */
                /*   _   */
  0b00100100,   /*  |_   */
                /*   _|  */
                /*   _   */
  0b00100000,   /*  |_   */
                /*  |_|  */
                /*   _   */
  0b00001111,   /*    |  */
                /*    |  */
                /*   _   */
  0b00000000,   /*  |_|  */
                /*  |_|  */
                /*   _   */
  0b00001100,   /*  |_|  */
                /*    |  */
                /*   _   */
  0b00001000,   /*  |_|  */
                /*  | |  */
                /*       */
  0b01100000,   /*  |_   */
                /*  |_|  */
                /*   _   */
  0b00110001,   /*  |    */
                /*  |_   */
                /*       */
  0b01000010,   /*   _|  */
                /*  |_|  */
                /*   _   */
  0b00110000,   /*  |_   */
                /*  |_   */
                /*   _   */
  0b00111000,   /*  |_   */
                /*  |    */
                /*       */
  0b01111111,   /* blank */
                /*       */
};
volatile uint8_t Digit1Segments;
volatile uint8_t Digit2Segments;

void __interrupt() ISR_handler(void)
{
    static uint8_t Timer0Ticks;
    
    if (TMR0IE && TMR0IF) {  /* TIMER0 asserts and interrupt every 1.024 milliseconds */
        TMR0IF=0;
        if ((Timer0Ticks++ & 0x0F) == 0) { /* every 16.384 drive a new digit */
            if ((TRISA & 3) == 2) {
                TRISA |= 1;     /* Turn off all digit drivers */
                PORTA = (1<<5);
                if ((Digit2Segments & (1<<6)) == 0 ) {
                    PORTA = 0;
                }
                PORTC = Digit2Segments;
                TRISAbits.TRISA1 = 0;    /* Drive digit 2 segments */
            }
            else { 
                TRISA |= 1;     /* Turn off all digit drivers */
                TRISA |= 2;
                PORTA = (1<<5);
                if ((Digit1Segments & (1<<6)) == 0 ) {
                    PORTA = 0;
                }
                PORTC = Digit1Segments;
                TRISAbits.TRISA0 = 0;    /* Drive digit 1 segments */
            }
        }
    }
}

void main(void) {
    uint8_t HexCount;
    /*
     * Initialize this PIC
     */
    INTCON = 0;
    OSCCON = 0x70;      /* Select 8MHz system oscillator */
    __delay_ms(500);    /* Give ICSP device programming tool a chance to get the PICs attention */
    
    Digit1Segments = 0b01111111;
    Digit2Segments = 0b01111111;
    TRISA = 0xDF;   /* PORTA bit 5 needs to be an output */
    TRISC = 0x00;
    ANSEL  = 0;
    OPTION_REG = 0b11000010; /* TIMER0 clock = FOSC/4, prescale 1:8 */
    PORTA = 0;
    PORTC = 0;
    CMCON0 = 7;
    TMR0 = 0;
    TMR0IF = 0;
    TMR0IE = 1;
    GIE = 1;
    /*
     * This is the application loop.
     * 
     * It counts up one count about every second.
     */
    HexCount = 0;
    while(1)
    {
        Digit1Segments = LEDDigit[HexCount & 0x0F];
        Digit2Segments = LEDDigit[(HexCount>>4) & 0x0F];
        __delay_ms(1000);
        HexCount++;
    }
}

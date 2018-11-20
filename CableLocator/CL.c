/*
 * Project name:  CABLE LOCATOR
     Signal generator to detect an underground cable with a standard
     Metal Detector
 * Copyright:
     (c) BW, 2017.
 * Configuration:
     MCU:             PIC16F1824
     Oscillator:      Internal, 32.0000 MHz
   * NOTES:
 */
//======================================================================
//
// constant values
//
//#define RS_OUTPUT

// I/O pins
//#define DAC_OUT          PORTA.RA0            // DAC OUT --> opamp DC offset
#define TRANSFO1         LATC.LATC3            // Transfo 1 FET =  pin 7
#define TRANSFO2         LATC.LATC4            // Transfo 2 FET =  pin 6

#define BURST_RELOAD     10000                // Nr of periods before changing the sweep

#define TRUE 1
#define FALSE 0

#define OFF 0
#define ON  1

#define IN 1
#define OUT 0


static int timervalue;
static int timerperiod;
static unsigned char state;
static unsigned int burst;

//======================================================================
//
//  ISR
//  Handle timer1 interrupt as timebase
//
//

void interrupt(void)
{

     // Timer1 interrupt on each overflow from FFFF to 0000
     if (PIR1.TMR1IF)
     {
        // Change state
        switch (state)
        {
               case 0:
                    TRANSFO1 = OFF;
                    TRANSFO2 = OFF;
                    state = 1;
                    break;
               case 1:
                    TRANSFO1 = ON;
                    TRANSFO2 = OFF;
                    state = 2;
                    break;
               case 2:
                    TRANSFO1 = OFF;
                    TRANSFO2 = OFF;
                    state = 3;
                    break;
               case 3:
                    TRANSFO1 = OFF;
                    TRANSFO2 = ON;
                    state = 0;
                    break;
               default:  // we should never get here
                    TRANSFO1 = OFF;
                    TRANSFO2 = OFF;
                    state = 0;
                    break;

        }

        // Reload timer
        TMR1H = (timervalue & 0xFF00) >> 8;
        TMR1L = timervalue & 0x00FF;
        
        // Reset interrupt flag
        PIR1.TMR1IF = 0;
      }
}




//===================================
//
// main idle loop
//
void main()
{
    unsigned int frequency_select;

    // oscillator
    OSCCON= 0xF0;
    
    // GPIO init
    // PORT A
    TRISA.TRISA0 = IN;  // PIN 13 = potmeter
    PORTA.RA0 = 1;
    TRISA.TRISA1 = IN;   // PIN 12
    PORTA.RA1 = 1;
    TRISA.TRISA2 = OUT;  // PIN 11
    PORTA.RA2 = 1;
    TRISA.TRISA3 = OUT;  // PIN 4
    PORTA.RA3 = 1;
    TRISA.TRISA4 = IN;   // PIN 3
    PORTA.RA4 = 1;
    TRISA.TRISA5 = OUT;  // PIN 2
    PORTA.RA5 = 1;

    // PORT C
    TRISC.TRISC0 = IN;  // PIN 10
    PORTC.RC0 = 1;
    TRISC.TRISC1 = IN;  // PIN 9
    PORTC.RC1 = 1;
    TRISC.TRISC2 = IN;  // PIN 8
    PORTC.RC2 = 1;
    TRISC.TRISC3 = OUT;  // PIN 7  = FET TRANSFO1
    PORTC.RC3 = 1;
    TRISC.TRISC4 = OUT; // PIN 6 = FET TRANSFO2
    PORTC.RC4 = 1;
    TRISC.TRISC5 = OUT; // PIN 5
    PORTC.RC5 = 1;

    // Analog input pins
    ANSELA.ANSA0 = 1;     //
    ANSELA.ANSA1 = 1;     // frequency potmeter in
    ANSELA.ANSA4 = 1;     //
    ANSELC.ANSC0 = 1;     //
    ANSELC.ANSC1 = 1;     //
    ANSELC.ANSC2 = 1;     //
    
  // Timer 1  16 bit timebase
    T1GCON.TMR1GE = 0;                // timer 1 gate
    T1CON.T1CKPS0 = 0;
    T1CON.T1CKPS1 = 0;                // timer 1 prescaler = 0
    T1CON.TMR1CS0 = 1;                // timer 1 clock = Fosc
    T1CON.TMR1CS1 = 0;                // timer 1 clock = Fosc
    T1CON.T1OSCEN = 0;                // timer 1 internal oscillator disabled
    T1CON.T1CKPS0 = 0;                // timer 1 prescaler 1:1
    T1CON.T1CKPS1 = 0;
    TMR1H = 0x00;                        // initial timer values
    TMR1L = 0x00;
    PIR1.TMR1IF = 0;

     // ADC input
    ADCON0.ADON = 1;            //  ADC on
    ADCON1.ADFM = 1;            // right justified
    ADCON1.ADCS0 = 0;           // Fosc / 64
    ADCON1.ADCS1 = 1;           // Fosc / 64
    ADCON1.ADCS2 = 1;           // Fosc / 64
    ADCON1.ADNREF = 0;          // Vref- = VSS
    ADCON1.ADPREF0 = 0;         // Vref+ = VDD
    ADCON1.ADPREF1 = 0;         // Vref+ = VDD

    // Startup values
    state = 0;


    // Enable timer1 interrupt
    T1CON.TMR1ON = 1;                 // timer 1 active
    INTCON.PEIE = 1;
    PIE1.TMR1IE = 1;                // timer 1 interrupt enabled
    INTCON.GIE = 1;

  

     // Main idle loop
     while(1)
     {
           // Frequency selection potmeter
           // When the value is too low, the period is too short, resulting in continuous interrupts,
           // so this ADC read won't happen again...
           frequency_select =  ADC_Read(0);
           if (frequency_select < 50)
           {
              frequency_select = 50;
           }
           frequency_select <<= 3; // 8 .. 8192 ( 1 .. 1024)

           timerperiod = -frequency_select;
           timervalue = timerperiod >> 2;       // period / 4
           

           // Delay
           Delay_ms(100);


    }  // while(1)

} //~!
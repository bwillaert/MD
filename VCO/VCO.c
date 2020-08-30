/*
 * Project name:
     VCO
 * Copyright:
     (c) BW, 2020.
 * Configuration:
     MCU:             PIC12F1840
     Oscillator:      Internal, 32.0000 MHz
   * FEATURES:
     - Enable input - high = enqbled



   12F1840:

Timer2: Fosc/4 = 8MHz
Prescaler 1;1
TMR2 comparator  - PR2 = 8  ==>overflow 1 MHz = 1 microseconde
Postscaler 1:1



Timer interrupt 1 microseconde

Vin = 0..1024 => 0..4096 (Hz) Vin = 0 => Vin = 1 = 10000000 s periode = 1s

periode = periode/2 = timer count = halve periode = output toggle

Periode in microseconden = 1000000/Vin


*/
//======================================================================
//
// constant values
//

// I/O pins
//#define COMP_IN          PORTA.RA4            // IN AN                  comparator 2 input     pin 3
#define BEEP             LATA.LATA2           // OUT DIG                VCO audio output       pin 5
#define VOLTAGE_IN       PORTA.RA1            // IN AN                  VCO voltage input  pin 6
//#define SUPPLY_SENSE     PORTA.RA0            // IN AN                  low battery voltage detection  pin 7
//#define PI_TX            LATA.LATA5           // OUT  DIG               TX pulse                pin 2
#define ENABLE           PORTA.RA3            // IN DIG                 enable VCO: active high      pin 4

#define VCO_PERIOD_MAX 1363968                // ° 1 microsecond  = 333 * 4096 for 3 kHz max
#define MAIN_LOOP_DURATION 300                // 300 microseconds

#define TRUE 1
#define FALSE 0

#define OFF 0
#define ON  1

#define IN 1
#define OUT 0




void VDelay_us(unsigned time_us)
{
  unsigned n_cyc;
  n_cyc = Clock_MHz()>>2;
  n_cyc *= time_us>>4;
  while (n_cyc--) 
  {
      asm nop;
      asm nop;
  }
}

//===================================
//
// Main idle loop
//
void main()
{
    unsigned long voltage_in;
    unsigned long vco_period;
    unsigned long main_loop_delay;
    unsigned long main_loop_delay_cnt;
    
    // oscillator
    OSCCON= 0xF0;       // PLLenabled + 8MHz internal
    
    // GPIO init
    // PORT A
    TRISA.TRISA0 = IN;  // low battery
    PORTA.RA0 = 1;
    TRISA.TRISA1 = IN;  // sensitivity potmeter
    PORTA.RA1 = 1;
    TRISA.TRISA2 = OUT;  // sound out
    PORTA.RA2 = 1;
    TRISA.TRISA3 = IN;   // pinpoint pushbutton in
    PORTA.RA3 = 1;
    TRISA.TRISA4 = IN;  // comparator in
    PORTA.RA4 = 1;
    TRISA.TRISA5 = OUT;  // TX pulse
    PORTA.RA5 = 0;

    // Analog input pins
    //ANSELA.ANSA0 = 1;     // low battery
    ANSELA.ANSA1 = 1;     // VCO voltage input
    //ANSELA.ANSA4 = 1;     // pulse in


     // ADC input
    ADCON0.ADON        = 1;        //  ADC on
    ADCON1.ADFM        = 1;        // right justified
    ADCON1.ADCS0       = 0;        // Fosc / 64
    ADCON1.ADCS1       = 1;        // Fosc / 64
    ADCON1.ADCS2       = 1;        // Fosc / 64
    //ADCON1.ADNREF = 0;           // Vref- = VSS
    ADCON1.ADPREF0     = 0;        // Vref+ = VDD
    ADCON1.ADPREF1     = 0;        // Vref+ = VDD


    // timer0 timebase  - for sound generation
    T2CON.T2CKPS0      = 0;       // Prescaler 1:1 => 8 Mhz = Fosc/4
    T2CON.T2CKPS1      = 0;
    T2CON.T2OUTPS0     = 0;      // Postscaler 1:1
    T2CON.T2OUTPS1     = 0;
    T2CON.T2OUTPS2     = 0;
    T2CON.T2OUTPS3     = 0;
    PR2                = 8;       // 8 Mhz / 8 = 1 MHz
    T2CON.TMR2ON       = 1;       // Timer2 ON
    PIE1.TMR2IE        = 1;       // Enable TMR2 interrupt

   // INTCON.GIE         = 1;
   //--------------------




    // Main loop
    while(TRUE)
    {
      // Check voltage input 0...4095
    //  voltage_in = ADC_Read(1) << 2;
      voltage_in = 1678;
      if (voltage_in == 0)
      {
         voltage_in = 1;
      }

      // Calculate new divider
      vco_period = VCO_PERIOD_MAX / voltage_in;
      vco_period >>= 1;
      
      vco_period = 90000;

      if (vco_period > MAIN_LOOP_DURATION)
      {
         main_loop_delay =  vco_period - MAIN_LOOP_DURATION;
         while (main_loop_delay)
         {
           if (main_loop_delay > 65535)
           {
              VDelay_us(65535);
              main_loop_delay -= 65535;
           }
           else
           {
              VDelay_us(main_loop_delay);
              main_loop_delay = 0;
           }
         }
      }
      BEEP = !BEEP;

    }  // while()

} //~!
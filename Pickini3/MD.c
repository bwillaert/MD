/*
 * Project name:
     PICKINI
     Pulse Induction
     Metal detector
 * Copyright:
     (c) BW, 2014.
 * Configuration:
     MCU:             PIC12F1840
     Oscillator:      Internal, 32.0000 MHz
   * NOTES:
     - Motion detection only

*/
//======================================================================
//
// constant values
//

// I/O pins
#define COMP_IN          PORTA.RA4            // IN AN                  comparator 2 input     pin 3
#define BEEP             LATA.LATA2           // OUT DIG                audio          pin 5
#define SENS_IN          PORTA.RA1            // IN AN                  potmeter sensitivity  pin 6
#define SUPPLY_SENSE     PORTA.RA0            // IN AN                  low battery voltage detection  pin 7
#define PI_TX            LATA.LATA5           // OUT  DIG               TX pulse                pin 2
#define PINPOINT         PORTA.RA3            // IN DIG                 pinpoint pushbutton


#define TX_PULSE_WIDTH           120     // TX pulse width in microseconds
#define PULSE_TIME_DIVIDER       4       // * 409.6 micros -- adapted dynamically with sensitivity
#define ACCUMULATE_MEASURE_CNT   5       // *  PULSE_TIME_DIVIDER * 409.6 micros
#define LV_TIME_DIVIDER          50000   // * 409.6 micros -- battery voltage check
#define SENSITIVITY_TIME_DIVIDER 2000    // * 409.6 micros -- sensitivity potmeter check
#define LOW_BATT_LIMIT           0x170   // 10 bits ADC  - max 3FF   0x170 = 10V - 47K+10K
#define STARTUP_DELAY            10000    // * 409.6 micros -- startup settling time
#define COIL_TIMEOUT             10      // * 409.6 micros -- coil pulse timeout
#define SAMPLE_ARRAYSIZE         32
#define SAMPLE_AVESHIFT          5
#define DIFF_ARRAYSIZE           8
#define DIFF_AVESHIFT            3
#define PULSE_ARRAY_SIZE         32
#define TRUE 1
#define FALSE 0

#define OFF 0
#define ON  1

#define IN 1
#define OUT 0


// globals
static unsigned char detection;
static unsigned int measurecnt;
static unsigned int measure_cal;
static unsigned int accumulate_measure_cnt;
static unsigned int pulse_time;
static unsigned char pulse_flag;
static unsigned int beepdivider;
static unsigned int beepcnt;
static unsigned int lv_time;
static unsigned char lv_flag;
static unsigned char sensitivity_flag;
static unsigned int sensitivity_time;
static unsigned int  startup_delay;
static unsigned char cal_cnt ;
static unsigned int coil_timeout;
static unsigned long pulse_average_array[PULSE_ARRAY_SIZE];
static unsigned char pulse_average_cnt;
static unsigned long pulse_average;


//======================================================================
//
//  ISR
//  handle timer interrupt for sound generation   - 512 microseconds
//
//
void interrupt(void)
{

     // timer0 interrupt
     if (INTCON.TMR0IF)
     {
          // sound
          if (beepcnt)
          {
            beepcnt--;
            {
               if (!beepcnt)
               {
                  BEEP = !BEEP;
                  beepcnt = beepdivider ;
               }
            }
          }
          else
          {
              beepcnt = beepdivider;
          }


          // induction pulse
          if (pulse_time)
          {
             pulse_time--;
             if (!pulse_time)
             {
                // reload pulse time divider
                pulse_time = PULSE_TIME_DIVIDER;
                pulse_flag = TRUE;
             }
          }

          // low voltage alarm
          if (lv_time)
          {
             lv_time--;
             if (!lv_time)
             {
                // reload low voltage control divider
                lv_time = LV_TIME_DIVIDER;
                lv_flag = TRUE;
             }
          }
          
         // sensitivity check
          if (sensitivity_time)
          {
             sensitivity_time--;
             if (!sensitivity_time)
             {
                // reload low voltage control divider
                sensitivity_time = SENSITIVITY_TIME_DIVIDER;
                sensitivity_flag = TRUE;
             }
          }

          // startup delay
          if (startup_delay)
          {
             startup_delay--;
          }
          
          // coil timeout
          if (coil_timeout)
          {
             coil_timeout--;
          }
        // reset interrupt flag
        INTCON.TMR0IF = 0;

     }
}







//======================================================================
//
//  play sound on a pin with a frequency and duration
//  period in 100 microseconds
//  duration in  100 microseconds
//
void sound (unsigned int period, unsigned long duration)
{
      unsigned long time_played;
      int i;
      time_played = 0;
      // period /2
      period >>=1 ;
      while (time_played < duration)
      {
          for (i = 0; i < period ; i++)
          {
              Delay_us(100);
          }
          time_played += period;
          // toggle the sound pin
          BEEP = !BEEP;
     }
}

void start_sound()
{
    INTCON.GIE = 0;
    Delay_ms(200);
    sound ( 50, 1000);    // 200 Hz
    sound ( 20, 1000);    // 500 Hz
    sound ( 10, 1000);    // 1 KHz
    // reset interrupt flags
    TMR0 = 0;
    INTCON.T0IF = 0;
    INTCON.GIE = 1;
}

void low_voltage_sound()
{
    // low battery sound
    INTCON.GIE = 0;
    sound ( 100, 1000);    // 100 Hz
    Delay_ms(200);
    sound ( 100, 1000);    // 100 Hz
   // reset interrupt flags
    TMR0 = 0;
    INTCON.T0IF = 0;
    INTCON.GIE = 1;
}

void coil_error_sound()
{
    // coil error sound
    INTCON.GIE = 0;
    sound ( 50, 500);     // 200 Hz
    Delay_ms(200);
    sound ( 33, 500);     // 300Hz
   // reset interrupt flags
    TMR0 = 0;
    INTCON.T0IF = 0;
    INTCON.GIE = 1;
}

unsigned long absvalue(unsigned long a, unsigned long b)
{
    if (a > b)
    {
      return (a-b);
    }
    else
    {
      return (b-a);
    }

}

//===================================
//
// TX pulse + measure return pulse
//
void tx_pulse_processing()
{
      static unsigned long pulsediff = 0;
      static unsigned long pulsevalue = 0;
      static unsigned long old_pulsevalue = 0;
      char i = 0;

      // Disable interrupts
      INTCON.GIE = 0;

      // Generate TX pulse
      PI_TX = 1;
      Delay_us(TX_PULSE_WIDTH);
      PI_TX = 0;

      // Re-enable interrupts - needed for coil timeout
      INTCON.GIE = 1;

      // Wait for comparator1 high
      coil_timeout = COIL_TIMEOUT;
      while (!CM1CON0.C1OUT && coil_timeout);
      if (!coil_timeout && !startup_delay)
      {
          // COIL ERROR
          coil_error_sound();
          return;
      }

      // Wait for comparator1 to go low again
      coil_timeout = COIL_TIMEOUT;
      while (CM1CON0.C1OUT && coil_timeout);
      if (!coil_timeout && !startup_delay)
      {
         // COIL ERROR
         coil_error_sound();
         return;
      }


     // Accumulate a number of measurements into TMR1 for more resolution
     // e.g. 4 measurements  every 2.5 ms = 100 Hz measurements
     // Pulse width = 100 micros @ 32 MHz = 3300
     pulsevalue += ((TMR1H<<8) | TMR1L);
     // Reset the pulse width counter
     TMR1H = 0;
     TMR1L = 0;
     PIR1.TMR1IF = 0;

     measurecnt--;
     if (!measurecnt)
     {
         // When we are past the startup delay, process the pulse
         if (!startup_delay)
         {
           // Get the average pulse value
            pulse_average = 0;
            for (i = 0; i < PULSE_ARRAY_SIZE; i++)
            {
                pulse_average += pulse_average_array[i];
            }
            pulse_average >>= 5;  // divide bij 32

            //  Get absolute difference with average value
            // Only when the new pulsevalue is larger than the pulse_average,
            // we take it into account to generate an alarm sound
            // 64 = 0.5% ---> 12800 / 4 = 3200 --> 64 = 2%
            
            if (pulsevalue > pulse_average)
            {
               pulsediff = absvalue(pulsevalue, pulse_average); 
            }
            else
            {
                pulsediff  = 0; 
            }

            // Limit difference to n steps
            // Should be in line with the number of accumulated timer slots
            // e.g. 5 slots accumulated x 50 us x 20MHz = 1000 x 5 = 5000
            // A pulsediff of 32 = 0.1 % diff allowed --> unstability !!
            if (pulsediff > 64) 
            {
               pulsediff = 64;
            }
 
           // Calculate audio divider -- based on 500us timer interrupt;
            beepdivider = (2048 - (pulsediff << 5)) + 1;
            if (beepdivider < beepcnt) beepcnt = beepdivider;
            
         } // if !startup_delay
         
         // Add new sample -- when pinpoint pushbutton is not active
         if (PINPOINT)
         {
             pulse_average_array[pulse_average_cnt%PULSE_ARRAY_SIZE] = pulsevalue;
         }
         
         pulsevalue  = 0;
         pulse_average_cnt++;
         measurecnt = accumulate_measure_cnt;
    }    // if (!measurecnt)



}

//===================================
//
// Main idle loop
//
void main()
{
    unsigned int batt_voltage;
    unsigned int new_sensitivity, old_sensitivity = 1;
    accumulate_measure_cnt = ACCUMULATE_MEASURE_CNT;
    measurecnt = accumulate_measure_cnt;
    sensitivity_flag = TRUE;
    pulse_flag = FALSE;
    beepdivider = 2048;
    beepcnt = 2048;
    lv_time = LV_TIME_DIVIDER;
    lv_flag = FALSE;
    startup_delay = STARTUP_DELAY;
    cal_cnt = 0;
    pulse_time = PULSE_TIME_DIVIDER;
    sensitivity_time = SENSITIVITY_TIME_DIVIDER;
    pulse_average_cnt = 0;

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
    ANSELA.ANSA0 = 1;     // low battery
    ANSELA.ANSA1 = 1;     // sensitivity potmeter
    ANSELA.ANSA4 = 1;     // pulse in

   // timer 1 gating
    T1GCON.TMR1GE = 1;
    T1GCON.T1GPOL = 1;             // timer 1 gate active high
    T1GCON.T1GTM =  0;
    T1GCON.T1GSPM = 0;
    T1GCON.T1GSS0 = 0;             // timer 1 gate source = comparator
    T1GCON.T1GSS1 = 1;             // timer 1 gate source = comparator
    
    T1CON.T1CKPS0 = 0;
    T1CON.T1CKPS1 = 0;             // timer 1 prescaler = 0
    T1CON.TMR1CS0 = 1;             // timer 1 clock = Fosc
    T1CON.TMR1CS1 = 0;             // timer 1 clock = Fosc
    T1CON.T1OSCEN = 0;             // timer 1 internal oscillator
    T1CON.TMR1ON =  1;             // timer 1 active
    
    TMR1H = 0;                     // initial timer values
    TMR1L = 0;
    PIR1.TMR1IF = 0;


    // DAC comparator reference
    DACCON0.DACEN = 1;             // DAC enable
    DACCON0.DACLPS = 0;            // Negative reference
    DACCON0.DACOE = 0;             // DAC output enable
    DACCON0.DACPSS0 = 0;           // VDD
    DACCON0.DACPSS1 = 0;           // VDD
 //   DACCON0.DACNSS = 0;          // GND
    DACCON1 = 24;                  // 5V / 32 * 24 = 3.75V

     // ADC input
    ADCON0.ADON = 1;               //  ADC on
    ADCON1.ADFM = 1;               // right justified
    ADCON1.ADCS0 = 0;              // Fosc / 64
    ADCON1.ADCS1 = 1;              // Fosc / 64
    ADCON1.ADCS2 = 1;              // Fosc / 64
 //   ADCON1.ADNREF = 0;           // Vref- = VSS
    ADCON1.ADPREF0 = 0;            // Vref+ = VDD
    ADCON1.ADPREF1 = 0;            // Vref+ = VDD

    // Comparator 1 init
    CM1CON0.C1POL = 0;             // comp output polarity is not inverted
    CM1CON0.C1OE = 0;              // comp output disabled
    CM1CON0.C1SP = 1;              // high speed
    CM1CON0.C1ON = 1;              // comp is enabled
    CM1CON0.C1HYS = 1;             // hysteresis enabled
    CM1CON0.C1SYNC = 0;            // comp output synchronous with timer 1

    CM1CON1.C1NCH =  1;            // C1IN1-
    CM1CON1.C1PCH0 = 1;            // DAC reference
    CM1CON1.C1PCH1 = 0;            // DAC reference

    // Fixed voltage reference = 1.024 V
    FVRCON.FVREN     = 1;         // enable
    FVRCON.CDAFVR1   = 1;         // Comparator and DAC FVR is 2x (1.024V)
    FVRCON.CDAFVR0   = 0;
    FVRCON.ADFVR1    = 0;         // ADC FVR off
    FVRCON.ADFVR0    = 0;

    // timer0 timebase  - for sound generation
    OPTION_REG.PS0      = 1;
    OPTION_REG.PS1      = 1;
    OPTION_REG.PS2      = 0;      //prescaler / 16
    OPTION_REG.PSA      = 0;
    OPTION_REG.TMR0CS   = 0;      // FOSC / 4   --> 8 MHz
    INTCON.TMR0IE       = 1;      // timer0 interrupt enable


   //--------------------

    // Startup sound
    start_sound();


    // Main loop
    while(1)
    {
      // Check pulse time
      if (pulse_flag)
      {
         // this is where TX pulse + RX processing takes place
         tx_pulse_processing();
         pulse_flag = FALSE;
      }


      // Check battery voltage
      if (lv_flag)
      {
           lv_flag = FALSE;
          //  10 bit ADC value  -> max = 0x3FF
           batt_voltage =  ADC_Read(0);
           // compare with half of the value
           if (batt_voltage < LOW_BATT_LIMIT )
           {
              low_voltage_sound();
           }
      }
      
      // Check sensitivity potmeter
      if (sensitivity_flag)
      {
           //  10 bit ADC value  -> max = 0x3FF
           // reduced to 6 bits: 0...63
           new_sensitivity = (ADC_Read(1) >> 4);
           if (new_sensitivity < 4)
           {
              new_sensitivity = 4;
           }
           if ( new_sensitivity > 63)
           {
              new_sensitivity = 63;
           }
    //       if (absvalue(old_sensitivity, new_sensitivity) > 1)
           {
              // Take backup of current sensitivity setting
              old_sensitivity = new_sensitivity;
              accumulate_measure_cnt = new_sensitivity;
           }

           sensitivity_flag = FALSE;

       } // if sensitivity flag

    }  // while(1)

} //~!
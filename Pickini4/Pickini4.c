/*
 * Project name:  PICKINI V4
     Pulse Induction
     Metal Detector
 * Copyright:
     (c) BW, 2014.
 * Configuration:
     MCU:             PIC16F1824
     Oscillator:      Internal, 32.0000 MHz
   * NOTES:
     - Motion detection 
     - Pinpoint button  switches to non-motion
     - Front end opamp DC auto offset adjust with DAC
     - Comparator slicing level set via PWM output
     
     v4.1:
     - adjust slicing level according to the sensitivity setting
     - higher sensitivity = higher slicing level ( closer to DC offset )
*/
//======================================================================
//
// constant values
//
// LARGE 1m x 1m coil
//#define LARGE_COIL


// I/O pins
#define DAC_OUT          PORTA.RA0             // DAC OUT --> opamp DC offset
#define BEEP             LATA.LATA2            // OUT  audio -- PWM3 RA2       pin 11 
#define PI_TX            LATC.LATC5            // OUT  MD pulse                pin 5
#define DETECTION_PIN    LATC.LATC3            // Digital detection output

#define PP_BUTTON        PORTA.RA3            // IN   pinpoint button      pin 4
#define COMP_IN          PORTC.RC1            // IN AN comparator 2 input     pin 9
#define PWM_OUT          PORTA.RA2            // PWM out  pin 11
#define LOW_VOLTAGE_IN   PORTA.RA1            // Low battery detection AN1      pin 12
#define SENSITIVITY_IN   PORTC.RA4            // Sensitivity potmeter  AN3      pin 3

#ifdef LARGE_COIL
#define TX_PULSE_WIDTH          200      // TX pulse width in microseconds
#else
#define TX_PULSE_WIDTH          100      // TX pulse width in microseconds
#endif

#define PULSE_TIME_DIVIDER       4       // * 512 micros

#define MAX_PULSEDIFF            64

#define ACCUMULATE_MEASURE_CNT   5       // *  PULSE_TIME_DIVIDER * 512 micros
#define LV_TIME_DIVIDER          50000   // * 512 micros -- battery voltage check - every 25 s
#define SENSITIVITY_TIME_DIVIDER 2000    // * 512 micros = 1s -- sensitivity potmeter check - every  1 s
#define LOW_BATT_LIMIT           0x200   // 10 bits ADC  - max 3FF   min 10V - 47K+10K => 1.75V =358 / 0X166
#define STARTUP_DELAY            5000     // * 512 micros -- startup settling time
#define COIL_TIMEOUT             2        // * 512 micros -- coil pulse timeout
#define CALIBRATION_DELAY        1500     // * 2 ms - delay between 2 calibration steps
#define ADC_TARGET               830      // (5V / 1024) * ADC_TARGET = DC offset
#define ADC_TOLERANCE            10
#define PULSE_ARRAY_SIZE         32

#define DC_OFFSET_MARGIN         50       // 0-255 = 0-5V



#define TRUE 1
#define FALSE 0

#define ON  1
#define OFF 0

#define IN 1
#define OUT 0


// globals
static unsigned int measurecnt;
static unsigned int accumulate_measure_cnt;
static unsigned int pulse_time;
static unsigned char pulse_flag;
static unsigned int beepdivider;
static unsigned int alternate_beepdivider;
static unsigned int alternate_beepdivider_time;
static unsigned int beepcnt;
static unsigned int lv_time;
static unsigned char lv_flag;
static unsigned char sensitivity_flag;
static unsigned int sensitivity_time;
static unsigned int  startup_delay;
static unsigned int coil_timeout;
static unsigned char DAC_value;
static unsigned int calibration_delay;
static unsigned char calibration_steps;
static unsigned char calibration_busy;
static unsigned char PWM_duty;
static unsigned char pulse_average_cnt;
static unsigned long pulse_average_array[PULSE_ARRAY_SIZE];
static unsigned long pulse_average;
static unsigned char motion_cal_cnt ;
static unsigned int measure_cal;
static unsigned char pulse_array_size;
static unsigned char pulse_array_shift;
static unsigned int DC_offset;
//static unsigned int max_diff_time;
//static unsigned int max_pulsediff;




//======================================================================
//
//  ISR
//  Handle timer interrupt as timebase for sound / timing
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

// Detector start sound
void start_sound()
{
    INTCON.GIE = 0;
    sound ( 50, 1000);    // 200 Hz
    sound ( 20, 1000);    // 500 Hz
    sound ( 10, 1000);    // 1 KHz
    // reset interrupt flags
    TMR0 = 0;
    INTCON.T0IF = 0;
    INTCON.GIE = 1;
}

// Calibration finished sound
void ready_sound()
{
    sound ( 10, 300);    // 1 KHz
    Delay_ms(200);
    sound ( 20, 300);    // 500 Hz
    Delay_ms(200);
    sound ( 10, 300);    // 1 KHz
}


// Aux: return abs value
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

//
// This is called every 2 ms
// Calibrate opamp DC offset to 4.0 V measured after TX pulse
//
void calibrate_offset()
{
     static char old_calibration_step = 0;
     char calibration_step = 0;
     calibration_delay--;
     if (!calibration_delay )
     {
        // Number of 2 ms steps between a DC offset adjustment step
        calibration_delay = CALIBRATION_DELAY;
        
        // Take DC sample and adjust offset DAC accordingly
        Delay_us (2 * TX_PULSE_WIDTH);
        DC_offset = ADC_Read(6) ;
 
        // Is calibration finished ?
        if (calibration_steps)     
        {   
          if (absvalue(DC_offset, ADC_TARGET) < ADC_TOLERANCE)
          {
             calibration_steps = 0;
          }
          else
          {
            // Increasing the DAC value reduces the DC offset
            if (DC_offset < ADC_TARGET )
            {     
                if (DAC_value > 0)
                {     
                   calibration_step = -1;    
                }
            }
            else
            {
                if (DAC_value < 31)
                {
                   calibration_step = 1;
                }
            }
            // Check for sign reversal
            // Except the first time where old_calibration_step is invalid ( 0 )
            if (calibration_steps && (old_calibration_step != 0) && (old_calibration_step != calibration_step))
            {
              // Sign reversal
              calibration_steps = 0;
              // Do not change the DAC output anymore
              calibration_step = 0;
            }  
            
             // When no sign reversal :  change DAC
             DAC_value += calibration_step;
             DACCON1 = DAC_value;

            // Take backup of this value
            old_calibration_step = calibration_step;

            if (calibration_steps)
            {
               calibration_steps--;
            }
          }
        }
        // Check if calibration is finished - calibration_steps  = 0
        else
        {

           // Save the current DAC value in EEPROM for the start of a next
           // calibration.
           EEPROM_Write(0x00,DAC_value);

           
           // Set the comparator 2 positive input 
           // DC generated by PWM2
           DC_offset >>= 2;           // 8 bit : 0-255 : 0-5V
           DC_offset = DC_offset - DC_OFFSET_MARGIN; // 1 V lower
           PWM2_Set_Duty(DC_offset);
  
           // Startup delay - stabilisation
           calibration_busy = 0;
 //          startup_delay = STARTUP_DELAY;
           ready_sound();
       }
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
      char i = 0;
  
      // start measurement

      // Disable interrupts
      INTCON.GIE = 0;

      // Reset the pulse width counter -- stop T1
      T1CON.TMR1ON = 0;
      TMR1H = 0;
      TMR1L = 0;
      PIR1.TMR1IF = 0;

      // Start the TX pulse
      PI_TX = 0;
      Delay_us(TX_PULSE_WIDTH);
      // Start T1
      T1CON.TMR1ON = 1;
      // Stop the TX pulse
      T1CON.TMR1ON = 1;

      PI_TX = 1;

      // Re-enable interrupts
      INTCON.GIE = 1;
      
      // Wait for comparator2 high only when calibration has finished
      if (!calibration_busy )
      {
         coil_timeout = COIL_TIMEOUT;    
         while(!CM2CON0.C2OUT && coil_timeout);
         if (!coil_timeout)
         {
            alternate_beepdivider_time = 5;
            alternate_beepdivider = 8;
            beepdivider = 8;
            return;
         }

          // Wait for comparator2 to go low again
          coil_timeout = COIL_TIMEOUT;
          while (CM2CON0.C2OUT && coil_timeout);
          if (!coil_timeout)
          {
             alternate_beepdivider_time = 5;
             alternate_beepdivider = 8;
             beepdivider = 8;
             return;
          }
      } // if !calibration_busy
      else
      {
         calibrate_offset();
         return;
      }

  
     // Accumulate a number of measurements for more resolution
     // e.g. 4 measurements  every 2 ms = 125 Hz measurements
     // Pulse width = 100 micros @ 32 MHz = Timer1 value: 3300
     pulsevalue += ((TMR1H<<8) | TMR1L);
     // Reset the pulse width counter
     TMR1H = 0;
     TMR1L = 0;
     PIR1.TMR1IF = 0;
     T1CON.TMR1ON = 0;

         
     // Check if measurement accumulation is complete
     measurecnt--;
     if (!measurecnt)
     {
        // Wait a number of measurements
        if (!startup_delay)
        {
           // Get the average pulse value
           pulse_average = 0;
           for (i = 0; i < pulse_array_size; i++)
           {
              pulse_average += pulse_average_array[i];
           }
           pulse_average >>= pulse_array_shift;  // divide by 
           
           // When in PinPoint mode: increase the pulse_average artificially
           // A measurement that is lower will be discarded
           // In PP mode we want a clear signal on top of the target
           if (!PP_BUTTON)
           {
              pulse_average += (accumulate_measure_cnt<<1);
           }

           //  Get absolute difference with average value
           // Only when the new pulsevalue is larger than the pulse_average,
           // we take it into account to generate an alarm sound
           // 64 = 0.5% ---> 12800 / 4 = 3200 --> 64 = 2%         
           if (pulsevalue > pulse_average)
           {
             pulsediff = pulsevalue - pulse_average; 
           }
           else
           {
              pulsediff  = 0; 
           }
   

           //  Limit pulse diff
           if (pulsediff > MAX_PULSEDIFF)
           {
             pulsediff = MAX_PULSEDIFF;
             DETECTION_PIN = 1;
           }
           else
           {
             DETECTION_PIN = 0;
           }
 
           // Calculate audio divider -- based on 500us timer interrupt;
           // alternate_beepdivider_time is used for other audio signals like
           // low power, coil error, ...
           if (!alternate_beepdivider_time)
           {
              beepdivider = (2048 - (pulsediff << 5)) + 1;
           }
           else
           {
              alternate_beepdivider_time--;
              beepdivider = alternate_beepdivider;
           }
           
           // Quicker update when frequency changes
           if (beepdivider < beepcnt) 
           {
              beepcnt = beepdivider;
           }
         } // if (!startup_delay)
       
        // Add new sample -- when pinpoint pushbutton is not active
         if (PP_BUTTON)
         {
             pulse_average_array[pulse_average_cnt%pulse_array_size] = pulsevalue;
         }

         // 
         // Restart new values
         //
         pulsevalue  = 0;
         pulse_average_cnt++;
         measurecnt = accumulate_measure_cnt;
     } // if (!measurecnt


}

//===================================
//
// main idle loop
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
    sensitivity_time = SENSITIVITY_TIME_DIVIDER;
    pulse_time = PULSE_TIME_DIVIDER;
    DAC_value = 16;
    calibration_delay = CALIBRATION_DELAY;
    calibration_steps = 31;
    calibration_busy = 1;
    PWM_duty = 0;  // 0...127
    pulse_average_cnt = 0;
    pulse_array_size = PULSE_ARRAY_SIZE;
    pulse_array_shift = 5;
    alternate_beepdivider_time = 0;

    // oscillator
    OSCCON= 0xF0;
    
    // GPIO init
    // PORT A
    TRISA.TRISA0 = OUT;  // PIN 13DAC out
    PORTA.RA0 = 1;
    WPUA.WPUA0 = 0;     // disable pullup R for DAC out !!
    TRISA.TRISA1 = IN;   // PIN 12 - low battery detection
    PORTA.RA1 = 1;
    TRISA.TRISA2 = OUT;  // PIN 11  - COMP1 out  
    PORTA.RA2 = 1;
    TRISA.TRISA3 = OUT;  // PIN 4
    PORTA.RA3 = 1;
    TRISA.TRISA4 = IN;   // PIN 3 sensitivity potmeter in 
    PORTA.RA4 = 1;
    TRISA.TRISA5 = OUT;  // PIN 2 PWM2 out
    PORTA.RA5 = 1;

    // PORT C
    TRISC.TRISC0 = IN;  // PIN 10 comparator2 + in
    PORTC.RC0 = 1;
    TRISC.TRISC1 = IN;  // PIN 9 = comparator in
    PORTC.RC1 = 1;
    TRISC.TRISC2 = IN;  // PIN 8 = ADC6 in
    PORTC.RC2 = 1;
    TRISC.TRISC3 = OUT;  // PIN 7 = RS232 out
    PORTC.RC3 = 1;
    TRISC.TRISC4 = OUT; // PIN 6 comparator 2 out
    PORTC.RC4 = 1;
    TRISC.TRISC5 = OUT; // PIN 5 pulse out
    PORTC.RC5 = 1;

    // Analog input pins
    ANSELA.ANSA0 = 1;     // DAC out
    ANSELA.ANSA1 = 1;     // low battery detection
    ANSELA.ANSA4 = 1;     // sensititity potmeter in
    ANSELC.ANSC0 = 1;     // comparator 2 + in
    ANSELC.ANSC1 = 1;     // pulse in
    ANSELC.ANSC2 = 1;     // pulse in
    
   // timer 1 gating
    T1GCON.TMR1GE = 1;
    T1GCON.T1GPOL = 1; // timer 1 gate active high
    T1GCON.T1GTM = 0;
    T1GCON.T1GSPM = 0;
    T1GCON.T1GSS1 = 1;  // comparator 2
    T1GCON.T1GSS0 = 1;  // comparator 2
    T1CON.T1CKPS0 = 0;
    T1CON.T1CKPS1 = 0;                // timer 1 prescaler = 0
    T1CON.TMR1CS0 = 1;                // timer 1 clock = Fosc
    T1CON.TMR1CS1 = 0;                // timer 1 clock = Fosc
    T1CON.T1OSCEN = 0;                // timer 1 internal oscillator
    T1CON.TMR1ON = 1;                // timer 1 active
    TMR1H = 0;          // initial timer values
    TMR1L = 0;
    PIR1.TMR1IF = 0;


    // DAC 
    DACCON0.DACEN = 1;          // DAC enable
    DACCON0.DACLPS = 0;         // Negative reference
    DACCON0.DACOE = 1;          // DAC output enable
    DACCON0.DACPSS0 = 0;        // VDD
    DACCON0.DACPSS1 = 0;        // VDD
    DACCON0.DACNSS = 0;         // GND

    // Get initial DAC value from EEPROM
    DAC_value = EEPROM_Read(0x00);
    if (DAC_value > 32)
    {
        // Invalid -> take the middle value
        DAC_value = 16;
    }


    // 5V / 32 * DAC_value
    DACCON1 = DAC_value;

     // ADC input
    ADCON0.ADON = 1;            //  ADC on
    ADCON1.ADFM = 1;            // right justified
    ADCON1.ADCS0 = 0;           // Fosc / 64
    ADCON1.ADCS1 = 1;           // Fosc / 64
    ADCON1.ADCS2 = 1;           // Fosc / 64
    ADCON1.ADNREF = 0;          // Vref- = VSS
    ADCON1.ADPREF0 = 0;         // Vref+ = VDD
    ADCON1.ADPREF1 = 0;         // Vref+ = VDD

    // Comparator 2 init
    CM2CON0.C2POL = 0;      // comp output polarity is not inverted
    CM2CON0.C2OE = 1;       // comp output enabled
    CM2CON0.C2SP = 1;       // high speed
    CM2CON0.C2ON = 1;       // comp is enabled
    CM2CON0.C2HYS = 1;      // hysteresis enabled
    CM2CON0.C2SYNC = 1;     // comp output synchronous with timer 1

    CM2CON1.C2NCH0 = 1;     // C12IN1-
    CM2CON1.C2NCH1 = 0;     // C12IN1-
    CM2CON1.C2PCH0 = 0;     // comparator + input pin
    CM2CON1.C2PCH1 = 0;     // comparator + input pin
    
    // Timer0 timebase  - for sound generation
    OPTION_REG.PS0 = 1;
    OPTION_REG.PS1 = 1;
    OPTION_REG.PS2 = 0;      //prescaler / 16
    OPTION_REG.PSA = 0;
    OPTION_REG.TMR0CS = 0;  // FOSC / 4   --> 8 MHz
    INTCON.TMR0IE = 1;          // timer0 interrupt enable
 

 
   //--------------------
   INTCON.GIE = 1;
   
   
   // Init PWM output = Comp2in+
   // PWM2 - pin 2 - on RA5
   APFCON1.CCP2SEL = 1;
  // APFCON1.P1CSEL = 1;
   PWM2_Init(250000);          // 250 kHz
   PWM2_Set_Duty(127);  
   PWM2_Start();

    // Startup sound
    start_sound();

    // Main loop
    while(1)
    {
      // Check pulse time
      if (pulse_flag)
      {
         // This is where TX pulse + RX processing takes place
         tx_pulse_processing();
         pulse_flag = FALSE;
      }


      // Check battery voltage
      if (lv_flag)
      {
           lv_flag = FALSE;
          //  10 bit ADC value  -> max = 0x3FF
           batt_voltage =  ADC_Read(1);
           // compare with half of the value
           if (batt_voltage < LOW_BATT_LIMIT )
           {
              alternate_beepdivider_time = 250;
              alternate_beepdivider_time = 4;
           }
      }
      

      // Check sensitivity potmeter
      if (sensitivity_flag)
      {
         //  10 bit ADC value  -> max = 0x3FF
         // reduced to 6 bits: 0...63
         new_sensitivity = (ADC_Read(3) >> 4);
         if (new_sensitivity < 4)
         {
            new_sensitivity = 4;
         }
         if ( new_sensitivity > 63)
         {
            new_sensitivity = 63;
         }
         
         if (absvalue(old_sensitivity, new_sensitivity) > 1)
         {
            accumulate_measure_cnt = new_sensitivity;
         }
         
         // Take backup of current sensitivity setting
         old_sensitivity = new_sensitivity;
         sensitivity_flag = FALSE;
         
         // Adjust averaging array size
         // Adjust comparator slicing value
         if (new_sensitivity < 15)
         {
             pulse_array_size = 32;
             pulse_array_shift = 5;
             PWM2_Set_Duty(DC_offset - 10);
         }
         else if (new_sensitivity < 30)
         {
             pulse_array_size = 16;
             pulse_array_shift = 4;
             PWM2_Set_Duty(DC_offset - 5);
         }
         else if (new_sensitivity < 40)
         {
             pulse_array_size = 8;
             pulse_array_shift = 3;
             PWM2_Set_Duty(DC_offset);
         }
         else 
         {
             pulse_array_size = 4;
             pulse_array_shift = 2;
             PWM2_Set_Duty(DC_offset);
         }
          
      } // if sensitivity flag
 
    }  // while(1)

} //~!
/*
 * Project name:  PICOSC
     Pulse Induction
     Metal Detector based on LC coil damping
 * Copyright:
     (c) BW, 2021.
 * Configuration:
     MCU:             PIC16F1824
     Oscillator:      Internal, 32.0000 MHz
   * NOTES:
     - Motion detection
     - Pinpoint button  switches to non-motion
     - TIMER 0 : sound generation
     - TIMER 1 : pulse timing measurement

*/
//======================================================================
//
// constant values
//


// I/O pins
#define DAC_OUT          PORTA.RA0             // DAC OUT --> opamp DC offset
#define BEEP             LATA.LATA2            // OUT  audio -- PWM3 RA2       pin 11
#define PI_TX            LATC.LATC5            // OUT  MD pulse                pin 5
#define DETECTION_PIN    LATC.LATC3            // Digital detection output     pin 7

#define PP_BUTTON        PORTA.RA3            // IN   pinpoint button          pin 4
#define COMP_IN          PORTC.RC1            // IN AN comparator 2 input      pin 9
#define PWM_OUT          PORTA.RA2            // PWM out                       pin 11
#define LOW_VOLTAGE_IN   PORTA.RA1            // Low battery detection AN1     pin 12
#define SENSITIVITY_IN   PORTC.RA4            // Sensitivity potmeter  AN3     pin 3

#define TX_PULSE_WIDTH           100      // TX pulse width in microseconds
#define PULSE_TIME_DIVIDER       4       // * 512 micros
#define ACCUMULATE_MEASURE_CNT   5       // *  PULSE_TIME_DIVIDER * 512 micros
#define LV_TIME_DIVIDER          50000   // * 512 micros -- battery voltage check - every 25 s
#define SENSITIVITY_TIME_DIVIDER 2000    // * 512 micros = 1s -- sensitivity potmeter check - every  1 s
#define LOW_BATT_LIMIT           0x200   // 10 bits ADC  - max 3FF   min 10V - 47K+10K => 1.75V = 358 / 0X166
#define STARTUP_DELAY            5000     // * 512 micros -- startup settling time
#define COIL_TIMEOUT             2        // * 512 micros -- coil pulse timeout
#define CALIBRATION_DELAY        1500     // * 2 ms - delay between 2 calibration steps
#define ADC_TARGET               830      // (5V / 1024) * ADC_TARGET = DC offset
#define ADC_TOLERANCE            10
#define PULSE_ARRAY_SIZE         32
#define PULSE_AVERAGE_DIVIDER    5
#define PULSE_SLICE_VALUE        5

// Line termination characters
#define LINE_NONE  0
#define LINE_CR    1
#define LINE_CR_LF 2


#define TRUE 1
#define FALSE 0

#define ON  1
#define OFF 0

#define IN 1
#define OUT 0


#define RS_OUTPUT


// globals
static unsigned int pulse_time;
static unsigned char pulse_flag;
static unsigned int beepdivider;
static unsigned int alternate_beepdivider;
static unsigned int alternate_beepdivider_time;
static unsigned int beepcnt;
static unsigned int lv_time;
static unsigned char lv_flag;
static unsigned char new_measurement_flag;
static unsigned char sensitivity_flag;
static unsigned int sensitivity_time;
static unsigned int  startup_delay;
static unsigned int coil_timeout;
static unsigned long pulse_average_array[PULSE_ARRAY_SIZE];
static unsigned long pulse_time_average;
static unsigned char pulse_average_cnt;
static unsigned int pulse_cnt;
static unsigned int pulse_time_measured;
static int pulse_time_diff;
static unsigned int max_pulse_cnt;
static unsigned char startup_flag;


//======================================================================
//
//  ISR
//  Handle timer interrupt as timebase for sound / timing
//
//
void interrupt(void)
{
     // comparator interrupt
     if (PIR2.C2IF)
     {
        pulse_cnt++;
        if (pulse_cnt == max_pulse_cnt)
        {
           // Stop timer1
           T1CON.TMR1ON = 0;
           pulse_time_measured = (TMR1H<<8) | TMR1L;
           new_measurement_flag = TRUE;
        }
        PIR2.C2IF = 0;
     }
     
     
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
// Send a single character over RS232
//
void sendchar( char c)
{
#ifdef RS_OUTPUT
      while (!UART1_Tx_Idle())
      {
         Delay_us(100);
      }
      UART1_Write(c);

#endif
}

//======================================================================
// Send a 16 bit number over RS232
//
void sendhex (unsigned long hexnumber, unsigned char cr )
{
#ifdef RS_OUTPUT
      int nibble = 0;
      const char hexnr[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

      for (nibble = 0; nibble < 6; nibble++)
      {
          sendchar(hexnr[(hexnumber&0xF00000)>>20]);
          hexnumber<<=4;
      }
      if (cr == LINE_CR_LF )
      {
       sendchar('\r');
       sendchar('\n');
      }
      else if (cr == LINE_CR)
      {
        sendchar('\r');
      }

#endif
}

//======================================================================
// Send a string over RS232
//
void sendstring (char* string, unsigned char cr)
{
#ifdef RS_OUTPUT
      int i = 0;
      char c;
      while (c=string[i++])
      {
              sendchar(c);
      }


      if (cr == LINE_CR_LF )
      {
       sendchar('\r');
       sendchar('\n');
      }
      else if (cr == LINE_CR)
      {
        sendchar('\r');
      }
#endif
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


//===================================
//
// TX pulse + measure return pulse
//
void tx_pulse()
{
      // Reset the pulse width counter
      TMR1H = 0;
      TMR1L = 0;
      PIR1.TMR1IF = 0;

      // Start the TX pulse
      PI_TX = 0;
      Delay_us(TX_PULSE_WIDTH);
      // Stop the TX pulse
      PI_TX = 1;
      
      // Start the timer
      T1CON.TMR1ON = 1;                // timer 1 active
      
      // Determine the max number of oscillations
      if (startup_flag && pulse_cnt)
      {
         startup_flag = FALSE;
         max_pulse_cnt = pulse_cnt - 3;
      }
      pulse_cnt = 0;

}

//===================================
//
// main idle loop
//
void main()
{
    unsigned int batt_voltage;
    unsigned int new_sensitivity, old_sensitivity = 1;
    int i = 0;
    sensitivity_flag = TRUE;
    pulse_flag = FALSE;
    new_measurement_flag = FALSE;
    beepdivider = 2048;
    beepcnt = 2048;
    lv_time = LV_TIME_DIVIDER;
    lv_flag = FALSE;
    startup_delay = STARTUP_DELAY;
    sensitivity_time = SENSITIVITY_TIME_DIVIDER;
    pulse_time = PULSE_TIME_DIVIDER;
    alternate_beepdivider_time = 0;
    startup_flag = FALSE;

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
    TRISC.TRISC4 = OUT; // PIN 6 TX UART
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

   // timer 1 = pulse timing
    T1CON.T1CKPS0 = 0;
    T1CON.T1CKPS1 = 0;               // timer 1 prescaler = 0
    T1CON.TMR1CS0 = 1;               // timer 1 clock = Fosc
    T1CON.TMR1CS1 = 0;               // timer 1 clock = Fosc
    T1CON.T1OSCEN = 0;               // timer 1 internal oscillator
    T1CON.TMR1ON = 0;                // timer 1 active
    TMR1H = 0;                       // initial timer values
    TMR1L = 0;
    PIR1.TMR1IF = 0;

    // DAC = comparator positive input  0..5V
    DACCON0.DACEN = 1;          // DAC enable
    DACCON0.DACLPS = 0;         // Negative reference
    DACCON0.DACOE = 0;          // DAC output enable
    DACCON0.DACPSS0 = 0;        // VDD
    DACCON0.DACPSS1 = 0;        // VDD
    DACCON0.DACNSS = 0;         // GND
    DACCON1 = PULSE_SLICE_VALUE;

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
    CM2CON1.C2PCH0 = 1;     // comparator + DAC
    CM2CON1.C2PCH1 = 0;     // comparator + DAC
    CM2CON1.C2INTP = 1;     // interrupt on positive edge
    PIE2.C2IE = 1;          // enable comparator interrupt
    PIR2.C2IF = 0;          // comp interrupt flag

    // Timer0 timebase  - for sound generation
    OPTION_REG.PS0 = 1;
    OPTION_REG.PS1 = 1;
    OPTION_REG.PS2 = 0;      //prescaler / 16
    OPTION_REG.PSA = 0;
    OPTION_REG.TMR0CS = 0;  // FOSC / 4   --> 8 MHz
    INTCON.TMR0IE = 1;          // timer0 interrupt enable

#ifdef RS_OUTPUT
    // RX pin on RC5
    APFCON0.RXDTSEL = 0;         // RX = pin 5 = RC5
    // TX pin on RC4
    APFCON0.TXCKSEL = 0;        // TX = pin 6 = RC4
    // Initialize UART
    UART1_Init(9600);
#endif


   //--------------------
   INTCON.GIE = 1;


    // Startup sound
    start_sound();

    // Setup stage
    // Measure amount of oscillation pulses
    while (startup_delay);
    startup_flag = TRUE;

    // Detector ready sound
    ready_sound();
    
    // Main loop
    while(1)
    {
      // Check pulse time
      if (pulse_flag)
      {
         // TX pulse
         tx_pulse();
         pulse_flag = FALSE;
      }

      // New measurement available
      if (new_measurement_flag)
      {
         new_measurement_flag = FALSE;
         pulse_average_array[pulse_average_cnt%PULSE_ARRAY_SIZE] = pulse_time_measured;
         // Get the average pulse value
         pulse_time_average = 0;
         for (i = 0; i < PULSE_ARRAY_SIZE; i++)
         {
              pulse_time_average += pulse_average_array[i];
         }
         pulse_time_average >>= PULSE_AVERAGE_DIVIDER;
         pulse_time_diff = pulse_time_measured - pulse_time_average;
         // Evalate pulse_time_diff pos or neg --> sensitivity --> 2 tone sound
         // XXX
         pulse_average_cnt++;
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

      } // if sensitivity flag

    }  // while(1)

} //~!
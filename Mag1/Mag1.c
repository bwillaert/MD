/*
 * Project name:  Fluxgate Magnetometer
     Fluxgate gradient magnetometer based on 2 FGM3 sensors
     www.fgsensors.com
 * Copyright:
     (c) BW, 2017.
 * Configuration:
     MCU:             PIC16F1824
     Oscillator:      Internal, 32.0000 MHz

 */
//======================================================================
//
// constant values
//
#define RS_OUTPUT       // UART TX/RX for debugging/control
// RS232 TX pin          RC4 p6
// RS232 RX pin          RC5 p5

// I/O pins
#define STATUS_LED       LATC.LATC3            // LED OUT     pin 7
#define BEEP             LATA.LATA2            // OUT  audio  pin 11
#define METER            LATA.LATA5            // PWM2 RA5    pin 2
#define CAL_BUTTON       PORTA.RA3             // IN   calibration button       pin 4
#define SENSITIVITY_HIGH PORTA.RA0             // IN pin 13 : low = sensitivity low - high = default (no switch) sensitivity high
#define POTMETER_IN      PORTC.RA4             // Potmeter  AN3     pin 3
#define BATT_VOLTAGE     PORTA.RA1             // Battery voltage measurement AN1 pin 12

// The 2 sensor signals are connected to CMP1 - inputs 1 and 2
// C12IN1-               RC1 p9 - sensor 1
// C12IN2-               RC2 p8 - sensor 2
// C1OUT                 RA2 p11 - not used, p11 = audio out

#define SAMPLE_ARRAY_SIZE_SHIFT 5            // 1 << SAMPLE_ARRAY_SIZE_SHIFT = Nr of samples into the running average array
#define SAMPLE_ARRAY_SIZE  (1 << SAMPLE_ARRAY_SIZE_SHIFT)            // 1 << SAMPLE_ARRAY_SIZE_SHIFT = Nr of samples into the running average array
#define SAMPLE_ACCU_TIME        100            // ms
#define METER_MIDDLE_VALUE      127
#define LOW_BATT_DELAY          1000           // * 50 ms
#define LOW_BATT_TRESHOLD       200            // 4V


#define TRUE 1
#define FALSE 0

#define OFF 0
#define ON  1

#define IN  1
#define OUT 0

// Line termination characters
#define LINE_NONE  0
#define LINE_CR    1
#define LINE_CR_LF 2



static unsigned char  sample_array_pointer;
static unsigned int   sample1, sample2;
static int            sample_array[SAMPLE_ARRAY_SIZE];
static unsigned char  meter_value;
static unsigned int   beepcnt, beepdivider;

//======================================================================
//
//  ISR
//
void interrupt(void)
{
    // Comparator1 output interrupt
     if (PIR2.C1IF)
     {
        sample1++;
        // Reset interrupt flag
        PIR2.C1IF = 0;
     }
   // Comparator2 output interrupt
     if (PIR2.C2IF)
     {
        sample2++;
        // Reset interrupt flag
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
    sound ( 50, 1000);    // 200 Hz
    Delay_ms(100);
    sound ( 20, 1000);    // 500 Hz
    Delay_ms(100);
    sound ( 10, 1000);    // 1 KHz
}

void low_batt_sound()
{
    sound ( 20, 1000);    // 500 Hz
    Delay_ms(100);
    sound ( 50, 1000);    // 200 Hz

}

void sensor_error_sound()
{
    sound ( 50, 1000);    // 200 Hz
}

//======================================================================
// Calc abs value of difference between 2 values
//
unsigned int absvalue(unsigned int a, unsigned int b)
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
// Write unsigned int value = 2 bytes to EEPROM
//
void write_EEPROM(unsigned int value, unsigned char address)
{
     // Each unsigned int storage entry is 2 bytes
     address <<= 1;
     EEPROM_Write(address, value&0xFF);
     Delay_ms(20);
     EEPROM_Write(address+1, (value>>8)&0xFF);
     Delay_ms(20);
}

//======================================================================
// Read unsigned int value = 2 bytes from EEPROM
//
unsigned int read_EEPROM(unsigned char address)
{
     // Each unsigned int storage entry is 2 bytes
     unsigned int value;
     address <<= 1;
     value = (EEPROM_Read(address+1) << 8) |  EEPROM_Read(address);
     return value;
}



//===================================
//
// main idle loop
//
void main()
{
   // char RXchar;
    int sample_diff, sample_diff_deviation;
    long sample_diff_sum;
    int sample_diff_average = METER_MIDDLE_VALUE;
    unsigned int audio_treshold, battery_voltage;
    unsigned int new_audio_treshold, old_audio_treshold = 0;
    unsigned int low_batt_count = LOW_BATT_DELAY;
    unsigned char i;
    beepdivider = 0;
    beepcnt = 0;

    // oscillator
    OSCCON= 0xF0;
    
    // GPIO init
    // PORT A
    TRISA.TRISA0 = IN;   // PIN 13 = sensitivity low/high
    PORTA.RA0 = 1;
    TRISA.TRISA1 = IN;   // PIN 12 = low battery
    PORTA.RA1 = 1;
    TRISA.TRISA2 = OUT;  // PIN 11 = audio out
    PORTA.RA2 = 1;
    TRISA.TRISA3 = OUT;  // PIN 4 = CAL button
    PORTA.RA3 = 1;
    TRISA.TRISA4 = IN;   // PIN 3 potmeter in
    PORTA.RA4 = 1;
    TRISA.TRISA5 = OUT;  // PIN 2  PWM2 out meter
    PORTA.RA5 = 1;

    // PORT C
    TRISC.TRISC0 = IN;  // PIN 10
    PORTC.RC0 = 1;
    TRISC.TRISC1 = IN;  // PIN 9 = measurement sensor input ( bottom ) C12IN1-
    PORTC.RC1 = 1;
    TRISC.TRISC2 = IN;  // PIN 8 = reference sensor input ( top )  C12IN2-
    PORTC.RC2 = 1;
    TRISC.TRISC3 = OUT;  // PIN 7
    PORTC.RC3 = 1;
    TRISC.TRISC4 = OUT; // PIN 6
    PORTC.RC4 = 1;
    TRISC.TRISC5 = OUT; // PIN 5
    PORTC.RC5 = 1;

    // Analog input pins
    ANSELA.ANSA0 = 0;     //
    ANSELA.ANSA1 = 1;     //
    ANSELA.ANSA4 = 1;     //  potmeter input
    ANSELC.ANSC0 = 0;     //
    ANSELC.ANSC1 = 1;     //  RC1 = C12IN1-
    ANSELC.ANSC2 = 1;     //  RC2 = C12IN2-
    
    // Comparator 1 init
    CM1CON0.C1POL = 1;      // comp output polarity is inverted
    CM1CON0.C1OE = 0;       // comp output disabled - pin 11 RA2
    CM1CON0.C1SP = 1;       // high speed
    CM1CON0.C1ON = 1;       // comp is enabled
    CM1CON0.C1HYS = 1;      // hysteresis enabled
    CM1CON0.C1SYNC = 0;     // comp output synchronous with timer 1
    CM1CON1.C1PCH0 = 0;     // FVR reference
    CM1CON1.C1PCH1 = 1;     // FVR reference
    CM1CON1.C1NCH0 = 1;     // C12IN1-
    CM1CON1.C1NCH1 = 0;     // C12IN1-
    CM1CON1.C1INTN = 0;     //
    CM1CON1.C1INTP = 1;     // C1IF flag set on rising edge

    // Comparator 2 init
    CM2CON0.C2POL = 1;      // comp output polarity is inverted
    CM2CON0.C2OE = 0;       // comp output disabled
    CM2CON0.C2SP = 1;       // high speed
    CM2CON0.C2ON = 1;       // comp is enabled
    CM2CON0.C2HYS = 1;      // hysteresis enabled
    CM2CON0.C2SYNC = 0;     // comp output synchronous with timer 1
    CM2CON1.C2PCH0 = 0;     // FVR reference
    CM2CON1.C2PCH1 = 1;     // FVR reference
    CM2CON1.C2NCH0 = 0;     // C12IN2-
    CM2CON1.C2NCH1 = 1;     // C12IN2-
    CM2CON1.C2INTN = 0;     //
    CM2CON1.C2INTP = 1;     // C2IF flag set on rising edge

    // Fixed voltage reference = 1.024 V
    FVRCON.FVREN     = 1;      // enable
    FVRCON.CDAFVR1   = 1;      // Comparator and DAC FVR is 2x (1.024V)
    FVRCON.CDAFVR0   = 0;
    FVRCON.ADFVR1    = 0;      // ADC FVR off
    FVRCON.ADFVR0    = 0;

    // ADC input
    ADCON0.ADON = 1;            //  ADC on
    ADCON1.ADFM = 1;            // right justified
    ADCON1.ADCS0 = 0;           // Fosc / 64
    ADCON1.ADCS1 = 1;           // Fosc / 64
    ADCON1.ADCS2 = 1;           // Fosc / 64
    ADCON1.ADNREF = 0;          // Vref- = VSS
    ADCON1.ADPREF0 = 0;         // Vref+ = VDD
    ADCON1.ADPREF1 = 0;         // Vref+ = VDD

    // Select PWM2 output pin
    APFCON1.CCP2SEL = 1;        // CCP2 function is on RA5

    // Enable comparator interrupts
    PIE2.C1IE = 0;
    PIE2.C2IE = 0;

   // Timer0 timebase  - for sound generation
    OPTION_REG.PS0 = 1;
    OPTION_REG.PS1 = 1;
    OPTION_REG.PS2 = 0;      // prescaler / 16
    OPTION_REG.PSA = 0;
    OPTION_REG.TMR0CS = 0;   // FOSC / 4   --> 8 MHz
    INTCON.TMR0IE = 1;       // timer0 interrupt enable

    // Enable global interrupt
    INTCON.PEIE = 1;
    INTCON.GIE = 1;

    // Initialize PWM output for meter
    PWM2_Init(4000);          // 1 kHz
    PWM2_Start();


#ifdef RS_OUTPUT
    // RX pin on RC5
    //APFCON0.RXDTSEL = 0;         // RX = pin 5 = RC5
    // TX pin on RC4
    APFCON0.TXCKSEL = 0;           // TX = pin 6 = RC4

    // Initialize UART
    UART1_Init(115200);
#endif

    // Startup values
    sample_array_pointer = 0;
    STATUS_LED = OFF;

    // Get the battery voltage ( V/2 > 4V )
    // Voltage divider: 8V = 5V       +--15K--10K--Gnd
    battery_voltage = ADC_Read(1) >> 2;
    PWM2_Set_Duty(battery_voltage);

    // Delay sound
    for (i = 0; i < 5; ++i)
    {
        Delay_ms(200);
        sound ( 10, 2000);    // 1 KHz
     }

    // Startup sound
    start_sound();

    PWM2_Set_Duty(METER_MIDDLE_VALUE);


#ifdef RS_OUTPUT
    // Logo
    //sendstring(STR_WELCOME, LINE_CR_LF);
#endif


     // Main idle loop
     while(1)
     {
        // Take a sample over SAMPLE_ACCU_TIME ms
        PIE2.C1IE = 0;
        PIE2.C2IE = 0;
        sample1 = 0;
        sample2 = 0;
        PIE2.C1IE = 1;
        PIE2.C2IE = 1;
        Delay_ms(SAMPLE_ACCU_TIME);
        PIE2.C1IE = 0;
        PIE2.C2IE = 0;

        // Status LED indicator - sensor error sound
        if (!sample1 || !sample2)
        {
           sensor_error_sound();
        }
        
        #ifdef RS_OUTPUT
        sendhex ((unsigned long)sample1, LINE_NONE);
        sendchar(',');
        sendchar(' ');
        sendhex ((unsigned long)sample2, LINE_NONE);
        sendchar(',');
        sendchar(' ');
        sendhex ((unsigned long)audio_treshold, LINE_CR_LF);
 #endif

        
        // Sample difference = int = sample1 - sample2
        sample_diff = (int)(sample1 - sample2);
        // If sensitivity switch is low : sample diff / 4  ( 0 ... 1023 instead of 0 ... 255)
        if (!SENSITIVITY_HIGH)
        {
           sample_diff >>= 2;
        }
        sample_array[sample_array_pointer % SAMPLE_ARRAY_SIZE] = sample_diff;
        sample_array_pointer++;
        
        // Check calibration pushbutton pushed
        if (!CAL_BUTTON)
        {
          // Take a new sample diff average = this actual sample
          sample_diff_sum = 0;
          for (i = 0; i < SAMPLE_ARRAY_SIZE; i++)
          {
             sample_array[i] = sample_diff;
             sample_diff_sum += sample_array[i];
          }
          sample_diff_average = sample_diff_sum >> SAMPLE_ARRAY_SIZE_SHIFT;
        }

        // This is the deviation of (the average difference between the 2 sensors) and
        // (the new difference value):
        sample_diff_deviation = sample_diff - sample_diff_average;
        
         // Adjust meter output PWM2
        if (sample_diff_deviation < -127)
        {
           sample_diff_deviation = -127;
        }
        else if (sample_diff_deviation > 128)
        {
           sample_diff_deviation = 128;
        }

        // Show the deviation from the middle value
        meter_value = (unsigned char) ((int)METER_MIDDLE_VALUE + sample_diff_deviation);
        PWM2_Set_Duty(meter_value);

        // Get potmeter value: 0...127 = audio treshold
        // 10 bit ADC value  -> max = 0x3FF
        // reduced to 8 bits: 0...255
        new_audio_treshold = (ADC_Read(3) >> 3);
        if (absvalue(new_audio_treshold, old_audio_treshold) > 1)
        {
           audio_treshold = new_audio_treshold;
        }
        old_audio_treshold = new_audio_treshold;
        if (audio_treshold < 2 )
        {
           audio_treshold = 2;
        }
        if (audio_treshold > 125 )
        {
           audio_treshold = 125;
        }

        // Audio treshold: minimum = very sensitive
        if (abs(sample_diff_deviation) > audio_treshold)
        {
           // Beep
           if (sample_diff_deviation > 0)
           {
               // 1 kHz
               beepcnt = 1;
               beepdivider = 1;
               STATUS_LED = ON;
           }
           else
           {
               // 500 Hz
               beepdivider = 2;
               beepcnt = 2;
               STATUS_LED = ON;
           }
        }
        else
        {
           // No beep
           beepcnt = 0;
           beepdivider = 0;
           STATUS_LED = OFF;
        }

        // Get the battery voltage ( V/2 > 4V )
        battery_voltage = ADC_Read(1) >> 2;  // min value = 4/5 = 200
        if (battery_voltage < LOW_BATT_TRESHOLD)
        {
           if (low_batt_count)
           {
              low_batt_count--;
           }
           else
           {
               low_batt_count = LOW_BATT_DELAY;
               low_batt_sound();
           }
        }
        else // battery voltage OK
        {
            low_batt_count = LOW_BATT_DELAY;
        }

    }  // while(1)

} //~!
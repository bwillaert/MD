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
#define RS_OUTPUT

// I/O pins

#define STATUS_LED       LATC.LATC5            // LED OUT     pin 5


// The 2 sensor signals are connected to CMP1 - inputs 1 and 2
// C12IN1-               RC1 p9
// C12IN2-               RC2 p8
// C1OUT                 RA2 p11
// RS232 TX pin          RC4 p6
// RS232 RX pin          RC5 p5

#define SAMPLE_ACCU_PERIODS     50             // Nr of output signal periods accumulated into 1 sample  - for 20  kHZ: *50 micros
#define SAMPLE_ARRAY_SIZE       64             // Nr of samples into the running average array

#define CMP_TIMEOUT             1000           // microseconds

#define CAL_BUTTON              PORTA.RA3            // IN   calibration button      pin 4
// p 11 = audio / PWM out
#define BEEP             LATA.LATA2            // OUT  audio -- PWM3 RA2       pin 11

// State machine states
#define SM_SENS12_INIT     0
#define SM_SENS12_START    1
#define SM_SENS12_COUNT    2
#define SM_SENS12_CALC     3

#define SM_CALIBRATION    10
#define SM_SENS_ERROR     20

#define SM_SENSOR_1       1
#define SM_SENSOR_2       2

#define TRUE 1
#define FALSE 0

#define OFF 0
#define ON  1

#define IN 1
#define OUT 0

// Line termination characters
#define LINE_NONE  0
#define LINE_CR    1
#define LINE_CR_LF 2


static unsigned char  sm_state;
static unsigned char  sm_sensor;
static unsigned int   sm_timeout;
static unsigned char  sample_period_cnt;
static unsigned int   sample1, sample_min1, sample_max1, sample1_calculated, sample1_diff;
static unsigned int   sample2, sample_min2, sample_max2;
static unsigned int   sample1_diff_correction;
static unsigned int   pwm_audio;

//static unsigned char  sample_array_pointer; //_1;
//static unsigned char  sample_array_pointer_2;
//static unsigned int   sample_diff_array[SAMPLE_ARRAY_SIZE];
//static unsigned long  sample_diff_average;

static unsigned char  sm_calibration;
static unsigned char  sm_calibration_show;

const char STR_ENTER_CALIBRATION[] = "Entering calibration mode";
const char STR_EXIT_CALIBRATION[] =  "Exit calibration mode";
const char STR_WELCOME[] = "=====MAGPIC V1=====";
const char STR_CAL_BUTTON[] = "CAL BUTTON:";

//======================================================================
//
//  ISR
//  Handle timer1 interrupt as timebase
//
//
#if 0
void interrupt(void)
{

     // Timer1 interrupt on each overflow from FFFF to 0000
     if (PIR1.TMR1IF)
     {


        // Reset interrupt flag
        PIR1.TMR1IF = 0;
      }
}
#endif

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

//======================================================================
// Show the calibration values
//
void send_calibration_values()
{
      sendhex (sample_min1, LINE_NONE);
      sendchar(',');
      sendchar(' ');
      sendhex (sample_min2, LINE_NONE);
      sendchar(',');
      sendchar(' ');
      sendhex (sample_max1, LINE_NONE);
      sendchar(',');
      sendchar(' ');
      sendhex (sample_max2, LINE_CR_LF);
}

//======================================================================
// Calculate the theoretical measured value
// based on min and max values and the ref value ( high sensor ) = sample2
//
unsigned long calculate_sample()
{
   unsigned long val1_l;
   unsigned int val1, val2;
   unsigned int val3;
   unsigned int val4;
   unsigned long val5;
   unsigned int val6;
   unsigned long result;
   if (sample1 > sample_min1)
   {
       val1 = sample2 - sample_min2;
       val1_l = (unsigned long)val1 << 10;
       val2 = sample_max2 - sample_min2;
       val3 = val1_l/val2;
       val4 = sample_max1 - sample_min1;
       val5 = (unsigned long)val3*(unsigned long)val4;
       val6 = val5 >> 10;
       result = sample_min1 + val6;
   }
   else
   {
       result = 0;
   }
   return result;
}

//===================================
//
// main idle loop
//
void main()
{
    unsigned char i;
    char RXchar;

    // oscillator
    OSCCON= 0xF0;
    
    // GPIO init
    // PORT A
    TRISA.TRISA0 = IN;  // PIN 13 = potmeter
    PORTA.RA0 = 1;
    TRISA.TRISA1 = IN;   // PIN 12 = low battery
    PORTA.RA1 = 1;
    TRISA.TRISA2 = OUT;  // PIN 11 = audio out
    PORTA.RA2 = 1;
    TRISA.TRISA3 = IN;  // PIN 4 = CAL button
    PORTA.RA3 = 1;
    TRISA.TRISA4 = OUT;   // PIN 3 sensitivity potmeter in
    PORTA.RA4 = 1;
    TRISA.TRISA5 = OUT;  // PIN 2
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
    ANSELA.ANSA1 = 1;     // frequency potmeter in
    ANSELA.ANSA4 = 1;     //
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
    CM1CON1.C1INTN = 1;     // C1IF flag set on negative edge

    // Fixed voltage reference = 1.024 V
    FVRCON.FVREN     = 1;      // enable
    FVRCON.CDAFVR1   = 1;      // Comparator and DAC FVR is 2x (1.024V)
    FVRCON.CDAFVR0   = 0;
    FVRCON.ADFVR1    = 0;      // ADC FVR off
    FVRCON.ADFVR0    = 0;

    // Timer 1 gating enabled by CMP1 output
    T1GCON.TMR1GE = 0;                // timer 1 gate control
    T1GCON.T1GPOL = 1;                // timer 1 gate active high
    T1GCON.T1GTM = 0;
    T1GCON.T1GSPM = 0;
    T1GCON.T1GSS1 = 1;                // comparator 1
    T1GCON.T1GSS0 = 0;                // comparator 1
    T1CON.T1CKPS0 = 0;
    T1CON.T1CKPS1 = 0;                // timer 1 prescaler = 0
    T1CON.TMR1CS0 = 1;                // timer 1 clock = Fosc
    T1CON.TMR1CS1 = 0;                // timer 1 clock = Fosc
    T1CON.T1OSCEN = 0;                // timer 1 internal oscillator
    T1CON.TMR1ON = 0;                 // timer 1 active
    TMR1H = 0;                        // initial timer values
    TMR1L = 0;
    PIR1.TMR1IF = 0;

  /*
    // timer0 timebase  - for sound generation
    OPTION_REG.PS0 = 0; //1;
    OPTION_REG.PS1 = 0; //1;
    OPTION_REG.PS2 = 0;      //prescaler / 16 --> /2 --> TMR0 overflow = 64 micros
    OPTION_REG.PSA = 0;
    OPTION_REG.TMR0CS = 0;  // FOSC / 4   --> 8 MHz
    INTCON.TMR0IE = 1;          // timer0 interrupt enable
   */
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
    sm_state = SM_SENS12_INIT;
    sm_sensor = SM_SENSOR_2;
//    sample_array_pointer = 0;
    sm_calibration = FALSE;
    sm_calibration_show = FALSE;
    sample1_diff_correction = 0;
    pwm_audio = 0;
//   sample_array_pointer_2 = 0;

    // Enable timer1 interrupt
    INTCON.PEIE = 1;
//    PIE1.TMR1IE = 1;                // timer 1 interrupt enabled
    INTCON.GIE = 1;

#ifdef RS_OUTPUT
    // RX pin on RC5
    APFCON0.RXDTSEL = 0;         // RX = pin 5 RC5
    // TX pin on RC4
    APFCON0.TXCKSEL = 0;        // pin 6 = RC4

    // Initialize UART
    UART1_Init(9600);


#endif


    // Startup sound
    start_sound();
    
    // Delay
    Delay_ms(1000);


    // Init PWM3 audio - pin 11
    PWM3_Init(4000);          // 1 kHz
    PWM3_Set_Duty(pwm_audio);
    PWM3_Start();



#ifdef RS_OUTPUT
    // Logo
    sendstring(STR_WELCOME, LINE_CR_LF);
#endif




    // Restore calibration values
    sample_min1 = read_EEPROM(0);
    sample_max1 = read_EEPROM(1);
    sample_min2 = read_EEPROM(2);
    sample_max2 = read_EEPROM(3);

     //  LED blink
     STATUS_LED = ON;
     Delay_ms(500);
     STATUS_LED = OFF;

     // Main idle loop
     while(1)
     {

        // State machine
        switch (sm_state)
        {
               case SM_SENS12_INIT:
                    // Start new measurement  - toggle sensor selection
                    if (sm_sensor == SM_SENSOR_1)
                    {
                       sm_sensor = SM_SENSOR_2;
                       // Select sensor input C1IN2-
                       CM1CON1.C1NCH0 = 0;     // C12IN1-
                       CM1CON1.C1NCH1 = 1;     // C12IN1-  -- reference
                    }
                    else
                    {
                       sm_sensor = SM_SENSOR_1;
                       // Select sensor input C1IN1-
                       CM1CON1.C1NCH0 = 1;     // C12IN1-
                       CM1CON1.C1NCH1 = 0;     // C12IN1-  -- measurement
                    }
                    // Timer1 gate control
                    T1CON.TMR1ON = 0;     // Timer 1 off
                    TMR1H = 0;            // Initial timer values
                    TMR1L = 0;
                    sm_timeout = CMP_TIMEOUT;
                    sm_state = SM_SENS12_START;
                    sample_period_cnt = 0;
                   break;
                    
               case SM_SENS12_START:
                    // Wait for a falling edge of the comparator
                    PIR2.C1IF = 0;
                    while (!PIR2.C1IF);
                    // Start the timer and count periods
                    T1CON.TMR1ON = 1;
                    sample_period_cnt =  SAMPLE_ACCU_PERIODS;
                    while (sample_period_cnt--)
                    {
                        PIR2.C1IF = 0;
                        // Wait for the next falling edge
                        while (!PIR2.C1IF);
                    }
                    // Stop timer1
                    T1CON.TMR1ON = 0;
                    sm_state = SM_SENS12_CALC;
                    break;
                    
              case SM_SENS12_CALC:
                   // We have a new sample
                  if (sm_sensor == SM_SENSOR_1)
                   {
                      sample1 = ((TMR1H<<8) | TMR1L);   // Measurement  = low sensor
                   }
                   else
                   {
                      sample2 = ((TMR1H<<8) | TMR1L);  // Reference = high sensor
                   }

#ifdef RS_OUTPUT
        // Send results over RS232
        if (sm_sensor == SM_SENSOR_2)
        { 
        
            // Predict the measured value - based on the ref values
            // Ref = sample2  Meas = sample1
            sample1_calculated = calculate_sample();

            // Send the results
            if (!sm_calibration && !sm_calibration_show) // This should be bit values ==> if no flags
            {
              sendhex (sample2, LINE_NONE);
              sendchar(',');
              sendchar(' ');
              sendhex (sample1, LINE_NONE);
              sendchar(',');
              sendchar(' ');
              sendhex (sample1_calculated, LINE_NONE);
              sendchar(',');
              sendchar(' ');
              sample1_diff = absvalue(sample1, sample1_calculated);
              sample1_diff = absvalue(sample1_diff_correction, sample1_diff);
              
              // Final - corrected sample diff
              sendhex(sample1_diff, LINE_CR_LF);
              
              // Audio
              if (sample1_diff > 512)
              {
                 pwm_audio = 512;
              }
              else
              {
                 pwm_audio = sample1_diff;
              }
              // Audio signal- max 128 = 50%
              //PWM3_Set_Duty(pwm_audio>>2);
              PWM3_Set_Duty(pwm_audio>>5);

              
            }
            else if (sm_calibration)
            {
                // Sample2 = reference - sample12 = measurement
                // Take a synchronous min and max of both sensors
                // This enables to correlate the values in between
                if (sample2 < sample_min2)
                {
                   sample_min2 = sample2;
                   sample_min1 = sample1;
                }
                if (sample2 > sample_max2)
                {
                   sample_max2 = sample2;
                   sample_max1 = sample1;
                }
                // Show calibration values
                send_calibration_values();

            }

            
        }
#endif

                   sm_state =  SM_SENS12_INIT;
                   break;

              case SM_SENS_ERROR:
                   // Comparator timeout
                   // Sensor error or not connected
                   // Error sound..
                   sm_state = SM_SENS12_INIT;
                   break;

              default:  // we should never get here
                    sm_state = SM_SENS12_INIT;
                    break;

        } // state machine switch ------------------------

        // Do other stuff   --------------
        
        // Sensitivity potmeter
        // ADC 0 - 512
        // sensitivity = 512 - ADC + 20 --- 20 = max diff for full PWM audio range
        
        // Audio PWM 0 - 512
        // abs(sample1_diff, sample1_diff_correction)
        
        // Battery voltage measurement
        // control red/green LED

        // Check calibration pushbutton
        if (!CAL_BUTTON)
        {
           sample1_diff_correction = sample1_diff;
           sendstring(STR_CAL_BUTTON, LINE_NONE);
           sendhex (sample1_diff_correction, LINE_CR_LF);
           Delay_ms(1000);

        }

        // Character received
        // If data is ready, read it:
        if (UART1_Data_Ready())
        {
           RXchar = UART1_Read();
           switch (RXchar)
           {
              case 'x':
              case 'X':
                   if (!sm_calibration)
                   {
                      sendstring(STR_ENTER_CALIBRATION, LINE_CR_LF);
                      sm_calibration = TRUE;
                      sample_min1 = sample_min2 = ~0;
                      sample_max1 = sample_max2 = 0;
                   }
                   else
                   {
                      sendstring(STR_EXIT_CALIBRATION, LINE_CR_LF);
                      sm_calibration = FALSE;
                      
                      // Write calibration values into EEPROM
                      write_EEPROM(sample_min1,0);
                      write_EEPROM(sample_max1,1);
                      write_EEPROM(sample_min2,2);
                      write_EEPROM(sample_max2,3);

                   }
                   break;

               case 'y':
               case 'Y':
                    if (!sm_calibration_show)
                    {
                       // Show calibration values
                       send_calibration_values();
                       sm_calibration_show = TRUE;
                    }
                    else
                    {
                       sm_calibration_show = FALSE;
                    }
                   break;

              default:
                   //sendchar(RXchar);
                   break;
           }
        }



    }  // while(1)

} //~!
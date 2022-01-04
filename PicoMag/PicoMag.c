#line 1 "C:/Users/b/Documents/GitHub/MD/Mag1/Mag1.c"
#line 58 "C:/Users/b/Documents/GitHub/MD/Mag1/Mag1.c"
static unsigned char sample_array_pointer;
static unsigned int sample1, sample2;
static int sample_array[ (1 << 5 ) ];
static unsigned char meter_value;
static unsigned int beepcnt, beepdivider;





void interrupt(void)
{

 if (PIR2.C1IF)
 {
 sample1++;

 PIR2.C1IF = 0;
 }

 if (PIR2.C2IF)
 {
 sample2++;

 PIR2.C2IF = 0;
 }


 if (INTCON.TMR0IF)
 {

 if (beepcnt)
 {
 beepcnt--;
 {
 if (!beepcnt)
 {
  LATA.LATA2  = ! LATA.LATA2 ;
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








void sound (unsigned int period, unsigned long duration)
{
 unsigned long time_played;
 int i;
 time_played = 0;

 period >>=1 ;
 while (time_played < duration)
 {
 for (i = 0; i < period ; i++)
 {
 Delay_us(100);
 }
 time_played += period;

  LATA.LATA2  = ! LATA.LATA2 ;
 }
}

void start_sound()
{
 sound ( 50, 1000);
 Delay_ms(100);
 sound ( 20, 1000);
 Delay_ms(100);
 sound ( 10, 1000);
}

void low_batt_sound()
{
 sound ( 20, 1000);
 Delay_ms(100);
 sound ( 50, 1000);

}

void sensor_error_sound()
{
 sound ( 50, 1000);
}




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




void sendchar( char c)
{

 while (!UART1_Tx_Idle())
 {
 Delay_us(100);
 }
 UART1_Write(c);


}




void sendhex (unsigned long hexnumber, unsigned char cr )
{

 int nibble = 0;
 const char hexnr[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

 for (nibble = 0; nibble < 6; nibble++)
 {
 sendchar(hexnr[(hexnumber&0xF00000)>>20]);
 hexnumber<<=4;
 }
 if (cr ==  2  )
 {
 sendchar('\r');
 sendchar('\n');
 }
 else if (cr ==  1 )
 {
 sendchar('\r');
 }


}




void sendstring (char* string, unsigned char cr)
{

 int i = 0;
 char c;
 while (c=string[i++])
 {
 sendchar(c);
 }


 if (cr ==  2  )
 {
 sendchar('\r');
 sendchar('\n');
 }
 else if (cr ==  1 )
 {
 sendchar('\r');
 }

}




void write_EEPROM(unsigned int value, unsigned char address)
{

 address <<= 1;
 EEPROM_Write(address, value&0xFF);
 Delay_ms(20);
 EEPROM_Write(address+1, (value>>8)&0xFF);
 Delay_ms(20);
}




unsigned int read_EEPROM(unsigned char address)
{

 unsigned int value;
 address <<= 1;
 value = (EEPROM_Read(address+1) << 8) | EEPROM_Read(address);
 return value;
}







void main()
{

 int sample_diff, sample_diff_deviation;
 long sample_diff_sum;
 int sample_diff_average =  127 ;
 unsigned int audio_treshold, battery_voltage;
 unsigned int new_audio_treshold, old_audio_treshold = 0;
 unsigned int low_batt_count =  1000 ;
 unsigned char i;
 beepdivider = 0;
 beepcnt = 0;


 OSCCON= 0xF0;



 TRISA.TRISA0 =  1 ;
 PORTA.RA0 = 1;
 TRISA.TRISA1 =  1 ;
 PORTA.RA1 = 1;
 TRISA.TRISA2 =  0 ;
 PORTA.RA2 = 1;
 TRISA.TRISA3 =  0 ;
 PORTA.RA3 = 1;
 TRISA.TRISA4 =  1 ;
 PORTA.RA4 = 1;
 TRISA.TRISA5 =  0 ;
 PORTA.RA5 = 1;


 TRISC.TRISC0 =  1 ;
 PORTC.RC0 = 1;
 TRISC.TRISC1 =  1 ;
 PORTC.RC1 = 1;
 TRISC.TRISC2 =  1 ;
 PORTC.RC2 = 1;
 TRISC.TRISC3 =  0 ;
 PORTC.RC3 = 1;
 TRISC.TRISC4 =  0 ;
 PORTC.RC4 = 1;
 TRISC.TRISC5 =  0 ;
 PORTC.RC5 = 1;


 ANSELA.ANSA0 = 0;
 ANSELA.ANSA1 = 1;
 ANSELA.ANSA4 = 1;
 ANSELC.ANSC0 = 0;
 ANSELC.ANSC1 = 1;
 ANSELC.ANSC2 = 1;


 CM1CON0.C1POL = 1;
 CM1CON0.C1OE = 0;
 CM1CON0.C1SP = 1;
 CM1CON0.C1ON = 1;
 CM1CON0.C1HYS = 1;
 CM1CON0.C1SYNC = 0;
 CM1CON1.C1PCH0 = 0;
 CM1CON1.C1PCH1 = 1;
 CM1CON1.C1NCH0 = 1;
 CM1CON1.C1NCH1 = 0;
 CM1CON1.C1INTN = 0;
 CM1CON1.C1INTP = 1;


 CM2CON0.C2POL = 1;
 CM2CON0.C2OE = 0;
 CM2CON0.C2SP = 1;
 CM2CON0.C2ON = 1;
 CM2CON0.C2HYS = 1;
 CM2CON0.C2SYNC = 0;
 CM2CON1.C2PCH0 = 0;
 CM2CON1.C2PCH1 = 1;
 CM2CON1.C2NCH0 = 0;
 CM2CON1.C2NCH1 = 1;
 CM2CON1.C2INTN = 0;
 CM2CON1.C2INTP = 1;


 FVRCON.FVREN = 1;
 FVRCON.CDAFVR1 = 1;
 FVRCON.CDAFVR0 = 0;
 FVRCON.ADFVR1 = 0;
 FVRCON.ADFVR0 = 0;


 ADCON0.ADON = 1;
 ADCON1.ADFM = 1;
 ADCON1.ADCS0 = 0;
 ADCON1.ADCS1 = 1;
 ADCON1.ADCS2 = 1;
 ADCON1.ADNREF = 0;
 ADCON1.ADPREF0 = 0;
 ADCON1.ADPREF1 = 0;


 APFCON1.CCP2SEL = 1;


 PIE2.C1IE = 0;
 PIE2.C2IE = 0;


 OPTION_REG.PS0 = 1;
 OPTION_REG.PS1 = 1;
 OPTION_REG.PS2 = 0;
 OPTION_REG.PSA = 0;
 OPTION_REG.TMR0CS = 0;
 INTCON.TMR0IE = 1;


 INTCON.PEIE = 1;
 INTCON.GIE = 1;


 PWM2_Init(4000);
 PWM2_Start();






 APFCON0.TXCKSEL = 0;


 UART1_Init(115200);



 sample_array_pointer = 0;
  LATC.LATC3  =  0 ;



 battery_voltage = ADC_Read(1) >> 2;
 PWM2_Set_Duty(battery_voltage);


 for (i = 0; i < 10; ++i)
 {
 Delay_ms(200);
 sound ( 10, 2000);
 }

 Delay_ms(500);
 start_sound();

 PWM2_Set_Duty( 127 );









 while(1)
 {

 PIE2.C1IE = 0;
 PIE2.C2IE = 0;
 sample1 = 0;
 sample2 = 0;
 PIE2.C1IE = 1;
 PIE2.C2IE = 1;
 Delay_ms( 100 );
 PIE2.C1IE = 0;
 PIE2.C2IE = 0;


 if (!sample1 || !sample2)
 {
 sensor_error_sound();
 }


 sendhex ((unsigned long)sample1,  0 );
 sendchar(',');
 sendchar(' ');
 sendhex ((unsigned long)sample2,  0 );
 sendchar(',');
 sendchar(' ');
 sendhex ((unsigned long)audio_treshold,  2 );




 sample_diff = (int)(sample1 - sample2);

 if (! PORTA.RA0 )
 {
 sample_diff >>= 2;
 }
 sample_array[sample_array_pointer %  (1 << 5 ) ] = sample_diff;
 sample_array_pointer++;


 if (! PORTA.RA3 )
 {

 sample_diff_sum = 0;
 for (i = 0; i <  (1 << 5 ) ; i++)
 {
 sample_array[i] = sample_diff;
 sample_diff_sum += sample_array[i];
 }
 sample_diff_average = sample_diff_sum >>  5 ;
 }



 sample_diff_deviation = sample_diff - sample_diff_average;


 if (sample_diff_deviation < -127)
 {
 sample_diff_deviation = -127;
 }
 else if (sample_diff_deviation > 128)
 {
 sample_diff_deviation = 128;
 }


 meter_value = (unsigned char) ((int) 127  + sample_diff_deviation);
 PWM2_Set_Duty(meter_value);




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


 if (abs(sample_diff_deviation) > audio_treshold)
 {

 if (sample_diff_deviation > 0)
 {

 beepcnt = 1;
 beepdivider = 1;
  LATC.LATC3  =  1 ;
 }
 else
 {

 beepdivider = 2;
 beepcnt = 2;
  LATC.LATC3  =  1 ;
 }
 }
 else
 {

 beepcnt = 0;
 beepdivider = 0;
  LATC.LATC3  =  0 ;
 }


 battery_voltage = ADC_Read(1) >> 2;
 if (battery_voltage <  200 )
 {
 if (low_batt_count)
 {
 low_batt_count--;
 }
 else
 {
 low_batt_count =  1000 ;
 low_batt_sound();
 }
 }
 else
 {
 low_batt_count =  1000 ;
 }

 }

}

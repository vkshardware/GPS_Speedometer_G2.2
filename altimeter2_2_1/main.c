	/*
	Hardware configuration Atmega324PA with four 7-segments led FJ5461BH in PCB layout
	
	Cathode a = pin30 = PA7;
	Cathode b = pin32 = PA5;
	Cathode c = pin34 = PA3;
	Cathode d = pin36 = PA1;
	Cathode e = pin37 = PA0;
	Cathode f = pin31 = PA6;
	Cathode g = pin33 = PA4;
	Cathode dp = pin32 = PA2;
	
	Anode 1 dig = pin40 = PB0;
	Anode 2 dig = pin41 = PB1;
	Anode 3 dig = pin42 = PB2;
	Anode 4 dig = pin43 = PB3;
	
	Leds configuration:
	
	LED2 = pin25 = PC6;
	LED3 = pin24 = PC5;
	LED4 = pin23 = PC4;
	LED5 = pin22 = PC3;
	
	Button:
	 
	KEY1 = pin21 = PC2;
	
	I2C NVRAM & real time clock M41T56
	
	SCL = pin19 = PC0
	SDA = pin20 = PC1
	
	GPS receiver GG1802 (USART0)
	
	RX0 = pin9
	TX0 = pin10
	
	*/
#define F_CPU                  16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#include <avr/eeprom.h>
#include "twim.h"

#define  DIG1 1 << PB0
#define  DIG2 1 << PB1
#define  DIG3 1 << PB2
#define  DIG4 1 << PB3

#define SEG_A 1 << PA7
#define SEG_B 1 << PA5
#define SEG_C 1 << PA3
#define SEG_D 1 << PA1
#define SEG_E 1 << PA0
#define SEG_F 1 << PA6
#define SEG_G 1 << PA4
#define SEG_DP 1 << PA2

#define  LED2 1 << PC6  
#define  LED3 1 << PC5  
#define  LED4 1 << PC4  
#define  LED5 1 << PC3 


#define KEY1 1 << PC2    //  button
#define PULSE 1 << PD7    //  Pulse ABS output


#define KEY1_PUSH_DELAY 5
#define KEY1_LONGPUSH_DELAY 100
#define KEY1_MIDDLEPUSH_DELAY 50
#define KEY1_OFF_DELAY 5
#define UPDATE_DISPLAY 3
#define KPH_PER_NAUTICALMILE 1.852f  //knots to kph
#define PI 3.1415926f
#define MAX_DISTANCE 300.0f 
#define MIN_DISTANCE 13.0f
#define MIN_SPEED 3.2f
#define EARTH_RADIUS_METERS  6372795.0f
#define MAX_DUTYCYCLE 99
#define MIN_DUTYCYCLE 1
#define MAX_FREQ 255 
#define MIN_FREQ 1
#define TIMER1_CALIBRATION 0xC2F7 // 1 Hz timer 1024 divider
#define TIMER2_CALIBRATION 0xDA // 50kHz timer
#define DEFAULT_FREQ 65 //Hz for Volkswagen Touran 1T1 ABS 
#define DEFAULT_DC 30 // % negative duty cycle

#define M41T56_ADR    0b1101000 // Address for RTC/SRAM M41T56 in read mode 


union met
{
	unsigned long number;
	unsigned char bytes[4];
} Meters;


char buff0;
unsigned char buff1[8]="";
char actval[4] = "";

unsigned char Key1button, Key1_delay, CurrScreen = 0;
unsigned char Key1_store = 0;
unsigned char Key1_En = 1;
unsigned char RXwaitpause, GPS_Status = 0;
unsigned int GPS_Speed = 0;
unsigned int Distance = 0;
unsigned int GPS_Altitude = 0;
unsigned int GPS_Time, GPS_SatCount = 0;
unsigned char GPS_StatPointTimer = 0; 
unsigned char Timer1_en = 0;
unsigned char ReadSRAM = 1;
unsigned char UTC_Change, Fr10_Change, dc_Change = 0;
signed char UTC_TCorrection = 0;
float fLatitude,fLongitude, fPrevLatitude, fPrevLongitude = 0;
unsigned int Fr10_Pulse, dc_Pulse = 0;
unsigned char FirstScan = 0;
unsigned char SetPulseScreen, blink = 0;

char Time[12]=""; 
char Status[2]=""; 
char SLatitude[16]="";  
char NS[3]="";                          
char SLongitude[12]="";        
char EW[3]="";                  
char CourseTrue[10]="";         
char Data[12]="";               
char SatCount[4]="";            
char AltitudaMSL[12]="";        
char ViewSat[4]="";   
char COG[8]="";                 
char COGstat[4]="";             
char Speed[8]="";               
char SpeedAlt[8]="";    
char UNUSED[16]="";     
char Knot[8]="";
char Checksum[2] = "";
char *const RMC[]={Time,Status,SLatitude,NS,SLongitude,EW,Knot,CourseTrue,Data,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const GGA[]={UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,SatCount,UNUSED,AltitudaMSL,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};
char *const GSV[]={UNUSED,UNUSED,ViewSat,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED,UNUSED};

unsigned char GLONAS_COUNT=0;
unsigned char GPS_COUNT=0;
volatile char DataDone=0;
unsigned char DataValid=0;

uint16_t EEMEM fr10_eeprom;
uint16_t EEMEM dc_eeprom;

unsigned int Timer2_count, period, dc = 0;

void LoadEeprom(){
    Fr10_Pulse = eeprom_read_word(&fr10_eeprom);
    _delay_ms(5);
  
   if ((Fr10_Pulse > MAX_FREQ) || (Fr10_Pulse < MIN_FREQ)) 
   {
	   Fr10_Pulse = DEFAULT_FREQ;
	   eeprom_write_word(&fr10_eeprom,Fr10_Pulse);
	   _delay_ms(5);
   }
   
    
  dc_Pulse = eeprom_read_word(&dc_eeprom);
  _delay_ms(5);
  
   if ((dc_Pulse > MAX_DUTYCYCLE)||(dc_Pulse < MIN_DUTYCYCLE)) 
   {
	   dc_Pulse = DEFAULT_DC;
	   eeprom_write_word(&dc_eeprom,dc_Pulse);
	   _delay_ms(5);
   }
}

void WriteEeprom()
{
     eeprom_write_word(&fr10_eeprom,Fr10_Pulse);
	 _delay_ms(5);
	
	eeprom_write_word(&dc_eeprom,dc_Pulse);
	 _delay_ms(5);
}

void setled(unsigned char led)
{
	switch(led)
	{
		case 1: PORTC &= ~(LED3 | LED4 | LED5);
		PORTC |= (LED2);
		break;
		
		case 2: PORTC &= ~(LED2 | LED4 | LED5);
		PORTC |= (LED3);
		break;
		
		case 3: PORTC &= ~(LED2 | LED3 | LED5);
		PORTC |= (LED4);
		break;
		
		case 4: PORTC &= ~(LED2 | LED3 | LED4);
		PORTC |= (LED5);
		break;

		case 5:  PORTC &= ~(LED2 | LED5);
		PORTC |= (LED3 | LED4);
		break;
		
		default: PORTC |= (LED2 | LED3 | LED4 | LED5);
		
		
	}
}


void setseg(unsigned char seg)
{
	//clear segment
	
	PORTA &= (SEG_DP);
	
	switch(seg)
	{
		case 10: PORTA |= (SEG_G);
		break;
		
		case 0:  PORTA |= (SEG_F | SEG_E | SEG_D | SEG_C | SEG_B | SEG_A);
		break;
		case 1:  PORTA |= (SEG_B | SEG_C);
		break;
		case 2:  PORTA |= (SEG_A | SEG_B | SEG_G | SEG_E | SEG_D);
		break;
		case 3:  PORTA |= (SEG_A | SEG_B | SEG_G | SEG_C | SEG_D);
		break;
		case 4:  PORTA |= (SEG_F | SEG_G | SEG_B | SEG_C);
		break;
		case 5:  PORTA |= (SEG_A | SEG_F | SEG_G | SEG_C | SEG_D);
		break;
		case 6:  PORTA |= (SEG_A | SEG_F | SEG_G | SEG_E | SEG_D | SEG_C);
		break;
		case 7:  PORTA |= (SEG_A | SEG_B | SEG_C);
		break;
		case 8:  PORTA |= (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G);
		break;
		case 9:  PORTA |= (SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G);
		break;
		
		default: //empty segment
		break;
	}
}

void selectseg(unsigned char selseg)
{
	//clear 7seg led anodes
	
	PORTB  |= DIG1 | DIG2 | DIG3 | DIG4;
	
	switch(selseg)
	{
		case 1: PORTB &= ~(DIG1);
		break;
		
		case 2: PORTB &= ~(DIG2);
		break;
		
		case 3: PORTB &= ~(DIG3);
		break;
		
		case 4: PORTB &= ~(DIG4);
		break;
		
		default:
		break;
	}
}


char CharToHex(char src)
{
	if ((src > 47) && (src < 58))  //this covers 0-9
	src -= 48;
	else if ((src > 64) && (src < 71)) // this covers A-F
	src -= 55;
	
	return src;
}

void Parser(unsigned char data)
{
	static unsigned char ByteCount=0xFF;
	static unsigned long MsgType;
	static char *MsgTxt=(char*)&MsgType;
	static unsigned char ComaPoint=0xFF;
	static unsigned char CharPoint=0;
	static unsigned char Checksum_en=0;
	static unsigned char Checksumcalc = 0;
	char tmp;
	
	
	if(data=='$'){
		ByteCount=0;
		ComaPoint=0xFF;
		MsgTxt=(char*)&MsgType; 
		return;
		} 
	
	
	if(ByteCount==0xFF) return;                                                                     
	ByteCount++;
	

	
	if(ByteCount<=1){
		Checksumcalc = data;
		return; 
	}
			
	
	if(data=='*')
	{
			MsgType=0;
			Checksum_en = 1;
			CharPoint = 0;
			return;
	}
	
	if (!Checksum_en) Checksumcalc ^= data;  // XOR all bytes between '?' and '*' symbols
	
	
	if((ByteCount<6) && (ByteCount>1))            //keep message header
	{
		*MsgTxt=data;   
		MsgTxt++;
		return;
	}



	if (Checksum_en)   
	{
		Checksum[CharPoint] = data;
		CharPoint++;
		
		if (CharPoint > 1)
		{
			Checksum_en = 0;
			//process
			tmp = CharToHex(Checksum[0])*16+CharToHex(Checksum[1]);  // load hex checksum value 
					
		//	if (tmp != Checksumcalc)  CurrScreen = 0; // print error message
			
		}
		
		return;
	}	
	

	
	switch(MsgType)
	{
		
		case    0x434D524E:                             //GNRMC
		if(data==',') 
		{
			ComaPoint++;     
			CharPoint=0;
			RMC[ComaPoint][0]=0;
			return;
		}
		
		
		RMC[ComaPoint][CharPoint++]=data;
		RMC[ComaPoint][CharPoint]=0;
		return;

        case    0x4147474E:                             //GNGGA
		
		if(data==',')  {
			ComaPoint++;    
			CharPoint=0;
			GGA[ComaPoint][0]=0;
			return;
		}
        

        GGA[ComaPoint][CharPoint++]=data;
        GGA[ComaPoint][CharPoint]=0;
        return;

        case    0x56534750:             //GPGSV
        if(data==',')  {
			
			ComaPoint++;    
			
			CharPoint=0;
			GSV[ComaPoint][0]=0;
			return;
		}
					
        GSV[ComaPoint][CharPoint++]=data;
        GSV[ComaPoint][CharPoint]=0; 
        
		return; 
		
		default:        
			ByteCount=0xFF; 
		    break; 
	} 

	ByteCount=0xFF;
}


ISR (TIMER0_OVF_vect)
{
	cli();
	

	if (!(PINC & KEY1))  // Pressing button SB1, set up antinoise timer
	{
	  if (Key1button < 0xFF) Key1button++; 	
	} else
	{
	  Key1button = 0;
	  Key1_En = 1;
	}
	if (Key1_delay > 0) Key1_delay--;
    
	RXwaitpause++;
	
	GPS_StatPointTimer++;
	
	TCNT0 = 0;
	
	sei();
}

ISR (TIMER1_OVF_vect) // 1 Hz Timer
{
   
   Meters.number += round(GPS_Speed/36.0f);
  
  if (Meters.number >= 1e7) Meters.number = 0;

  blink =!blink;
  
  Timer1_en++; 
  TCNT1 = TIMER1_CALIBRATION;   
}

ISR (TIMER2_OVF_vect)
{
	cli();
	TCNT2 = TIMER2_CALIBRATION;
	Timer2_count++;
	if ((Timer2_count == 0xFFFF) || (Timer2_count >= period)) Timer2_count = 0;
	
	
	if ((Timer2_count == 0) || (period == 0xFFFF))
	{
		PORTD |= (PULSE);
	}
	
	if ((Timer2_count == dc) && (dc != 0xFFFF))
	    PORTD &= ~(PULSE);
	sei();
}

ISR (USART0_RX_vect)
{
	cli(); 
    buff0 = UDR0;
	
	Parser(buff0); 
	
	RXwaitpause = 0;


	sei();
}

ISR(TWI_vect)	
{
	TWI_Process();
}


int init_UART(void)
{

	UBRR0H=0;    //  
	UBRR0L=102;   //  UBRR=f/(16*band)-1 f=16000000Ãö band=9600,
	UCSR0A = 0;
	UCSR0B = (1 << RXCIE0)|(1 << TXEN0)|(1 << RXEN0);   //  interrupt enable, transmitting and receiving enable
	UCSR0C = (1 << UCSZ00)|(1 << UCSZ01);   //  8-bit

    return 0;
} 


unsigned int TimeToInt(char *src, unsigned short src_size, signed char utc_time)
{
   unsigned int num=0;
   unsigned int utc;
   
   utc = abs(utc_time)*100;
   
   for(int i=0; i<src_size; i++)
   {   
	   if ((src[i]-0x30) >= 10) return 0;
	   
	   num=num*10 + src[i]-0x30;
   }
   
   if (utc_time >= 0)
       num +=  utc;
	else
	{
		if (num < utc)
			num += 2400-utc;
		else
	        num -= utc;
	}
  
   if ((num / 100) >= 24) num -= 2400;
   
   return num;
}


// 4 digit point - used as GPS status indication.
//Low level - no data, blink - invalid data (V status), high - valid data (A status)

void print_gpsstatuspoint(void)
{
    switch (GPS_Status)
	{
	case 'V':	if (GPS_StatPointTimer >=30) PORTA |= SEG_DP;
				break;
	case 'A':   PORTA |=(SEG_DP);
	            break;
	default:    PORTA &= ~(SEG_DP);
	            break;
	}
}

void select_U(void)
{
	//Print "U" symbol
	
	PORTA &= ~(SEG_A | SEG_G | SEG_DP);
	
	PORTA |= (SEG_F | SEG_E | SEG_D | SEG_C | SEG_B);
	
}

void select_S(void)
{
	//Print "S" symbol
	
	PORTA &= ~(SEG_B | SEG_E | SEG_DP);
	
	PORTA |= (SEG_A | SEG_C | SEG_D | SEG_F | SEG_G);
	
}

void select_P(void)
{
	//Print "P" symbol
	
	PORTA &= ~(SEG_C | SEG_D | SEG_DP);
	
	PORTA |= (SEG_A | SEG_B | SEG_E | SEG_F | SEG_G);
	
}

void select_F(void)
{
	//Print "F" symbol
	
	PORTA &= ~(SEG_B | SEG_C | SEG_D | SEG_DP);
	
	PORTA |= (SEG_A | SEG_E | SEG_F | SEG_G);
	
}


void select_E(void)
{
	//Print "E" symbol
	
	PORTA &= ~(SEG_B | SEG_C | SEG_DP);
	
	PORTA |= (SEG_A | SEG_D | SEG_E |SEG_F |SEG_G);
	
}

void select_d(void)
{
	//Print "d" symbol
	
	PORTA &= ~(SEG_A | SEG_F | SEG_DP);
	
	PORTA |= (SEG_B | SEG_C | SEG_D |SEG_E |SEG_G);
	
}

void select_c(void)
{
	//Print "c" symbol
	
	PORTA &= ~(SEG_A | SEG_B | SEG_C | SEG_F | SEG_DP);
	
	PORTA |= (SEG_D | SEG_E | SEG_G);
	
}

void select_r(void)
{
	//Print "r" symbol
	
	PORTA &= ~(SEG_A | SEG_F | SEG_B | SEG_C | SEG_D | SEG_DP);

	PORTA |= (SEG_E |SEG_G);
}

void select_minus(void)
{
	// Print "-" symbol
	
	PORTA &= ~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_DP);

	PORTA |= (SEG_G);
	
}

void print_UTC(signed char val)
{
	unsigned char i = 0;
	
	actval[3] = abs(val % 10);
	actval[2] = abs((val/=10) % 10);
	
	selectseg(1);
	select_U();
	_delay_ms(UPDATE_DISPLAY);
	
	selectseg(2);
	
	if (UTC_TCorrection < 0) select_minus();
	else
	  selectseg(11);
	_delay_ms(UPDATE_DISPLAY);
	
	
	for(i=3;i<=4;i++)  {
			
		selectseg(i);
			
		PORTA &= ~(SEG_DP);
			
		if (i == 4) print_gpsstatuspoint();
			
			setseg(actval[i-1]);
			
			_delay_ms(UPDATE_DISPLAY);
	}
}

void print_decimal(unsigned int val, char point_pos,char blink_pos)
{
	unsigned char i,n = 0;
	
	actval[3] = val % 10;	
	actval[2] = (val/=10) % 10;
	actval[1] = (val/=10) % 10;
	actval[0] = (val/=10) % 10;
	
	if (!blink_pos)
	{
		
	  while ((n<3) && (actval[n] == 0)) n++;  //remove excess zeroes
	
	  if (4-point_pos < n) n = 4-point_pos;
	}
	
	for(i=n+1;i<=4;i++)  
	if (((blink_pos == (5-i)) && blink) || (blink_pos != (5-i)))
	{
	   
	   selectseg(i);      
	   
	   PORTA &= ~(SEG_DP);
	   	   
	   if (point_pos == 5-i)   PORTA |= (SEG_DP); 
	   
	   if (i == 4) print_gpsstatuspoint();
       setseg(actval[i-1]);
	   
	   
	   _delay_ms(UPDATE_DISPLAY);
	}	
	
}

void print_nodata(void)
{
	int i;
	
	for(i=1;i<=4;i++)  {
		selectseg(i);
		
		PORTA &= ~(SEG_DP);
		
		if (i == 4) print_gpsstatuspoint();
				
		setseg(10);
		_delay_ms(UPDATE_DISPLAY);
	}
	   
}

void print_SP10(void)
{
	selectseg(1);
	select_S();
	_delay_ms(UPDATE_DISPLAY);
	selectseg(2);
	select_P();
	_delay_ms(UPDATE_DISPLAY);
	selectseg(3);
	setseg(1);
	_delay_ms(UPDATE_DISPLAY);	
	selectseg(4);
	setseg(0);
	_delay_ms(UPDATE_DISPLAY);
}

void print_Fr10(void)
{
	selectseg(1);
	select_F();
	_delay_ms(UPDATE_DISPLAY);
	selectseg(2);
	select_r();
	_delay_ms(UPDATE_DISPLAY);
	selectseg(3);
	setseg(1);
	_delay_ms(UPDATE_DISPLAY);
	selectseg(4);
	setseg(0);
	_delay_ms(UPDATE_DISPLAY);
}

void print_dc(void)
{
	selectseg(1);
	select_d();
	_delay_ms(UPDATE_DISPLAY);
	selectseg(2);
	select_c();
	_delay_ms(UPDATE_DISPLAY);
}

void print_error(void)
{
	
	selectseg(1);
	select_E();
	_delay_ms(UPDATE_DISPLAY);
	
	selectseg(2);
	select_r();
	_delay_ms(UPDATE_DISPLAY);
	
	selectseg(3);
	select_r();
	_delay_ms(UPDATE_DISPLAY);
	
}

float radians(float src)
{
	
	return PI*src/180;
}

float DistanceBetween2Points( float Lat, float Lon, float prev_Lat, float prev_Lon)
{
	float dLat = radians(Lat - prev_Lat);
	float dLon = radians(Lon - prev_Lon);
	float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +	cos(radians(prev_Lat)) * cos(radians(Lat)) * sin(dLon / 2.0f) * sin(dLon / 2.0f);
	
	float d = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
	return d * EARTH_RADIUS_METERS;
}

int HexToInt(char src)
{
	int dec,units;
	
	dec = (src & 0xF0) >> 4;
	units = src & 0x0F;
	
	
	return dec*10+units;
}

char IntToHex(int src)
{
	int dec,units;
	
	dec = src / 10;
	units = src % 10;
	
	return (dec << 4) | units;
	
}

void CalculateABSPulse()
{
	
	if (GPS_Speed == 0)
	{
		   period = 0xFFFF;
		   dc=0xFFFF;
		   return;
	}
	if (Fr10_Pulse > 0)
	  period = round((float)5e6/(Fr10_Pulse*GPS_Speed));
	else
	  period = 0xFFFF; 
	
	dc = round((float)period*dc_Pulse*0.01);
}

uint16_t AddDigitPulse(uint16_t src, uint16_t digit)
{
	uint16_t digit_fact = 0;
	uint16_t result = 0;
	uint16_t pow_t = 0;
	
	if (digit == 0) digit = 1;
		
	pow_t = round(pow(10,digit-1));
		
	digit_fact =  (src / pow_t);
	digit_fact = digit_fact % 10;
	
	src -= digit_fact*pow_t;
	
	digit_fact++;
	
	if (digit_fact > 9) digit_fact = 0;
	
	
	result = src + digit_fact*pow_t;
	
	return result;
}


void CheckRangePulse()
{
	if (Fr10_Pulse > MAX_FREQ) Fr10_Pulse = MAX_FREQ;
	if (Fr10_Pulse < MIN_FREQ) Fr10_Pulse = MIN_FREQ;
	
	if (dc_Pulse > MAX_DUTYCYCLE) dc_Pulse = MAX_DUTYCYCLE;
	if (dc_Pulse < MIN_DUTYCYCLE) dc_Pulse = MIN_DUTYCYCLE;
}
int main(void)
{    
    int s1,s2 = 0;
	unsigned char k;
	
	Meters.number = 0;
    period = 0xFFFF;
	dc=0xFFFF;	
	
	//configuring outputs
	
    DDRA = 0xFF;
	DDRB = DIG1|DIG2|DIG3|DIG4;
	DDRC = LED2|LED3|LED4|LED5;
	DDRD = PULSE;
	
	
	// set up timer0 
	TCCR0A = 0;
	TCCR0B = (1 << CS02)|(1 << CS00);//1024 divider
	   
	TCNT0 = 0;
	   
	TIFR0 = (1<<TOV0);
	TIMSK0 = (1<<TOIE0);
	
	//set up timer1
	TCCR1A = 0;
	TCCR1B = (1 << CS12)|(1 << CS10); //1024 divider
	
	TCNT1 = TIMER1_CALIBRATION;
	
	TIFR1 = (1<<TOV1);
	TIMSK1 = (1<<TOIE1);
	
	TCCR2B = (1 << CS21);    //8 divider
	
	TCNT2 = TIMER2_CALIBRATION;
	
	TIFR2 = (1<<TOV2);
	TIMSK2 = (1<<TOIE2);	
	
	
	WDTCSR |= (1<<WDCE) | (1<<WDE);	 
	WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP1);  //watchdog 1 sec reset
	   
	LoadEeprom();
	
	// initializing USART0 connected to GPS 
	init_UART(); 
	
    TWI_MasterInit(60);
	
	  
	sei();   
	
	 _delay_ms(250); // delay for peripheral initialization
		 
	buff1[0] = (M41T56_ADR<<1)|0; //  M41T56 Slave receiver mode
	buff1[1] = 0x08;                 // set register pointer to 08H - using SRAM
		 
	TWI_SendData(buff1, 2);
		 
	buff1[0] = (M41T56_ADR<<1)|1; // M41T56 Slave transmitter mode
	TWI_SendData(buff1, 7);
		 
	TWI_GetData(buff1, 7); // get 6 bytes from M41T56
		 
	for (k=0; k<=3; k++)   //Load stored meters count
	Meters.bytes[k] = buff1[k+1];
		 
	UTC_TCorrection = buff1[5];
		 
	CurrScreen = buff1[6]; 
	
	if ((CurrScreen > 5)||(CurrScreen == 0)) CurrScreen = 1;
		 
	if ((UTC_TCorrection > 13) || (UTC_TCorrection < -12)) UTC_TCorrection = 0; 


    while (1)      // main cycle
    {  	
	
	 asm("wdr");    //watchdog reset
	 
   
	 
	 if (Timer1_en >= 3) // Load to SRAM actual meter count each 3 seconds
	 {
		 buff1[0] = (M41T56_ADR<<1)|0; //  M41T56 Slave receiver mode
		 buff1[1] = 0x08;              // set register pointer to 08H - using SRAM
		 
		 for (k=0; k<=3; k++)          // write new meters count
		 		 buff1[k+2] = Meters.bytes[k]; 
		 
		 buff1[6] = UTC_TCorrection;
		 buff1[7] = CurrScreen;
		 
		 TWI_SendData(buff1, 8);
		 Timer1_en = 0;
	 }
	  
	  if ((Key1button == 0) && (Key1_store >= KEY1_PUSH_DELAY) && (Key1_store <= KEY1_MIDDLEPUSH_DELAY) && (Key1_delay == 0)) // Simple push button
	  {		
		  	  if (UTC_Change)  // UTC time change mode
			  {
					UTC_TCorrection++;
					if (UTC_TCorrection > 13) UTC_TCorrection = -12;
					
					if (UTC_TCorrection < -12) UTC_TCorrection = 0;
			  } else
				
			 if (SetPulseScreen == 1)
			 {
				if (Fr10_Change)
				{
							
					Fr10_Pulse=AddDigitPulse(Fr10_Pulse,Fr10_Change);
					
	   
				} else
				Fr10_Change=1;
				
				
			 } else
			
			 if (SetPulseScreen == 2)
			 {
			    if (dc_Change)
				{
					dc_Pulse =AddDigitPulse(dc_Pulse,dc_Change);
					
				} else
				dc_Change =1;
				
			 } else
			 

				
			 if (CurrScreen < 5) CurrScreen++;  // change current screen
			 
			    else CurrScreen=1;
		    
		    Key1_delay = KEY1_OFF_DELAY;    
			  	  	  
	  }
	  
	  
	  
	  if ((Key1button == 0) && (Key1_store >= KEY1_MIDDLEPUSH_DELAY) && (Key1_store <= KEY1_LONGPUSH_DELAY) && Key1_En) // Middle (around 1,5sec) push button
	  {
		   
		   CheckRangePulse();
		   
		   if (SetPulseScreen == 1)
		   {
			    Fr10_Change++;
			   	if (Fr10_Change > 3) Fr10_Change = 1;
				
		   
		   } else
		   if (SetPulseScreen == 2)
		   {
			   dc_Change++;
			   if (dc_Change > 2) dc_Change = 1;
		   }	   
		  
		  Key1_En = 0;
		  Key1_delay = KEY1_OFF_DELAY;
		  blink = 0;
	  
	  }
	  
	  Key1_store = Key1button; 
	  
	   if ((Key1_store >= KEY1_LONGPUSH_DELAY) && Key1_En) // Long (around 3sec) push button
	   {		   
		  if (CurrScreen == 1)
		  {
			 
			 SetPulseScreen++;
			 if (SetPulseScreen > 2) SetPulseScreen = 0; 
			
			 Fr10_Change = 0;
			 dc_Change = 0;
			 
			 CheckRangePulse();

			 WriteEeprom();
		  } else
		  
		  
		   if (CurrScreen == 2)    //reset distance
		   {
			
		     Meters.number = 0;
		     Distance = 0;
		   
		     buff1[0] = (M41T56_ADR<<1)|0; //  M41T56 Slave receiver mode
		     buff1[1] = 0x08;              // set register pointer to 08H - using SRAM 
		   
		     for (k=0; k<=3; k++)	   buff1[k+2] = 0;                 // Clear distance
		  			  
		     TWI_SendData(buff1, 6);
		   } else
		   		   
		   if (CurrScreen == 4)    // Change UTC correction
		   {
			   if (UTC_Change)
			   {
				   //Save actual UTC time to SRAM memory
				   buff1[0] = (M41T56_ADR<<1)|0; //  M41T56 Slave receiver mode
				   buff1[1] = 0x0C;              // set register pointer to 0CH - using SRAM
				   buff1[2]	= UTC_TCorrection;
				   		     
				   TWI_SendData(buff1, 3);
			   }
			   
			   UTC_Change = !UTC_Change;
		   }
			   
		   Key1_En = 0;
		   Key1_delay = KEY1_OFF_DELAY;
	   } 
		  	   

	 
	  setled(CurrScreen); //set information LED
	    
	  if (RXwaitpause > 5) // GPS receiver did sent all data per 1 second, processing values
	  {

		 
		 GPS_Status = Status[0];
		 
		 if (GPS_Status == 'A')
		 {

				/*	 
		     sscanf(SLatitude,"%3d%2d.%4d",&s0,&s1,&s2);
		     fLatitude = s0+(s1+s2*0.0001)/60;
		     		
		     sscanf(SLongitude,"%3d%2d.%4d",&s0,&s1,&s2);
		     fLongitude = s0+(s1+s2*0.0001)/60;
		     		
		     if (fPrevLatitude == 0) fPrevLatitude = fLatitude;
		     if (fPrevLongitude == 0) fPrevLongitude = fLongitude;
			*/

					 
			sscanf(Knot,"%d.%1d",&s1,&s2);
			
			GPS_Speed =round((s1*10+s2)*KPH_PER_NAUTICALMILE);
			
			 CalculateABSPulse();
		
		    /*
		    if(GPS_Speed > MIN_SPEED)
		      {
			      dst = DistanceBetween2Points(fLatitude,fLongitude,fPrevLatitude,fPrevLongitude);
  
				      if ((dst <= MAX_DISTANCE) && (dst >=MIN_DISTANCE)) // Accumulate every MINIMUM_ACCUMULATE_DISTANCE
				      {
					      //Meters.number += round(dst);
			              fPrevLatitude = fLatitude;
			              fPrevLongitude = fLongitude;
				      }
			      
		      }
	      	*/
		 }
		 else
		 {
		   
		    GPS_Speed = 0;
			CalculateABSPulse(); 
		 }
		 
     	if (SetPulseScreen)
     	{
	     	GPS_Speed = 100;
	     	CalculateABSPulse();
	     	
     	}


		 sscanf(AltitudaMSL,"%d.%1d",&s1,&s2);
		 
		 if (s1 < 1000)
		    GPS_Altitude = s1*10+s2;
		 else
		    GPS_Altitude = s1;
		 
		 if (Meters.number < 1e6 )
		 {
		     if (Meters.number < 1e4)
			     Distance = Meters.number/10;
			 else
			     Distance = Meters.number/100;
		 }
		 else
		     Distance = Meters.number/1000;

		
	
		
		 if (GPS_Status == 'A') 
		 {
		    GPS_Time = TimeToInt(Time,4,UTC_TCorrection); //load time from NMEA
		 	
			if (FirstScan == 0) 
			{
				
			  buff1[0] = (M41T56_ADR<<1)|0; //  M41T56 Slave receiver mode
			  buff1[1] = 0x00;                 // set register pointer to 08H - using SRAM
			  buff1[2] = ((Time[4]-0x30) << 4) | (Time[5]-0x30); //sec
			  buff1[3] = IntToHex(GPS_Time % 100); //min
			  buff1[4] = IntToHex(GPS_Time / 100); //hours
			  TWI_SendData(buff1, 5); 
			  
			}
			FirstScan = 1;
						   	
		    
		 }
		 
    	if (GPS_Status != 'A')
     	{
			   	
			   	buff1[0] = (M41T56_ADR<<1)|0; //  M41T56 Slave receiver mode
			   	buff1[1] = 0x00;                 // set register pointer to 08H - using SRAM
			   	
			   	TWI_SendData(buff1, 2);
			   	
			   	buff1[0] = (M41T56_ADR<<1)|1; // M41T56 Slave transmitter mode
			   	TWI_SendData(buff1, 7);
			   	
			   	TWI_GetData(buff1, 7); // get 7 bytes from M41T56
			   
			   	GPS_Time =HexToInt(buff1[3] & 0xBF)*100+HexToInt(buff1[2]);
				FirstScan = 0;
				
		 	}
		 
		 
		 GPS_SatCount = atoi(SatCount);
		 RXwaitpause = 0;
	  }
	  
	  if (GPS_StatPointTimer >=60) GPS_StatPointTimer = 0;
	  
	    		  
	  switch(CurrScreen) //printing display data
	  {
		  case 0: print_error();
		          break;
		  
		  case 1: if (SetPulseScreen == 0){
			  		  if (GPS_Status == 'A')
			  		  {
				  		   print_decimal(GPS_Speed,2,0); // Speed
			  		  }   else
		  		      print_nodata();
				   }
				else 		
		  
		       if (SetPulseScreen == 1) {
			       if (Fr10_Change)
		    	   {
				      print_decimal(Fr10_Pulse,0,Fr10_Change);
			       } else
			         print_Fr10();
		      } else
		  
		      if (SetPulseScreen == 2) {
			     if (dc_Change)
			     {
			    	  print_decimal(dc_Pulse,0,dc_Change);
			     } else
			         print_dc();
		      }
		  
		       break;
		  case 2: 	if (Meters.number < 1e6 )
		  		      {
					    if (Meters.number < 1e4)
						  print_decimal(Distance,3,0);
						else
		        		  print_decimal(Distance,2,0);  //Distance
					  }
					  else
					      print_decimal(Distance,0,0); 
		          break;
		  case 3: if (GPS_Status == 'A') 
				  {
		              if (s1 < 1000)
					    print_decimal(GPS_Altitude,2,0);   //Altitude
					  else
					    print_decimal(GPS_Altitude,0,0);
				  }
				  else
				      print_nodata();
		          break;
		  
		  case 4: if (UTC_Change)
		              print_UTC(UTC_TCorrection);
				  else
				      print_decimal(GPS_Time,3,0);      // Print GPS time hh.mm (with user UTC)
		          break;
				  
		  case 5: print_decimal(GPS_SatCount,0,0);     // Satellites count
		          break;
	  }
	  
	  
		
    }
}


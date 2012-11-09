/** 
 *  gp2d12.c - GP2D12 functions.
 *
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @file gp2d12.c
 * @brief GP2D12 functions.
 */

#include <avr/io.h>
#include <util/delay.h>

#include "gp2d12.h"
#include "uart.h"
#include "main.h"
#include "position.h"
#include "messages.h"


//Extern variables
uint8_t ui8_sensornr     = 0;


/*! Last value read from A/D converter. */
//volatile uint8_t currentA2DValue;

  uint8_t ui8_analogToCM[4][256]={ 
  {255,255,255,255,255,255,255,255,255,255,255,
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255, //29
  255,255,255,255,255,80,77,75,73,72,70,68,66,65,63,62,60,59,58,56,55,54,53,52,//53
  51,50,49,48,47,46,45,45,44,43,42,42,41,40,40,39,39,38,37,37,36,36,35,35,34,34,34,//80
  33,33,32,32,32,31,31,30,30,30,29,29,29,28,28,28,28,27,27,27,26,26,26,26,25,25,25,25,//108
  24,24,24,24,23,23,23,23,23,22,22,22,22,21,21,21,21,21,21,20,20,20,20,20,19,19,19,19,//136
  19,19,19,18,18,18,18,18,18,18,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,15,//164
  15,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,14,14,13,13,13,13,13,13,13,//193
  13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,//222
  11,11,11,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,255,255,255,255,255,255,255,255,255,255,//250
  255,255,255,255,255}, //255
  {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,//14
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,//33
  255,79,77,75,73,71,70,68,66,65,63,62,60,59,58,57,55,54,53,52,51,50,49,48,48,47,//59
  46,45,44,44,43,42,42,41,40,40,39,39,38,38,37,37,36,36,35,35,34,34,33,33,32,32,32,31,31,31,//89
  30,30,29,29,29,28,28,28,27,27,27,27,26,26,26,25,25,25,25,24,24,24,24,23,23,23,23,23,22,22,22,//120
  22,22,21,21,21,21,21,20,20,20,20,20,20,20,19,19,19,19,19,19,18,18,18,18,18,18,18,18,17,17,17,17,//152
  17,17,17,17,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,13,13,//185
  13,13,13,13,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,//219
  11,11,11,11,11,11,11,10,10,10,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,//244
  255,255,255,255,255,255,255,255,255,255,255},
  {255,255,255,255,255,255,255,255,255,255,255,//10
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,//29
  255,255,255,255,255,225,80,77,74,72,70,68,66,64,63,61,60,58,57,56,55,53,52,51,50,49,//55
  48,47,46,45,45,44,43,42,42,41,40,40,39,38,38,37,37,36,36,35,35,34,34,33,33,33,32,32,31,31,31,//86
  30,30,30,29,29,29,28,28,28,27,27,27,26,26,26,26,25,25,25,25,24,24,24,24,23,23,23,23,23,22,22,22,22,//119
  21,21,21,21,21,21,20,20,20,20,20,19,19,19,19,19,19,19,18,18,18,18,18,18,18,17,17,17,17,17,17,17,17,16,16,//154
  16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,13,13,13,13,13,13,13,13,13,//191
  13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,//228
  11,10,10,10,10,10,10,10,10,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255},//255
  {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,//15
  255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,//33
  255,255,255,255,255,255,79,77,75,73,72,70,68,67,65,64,63,61,60,59,58,57,55,54,53,52,51,50,50,49,//63
  48,47,46,45,45,44,43,42,42,41,40,40,39,39,38,38,37,36,36,35,35,35,34,34,33,33,32,32,32,31,31,30,30,30,//97
  29,29,29,28,28,28,27,27,27,27,26,26,26,25,25,25,25,24,24,24,24,24,23,23,23,23,22,22,22,22,22,21,21,21,21,//132
  21,21,20,20,20,20,20,20,19,19,19,19,19,19,19,18,18,18,18,18,18,18,17,17,17,17,17,17,17,17,16,16,16,16,16,16,//168
  16,16,16,16,15,15,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,14,13,13,13,13,13,13,13,13,13,13,//204
  13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,//241
  10,10,10,10,10,10,10,255,255,255,255,255,255,255}};//255



/** 
 * Initialize A/D converter that is used to read from the GP2D12 unit.
 */
int gp2d12_init(void)
{
  	// Set ADC pins (A0-A3) to input 
	DDRA &= ~(_BV(PA0)) & ~(_BV(PA1)) & ~(_BV(PA2)) & ~(_BV(PA3));
	
	//Set ADC pin A4 to output (this pin is used for turning on IR-sensors)
	DDRA |= _BV(PA4);

	//Turn off IR-sensors
	PORTA &= ~(_BV(PA4));

	// Enable internal 2,54V AREF
  	ADMUX |= _BV(REFS1) | _BV(REFS0);
	
	// ADC enable; ADC prescaler setting (div. factor = 16)
  	ADCSRA |= _BV(ADEN) | _BV(ADPS2);  


  	return 0;		
}

/* Turn on IR-sensors */
int gp2d_12_on(void)
{
	PORTA |= (_BV(PA4));
	wait_ms(50);

	return 0;
}

/* Turn off IR-sensors */
int gp2d_12_off(void)
{
	PORTA &= ~(_BV(PA4));
	return 0;
}

/**
 * Reads a value from the IR sensor ui8_num and returns a value in cm
 */
inline uint8_t gp2d12_read(uint8_t ui8_num)
{
  	uint8_t ui8_analogValue;

	// Choose channel
	ADMUX = ui8_num;           
	// Enable internal 2,54V AREF
  	ADMUX |= (1<<REFS1) | (1<<REFS0);
	// Start conversion
  	ADCSRA |= _BV(ADSC);
  	loop_until_bit_is_clear(ADCSRA, ADSC);

  	/* Return the 8 most significant bits from the 10 bit register */
  	ui8_analogValue = (ADCL >> 2) | (ADCH << 6);

  	return ui8_analogToCM[ui8_num][ui8_analogValue]; 
  
}


/**
 * Reads n values from the IR sensor and returns the mean value in cm
 */
uint8_t ui8_array[5];
uint8_t ui8_analogValue;
int8_t i8_i;
int8_t i8_j;
uint8_t ui8_T;
inline uint8_t gp2d12_read_n(uint8_t ui8_num, uint8_t ui8_n)
{
	// Choose channel
	ADMUX = ui8_num;           
	// Enable internal 2,54V AREF
  	ADMUX |= (1<<REFS1) | (1<<REFS0);
	
	
	uint8_t ui8_a;
	for (ui8_a=0;ui8_a<ui8_n;ui8_a++)
	{
		ui8_analogValue = 0;
		// Start conversion
  		ADCSRA |= _BV(ADSC);
  		loop_until_bit_is_clear(ADCSRA, ADSC);
  		/* Return the 8 most significant bits from the 10 bit register */
  		ui8_analogValue = (ADCL >> 2) | (ADCH << 6);
		/* Array which contains ui8_n IR measurments */
		ui8_array[ui8_a] = ui8_analogValue; 
	}
	/* Sorts the array in ascending order*/
	for(i8_i = ui8_n; --i8_i>=0;)
		for(i8_j=0; i8_j<i8_i; i8_j++) {
			if(ui8_array[i8_j] > ui8_array[i8_j+1]) {
				ui8_T = ui8_array[i8_j];
				ui8_array[i8_j] = ui8_array[i8_j+1];
				ui8_array[i8_j+1] = ui8_T;
			}
		}
	/* Takes the three measurments in the middle and divide by 3*/
	uint8_t ui8_b = floor(ui8_n/2);
	ui8_analogValue = floor((ui8_array[ui8_b]+ui8_array[ui8_b+1]+ui8_array[ui8_b-1])/3);		
	//ui16_analogValue/=ui8_n;
  	return ui8_analogToCM[ui8_num][ui8_analogValue];
  
}


/**
 * send xy position of the meassured object
 */
inline uint8_t sendIR(uint8_t ui8_num)
{
	/*For full scans the sensors will be turned on 
	before this, to avoid too much delays*/
	if(ui8_status!=5) gp2d_12_on();

	//Measure
	uint8_t ui8_temp = gp2d12_read_n(ui8_num, 5);
	//Turn off sensors if not full scan
	if(ui8_status!=5) gp2d_12_off();

	if(ui8_h==0) //human mode off
	{
		if (ui8_temp == 0xFF)
		{
			sendNumberNoData(ui8_num);
		}
		else
		{
			sendNumber(ui8_num);
			uart_send16(objectPosX(90*ui8_num, ui8_temp));
			uart_send16(objectPosY(90*ui8_num, ui8_temp));
		}
	}
	else //human mode on
	{
		uart_send(0x0A);
		uart_send(0x0D);
		uart_send('S');
		uart_send('e');
		uart_send('n');
		uart_send('s');
		uart_send('o');
		uart_send('r');
		uart_sendh(ui8_num);
		uart_send(':');
		uart_send(' ');

		if (ui8_temp == 0xFF)
		{
			uart_send('n');
			uart_send('o');
			uart_send(' ');
			uart_send('d');
			uart_send('a');
			uart_send('t');
			uart_send('a');
		}
		else
		{
			
			uart_send('x');
			uart_send(':');
			uart_send16h(objectPosX(90*ui8_num, ui8_temp));
			uart_send(' ');
			uart_send('y');
			uart_send(':');
			uart_send16h(objectPosY(90*ui8_num, ui8_temp));
		}
		uart_send(' ');
		uart_send(' ');
		uart_send(' ');
	}

	return ui8_temp;
}



/**
 * Sends Data header for sensor ui8_num
 */
inline void sendNumber(uint8_t ui8_num)
{
	switch(ui8_num)
	{
		case 0:	
			uart_send(V2_MSG_IR_HEADER_1);
			break;
		case 1:	
			uart_send(V2_MSG_IR_HEADER_2);
			break;
		case 2:	
			uart_send(V2_MSG_IR_HEADER_3);
			break;
		case 3:	
			uart_send(V2_MSG_IR_HEADER_4);
			break;
	}

}

/**
 * Sends No-Data header for sensor ui8_num
 */
inline void sendNumberNoData(uint8_t ui8_num)
{
	switch(ui8_num)
	{
		case 0:	
			uart_send(V2_MSG_IR_HEADER_NO_DATA_1);
			break;
		case 1:	
			uart_send(V2_MSG_IR_HEADER_NO_DATA_2);
			break;
		case 2:	
			uart_send(V2_MSG_IR_HEADER_NO_DATA_3);
			break;
		case 3:	
			uart_send(V2_MSG_IR_HEADER_NO_DATA_4);
			break;
	}

}

/**
 * For the given sensor, the analog value is sent to Matlab 10 times
 */
 inline void sendAnalogValue()
 {
 	// Choose channel, 0,1,2 or 3
	ADMUX = ui8_sensornr;           
	// Enable internal 2,54V AREF
  	ADMUX |= (1<<REFS1) | (1<<REFS0);
	
	uint16_t ui16_analogValue=0;
	uint8_t ui8_j;
	uint8_t ui8_i;
	for (ui8_j=0;ui8_j<10;ui8_j++)
	{
		ui16_analogValue = 0;
		for (ui8_i=0;ui8_i<3;ui8_i++)
		{
			// Start conversion
  			ADCSRA |= _BV(ADSC);
  			loop_until_bit_is_clear(ADCSRA, ADSC);
  			/* Return the 8 most significant bits from the 10 bit register */
  			ui16_analogValue += (ADCL >> 2) | (ADCH << 6);
		}
		ui16_analogValue/=3;
		ui8_i = 0;
		uart_send((uint8_t)(ui16_analogValue));
	}

	uart_send(V2_MSG_IR_CALIBRATION_END);
 }

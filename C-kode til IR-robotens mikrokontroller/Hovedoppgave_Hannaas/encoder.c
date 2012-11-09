/** 
 *  encoder.c - Interface to angle sensor.
 *
 * @author Bjørn Syvertsen
 * @author Changed by Johannes Schrimpf
 * @file encoder.c
 * @brief Interface to angle sensor.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "encoder.h"
#include "lego.h"
#include "uart.h"
#include "messages.h"



int16_t i16_rightWheelTicks = 0;
int16_t i16_leftWheelTicks  = 0;


/**
 * Initializes the encoder
 */
int encoder_init()
{
  
	//set pin 2 & 3, port D as input 
	DDRD &= ~(_BV(PD2));
	DDRD &= ~(_BV(PD3));

	// set interrupt to trigger on rising edge
	MCUCR |= _BV(ISC11);  
	MCUCR |= _BV(ISC10);
	MCUCR |= _BV(ISC01);
	MCUCR |= _BV(ISC00);

	// clear interrupt flag
	GIFR &= ~(_BV(INTF0));
	GIFR &= ~(_BV(INTF1));

	//enable external interrupt 0 & 1
	GICR |= _BV(INT0);
	GICR |= _BV(INT1);

  	return 0;
} // encoder_init()


/**
 * Handle input from the left wheel counter
 */
SIGNAL(SIG_INTERRUPT0)
{
	//uart_send(0x35);
	if(i8_leftWheelDirection==WHEEL_DIRECTION_FORWARD)
	{
	  	i16_leftWheelTicks++;
	}
	else if(i8_leftWheelDirection==WHEEL_DIRECTION_BACKWARD)
	{
  		i16_leftWheelTicks--;
	}

} // SIGNAL(SIG_INTERRUPT0)


/**
 * Handle input from the right wheel counter
 */
SIGNAL(SIG_INTERRUPT1)
{
	//uart_send(0x36);
	if(i8_rightWheelDirection==WHEEL_DIRECTION_FORWARD)
	{
		i16_rightWheelTicks++;
	}
	else if(i8_rightWheelDirection==WHEEL_DIRECTION_BACKWARD)
	{
		i16_rightWheelTicks--;
	}

} // SIGNAL(SIG_INTERRUPT1)

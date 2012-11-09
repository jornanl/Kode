/** 
 *  servo.c - Servo implementation.
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Johannes Schrimpf
 * @file main.h
 * @brief Servo implementation.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>

#include "servo.h"
#include "main.h"
#include "position.h"
#include "messages.h"



/**
 * Initialize servo (pulse-width-modulation) system for servo.
 *
 * 20ms duty cycle, 1ms - 2ms cycle time.
 */
int servo_init(int8_t i8_angle)
{
	// Clear on C1A & CK/8
  	TCCR1B = _BV(WGM12) | _BV(CS11);                    

	// Enable interrupts 
  	timer_enable_int(_BV(OCIE1A) | _BV(OCIE1B));     

	// Set output pin 
  	DDRB |= _BV(PB0);                                     

  	OCR1A = SERVO_DUTY_CYCLE_TICS;
  	servo_set_angle(i8_angle);
  	i8_servoAngle=i8_angle;
  	return 0;
}


/**
 * Set new angle on the servo.
 */
inline void servo_set_angle(int8_t i8_angle)
{
	if (i8_angle > 90)
	{
		i8_angle = 90;
	}
	else if(i8_angle < -90)
	{
		i8_angle = -90;
	}

  	int16_t i16_res_angle =  i8_angle;
  	i16_res_angle 		  *= ONE_DEGREE_FACTOR;
  	i16_res_angle 		  /= 10;
  	OCR1B		          =  SERVO_CENTER_CYCLE_TICS + i16_res_angle;
  	i8_servoAngle		  =  i8_angle;
}



/**
 * Increases the servoangle 5 degrees
 */
inline void servo_set_angle_p5()
{
	servo_set_angle(i8_servoAngle+5);
}



/**
 * 
 */
SIGNAL(SIG_OUTPUT_COMPARE1A)
{
  PORTB |= _BV(PB0);
}


/**
 * 
 */
SIGNAL(SIG_OUTPUT_COMPARE1B)
{
	PORTB &= ~(_BV(PB0));
}

/** 
 *  lego.c - LEGO driver functions.
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @author Changed by Sigurd Hannaas
 * @file lego.c
 * @brief LEGO driver functions.
 */

#include <avr/pgmspace.h>

#include "lego.h"
#include "main.h"
#include "messages.h"


int8_t i8_rightWheelDirection = WHEEL_DIRECTION_STOP;
int8_t i8_leftWheelDirection  = WHEEL_DIRECTION_STOP;


/**
 * Initialize LEGO driver.
 */



void lego_init(void)
{
  	/* Set direction to output */
  	DDRC |= _BV(PC7) | _BV(PC6) | _BV(PC5)| _BV(PC4)| _BV(PC3)| _BV(PC2) | _BV(PC1) | _BV(PC0);

  	/* Default to 0V */
  	PORTC &= ~_BV(PA7) & ~_BV(PA6) & ~_BV(PA5) & ~_BV(PA4)& ~_BV(PA3) & ~_BV(PA2) & ~_BV(PA1) & ~_BV(PA0) ;
}




/**
 * Set driving direction command to LEGO robot.
 */
void lego_drive(int direction)
{
  	switch(direction)
  	{
    	case LEGO_RIGHT:

			i8_rightWheelDirection = WHEEL_DIRECTION_BACKWARD;
			i8_leftWheelDirection  = WHEEL_DIRECTION_FORWARD;
			

			SET_PC0;
			CLR_PC1;
			SET_PC6;
			CLR_PC7;

	
    		break;

    	case LEGO_LEFT:

			i8_rightWheelDirection = WHEEL_DIRECTION_FORWARD;
			i8_leftWheelDirection  = WHEEL_DIRECTION_BACKWARD;

      		CLR_PC0;
			SET_PC1;
			CLR_PC6;
			SET_PC7;

    		break;

    	case LEGO_FORWARD:

			i8_rightWheelDirection = WHEEL_DIRECTION_FORWARD;
			i8_leftWheelDirection  = WHEEL_DIRECTION_FORWARD;
			
			SET_PC0;
			CLR_PC1;
			CLR_PC6;
			SET_PC7;
			break;

    	case LEGO_BACKWARD:

			i8_rightWheelDirection = WHEEL_DIRECTION_BACKWARD;
			i8_leftWheelDirection  = WHEEL_DIRECTION_BACKWARD;
		
			CLR_PC0;
			SET_PC1;
			SET_PC6;
			CLR_PC7;

			break;

		case LEGO_STOP:
	
			i8_rightWheelDirection = WHEEL_DIRECTION_STOP;
			i8_leftWheelDirection  = WHEEL_DIRECTION_STOP;
			
			SET_PC0;
			SET_PC1;
			SET_PC6;
			SET_PC7;

			break;
			
		case LEGO_PWM_BREAK_RIGHT:
	
// 			NO CHANGE TO DIRECTION VARIABLES, AS GENERAL DIRECTION IS PRESERVED.
//			i8_rightWheelDirection = WHEEL_DIRECTION_STOP;
// 			i8_leftWheelDirection  = WHEEL_DIRECTION_FORWARD;
// 			
			//NO CHANGE TO LEFT WHEEL
//			SET_PC0;
//			CLR_PC1;
			SET_PC6;
			SET_PC7;

			break;
			
		case LEGO_PWM_BREAK_LEFT:
	
// 			NO CHANGE TO DIRECTION VARIABLES, AS GENERAL DIRECTION IS PRESERVED.
//			i8_rightWheelDirection = WHEEL_DIRECTION_STOP;
// 			i8_leftWheelDirection  = WHEEL_DIRECTION_FORWARD;
// 			
			SET_PC0;
			SET_PC1;
			
			//NO CHANGE TO RIGHT WHEEL
//			SET_PC6;
//			SET_PC7;

			break;
    	
		default:
			break;
    }
}



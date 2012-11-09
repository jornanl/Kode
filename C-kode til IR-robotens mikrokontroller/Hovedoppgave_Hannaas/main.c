/** 
 *  main.c - main() implementation.
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Radicaly changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @author Changed by Sigurd Hannaas 2011
 * @file main.c
 * @brief main() implementation.
 */

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>

#include "gp2d12.h"
#include "main.h"
#include "lego.h"
#include "servo.h"
#include "timer.h"
#include "uart.h"
#include "encoder.h"
#include "position.h"
#include "navigation.h"
#include "messages.h"
#include "battery.h"


/**
 * State Variable
 * 0: init
 * 1: idle
 * 2: robot is rotating
 * 3: robot is rotating and will drive afterwards
 * 4: driving to position
 * 5: IR-Scan
 * 6: low level command
 * 7: backing up to charging station
 * 8: performing a calibration of the IR sensors
 */
uint8_t ui8_status = 0; // state variable
uint8_t ui8_n      = 0; // counter for loops
uint8_t ui8_h      = 0; // human modus aktivated=1

/**
 * Initializes the system
 */
void init()
{
	// Initialize UART (RS-232) driver. Using PORT D (0, 1)
  	uart_init(57600);
  	// Initialize internal AD converter 
  	gp2d12_init();
  	// Initialize servo (PWM). Using PORT B (0) 
  	servo_init(0);
  	// Initialize timer 
  	timer_init();
  	// Initialize LEGO system 
  	lego_init();
  	// Initialize encoders 
  	encoder_init();
	//Initialize battery simulator (battery capacity in mAh as input)
	battery_init(5200);
  	// Enable global interrupt flag 
  	sei();
	// Reset Position 
	resetRobotPos();
}




/**
 * Main loop
 */
int main(void)
{
	init(); 

	ui8_status=1;
	
	if(ui8_h==1)
	{
		send_msg(1);
	}
	//Main-loop
	while(1) 
	{
		updateRobotPos();
		switch(ui8_status)
		{
		
		case 5:
			if(ui8_h==1)
			{
				send_msg(2);
			}
			else
			{
				uart_send(V2_MSG_IR_SCAN_HEADER);
			}
						
			if (i8_servoAngle !=0)
			{
				servo_set_angle(0);
				wait_ms(1000);	   	
			}
			
			gp2d_12_on();
			for (ui8_n=0;ui8_n<18;ui8_n++) 
			{	
				sendIR(0);
				sendIR(1);
				sendIR(2);
				sendIR(3);
				servo_set_angle_p5();
				wait_ms(200);
			}
			gp2d_12_off();
			
			servo_set_angle(0);
			wait_ms(1000);	    
			ui8_status=1;
			if(ui8_h==1)
			{
				send_msg(3);
			}
			else
			{
				uart_send(V2_MSG_IR_SCAN_HEADER_END);
			}
			break;
		case 1:
			robotControl(LEGO_STOP);
			break;
		case 2:
		case 3:
			robotRotate();
			break;
		case 4:
			//uart_send(V2_DEBUG_MARKER1); //DEBUGGING
			robotDrive();
			break;
		case 6:
			if(i8_rightWheelDirection==WHEEL_DIRECTION_FORWARD && i8_leftWheelDirection==WHEEL_DIRECTION_FORWARD)
			{
				robotControl(LEGO_FORWARD);
				break;
			}
			else if (i8_rightWheelDirection==WHEEL_DIRECTION_FORWARD && i8_leftWheelDirection==WHEEL_DIRECTION_BACKWARD)
			{
				robotControl(LEGO_LEFT);
				break;
			}
			else if (i8_rightWheelDirection==WHEEL_DIRECTION_BACKWARD && i8_leftWheelDirection==WHEEL_DIRECTION_FORWARD)
			{
				robotControl(LEGO_RIGHT);
				break;
			}
			else if (i8_rightWheelDirection==WHEEL_DIRECTION_BACKWARD && i8_leftWheelDirection==WHEEL_DIRECTION_BACKWARD)
			{
				robotControl(LEGO_BACKWARD);
				break;
			}
			else
			{
				robotControl(LEGO_STOP);
				ui8_status=1;
				if(ui8_h==1)
				{
					send_msg(8);
				}
			}
			break;
		case 7:
			robotControl(LEGO_BACKWARD);	//backing up to charging station
			break;
		case 8:
			gp2d_12_on();
			sendAnalogValue();
			gp2d_12_off();
			ui8_status=1;
			break;
		default:
			break;
		}//Select


	} //while


	return 0;
} //main


/**
 * Waits ui16_ms milliseconds.
 */
void wait_ms(uint16_t ui16_ms)
{
	uint16_t ui16_m=0;

	for (ui16_m = 0 ; ui16_m< ui16_ms ; ui16_m++)
	{
		_delay_ms(1);
	}
}



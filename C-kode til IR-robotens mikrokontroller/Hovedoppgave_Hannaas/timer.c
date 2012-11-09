/** 
 *  timer.c - timer functions.
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @author Changed by Sigurd Hannaas
 * @file timer.c
 * @brief Timer functions.
 */

#include <avr/interrupt.h>
#include <avr/io.h>

#include "timer.h"
#include "servo.h"
#include "main.h"
#include "position.h"
#include "uart.h"
#include "lego.h"
#include "gp2d12.h"
#include "messages.h"
#include "navigation.h"
#include "battery.h"


uint16_t ui16_ms_count = 0;			//not really millisecs. incremented approx. 61 times per sec.
uint16_t ui16_simLimiter = 0;		//counter to limit
uint16_t ui16_TEMPms_count = 0;
uint16_t ui16_sek = 0;
uint16_t ui16_min = 0;

uint8_t ui8_batterySimFreq = 20;	//battery simulation update frequency [Hz]


/**
 * Initialize timer.
 * Sets timer interrupts to approx. 17314,45 HZ 
 * CORRECTION: 16 MHz crystal / 1024 = 15625 Hz --> Overflow frequency (8 bit timer): 15625 / 256 = 61,03515625 Hz.
 */
void timer_init(void)
{

  	TIFR  &= ~(1 << TOV1);	//??? Bit used by 16 bit TIMER1, not TIMER0.
  	TIFR  |= _BV(TOIE0);	//??? TOIE0 is a bit in TIMSK register, not TIFR...

	// prescaler = 1024
  	TCCR0 |= _BV(CS02) | _BV(CS00);  

	 // start timer 
  	TIMSK |= _BV(TOIE0);             
	TCNT0 = 0;  
}


/**
 * Delays for approx. ms number and milliseconds.
 */
void ms_sleep(uint16_t ui16_ms)
{
	// Restart HW counter
  	TCNT0 = 0;          
	          
	// Restart software limiter 
  	ui16_ms_count = 0;                 
  	while (ui16_ms_count < ui16_ms);
}


/**
 * Increment ms_count
 * Collision system. When backing up, collision system is off.
 */
SIGNAL(SIG_OVERFLOW0)
{
  	ui16_ms_count++;	//(is later reset if > 60)
	
	ui16_simLimiter++;
	
	if(ui16_simLimiter > ((61 / ui8_batterySimFreq) - 1)) // batterySimFreq = 20
	{
		ui16_simLimiter = 0;
		
		//update battery simulation
		updateBatteryCapacity();
		
		//Ensuring robot movements
		switch (ui8_status)
		{
			case 1:
				robotControl(LEGO_STOP);
				break;
			case 2:
			case 3:
				robotRotate();
				break;
			case 4:
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
				robotControl(LEGO_BACKWARD);
				break;
			default:
				break;
		}
	}
	//end simLimit interrupt
	
  	if (ui16_ms_count>60)
  	{
  		if ((ui8_status == 4 || ui8_status == 6) && (ui8_status != 7))
		{
			//uart_send(V2_DEBUG_MARKER1);		//DEBUG: send signal every 1 sec? TEST SHOWS 1.197 secs. (61 / 61,03515625 Hz = 0,999 theoretically) --> CPUFREQUENCY= 19140 KHz??
			//collision

			if ( i8_servoAngle > 2)
			{
				servo_set_angle(0);
				wait_ms(500);	 
			}


			if(i8_rightWheelDirection==WHEEL_DIRECTION_FORWARD)
			{
				
				uint8_t ui8_ir=sendIR(0);
				sendIR(1);
				sendIR(3);
				if (ui8_ir < 24) // Stops when the robots front is about 12 cm from an obstacle
				{
					lego_drive(LEGO_STOP);
					ui8_status=1;
					if(ui8_h==1)
					{
						send_msg(6);
					}
					else
					{
						uart_send(V2_COLLISION_ERROR);
					}
					
				}

				else if(ui8_ir < 27) // collision warning when the robot is 15 cm from an obstacle
				{
					if(ui8_h==1)
					{
						send_msg(5);
					}
					else
					{
					uart_send(V2_COLLISION_WARNING);
					}
					
				}

			}
			else if(i8_rightWheelDirection==WHEEL_DIRECTION_BACKWARD)
			{
				uint8_t ui8_ir=sendIR(2);
				sendIR(1);
				sendIR(3);
				if (ui8_ir < 17.5) // Stops when the robots back is 10 cm from an obstacle
				{
					lego_drive(LEGO_STOP);
					ui8_status=1;
					if(ui8_h==1)
					{
						send_msg(6);
					}
					else
					{
						uart_send(V2_COLLISION_ERROR);
					}
					
				}
				else if(ui8_ir < 22.5) // Collision warning when the robots back is 15 cm from an obstacle
				{
					if(ui8_h==1)
					{
						send_msg(5);
					}
					else
					{
					uart_send(V2_COLLISION_WARNING);
					}
					
				}

			}
			
			
		}// if status
		ui16_ms_count=0;
		ui16_sek++;
	}// if count
} // SIGNAL


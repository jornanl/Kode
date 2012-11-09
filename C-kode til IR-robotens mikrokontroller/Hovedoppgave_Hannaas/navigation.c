/** 
 *  navigation.c - navigation routines
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Radicaly changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed and moved to own file by Johannes Schrimpf
 * @author Changed by Sigurd Hannaas
 * @file navigation.c
 * @brief navigation routines
 */

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stdio.h>


#include "navigation.h"
#include "position.h"
#include "lego.h"
#include "main.h"
#include "uart.h"
#include "messages.h"
#include "encoder.h"



// extern variables
uint8_t ui8_targetHeading     = 0;
int8_t i8_targetDistance      = 0;


// intern variables
int8_t i8_status_distance = 1;
float f_startPointX=0;
float f_startPointY=0;
int16_t i16_pwm_maxError = 1;			//maximum acceptable number of ticks different between right and left wheel
uint8_t ui8_messageLimiter = 0;			//limiter for debug messages
uint8_t ui8_lastDirection = 255;		//Status change control
uint8_t ui8_lastStatus = 0;				//Status change control
int16_t i16_startRightWheelTicks = 0;	//temporary ticks counter (signed int)
int16_t i16_startLeftWheelTicks = 0;	//temporary ticks counter (signed int)
int16_t i16_tempRightWheelTicks = 0;	//temporary ticks counter (signed int)
int16_t i16_tempLeftWheelTicks = 0;		//temporary ticks counter (signed int)


/**
 * rotates the robot to the desired heading
 */
void robotRotate()
{
	float f_deadZone     = (float) ((2.0*M_PI)/180.0); // 2 degree's
	float f_target       = (float) ui8_targetHeading;
	float f_targetInRads = M_PI*2.0*f_target/180.0;

	if( f_targetInRads-f_robotTheta > M_PI )
	{			
		robotControl(LEGO_RIGHT);
	}
	else if( f_targetInRads-f_robotTheta < -M_PI )
	{				
		robotControl(LEGO_LEFT);
	}
	else if( f_targetInRads+f_deadZone < f_robotTheta )
	{				
		robotControl(LEGO_RIGHT);
	}
	else if( f_targetInRads-f_deadZone > f_robotTheta )
	{
		robotControl(LEGO_LEFT);
	}
	else
	{
		if (ui8_status==3)
		{
			ui8_status=4;
		}
		else
		{
			ui8_status=1;
		}
		lego_drive(LEGO_STOP);	
	}
}


/**
 * moves the robot a desired distance
 */
void robotDrive()
{

	if(i8_status_distance==1 )
	{
		if(i8_targetDistance !=0 )
		{
			f_startPointX=f_robotX;
			f_startPointY=f_robotY;
			i8_status_distance=2;
			robotControl(LEGO_FORWARD);
		}
		else
		{
			ui8_status=1;
		}
	}
	else if(i8_status_distance==2)
	{
		float diffXSquareInCm = ((f_startPointX-f_robotX)*(f_startPointX-f_robotX))/100;
		float diffYSquareInCm = ((f_startPointY-f_robotY)*(f_startPointY-f_robotY))/100;

		if( (i8_targetDistance*i8_targetDistance) <= (diffXSquareInCm+diffYSquareInCm) )
		{
			ui8_status=1;
			i8_status_distance=1;
			i8_targetDistance=0;
			lego_drive(LEGO_STOP);
			if(ui8_h==1)
			{
				send_msg(4);
			}
			else
			{
				uart_send(V2_MSG_ARRIVED);						
			}	
		}
		else
		{
			robotControl(LEGO_FORWARD);	
		}
		
	}
}
/**
 * Controls the robot's movements by comparing ticks
 */
void robotControl(int direction)
{	
	if (direction != ui8_lastDirection || ui8_status != ui8_lastStatus)		//resetting controller tick counters 
	{
		i16_startLeftWheelTicks = i16_leftWheelTicks;
		i16_startRightWheelTicks = i16_rightWheelTicks;
		ui8_lastDirection = direction;
		ui8_lastStatus = ui8_status;
	}
	
	i16_tempLeftWheelTicks = i16_leftWheelTicks - i16_startLeftWheelTicks;
	i16_tempRightWheelTicks = i16_rightWheelTicks - i16_startRightWheelTicks;
	
	lego_drive(direction);		//ensuring proper movement, as the brakes do not affect the other wheel.
	
	switch (direction)
	{
		case LEGO_FORWARD:
		
			if (i16_tempRightWheelTicks > i16_tempLeftWheelTicks + i16_pwm_maxError)
			{
				//uart_send(V2_DEBUG_MARKER2);
				lego_drive(LEGO_PWM_BREAK_RIGHT);
			}
			else if (i16_leftWheelTicks > i16_rightWheelTicks + i16_pwm_maxError)
			{
				//uart_send(V2_DEBUG_MARKER3);
				lego_drive(LEGO_PWM_BREAK_LEFT);
			}
			
			break;
		
		
		case LEGO_BACKWARD:

			if (i16_tempRightWheelTicks < i16_tempLeftWheelTicks - i16_pwm_maxError)
			{
				lego_drive(LEGO_PWM_BREAK_RIGHT);
			}
			else if (i16_tempLeftWheelTicks < i16_tempRightWheelTicks - i16_pwm_maxError)
			{
				lego_drive(LEGO_PWM_BREAK_LEFT);
			}
			
			break;
			
		
		case LEGO_RIGHT:
		
			if (-i16_tempRightWheelTicks > i16_tempLeftWheelTicks + i16_pwm_maxError)
			{
				lego_drive(LEGO_PWM_BREAK_RIGHT);
			}
			else if (i16_leftWheelTicks > -i16_rightWheelTicks + i16_pwm_maxError)
			{
				lego_drive(LEGO_PWM_BREAK_LEFT);
			}
			
			break;
			
			
		case LEGO_LEFT:
		
			if (i16_tempRightWheelTicks > -i16_tempLeftWheelTicks + i16_pwm_maxError)
			{
				lego_drive(LEGO_PWM_BREAK_RIGHT);
			}
			else if (-i16_leftWheelTicks > i16_rightWheelTicks + i16_pwm_maxError)
			{
				lego_drive(LEGO_PWM_BREAK_LEFT);
			}
			
			break;
			
			
		default:
			
			break;

	} //switch case
}
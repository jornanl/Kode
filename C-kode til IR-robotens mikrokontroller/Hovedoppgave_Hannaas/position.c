/** 
 *  position.c - position routines
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Radicaly changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed and moved to own file by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @file position.c
 * @brief position routines
 */

#include <avr/io.h>
#include <math.h>
#include <stdio.h>

#include "position.h"
#include "navigation.h"
#include "encoder.h"
#include "main.h"
#include "messages.h"


//extern globals

float  		f_robotX     	= 0;	// robot X pos in mm 
float  		f_robotY     	= 0;	// robot Y pos in mm
float  		f_robotTheta 	= 0; 	// robot heading in rad
int16_t    	i8_servoAngle 	= 0; 	// Servo Angle in deg (90 is on left side and -90 on right side)
int8_t 		i8_reset    	= 0;	// reset=1 => position will be resetet in next updatePos()


// intern
int16_t i16_leftWheelOld    = 0;
int16_t i16_rightWheelOld   = 0;
int16_t i16_leftWheelNew    = 0;
int16_t i16_rightWheelNew   = 0;


/**
 * reset the robotposition to (0,0,0)
 */
void resetRobotPos()
{

	f_robotX            = 0; 
	f_robotY            = 0; 
	f_robotTheta        = 0; 

	i16_leftWheelOld    = 0;
	i16_rightWheelOld   = 0;
	i16_leftWheelNew    = 0;
	i16_rightWheelNew   = 0;

	i16_rightWheelTicks = 0;
	i16_leftWheelTicks  = 0;
	i8_reset            = 0;
}

/**
 * Calculates the position of the robot
 */
void calcRobotPos(int16_t i16_oldTicksLeft,int16_t i16_newTicksLeft,int16_t i16_oldTicksRight,int16_t i16_newTicksRight)
{

	
	float f_dthetaR = 0;
	float f_dxR     = 0;
	float f_sl      = 0;
	float f_sr      = 0;

	if(ui8_status == 2 || ui8_status == 3)
	{
		f_sl		= (i16_newTicksLeft  - i16_oldTicksLeft ) * X_FACTOR_ROTATE;
		f_sr        = (i16_newTicksRight - i16_oldTicksRight) * X_FACTOR_ROTATE;
	}
	else if(ui8_status == 4)
	{
		f_sl		= (i16_newTicksLeft  - i16_oldTicksLeft ) * X_FACTOR_DRIVE;
		f_sr        = (i16_newTicksRight - i16_oldTicksRight) * X_FACTOR_DRIVE;	
	}
	else
	{
		f_sl		= (i16_newTicksLeft  - i16_oldTicksLeft ) * X_FACTOR; //How far the left wheel has travelled in [mm]. See encoder.h for X_FACTOR
		f_sr        = (i16_newTicksRight - i16_oldTicksRight) * X_FACTOR; // How far the right wheel has travelled in [mm]
	}
	
	
	f_dxR           = (f_sr+f_sl)/2.0 ; // How far the midpoint of the robot has travelled  
	f_dthetaR       = (f_sr - f_sl) / WHEELBASE_MM ; // Find dthetaR by using the formula for arclength, (buelengde).

	// These formulas can be found in Syvertsen Master 2006 report, page 15.
	f_robotX        = f_robotX + ( f_dxR * cos(f_robotTheta + f_dthetaR/2.0) );
    f_robotY        = f_robotY + ( f_dxR * sin(f_robotTheta + f_dthetaR/2.0) );
    f_robotTheta    = f_robotTheta + f_dthetaR;
    

    if( f_robotTheta >= TO_PI )
       	f_robotTheta = f_robotTheta - TO_PI;
    else if( f_robotTheta < 0 )
       	f_robotTheta = f_robotTheta + TO_PI;
}


/**
 * Set a new robot position
 */
void setRobotPos(int16_t x, int16_t y, int16_t theta){
	f_robotX     = (float) x; 
	f_robotY     = (float) y; 
	f_robotTheta = ((float) theta)/1000; 

}



/**
 * Updates the position of the robot
 */
inline void updateRobotPos(){

	//sample
	i16_leftWheelNew  = i16_leftWheelTicks;
	i16_rightWheelNew = i16_rightWheelTicks;
	
	//calculate
	calcRobotPos(i16_leftWheelOld,i16_leftWheelNew,i16_rightWheelOld,i16_rightWheelNew);
	
	//update
	i16_leftWheelOld  = i16_leftWheelNew;
	i16_rightWheelOld = i16_rightWheelNew;

	if( i8_reset )
	{
		resetRobotPos();
	}

}


/**
 *  calculates the x-position of a meassured object. 
 */
inline int16_t objectPosX(int16_t i16_sensorT, uint8_t ui8_sensorDelta)
{	
	
	ui8_sensorDelta+=2;
	//i16_servoAngle negative because -90 is on left side and +90 on right side
	float f_theta   = ((float)(i16_sensorT+i8_servoAngle))/180*M_PI + f_robotTheta;
	float f_objectX = f_robotX - (32*cos(f_robotTheta)) + ( ((float)ui8_sensorDelta*10) * cos(f_theta));
	
	return((int16_t)f_objectX);

}

/**
 *  calculates the y-position of a meassured object.
 */
inline int16_t objectPosY(int16_t i16_sensorT, uint8_t ui8_sensorDelta)
{
	ui8_sensorDelta+=2;
	//i16_servoAngle negative because -90 is on left side and +90 on right side
	float f_theta   = ((float)(i16_sensorT+i8_servoAngle))*M_PI/180 + f_robotTheta;
	float f_objectY = f_robotY - (32*sin(f_robotTheta)) + ( ((float)ui8_sensorDelta*10) * sin(f_theta));
	
	return((int16_t)f_objectY);

}



/**
 * Calculates a heading and a distance to move to a position
 */
void goRobotPos(int16_t i16_goX, int16_t i16_goY){
	float f_goX           = (float) i16_goX;
	float f_goY           = (float) i16_goY;
	float diffXSquareInCm = ((f_goX-f_robotX)*(f_goX-f_robotX))/100;
	float diffYSquareInCm = ((f_goY-f_robotY)*(f_goY-f_robotY))/100;
	int16_t i16_angle       = (int16_t) (atan2(f_goY-f_robotY,f_goX-f_robotX)*180/M_PI);
	if (i16_angle<0)
	{
		i16_angle+=360;
	}
	ui8_status=3;
	ui8_targetHeading      = i16_angle/2;
	i8_targetDistance     = (int8_t)sqrt(diffXSquareInCm+diffYSquareInCm);

}




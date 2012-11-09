/*
 * battery.c - simple battery capacity simulator/monitor
 *
 * Created: 10.05.2011 16:51:03
 *  Author: Sigurd Hannaas
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "main.h"
#include "timer.h"
#include "battery.h"
#include "uart.h"




//capacity variables:
float f_batteryCapacity;
float f_alarmCapacity;

//Calibration coefficients
float f_batteryMaxCal;
float f_batteryLoadGain;

//current drawn from battery [mA]
float f_currentFlow;

//Simulation step period [h] 
float f_batterySimPeriod;

//Flag to set robot's need to charge
uint8_t ui8_needsToCharge;

//Preferred charging time [seconds]
uint16_t ui16_chargingTime;

//counter
uint8_t ui8_alarmCounter = 0;
uint8_t ui8_debugCounter = 0;
uint8_t ui8_debugCounterTwo = 0;


void battery_init(uint16_t ui16_startCapacity)
{
	f_alarmCapacity = 10.0;				// minimum remaining capacity before charging [mAh]
	f_batteryMaxCal = 0.606;			//calibration coefficient for battery life
	f_batteryLoadGain = (float)167/126;	//calibration coefficient for consumption
	
	f_batteryCapacity = f_batteryMaxCal * ui16_startCapacity;	// mAh
	
	f_currentFlow = 60.0;			// mA (idle)
	
	f_batterySimPeriod = 1/(3600*(float)ui8_batterySimFreq);	//(in hours to preserve units)
	
	ui8_needsToCharge = 0;
	
	ui16_chargingTime = 5*60*60;		//charging time in seconds (5 hours)
	
}

void updateBatteryCapacity()
{	

	switch (ui8_status)
	{
		case 1:	//idle
			f_currentFlow = 60.0;	//mA
			break;
		case 5:	//scanning
			f_currentFlow = 180.0;	//idle=60, ir=4*30 mA
			break;
		case 2:	//turning
		case 3:	//turning
		case 4:	//forward
		case 7: //backward
			f_currentFlow = 100.0;	//idle=60, motors=2*20 mA
			break;
		case 6:	//low-level command. Means driving.
			f_currentFlow = 100.0;	//idle=60, motors=2*20 mA
			break;
		case 8:	//calibration of IR-sensors (one at a time)
			f_currentFlow = 90.0;	//idle=60, ir=1*30 mA
			break;
		default:	//should never occur
			f_currentFlow = 220.0;	//assumes max usage, all bets are off if this is ever invoked.
			break;
	}	
	
	
	f_batteryCapacity -= f_batteryLoadGain*f_currentFlow*f_batterySimPeriod;		//mA*h
	
	//if (f_batteryCapacity >15.59)	//0,3 sec
	//{
		//uart_send(V2_DEBUG_MARKER1);
	//}		
	//else if (f_batteryCapacity < 14.766 && f_batteryCapacity > 14.0)	//30 secs
	//{
		//uart_send(V2_DEBUG_MARKER2);
	//}
	//else if (f_batteryCapacity < 13.933 && f_batteryCapacity > 13.5)	//1 min.
	//{
		//uart_send(V2_DEBUG_MARKER3);
	//}
	if (f_batteryCapacity < f_alarmCapacity)
	{
		//ALARM!!! recharge!
		ui8_needsToCharge = 1;
	}
}
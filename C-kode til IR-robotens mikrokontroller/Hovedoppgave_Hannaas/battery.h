/*
 * battery.h
 *
 * Created: 10.05.2011 16:51:32
 *  Author: Sigurd Hannaas
 */ 


#ifndef __battery_h__
#define __battery_h__


#include <inttypes.h>
#include <math.h>
#include <stdio.h>

extern uint8_t ui8_needsToCharge;
extern uint16_t ui16_chargingTime;

void battery_init(uint16_t ui16_startCapacity);

void updateBatteryCapacity();


#endif /* __battery_h__ */
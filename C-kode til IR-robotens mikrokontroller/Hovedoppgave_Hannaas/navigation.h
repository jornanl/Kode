/** 
 *  navigation.h - navigation routines
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Radicaly changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed and moved to own file by Johannes Schrimpf
 * @author Changed by Sigurd Hannaas
 * @file navigation.h
 * @brief navigation routines
 */


#ifndef __navigation_h__
#define __navigation_h__


#include <inttypes.h>

//extern int8_t i8_targetHeadingSat;
//extern int8_t i8_targetDistanceSat;
extern uint8_t ui8_targetHeading;
extern int8_t i8_targetDistance;


int8_t i8_status_distance;
float f_startPointX;
float f_startPointY;

void robotRotate();
void robotDrive();
void robotControl(int direction);


#endif /* __navigation_h__ */

/** 
 *  position.h - position routines
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Radicaly changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed and moved to own file by Johannes Schrimpf
 * @file position.h
 * @brief position routines
 */

#ifndef __position_h__
#define __position_h__

#include <inttypes.h>

extern float 	f_robotX; // robot X pos in mm
extern float 	f_robotY; // robot Y pos in mm
extern float 	f_robotTheta; // robot heading in rads
extern int16_t	i8_servoAngle;


extern int8_t i8_reset;

int16_t i16_leftWheelOld;
int16_t i16_rightWheelOld;
int16_t i16_leftWheelNew;
int16_t i16_rightWheelNew;


void calcRobotPos( signed int oldTicksLeft,signed int newTicksLeft,signed int oldTicksRight,signed int newTicksRight);
void resetRobotPos();
void setRobotPos(int16_t x, int16_t y, int16_t theta);
inline void updateRobotPos();

inline int16_t objectPosX(int16_t i16_sensorT, uint8_t ui8_sensorDelta);
inline int16_t objectPosY(int16_t i16_sensorT, uint8_t ui8_sensorDelta);
void goRobotPos(int16_t i16_goX, int16_t i16_goY);

#endif /* __position_h__ */

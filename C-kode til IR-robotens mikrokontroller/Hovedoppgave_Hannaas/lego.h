/** 
 *  lego.h - LEGO driver interface.
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Bjørn Syvertsen
 * @author Changed by Johannes Schrimpf
 * @author Changed by Sigurd Hannaas
 * @file lego.h
 * @brief LEGO driver interface.
 */


#ifndef __lego_h__
#define __lego_h__

#include <inttypes.h>

#define LEGO_LEFT				0
#define LEGO_RIGHT				1
#define LEGO_FORWARD			2
#define LEGO_BACKWARD			3
#define LEGO_STOP				4
#define LEGO_PWM_BREAK_RIGHT	5
#define LEGO_PWM_BREAK_LEFT		6

#define WHEEL_DIRECTION_STOP 		0
#define WHEEL_DIRECTION_FORWARD 	1
#define WHEEL_DIRECTION_BACKWARD 	2

#define SET_PC0 PORTC |= _BV(PC0)
#define SET_PC1 PORTC |= _BV(PC1)
#define SET_PC2 PORTC |= _BV(PC2)
#define SET_PC3 PORTC |= _BV(PC3)
#define SET_PC4 PORTC |= _BV(PC4)
#define SET_PC5 PORTC |= _BV(PC5)
#define SET_PC6 PORTC |= _BV(PC6)
#define SET_PC7 PORTC |= _BV(PC7)

#define CLR_PC0 PORTC &= ~_BV(PC0)
#define CLR_PC1 PORTC &= ~_BV(PC1)
#define CLR_PC2 PORTC &= ~_BV(PC2)
#define CLR_PC3 PORTC &= ~_BV(PC3)
#define CLR_PC4 PORTC &= ~_BV(PC4)
#define CLR_PC5 PORTC &= ~_BV(PC5)
#define CLR_PC6 PORTC &= ~_BV(PC6)
#define CLR_PC7 PORTC &= ~_BV(PC7)



extern int8_t i8_rightWheelDirection;
extern int8_t i8_leftWheelDirection;


/**
 * Initialize LEGO driver.
 */
void lego_init(void);

/**
 * Set driving direction command to LEGO robot.
 */
void lego_drive(int direction);


#endif /* __lego_h__ */

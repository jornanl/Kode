/** 
 *  servo.h - Servo interface.
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @file servo.h
 * @brief Servo interface.
 */


#ifndef __servo_h__
#define __servo_h__

#include <stdio.h>


#define SERVO_DUTY_CYCLE_TICS        40000
#define SERVO_CENTER_CYCLE_TICS       2915
#define ONE_DEGREE_FACTOR              133


/**
 * The variable 'angle' is given with:
 * -90 -> 90 degrees to the left
 * 0   -> Center
 * +90 -> 90 degrees to the right
 */

/**
 * Initialize servo (pulse-width-modulation) system for servo.
 *
 * 20ms duty cycle, 1ms - 2ms cycle time.
 */
int servo_init(int8_t angle);

/**
 * Set new angle on the servo.
 */
inline void servo_set_angle(int8_t angle);

inline void servo_set_angle_p5();


#endif /* __lservo_h__ */

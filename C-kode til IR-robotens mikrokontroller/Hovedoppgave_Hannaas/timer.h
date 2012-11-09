/** 
 *  timer.h - timer interface.
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Johannes Schrimpf
 * @file timer.h
 * @brief Timer interface.
 */


#ifndef __timer_h__
#define __timer_h__


#include <avr/pgmspace.h>

/**
 * Simulation update frequency [Hz]
 */
extern uint8_t ui8_batterySimFreq;

/**
 * Global counter variable.
 * This variable is increased approx. every millisecond
 */
uint16_t ms_count;


/**
 * Initialize timer.
 * Sets timer interrupts to approx. every milliseconds.
 */
void timer_init(void);

/**
 * Delays for approx. ms number og milliseconds.
 */
void ms_sleep(uint16_t ms);


#endif /* __timer_h__ */

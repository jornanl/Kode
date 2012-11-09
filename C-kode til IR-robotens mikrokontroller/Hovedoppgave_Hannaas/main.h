/** 
 *  main.c - main() interface.
 * 
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Sveinung Helgeland for version 2.0 of RoboRadar
 * @author Changed by Bjørn Syvertsen for version 3.0 of RoboRadar
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke selnes Tusvik
 * @file main.h
 * @brief main() implementation.
 */

#ifndef __main_h__
#define __main_h__


#include <inttypes.h>
#include <math.h>
#include <stdio.h>

/* Set to clock frequency in Hz */
#define CPU_FREQ        16000000L


extern uint8_t ui8_status;
extern uint8_t ui8_h;

void init();


void wait_ms(uint16_t ui16_ms);


#endif /* __main_h__ */

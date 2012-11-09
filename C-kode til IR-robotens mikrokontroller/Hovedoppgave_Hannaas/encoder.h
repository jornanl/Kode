/** 
 *  encoder.h - Interface to angle sensor.
 *
 * @author Bjrøn Syvertsen
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @author Changed by Sigurd Hannaas
 * @file encoder.h
 * @brief Interface to angle sensor.
 */
#ifndef __encoder_h__
#define __encoder_h__

#include <inttypes.h>

//#define WHEELRADIUS_MM 22.34 versjon 3
//#define WHEELRADIUS_MM 21.0*1.004629 versjon 2
//#define NUM_TICS_ROT 200 /* Number of tics per rotation of wheel.*/
//#define RADS_PER_TIC 0.0314159265

//#define WHEELBASE_MM 189.4938  // found by testing. Not measured
#define WHEELBASE_MM 210			// Measured (Sig)
#define TO_PI 2*M_PI
//#define X_FACTOR 0.675			//Circumference(135mm)/Tics pr rotation(200) SHOULD WORK. How far the robot travles per tick
#define X_FACTOR 0.78			//Found by calculation. Circumference(156 mm)/Tics per rotation (200)
#define X_FACTOR_ROTATE 0.7587;	//Found by testing (Sig)
#define X_FACTOR_DRIVE 0.8315;	//Found by testing (Sig)

extern int16_t i16_rightWheelTicks ;
extern int16_t i16_leftWheelTicks ;
/* Distance measured by angle sensor given as number of tics registered.*/

int encoder_init();

#endif  /* __encoder_h__ */

/** 
 *  gp2d12.h - GP2D12 interface.
 *
 * @author Håkon Skjelten <skjelten@pvv.org>
 * @author Changed by Johannes Schrimpf
 * @author Changed by Jannicke Selnes Tusvik
 * @file gp2d12.h
 * @brief GP2D12 interface.
 */


#ifndef __gp2d12_h__
#define __gp2d12_h__


#include <inttypes.h>

extern uint8_t ui8_sensornr;


int    gp2d12_init(void);
int    gp2d_12_on(void);
int    gp2d_12_off(void);
inline uint8_t gp2d12_read(uint8_t ui8_num);
inline uint8_t gp2d12_read_n(uint8_t ui8_num, uint8_t ui8_n);
inline uint8_t sendIR(uint8_t ui8_num);
inline void sendNumber(uint8_t ui8_num);
inline void sendNumberNoData(uint8_t ui8_num);
inline void sendAnalogValue();
#endif /* __gp2d12_h__ */

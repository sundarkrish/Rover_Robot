/*---------------------------------------------------------------------------
*timestamp.h
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reuse: 
*
**--------------------------------------------------------------------------*/

#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_


#include <stdint.h>

extern uint32_t clock_counter;
void timestamp_init();
uint32_t get_millis();
void get_timestamp(char *);


#endif /* TIMESTAMP_H_ */

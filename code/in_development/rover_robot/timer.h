/*--------------------------------------------------------------------------------------------
*timer.h
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------------------------
*Code Referenced:   MSP432 example code msp432p401x_ta0_03
*
**------------------------------------------------------------------------------------------*/

#ifndef TIMER_H_
#define TIMER_H_


//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//libraries
#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

#define TIMER_A_STOP            0xFFCF
#define ONE_USEC_COUNTS         0x0002
#define ONE_MSEC_COUNTS         0x05DC
#define TWENTY_FIVE_MSEC        0x927C

#define USEC_TO_COUNTS(x)       (float)(x / .667)
#define MSEC_TO_COUNTS(x)       (float)((x * 1000) / .667)

#define COUNTS_TO_USEC(x)       (.667 * x)

//debug flag
//#define _TIMER_DEBUG_

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

enum timer_number{
    timer0,
    timer1,
    timer2,
    timer3
};

enum interrupts{
    no_interrupts,
    interrupts
};

//-------------------------------------------------------------------------------------------
//                              PUBLIC FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//void timer_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   configures timer_AN as a simple up counter with SMCLK as the source and a clock
//          divider of 8.  If interrupts are enabled the timer overflow will cause it's
//          respective interrupt to fire (i.e. TA0_N_IRQHandler()...)
//
//@param:   void
//@return:  void
//@outcome: timer_A0 is configured, and interrupts are enabled or disabled.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_init(uint8_t timer, bool interrupts);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void timer_start(uint16_t val)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   starts timer and sets CCR0 to desired overflow value.
//
//
//@param:   uint16_t val -- count value which timer will overflow
//@return:  void
//@outcome: timer starts running and CCR0 set.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_start(uint8_t timer, uint16_t val);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void timer_stop(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   stops timer from running by setting MC bits to 0
//
//
//@param:   void
//@return:  void
//@outcome: timer stopped
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_stop(uint8_t timer);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void timer_clear(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   clears the timer count register and overflow flag.  MUST BE CALLED AFTER TIMER
//          HAS BEEN STOPPED.
//
//@param:   void
//@return:  void
//@outcome: overflow flag cleared and timer value reset to 0
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_clear(uint8_t timer);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline uint16_t timer_getval(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   gets the current value of the timer counter.  TIMER MUST BE STOPPED BEFORE.
//
//
//@param:   uint8_t timer -- the timer which we are reading
//@return:  uint16_t -- value of timer
//@outcome: timer value read.
/////////////////////////////////////////////////////////////////////////////////////////////

inline uint16_t timer_getval(uint8_t timer);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline bool timer_overflow(uint8_t timer)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns value of timer overflow flag.
//
//
//@param:   uint8_t timer -- the timer we are checking
//@return:  bool -- true if timer overflow
//@outcome: returns overflow status of timer.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_overflow(uint8_t timer);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline bool timer_overflow_flag_clear(uint8_t timer)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   clears the overflow flag of a selected timer
//
//
//@param:   uint8_t timer -- timer flag we are clearing
//@return:  bool -- true if timer successfully cleared
//@outcome: a timer's overflow flag is cleared and ISRs stop occuring.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_overflow_flag_clear(uint8_t timer);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void delay_sec(uint16_t sec)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer based delay with one second increments.  USES TIMER A0!
//
//
//@param:   uint16_t sec    -- number of seconds to wait
//@return:  void
//@outcome: processor delayed X seconds.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void delay_sec(uint16_t sec);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void delay_ms(uint16_t msec)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer based delay with millisecond increments.  USES TIMER A0!
//
//
//@param:   uint16_t msec -- number of milliseconds to wait.
//@return:  void
//@outcome: processor delayed x milliseconds.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void delay_ms(uint16_t msec);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void delay_us(uint16_t usec)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer based delay with microsecond increments.  USES TIMER A0!
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void delay_us(uint16_t usec);

#endif /* TIMER_H_ */

/*---------------------------------------------------------------------------------------
*clock.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
**---------------------------------------------------------------------------------------
*@brief: This program allocates a buffer of a particular size as requested by
*a user over a serial connection.
*
*Code Use:  modified version of MSP432 Example euscia0_uart_01
*
*--------------------------------------------------------------------------------------*/

#include "clock.h"

////////////////////////////////////////////////////////////////////////////////////////////
//void clock_init(void)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Initializes all clocks to run at DCO of 12 MHz and ACLK to REFO (32KHz)
//
//
//@param:   void
//@return:  void
//@outcome: Clocks initialized
//
////////////////////////////////////////////////////////////////////////////////////////////

void clock_init(void)
{
    //setup CTL Clock Sources
    CS->KEY = CS_KEY_VAL;                   // writing 0x695A unlocks CS module for register access
    CS->CTL0 = 0;                           // Reset all tuning parameters to zero
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Auxiliary clock, select ACLK = REFO, selectable by peripheral modules
            CS_CTL1_SELS_3 |                // SMCLK = DCO, low speed subsystem master clock, f_max = 1/2*f_hsmclk
            CS_CTL1_SELM_3;                 // MCLK = DCO, master clock used by CPU and peripheral module interfaces.
    CS->KEY = 0;                            // Lock CS module from unintended accesses

}

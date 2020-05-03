/*-------------------------------------------------------------------------------------------
*main.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*--------------------------------------------------------------------------------------------
*Code Reuse:
*
**-----------------------------------------------------------------------------------------*/


//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "msp.h"

//Project Libraries
#include "clock.h"
#include "timer.h"
#include "serial.h"
#include "utility.h"
#include "motor.h"

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

void system_init(void);


//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

void main(void)
{

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     /*stop that dog watching us*/

    // Enable global interrupt
    __enable_irq();

	system_init();                                  /*initialize system peripherals*/

	while(1)
	{
	    if(!serial_rx_buffer_empty())
	    {
	        printf("%c",serial_read_byte());
	    }
	}

}

/////////////////////////////////////////////////////////////////////////
//void system_int(void)
/////////////////////////////////////////////////////////////////////////
//@brief:   initialization function.  Calls hardware configuration functions
//          to setup processor and peripherals.
//
//
//@param:   void
//@return:  void
//@outcome: SFRs configured
//////////////////////////////////////////////////////////////////////////

void system_init(void)
{
    clock_init();
    serial_init();
    timer_init(timer0,no_interrupts);
    motor_init();
}


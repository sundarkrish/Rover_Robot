/*---------------------------------------------------------------------------
*main.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reuse:
*
**--------------------------------------------------------------------------*/


//---------------------------------------------------------------------------
//                              INCLUDES
//---------------------------------------------------------------------------

//Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "msp.h"

//Project Libraries

#include "ultrasonic.h"
#include "clock.h"
#include "timer.h"
#include "serial.h"
#include "utility.h"

//---------------------------------------------------------------------------
//                              DEFINES
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//                          FUNCTION PROTOTYPES
//---------------------------------------------------------------------------

void system_init(void);

//---------------------------------------------------------------------------
//                          EXTERNAL VARIABLES
//---------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     /*stop that dog watching us*/

    // Enable global interrupt
    __enable_irq();

	system_init();                                  /*initialize system peripherals*/

	printf("System initialized!\n\r");

	while(1)
	{
	    printf("Object Distance: %.2lf (inches)\n\r",ultrasonic_getdist(INCHES));
	    delay_ms(300);
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
    ultrasonic_init();

}


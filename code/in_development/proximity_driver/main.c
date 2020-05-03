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
#include "i2c.h"
#include "utility.h"
#include "apds9960.h"

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
    uint8_t data;

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     /*stop that dog watching us*/

    // Enable global interrupt
    __enable_irq();

	system_init();                                  /*initialize system peripherals*/

	printf("system finished booting up!\n\r");

	while(1)
	{
	    if(I2C1_Read(APDS9960_ADDR, APDS9960_ENABLE, &data));
	    {
	        printf("I2C Read ADPS9960 Register 0x80 = %X\n\r",data);
	    }

	    I2C1_Write(0x39, 0x80, 0x01);

	    delay_sec(3);
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
    I2C1_init();
}


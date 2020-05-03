/*--------------------------------------------------------------------------------------------
*template.h
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------------------------
*Code Referenced:    https://gist.github.com/alexjaw/3be818372e150a13ec5904d14361b12e
*
**------------------------------------------------------------------------------------------*/

#ifndef _APPLICATION_H_
#define _APPLICATION_H_

//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//libraries
#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//project libraries

#include "clock.h"
#include "timer.h"
#include "serial.h"
#include "utility.h"
#include "motor.h"
#include "ultrasonic.h"
#include "i2c.h"
#include "temperature.h"
#include "apds9960.h"
#include "navigate.h"
#include "mavlink_tx.h"

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

#define __PROGRAM_VERSION__        "1.0"                                //current version of program

#define PROGRAM_FREQ                100L                                //program loop rate in Hz.
#define SYSTICK_COUNT_VAL           (F_CPU / PROGRAM_FREQ)              //equivalent systick counts

#define TEN_LOOPS                   10                                  //program_frequency/10
#define ONE_HUNDRED_LOOPS           100                                 //program_frequency/100

#define TERMINAL_CLEAR              printf("\033[2J")                   // Clear screen
#define TERMINAL_MOVE_CURSOR_HOME   printf("\033[0;0H")                 //move cursor to (0,0)

#define MAX_CMD_LENGTH              40                                  //longest possible command


//Pin  Definitions

#define IMU_RESET                   0x01

#define _5V_ENABLE                  P4->OUT  |= 0x40                    //5V Power Supply Enable (P4.6)
#define _5V_DISABLE                 P4->OUT &= ~0x40                    //5V Power Supply Disable

#define RED_LED_TOGGLE              P1->OUT ^= BIT0

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

/* Command Handler Variables start */

typedef void (* functionPointer_t)(void);

typedef struct{
    const char  *name;
    functionPointer_t run;
    const char *help;
}commandStruct_t;

extern bool fota_flag;
/* Command Handler Variables end */



//-------------------------------------------------------------------------------------------
//                              PUBLIC FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//void CmdGet(void) -- NON-BLOCKING
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Handles user input received over serial port.  Reads characters from RX Buffer
//          and executes requested tasks.
//
//@param:   void
//@return:  void
//@outcome: user requested action is executed
/////////////////////////////////////////////////////////////////////////////////////////////

void CmdGet(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool CmdHandler(char * cmd)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   compares command string against command list and executes the appropriate function
//          through function pointer.
//
//@param:   char *cmd -- pointer to command string.
//@return:  bool -- true if no command found.
//@outcome: command executed, or error message reported.
/////////////////////////////////////////////////////////////////////////////////////////////

bool CmdHandler(char * cmd);

/////////////////////////////////////////////////////////////////////////////////////////////
//void CmdHelp(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   prints out all command strings and their descriptions.
//
//
//@param:   void
//@return:  void
//@outcome: list of commands and their descriptions displayed
/////////////////////////////////////////////////////////////////////////////////////////////

void CmdHelp(void);


/* Command Handler Functions end*/

/////////////////////////////////////////////////////////////////////////////////////////////
//void system_int(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   initialization function.  Calls hardware configuration functions
//          to setup processor and peripherals.
//
//
//@param:   void
//@return:  void
//@outcome: SFRs configured
/////////////////////////////////////////////////////////////////////////////////////////////

void system_init(void);

#endif  //__APPLICATION_H__

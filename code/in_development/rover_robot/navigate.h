/*--------------------------------------------------------------------------------------------
*template.h
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------------------------
*Code Reuse:
*
**------------------------------------------------------------------------------------------*/

#ifndef _NAVIGATE_H_
#define _NAVIGATE_H_

//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//libraries
#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//Custom Libraries

//#include "motor.h"          //include for direction_t enum
//#include "ultrasonic.h"

#include "application.h"

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------


#define MAX_SAVED_STATES        2

#define MAX_HIDING_LIGHT_LEVEL  2000

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

typedef enum{
    HIDING,
    SEARCHING,
    WAITING,
    SCANNING,
}rover_mode_t;

typedef struct{
   uint16_t yaw;
   uint16_t pitch;
   uint16_t roll;
   direction_t direction;
   uint8_t speed;
   bool driving;
   uint16_t light;
   uint16_t temp;
   float range;
   rover_mode_t mode;
}rover_state_t;

//-------------------------------------------------------------------------------------------
//                              PUBLIC FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//void navigate_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   intializes rover data struct to default values and sensor data
//
//
//@param:   void
//@return:  void
//@outcome: rover state struct initialized.
/////////////////////////////////////////////////////////////////////////////////////////////

void navigate_init(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//void navigate_execute(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   simple autonomous navigation routine looks for dark areas of room.  If obstact
//          detected the rover backs up and turns left or right before continuing forward.
//
//@param:   void
//@return:  void
//@outcome: rover attempts to find dark area of room to hide.
/////////////////////////////////////////////////////////////////////////////////////////////

void navigate_execute(void);

#endif  //_NAVIGATE_H_

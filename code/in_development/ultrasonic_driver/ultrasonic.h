/*---------------------------------------------------------------------------
*@file ultrasonic.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reuse:
*
**--------------------------------------------------------------------------*/

#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "msp.h"

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

#define ECHO                0x0020
#define TRIGGER             0x0010

#define SET_TRIGGER         P5->OUT |= TRIGGER
#define CLR_TRIGGER         P5->OUT &= ~TRIGGER

#define MAX_DIST           0x89DE

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

typedef enum
{
    METERS,
    CENTIMETERS,
    MILLIMETERS,
    INCHES
}ultrasonic_unit_t;

typedef struct
{
    bool initialized;
    bool trigger;
    uint16_t distance;
    uint16_t echo;
} ultrasonic_t;


typedef struct
{
    uint16_t sample_rate;
    ultrasonic_unit_t units;
}ultrasonic_config_t;


//--------------------------------------------------------------------------------------------
//                              PUBLIC FUNCTION PROTOTYPES
//--------------------------------------------------------------------------------------------


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

void ultrasonic_trigger(void);

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

double ultrasonic_getdist(uint8_t units);

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

void ultrasonic_init(void);


void ultrasonic_interrupt_init(void);

#endif /* _ULTRASONIC_H_ */

/*---------------------------------------------------------------------------
*@file ultrasonic.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Referenced:    https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
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

//Object Near Defines

#define OBJECT_NEAR_INCHES      6
#define OBJECT_NEAR_METERS      .1524
#define OBJECT_NEAR_CENTIMETERS 15
#define OBJECT_NEAR_MILLIMETERS 152
#define OBJECT_NEAR_COUNTS      1331

#define RED_LED_ON              P2->OUT |= BIT0;                //turn on red LED
#define RED_LED_OFF             P2->OUT &= ~BIT0;               //turn off red led

#define _ULTRASONIC_                  //define if ultrasonic sensor if connected

//#define _ULTRASONIC_DEBUG_

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

typedef enum
{
    METERS,
    CENTIMETERS,
    MILLIMETERS,
    INCHES,
    COUNTS
}ultrasonic_unit_t;

typedef struct
{
    ultrasonic_unit_t units;
    bool initialized;
    bool trigger;
    float distance;
    uint16_t echo;
    bool object_near;
} ultrasonic_t;




//--------------------------------------------------------------------------------------------
//                              PUBLIC FUNCTION PROTOTYPES
//--------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//void ultrasonic_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   configures GPIO for transmitting trigger and receiving echo.  Enables inbterrupts
//          on echo pin (P5.5) and sets up timer to measure pulse length. 
//
//@param:   void
//@return:  void
//@outcome: hardware configured to measure ultrasonic sensor.
/////////////////////////////////////////////////////////////////////////////////////////////

void ultrasonic_init(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//void ultrasonic_set_units(ultrasonic_unit_t units)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets the units the ultrasonic sensor measures distance in
//
//
//@param:   ultrasonic_unit_t units -- units in inches, mm, cm, meters or counts
//@return:  void
//@outcome: measurment unit set.
/////////////////////////////////////////////////////////////////////////////////////////////

void ultrasonic_set_units(ultrasonic_unit_t units);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void ultrasonic_trigger(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Transmits a 10 us pulse on trigger pin P5.4.
//
//
//@param:   void
//@return:  void
//@outcome: Ultrasonic sensor is triggered.
/////////////////////////////////////////////////////////////////////////////////////////////

void ultrasonic_trigger(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//float ultrasonic_sample(ultrasonic_unit_t units)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:  Sends trigger pulse to sensor and returns the previous ultrasonic measure distance.
//
//
//@param:   ultrasonic_unit_t units -- units of returned measurement.
//@return:  previous ultrasonic distance measurement.
//@outcome: new distance calculated, previous measurement returned.
/////////////////////////////////////////////////////////////////////////////////////////////

float ultrasonic_sample(ultrasonic_unit_t units);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool ultrasonic_object_near(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks if current distance measurment is above or below "object near" threshold.
//
//
//@param:   void
//@return:  bool -- true if object is near
//@outcome: determines if object is in front of rover.
/////////////////////////////////////////////////////////////////////////////////////////////

bool ultrasonic_object_near(void);

#endif /* _ULTRASONIC_H_ */

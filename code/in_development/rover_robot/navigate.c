/*-------------------------------------------------------------------------------------------
*template.c
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

//libraries
#include "navigate.h"

//Custom Libraries

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

rover_state_t rover[2];

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

void get_vehicle_state(void);
rover_mode_t get_vehicle_mode(void);
void set_vehicle_mode(rover_mode_t mode);

//-------------------------------------------------------------------------------------------
//                                 FUNCTION DECLARATIONS
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

void navigate_init(void)
{
    rover[0].mode       = SEARCHING;
    rover[0].range      = ultrasonic_sample(COUNTS);
    rover[0].light      = apds9960_get_light();
    rover[0].direction  = motor_get_direction();
    rover[0].speed      = motor_get_speed();
    rover[0].temp       = temp_read('f');
}

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

void navigate_execute(void)
{

    static bool object_detected = 0;
    static bool go_left = 0;

    get_vehicle_state();                                                    //get our current state

    if(get_vehicle_mode() == HIDING)
    {
        if(rover[0].light > MAX_HIDING_LIGHT_LEVEL)                         //if we've been found
        {
            set_vehicle_mode(SEARCHING);                                    //set our vehicle mode to searching
            motor_go(forward, low, MOTOR_RUN_CONTINUOUS);                     //lets get out of here

            return;
        }
        else                                                                    //we haven't been discovered
        {
            rover[0].mode = HIDING;                                             //continue hiding

            return;
        }
    }
    else if(get_vehicle_mode() == SEARCHING)                                    //if we are currently searching
    {
        if(rover[0].light < MAX_HIDING_LIGHT_LEVEL)                             //if the light level is acceptable
        {
            motor_stop();                                                       //stop the motors
            rover[0].mode = HIDING;                                             //enter hiding mode

            return;
        }

        if(ultrasonic_object_near() && !object_detected)                        //check if object is near us
        {
            object_detected = true;

            if(motor_running() && motor_get_direction() == forward)
            {
                motor_stop();                                                   //stop the motors
                motor_go(reverse, medium, 5);                                      //go in reverse for a second

                return;
            }
        }

        if(!motor_running() && object_detected)
        {
            if(go_left)                                                         //if we are going left
            {
                go_left = 0;                                                    //next time we will go right
                motor_go(left,medium,4);                                             //go left for a second
            }
            else                                                                //if we are going right
            {
                go_left = 1;                                                    //next time we will go left
                motor_go(right,medium,4);                                          //go left
            }

            object_detected = false;
        }
    }
}

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

void put_vehicle_state(void)
{

}

/////////////////////////////////////////////////////////////////////////////////////////////
//void get_vehicle_state(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   updates vehicle state struct with sensor data.
//
//
//@param:   void
//@return:  void
//@outcome: vehicle state measured.
/////////////////////////////////////////////////////////////////////////////////////////////

void get_vehicle_state(void)
{

    //get_imu();
    rover[0].range = ultrasonic_sample(INCHES);
    rover[0].light = apds9960_get_light();
    rover[0].direction = motor_get_direction();
    rover[0].speed = motor_get_speed();
    rover[0].temp = temp_read('f');
}

/////////////////////////////////////////////////////////////////////////////////////////////
//rover_mode_t get_vehicle_mode(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns the rover's operating mode (searching, hiding, idle etc.).
//
//
//@param:   void
//@return:  rover_mode_t -- current operating mode (searching, hiding, idle etc.).
//@outcome: operating mode determined.
/////////////////////////////////////////////////////////////////////////////////////////////

rover_mode_t get_vehicle_mode(void)
{
    return rover[0].mode;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void set_vehicle_mode(rover_mode_t mode)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   set the rover's operating mode.
//
//
//@param:   rover_mode_t mode -- set mode to searching, idle etc.
//@return:  void
//@outcome: rover mode is updated.
/////////////////////////////////////////////////////////////////////////////////////////////

void set_vehicle_mode(rover_mode_t mode)
{
    rover[0].mode = mode;
}

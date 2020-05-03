/*--------------------------------------------------------------------------------------------
*motor.h
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------------------------
*Code Reference:    Example code msp432p401x_portmap_01 (PWM port mapping)     
*
**------------------------------------------------------------------------------------------*/

#ifndef _MOTOR_H_
#define _MOTOR_H_

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

#define AIN1                    0x0080              //P2.7 PWM/GPIO
#define AIN2                    0x0040              //P2.6 PWM/GPIO
#define BIN1                    0x0010              //P2.4 PWM/GPIO
#define BIN2                    0x0020              //P2.5 PWM/GPIO
#define SLEEP_N                 0x0002              //P5.1
#define FAULT_N                 0x0020              //P3.5

#define TIMER_MS                25UL
#define SEC_TO_OVCOUNT(x)       (float)(x * 1000 / TIMER_MS)

#define MOTOR_RUN_CONTINUOUS    255

#define PWM_FREQ                16000UL
#define PWM_CCR0_COUNTS         ( F_CPU / (PWM_FREQ * 2 ) )

#define PWM_FREQ_TO_COUNTS(frequency)       ( F_CPU / ( frequency * 2 ) )
#define DUTY_CYCLE_TO_COUNTS(duty_cycle)    (uint16_t)( PWM_CCR0_COUNTS - PWM_CCR0_COUNTS * duty_cycle / 100 )

#define GRN_LED_ON              P2->OUT |= BIT1
#define GRN_LED_OFF             P2->OUT &= ~BIT1

#define MOTOR_SPEED_UP_DELTA       6
#define MOTOR_SPEED_DOWN_DELTA     15

//#define _MOTOR_DEBUG_

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

typedef enum{
    continuous = 15,
}motor_time_t;

typedef enum{
    forward,
    reverse,
    left,
    right,
    stop
}direction_t;

typedef enum{
    zero = 0,
    very_low = 15,
    low = 35,
    medium = 50,
    high = 75,
    full = 100
}speed_t;

typedef struct{
    uint8_t speed;
    uint8_t speed_desired;
    uint8_t time;
    direction_t direction;
    direction_t direction_desired;
    bool running;
    bool controller_active;
    bool sleep;
    bool fault;
}status_t;

//-------------------------------------------------------------------------------------------
//                              PUBLIC FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   configures I/O to driver/read motor driver.  P2.7 = AIN1, P2.6 = AIN2,
//          P2.4 = BIN1, P2.5 = BIN2, P5.1 = SLEEP_N and P3.5 = FAULT_N.
//          sets all output pins low, and puts motor driver to sleep.
//
//@param:   void
//@return:  void
//@outcome: I/O configured to control H-Bridge and driver is put to sleep.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_init(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_pwm_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   configured TIMER_A1 to generate 16 KHz PWM on AIN1, AIN2, BIN1 and BIN2 pins.
//
//
//@param:   void
//@return:  void
//@outcome: PWM signals are mapped to secondary functions on motor control pins.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_pwm_init(void);


/////////////////////////////////////////////////////////////////////////////////////////////
//direction_t motor_get_direction(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns the current motor drive direction
//
//
//@param:   void
//@return:  direction_t -- stopped, forward, reverse, left and right.
//@outcome: drive direction determined.
/////////////////////////////////////////////////////////////////////////////////////////////

direction_t motor_get_direction(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_set_direction_desired(direction_t direction)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets the desired direction of the motors (not the actual direction).  The motor
//          controller will adjust the actual direction to match the desired.
//
//@param:   direction_t direction -- set the desired motor direction.
//@return:  void
//@outcome: desired direction modified.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_set_direction_desired(direction_t direction);

/////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t motor_get_speed(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   gets the current motor operating speed.
//
//
//@param:   void
//@return:  uint8_t -- speed from 0-100%
//@outcome: motor speed determined.
/////////////////////////////////////////////////////////////////////////////////////////////

uint8_t motor_get_speed(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_set_speed(uint8_t speed)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets the motor speed directly, updates PWM duty cycle to alter speed.
//
//
//@param:   uint8_t speed -- 0 - 100% speed value
//@return:  void
//@outcome: motor speed is altered immediately (overrides controller)
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_set_speed(uint8_t speed);

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_set_speed_desired(uint8_t speed)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets the desired motor speed, to be reached with the motor controller.
//
//
//@param:   uint8_t speed -- 0-100% desired speed.
//@return:  void
//@outcome: desired motor speed set.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_set_speed_desired(uint8_t speed);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool motor_running(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks whether the motors are running or not.  Returns true if they are.
//
//
//@param:   void
//@return:  bool -- true if running, false if stopped.
//@outcome: motor run state determined.
/////////////////////////////////////////////////////////////////////////////////////////////

bool motor_running(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool motor_fault(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks status of fault line and updates fault variable.
//
//
//@param:   void
//@return:  bool -- state of fault line.
//@outcome: fault status is determined.
/////////////////////////////////////////////////////////////////////////////////////////////

bool motor_fault(void);

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

void motor_stop(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool motor_controller_active(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks if the motor controller is currently running.
//
//
//@param:   void
//@return:  bool -- returns true of motor controller is currently active
//@outcome: motor controller state determined.
/////////////////////////////////////////////////////////////////////////////////////////////

bool motor_controller_active(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_controller(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   open loop motor speed and direction controller.  handles speeding motor up
//          and down at a constant rate, and adjusting motor direction.
//
//@param:   void
//@return:  void
//@outcome: motor speed is actively controlled and direction changes are handled.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_controller(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool motor_go(direction_t direction, uint8_t speed, float time_sec)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   overrides motor controller and immediately sets the motor direction, speed 
//          and run time.
//  
//@param:   direction_t direction, uint8_t speed, float time_sec -- direction, speed and 
//          run time.
//@return:  void
//@outcome: rover drives in specified direction, at speed for period of time.
/////////////////////////////////////////////////////////////////////////////////////////////

bool motor_go(direction_t direction, uint8_t speed, float time_sec);

#endif  //_MOTOR_H_

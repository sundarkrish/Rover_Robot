/*-------------------------------------------------------------------------------------------
*motor.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*--------------------------------------------------------------------------------------------
*Code Reference:    Example code msp432p401x_portmap_01 (PWM port mapping)         
*
**-----------------------------------------------------------------------------------------*/


//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//libraries
#include "motor.h"
#include "timer.h"
#include "clock.h"

//Custom Libraries

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

volatile status_t motor = {zero, zero, 0, stop, stop, 0, 0, 0, 1};

uint8_t overflow_count = 0;
uint16_t stop_overflow_count = 0;

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

void motor_sleep(void);
void motor_wake(void);
void motor_set(bool ain1, bool ain2, bool bin1, bool bin2);
void motor_forward(void);
void motor_reverse(void);
void motor_left(void);
void motor_right(void);

//-------------------------------------------------------------------------------------------
//                                 FUNCTION DECLARATIONS
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

void motor_init(void)
{
    timer_init(timer2,interrupts);
    motor_pwm_init();

    //set direction of pins
    P2->DIR |= (AIN1 | AIN2 | BIN1 | BIN2);                             //set AIN1, AIN2 and BIN1 to outputs
    P5->DIR |= (SLEEP_N);                                               //set BIN2 and SLEEP_N to outputs
    P3->DIR &= ~FAULT_N;                                                //set FAULT_N pin to input

    //set pins to low before setting their direction
    P2->OUT |= (AIN1 | AIN2 | BIN1 | BIN2);                             //set all driver control signals high
    P5->OUT &= ~(SLEEP_N);                                              //set BIN2 low, put driver to sleep

    //set pulldowns/pullups on pins
    P3->OUT |= FAULT_N;                                                 //select pullups on fault pin
    P3->REN |= FAULT_N;                                                 //enable pullups

    // enable button press interrupts
    P3->IES  |= FAULT_N;                                                // P1IFG flag set on high to low transition of P1.4
    P3->IFG  = 0;                                                       // lets go ahead and clear any pending interrupts
    P3->IE   |= FAULT_N;                                                // enable interrupts

    // Enable timer and port interrupts on the NVIC
    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31);                           // Enable Port 3 interrupt on the NVIC

    //set pins to GPIO
    P2->SEL0 &= ~(AIN1 | AIN2 | BIN1 | BIN2);                           //set port 2 function to GPIO
    P2->SEL1 &= ~(AIN1 | AIN2 | BIN1 | BIN2);

    P5->SEL0 &= ~(SLEEP_N);                                             //set port 5 function to GPIO
    P5->SEL1 &= ~(SLEEP_N);

    P3->SEL0 &= ~FAULT_N;                                               //set port 3 function to GPIO
    P3->SEL1 &= ~FAULT_N;

    motor_wake();                                                       //wake up the h-bridge
}

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

void motor_pwm_init(void)
{
    PMAP->KEYID = PMAP_KEYID_VAL;                                       //unlock the port map registers

    //pin map timer 2 CCR1A output to connect to P2.4,6,7 and P5.6
    P2MAP->PMAP_REGISTER4 = PMAP_TA1CCR1A;
    P2MAP->PMAP_REGISTER6 = PMAP_TA1CCR1A;
    P2MAP->PMAP_REGISTER7 = PMAP_TA1CCR1A;
    P2MAP->PMAP_REGISTER5 = PMAP_TA1CCR1A;

    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_OUTMOD_4;                         //setup timer 1 to output PWM
    TIMER_A1->CCR[0]  = PWM_FREQ_TO_COUNTS(PWM_FREQ);                                            //PWM Period/2
    TIMER_A1->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6;                         //CCTL[1] generates PWM
    TIMER_A1->CCR[1]  = DUTY_CYCLE_TO_COUNTS(low);                                            //25% duty cycle

    TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC_3;          // Up-down mode with ACKL as source


    PMAP->KEYID = 0;                                                    //lock the port map registers
}

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

direction_t motor_get_direction(void)
{
    return motor.direction;
}

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

void motor_set_direction_desired(direction_t direction)
{
    motor.direction_desired = direction;
}

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

bool motor_running(void)
{
    return motor.running;
}

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

uint8_t motor_get_speed(void)
{
    return motor.speed;
}

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

void motor_set_speed(uint8_t speed)
{
    if(speed > 100)
    {
        speed = 100;
    }

    TIMER_A1->CCR[1]  = DUTY_CYCLE_TO_COUNTS((100 - speed));            //update the PWM value

    motor.speed = speed;                                            //update vehicle speed value
}

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

void motor_set_speed_desired(uint8_t speed)
{
    if(speed > 100)
    {
        speed = 100;
    }

    motor.speed_desired = speed;                                   //update vehicle speed value
}

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

bool motor_fault(void)
{
    motor.fault = (P3->IN & FAULT_N);                                   //update fault state

#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > motor.fault = %s\n\r",motor.fault ? "true" : "false");
#endif

    return motor.fault;                                                 //return fault state
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_wake(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   wakes up the h-bridge so motors can be driven.
//
//
//@param:   void
//@return:  void
//@outcome: motor driver is in ready state.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_wake(void)
{

    P5->OUT |= SLEEP_N;                                                 //wake up h-bridge
    motor.sleep = false;

    delay_ms(1);                                                        //give h-bridge time to wake
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_sleep(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   puts h-bridge into low power state.
//
//
//@param:   void
//@return:  void
//@outcome: motor driver is in sleep state.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_sleep(void)
{
    P5->OUT &= ~SLEEP_N;                                               //put h-bridge driver to sleep
    motor.sleep = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_set(bool ain1, bool ain2, bool bin1, bool bin2)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   LUT generates the appropriate PWM or logic high signal to drive motor
//          in any direction.
//
//@param:   bool ain1, bool ain2, bool bin1, bool bin2 -- four bits for controlling motor direction
//@return:  void
//@outcome: motors run in desirec direction.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_set(bool ain1, bool ain2, bool bin1, bool bin2)
{
    //set the H-Bridge inputs accordingly
//    ain1 ? (P2->OUT |= AIN1) : (P2->OUT &= ~AIN1);                      //set AIN1 high or low
//    ain2 ? (P2->OUT |= AIN2) : (P2->OUT &= ~AIN2);                      //set AIN2 high or low
//    bin1 ? (P2->OUT |= BIN1) : (P2->OUT &= ~BIN1);                      //set BIN1 high or low
//    bin2 ? (P5->OUT |= BIN2) : (P5->OUT &= ~BIN2);                      //set BIN2 high or low

    if(ain1){ P2->SEL0 |=  AIN1; P2->SEL1 &= ~AIN1; }                       //if we are enabling PWM, select peripheral output
    else    { P2->SEL0 &= ~AIN1; P2->SEL1 &= ~AIN1; }                       //if we are disabling PWM, select GPIO

    if(ain2){ P2->SEL0 |=  AIN2; P2->SEL1 &= ~AIN2; }                       //if we are enabling PWM, select peripheral output
    else    { P2->SEL0 &= ~AIN2; P2->SEL1 &= ~AIN2; }                       //if we are disabling PWM, select GPIO

    if(bin1){ P2->SEL0 |=  BIN1; P2->SEL1 &= ~BIN1; }                       //if we are enabling PWM, select peripheral output
    else    { P2->SEL0 &= ~BIN1; P2->SEL1 &= ~BIN1; }                       //if we are disabling PWM, select GPIO

    if(bin2){ P2->SEL0 |=  BIN2; P2->SEL1 &= ~BIN2; }                       //if we are enabling PWM, select peripheral output
    else    { P2->SEL0 &= ~BIN2; P2->SEL1 &= ~BIN2; }                       //if we are disabling PWM, select GPIO
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_forward(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   h-bridge configured to drive motors forward
//
//
//@param:   void
//@return:  void
//@outcome: motors drive forward.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_forward(void)
{
    motor_set(1,0,1,0);                                                 //set motor forward

    motor.running = true;                                               //update running flag
    motor.direction = forward;                                          //update direction status

#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > driving forward\n\r");
#endif

}

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_reverse(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   h-bridge configured to drive motors in reverse
//
//
//@param:   void
//@return:  void
//@outcome: motors drive in reverse.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_reverse(void)
{
    motor_set(0,1,0,1);

    motor.running = true;
    motor.direction = reverse;

#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > driving reverse\n\r");
#endif

}

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_left(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   h-bridge configured to drive motors left
//
//
//@param:   void
//@return:  void
//@outcome: motors drive left.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_left(void)
{
    motor_set(0,1,1,0);

    motor.running = true;
    motor.direction = left;

#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > driving left\n\r");
#endif

}

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_right(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   h-bridge configured to drive motors right
//
//
//@param:   void
//@return:  void
//@outcome: motors drive right.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_right(void)
{
    motor_set(1,0,0,1);

    motor.running = true;
    motor.direction = right;

#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > driving right\n\r");
#endif

}

/////////////////////////////////////////////////////////////////////////////////////////////
//void motor_stop(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   stops the motors from running.
//
//
//@param:   void
//@return:  void
//@outcome: rover stops moving.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_stop(void)
{

    motor_set(0,0,0,0);                             //set motors to zero
    //motor_sleep();                                //putting h-bridge to sleep causes fault on wakeup

    motor.speed = zero;                             //motor speed is now zero
    motor.direction = stop;                         //set motor direction to stopped
    motor.running = false;                          //running is false

    GRN_LED_OFF;                                    //turn off green LED

#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > motor stopped\n\r");
#endif

}

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

bool motor_controller_active(void)
{
    return motor.controller_active;
}

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

void motor_controller(void)
{

    if((motor.direction == motor.direction_desired) && motor.speed == motor.speed_desired)
    {
        motor.controller_active = false;

        return;
    }
    else                                                                            //we are changing speed or direction
    {
        motor.controller_active = true;

        /*Motor Speed Controller */

        if(motor.speed_desired > motor.speed)                                       //if desired speed is higher
        {
            if(motor.speed + MOTOR_SPEED_UP_DELTA > full)                              //if next motor speed is greater than 100%
            {
                motor.speed = full;                                                 //make sure speed never goes above 100
            }
            else if(motor.speed + MOTOR_SPEED_UP_DELTA > motor.speed_desired)          //if we are one step away from speed_desired
            {
                motor.speed = motor.speed_desired;                                  //just set the motor.speed to speed_desired
                TIMER_A1->CCR[1]  = DUTY_CYCLE_TO_COUNTS((100 - motor.speed));      //update the PWM value
            }
            else                                                                    //otherwise, increase the motor speed
            {
                motor.speed += MOTOR_SPEED_UP_DELTA;                                   //speed ramp rate
                TIMER_A1->CCR[1]  = DUTY_CYCLE_TO_COUNTS((100 - motor.speed));      //update the PWM value
            }
        }
        else                                                                        //else we are slowing down
        {
            if(motor.speed - MOTOR_SPEED_DOWN_DELTA < zero)
            {
                motor.speed = zero;                                                 //set motor speed to zero
                motor_stop();                                                       //stop the motors
            }
            else
            {
                motor.speed -= MOTOR_SPEED_DOWN_DELTA;                                   //slow down at controlled rate
                TIMER_A1->CCR[1]  = DUTY_CYCLE_TO_COUNTS((100 - motor.speed));      //update the PWM value
            }
        }

        /*Motor Direction Controller */

        if(motor.direction == stop)                                         //if we have come to a stop, we can change direction
        {
            motor.direction = motor.direction_desired;                      //set our current direction equal to desired
            GRN_LED_ON;                                                     //green LED on when motor is running


            switch(motor.direction)
            {
                case forward: { motor_forward(); break; }                   //go forward
                case reverse: { motor_reverse(); break; }                   //go back
                case left:    { motor_left();    break; }                   //go left
                case right:   { motor_right();   break; }                   //go right
                default:      { motor_stop();           }                   //stop the motor
            }
        }
        else if(motor.direction != motor.direction_desired)                 //if we need to change directions
        {
            motor_set_speed_desired(zero);                                  //set the desired speed to zero so we come to a stop
        }


    }
}

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

bool motor_go(direction_t direction, uint8_t speed, float time_sec)
{
    //@TODO some sort of bug here, if I try to invert motor.fault it doesn't work
    //      need to investigate.  Fix for now it to invert the output of motor_fault();

//    if(!motor_fault())                                       //if we are in a fault state
//    {
//        return 0;                                           //don't run motors, return
//    }
//    else
//    {
//        motor_wake();                                       //wake up the h-bridge
//    }

    //turn on green LED to show motor is running
    GRN_LED_ON;                                             //green LED on when motor is running

    //set the motor speed
    motor_set_speed(speed);                                 //set the PWM duty cycle aka motor speed

    switch(direction)
    {
        case forward: { motor_forward(); break; }           //go forward
        case reverse: { motor_reverse(); break; }           //go back
        case left:    { motor_left();    break; }           //go left
        case right:   { motor_right();   break; }           //go right
        default:      { motor_stop();           }
    }

    stop_overflow_count = SEC_TO_OVCOUNT(time_sec);         //calculate number of timer overflows for x seconds

    if(time_sec == MOTOR_RUN_CONTINUOUS )                   //if motors is to run continuous
    {
        timer_stop(timer2);
        timer_clear(timer2);                                //motor runs indefinitely
    }
    else                                                    //else number of seconds very large, just run motors forever
    {
        timer_start(timer2,MSEC_TO_COUNTS(TIMER_MS));       //set a timer and go
    }

    return 1;
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

void TA2_N_IRQHandler(void)
{
    if(timer_overflow(timer2))
    {
        timer_overflow_flag_clear(timer2);
        overflow_count++;

        if(overflow_count == stop_overflow_count)
        {
            overflow_count = 0;
            timer_stop(timer2);
            timer_clear(timer2);
            motor_stop();
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

void PORT3_IRQHandler(void)
{
    P3->IFG  &= ~FAULT_N;                                           //clear Port 3 interrupt flag
    motor_stop();                                                   //stop motor from running

    motor.fault = true;
#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > ERROR: > motor driver fault triggered\n\r");
#endif //_MOTOR_DEBUG_

}

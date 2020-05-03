/*-------------------------------------------------------------------------------------------
*motor.c
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
#include "motor.h"
#include "timer.h"

//Custom Libraries

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

volatile status_t motor = {0, 0, stopped, 0, 0, 1};

uint8_t overflow_count = 0;
uint16_t stop_count = 0;

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
//          P2.4 = BIN1, P5.6 = BIN2, P5.1 = SLEEP_N and P3.5 = FAULT_N.
//          sets all output pins low, and puts motor driver to sleep.
//
//@param:   void
//@return:  void
//@outcome: I/O configured to control H-Bridge and driver is put to sleep.
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_init(void)
{
    timer_init(timer1,interrupts);

    //set direction of pins
    P2->DIR |= (AIN1 | AIN2 | BIN1);                                    //set AIN1, AIN2 and BIN1 to outputs
    P5->DIR |= (BIN2 | SLEEP_N);                                        //set BIN2 and SLEEP_N to outputs
    P3->DIR &= ~FAULT_N;                                                //set FAULT_N pin to input

    //set pins to low before setting their direction
    P2->OUT &= ~(AIN1 | AIN2 | BIN1);                                   //set all driver contorl signals low
    P5->OUT &= ~(BIN2 | SLEEP_N);                                       //set BIN2 low, put driver to sleep

    //set pulldowns/pullups on pins
    P3->OUT |= FAULT_N;                                                 //select pullups on fault pin
    P3->REN |= FAULT_N;                                                 //enable pullups

    // enable button press interrupts
    P3->IES  |= FAULT_N;                                                // P1IFG flag set on high to low transition of P1.4
    P3->IFG  = 0;                                                       // lets go ahead and clear any pending interrupts
    P3->IE   |= FAULT_N;                                                // enable interrupts


    // Enable timer and port interrupts on the NVIC
    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31);                           // Enable Port 1 interrupt on the NVIC

    //set pins to GPIO
    P2->SEL0 &= ~(AIN1 | AIN2 | BIN1);                                  //set port 2 function to GPIO
    P2->SEL1 &= ~(AIN1 | AIN2 | BIN1);

    P5->SEL0 &= ~(BIN2 | SLEEP_N);                                      //set port 5 function to GPIO
    P5->SEL1 &= ~(BIN2 | SLEEP_N);


    P3->SEL0 &= ~FAULT_N;                                               //set port 3 function to GPIO
    P3->SEL1 &= ~FAULT_N;
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

bool motor_fault(void)
{
    motor.fault = (P3->IN & FAULT_N);                                   //update fault state

#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > motor.fault = %s\n\r",motor.fault ? "true" : "false");
#endif

    return motor.fault;                                                 //return fault state
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

void motor_wake(void)
{

    P5->OUT |= SLEEP_N;                                               //wake up h-bridge
    motor.sleep = false;


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

void motor_sleep(void)
{
    P5->OUT &= ~SLEEP_N;                                               //put h-bridge driver to sleep
    motor.sleep = true;
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

void motor_set(bool ain1, bool ain2, bool bin1, bool bin2)
{
    //set the H-Bridge inputs accordingly
    ain1 ? (P2->OUT |= AIN1) : (P2->OUT &= ~AIN1);                      //set AIN1 high or low
    ain2 ? (P2->OUT |= AIN2) : (P2->OUT &= ~AIN2);                      //set AIN2 high or low
    bin1 ? (P2->OUT |= BIN1) : (P2->OUT &= ~BIN1);                      //set BIN1 high or low
    bin2 ? (P5->OUT |= BIN2) : (P5->OUT &= ~BIN2);                      //set BIN2 high or low
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
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
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
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
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
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
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
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

void motor_stop(void)
{

    motor_set(0,0,0,0);
    motor_sleep();

    motor.direction = stopped;
    motor.running = false;

#ifdef _MOTOR_DEBUG_
    printf("MOTOR.C: > motor stopped\n\r");
#endif

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

bool motor_go(uint8_t direction, uint8_t speed, uint8_t time_sec)
{
    //@TODO some sort of bug here, if I try to invert motor.fault it doesn't work
    //      need to investigate.  Fix for now it to invert the output of motor_fault();
    if(!motor_fault())                                       //if we are in a fault state
    {
        return 0;
    }
    else
    {
        motor_wake();                                       //wake up the h-bridge
    }



    //pass speed to functions
    switch(direction)
    {
        case forward:
        {
            motor_forward();
            break;
        }
        case reverse:
        {
            motor_reverse();
            break;
        }
        case left:
        {
            motor_left();
            break;
        }
        case right:
        {
            motor_right();
            break;
        }
        default:
        {
            motor_stop();
        }
    }

    stop_count = 40*time_sec;                                 //write time in seconds into stop_count variable

    if(time_sec < 15)                                       //if motors are running for less than 15 seconds
    {
        timer_start(timer1,MSEC_TO_COUNTS(25));             //set a timer and go
    }
    else
    {
        timer_stop(timer1);
        timer_clear(timer1);                                //motor runs indefinitely
    }

    return 1;
}

//@TODO create a timer ISR that calls motor stop after a certain period of time

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

void TA1_N_IRQHandler(void)
{
    if(timer_overflow(timer1))
    {
        timer_overflow_flag_clear(timer1);
        overflow_count++;

        if(overflow_count == stop_count)
        {
            overflow_count = 0;
            timer_stop(timer1);
            timer_clear(timer1);
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

    //motor.fault = true;

    printf("MOTOR.C: > ERROR: > motor driver fault triggered, stopping motors\n\r");
}

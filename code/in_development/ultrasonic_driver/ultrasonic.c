/*---------------------------------------------------------------------------
*ultrasonic.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reuse:
*
**--------------------------------------------------------------------------*/

//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//Custom Libraries
#include "ultrasonic.h"
#include "timer.h"
#include "clock.h"



//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

ultrasonic_t range_sensor;

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

inline void ultrasonic_trigger(void);
inline bool ultrasonic_echo(void);
void ultrasonic_interrupt_init(void);

//-------------------------------------------------------------------------------------------
//                                 FUNCTION DECLARATIONS
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

void ultrasonic_init(void)
{
    timer_init(timer0,no_interrupts);                                       //setup timer

    //initialize port P2.6 as trigger, and P2.7 as echo
    P5->DIR |= TRIGGER;                                 //set trigger as P5.4 (output)
    P5->DIR &= ~ECHO;                                   //set echo as P5.5 (input)
    P5->OUT &= ~(TRIGGER | ECHO);                       //set pulldowns on echo pin and trigger = 0
    P5->REN |= ECHO;                                    //enable pulldown on echo pin

    P5->SEL0 &= ~(TRIGGER | ECHO);                      //set port function to GPIO
    P5->SEL1 &= ~(TRIGGER | ECHO);

    //ultrasonic_interrupt_init();                      //echo signal generates an interrupt
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

void ultrasonic_interrupt_init(void)
{
    // enable button press interrupts
    P5->IES  &= ~ECHO;                                      // P2IFG flag set on low to high transition of echo
    P5->IE   |= ECHO;                                       // enable interrupts on echo signal
    P5->IFG  = 0;                                           // lets go ahead and clear any pending interrupts

    // Enable timer and port interrupts on the NVIC
    NVIC->ISER[1] = 1 << ((PORT5_IRQn) & 31);               // Enable Port 2 interrupt on the NVIC
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

inline void ultrasonic_trigger(void)
{

    CLR_TRIGGER;                        //make sure trigger is low
    delay_us(10);                       //delay for a bit
    SET_TRIGGER;                        //set trigger high
    delay_us(10);                       //delay for 10 us
    CLR_TRIGGER;                        //set trigger low again
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

inline bool ultrasonic_echo(void)
{
    return (P5->IN & ECHO);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//double ultrasonic_getdist(uint8_t units)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   BLOCKING.
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

double ultrasonic_getdist(uint8_t units)
{
    uint16_t stop_time = 0;

    ultrasonic_trigger();                           //send out trigger pulse
    timer_start(timer0,MAX_DIST);                          //start counter, set timer to count to maximum distance time

    while(!ultrasonic_echo())                         //while we haven't received an echo
    {
        //if timer overflows, return 0
        if(timer_overflow(timer0))                        //if an echo wasn't received in maximum allowable time
        {
            return -1;                              //the echo isn't coming return -1
        }
    }

    //echo received record timer value
    timer_start(timer0,MAX_DIST);                          //start timer again

    while(ultrasonic_echo());                         //while echo is high

    timer_stop(timer0);                                   //make sure timer isn't running
    stop_time = timer_getval(timer0);                     //get the current value of the timer
    timer_clear(timer0);                                  //clear the timer for next user

    switch(units)
    {
        case INCHES:
        {
            return ((.667*stop_time) / 148);
        }
        case METERS:
        {
            return ((.667*stop_time*340) / 2);
        }
        case CENTIMETERS:
        {
            return ((.667*stop_time) / 58);
        }
        case MILLIMETERS:
        {
            return ((.667*stop_time) / 580);
        }
        default:
        {
            return ((.667*stop_time) / 148);
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

void PORT5_IRQHandler(void)
{
    uint16_t stop_time;

    P5->IE   &= ~ECHO;                                      // disable interrupts on echo signal
    P5->IFG  &= ~ECHO;                                      // clear echo interrupt flag

    //echo received record timer value
    timer_start(timer2,MAX_DIST);                                  //start timer again

    while(ultrasonic_echo());                               //while echo is high

    timer_stop(timer2);                                           //make sure timer isn't running
    stop_time = timer_getval(timer2);                             //get the current value of the timer
    timer_clear(timer2);                                          //clear the timer for next user

    range_sensor.distance = ((.667*stop_time*340) / 2);

    #ifdef _ultrasonic_debug
        printf("Object Distance: %.2lf (inches)\n\r",range_sensor.distance);
    #endif

    P5->IE   |= ECHO;                                       // enable interrupts on echo signal

}

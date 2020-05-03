/*---------------------------------------------------------------------------
*template.c
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

//libraries
#include "timer.h"

//Custom Libraries

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//void timer_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   configures timer_A0 as a simple up counter with SMCLK as the source and a clock
//          divider of 8.  If interrupts are enabled the timer overflow will cause it's
//          respective interrupt to fire (i.e. TA0_N_IRQHandler()...)
//
//@param:   void
//@return:  void
//@outcome: timer_A0 is configured, and interrupts are enabled or disabled.
/////////////////////////////////////////////////////////////////////////////////////////////

//@TODO make timer_init pass uint16_t CCR and CTL so users can configure the timers as needed

bool timer_init(uint8_t timer, bool interrupts)
{

    switch(timer)
    {
        case timer0:
        {
            TIMER_A0->CCR[0] = 0x0000;                                          //make sure timer is not counting
            TIMER_A0->CTL |= ( TIMER_A_CTL_SSEL__SMCLK |
                               TIMER_A_CTL_ID__8);         //set clock source as SMCLK and count to up

            if(interrupts)
            {
                timer_overflow_flag_clear(timer);
                TIMER_A0->CTL |= TIMER_A_CTL_IE;                                // TA0 interrupt enabled
                NVIC->ISER[0] = 1 << ((TA0_N_IRQn) & 31);                       //enable TA0_N_IRQHandler
            }

            break;
        }
        case timer1:
        {
            TIMER_A1->CCR[0] = 0x0000;                                          //make sure timer is not counting
            TIMER_A1->CTL |= ( TIMER_A_CTL_SSEL__SMCLK |
                               TIMER_A_CTL_ID__8);         //set clock source as SMCLK and count to up

            if(interrupts)
            {
                timer_overflow_flag_clear(timer);
                TIMER_A1->CTL |= TIMER_A_CTL_IE;                                // TA1 interrupt enabled
                NVIC->ISER[0] = 1 << ((TA1_N_IRQn) & 31);                       //enable TA1_N_IRQHandler
            }

            break;
        }
        case timer2:
        {
            TIMER_A2->CCR[0] = 0x0000;                                          //make sure timer is not counting
            TIMER_A2->CTL |= ( TIMER_A_CTL_SSEL__SMCLK |
                               TIMER_A_CTL_ID__8);         //set clock source as SMCLK and count to up

            if(interrupts)
            {
                timer_overflow_flag_clear(timer);
                TIMER_A2->CTL |= TIMER_A_CTL_IE;                                // TA2 interrupt enabled
                NVIC->ISER[0] = 1 << ((TA2_N_IRQn) & 31);                       //enable TA2_N_IRQHandler
            }

            break;
        }
        case timer3:
        {
            TIMER_A3->CCR[0] = 0x0000;                                          //make sure timer is not counting
            TIMER_A3->CTL |= ( TIMER_A_CTL_SSEL__SMCLK |
                               TIMER_A_CTL_ID__8);         //set clock source as SMCLK and count to up

            if(interrupts)
            {
                timer_overflow_flag_clear(timer);
                TIMER_A3->CTL |= TIMER_A_CTL_IE;                                // TA3 interrupt enabled
                NVIC->ISER[0] = 1 << ((TA3_N_IRQn) & 31);                       //enable TA3_N_IRQHandler
            }

            break;
        }
        default:
        {
            #ifdef _TIMER_DEBUG_
                printf("TIMER.C: > timer A%d not set\n\r",timer);
            #endif

            return 0;
        }
    }

    #ifdef _TIMER_DEBUG_
        printf("TIMER.C: > timer A%d was setup\n\r",timer);
    #endif

    return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void timer_start(uint16_t val)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   starts timer and sets CCR0 to desired overflow value.
//
//
//@param:   uint16_t val -- count value which timer will overflow
//@return:  void
//@outcome: timer starts running and CCR0 set.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_start(uint8_t timer,uint16_t val)
{
    timer_stop(timer);                                                      //stop timer (if it was running)
    timer_clear(timer);                                                     //clear any value in timer

    if(timer == timer0)
    {
        TIMER_A0->CCR[0] = val;                                             //set counter value
        TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;                                //start counter
    }
    else if(timer == timer1)
    {
        TIMER_A1->CCR[0] = val;                                             //set counter value
        TIMER_A1->CTL |= TIMER_A_CTL_MC__UP;                                //start counter
    }
    else if(timer == timer2)
    {
        TIMER_A2->CCR[0] = val;                                             //set counter value
        TIMER_A2->CTL |= TIMER_A_CTL_MC__UP;                                //start counter
    }
    else if(timer == timer3)
    {
        TIMER_A3->CCR[0] = val;                                             //set counter value
        TIMER_A3->CTL |= TIMER_A_CTL_MC__UP;                                //start counter
    }
    else
    {
        #ifdef _TIMER_DEBUG_
            printf("TIMER.C: > timer A%d not started\n\r",timer);
        #endif

        return 0;
    }

    #ifdef _TIMER_DEBUG_
        printf("TIMER.C: > timer A%d started\n\r",timer);
    #endif

    return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void timer_stop(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   stops timer from running by setting MC bits to 0
//
//
//@param:   void
//@return:  void
//@outcome: timer stopped
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_stop(uint8_t timer)
{
    if(timer == timer0)
    {
        TIMER_A0->CTL &= TIMER_A_STOP;                                        //stop counter
    }
    else if(timer == timer1)
    {
        TIMER_A1->CTL &= TIMER_A_STOP;                                        //stop counter
    }
    else if(timer == timer2)
    {
        TIMER_A2->CTL &= TIMER_A_STOP;                                        //stop counter
    }
    else if(timer == timer3)
    {
        TIMER_A3->CTL &= TIMER_A_STOP;                                        //stop counter
    }
    else
    {
        #ifdef _TIMER_DEBUG_
            printf("TIMER.C: > timer A%d not stopped\n\r",timer);
        #endif

        return 0;
    }

    #ifdef _TIMER_DEBUG_
        printf("TIMER.C: > timer A%d stopped\n\r",timer);
    #endif

    return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void timer_clear(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   clears the timer count register and overflow flag.  MUST BE CALLED AFTER TIMER
//          HAS BEEN STOPPED.
//
//@param:   void
//@return:  void
//@outcome: overflow flag cleared and timer value reset to 0
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_clear(uint8_t timer)
{

    if(timer == timer0)
    {
        TIMER_A0->R = 0x0000;                                   //clear the current timer count
        TIMER_A0->CTL &= ~TIMER_A_CTL_IFG;                      //clear overflow flag too
    }
    else if(timer == timer1)
    {
        TIMER_A1->R = 0x0000;                                   //clear the current timer count
        TIMER_A1->CTL &= ~TIMER_A_CTL_IFG;                      //clear overflow flag too
    }
    else if(timer == timer2)
    {
        TIMER_A2->R = 0x0000;                                   //clear the current timer count
        TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;                      //clear overflow flag too
    }
    else if(timer == timer3)
    {
        TIMER_A3->R = 0x0000;                                   //clear the current timer count
        TIMER_A3->CTL &= ~TIMER_A_CTL_IFG;                      //clear overflow flag too
    }
    else
    {
        #ifdef _TIMER_DEBUG_
            printf("TIMER.C: > timer A%d not cleared\n\r",timer);
        #endif

        return 0;
    }

    return 1;
}


/////////////////////////////////////////////////////////////////////////////////////////////
//inline uint16_t timer_getval(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   gets the current value of the timer counter.  TIMER MUST BE STOPPED BEFORE.
//
//
//@param:   uint8_t timer -- the timer which we are reading
//@return:  uint16_t -- value of timer
//@outcome: timer value read.
/////////////////////////////////////////////////////////////////////////////////////////////

inline uint16_t timer_getval(uint8_t timer)
{
    if(timer == timer0)
    {
        return TIMER_A0->R;
    }
    else if(timer == timer1)
    {
        return TIMER_A1->R;
    }
    else if(timer == timer2)
    {
        return TIMER_A2->R;
    }
    else if(timer == timer3)
    {
        return TIMER_A3->R;
    }
    else
    {
        #ifdef _TIMER_DEBUG_
            printf("TIMER.C: > timer A%d not read\n\r",timer);
        #endif

            return 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline bool timer_overflow(uint8_t timer)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns value of timer overflow flag.
//
//
//@param:   uint8_t timer -- the timer we are checking
//@return:  bool -- true if timer overflow
//@outcome: returns overflow status of timer.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_overflow(uint8_t timer)
{
    if(timer == timer0)
    {
        return (TIMER_A0->CTL & TIMER_A_CTL_IFG);
    }
    else if(timer == timer1)
    {
        return (TIMER_A1->CTL & TIMER_A_CTL_IFG);
    }
    else if(timer == timer2)
    {
        return (TIMER_A2->CTL & TIMER_A_CTL_IFG);
    }
    else if(timer == timer3)
    {
        return (TIMER_A3->CTL & TIMER_A_CTL_IFG);
    }
    else
    {
        #ifdef _TIMER_DEBUG_
            printf("TIMER.C: > timer A%d overflow flag not checked\n\r",timer);
        #endif

        return 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline bool timer_overflow_flag_clear(uint8_t timer)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   clears the overflow flag of a selected timer
//
//
//@param:   uint8_t timer -- timer flag we are clearing
//@return:  bool -- true if timer successfully cleared
//@outcome: a timer's overflow flag is cleared and ISRs stop occuring.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool timer_overflow_flag_clear(uint8_t timer)
{
    if(timer == timer0)
    {
        TIMER_A0->CTL &= ~TIMER_A_CTL_IFG;          //clear overflow flag
    }
    else if(timer == timer1)
    {
        TIMER_A1->CTL &= ~TIMER_A_CTL_IFG;          //clear overflow flag
    }
    else if(timer == timer2)
    {
        TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;          //clear overflow flag
    }
    else if(timer == timer3)
    {
        TIMER_A3->CTL &= ~TIMER_A_CTL_IFG;          //clear overflow flag
    }
    else
    {
        #ifdef _TIMER_DEBUG_
            printf("TIMER.C: > timer A%d overflow flag not cleared\n\r",timer);
        #endif

        return 0;
    }

    return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void delay_sec(uint16_t sec)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer based delay with one second increments.  USES TIMER A0!
//
//
//@param:   uint16_t sec    -- number of seconds to wait
//@return:  void
//@outcome: processor delayed X seconds.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void delay_sec(uint16_t sec)
{

    uint16_t i = 0;

    for(i = 0; i < sec; i++)
    {
        delay_ms(1000);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void delay_ms(uint16_t msec)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer based delay with millisecond increments.  USES TIMER A0!
//
//
//@param:   uint16_t msec -- number of milliseconds to wait.
//@return:  void
//@outcome: processor delayed x milliseconds.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void delay_ms(uint16_t msec)
{
    uint16_t ms_count = 0;

    timer_start(timer0, ONE_MSEC_COUNTS);                   //start timer and set to X msec worth of counts

    while(ms_count <= msec)
    {
        if(timer_overflow(timer0))
        {
            ms_count++;                                 //increment ms counter
            timer_overflow_flag_clear(timer0);
        }
    }

    timer_stop(timer0);                                     //stop the timer
    timer_clear(timer0);                                    //clear timer count register
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void delay_us(uint16_t usec)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer based delay with microsecond increments.  USES TIMER A0!
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void delay_us(uint16_t usec)
{
    timer_start(timer0, ONE_USEC_COUNTS*usec);                      //start timer and set to X usec worth of counts

    while(!timer_overflow(timer0));                               //while overflow hasn't happened

    timer_stop(timer0);                                           //stop the timer
    timer_clear(timer0);                                          //clear timer count register
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void TA0_N_IRQHandler(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer overflow interrupt ISR.
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

void TA0_N_IRQHandler(void)
{
    TIMER_A0->CTL &= ~TIMER_A_CTL_IFG;      // Clear timer overflow interrupt flag
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void TA2_N_IRQHandler(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer overflow interrupt ISR
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

void TA2_N_IRQHandler(void)
{
    TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void TA3_N_IRQHandler(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timer overflow interrupt ISR
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

void TA3_N_IRQHandler(void)
{
    TIMER_A3->CTL &= ~TIMER_A_CTL_IFG;
}

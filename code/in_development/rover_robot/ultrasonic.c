/*---------------------------------------------------------------------------
*ultrasonic.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Referenced: https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/
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

ultrasonic_t ultrasonic;

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

inline bool ultrasonic_echo(void);
inline void    ultrasonic_interrupt_init(void);

inline float calc_distance(uint16_t pulse_length, ultrasonic_unit_t units);
uint16_t get_pulse_length(void);


//-------------------------------------------------------------------------------------------
//                                 FUNCTION DECLARATIONS
//-------------------------------------------------------------------------------------------

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

void ultrasonic_init(void)
{
    timer_init(timer3,no_interrupts);

    //initialize port P5.4 as trigger, and P5.5 as echo
    P5->DIR |= TRIGGER;                                             //set trigger as P5.4 (output)
    P5->DIR &= ~ECHO;                                               //set echo as P5.5 (input)
    P5->OUT &= ~(TRIGGER | ECHO);                                   //set pulldowns on echo pin and trigger = 0
    P5->REN |= ECHO;                                                //enable pulldown on echo pin

    P5->SEL0 &= ~(TRIGGER | ECHO);                                  //set port function to GPIO
    P5->SEL1 &= ~(TRIGGER | ECHO);

    ultrasonic.units = INCHES;                                      //set default units to counts

    ultrasonic_interrupt_init();                                    //echo signal generates an interrupt
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void ultrasonic_interrupt_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   initializes interrupts on the echo pin P5.5 GPIO for a rising edge.  
//
//
//@param:   void
//@return:  void
//@outcome: PORT5_IRQHandler called when rising edge of echo is detected.
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
//void ultrasonic_set_units(ultrasonic_unit_t units)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets the units the ultrasonic sensor measures distance in
//
//
//@param:   ultrasonic_unit_t units -- units in inches, mm, cm, meters or counts
//@return:  void
//@outcome: measurment unit set.
/////////////////////////////////////////////////////////////////////////////////////////////

void ultrasonic_set_units(ultrasonic_unit_t units)
{
    ultrasonic.units = units;
}

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

inline void ultrasonic_trigger(void)
{

    CLR_TRIGGER;                        //make sure trigger is low
    delay_us(10);                       //delay for a bit
    SET_TRIGGER;                        //set trigger high
    delay_us(10);                       //delay for 10 us
    CLR_TRIGGER;                        //set trigger low again
}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline bool ultrasonic_echo(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   measures logic level of echo signal P5.5.
//
//
//@param:   void
//@return:  bool -- current value of P5.5 (echo)
//@outcome: determines if echo signal is high or low.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool ultrasonic_echo(void)
{
    return (P5->IN & ECHO);
}

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

float ultrasonic_sample(ultrasonic_unit_t units)
{

    ultrasonic.units = units;                               //update the units

    ultrasonic_trigger();                                   //send out trigger pulse

    return ultrasonic.distance;

}

/////////////////////////////////////////////////////////////////////////////////////////////
//inline float calc_distance(uint16_t pulse_length, ultrasonic_unit_t units)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   calculates distance from the measured echo pulse.
//
//
//@param:   uint16_t pulse_length   --  pulse length in timer counts
//@param:   ultrasonic_unit_t units --  units of distance calculation
//@return:  float -- calculated distance.
//@outcome: distance calculated.
/////////////////////////////////////////////////////////////////////////////////////////////

inline float calc_distance(uint16_t pulse_length, ultrasonic_unit_t units)
{
    switch(units)
    {
        case INCHES:      { ultrasonic.distance = ((.667*pulse_length) / 148); break;    }
        case METERS:      { ultrasonic.distance =  ((.667*pulse_length*340) / 2); break; }
        case CENTIMETERS: { ultrasonic.distance = ((.667*pulse_length) / 58);     break; }
        case MILLIMETERS: { ultrasonic.distance = ((.667*pulse_length) / 580);    break; }
        case COUNTS:      { ultrasonic.distance = pulse_length; break;                   }
    }

    return ultrasonic.distance;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t get_pulse_length(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   BLOCKING, measures the length of the echo pulse using TIMER_A3
//
//
//@param:   void
//@return:  uint16_t -- pulse length in counts.
//@outcome: echo pulse length measured.
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t get_pulse_length(void)
{
    uint16_t stop_time;

    timer_start(timer3,MAX_DIST);                           //start timer

    while(ultrasonic_echo());                               //while echo is high

    timer_stop(timer3);                                     //make sure timer isn't running
    stop_time = timer_getval(timer3);                       //get the current value of the timer
    timer_clear(timer3);                                    //clear the timer for next user

    return stop_time;
}

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

bool ultrasonic_object_near(void)
{
    ultrasonic.object_near = false;

    switch(ultrasonic.units)
    {
        case INCHES:      { if(ultrasonic.distance <= OBJECT_NEAR_INCHES) { ultrasonic.object_near = true;} break;}
        case METERS:      { if(ultrasonic.distance <= OBJECT_NEAR_METERS) { ultrasonic.object_near = true;}break;}
        case CENTIMETERS: { if(ultrasonic.distance <= OBJECT_NEAR_CENTIMETERS) { ultrasonic.object_near = true;}break;}
        case MILLIMETERS: { if(ultrasonic.distance <= OBJECT_NEAR_MILLIMETERS) { ultrasonic.object_near = true;}break;}
        case COUNTS:      { if(ultrasonic.distance <= OBJECT_NEAR_COUNTS) { ultrasonic.object_near = true;}break;}
    }

    if(ultrasonic.object_near)
    {
        RED_LED_ON;
    }
    else
    {
        RED_LED_OFF;
    }

    return ultrasonic.object_near;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void PORT5_IRQHandler(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   interrupt triggered when echo pulse transitions from low to high in P5.5.
//
//
//@param:   void
//@return:  void
//@outcome: echo pulse measured and distance is calculated.
/////////////////////////////////////////////////////////////////////////////////////////////

void PORT5_IRQHandler(void)
{

    P5->IE   &= ~ECHO;                                      //disable interrupts on echo signal
    P5->IFG  &= ~ECHO;                                      //clear echo interrupt flag

    calc_distance(get_pulse_length(),ultrasonic.units);     //measure pulse length and calculate distance

    #ifdef _ULTRASONIC_DEBUG_
        printf("Object Distance: %.2lf (inches)\n\r",ultrasonic.distance);
    #endif

    P5->IE   |= ECHO;                                       //enable interrupts on echo signal

}

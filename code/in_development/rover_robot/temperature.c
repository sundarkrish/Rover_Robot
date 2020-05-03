/*---------------------------------------------------------------------------------------
*temperature.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
**---------------------------------------------------------------------------------------
*@brief: This program allocates a buffer of a particular size as requested by
*a user over a serial connection.
*
*Code Use:  code is modified versions of MSP432 code example adc14_10
*
*--------------------------------------------------------------------------------------*/

//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

#include "temperature.h"

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

    // Variables to store the ADC temperature reference calibration value
static    int32_t adcRefTempCal_1_2v_30;
static    int32_t adcRefTempCal_1_2v_85;


volatile long temp;      //temperature sensor value
volatile float IntDegF;         //temperature in F
volatile float IntDegC;         //temperature in C

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//                                 FUNCTION DECLARATIONS
//-------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////
//void temp_interrupt_init(void)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables ADC14 interrupts
//
//
//@param:   void
//@return:  void
//@outcome: ADC14 can cause interrupts
//
////////////////////////////////////////////////////////////////////////////////////////////


void temp_interrupt_init(void)
{
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);// Enable ADC interrupt in NVIC module
}

////////////////////////////////////////////////////////////////////////////////////////////
//void temp_init(void)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets up ADC14 to read voltage of internal temperature sensor with 5 us sample
//          period.  Single ended measurment.
//
//@param:   void
//@return:  void
//@outcome: ADC14 is connected to temperature sensor
//
////////////////////////////////////////////////////////////////////////////////////////////

void temp_init(void)
{

    // Read the ADC temperature reference calibration value
    adcRefTempCal_1_2v_30 = TLV->ADC14_REF1P2V_TS30C;           //read TLV desc. for 1.2V @ 30C
    adcRefTempCal_1_2v_85 = TLV->ADC14_REF1P2V_TS85C;           //read TLV descriptor for 1.2V @ 85C

    // Initialize the shared reference module
    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
    while(REF_A->CTL0 & REF_A_CTL0_GENBUSY);// If ref generator busy, WAIT
    REF_A->CTL0 |= REF_A_CTL0_VSEL_0 |      // Enable internal 1.2V reference,
            REF_A_CTL0_ON;                  // Turn reference on

    REF_A->CTL0 &= ~REF_A_CTL0_TCOFF;       // Enable temperature sensor

    // Configure ADC - Pulse sample mode; ADC14_CTL0_SC trigger
    ADC14->CTL0 |= ADC14_CTL0_SHT0_6 |      // ADC ON,temperature sample period>5us, 128 CLKs
            ADC14_CTL0_ON |                 //turn on ADC14 power
            ADC14_CTL0_SHP;                 //SAMPCON signal sourced from sampling timer
    ADC14->CTL1 |= ADC14_CTL1_TCMAP;        //Enable internal temperature sensor on ADC14
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_1 |  //VR+ = VREF buffered, V(R-) = AVSS (GND)
            ADC14_MCTLN_INCH_22;            //ADC input ch A22 => temp sense, single ended.
    ADC14->IER0 = 0x0001;                   //ADC_IFG upon conv result-ADCMEM0

    // Wait for reference generator to settle
    while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));

    ADC14->CTL0 |= ADC14_CTL0_ENC;          //ADC14 enable conversion

    temp_interrupt_init();                  //enable interrupts
    temp_read('F');                         //dummy read, discard junk value

}

////////////////////////////////////////////////////////////////////////////////////////////
//float temp_read(char _c)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Causes interrupt that reads temperature sensor, then converts value into
//          human readable F or C value.
//
//@param:   char _c -- determines if temperature is returned in C or F
//@return:  Temperature of MSP432 in F or C
//@outcome: Internal temperature read
//
////////////////////////////////////////////////////////////////////////////////////////////

float temp_read(char _c)
{
    ADC14->CTL0 |= ADC14_CTL0_SC;       // Sampling and conversion start
    // Temperature in Celsius
    // The temperature (Temp, C)=
    IntDegC = (((float) temp - adcRefTempCal_1_2v_30) * (85 - 30)) /
            (adcRefTempCal_1_2v_85 - adcRefTempCal_1_2v_30) + 30.0f;

    // Temperature in Fahrenheit
    // Tf = (9/5)*Tc | 32
    IntDegF = ((9 * IntDegC) / 5) + 32;

    if(_c == 'c' || _c == 'C')
        return IntDegC;     //if Celsius is selected
    if(_c == 'f' || _c == 'F')
        return IntDegF;     //if Fahrenheit is selected

    return IntDegC;         //if anything else is received default return Celsius
}

////////////////////////////////////////////////////////////////////////////////////////////
//void ADC14_IRQHandler(void)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   ADC14 interrupt, reads latest ADC conversion into temp variable
//
//
//@param:   void
//@return:  void
//@outcome: temperature measured
//
////////////////////////////////////////////////////////////////////////////////////////////

// ADC14 interrupt service routine
void ADC14_IRQHandler(void)
{
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG0)                //check if ADC14 caused the interrupt
    {
        temp = ADC14->MEM[0];                           //read conversion result into temp
    }
}

/*-------------------------------------------------------------------------------------------
*template.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*--------------------------------------------------------------------------------------------
*Code References:   https://github.com/adafruit/Adafruit_APDS9960
*
**-----------------------------------------------------------------------------------------*/


//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//libraries
#include "apds9960.h"

//Custom Libraries

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

/* Get Struct Value Functions */
uint8_t get_enable(void);
uint8_t get_control(void);

/* I2C Register Read/Write Functions */
bool write_register_8bits(uint8_t _register, uint8_t data);
bool read_register_8bits(uint8_t _register, uint8_t * data);
bool read_register_16bits(uint8_t _register, uint16_t * data);

/* Enable Peripheral Functions */
void enable_power(bool _enable);
void enable_color(bool _enable);

/* Get Peripheral Status Functions */

bool color_DataReady(void);

/* Update Peripheral Setting Functions */
void    set_adcGain(uint8_t gain);
AGain_t get_adcGain(void);

void    set_adcIntegrationTime(uint8_t time_ms);
float get_adcIntegrationTime(void);

/* Peripheral Results Functions */



//-------------------------------------------------------------------------------------------
//                                 FUNCTION DECLARATIONS
//-------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//bool apds9960_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   configures the APDS9960 proximity/light sensor to detect ambient light.
//          sets ADC gain and integration time, enables power.
//
//@param:   void
//@return:  bool    -- 1 if device failed to initialize, 0 if success
//@outcome: light sensor configured and powered on.
/////////////////////////////////////////////////////////////////////////////////////////////

bool apds9960_init(void)
{
    uint8_t data;

    read_register_8bits(APDS9960_ID, &data);                                 //read ID Register
    if(data != APDS9960_ID_VAL) { return 1; }

    enable_color(false);
    enable_power(true);

    set_adcGain(APDS9960_AGAIN_64X);
    set_adcIntegrationTime(25);

    enable_color(true);

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//bool apds9960_color_DataReady(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks if color data is available from sensor, reutrns true if yes.
//
//
//@param:   void
//@return:  bool -- true if data is available, false if no data.
//@outcome: status of color data ready determined.
/////////////////////////////////////////////////////////////////////////////////////////////

bool apds9960_color_DataReady(void)
{
    uint8_t temp;

    read_register_8bits(APDS9960_STATUS, &temp);                            //read status register
    status.AVALID = (BIT0 & temp);                                          //update the AVALID bit in the status struct

    return status.AVALID;                                                   //return AVALID

}

/////////////////////////////////////////////////////////////////////////////////////////////
//bool apds9960_get_ColorAll(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns RGB and clear color data back from sensor as 16 bit values
//
//
//@param:   color pointers -- pointers to uint16_t variables where color will be stored
//@return:  bool -- true if color data is ready
//@outcome: color data read back from sensor.
/////////////////////////////////////////////////////////////////////////////////////////////

bool apds9960_get_ColorAll(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear)
{
    if(!apds9960_color_DataReady())
    {
        return 1;
    }

    read_register_16bits(APDS9960_RDATAL, red);
    read_register_16bits(APDS9960_GDATAL, green);
    read_register_16bits(APDS9960_BDATAL, blue);
    read_register_16bits(APDS9960_CDATAL, clear);

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t apds9960_get_color(color_t color)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads back a single color channel from the color sensor
//
//
//@param:   color_t color -- the color we are reading back
//@return:  uint16_t -- color data 
//@outcome: single channel color data is read back.
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t apds9960_get_color(color_t color)
{
    uint16_t data;

    if(!apds9960_color_DataReady())
    {
        return 1;
    }

    switch(color)
    {
        case GREEN: { read_register_16bits(APDS9960_GDATAL, &data); break;}
        case RED:   { read_register_16bits(APDS9960_RDATAL, &data); break;}
        case BLUE:  { read_register_16bits(APDS9960_BDATAL, &data); break;}
        case CLEAR: { read_register_16bits(APDS9960_CDATAL, &data); break;}
        default:    { return 1;}
    }

    return data;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t apds9960_get_light(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads back the clear channel from the light sensor, which is analogous to 
//          brightness.
//
//@param:   void
//@return:  uint16_t -- clear channel value
//@outcome: clear channel read back.
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t apds9960_get_light(void)
{
    return apds9960_get_color(CLEAR);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//bool apds9960_IsDark(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns whetehr the environment light is above a programmable threshold.  
//          
//
//@param:   void
//@return:  bool -- true if there is light, false if dark
//@outcome: determine if the room is dark or light.
/////////////////////////////////////////////////////////////////////////////////////////////

bool apds9960_IsDark(void)
{
    if(apds9960_get_light() < 2000)                            //if there is very little light
    {
        BLUE_LED_ON;                                        //turn on the blue LED

        return true;                                        //return true
    }

    BLUE_LED_OFF;                                           //turn off the blue LED

    return false;

}

/////////////////////////////////////////////////////////////////////////////////////////////
//bool write_register_8bits(uint8_t _register, uint8_t data)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   writes a byte worth of data to a particular register
//
//
//@param:   _register   -- the register we are writing to
//@param:   data        -- 
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

bool write_register_8bits(uint8_t _register, uint8_t data)
{
    return I2C1_Write(APDS9960_ADDR, _register, data);                      //write a byte to APDS9960
}

/////////////////////////////////////////////////////////////////////////////////////////////
//bool read_register_8bits(uint8_t _register, uint8_t * data)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads a byte back from any register in the APDS9960
//
//
//@param:   uint8_t _register   -- accessed register address   
//@param:   uint8_t * data      -- returned 8 bit register data
//@return:  bool                -- true if read failed
//@outcome: a byte is read from a register in the APDS9960
/////////////////////////////////////////////////////////////////////////////////////////////

bool read_register_8bits(uint8_t _register, uint8_t * data)
{
    return I2C1_Read(APDS9960_ADDR, _register, data);                       //write a byte to APDS9960
}

/////////////////////////////////////////////////////////////////////////////////////////////
//bool read_register_16bits(uint8_t _register, uint16_t * data)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads two consecutive registers at addr & addr + 1, then combines the reult into
//          16 bit value.
//
//@param:   uint8_t _register   -- accessed register address
//@param:   uint16_t * data     -- returned 16 bit register data
//@return:  bool                -- true if failed to read
//@outcome: a 16 bit register is read from the APDS9960
/////////////////////////////////////////////////////////////////////////////////////////////

bool read_register_16bits(uint8_t _register, uint16_t * data)
{
    bool error;
    uint8_t temp[2];

    error = read_register_8bits(_register, temp);                           //get the LSB
    if(error) { return 1; }                                                 //if read failed, return
    error = read_register_8bits(++_register, (temp+1));                     //get the MSB
    if(error) { return 1; }                                                 //if read failed, return

    *data =  (temp[1] << 8) | temp[0];                                      //combine MSB with LSB

    return 0;                                                               //return success
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void enable_power(bool _enable)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   powers up the APDS9960 device into idle mode
//
//
//@param:   bool _enable -- set to true to power on, false to power off
//@return:  void
//@outcome: APDS9960 is powered on or off.
/////////////////////////////////////////////////////////////////////////////////////////////

void enable_power(bool _enable)
{
    //power on device
    enable.PON = _enable;                                                   //set PEN to true in struct
    write_register_8bits(APDS9960_ENABLE, get_enable());                    //enable power to APDS9960
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void enable_color(bool _enable)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables the color engine inside the APDS9960 allowing color data to be sampled.
//
//
//@param:   bool _enable -- true to enable color engine, false to disable
//@return:  void
//@outcome: Color sampling enabled or disabled.
/////////////////////////////////////////////////////////////////////////////////////////////

void enable_color(bool _enable)
{

    enable.AEN = _enable;                                                   //update AEN
    write_register_8bits(APDS9960_ENABLE, get_enable());                    //write AEN bit to sensor
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void set_adcGain(uint8_t gain)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets ADC gain of all color channels 
//
//
//@param:   uint8_t gain -- gain value (1x, 4x, 16x, 64x)
//@return:  void
//@outcome: ADC gain set
/////////////////////////////////////////////////////////////////////////////////////////////

void set_adcGain(uint8_t gain)
{
    control.AGAIN = gain;                                                   //write gain value into struct
    write_register_8bits(APDS9960_CONTROL, get_control());                  //get struct and write into control register
}

/////////////////////////////////////////////////////////////////////////////////////////////
//AGain_t get_adcGain(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns the current gain setting of the all color ADC channels
//
//
//@param:   void
//@return:  AGain_t -- gain setting (1x, 4x, 16x, 64x)
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

AGain_t get_adcGain(void)
{
    uint8_t temp;

    read_register_8bits(APDS9960_CONTROL, &temp);

    return (AGain_t)(temp & 0x03);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//
// void set_adcIntegrationTime(uint8_t time_ms)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets the integration time of the color ADC.
//
//
//@param   :   uint8_t time_ms -- number of milliseconds to integrate the signal.
//@return  :  void
//@outcome : Color ADC integration time set to desired value.
/////////////////////////////////////////////////////////////////////////////////////////////

void set_adcIntegrationTime(uint8_t time_ms)
{
    float temp;

    temp = time_ms;
    temp /= 2.78;
    temp = 256 - temp;

    if(temp > 255) { temp = 255; }                                          //if result is bigger than 8 bit value
    if(temp < 0) { temp = 0; }                                              //if result is negative

    write_register_8bits(APDS9960_ATIME, (uint8_t)temp);                    //write the integration time into the register
}

/////////////////////////////////////////////////////////////////////////////////////////////
//float get_adcIntegrationTime(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads the current color ADC intergration time setting.
//
//
//@param:   void
//@return:  float -- current integration time in ms.
//@outcome: ADC integration time determined.
/////////////////////////////////////////////////////////////////////////////////////////////

float get_adcIntegrationTime(void)
{
    float temp;
    uint8_t data;

    read_register_8bits(APDS9960_ATIME, &data);

    temp = data;

    temp = 256 - temp;
    temp *= 2.78;

    return temp;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t get_enable(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads back the APDS9960 class enable struct bits
//
//
//@param:   void
//@return:  uint8_t -- the enable struct value
//@outcome: determines the enable status of the APDS9960 sensor.
/////////////////////////////////////////////////////////////////////////////////////////////

uint8_t get_enable(void)
{
    return (enable.GEN << 6) | (enable.PIEN << 5) | (enable.AIEN << 4) |
           (enable.WEN << 3) | (enable.PEN << 2) | (enable.AEN << 1) |
           (enable.PON);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t get_control(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns the current APDS9960 class control struct bits.
//
//
//@param:   void
//@return:  uint8_t -- the control struct value
//@outcome: determines the control status of the APDS9960 sensor.
/////////////////////////////////////////////////////////////////////////////////////////////

uint8_t get_control(void)
{
    return (control.LDRIVE << 6) | (control.PGAIN << 2) | control.AGAIN;
}

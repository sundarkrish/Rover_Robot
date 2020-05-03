/*--------------------------------------------------------------------------------------------
*template.h
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------------------------
*Code Reuse:
*
**------------------------------------------------------------------------------------------*/

#ifndef _TEMPLATE_H_
#define _TEMPLATE_H_

//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//libraries
#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

//Custom Libraries
#include "i2c.h"
#include "timer.h"
//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

#define APDS9960_ADDR       (0x39) /* I2C Address */
#define APDS9960_ID_VAL     (0xAB) /* Device ID */

#define BLUE_LED_ON         P2->OUT |= BIT2
#define BLUE_LED_OFF        P2->OUT &= ~BIT2

#define _APDS9960_          //if the light sensor is connected define this

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

enum {
    APDS9960_RAM = 0x00,
    APDS9960_ENABLE = 0x80,
    APDS9960_ATIME = 0x81,
    APDS9960_WTIME = 0x83,
    APDS9960_AILTIL = 0x84,
    APDS9960_AILTH = 0x85,
    APDS9960_AIHTL = 0x86,
    APDS9960_AIHTH = 0x87,
    APDS9960_PILT = 0x89,
    APDS9960_PIHT = 0x8B,
    APDS9960_PERS = 0x8C,
    APDS9960_CONFIG1 = 0x8D,
    APDS9960_PPULSE = 0x8E,
    APDS9960_CONTROL = 0x8F,
    APDS9960_CONFIG2 = 0x90,
    APDS9960_ID = 0x92,
    APDS9960_STATUS = 0x93,
    APDS9960_CDATAL = 0x94,
    APDS9960_CDATAH = 0x95,
    APDS9960_RDATAL = 0x96,
    APDS9960_RDATAH = 0x97,
    APDS9960_GDATAL = 0x98,
    APDS9960_GDATAH = 0x99,
    APDS9960_BDATAL = 0x9A,
    APDS9960_BDATAH = 0x9B,
    APDS9960_PDATA = 0x9C,
    APDS9960_POFFSET_UR = 0x9D,
    APDS9960_POFFSET_DL = 0x9E,
    APDS9960_CONFIG3 = 0x9F,
    APDS9960_GPENTH = 0xA0,
    APDS9960_GEXTH = 0xA1,
    APDS9960_GCONF1 = 0xA2,
    APDS9960_GCONF2 = 0xA3,
    APDS9960_GOFFSET_U = 0xA4,
    APDS9960_GOFFSET_D = 0xA5,
    APDS9960_GOFFSET_L = 0xA7,
    APDS9960_GOFFSET_R = 0xA9,
    APDS9960_GPULSE = 0xA6,
    APDS9960_GCONF3 = 0xAA,
    APDS9960_GCONF4 = 0xAB,
    APDS9960_GFLVL = 0xAE,
    APDS9960_GSTATUS = 0xAF,
    APDS9960_IFORCE = 0xE4,
    APDS9960_PICLEAR = 0xE5,
    APDS9960_CICLEAR = 0xE6,
    APDS9960_AICLEAR = 0xE7,
    APDS9960_GFIFO_U = 0xFC,
    APDS9960_GFIFO_D = 0xFD,
    APDS9960_GFIFO_L = 0xFE,
    APDS9960_GFIFO_R = 0xFF,
};

/** ADC gain settings */
typedef enum {
    APDS9960_AGAIN_1X = 0x00,  /**< No gain */
    APDS9960_AGAIN_4X = 0x01,  /**< 2x gain */
    APDS9960_AGAIN_16X = 0x02, /**< 16x gain */
    APDS9960_AGAIN_64X = 0x03  /**< 64x gain */
}AGain_t;

typedef enum {
    BLUE,
    GREEN,
    RED,
    CLEAR
}color_t;

typedef enum {
    BRIGHT,
    WELL_LIT,
    DIM,
    SHADOW,
    DARK
}light_t;

struct enable_t{

    // power on
    uint8_t PON : 1;

    // ALS enable
    uint8_t AEN : 1;

    // Proximity detect enable
    uint8_t PEN : 1;

    // wait timer enable
    uint8_t WEN : 1;

    // ALS interrupt enable
    uint8_t AIEN : 1;

    // proximity interrupt enable
    uint8_t PIEN : 1;

    // gesture enable
    uint8_t GEN : 1;
};

struct enable_t enable;

struct status_t{
    /* ALS Valid. Indicates that an ALS cycle has completed since AEN was
    asserted or since a read from any of the ALS/Color data registers.
    */
    uint8_t AVALID : 1;

    /* Proximity Valid. Indicates that a proximity cycle has completed since PEN
    was asserted or since PDATA was last read. A read of PDATA automatically
    clears PVALID.
    */
    uint8_t PVALID : 1;

    /* Gesture Interrupt. GINT is asserted when GFVLV becomes greater than
    GFIFOTH or if GVALID has become asserted when GMODE transitioned to zero.
    The bit is reset when FIFO is completely emptied (read).
    */
    uint8_t GINT : 1;

    // ALS Interrupt. This bit triggers an interrupt if AIEN in ENABLE is set.
    uint8_t AINT : 1;

    // Proximity Interrupt. This bit triggers an interrupt if PIEN in ENABLE is
    // set.
    uint8_t PINT : 1;

    /* Indicates that an analog saturation event occurred during a previous
    proximity or gesture cycle. Once set, this bit remains set until cleared by
    clear proximity interrupt special function command (0xE5 PICLEAR) or by
    disabling Prox (PEN=0). This bit triggers an interrupt if PSIEN is set.
    */
    uint8_t PGSAT : 1;

    /* Clear Photodiode Saturation. When asserted, the analog sensor was at the
    upper end of its dynamic range. The bit can be de-asserted by sending a
    Clear channel interrupt command (0xE6 CICLEAR) or by disabling the ADC
    (AEN=0). This bit triggers an interrupt if CPSIEN is set.
    */
    uint8_t CPSAT : 1;

};

struct status_t status;

struct control_t{
    // ALS and Color gain control
    uint8_t AGAIN : 2;

    // proximity gain control
    uint8_t PGAIN : 2;

    // led drive strength
    uint8_t LDRIVE : 2;
};

struct control_t control;

//-------------------------------------------------------------------------------------------
//                              PUBLIC FUNCTION PROTOTYPES
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

bool apds9960_init(void);

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

bool apds9960_color_DataReady(void);


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

uint16_t apds9960_get_color(color_t color);
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

bool apds9960_get_ColorAll(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear);

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

bool apds9960_IsDark(void);

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

uint16_t apds9960_get_light(void);


#endif  //_TEMPLATE_H_

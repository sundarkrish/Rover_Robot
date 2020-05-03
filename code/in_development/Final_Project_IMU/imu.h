/*---------------------------------------------------------------------------
*imu.h
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reference :
*
**--------------------------------------------------------------------------*/

#ifndef _IMU_H_
#define _IMU_H_


//---------------------------------------------------------------------------
//                              INCLUDES
//---------------------------------------------------------------------------

//libraries


#include "i2c.h"

//---------------------------------------------------------------------------
//                              DEFINES
//---------------------------------------------------------------------------
#define FXOS8700_ADDRESS (0x1F) // 0011111 accel and magnetometer IC
#define FXOS8700_ID (0xC7) // 1100 0111 stored at WHOAMI register(for POST test purpose)
#define FXOS8700_REGISTER_WHO_AM_I (0x0D) // sanity check register


////////////////////////////////////////////// FXOS8700 related ///////////////////////////////////////////////////////////////
#define FXOS8700CQ_STATUS 0x00
#define FXOS8700CQ_WHOAMI 0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 0x2A
#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C
// hyb_autoinc_mode bit has been set to enable the
// reading of all accelerometer and magnetometer data in a single-burst, read operation.
#define FXOS8700CQ_READ_LEN 13 // status byte  + 6 bytes accel reading + 6 bytes of mag reading


/** Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.244mg) */
#define ACCEL_MG_LSB_2G (0.244F)
/** Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.488mg) */
#define ACCEL_MG_LSB_4G (0.488F)
/** Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.976mg) */
#define ACCEL_MG_LSB_8G (0.976F)
/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB (0.1F)


// For raw X,Y and Z accelerometer values.
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

}accel_raw_typedef;

// For X,Y and Z values in milliG(accelerometer)
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

}accel_data_typedef;


// For raw X,Y and Z magnetometer values.
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

}mag_raw_typedef;

// For X,Y and Z values in uT
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

}mag_data_typedef;

////////////////////////////////////////////// FXOS8700 related ///////////////////////////////////////////////////////////////



////////////////////////////////////////////// FXAS21002C related ///////////////////////////////////////////////////////////////

/** 7-bit address for this sensor */
#define FXAS21002C_ADDRESS (0x21) // 0100001
/** Device ID for this sensor (used as a sanity check during init) */
#define FXAS21002C_ID (0xD7) // 1101 0111
#define FXAS21002C_REGISTER_WHO_AM_I  (0x0C) // sanity check register

#define FXAS21002C_READ_LEN 7 // status byte  + 6 bytes gyro axis reading

#define FXAS21002C_REGISTER_STATUS (0x00)
#define FXAS21002C_REGISTER_OUT_X_MSB (0x01)
#define FXAS21002C_REGISTER_OUT_X_LSB (0x02)
#define FXAS21002C_REGISTER_OUT_Y_MSB (0x03)
#define FXAS21002C_REGISTER_OUT_Y_LSB (0x04)
#define FXAS21002C_REGISTER_OUT_Z_MSB (0x05)
#define FXAS21002C_REGISTER_OUT_Z_LSB (0x06)

#define FXAS21002C_REGISTER_CTRL_REG0 (0x0D)
#define FXAS21002C_REGISTER_CTRL_REG1 (0x13)

#define FXAS21002C_TEMP_REGISTER (0x12) // 8bit 2s complement temperature register
#define FXAS21002C_TEMP_SCALE (1) //  1 °C/LSB

/** Gyroscope sensitivity at 250dps */
#define GYRO_SENSITIVITY_250DPS (7.8125F) // 1LSB = 7.8125 mdps
/** Gyroscope sensitivity at 500dps */
#define GYRO_SENSITIVITY_500DPS (15.625F) // 1LSB = 15.625 mdps
/** Gyroscope sensitivity at 1000dps */
#define GYRO_SENSITIVITY_1000DPS (31.25F) // 1LSB = 31.25 mdps
/** Gyroscope sensitivity at 2000dps */
#define GYRO_SENSITIVITY_2000DPS (62.5F) // 1LSB = 62.5 mdps

#define FXAS21002C_MDPS_TO_MRAD (0.01745F) // 1 degree = 0.0174533 rad


// For raw X,Y and Z gyroscope values.
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

}gyro_raw_typedef;

// For X,Y and Z values in mdps
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;

}gyro_data_typedef;

////////////////////////////////////////////// FXAS21002C related ///////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
//                          EXTERNAL VARIABLES
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//                          FUNCTION PROTOTYPES
//---------------------------------------------------------------------------


/////////////////////////////////////////////////////////////////////////////
//bool FXOS8700_POST_test(void)
/////////////////////////////////////////////////////////////////////////////
//@brief: sanity check by reading WHOAMI register for FXOS8700
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////
bool FXOS8700_POST_test(void);

/////////////////////////////////////////////////////////////////////////////
//bool FXAS21002C_POST_test(void)
/////////////////////////////////////////////////////////////////////////////
//@brief: sanity check by reading WHOAMI register for FXAS21002C
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////
bool FXAS21002C_POST_test(void);

/////////////////////////////////////////////////////////////////////////////
//void IMU_init(void)
/////////////////////////////////////////////////////////////////////////////
//@brief: IMU initial configurations for accelerometer , magnetometer and gyroscope happens here
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////
bool IMU_init(void);

/////////////////////////////////////////////////////////////////////////////
//bool IMU_accelmag_read(uint8_t * status, accel_data_typedef * accel_data, mag_data_typedef * mag_data)
/////////////////////////////////////////////////////////////////////////////
//@brief: Status register read
//        Accelerometer and magnetometer values read and converted to milliGs and uT respectively
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////
bool IMU_accelmag_read(int8_t * status, accel_data_typedef * accel_data, mag_data_typedef * mag_data);


/////////////////////////////////////////////////////////////////////////////
//bool IMU_gyrotemp_read(int8_t * status, gyro_data_typedef * gyro_data, int8_t * temperature_data)
/////////////////////////////////////////////////////////////////////////////
//@brief: Status register read
//        Gyroscope and temperature sensor values read and converted to millirads/s and degreeC respectively
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////
bool IMU_gyrotemp_read(int8_t * status, gyro_data_typedef * gyro_data, int8_t * temperature_data);


#endif  //_IMU_H_


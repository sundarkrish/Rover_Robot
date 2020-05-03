/*---------------------------------------------------------------------------
*imu.c
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reference:    https://github.com/adafruit/Adafruit_FXOS8700/blob/master/Adafruit_FXOS8700.cpp(accel+mag module init sequence & scaling procedure)
*                   https://cdn-learn.adafruit.com/assets/assets/000/043/458/original/FXOS8700CQ.pdf?1499125614(accel+mag module init sequence)
*                   https://github.com/adafruit/Adafruit_FXAS21002C/blob/master/Adafruit_FXAS21002C.cpp(gyro module init sequence & scaling procedure)
**--------------------------------------------------------------------------*/

//---------------------------------------------------------------------------
//                              INCLUDES
//---------------------------------------------------------------------------

//libraries
#include "imu.h"

//Custom Libraries

//---------------------------------------------------------------------------
//                              DEFINES
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//                          EXTERNAL VARIABLES
//---------------------------------------------------------------------------


accel_raw_typedef accel_raw;
mag_raw_typedef mag_raw;
gyro_raw_typedef gyro_raw;

uint8_t buff[15];


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
bool FXOS8700_POST_test(void)
{
    unsigned char data = 0;

    // test fail due to NACK from slave / bus arbitration lost
    if (I2C1_Read(FXOS8700_ADDRESS,FXOS8700_REGISTER_WHO_AM_I, &data)) return 1;

    if (data == FXOS8700_ID)
    {
        return 0; // no error

    }
    else
    {

        return 1; // error

    }
}


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
bool FXAS21002C_POST_test(void)
{
    unsigned char data = 0;

    // test fail due to NACK from slave / bus arbitration lost
    if (I2C1_Read(FXAS21002C_ADDRESS,FXAS21002C_REGISTER_WHO_AM_I, &data)) return 1;

    if (data == FXAS21002C_ID)
    {
        return 0; // no error

    }
    else
    {

        return 1; // error

    }
}


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
bool IMU_init(void)
{
    bool error_status = 0;
    uint8_t i = 0;

    /* FXOS8700 Accelerometer and magnetometer initial configurations starts here */

    // Place to FXOS8700 in standby mode to make configurations to Accelerometer Control register 1
    error_status = I2C1_Write(FXOS8700_ADDRESS, FXOS8700CQ_CTRL_REG1, 0x00);

    // // write 0001 1111 = 0x1F to magnetometer control register 1
    // [7]: m_acal=0: auto calibration disabled
    // [6]: m_rst=0: no one-shot magnetic reset
    // [5]: m_ost=0: no one-shot magnetic measurement
    // [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce
    error_status = I2C1_Write(FXOS8700_ADDRESS, FXOS8700CQ_M_CTRL_REG1, 0x1F);


    // write 0010 0000 = 0x20 to magnetometer control register 2
    // [7]: reserved
    // [6]: reserved
    // [5]: hyb_autoinc_mode=1 to map the magnetometer registers to
    //follow the
    // accelerometer registers
    // [4]: m_maxmin_dis=0 to retain default min/max latching even
    //though not used
    // [3]: m_maxmin_dis_ths=0
    // [2]: m_maxmin_rst=0
    // [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
    error_status = I2C1_Write(FXOS8700_ADDRESS, FXOS8700CQ_M_CTRL_REG2, 0x20);

    // write 0000 0001= 0x01 to XYZ_DATA_CFG register
    // [7]: reserved
    // [6]: reserved
    // [5]: reserved
    // [4]: hpf_out=0
    // [3]: reserved
    // [2]: reserved
    // [1-0]: fs=01 for accelerometer range of +/-4g range with
    // 0.488mg/LSB
    error_status = I2C1_Write(FXOS8700_ADDRESS, FXOS8700CQ_XYZ_DATA_CFG, 0x01);


    // write 0000 1101 = 0x0D to accelerometer control register 1
    // [7-6]: aslp_rate=00

    // [5-3]: dr=110 for 3.125Hz data rate (when in hybrid mode)
    // [2]: lnoise=1 for low noise mode
    // [1]: f_read=0 for normal 16 bit reads
    // [0]: active=1 to take the part out of standby and enable
    error_status = I2C1_Write(FXOS8700_ADDRESS, FXOS8700CQ_CTRL_REG1, 0x35);


    /* FXOS8700 Accelerometer and magnetometer initial configurations stops here */


    /* FXAS21002C gyroscope and temperature sensor initial configurations starts here */


    /* Set CTRL_REG1 (0x13)
      ====================================================================
      BIT  Symbol    Description                                   Default
      ---  ------    --------------------------------------------- -------
        6  RESET     Reset device on 1                                   0
        5  ST        Self test enabled on 1                              0
      4:2  DR        Output data rate                                  000
                     000 = 800 Hz
                     001 = 400 Hz
                     010 = 200 Hz
                     011 = 100 Hz
                     100 = 50 Hz
                     101 = 25 Hz
                     110 = 12.5 Hz
                     111 = 12.5 Hz
        1  ACTIVE    Standby(0)/Active(1)                                0
        0  READY     Standby(0)/Ready(1)                                 0
     */

     /* Set CTRL_REG0 (0x0D)  Default value 0x00
     =====================================================================
     BIT  Symbol     Description                                   Default
     7:6  BW         cut-off frequency of low-pass filter               00
       5  SPIW       SPI interface mode selection                        0
     4:3  SEL        High-pass filter cutoff frequency selection        00
       2  HPF_EN     High-pass filter enable                             0
     1:0  FS         Full-scale range selection
                     00 = +-2000 dps
                     01 = +-1000 dps
                     10 = +-500 dps
                     11 = +-250 dps
     The bit fields in CTRL_REG0 should be changed only in Standby or Ready modes.
     */



    /* Goto Standby -> Reset then switch to active mode with 100Hz output */
    error_status = I2C1_Write(FXAS21002C_ADDRESS, FXAS21002C_REGISTER_CTRL_REG1, 0x00); // Goto Standby mode to do configuration
    if (error_status) return 1;
    // Slave does not ACK the master when RESET signal is written - problematic operation -> Slave stops responding in next steps
    error_status = I2C1_Write(FXAS21002C_ADDRESS, FXAS21002C_REGISTER_CTRL_REG1, (1 << 6)); // Reset

    lock_release();
    error_status = 0; //Above i2c write operation always returns 1

    error_status = I2C1_Write(FXAS21002C_ADDRESS, FXAS21002C_REGISTER_CTRL_REG0, 0x00); // Set sensitivity as +/-2000dps

    error_status = I2C1_Write(FXAS21002C_ADDRESS, FXAS21002C_REGISTER_CTRL_REG1, 0x1E); // Active and ODR : 12.5Hz
    for(i = 0; i < 200; i++){}; // 60 ms + 1/ODR is needed for transition from standby to active mode

    /* FXAS21002C gyroscope and temperature sensor initial configurations stops here */

    return error_status; // 0 - no error; 1 - error


}


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
bool IMU_accelmag_read(int8_t * status, accel_data_typedef * accel_data, mag_data_typedef * mag_data)
{
    bool error_status = 0;
    error_status = I2C1_burstRead(FXOS8700_ADDRESS, FXOS8700CQ_STATUS, FXOS8700CQ_READ_LEN, buff);

    if(error_status > 0)
    {
        return 1; // error occurred
    }

    // Status register content
    *status = (int8_t)buff[0];

    if(*status < 0)
    {
        return 1; // error occurred
    }

    // copy the 14 bit accelerometer byte data into 16 bit words
    accel_raw.x = (int16_t)(((buff[1] << 8) | buff[2]))>> 2;
    accel_raw.y = (int16_t)(((buff[3] << 8) | buff[4]))>> 2;
    accel_raw.z = (int16_t)(((buff[5] << 8) | buff[6]))>> 2;

    // copy the magnetometer byte data into 16 bit words
    mag_raw.x = (buff[7] << 8) | buff[8];
    mag_raw.y = (buff[9] << 8) | buff[10];
    mag_raw.z = (buff[11] << 8) | buff[12];

    // +/- 4g selected during FXOS8700 configuration
    // Conversion from raw readings to milliGs
    accel_data->x = accel_raw.x * ACCEL_MG_LSB_4G;
    accel_data->y = accel_raw.y * ACCEL_MG_LSB_4G;
    accel_data->z = accel_raw.z * ACCEL_MG_LSB_4G;

    // Conversion from raw readings to uT then to Milligauss(mG) ; 1 Microtesla [µT] = 10 Milligauss [mG]
    mag_data->x = mag_raw.x * MAG_UT_LSB * 10;
    mag_data->y = mag_raw.y * MAG_UT_LSB * 10;
    mag_data->z = mag_raw.z * MAG_UT_LSB * 10;


    return error_status; // 0 - no error ; 1 - error

}



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
bool IMU_gyrotemp_read(int8_t * status, gyro_data_typedef * gyro_data, int8_t * temperature_data)
{

    bool error_status = 0;
    unsigned char data = 0;

    error_status = I2C1_burstRead(FXAS21002C_ADDRESS, FXAS21002C_REGISTER_STATUS, FXAS21002C_READ_LEN, buff);

    if(error_status > 0)
    {
        return 1; // error occurred
    }

    // Status register content
    *status = (int8_t)buff[0];

    if(*status < 0)
    {
        return 1; // error occurred
    }

    // copy the gyroscope byte data into 16 bit words
    gyro_raw.x = (buff[1] << 8) | buff[2];
    gyro_raw.y = (buff[3] << 8) | buff[4];
    gyro_raw.z = (buff[5] << 8) | buff[6];


    // Conversion from raw readings to millidps
    gyro_data->x = gyro_raw.x * GYRO_SENSITIVITY_2000DPS;
    gyro_data->y = gyro_raw.y * GYRO_SENSITIVITY_2000DPS;
    gyro_data->z = gyro_raw.z * GYRO_SENSITIVITY_2000DPS;

    // Conversion from dps to millrad/s
    gyro_data->x *= FXAS21002C_MDPS_TO_MRAD;
    gyro_data->y *= FXAS21002C_MDPS_TO_MRAD;
    gyro_data->z *= FXAS21002C_MDPS_TO_MRAD;

    error_status = I2C1_Read(FXAS21002C_ADDRESS, FXAS21002C_TEMP_REGISTER, &data);
    *temperature_data = (int8_t)(data * FXAS21002C_TEMP_SCALE);

    return error_status ; // 0 - no error ; 1 - error
}


/*---------------------------------------------------------------------------
*imu_test.c
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reuse:
*
**--------------------------------------------------------------------------*/


//---------------------------------------------------------------------------
//                              INCLUDES
//---------------------------------------------------------------------------

//Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "msp.h"

//Project Libraries


#include "clock.h"
#include "serial.h"
#include "imu.h"


//---------------------------------------------------------------------------
//                              DEFINES
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//                          FUNCTION PROTOTYPES
//---------------------------------------------------------------------------

void system_init(void);

//---------------------------------------------------------------------------
//                          EXTERNAL VARIABLES
//---------------------------------------------------------------------------

accel_data_typedef accel_data;
mag_data_typedef mag_data;
gyro_data_typedef gyro_data;
int8_t temperature_data;
int8_t status_accel;
int8_t status_gyro;


/////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     /* stop that dog watching us */
    serial_init();
    int j;

    system_init();
    printf("POST TEST - FX8700 : %d\n\r", FXOS8700_POST_test());
    printf("POST TEST - FXAS21002C : %d\n\r", FXAS21002C_POST_test());
    printf("IMU INIT : %d\n\r", IMU_init());

    while (1)
    {


        // Can read values now

        if (IMU_accelmag_read(&status_accel, &accel_data, &mag_data))
        {
            printf("Error occurred while reading accelerometer data\n\r");
            continue;
        }



        printf("Accelerometer Status : %d\n\r", status_accel);
        printf("[milliG] : accelX-axis : %d accelY-axis : %d accelZ-axis : %d\n\r", accel_data.x, accel_data.y, accel_data.z);
        printf("[milligauss] : magX-axis : %d magY-axis : %d magZ-axis : %d\n\r", mag_data.x, mag_data.y, mag_data.z);

        for(j = 0; j < 100; j++){};

        printf("\n\r");


        if (IMU_gyrotemp_read(&status_gyro, &gyro_data, &temperature_data))
        {
            printf("Error occurred while reading gyroscope data\n\r");
            continue;
        }


        printf("Gyroscope Status : %d\n\r", status_gyro);
        printf("[mrad/s] : gyroX-axis : %d gyroY-axis : %d gyroZ-axis : %d\n\r", gyro_data.x, gyro_data.y, gyro_data.z);
        printf("[degreeC] : Temperature : %d\n\r", temperature_data);

        printf("\n\r");

        for(j = 0; j < 5000; j++){};
    }



}

/////////////////////////////////////////////////////////////////////////
//void system_int(void)
/////////////////////////////////////////////////////////////////////////
//@brief:   initialization function.  Calls hardware configuration functions
//          to setup processor and peripherals.
//
//
//@param:   void
//@return:  void
//@outcome: SFRs configured
//////////////////////////////////////////////////////////////////////////

void system_init(void)
{
    clock_init();
    serial_init();
    I2C1_init();


}


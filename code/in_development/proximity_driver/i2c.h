/*---------------------------------------------------------------------------
*i2c.h
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reference : http://www.microdigitaled.com/ARM/MSP432_ARM/Code/MSP432_red_codes.htm
*
**--------------------------------------------------------------------------*/

#ifndef _I2C_H_
#define _I2C_H_


//---------------------------------------------------------------------------
//                              INCLUDES
//---------------------------------------------------------------------------

//libraries

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "msp.h"

//---------------------------------------------------------------------------
//                              DEFINES
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//                          EXTERNAL VARIABLES
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//                          FUNCTION PROTOTYPES
//---------------------------------------------------------------------------


/////////////////////////////////////////////////////////////////////////////
//void I2C1_init(void)
/////////////////////////////////////////////////////////////////////////////
//@brief: Initiates eUSCI -B1 for I2C operation.I2C initial configuration done here
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////
void I2C1_init(void);

/////////////////////////////////////////////////////////////////////////////
//int I2C1_Write(int slaveAddr, unsigned char memAddr, unsigned char data)
/////////////////////////////////////////////////////////////////////////////
//@brief: Write a single byte at memAddr
//        write: S-(slaveAddr+w)-ACK-memAddr-ACK-data-ACK-P
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////
bool I2C1_Write(int slaveAddr, unsigned char memAddr, unsigned char data);


/////////////////////////////////////////////////////////////////////////////
// int I2C1_Read(int slaveAddr, unsigned char memAddr, unsigned char* data)
/////////////////////////////////////////////////////////////////////////////
//@brief: Read a single byte at memAddr
//        read: S-(slaveAddr+w)-ACK-memAddr-ACK-R-(saddr+r)-ACK-data-NACK-P
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////

bool I2C1_Read(int slaveAddr, unsigned char memAddr, unsigned char* data);


/////////////////////////////////////////////////////////////////////////////
// int8_t I2C1_burstRead(int slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data)
/////////////////////////////////////////////////////////////////////////////
//@brief: Use burst read to read multiple bytes from consecutive locations
//        read: S-(slaveAddr+w)-ACK-memAddr-ACK-R-(slaveAddr+r)-ACK-data-ACK-...-data-NACK-P
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////

int8_t I2C1_burstRead(int slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data);



/////////////////////////////////////////////////////////////////////////////
// void lock_release(void)
/////////////////////////////////////////////////////////////////////////////
//@brief: Used to release i2c bus lock up
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////

void lock_release(void);

#endif  //_I2C_H_

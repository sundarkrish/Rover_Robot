/*---------------------------------------------------------------------------------------
*main.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
**---------------------------------------------------------------------------------------*/

#ifndef SERIAL_H_
#define SERIAL_H_


#include "msp.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"


//-------------------------------------------------------------------------------------------
//                                              DEFINES
//-------------------------------------------------------------------------------------------

//ASCII Values
#define CR 0x0D

//-------------------------------------------------------------------------------------------
//                                          EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
//                                          FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////
//void serial_init(void)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Initializes serial port (TX P1.2 and RX P1.3) for 8bit, 115,200 bps
//
//
//@param:   void
//@return:  void
//@outcome: serial port setup for UART
//
////////////////////////////////////////////////////////////////////////////////////////////

void serial_init(void);

////////////////////////////////////////////////////////////////////////////////////////////
//int fputc(int _c, register FILE *_fp)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Transmits a character over serial port
//
//
//@param:   int _c              -- character to transmit
//@param:   register FILE *_fp  -- pointer to file structure
//@return:  character transmitted
//@outcome: character is transmitted over serial port
//
////////////////////////////////////////////////////////////////////////////////////////////

int fputc(int _c, register FILE *_fp);

////////////////////////////////////////////////////////////////////////////////////////////
//int putchar(int c)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Transmits a character over serial port
//
//
//@param:   int _c              -- character to transmit
//@return:  character transmitted
//@outcome: character is transmitted over serial port
//
////////////////////////////////////////////////////////////////////////////////////////////

int putchar(int c);

////////////////////////////////////////////////////////////////////////////////////////////
//int fputs(const char *_ptr, register FILE *_fp)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   transmits a string over the serial port
//
//
//@param:   const char * _ptr   -- pointer to string being transmitted
//@param:   register FILE *_fp  -- pointer to file structure
//@return:  length of string transmitted
//@outcome: string transmitted over serial
//
////////////////////////////////////////////////////////////////////////////////////////////

int fputs(const char *_ptr, register FILE *_fp);

////////////////////////////////////////////////////////////////////////////////////////////
//int fgetc(FILE *_fp)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   receives a byte from serial port
//
//
//@param:   FILE * _fp -- pointer to file structure
//@return:  character received
//@outcome: character read from serial buffer and returned
//
////////////////////////////////////////////////////////////////////////////////////////////

int fgetc(FILE *_fp);

////////////////////////////////////////////////////////////////////////////////////////////
//int getchar(void)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   get a character from the serial port
//
//
//@param:   void
//@return:  int -- character read from serial port RX
//@outcome: byte received from serial port
//
////////////////////////////////////////////////////////////////////////////////////////////

int getchar(void);

////////////////////////////////////////////////////////////////////////////////////////////
//int getstr(char *s,unsigned int len)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief: gets a string from the user and stores it.  writes a NULL character
//        to end of string so other stdlib.h functions can operate on it.
//
//@param:   char *s -- pointer to string to be written in
//@param:   unsigned int len -- length of string (sizeof(str) - 1 aka no NULL)
//@return:  length of string
//@outcome: writes user input to string, returns length read
////////////////////////////////////////////////////////////////////////////////////////////

uint16_t  getstr(uint8_t *s,uint16_t len);

#endif /* SERIAL_H_ */

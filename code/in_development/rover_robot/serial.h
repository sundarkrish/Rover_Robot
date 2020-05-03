/*---------------------------------------------------------------------------------------
*serial.h
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
//                                     DEFINES
//-------------------------------------------------------------------------------------------

//ASCII Value Definitions
#define CR                      0x0D                        //enter button
#define SP                      0x20                        //space bar
#define BS                      0x7F                        //back space
#define ESC                     '\033'                      //esc key


#define RX_SBUF                 EUSCI_A0->RXBUF             //uart rx buffer definition
#define TX_SBUF                 EUSCI_A0->TXBUF             //uart tx buffer definition

#define _TX_INTERRUPTS_         //we are using TX interrupts, comment out to use blocking tx
#define _RX_INTERRUPTS_         //we are using RX interrupts, comment out to use blocking rx

#define _TX__FLOW_CONTROL_      //data will only be transmitted if software triggers TX interrupt

#define BUFFER_ALMOST_FULL      BUFFER_SIZE*(.75)    //threshold at which TX automatically occurs

//#define _BROKEN_MSP432_         //if your MSP432 is broken

#define BAUD                    9600

#ifdef _BROKEN_MSP432_
#define UART_RX                 0x04
#endif //_BROKEN_MSP432
//-------------------------------------------------------------------------------------------
//                                EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

typedef enum{                   //transmit execute flag enum
    transmit = 1,
    no_transmit = 0
}transmit_t;

typedef enum{                   //input/output type enum
    receiver,
    transmitter
}io_type_t;

//-------------------------------------------------------------------------------------------
//                                FUNCTION PROTOTYPES
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

/////////////////////////////////////////////////////////////////////////////////////////////
//void serial_interrupt_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables RX, TX (or both) serial interrupts
//
//
//@param:   void
//@return:  void
//@outcome: interrupts enabled based on preprocessor directives
/////////////////////////////////////////////////////////////////////////////////////////////

void serial_interrupt_init(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool serial_interrupts_enabled(uint8_t type)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks whether TX or RX interrupts are enabled based on passed parameter.
//
//
//@param:   uint8_t type -- RX or TX
//@return:  bool -- true if interrupt is enabled
//@outcome: interupt status is determined.
/////////////////////////////////////////////////////////////////////////////////////////////

bool serial_interrupts_enabled(uint8_t type);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void serial_tx_interrupt_enable(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables TX interrupts
//
//
//@param:   void
//@return:  void
//@outcome: tx interrupts are enabled.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_interrupt_enable(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void serial_tx_interrupt_disable(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   disables TX interrupts.
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_interrupt_disable(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void serial_rx_interrupt_enable(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables uart based rx interrupts
//
//
//@param:   void
//@return:  void
//@outcome: interrupts occur upon reception of data
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_rx_interrupt_enable(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void serial_rx_interrupt_disable(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   disables serial rx interrupts
//
//
//@param:   void
//@return:  void
//@outcome: receiving uart data doesn't cause an interrupt
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_rx_interrupt_disable(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline bool serial_rx_flag(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   gets the status of the rx interrupt flag
//
//
//@param:   void
//@return:  bool -- true if pending interrupt
//@outcome: returns rx interrupt flag
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool serial_rx_flag(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void serial_rx_flag_set(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets the rx interrupt flag high.
//
//
//@param:   void
//@return:  void
//@outcome: RX interrupt flag set high.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_rx_flag_set(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void serial_rx_flag_clear(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   clears the RX interrupt flag
//
//
//@param:   void
//@return:  void
//@outcome: RX interrupt flag set low.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_rx_flag_clear(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline bool serial_tx_flag(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   get the status of the tx interrupt flag
//
//
//@param:   void
//@return:  bool -- true if pending tx interrupt
//@outcome: returns status of tx interrupt flag.
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool serial_tx_flag(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void serial_tx_flag_set(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   sets the tx interrupt flag high
//
//
//@param:   void
//@return:  void
//@outcome: serial flag set high
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_flag_set(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//inline void serial_tx_flag_clear(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   clears tx interrupt flag.
//
//
//@param:   void
//@return:  void
//@outcome: tx interrupt flag cleared.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_flag_clear(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool serial_rx_buffer_empty(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks if there is data in the serial rx buffer
//
//
//@param:   void
//@return:  bool -- true if there is no data
//@outcome: determines if data is in the rx buffer.
/////////////////////////////////////////////////////////////////////////////////////////////

bool serial_rx_buffer_empty(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t serial_rx_buffer_space(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   returns the amount of space available in serial rx buffer.
//
//
//@param:   void
//@return:  uint16_t -- number of bytes remaining
//@outcome: rx buffer space remaining determined
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t serial_rx_buffer_space(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//bool serial_tx_buffer_empty(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks if serial tx buffer has data in it.
//
//
//@param:   void
//@return:  bool -- true if there is no data
//@outcome: determines if tx buffer is empty.
/////////////////////////////////////////////////////////////////////////////////////////////

bool serial_tx_buffer_empty(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t serial_tx_buffer_space(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   gets the amount of space remaining in serial tx buffer
//
//
//@param:   void
//@return:  uint16_t -- number of bytes remaining
//@outcome: tx buffer space remaining is determined
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t serial_tx_buffer_space(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t serial_read(uint8_t *ptr, uint16_t bytes)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads X data bytes out of rx buffer.
//
//
//@param:   uint8_t *ptr   -- pointer to location where rx buffer is being read into
//@param:   uint16_t bytes -- number of bytes we are reading from rx buffer
//@return:  number of bytes actually read, since there may less or no data available.
//@outcome: data read from rx buffer
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t serial_read(uint8_t *ptr, uint16_t bytes);

/////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t serial_read_byte(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads a single byte from the serial rx buffer.
//
//
//@param:   void
//@return:  uint8_t -- byte read back from serial rx buffer
//@outcome: a byte is read from serial rx buffer.
/////////////////////////////////////////////////////////////////////////////////////////////

uint8_t serial_read_byte(void);

/////////////////////////////////////////////////////////////////////////////////////////////
//uint16_t serial_write(uint8_t *data, uint16_t length, bool transmit)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   writes a number of bytes into the serial tx buffer, with option to immediately
//          transmit flush the buffer over the uart.
//
//@param:   uint8_t *data   -- pointer to data we are writing
//@param:   uint16_t length -- number of bytes we are writing
//@param:   bool transmit   -- transmit flag, true if send now, false if only buffer.
//@return:  uint16_t        -- number of bytes actually written, since buffer could be full.
//@outcome: bytes written into tx buffer with or without transmission.
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t serial_write(uint8_t *data, uint16_t length, bool transmit);

/////////////////////////////////////////////////////////////////////////////////////////////
//void serial_write_byte(uint8_t _c, bool transmit)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   writes a single byte into the serial tx buffer.
//
//
//@param:   uint8_t _c      -- the byte we are writing
//@param:   bool transmit   -- true is transmit now, false to just buffer
//@return:  void
//@outcome: a byte is written to the tx buffer, and either transmitted or buffered.
/////////////////////////////////////////////////////////////////////////////////////////////

void serial_write_byte(uint8_t _c, bool transmit);

/////////////////////////////////////////////////////////////////////////////////////////////
//void serial_transmit_buffer(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables TX interrupts so all data located in buffer is sent immediately
//
//
//@param:   void
//@return:  void
//@outcome: tx buffer is flushed and transmitted over serial port.
/////////////////////////////////////////////////////////////////////////////////////////////

void serial_transmit_buffer(void);

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

#ifndef _TX_INTERRUPTS_

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

#endif //_TX_INTERRUPTS_

#ifndef _RX_INTERRUPTS_

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

uint16_t getstr(uint8_t *s,uint16_t len);

#endif  //_RX_INTERRUPTS

#endif /* SERIAL_H_ */

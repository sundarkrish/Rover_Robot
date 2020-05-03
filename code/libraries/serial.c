/*---------------------------------------------------------------------------------------
*serial.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
**---------------------------------------------------------------------------------------
*@brief:
*
*
*Code Use:  Code is modified versions of MSP432 example code euscia0_uart_01
*
*--------------------------------------------------------------------------------------*/





#include "serial.h"

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

void serial_init(void)
{


    // Configure UART pins
    P1->SEL0 |= BIT2 | BIT3;                                        // set 2-UART pin as secondary function

    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;                         // Put eUSCI_A0 in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST |                         // keep eUSCI_A0 in reset
            EUSCI_B_CTLW0_SSEL__SMCLK;                              // Configure eUSCI clock source for SMCLK (UCSSEL = 10 aka SMCLK)
    // Baud Rate calculation
    // 12000000/(16*115200) = 6.5104
    // Fractional portion = 0.5104
    // User's Guide Table 24-4: UCBRSx = 0x20 - 0xAA
    // UCBRFx = int ( (78.125-78)*16) = 8
    EUSCI_A0->BRW = 6;                                              //12000000/16/115200 = 6.510
    EUSCI_A0->MCTLW |= ((BITD) |(BIT7) |                            //set UCBRF = 0x08 and UCBRS = 0x20 and UCOS16 = 1
            EUSCI_A_MCTLW_OS16);

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;                        // Initialize eUSCI
    EUSCI_A0->IFG &= ~(EUSCI_A_IFG_RXIFG);                          // Clear eUSCI RX interrupt flag
    EUSCI_A0->IFG |=  (EUSCI_A_IFG_TXIFG);                          // set eUSCI TX interrupt flag (we are clear to transmit)
}

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

int fputc(int _c, register FILE *_fp)
{
  //@TODO make serial_int_disable for RX or TX so we can pick.


  while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));    //wait until previous byte has finished transmitting
  EUSCI_A0->TXBUF = (unsigned char) _c;             //write the next byte, clear IFG_TX_IFG flag


  return((unsigned char)_c);
}

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

int putchar(int c)
{
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));    //wait until previous byte has finished transmitting
    EUSCI_A0->TXBUF = (unsigned char) c;             //write the next byte, clear IFG_TX_IFG flag

    return((unsigned char)c);
}

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

int fputs(const char *_ptr, register FILE *_fp)
{

  unsigned int i, len;

  len = strlen(_ptr);

  for(i=0 ; i<len ; i++)
  {
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));               //wait until previous byte finishes transmitting
    EUSCI_A0->TXBUF = (unsigned char) _ptr[i];                 //transmit the next byte, clears TXIFG flag
  }

  return len;
}

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

int fgetc(FILE *_fp)
{
    uint16_t c;

    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG));                //wait until we have received a byte
    c = EUSCI_A0->RXBUF;                                        //read RX buffer and clear RX flag

    return ((unsigned char)c);                                  //cast as a char
}

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

int getchar(void)
{
    uint16_t c;

    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG));                //wait until we have received a byte
    c = EUSCI_A0->RXBUF;                                        //read RX buffer and clear RX flag

    return ((unsigned char)c);                                  //cast as a char
}

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

uint16_t getstr(uint8_t *s,uint16_t len)
{
    int i = 0;

    while(i < len)          //read while we haven't reached end of buffer
    {
        *s = getchar();     //read in character
        putchar(*s);        //echo character back

        if(*s == CR)        //if enter is pressed
        {
            break;          //get out of loop
        }
        s++;                //increment pointer by one
        i++;                //increment index counter by one
    }
                            //whether we break out early, or we reached end of string
    *s = '\0';              //set end of string equal to NULL character

    #ifdef _debug_serial_
    printf("getstr(): > character entered: %d\n\r",i+1);       //received a byte package message
    #endif // _debug_

    return i;               //return length of string
}

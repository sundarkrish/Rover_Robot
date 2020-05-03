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


//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

#include "serial.h"
#include "circ_buff.h"


//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

circ_buff_t rx_buff;
circ_buff_t tx_buff;

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

inline bool serial_rx_flag(void);
inline bool serial_tx_flag(void);

//-------------------------------------------------------------------------------------------
//                                 FUNCTION DECLARATIONS
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

void serial_init(void)
{

    circ_buff_init(&rx_buff,BUFFER_SIZE);                                    //initialize rx_buffer
    circ_buff_init(&tx_buff,BUFFER_SIZE);                                    //initialize tx_buffer

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

    serial_interrupt_init();
}

////////////////////////////////////////////////////////////////////////////////////////////
//void serial_int_enable(void)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables interrupts for serial reception
//
//
//@param:   void
//@return:  void
//@outcome: interrupt occurs when RX receives a packet
//
////////////////////////////////////////////////////////////////////////////////////////////

void serial_interrupt_init(void)
{
    serial_rx_flag_clear();                                                 // Clear eUSCI RX interrupt flag

    #ifdef _RX_INTERRUPTS_
        serial_rx_interrupt_enable();                                       // Enable USCI_A0 TX/RX interrupt
    #endif //_RX_INTERRUPTS

    #ifdef _TX_INTERRUPTS_                                                  //if tx interrupts are enabled
        serial_tx_flag_set();                                               //
    #else
        serial_tx_flag_set();                                               // set eUSCI TX interrupt flag (we are clear to transmit)
    #endif //_TX_INTERRUPTS

    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);                             // Enable eUSCIA0 interrupt in NVIC module
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_interrupt_enable(void)
{
    EUSCI_A0->IE |= EUSCI_A_IE_TXIE;                              // Enable USCI_A0 TX/RX interrupt
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_interrupt_disable(void)
{
    EUSCI_A0->IE &= ~EUSCI_A_IE_TXIE;                              // Enable USCI_A0 TX/RX interrupt
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_rx_interrupt_enable(void)
{
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;                              // Enable USCI_A0 TX/RX interrupt
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_rx_interrupt_disable(void)
{
    EUSCI_A0->IE &= ~EUSCI_A_IE_RXIE;                              // Enable USCI_A0 TX/RX interrupt
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool serial_rx_flag(void)
{
    return (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_rx_flag_set(void)
{
    EUSCI_A0->IFG |=  EUSCI_A_IFG_RXIFG;                            // set eUSCI TX interrupt flag (we are clear to transmit)
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_rx_flag_clear(void)
{
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline bool serial_tx_flag(void)
{
    return (EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_flag_set(void)
{
    EUSCI_A0->IFG |=  EUSCI_A_IFG_TXIFG;                            // set eUSCI TX interrupt flag (we are clear to transmit)
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_flag_clear(void)
{
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_TXIFG;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

bool serial_interrupts_enabled(uint8_t type)
{
    if(type == transmitter)
    {
        return (EUSCI_A0->IE & EUSCI_A_IE_TXIE);
    }
    else
    {
        return (EUSCI_A0->IE & EUSCI_A_IE_RXIE);
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

bool serial_rx_buffer_empty(void)
{
    return circ_buff_empty(&rx_buff);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t serial_rx_buffer_space(void)
{
    return circ_buff_space(&rx_buff) ;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

bool serial_tx_buffer_empty(void)
{
    return circ_buff_empty(&tx_buff);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t  serial_tx_buffer_space(void)
{
    return circ_buff_space(&tx_buff);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t serial_read(uint8_t *ptr, uint16_t bytes)
{
    uint16_t byte_count = 0;                                //number of bytes we've read

    while(!circ_buff_empty(&rx_buff))                       //while there is data in the rx_buffer
    {
        circ_buff_get(&rx_buff,ptr++);                      //pass a byte from rx_buffer through pointer
        byte_count++;                                       //increment byte counter

        if(byte_count == bytes) { break;}                   //if we've read the requested number of bytes quit
    }

    return byte_count;                                      //return number of bytes read, since it can be different than requested value
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

uint8_t serial_read_byte(void)
{
    uint8_t data;

    if(circ_buff_get(&rx_buff, &data))
    {
        return data;
    }

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t serial_write(uint8_t *data, uint16_t length, bool transmit)
{
    uint16_t bytes = 0;
    int error = 0;

    //@TODO determine if I want to always transmit previous data in the buffer while I'm writing
    //into it, or do I want to write first, transmit later.
    if(circ_buff_used(&tx_buff) == BUFFER_ALMOST_FULL)              //if buffer is almost full
    {
        serial_transmit_buffer();                                   //start flushing
    }

    while(bytes < length)                                               //while there is still data
    {
        //@TODO: if serial buffer is full, flush it immediately
        error = circ_buff_put(&tx_buff, *(data + bytes));
        if(error == -1) { break; }                                      //overflow has occurred
        bytes++;
    }

    if(transmit)                                                        //if we are transmitting as well
    {
            serial_transmit_buffer();                                   //cause tx serial interrupt
    }

    return bytes;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

void serial_write_byte(uint8_t _c, bool transmit)
{

    if(circ_buff_used(&tx_buff) == BUFFER_ALMOST_FULL)              //if buffer is full, immediately flush buffer
    {
        serial_transmit_buffer();
    }

    circ_buff_put(&tx_buff, _c);                                    //write a byte into the tx_buff

    if(transmit)                                                    //if we are transmitting immediately
    {
        serial_transmit_buffer();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////////////////////

void serial_transmit_buffer(void)
{
    serial_tx_interrupt_enable();
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

#ifdef _TX_INTERRUPTS_

    circ_buff_put(&tx_buff, _c);                                   //put a character into the tx_buff

#else

    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));                    //wait until previous byte has finished transmitting
    EUSCI_A0->TXBUF = (unsigned char) _c;                           //write the next byte, clear IFG_TX_IFG flag

#endif //_TX_INTERRUPTS

    return((unsigned char)_c);
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
    uint16_t bytes = 0;

#ifdef _TX_INTERRUPTS_
    int error = 0;                                                      //error flag

    while(*_ptr)                                                     //while we haven't reached the end of the string
    {
        error = circ_buff_put(&tx_buff,*_ptr++);                    //put a character in the tx_buff
        if(error == -1) { break; }                                  //if buffer is full break out of loop

        bytes++;                                                    //number of bytes written into buffer
    }
#else   //Blocking transmission
    while(*_ptr)
    {
        putchar(*_ptr++);
        bytes++;
    }
#endif //_TX_INTERRUPTS

    return bytes;                                   //return number of bytes written
}



////////////////////////////////////////////////////////////////////////////////////////////
//int putchar(int c)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Transmits a character over serial port.  BLOCKING
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

/////////////////////////////////////////////////////////////////////////
//void EUSCIA0_IRQHandler(void)
/////////////////////////////////////////////////////////////////////////
//@brief:   Serial interrupt service routine.  Receives bytes over UART
//          and store them in buffer.  when TX interrupts triggered
//          gets data from buffer and transmits.
//
//@param:   void
//@return:  void
//@outcome: data transmitted and received as needed
/////////////////////////////////////////////////////////////////////////

void EUSCIA0_IRQHandler(void)
{
    uint8_t data;

    serial_rx_interrupt_disable();                      //disable receiver interrupts

    if(serial_rx_flag())                                //if a character was received
    {
        circ_buff_put(&rx_buff, RX_SBUF);               //read from serial port and store in rx_buff (clears RX flag)
    }

    serial_rx_interrupt_enable();                       //enable receiver interrupts, so we can get data even during tx interrupt

    if(serial_tx_flag())                                //if transmitter is ready
    {
        if(circ_buff_get(&tx_buff, &data))              //if there is data available
        {
            TX_SBUF = data;                             //write it into TX_SBUF
        }
        else
        {
            serial_tx_interrupt_disable();              //disable transmitter interrupts before transmitting

        }
    }
}

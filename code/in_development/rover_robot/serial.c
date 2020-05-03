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

circ_buff_t rx_buff;                                    //serial rx buffer for storing received data
circ_buff_t tx_buff;                                    //serial tx buffer for storing data for transmission

static volatile bool transmit_now = false;              //transmit flag, only transmit if software triggered TX interrupt
                                                        //otherwise an RX interrupt could also cause a transmit to occur if data
                                                        //is available.
//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

inline bool serial_rx_flag(void);                       //checks status of rx interrupt flag
inline bool serial_tx_flag(void);                       //checks status of tx interrupt flag

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
#if BAUD == 115200
    EUSCI_A0->BRW = 6;                                              //12000000/16/115200 = 6.510
    EUSCI_A0->MCTLW |= ((BITD) |(BIT7) |                            //set UCBRF = 0x08 and UCBRS = 0x20 and UCOS16 = 1
            EUSCI_A_MCTLW_OS16);
#elif BAUD == 9600
    EUSCI_A0->BRW = 78;                     // 12000000/16/9600
    EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;
#endif
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;                        // Initialize eUSCI
#ifdef _BROKEN_MSP432_
    PMAP->KEYID = PMAP_KEYID_VAL;                                       //unlock the port map registers
    P3->DIR &= ~UART_RX;                                            //set pin to input
    P3MAP->PMAP_REGISTER2 = PMAP_UCA0RXD;                           //map Pin3.2 to UART RX Peripheral
    P3->SEL1 |= UART_RX;                                            //set port 3.2 function to PMAP
    P3->SEL0 &= ~UART_RX;
    PMAP->KEYID = 0;                                                    //lock the port map registers
#endif //_BROKEN_MSP432
    serial_interrupt_init();
}

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
//bool serial_interrupts_enabled(uint8_t type)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   checks whether TX or RX interrupts are enabled based on passed parameter.
//
//
//@param:   uint8_t type -- RX or TX
//@return:  bool -- true if interrupt is enabled
//@outcome: interupt status is determined.
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
//inline void serial_tx_interrupt_enable(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables TX interrupts
//
//
//@param:   void
//@return:  void
//@outcome: tx interrupts are enabled.
/////////////////////////////////////////////////////////////////////////////////////////////

inline void serial_tx_interrupt_enable(void)
{
    EUSCI_A0->IE |= EUSCI_A_IE_TXIE;                              // Enable USCI_A0 TX/RX interrupt
}

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

inline void serial_tx_interrupt_disable(void)
{
    EUSCI_A0->IE &= ~EUSCI_A_IE_TXIE;                              // Enable USCI_A0 TX/RX interrupt
}

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

inline void serial_rx_interrupt_enable(void)
{
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;                              // Enable USCI_A0 TX/RX interrupt
}

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

inline void serial_rx_interrupt_disable(void)
{
    EUSCI_A0->IE &= ~EUSCI_A_IE_RXIE;                              // Enable USCI_A0 TX/RX interrupt
}

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


inline bool serial_rx_flag(void)
{
    return (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG);
}

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

inline void serial_rx_flag_set(void)
{
    EUSCI_A0->IFG |=  EUSCI_A_IFG_RXIFG;                            // set eUSCI TX interrupt flag (we are clear to transmit)
}

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

inline void serial_rx_flag_clear(void)
{
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;
}

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

inline bool serial_tx_flag(void)
{
    return (EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG);
}

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

inline void serial_tx_flag_set(void)
{
    EUSCI_A0->IFG |=  EUSCI_A_IFG_TXIFG;                            // set eUSCI TX interrupt flag (we are clear to transmit)
}

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

inline void serial_tx_flag_clear(void)
{
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_TXIFG;
}

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

bool serial_rx_buffer_empty(void)
{
    return circ_buff_empty(&rx_buff);
}

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

uint16_t serial_rx_buffer_space(void)
{
    return circ_buff_space(&rx_buff) ;
}

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

bool serial_tx_buffer_empty(void)
{
    return circ_buff_empty(&tx_buff);
}

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

uint16_t  serial_tx_buffer_space(void)
{
    return circ_buff_space(&tx_buff);
}

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
//uint8_t serial_read_byte(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   reads a single byte from the serial rx buffer.
//
//
//@param:   void
//@return:  uint8_t -- byte read back from serial rx buffer
//@outcome: a byte is read from serial rx buffer.
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
//void serial_transmit_buffer(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   enables TX interrupts so all data located in buffer is sent immediately
//
//
//@param:   void
//@return:  void
//@outcome: tx buffer is flushed and transmitted over serial port.
/////////////////////////////////////////////////////////////////////////////////////////////

void serial_transmit_buffer(void)
{

    #ifdef _TX__FLOW_CONTROL_
        transmit_now = true;
    #endif  //_TX_FLOW_CONTROL_

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

    #ifndef _TX__FLOW_CONTROL_                          //if we aren't using flow control
        transmit_now = true;                            //the data will be transmitted now
    #endif //_TX_FLOW_CONTROL_

        if((circ_buff_get(&tx_buff, &data) && transmit_now))              //if there is data available
        {
            TX_SBUF = data;                             //write it into TX_SBUF
        }
        else
        {
            serial_tx_interrupt_disable();              //disable transmitter interrupts before transmitting
            transmit_now = false;
        }
    }
}

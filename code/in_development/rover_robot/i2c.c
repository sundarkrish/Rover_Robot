/*---------------------------------------------------------------------------
*i2c.c
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reference: http://www.microdigitaled.com/ARM/MSP432_ARM/Code/MSP432_red_codes.htm(I2C read,write and burst read procedure)
*
**--------------------------------------------------------------------------*/

//---------------------------------------------------------------------------
//                              INCLUDES
//---------------------------------------------------------------------------

//libraries
#include <i2c.h>

//Custom Libraries

//---------------------------------------------------------------------------
//                              DEFINES
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//                          EXTERNAL VARIABLES
//---------------------------------------------------------------------------
static uint8_t lock_detect = 0;
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
void I2C1_init(void)
{
    EUSCI_B1->CTLW0 |= 1;       /* disable UCB1 during config */
    EUSCI_B1->CTLW0 = 0x0F81;   /* 7-bit slave addr, master, I2C, synch mode, use SMCLK (SMCLK uses DCO @ 12MHz)*/
    EUSCI_B1->BRW = 30;         /* set clock prescaler 12MHz / 30 = 400kHz */
    P6->SEL0 |= 0x30;           /* P6.5, P6.4 for UCB1 */
    P6->SEL1 &= ~0x30;
    EUSCI_B1->CTLW0 &= ~1;      /* enable UCB1 after config */
    lock_detect = 0;
}

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
bool I2C1_Write(int slaveAddr, unsigned char memAddr, unsigned char data)
{
    EUSCI_B1->I2CSA = slaveAddr;    /* setup slave address */
    EUSCI_B1->CTLW0 |= 0x0010;      /* enable transmitter */
    EUSCI_B1->CTLW0 |= 0x0002;      /* generate START and send slave address */
    lock_detect = 0;
    while((EUSCI_B1->CTLW0 & 2) && (200 > lock_detect++));   /* wait until slave address is sent */
    if (lock_detect > 200) return 1;
    if (EUSCI_B1->IFG & 0x0020) return 1;  /* NACK received from slave */
    if (EUSCI_B1->IFG & 0x0010) return 1;  /* Arbitration lost */
    EUSCI_B1->TXBUF = memAddr;      /* send memory address to slave */
    lock_detect = 0;
    while(!(EUSCI_B1->IFG & 2) && (200 > lock_detect++));    /* wait till it's ready to transmit */
    if (lock_detect > 200) return 1;
    if (EUSCI_B1->IFG & 0x0020) return 1;  /* NACK received from slave */
    EUSCI_B1->TXBUF = data;         /* send data to slave */
    lock_detect = 0;
    while(!(EUSCI_B1->IFG & 2) && (200 > lock_detect++));    /* wait till last transmit is done */
    if (lock_detect > 200) return 1;
    if (EUSCI_B1->IFG & 0x0020) return 1;  /* NACK received from slave */
    EUSCI_B1->CTLW0 |= 0x0004;      /* send STOP */
    lock_detect = 0;
    while((EUSCI_B1->CTLW0 & 4) && (200 > lock_detect++)) ;    /* wait until STOP is sent */
    if (lock_detect > 200) return 1;
    return 0;                       /* no error */
}


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
/* Read a single byte at memAddr
 * read: S-(slaveAddr+w)-ACK-memAddr-ACK-R-(saddr+r)-ACK-data-NACK-P
 */
bool I2C1_Read(int slaveAddr, unsigned char memAddr, unsigned char* data)
{
    EUSCI_B1->I2CSA = slaveAddr;    /* setup slave address */
    EUSCI_B1->CTLW0 |= 0x0010;      /* enable transmitter */
    EUSCI_B1->CTLW0 |= 0x0002;      /* generate START and send slave address */
    lock_detect = 0;
    while((EUSCI_B1->CTLW0 & 2) && (200 > lock_detect++));   /* wait until slave address is sent */
    if (lock_detect > 200) return 1;
    if (EUSCI_B1->IFG & 0x0020) return 1;  /* NACK received from slave */
    if (EUSCI_B1->IFG & 0x0010) return 1;  /* Arbitration lost */
    EUSCI_B1->TXBUF = memAddr;      /* send memory address to slave */
    lock_detect = 0;
    while(!(EUSCI_B1->IFG & 2) && (200 > lock_detect++));    /* wait till it's ready to transmit */
    if (lock_detect > 200) return 1;
    if (EUSCI_B1->IFG & 0x0020) return 1;  /* NACK received from slave */
    if (EUSCI_B1->IFG & 0x0010) return 1;  /* Arbitration lost */
    EUSCI_B1->CTLW0 &= ~0x0010;     /* enable receiver */
    EUSCI_B1->CTLW0 |= 0x0002;      /* generate RESTART and send slave address */
    lock_detect = 0;
    while((EUSCI_B1->CTLW0 & 2) && (200 > lock_detect++));     /* wait till restart is finished */
    if (lock_detect > 200) return 1;
    EUSCI_B1->CTLW0 |= 0x0004;      /* setup to send STOP after the byte is received */
    lock_detect = 0;
    while(!(EUSCI_B1->IFG & 1) && (200 > lock_detect++));    /* wait till data is received */
    if (lock_detect > 200) return 1;
    *data = EUSCI_B1->RXBUF;        /* read the received data */
    lock_detect = 0;
    while((EUSCI_B1->CTLW0 & 4) && (200 > lock_detect++)) ;    /* wait until STOP is sent */
    if (lock_detect > 200) return 1;
    return 0;                       /* no error */
}


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

int8_t I2C1_burstRead(int slaveAddr, unsigned char memAddr, int byteCount, unsigned char* data)
{

    if (byteCount <= 0)
    {

        return -1; /* No read was performed */
    }

    EUSCI_B1->CTLW0 |= 0x0010; /* Enable trasnmitter mode */
    EUSCI_B1->I2CSA= slaveAddr; /* assign slave address */
    EUSCI_B1->CTLW0 |= 0X0002; /* Transmit START bit and slave address */
    lock_detect = 0;
    while((EUSCI_B1->CTLW0 & 0x0002) && (200 > lock_detect++)); /* wait until START bit and slave address are transmitted */
    if (lock_detect > 200) return 1;
    if (EUSCI_B1->IFG & 0x0020) return 1;  /* NACK received from slave */
    if (EUSCI_B1->IFG & 0x0010) return 1;  /* Arbitration lost */
    EUSCI_B1->TXBUF = memAddr; /* Send memory/register address to slave */
    lock_detect = 0;
    while(!(EUSCI_B1->IFG & 2) && (200 > lock_detect++)); /* Wait till transmit is done */
    if (lock_detect > 200) return 1;
    if (EUSCI_B1->IFG & 0x0020) return 1;  /* NACK received from slave */
    // In master receiver mode when START bit is set, a  REPEATED START occurs
    //It is preceded by a NACK
    EUSCI_B1->CTLW0 &= ~0x0010;       /* Transmitter mode off, enable receiver mode */
        EUSCI_B1->CTLW0 |= 0x0002;        /* generate RESTART and send slave address */
        lock_detect = 0;
        while((EUSCI_B1->CTLW0 & 2) && (200 > lock_detect++));       /* wait till RESTART is finished */
        if (lock_detect > 200) return 1;
        if (EUSCI_B1->IFG & 0x0020) return 1;  /* NACK received from slave */
        /* receive data one byte at a time */
        do {
            if (byteCount == 1)             /* when only one byte of data is left */
                EUSCI_B1->CTLW0 |= 0x0004; /* setup to send STOP after the last byte is received */

            lock_detect = 0;
            while(!(EUSCI_B1->IFG & 1) && (200 > lock_detect++));  /* wait till data is received */
            if (lock_detect > 200) return 1;
            *data = EUSCI_B1->RXBUF;    /* read the received data */
            data++;
            byteCount--;
        } while (byteCount);

        lock_detect = 0;
        while((EUSCI_B1->CTLW0 & 4) && (200 > lock_detect++)) ;      /* wait until STOP is sent */
        if (lock_detect > 200) return 1;

        return byteCount;

        /*  0 - no error */
        /*  -1 - no read was done */
        /*  1 - error */
        /*  >1 - error.Reading did not happen due to some eror*/



}



/////////////////////////////////////////////////////////////////////////////
// void lock_release(void)
/////////////////////////////////////////////////////////////////////////////
//@brief: Used to release i2c bus lock up.Contains the reset sequence
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////

bool lock_release(void)
{


    EUSCI_B1->IFG &= ~0x0020; // Clear NACK flag
    // Sending the STOP signal make master to release control over the bus
    EUSCI_B1->CTLW0 |= 0x0004;      /* send STOP */
    lock_detect = 0;
    while((EUSCI_B1->CTLW0 & 4) && (200 > lock_detect++)) ;    /* wait until STOP is sent */
    if (lock_detect > 200) return 1;

    return 0; // 0 no error ; 1 error
}
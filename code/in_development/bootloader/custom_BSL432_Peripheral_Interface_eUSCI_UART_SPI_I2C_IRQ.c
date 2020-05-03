/**
 * TEXAS INSTRUMENTS TEXT FILE LICENSE
 * 
 * 
 * Copyright (c) 2014 - 2017 Texas Instruments Incorporated
 * 
 * All rights reserved not granted herein.
 * 
 * Limited License.  
 * 
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive license under copyrights and patents it now or hereafter owns or controls to make, have made, use, import, offer to sell and sell ("Utilize") this software subject to the terms herein.  With respect to the foregoing patent license, such license is granted  solely to the extent that any such patent is necessary to Utilize the software alone.  The patent license shall not apply to any combinations which include this software, other than combinations with devices manufactured by or for TI ("TI Devices").  No hardware patent is licensed hereunder.
 * 
 * Redistributions must preserve existing copyright notices and reproduce this license (including the above copyright notice and the disclaimer and (if applicable) source code license limitations below) in the documentation and/or other materials provided with the distribution
 * 
 * Redistribution and use in binary form, without modification, are permitted provided that the following conditions are met:
 * 
 * *    No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any software provided in binary form.
 * 
 * *    any redistribution and use are licensed by TI for use only with TI Devices.
 * 
 * *    Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
 * 
 * If software source code is provided to you, modification and redistribution of the source code are permitted provided that the following conditions are met:
 * 
 * *    any redistribution and use of the source code, including any resulting derivative works, are licensed by TI for use only with TI Devices.
 * 
 * *    any redistribution and use of any object code compiled from the source code and any resulting derivative works, are licensed by TI for use only with TI Devices.
 * 
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * DISCLAIMER.
 * 
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file BSL432_Peripheral_Interface_eUSCI_UART_SPI_I2C_IRQ.c
 * \brief Source file for the BSL Peripheral Interface functions.
 */

/*
 * Changelog
 * 2015-05-29  0103  MG  Adapting for MSP432P401R Rev C.
 *                       Adding TLV parser, using more DriverLib ROM functions
 *                       Disable unused PI modules
 * 2016-02-16  0201  MG  IRQ-based peripheral interface.
 * 2016-02-18  0202  MG  Separating SPI IRQ routines into RX/TX.
 * 2016-03-06  0203  MG  Disable IRQs after RX until ready for TX
 *                       Added delay for UART detect to make it half-duplex
 * 2016-03-08  0204  MG  Changed SMCLK to 12 MHz
 * 2016-07-25  0205  MG  Disable UARD RXD pin IRQ before LPM entry.
 *                       Disable UART RXD pin IRQ when I2C or SPI was detected.
 * 2017-11-28  0205  FS  Add MSP432P4111, MSP432P411V, and MSP432P411Y differentiation in FCTL_A and SYSCTL_A.
 */

#include "BSL432_API.h"
#include "BSL432_Peripheral_Interface.h"
#include "BSL432_Command_Interpreter.h"
#include "BSL432_Command_Definitions.h"
#if defined (__MSP432P401R__) || defined (__MSP432P401M__)
   #include "MSP432P401x/BSL432_device_file.h"
#elif defined (__MSP432P4111__) || defined (__MSP432P411V__) || defined (__MSP432P411Y__)
   #include "MSP432P4111/BSL432_device_file.h"
#else
#endif

#pragma RETAIN(BSL432_VersionPeripheralInterface)
#pragma DATA_SECTION(BSL432_VersionPeripheralInterface, ".BSL432_VERSION_PI")
const uint8_t BSL432_VersionPeripheralInterface[2] = {0x02, 0x05};

BSL432_eventFlags BSL432_PI_eventFlags = Flags_rxPacketOngoing;

#pragma DATA_ALIGN(BSL432_RAM_TX_Buf, 2)
#pragma NOINIT(BSL432_RAM_TX_Buf)
volatile uint8_t BSL432_RAM_TX_Buf[BSL432_MAX_BUFFER_SIZE + 6];
#pragma DATA_ALIGN(BSL432_RAM_RX_Buf, 2)
#pragma NOINIT(BSL432_RAM_RX_Buf)
volatile uint8_t BSL432_RAM_RX_Buf[BSL432_MAX_BUFFER_SIZE + 6];

/* Function pointers to the current peripherals functions */
void (*BSL432_PI_sendByte)(uint8_t) = 0;
uint8_t (*BSL432_PI_receiveByte)() = 0;
void (*BSL432_PI_sendData)(uint32_t) = 0;
BSL432_eventFlags (*BSL432_PI_receivePacket)() = 0;

BSL432_ActivePeripherals BSL432_ActivePeripheral = BSL432_AP_Auto;
uint32_t BSL432_UART_MODULE = EUSCI_A0_BASE;
uint32_t BSL432_SPI_MODULE = EUSCI_B0_BASE;
uint32_t BSL432_I2C_MODULE = EUSCI_B2_BASE;
uint16_t BSL432_I2C_SLAVE_ADDRESS = 0x48;
/* UART pin settings for configuration after baud rate detection */
uint8_t BSL432_UART_PORT = 0;
uint16_t BSL432_UART_RXD_PIN = 0;
uint16_t BSL432_UART_TXD_PIN = 0;
uint8_t BSL432_UART_RXD_MODE = 0;
uint8_t BSL432_UART_TXD_MODE = 0;
uint8_t BSL432_UART_RXD_IES_PREV_STATE = 0;
uint8_t BSL432_UART_RXD_IE_PREV_STATE = 0;

/* Counting variables for received and sent bytes, state of RX/TX */
volatile uint32_t BSL432_PI_IRQ_RxDataCnt = 0;
volatile uint32_t BSL432_PI_IRQ_TxDataCnt = 0;
volatile uint32_t BSL432_PI_IRQ_TxDataSent = 0;
volatile BSL432_IRQ_states BSL432_IRQ_state = IRQ_idle;


/**
 * PI initialization routine configures all peripherals.
 *
 * \return None
 */
void BSL432_PI_init()
{
    // Set Flash wait state to 1 (up to 24 MHz)
#if defined (__MSP432P401R__) || defined (__MSP432P401M__)
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 1);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 1);
#elif defined (__MSP432P4111__) || defined (__MSP432P411V__) || defined (__MSP432P411Y__)
    MAP_FlashCtl_A_setWaitState(FLASH_A_BANK0, 2);
    MAP_FlashCtl_A_setWaitState(FLASH_A_BANK1, 2);
#else
    #error Device not defined.
#endif

    // Configure clock system
    // MCLK = 24 MHz, ACLK = 32768 Hz
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_32KHZ);
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFO_32KHZ, CS_CLOCK_DIVIDER_1);

    // Configure power mode and core voltage
    MAP_PCM_setPowerState(PCM_AM_LDO_VCORE0);

    // Extract BSL parameters given by boot code or user application
    // Interface selection (15:13)
    if(((BSL432_BSL_PARAMS >> 13) & 0b111) == BSL432_AP_UART)
    {
        BSL432_ActivePeripheral = BSL432_AP_UART;
    }
    else if(((BSL432_BSL_PARAMS >> 13) & 0b111) == BSL432_AP_SPI)
    {
        BSL432_ActivePeripheral = BSL432_AP_SPI;
    }
    else if(((BSL432_BSL_PARAMS >> 13) & 0b111) == BSL432_AP_I2C)
    {
        BSL432_ActivePeripheral = BSL432_AP_I2C;
    }
    else  // Default to auto interface selection
    {
        BSL432_ActivePeripheral = BSL432_AP_Auto;
    }

    // I2C slave address from BSL parameters (25:16)
    BSL432_I2C_SLAVE_ADDRESS = (BSL432_BSL_PARAMS >> 16) & 0x3FF;

    BSL432_PI_configureInterfaces();

    // Initialize peripheral
    switch(BSL432_ActivePeripheral) {
        case BSL432_AP_Auto:
            BSL432_PI_initUARTdetect();
            BSL432_PI_initSPI();
            BSL432_PI_initI2C();
            break;
        case BSL432_AP_UART:
            BSL432_PI_initUARTdetect();
            break;
        case BSL432_AP_SPI:
            BSL432_PI_initSPI();
            break;
        case BSL432_AP_I2C:
            BSL432_PI_initI2C();
            break;
        default:
            break;
    }

    BSL432_ReceiveBuffer = (uint8_t *)BSL432_RAM_RX_Buf;
    BSL432_SendBuffer = (uint8_t *)BSL432_RAM_TX_Buf;

    MAP_Interrupt_enableMaster();
}

/**
 * PI interface configuration routine that sets up the used peripheral modules and handles the port mapping.
 *
 * \return None
 *
 */
void BSL432_PI_configureInterfaces()
{
    // Read configuration from TLV
    struct
    {
        uint32_t BSLPeripheralInterfaceSelection;
        uint32_t BSLPortInterfaceConfigurationUART;
        uint32_t BSLPortInterfaceConfigurationSPI;
        uint32_t BSLPortInterfaceConfigurationI2C;
    } * BSL432_TLV_BSL_Configuration;

    uint_fast8_t BSL432_TLV_Length;

#if defined (__MSP432P401R__) || defined (__MSP432P401M__)
    MAP_SysCtl_getTLVInfo(BSL432_TLV_BSL_CONFIGURATION_TAG, BSL432_TLV_INSTANCE_BSL_PERIPHERAL_INTERFACE_SELECTION, &BSL432_TLV_Length, (uint32_t **)&BSL432_TLV_BSL_Configuration);
#elif defined (__MSP432P4111__) || defined (__MSP432P411V__) || defined (__MSP432P411Y__)
    MAP_SysCtl_A_getTLVInfo(BSL432_TLV_BSL_CONFIGURATION_TAG, BSL432_TLV_INSTANCE_BSL_PERIPHERAL_INTERFACE_SELECTION, &BSL432_TLV_Length, (uint32_t **)&BSL432_TLV_BSL_Configuration);
#else
#error Device not defined.
#endif

    // UART module
    if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_UART)
    {
        if(BSL432_BSL_PER_IF_SEL_UART_MOD == 0x0)  // eUSCI_A
        {
            if(BSL432_BSL_PER_IF_SEL_UART_INST == 0x0)
            {
                BSL432_UART_MODULE = EUSCI_A0_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_UART_INST == 0x1)
            {
                BSL432_UART_MODULE = EUSCI_A1_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_UART_INST == 0x2)
            {
                BSL432_UART_MODULE = EUSCI_A2_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_UART_INST == 0x3)
            {
                BSL432_UART_MODULE = EUSCI_A3_BASE;
            }
        }
    }

    // SPI module
    if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_SPI)
    {
        if(BSL432_BSL_PER_IF_SEL_SPI_MOD == 0x0)  // eUSCI_A
        {
            if(BSL432_BSL_PER_IF_SEL_SPI_INST == 0x0)
            {
                BSL432_SPI_MODULE = EUSCI_A0_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_SPI_INST == 0x1)
            {
                BSL432_SPI_MODULE = EUSCI_A1_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_SPI_INST == 0x2)
            {
                BSL432_SPI_MODULE = EUSCI_A2_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_SPI_INST == 0x3)
            {
                BSL432_SPI_MODULE = EUSCI_A3_BASE;
            }
        }
        else if(BSL432_BSL_PER_IF_SEL_SPI_MOD == 0x1)  // eUSCI_B
        {
            if(BSL432_BSL_PER_IF_SEL_SPI_INST == 0x0)
            {
                BSL432_SPI_MODULE = EUSCI_B0_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_SPI_INST == 0x1)
            {
                BSL432_SPI_MODULE = EUSCI_B1_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_SPI_INST == 0x2)
            {
                BSL432_SPI_MODULE = EUSCI_B2_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_SPI_INST == 0x3)
            {
                BSL432_SPI_MODULE = EUSCI_B3_BASE;
            }
        }
    }

    // I2C module
    if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_I2C)
    {
        if(BSL432_BSL_PER_IF_SEL_I2C_MOD == 0x0)  // eUSCI_B
        {
            if(BSL432_BSL_PER_IF_SEL_I2C_INST == 0x0)
            {
                BSL432_I2C_MODULE = EUSCI_B0_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_I2C_INST == 0x1)
            {
                BSL432_I2C_MODULE = EUSCI_B1_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_I2C_INST == 0x2)
            {
                BSL432_I2C_MODULE = EUSCI_B2_BASE;
            }
            else if(BSL432_BSL_PER_IF_SEL_I2C_INST == 0x3)
            {
                BSL432_I2C_MODULE = EUSCI_B3_BASE;
            }
        }
    }

    const uint8_t ports_mapping[] =  // Port mapping between BSL configuration and Driver Library
    {
        GPIO_PORT_P1, GPIO_PORT_P2, GPIO_PORT_P3, GPIO_PORT_P4,
        GPIO_PORT_P5, GPIO_PORT_P6, GPIO_PORT_P7, GPIO_PORT_P8,
        GPIO_PORT_P9, GPIO_PORT_P10, GPIO_PORT_PJ, GPIO_PORT_P1,  // Note: only 11 ports supported in DriverLib for now
        GPIO_PORT_P1, GPIO_PORT_P1, GPIO_PORT_P1, GPIO_PORT_P1
    };

    const uint16_t pins_mapping[] =  // Pin mapping between BSL configuration and Driver Library
    {
        GPIO_PIN0, GPIO_PIN1, GPIO_PIN2, GPIO_PIN3,
        GPIO_PIN4, GPIO_PIN5, GPIO_PIN6, GPIO_PIN7
    };

    // PxSELy mapping between BSL configuration and Driver Library: mode_mapping[PxSEL1][PxSEL0]
    const uint8_t mode_mapping[2][2] =
    {
        {0, GPIO_PRIMARY_MODULE_FUNCTION},
        {GPIO_SECONDARY_MODULE_FUNCTION, GPIO_TERTIARY_MODULE_FUNCTION}
    };

    // UART module muxing
    if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_UART)
    {
        if(BSL432_BSL_PER_IF_SEL_UART_MUX == 0x1)
        {
            // Save UART configuration for now, configure later in detection and active communication routines
            BSL432_UART_PORT = ports_mapping[BSL432_BSL_PORTCNF_UART_UART_PORT];
            BSL432_UART_RXD_PIN = pins_mapping[BSL432_BSL_PORTCNF_UART_BITPOS_RXD];
            BSL432_UART_TXD_PIN = pins_mapping[BSL432_BSL_PORTCNF_UART_BITPOS_TXD];
            BSL432_UART_RXD_MODE = mode_mapping[BSL432_BSL_PORTCNF_UART_PSEL1_RXD][BSL432_BSL_PORTCNF_UART_PSEL0_RXD];
            BSL432_UART_TXD_MODE = mode_mapping[BSL432_BSL_PORTCNF_UART_PSEL1_TXD][BSL432_BSL_PORTCNF_UART_PSEL0_TXD];
        }
    }

    // SPI module muxing
    if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_SPI)
    {
        if(BSL432_BSL_PER_IF_SEL_SPI_MUX == 0x1)
        {
            if(BSL432_BSL_PORTCNF_SPI_PSEL0_SOMI != 0 || BSL432_BSL_PORTCNF_SPI_PSEL1_SOMI != 0)
            {
                MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ports_mapping[BSL432_BSL_PORTCNF_SPI_SPI_PORT],
                        pins_mapping[BSL432_BSL_PORTCNF_SPI_BITPOS_SOMI],
                        mode_mapping[BSL432_BSL_PORTCNF_SPI_PSEL1_SOMI][BSL432_BSL_PORTCNF_SPI_PSEL0_SOMI]);
            }
            if(BSL432_BSL_PORTCNF_SPI_PSEL0_SIMO != 0 || BSL432_BSL_PORTCNF_SPI_PSEL1_SIMO != 0)
            {
                MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ports_mapping[BSL432_BSL_PORTCNF_SPI_SPI_PORT],
                        pins_mapping[BSL432_BSL_PORTCNF_SPI_BITPOS_SIMO],
                        mode_mapping[BSL432_BSL_PORTCNF_SPI_PSEL1_SIMO][BSL432_BSL_PORTCNF_SPI_PSEL0_SIMO]);
            }
            if(BSL432_BSL_PORTCNF_SPI_PSEL0_CLK != 0 || BSL432_BSL_PORTCNF_SPI_PSEL1_CLK != 0)
            {
                MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ports_mapping[BSL432_BSL_PORTCNF_SPI_SPI_PORT],
                        pins_mapping[BSL432_BSL_PORTCNF_SPI_BITPOS_CLK],
                        mode_mapping[BSL432_BSL_PORTCNF_SPI_PSEL1_CLK][BSL432_BSL_PORTCNF_SPI_PSEL0_CLK]);
            }
            if(BSL432_BSL_PORTCNF_SPI_PSEL0_STE != 0 || BSL432_BSL_PORTCNF_SPI_PSEL1_STE != 0)
            {
                MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ports_mapping[BSL432_BSL_PORTCNF_SPI_SPI_PORT],
                        pins_mapping[BSL432_BSL_PORTCNF_SPI_BITPOS_STE],
                        mode_mapping[BSL432_BSL_PORTCNF_SPI_PSEL1_STE][BSL432_BSL_PORTCNF_SPI_PSEL0_STE]);
            }
        }
    }

    // I2C module muxing
    if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_I2C)
    {
        if(BSL432_BSL_PER_IF_SEL_I2C_MUX == 0x1)
        {
            if(BSL432_BSL_PORTCNF_I2C_PSEL0_SCL != 0 || BSL432_BSL_PORTCNF_I2C_PSEL1_SCL != 0)
            {
                MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ports_mapping[BSL432_BSL_PORTCNF_I2C_I2C_PORT],
                        pins_mapping[BSL432_BSL_PORTCNF_I2C_BITPOS_SCL],
                        mode_mapping[BSL432_BSL_PORTCNF_I2C_PSEL1_SCL][BSL432_BSL_PORTCNF_I2C_PSEL0_SCL]);
            }
            if(BSL432_BSL_PORTCNF_I2C_PSEL0_SDA != 0 || BSL432_BSL_PORTCNF_I2C_PSEL1_SDA != 0)
            {
                MAP_GPIO_setAsPeripheralModuleFunctionInputPin(ports_mapping[BSL432_BSL_PORTCNF_I2C_I2C_PORT],
                        pins_mapping[BSL432_BSL_PORTCNF_I2C_BITPOS_SDA],
                        mode_mapping[BSL432_BSL_PORTCNF_I2C_PSEL1_SDA][BSL432_BSL_PORTCNF_I2C_PSEL0_SDA]);
            }
        }
    }
}

/**
 * PI routine that scans all active interfaces for received data
 *
 * \return None
 */
void BSL432_PI_detectInterface()
{
    // BSL timeout loop
    while(1)
    {
        // Check if timeout period passed
        if(MAP_Timer_A_getInterruptStatus(TIMER_A0_BASE) == TIMER_A_INTERRUPT_PENDING)
        {
            /* Make sure the UART RXD pin cannot wake the device from LPM
             * unless it was configured for wake up previously. */
            MAP_GPIO_disableInterrupt(BSL432_UART_PORT, BSL432_UART_RXD_PIN);
            MAP_GPIO_unregisterInterrupt(BSL432_UART_PORT);
            /* Restore previous IEs and IE state of UARD RXD pin. */
            if(BSL432_UART_PORT == GPIO_PORT_P1) {
                /* Set or clear the IE(S) bits depending on previous states.
                 * Variables with previous states contain the bit mask. */
                P1->IES = (P1->IES & ~BSL432_UART_RXD_PIN) | BSL432_UART_RXD_IES_PREV_STATE;//P1IES = (P1IES & ~BSL432_UART_RXD_PIN) | BSL432_UART_RXD_IES_PREV_STATE;
                P1->IE = (P1->IE & ~BSL432_UART_RXD_PIN) | BSL432_UART_RXD_IE_PREV_STATE;//P1IE = (P1IE & ~BSL432_UART_RXD_PIN) | BSL432_UART_RXD_IE_PREV_STATE;
            }
            else if(BSL432_UART_PORT == GPIO_PORT_P2) {
                /* Set or clear the IE(S) bits depending on previous states.
                 * Variables with previous states contain the bit mask. */
                P2->IES = (P2->IES & ~BSL432_UART_RXD_PIN) | BSL432_UART_RXD_IES_PREV_STATE;//P2IES = (P2IES & ~BSL432_UART_RXD_PIN) | BSL432_UART_RXD_IES_PREV_STATE;
                P2->IE = (P2->IE & ~BSL432_UART_RXD_PIN) | BSL432_UART_RXD_IE_PREV_STATE;//P2IE = (P2IE & ~BSL432_UART_RXD_PIN) | BSL432_UART_RXD_IE_PREV_STATE;
            }
#ifdef _DEBUG
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            while(1);  // Prevent entering LPM during debug.
#endif
            BSL432_API_enterLPM();
        }

        // Check for activity on the peripheral interfaces
        if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_UART)
        {
            if(BSL432_RAM_RX_Buf[BSL432_MAX_BUFFER_SIZE + 5] == BSL432_AP_UART)
            {
                BSL432_ActivePeripheral = BSL432_AP_UART;
                BSL432_PI_disableSPI();
                BSL432_PI_disableI2C();
                break;
            }
        }
        if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_SPI)
        {
            if(BSL432_RAM_RX_Buf[BSL432_MAX_BUFFER_SIZE + 5] == BSL432_AP_SPI)
            {
                BSL432_ActivePeripheral = BSL432_AP_SPI;
                /* Configure function pointers for current peripheral */
                BSL432_PI_sendByte = &BSL432_PI_sendByteSPI;
                BSL432_PI_receiveByte = 0;
                BSL432_PI_sendData = &BSL432_PI_sendDataSPI;
                BSL432_PI_receivePacket = BSL432_PI_receivePacketSPI;
                BSL432_PI_disableUART();
                BSL432_PI_disableI2C();
                break;
            }
        }
        if(BSL432_ActivePeripheral == BSL432_AP_Auto || BSL432_ActivePeripheral == BSL432_AP_I2C)
        {
            if(BSL432_RAM_RX_Buf[BSL432_MAX_BUFFER_SIZE + 5] == BSL432_AP_I2C)
            {
                BSL432_ActivePeripheral = BSL432_AP_I2C;
                /* Configure function pointers for current peripheral */
                BSL432_PI_sendByte = &BSL432_PI_sendByteI2C;
                BSL432_PI_receiveByte = 0;
                BSL432_PI_sendData = &BSL432_PI_sendDataI2C;
                BSL432_PI_receivePacket = &BSL432_PI_receivePacketI2C;
                BSL432_PI_disableUART();
                BSL432_PI_disableSPI();
                break;
            }
        }
    }
}

/**
 * PI initialization routine for passive (RX only) UART auto baudrate detection.
 *
 * \return None
 */
void BSL432_PI_initUARTdetect()
{
    /* Configure and enable SysTick. Set the reload value to the maximum */
    MAP_SysTick_setPeriod(1000000);
    MAP_SysTick_enableModule();

    /* Save previous IEs and IE state of UARD RXD pin. Will be restored on LPM entry. */
    if(BSL432_UART_PORT == GPIO_PORT_P1) {
        BSL432_UART_RXD_IES_PREV_STATE = P1->IES & BSL432_UART_RXD_PIN;//BSL432_UART_RXD_IES_PREV_STATE = P1IES & BSL432_UART_RXD_PIN;
        BSL432_UART_RXD_IE_PREV_STATE = P1->IE & BSL432_UART_RXD_PIN;//BSL432_UART_RXD_IE_PREV_STATE = P1IE & BSL432_UART_RXD_PIN;
    }
    else if(BSL432_UART_PORT == GPIO_PORT_P2) {
        BSL432_UART_RXD_IES_PREV_STATE = P2->IES & BSL432_UART_RXD_PIN;//BSL432_UART_RXD_IES_PREV_STATE = P2IES & BSL432_UART_RXD_PIN;
        BSL432_UART_RXD_IE_PREV_STATE = P2->IE & BSL432_UART_RXD_PIN;//BSL432_UART_RXD_IE_PREV_STATE = P2IE & BSL432_UART_RXD_PIN;
    }
    /* Set up RX pin for input with pull-up for baud rate detection */
    MAP_GPIO_setAsInputPinWithPullUpResistor(BSL432_UART_PORT, BSL432_UART_RXD_PIN);
    MAP_GPIO_interruptEdgeSelect(BSL432_UART_PORT, BSL432_UART_RXD_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_registerInterrupt(BSL432_UART_PORT, BSL432_PI_UARTdetect);
    MAP_GPIO_enableInterrupt(BSL432_UART_PORT, BSL432_UART_RXD_PIN);
}

/**
 * PI auto baud rate detection
 * Checks for the used baud rate with the help of the 0xFF synchronisation character.
 * Called when high/low IRQ is triggered on UART RX pin.
 *
 * /returns None
 */
void BSL432_PI_UARTdetect()
{

/*
    BSL432_PI_initUART(BSL432_UARTBaudRates115200);
    BSL432_PI_sendByteUART(BSL432_ACK);
*/

    /*
     * Synchronization charater 0xFF
     * ____   ________________   ____
     *     |_|1 2 3 4 5 6 7 8 |_|
     */
    MAP_GPIO_clearInterruptFlag(BSL432_UART_PORT, BSL432_UART_RXD_PIN);

    /* Check if pin is already low... */
    SysTick->LOAD = BSL432_UART_BAUDRATE_CPS_9600_MAX;  // Timeout of lowest baud rate (STRVR)
    SysTick->VAL = 0; // Reset SysTick to period value (STCVR)
    SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);  // (STCSR)
    if(!(MAP_GPIO_getInputPinValue(BSL432_UART_PORT, BSL432_UART_RXD_PIN)))
    {
        /* ... wait for pin high with */
        while(!(MAP_GPIO_getInputPinValue(BSL432_UART_PORT, BSL432_UART_RXD_PIN)))
        {
            /* Check for timeout */
            if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            {
                return; // BSL432_UARTBaudRatesAuto;
            }
        }
        /* Pin went high, count for 8 symbols until pin goes low. */
        SysTick->LOAD = (BSL432_UART_BAUDRATE_CPS_9600_MAX * 8);  // Timeout for lowest baud rate * 8 symbols
        SysTick->VAL = 0; // Reset SysTick to period value
        SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);

        /* Check for timeout */
        while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
        {
            //return; // BSL432_UARTBaudRatesAuto;
        }

        /* Evaluate elapsed time */
        uint32_t currentSysTick = SysTick->VAL;
        currentSysTick = (BSL432_UART_BAUDRATE_CPS_9600_MAX * 8) - currentSysTick;

/*        if(currentSysTick > (8 * BSL432_UART_BAUDRATE_CPS_115200_MIN) && currentSysTick < (8 * BSL432_UART_BAUDRATE_CPS_115200_MAX))  // 115200 baud
        {
            BSL432_PI_initUART(BSL432_UARTBaudRates115200);
            BSL432_PI_sendByteUART(BSL432_ACK);
            return;
        }
        else if(currentSysTick > (8 * BSL432_UART_BAUDRATE_CPS_57600_MIN) && currentSysTick < (8 * BSL432_UART_BAUDRATE_CPS_57600_MAX))  // 57600 baud
        {
            BSL432_PI_initUART(BSL432_UARTBaudRates57600);
            BSL432_PI_sendByteUART(BSL432_ACK);
            return;
        }
        else if(currentSysTick > (8 * BSL432_UART_BAUDRATE_CPS_38400_MIN) && currentSysTick < (8 * BSL432_UART_BAUDRATE_CPS_38400_MAX))  // 38400 baud
        {
            BSL432_PI_initUART(BSL432_UARTBaudRates38400);
            BSL432_PI_sendByteUART(BSL432_ACK);
            return;
        }
        else if(currentSysTick > (8 * BSL432_UART_BAUDRATE_CPS_19200_MIN) && currentSysTick < (8 * BSL432_UART_BAUDRATE_CPS_19200_MAX))  // 19200 baud
        {
            BSL432_PI_initUART(BSL432_UARTBaudRates19200);
            BSL432_PI_sendByteUART(BSL432_ACK);
            return;
        }
        else if(currentSysTick > (8 * BSL432_UART_BAUDRATE_CPS_9600_MIN) && currentSysTick < (8 * BSL432_UART_BAUDRATE_CPS_9600_MAX))  // 9600 baud
        {*/
        BSL432_PI_initUART(BSL432_UARTBaudRates9600);
        BSL432_PI_sendByteUART(BSL432_ACK);
/*            return;*/
/*        }
        else
        {
             No known baud rate detected
            return;
        }*/
    }
    return;



}

/**
 * PI initialization routine for UART.
 *
 * \param BSL432_UARTBaudRate The baud rate parameter.
 *
 * \return None
 */
void BSL432_PI_initUART(BSL432_UARTBaudRates BSL432_UARTBaudRate)
{
    const eUSCI_UART_Config BSL432_UARTConfig_auto =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        156,  // UCBRx
        4,    // firstModReg UCBRFx
        0,    // secondMod UCBRSx
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };

    const eUSCI_UART_Config BSL432_UARTConfig_9600 =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        78,  // UCBRx
        2,    // firstModReg UCBRFx
        0,    // secondMod UCBRSx
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };

    const eUSCI_UART_Config BSL432_UARTConfig_19200 =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        39,  // UCBRx
        1,   // firstModReg UCBRFx
        0,   // secondMod UCBRSx
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };

    const eUSCI_UART_Config BSL432_UARTConfig_38400 =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        19,  // UCBRx
        8,   // firstModReg UCBRFx
        85,   // secondMod UCBRSx
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };

    const eUSCI_UART_Config BSL432_UARTConfig_57600 =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        13,  // UCBRx
        0,   // firstModReg UCBRFx
        37,   // secondMod UCBRSx
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };

    const eUSCI_UART_Config BSL432_UARTConfig_115200 =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        6,  // UCBRx
        8,   // firstModReg UCBRFx
        32,  // secondMod UCBRSx
        EUSCI_A_UART_NO_PARITY,
        EUSCI_A_UART_LSB_FIRST,
        EUSCI_A_UART_ONE_STOP_BIT,
        EUSCI_A_UART_MODE,
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
    };

    /* Disable SysTick */
    MAP_SysTick_disableModule();

    /* Selecting RXD and TXD pins in UART mode */
    if(BSL432_UART_RXD_MODE != 0)
    {
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(BSL432_UART_PORT,
                BSL432_UART_RXD_PIN, BSL432_UART_RXD_MODE);
    }
    if(BSL432_UART_TXD_MODE != 0)
    {
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(BSL432_UART_PORT,
                BSL432_UART_TXD_PIN, BSL432_UART_TXD_MODE);
    }

    /* Configure function pointers for current peripheral */
    BSL432_PI_sendByte = &BSL432_PI_sendByteUART;
    BSL432_PI_receiveByte = 0;
    BSL432_PI_sendData = &BSL432_PI_sendDataUART;
    BSL432_PI_receivePacket = &BSL432_PI_receivePacketUART;

    /* Configuring UART Module */
    switch (BSL432_UARTBaudRate) {
        case BSL432_UARTBaudRatesAuto:
            MAP_UART_initModule(BSL432_UART_MODULE, &BSL432_UARTConfig_auto);
            break;
        case BSL432_UARTBaudRates9600:
            MAP_UART_initModule(BSL432_UART_MODULE, &BSL432_UARTConfig_9600);
            break;
        case BSL432_UARTBaudRates19200:
            MAP_UART_initModule(BSL432_UART_MODULE, &BSL432_UARTConfig_19200);
            break;
        case BSL432_UARTBaudRates38400:
            MAP_UART_initModule(BSL432_UART_MODULE, &BSL432_UARTConfig_38400);
            break;
        case BSL432_UARTBaudRates57600:
            MAP_UART_initModule(BSL432_UART_MODULE, &BSL432_UARTConfig_57600);
            break;
        case BSL432_UARTBaudRates115200:
            MAP_UART_initModule(BSL432_UART_MODULE, &BSL432_UARTConfig_115200);
            break;
        default:
            MAP_UART_initModule(BSL432_UART_MODULE, &BSL432_UARTConfig_9600);
            break;
    }

    MAP_GPIO_disableInterrupt(BSL432_UART_PORT, BSL432_UART_RXD_PIN);
    MAP_GPIO_unregisterInterrupt(BSL432_UART_PORT);

    BSL432_RAM_RX_Buf[BSL432_MAX_BUFFER_SIZE + 5] = BSL432_AP_UART;  // Signal that UART is PI active now

    // Short delay to ensure "half-duplex" UART communication at lower baud rate
    uint32_t i = 1000;
    for(i = 1000; i > 0; i--) {
        __no_operation();
    }

    /* Enable UART module */
    MAP_UART_enableModule(BSL432_UART_MODULE);
    MAP_UART_registerInterrupt(BSL432_UART_MODULE, BSL432_PI_IRQ_UART);
    MAP_UART_enableInterrupt(BSL432_UART_MODULE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/**
 * PI initialization routine for SPI.
 *
 * Configures the SPI module and send and receive functions.
 *
 * \return None
 */
void BSL432_PI_initSPI()
{
    const eUSCI_SPI_SlaveConfig BSL432_SPIConfig =
    {
        EUSCI_SPI_MSB_FIRST,
        EUSCI_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
        EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,
        EUSCI_SPI_4PIN_UCxSTE_ACTIVE_LOW
    };

    MAP_SPI_initSlave(BSL432_SPI_MODULE, &BSL432_SPIConfig);

    /* Enable SPI module */
    MAP_SPI_enableModule(BSL432_SPI_MODULE);
    MAP_SPI_registerInterrupt(BSL432_SPI_MODULE, BSL432_PI_IRQ_SPIdetect);
    MAP_SPI_enableInterrupt(BSL432_SPI_MODULE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
}

/**
 * PI initialization routine for I2C.
 *
 * Configures the I2C module and send and receive functions.
 *
 * \return None
 */
void BSL432_PI_initI2C()
{
    MAP_I2C_initSlave(BSL432_I2C_MODULE, BSL432_I2C_SLAVE_ADDRESS,
            EUSCI_B_I2C_OWN_ADDRESS_OFFSET0, EUSCI_B_I2C_OWN_ADDRESS_ENABLE);

    /* Enable I2C module */
    MAP_I2C_enableModule(BSL432_I2C_MODULE);
    MAP_SPI_registerInterrupt(BSL432_I2C_MODULE, BSL432_PI_IRQ_I2Cdetect);
    MAP_SPI_enableInterrupt(BSL432_I2C_MODULE, EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_TRANSMIT_INTERRUPT0);
}

/**
 * PI disable function for UART
 *
 * Disables the UART module.
 *
 * \return None
 */
void BSL432_PI_disableUART()
{
    MAP_GPIO_disableInterrupt(BSL432_UART_PORT, BSL432_UART_RXD_PIN);
    MAP_GPIO_unregisterInterrupt(BSL432_UART_PORT);
    MAP_UART_disableModule(BSL432_UART_MODULE);
}

/**
 * PI disable function for SPI
 *
 * Disables the SPI module.
 *
 * \return None
 */
void BSL432_PI_disableSPI()
{
    MAP_SPI_disableModule(BSL432_SPI_MODULE);
}

/**
 * PI disable function for I2C
 *
 * Disables the I2C module.
 *
 * \return None
 */
void BSL432_PI_disableI2C()
{
    MAP_I2C_disableModule(BSL432_I2C_MODULE);
}

/**
 * PI initialization routine for BSL timeout using Timer_A
 *
 * \return None
 */
void BSL432_PI_initTimeout()
{
    // Timer_A is sourced by ACLK (32768 Hz) with a devider of 10

    const Timer_A_UpModeConfig BSL432_TimeoutConfig =
    {
        TIMER_A_CLOCKSOURCE_ACLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_10,
        BSL432_TIMEOUT,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
        TIMER_A_DO_CLEAR
    };

    MAP_Timer_A_stopTimer(TIMER_A0_BASE);

    MAP_Timer_A_clearInterruptFlag(TIMER_A0_BASE);

    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &BSL432_TimeoutConfig);

    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
}

/**
 * Processes PI commands (for UART only)
 *
 * returns None
 */
void BSL432_PI_interpretPICommand()
{
    uint8_t command = BSL432_ReceiveBuffer[3];
    if(command == BSL432_CHANGE_BAUD_RATE)
    {
        uint8_t rate = BSL432_ReceiveBuffer[4];
        if((rate < BSL432_UARTBaudRates9600) || (rate > BSL432_UARTBaudRates115200))
        {
            BSL432_PI_sendByte(BSL432_UNKNOWN_BAUD_RATE);
        }
        else
        {
            BSL432_PI_sendByte(BSL432_ACK);
            while(MAP_UART_queryStatusFlags(BSL432_UART_MODULE, EUSCI_A_UART_BUSY));
            BSL432_PI_initUART((BSL432_UARTBaudRates)rate);
        }
    }
}

/**
 * \return the maximum data buffer size for the current PI
 */
uint32_t BSL432_PI_getBufferSize()
{
    return (uint32_t)BSL432_MAX_BUFFER_SIZE;
}

/**
 * Verifies the data in the receive buffer against a given checksum
 */
uint32_t BSL432_PI_verifyData(uint32_t checksum)
{
    MAP_CRC32_setSeed(0xFFFF, CRC16_MODE);  // Initialize CRC

    uint32_t bufferIndex = 0;
    for(bufferIndex = 3; bufferIndex < (BSL432_ReceiveBufferSize + 3); bufferIndex++)
    {
        MAP_CRC32_set8BitDataReversed(BSL432_ReceiveBuffer[bufferIndex], CRC16_MODE);
    }
    return (MAP_CRC32_getResult(CRC16_MODE) == checksum);
}

/**
 * Reads and entire packet from the peripheral, verifies it,
 * and sends it to the core to be interpreted
 *
 * \return BSL event flags
 */
BSL432_eventFlags BSL432_PI_receivePacketUART()
{
    BSL432_PI_eventFlags = Flags_rxPacketOngoing;

    uint32_t dataPointer = 0;
    uint32_t checksum = 0;

    while(BSL432_IRQ_state == IRQ_receiving)
    {
        if(dataPointer == 0 && BSL432_PI_IRQ_RxDataCnt > 0)  // first byte is the size of the Core packet
        {
            if(BSL432_ReceiveBuffer[0] != 0x80)  // first byte in packet should be 0x80
            {
                BSL432_PI_sendByte(BSL432_HEADER_INCORRECT);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_idle;
            }
            else
            {
                dataPointer++;
            }
        }
        else if (dataPointer == 1 && BSL432_PI_IRQ_RxDataCnt > 1)
        {
            BSL432_ReceiveBufferSize = BSL432_ReceiveBuffer[1];
            dataPointer++;
        }
        else if (dataPointer == 2 && BSL432_PI_IRQ_RxDataCnt > 2)
        {
            BSL432_ReceiveBufferSize |= (uint32_t)BSL432_ReceiveBuffer[2] << 8;
            if(BSL432_ReceiveBufferSize == 0)
            {
                BSL432_PI_sendByte(BSL432_PACKET_SIZE_ZERO);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_idle;
            }
            if(BSL432_ReceiveBufferSize > BSL432_MAX_BUFFER_SIZE)  // For future devices that might need smaller packets
            {
                BSL432_PI_sendByte(BSL432_PACKET_SIZE_TOO_BIG);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_idle;
            }
            dataPointer = BSL432_ReceiveBufferSize + 3;
        }
        else if(dataPointer == (BSL432_ReceiveBufferSize + 3) && BSL432_PI_IRQ_RxDataCnt > (BSL432_ReceiveBufferSize + 3))
        {
            // if the pointer is pointing to the Checksum low data byte which resides
            // after 0x80, rSize, Core Command.
            checksum = BSL432_ReceiveBuffer[BSL432_ReceiveBufferSize + 3];
            dataPointer++;
        }
        else if(dataPointer == (BSL432_ReceiveBufferSize + 4) && BSL432_PI_IRQ_RxDataCnt > (BSL432_ReceiveBufferSize + 4))
        {
            // if the pointer is pointing to the Checksum low data byte which resides
            // after 0x80, rSize, Core Command, CKL.
            checksum = checksum | BSL432_ReceiveBuffer[BSL432_ReceiveBufferSize + 4]<<8;
            if(BSL432_PI_verifyData(checksum))
            {
                if((BSL432_ReceiveBuffer[3] & 0xF0) == BSL432_PI_COMMAND_UPPER)
                {
                    BSL432_PI_interpretPICommand();
                    BSL432_PI_eventFlags = Flags_rxPacketOngoing;
                    dataPointer = 0;
                    checksum = 0;
                    BSL432_IRQ_state = IRQ_idle;
                }
                else
                {
                    BSL432_PI_sendByte(BSL432_ACK);
                    BSL432_PI_eventFlags = Flags_data_received;
                    BSL432_IRQ_state = IRQ_readytosend;
                }
            }
            else
            {
                BSL432_PI_sendByte(BSL432_CHECKSUM_INCORRECT);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_idle;
            }
        }
    }
    return BSL432_PI_eventFlags;
}

/**
 * Sends a single byte via UART peripheral
 * Waits for the send buffer of the peripheral to be empty.
 *
 * \param data The byte to be transmitted.
 *
 * \return None
 */
void BSL432_PI_sendByteUART(uint8_t data)
{
    /* Wait until the TX buffer is empty, then send */
    while(!(MAP_UART_getInterruptStatus(BSL432_UART_MODULE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)));
    MAP_UART_transmitData(BSL432_UART_MODULE, data);
}

/**
 * Sends data in the data buffer via UART
 *
 * \param size The number of byte to send from the data buffer
 *
 * \return None
 */
void BSL432_PI_sendDataUART(uint32_t size)
{
    BSL432_PI_sendByteUART(0x80);  // BSL package header
    BSL432_PI_sendByteUART(size & 0xFF);  // BSL package size
    BSL432_PI_sendByteUART((size >> 8) & 0xFF);
    MAP_CRC32_setSeed(0xFFFF, CRC16_MODE);  // Initialize CRC
    uint32_t bufferIndex = 0;
    for(bufferIndex = 3; bufferIndex < (size + 3); bufferIndex++)
    {
        MAP_CRC32_set8BitDataReversed(BSL432_RAM_TX_Buf[bufferIndex], CRC16_MODE);
        MAP_UART_transmitData(BSL432_UART_MODULE, BSL432_RAM_TX_Buf[bufferIndex]);
    }
    BSL432_PI_sendByteUART(MAP_CRC32_getResult(CRC16_MODE) & 0xFF);  // BSL package CRC
    BSL432_PI_sendByteUART((MAP_CRC32_getResult(CRC16_MODE) >> 8) & 0xFF);
    BSL432_IRQ_state = IRQ_idle;
}

/**
 * Reads and entire packet from the SPI peripheral, verifies it,
 * and sends it to the core to be interpreted
 *
 * \return BSL event flags
 */
BSL432_eventFlags BSL432_PI_receivePacketSPI()
{
    BSL432_PI_eventFlags = Flags_rxPacketOngoing;

    uint32_t dataPointer = 0;
    uint32_t checksum = 0;

    while(BSL432_IRQ_state == IRQ_receiving)
    {
        // Header byte of BSL data packet
        if(dataPointer == 0 && BSL432_PI_IRQ_RxDataCnt > 0)
        {
            if(BSL432_ReceiveBuffer[0] != 0x80)  // first byte in packet must be 0x80
            {
                BSL432_PI_sendByteImmediatelySPI(BSL432_HEADER_INCORRECT);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_idle;
            }
            else
            {
                dataPointer++;
            }
        }
        // Length (low) byte of BSL package
        else if (dataPointer == 1 && BSL432_PI_IRQ_RxDataCnt > 1)
        {
            BSL432_ReceiveBufferSize = BSL432_ReceiveBuffer[1];
            dataPointer++;
        }
        // Length (high) byte of BSL package
        else if (dataPointer == 2 && BSL432_PI_IRQ_RxDataCnt > 2)
        {
            BSL432_ReceiveBufferSize |= (uint32_t)BSL432_ReceiveBuffer[2] << 8;
            if(BSL432_ReceiveBufferSize == 0)
            {
                BSL432_PI_sendByteImmediatelySPI(BSL432_PACKET_SIZE_ZERO);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_idle;
            }
            if(BSL432_ReceiveBufferSize > BSL432_MAX_BUFFER_SIZE)  // For future devices that might need smaller packets
            {
                BSL432_PI_sendByteImmediatelySPI(BSL432_PACKET_SIZE_TOO_BIG);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_idle;
            }
            dataPointer = BSL432_ReceiveBufferSize + 3;
        }
        // CRC (low) byte of BSL package
        else if(dataPointer == (BSL432_ReceiveBufferSize + 3) && BSL432_PI_IRQ_RxDataCnt > (BSL432_ReceiveBufferSize + 3))
        {
            checksum = BSL432_ReceiveBuffer[BSL432_ReceiveBufferSize + 3];
            dataPointer++;
        }
        // CRC (high) byte of BSL package
        else if(dataPointer == (BSL432_ReceiveBufferSize + 4) && BSL432_PI_IRQ_RxDataCnt > (BSL432_ReceiveBufferSize + 4))
        {
            checksum = checksum | BSL432_ReceiveBuffer[BSL432_ReceiveBufferSize + 4]<<8;
            if(BSL432_PI_verifyData(checksum))
            {
                MAP_Interrupt_disableMaster();  // Disable interrupt until TX buffer is prepared
                BSL432_PI_sendByteImmediatelySPI(BSL432_ACK);
                BSL432_PI_eventFlags = Flags_data_received;
                BSL432_IRQ_state = IRQ_readytosend;
            }
            else
            {
                BSL432_PI_sendByteImmediatelySPI(BSL432_CHECKSUM_INCORRECT);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_idle;
            }
        }
    }
    return BSL432_PI_eventFlags;
}

/**
 * Sends a single byte via SPI peripheral
 * Adds byte to the send buffer.
 *
 * \param data The byte to be transmitted.
 *
 * \return None
 */
void BSL432_PI_sendByteSPI(uint8_t data)
{
    BSL432_RAM_TX_Buf[BSL432_PI_IRQ_TxDataCnt++] = data;
}

/**
 * * Sends a single byte via SPI peripheral directly without using the send buffer
 *
 * \param data The byte to be transmitted.
 *
 * \return None
 */
void BSL432_PI_sendByteImmediatelySPI(uint8_t data)
{
    MAP_SPI_transmitData(BSL432_SPI_MODULE, data);
}

/**
 * Sends data in the data buffer via SPI
 *
 * This function sends the data in the global BSL432_SendBuffer.
 *
 * \param size Number of bytes to send.
 *
 * \return None
 */
void BSL432_PI_sendDataSPI(uint32_t size)
{
    BSL432_PI_sendByteSPI(0x80);  // BSL package header
    BSL432_PI_sendByteSPI(size & 0xFF);  // BSL package size
    BSL432_PI_sendByteSPI((size >> 8) & 0xFF);
    MAP_CRC32_setSeed(0xFFFF, CRC16_MODE);  // Initialize CRC
    uint32_t bufferIndex = 0;
    for(bufferIndex = 3; bufferIndex < (size + 3); bufferIndex++)
    {
        MAP_CRC32_set8BitDataReversed(BSL432_RAM_TX_Buf[bufferIndex], CRC16_MODE);
    }
    BSL432_PI_IRQ_TxDataCnt += size;
    BSL432_PI_sendByteSPI(MAP_CRC32_getResult(CRC16_MODE) & 0xFF);  // BSL package CRC
    BSL432_PI_sendByteSPI((MAP_CRC32_getResult(CRC16_MODE) >> 8) & 0xFF);
    BSL432_IRQ_state = IRQ_sending;
    MAP_SPI_disableInterrupt(BSL432_SPI_MODULE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
    MAP_SPI_registerInterrupt(BSL432_SPI_MODULE, BSL432_PI_IRQ_SPI_TX);
    MAP_SPI_enableInterrupt(BSL432_SPI_MODULE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
    MAP_Interrupt_enableMaster();  // Enable sending after buffer is prepared
    while(BSL432_IRQ_state != IRQ_idle);  // Wait for previous TX to finish
}

/**
 * Handles the receiving and sending of bytes via I2C.
 *
 * \return BSL432_eventFlags
 */
BSL432_eventFlags BSL432_PI_receivePacketI2C()
{
    BSL432_PI_eventFlags = Flags_rxPacketOngoing;

    uint32_t dataPointer = 0;
    uint32_t checksum = 0;

    while(BSL432_IRQ_state == IRQ_receiving)
    {
        // Header byte of BSL data packet
        if(dataPointer == 0 && BSL432_PI_IRQ_RxDataCnt > 0)
        {
            if(BSL432_ReceiveBuffer[0] != 0x80)  // first byte in packet must be 0x80
            {
                BSL432_PI_sendByteImmediatelyI2C(BSL432_HEADER_INCORRECT);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_sending;
            }
            else
            {
                dataPointer++;
            }
        }
        // Length (low) byte of BSL package
        else if (dataPointer == 1 && BSL432_PI_IRQ_RxDataCnt > 1)
        {
            BSL432_ReceiveBufferSize = BSL432_ReceiveBuffer[1];
            dataPointer++;
        }
        // Length (high) byte of BSL package
        else if (dataPointer == 2 && BSL432_PI_IRQ_RxDataCnt > 2)
        {
            BSL432_ReceiveBufferSize |= (uint32_t)BSL432_ReceiveBuffer[2] << 8;
            if(BSL432_ReceiveBufferSize == 0)
            {
                BSL432_PI_sendByteImmediatelyI2C(BSL432_PACKET_SIZE_ZERO);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_sending;
            }
            if(BSL432_ReceiveBufferSize > BSL432_MAX_BUFFER_SIZE)  // For future devices that might need smaller packets
            {
                BSL432_PI_sendByteImmediatelyI2C(BSL432_PACKET_SIZE_TOO_BIG);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_sending;
            }
            dataPointer = BSL432_ReceiveBufferSize + 3;
        }
        // CRC (low) byte of BSL package
        else if(dataPointer == (BSL432_ReceiveBufferSize + 3) && BSL432_PI_IRQ_RxDataCnt > (BSL432_ReceiveBufferSize + 3))
        {
            checksum = BSL432_ReceiveBuffer[BSL432_ReceiveBufferSize + 3];
            dataPointer++;
        }
        // CRC (high) byte of BSL package
        else if(dataPointer == (BSL432_ReceiveBufferSize + 4) && BSL432_PI_IRQ_RxDataCnt > (BSL432_ReceiveBufferSize + 4))
        {
            checksum = checksum | BSL432_ReceiveBuffer[BSL432_ReceiveBufferSize + 4]<<8;
            if(BSL432_PI_verifyData(checksum))
            {
                MAP_Interrupt_disableMaster();  // Disable interrupt until TX buffer is prepared
                BSL432_PI_sendByteImmediatelyI2C(BSL432_ACK);
                BSL432_PI_eventFlags = Flags_data_received;
                BSL432_IRQ_state = IRQ_readytosend;
            }
            else
            {
                BSL432_PI_sendByteImmediatelyI2C(BSL432_CHECKSUM_INCORRECT);
                BSL432_PI_eventFlags = Flags_rxErrorRecoverable;
                BSL432_IRQ_state = IRQ_sending;
            }
        }
    }
    return BSL432_PI_eventFlags;
}

/**
 * Places a single byte to be sent into the transmit buffer
 *
 * \param data The byte to be transmitted.
 *
 * \return None
 */
void BSL432_PI_sendByteI2C(uint8_t data)
{
    BSL432_RAM_TX_Buf[BSL432_PI_IRQ_TxDataCnt++] = data;
}

/**
 * Sends a single byte via I2C peripheral directly without using the send buffer
 *
 * \param data The byte to be transmitted.
 *
 * \return None
 */
void BSL432_PI_sendByteImmediatelyI2C(uint8_t data)
{
    BSL432_RAM_TX_Buf[BSL432_MAX_BUFFER_SIZE + 5] = data;  // Place the BSL core response at the end of TX buffer
}

/**
 * Sends data in the data buffer via I2C
 *
 * This function sends the data in the global BSL432_SendBuffer.
 *
 * \param size Number of bytes to send.
 *
 * \return None
 */
void BSL432_PI_sendDataI2C(uint32_t size)
{
    BSL432_PI_sendByteI2C(0x80);  // BSL package header
    BSL432_PI_sendByteI2C(size & 0xFF);  // BSL package size
    BSL432_PI_sendByteI2C((size >> 8) & 0xFF);
    MAP_CRC32_setSeed(0xFFFF, CRC16_MODE);  // Initialize CRC
    uint32_t bufferIndex = 0;
    for(bufferIndex = 3; bufferIndex < (size + 3); bufferIndex++)
    {
        MAP_CRC32_set8BitDataReversed(BSL432_RAM_TX_Buf[bufferIndex], CRC16_MODE);
    }
    BSL432_PI_IRQ_TxDataCnt += size;
    BSL432_PI_sendByteI2C(MAP_CRC32_getResult(CRC16_MODE) & 0xFF);  // BSL package CRC
    BSL432_PI_sendByteI2C((MAP_CRC32_getResult(CRC16_MODE) >> 8) & 0xFF);
    BSL432_IRQ_state = IRQ_sending;
    MAP_Interrupt_enableMaster();  // Enable sending after buffer is prepared
    while(BSL432_IRQ_state != IRQ_idle);  // Wait for previous TX to finish
}


// IRQ handler //////////////////////////////////////////////////////////////

/**
 * SPI interrupt routine used for detection of SPI activity.
 * After receiving the first byte the regular RX/TX SPI IRQ routine is registered.
 *
 * \return None
 */
void BSL432_PI_IRQ_SPIdetect(void)
{
    // Interrupt status flag doesn't need to be read or reset. Reading RX buffer clears the flag.

    MAP_SPI_registerInterrupt(BSL432_SPI_MODULE, BSL432_PI_IRQ_SPI_RX);
    BSL432_RAM_RX_Buf[BSL432_MAX_BUFFER_SIZE + 5] = BSL432_AP_SPI;  // Signal that SPI is PI active now

    uint8_t receivedData = MAP_SPI_receiveData(BSL432_SPI_MODULE);

    if(receivedData != 0xFF) // 0xFF is ignored. Just shifts data out
    {
        BSL432_RAM_RX_Buf[0] = receivedData;
        BSL432_PI_IRQ_RxDataCnt = 1;
        BSL432_IRQ_state = IRQ_receiving;
    }
}

/**
 * SPI interrupt routine for RX operation.
 * Places received bytes in the buffer.
 *
 * \return None
 */
void BSL432_PI_IRQ_SPI_RX(void)
{
    // Interrupt status flag doesn't need to be read or reset. Reading RX buffer clears the flag.

    uint8_t receivedData = MAP_SPI_receiveData(BSL432_SPI_MODULE);
    if(BSL432_IRQ_state == IRQ_idle)
    {
        BSL432_PI_IRQ_RxDataCnt = 1;
        if(receivedData != 0xFF) // 0xFF is ignored. Just shifts data out
        {
            BSL432_RAM_RX_Buf[0] = receivedData;
            BSL432_IRQ_state = IRQ_receiving;
        }
    }
    else if(BSL432_IRQ_state == IRQ_receiving)
    {
        if(BSL432_PI_IRQ_RxDataCnt < (BSL432_MAX_BUFFER_SIZE + 4))
        {
            BSL432_RAM_RX_Buf[BSL432_PI_IRQ_RxDataCnt++] = receivedData;
        }
    }
}

/**
 * SPI interrupt routine for TX operation.
 * Sends from the buffer.
 *
 * \return None
 */
void BSL432_PI_IRQ_SPI_TX(void)
{
    // Interrupt status flag doesn't need to be read or reset. Writing TX buffer clears the flag.
    // No check for 'sending' state needed, always in send mode when this IRQ routine is registered.

    if(BSL432_PI_IRQ_TxDataCnt > BSL432_PI_IRQ_TxDataSent)
    {
        MAP_SPI_transmitData(BSL432_SPI_MODULE, BSL432_RAM_TX_Buf[BSL432_PI_IRQ_TxDataSent++]);
    }
    else
    {
        // Return to 'receive' state after all data in buffer was sent.
        MAP_SPI_transmitData(BSL432_SPI_MODULE, 0x00);
        BSL432_PI_IRQ_TxDataCnt = 0;
        BSL432_PI_IRQ_TxDataSent = 0;
        MAP_SPI_disableInterrupt(BSL432_SPI_MODULE, EUSCI_A_SPI_TRANSMIT_INTERRUPT);
        MAP_SPI_registerInterrupt(BSL432_SPI_MODULE, BSL432_PI_IRQ_SPI_RX);
        MAP_SPI_enableInterrupt(BSL432_SPI_MODULE, EUSCI_A_SPI_RECEIVE_INTERRUPT);
        BSL432_IRQ_state = IRQ_idle;
    }
}

/**
 * UART interrupt routine to fill the RX buffer with received bytes.
 *
 * \return None
 */
void BSL432_PI_IRQ_UART(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(BSL432_UART_MODULE);

    MAP_UART_clearInterruptFlag(BSL432_UART_MODULE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t receivedData = MAP_UART_receiveData(BSL432_UART_MODULE);
        if(BSL432_IRQ_state == IRQ_idle)
        {
            BSL432_PI_IRQ_RxDataCnt = 1;
            BSL432_ReceiveBuffer[0] = receivedData;
            BSL432_IRQ_state = IRQ_receiving;
        }
        else if(BSL432_IRQ_state == IRQ_receiving)
        {
            if(BSL432_PI_IRQ_RxDataCnt < (BSL432_MAX_BUFFER_SIZE + 4))
            {
                BSL432_ReceiveBuffer[BSL432_PI_IRQ_RxDataCnt++] = receivedData;
            }
        }
    }
}

/**
 * I2C interrupt routine used for detection of I2C activity.
 * After receiving the first byte the regular RX/TX I2C IRQ routine is registered.
 *
 * \return None
 */
void BSL432_PI_IRQ_I2Cdetect(void)
{
    uint_fast16_t status = MAP_I2C_getEnabledInterruptStatus(BSL432_I2C_MODULE);
    MAP_I2C_clearInterruptFlag(BSL432_I2C_MODULE, status);

    MAP_SPI_registerInterrupt(BSL432_I2C_MODULE, BSL432_PI_IRQ_I2C);
    BSL432_RAM_RX_Buf[BSL432_MAX_BUFFER_SIZE + 5] = BSL432_AP_I2C;  // Signal that I2C is PI active now

    if(status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        uint8_t receivedData = MAP_I2C_slaveGetData(BSL432_I2C_MODULE);
        BSL432_RAM_RX_Buf[0] = receivedData;
        BSL432_PI_IRQ_RxDataCnt = 1;
        BSL432_IRQ_state = IRQ_receiving;
    }
}

/**
 * I2C interrupt routine for RX/TX operation.
 * Places received bytes in the buffer, sends from the buffer.
 *
 * \return None
 */
void BSL432_PI_IRQ_I2C(void)
{
    uint_fast16_t status = MAP_I2C_getEnabledInterruptStatus(BSL432_I2C_MODULE);
    MAP_I2C_clearInterruptFlag(BSL432_I2C_MODULE, status);

    if(status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
    {
        uint8_t receivedData = MAP_I2C_slaveGetData(BSL432_I2C_MODULE);
        if(BSL432_IRQ_state == IRQ_idle)
        {
            BSL432_PI_IRQ_RxDataCnt = 1;
            BSL432_RAM_RX_Buf[0] = receivedData;
            BSL432_IRQ_state = IRQ_receiving;
        }
        else if(BSL432_IRQ_state == IRQ_receiving)
        {
            if(BSL432_PI_IRQ_RxDataCnt < (BSL432_MAX_BUFFER_SIZE + 4))
            {
                BSL432_RAM_RX_Buf[BSL432_PI_IRQ_RxDataCnt++] = receivedData;
            }
        }
    }
    if(status & EUSCI_B_I2C_TRANSMIT_INTERRUPT0)
    {
        if (BSL432_IRQ_state == IRQ_sending)
        {
            if(BSL432_RAM_TX_Buf[BSL432_MAX_BUFFER_SIZE + 5] != 0xFF)
            {
                MAP_I2C_slavePutData(BSL432_I2C_MODULE, BSL432_RAM_TX_Buf[BSL432_MAX_BUFFER_SIZE + 5]);
                BSL432_RAM_TX_Buf[BSL432_MAX_BUFFER_SIZE + 5] = 0xFF;
            }
            else if(BSL432_PI_IRQ_TxDataCnt > BSL432_PI_IRQ_TxDataSent)
            {
                MAP_I2C_slavePutData(BSL432_I2C_MODULE, BSL432_RAM_TX_Buf[BSL432_PI_IRQ_TxDataSent++]);
            }
            else
            {
                BSL432_PI_IRQ_TxDataCnt = 0;
                BSL432_PI_IRQ_TxDataSent = 0;
                BSL432_IRQ_state = IRQ_idle;
            }
        }
    }
}

/*-------------------------------------------------------------------------------------------
*main.c
*Parker McDonnell
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*--------------------------------------------------------------------------------------------
*Code Reuse: http://www.ti.com/lit/ug/slau622i/slau622i.pdf?ts=1587869186885(Bootloader SW invocation procedure inside main())
*
**-----------------------------------------------------------------------------------------*/


//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

//Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "msp.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//Project Libraries
#include "application.h"    //

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------
#define BSL_PARAM 0xFC48FFFF // I2C slave address = 0x48, Interface selection = Auto
// #define BSL_API_TABLE_ADDR 0x00202000 // Address of BSL API table
#define BSL_ENTRY_FUNCTION (*((uint32_t *)BSL_API_TABLE_ADDR))
//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------
bool fota_flag = 0;
/////////////////////////////////////////////////////////////////////////////////////////////
//void main(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   contains bootloader entry code and main program loop.
//
//
//@param:   void
//@return:  void
//@outcome: program continuously runs.
/////////////////////////////////////////////////////////////////////////////////////////////

void main(void)
{
    MAP_FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,FLASH_SECTOR31);


    __disable_irq();

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;         //stop that dog watching us*/

    /* Firmware update related code */

    if ((*(uint32_t *)0x0003F000 & 0xFF)== 0)
    {

        if(!MAP_FlashCtl_eraseSector(0x0003F000))
            while(1);

        MAP_FlashCtl_protectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,FLASH_SECTOR31);

        __disable_interrupt();


        // Setup interrupt priorities into 0x00 before entering bootloader
        int i;
        for (i=0; i < 240; i++)
        {
            NVIC->IP[i] = 0;
        }
        NVIC->ICER[0] = 0xFFFF;
        NVIC->ICPR[0] = 0xFFFF;
        NVIC->ICER[1] = 0xFFFF;
        NVIC->ICPR[1] = 0xFFFF;
        // Call the BSL with given BSL parameters
        ((void (*)())BSL_ENTRY_FUNCTION)((uint32_t)BSL_PARAM);
        while(1);

    }

    /* Firmware update related code */

    system_init();                                      //initialize system peripherals and drivers

    __enable_irq();                                     // Enable global interrupt

    //SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;                // Enable sleep on exit from ISR

    //__DSB();                                            // Ensures SLEEPONEXIT takes effect immediately

    while(1)                                            //run forever
    {

        /* Firmware update related code */

        if (fota_flag == 1)
        {
            fota_flag = 0;
            MAP_FlashCtl_unprotectSector(FLASH_MAIN_MEMORY_SPACE_BANK1,FLASH_SECTOR31);
            if(!MAP_FlashCtl_programMemory(0,
                    (void*) 0x0003F000, 1))
                        while(1);

        }

        if ((*(uint32_t *)0x0003F000 & 0xFF)== 0)
        {

            __disable_interrupt();
            while(1); // In fota mode.Code hangs here.Power cycle/reset the board & run the script

        }

        /* Firmware update related code */

        __sleep();                                      //put processor to sleep outside of interrupts


    }

}

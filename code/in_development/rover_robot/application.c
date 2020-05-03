/*-------------------------------------------------------------------------------------------
*template.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*--------------------------------------------------------------------------------------------
*Code Referenced:  https://gist.github.com/alexjaw/3be818372e150a13ec5904d14361b12e
*
**-----------------------------------------------------------------------------------------*/


//-------------------------------------------------------------------------------------------
//                                        INCLUDES
//-------------------------------------------------------------------------------------------

//Project Libraries
#include "application.h"

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

bool navigate_auto = 0;
char timestamp_holder[15];
bool mavlink_flag = 0; // Disable mavlink by default

/*-------------------------------------------------------------------------------------------
//                               COMMAND FUNCTION PROTOTYPES
//-----------------------------------------------------------------------------------------*/

void CmdVersion(void){printf("Version %s, %s, %s.\n\r", __PROGRAM_VERSION__,__DATE__,__TIME__);}
void CmdBlinkLed(void){P2->OUT |= BIT2; delay_sec(1); P2->OUT &= ~BIT2;}
void CmdUltrasonicRead(void){printf("Distance = %.2lf inches\n\r",ultrasonic_sample(INCHES));}
void CmdTemperature(void){ printf("msp432 temp = %.2f *F\n\r",temp_read('F')); }
void CmdLight(void) { if(apds9960_color_DataReady()) { printf("ambient light = %d\n\r",apds9960_get_light()); }}
void CmdColor(void) { uint16_t red, green, blue, clear; if(apds9960_color_DataReady())
                      apds9960_get_ColorAll(&red, &green, &blue, &clear);
                      printf("color sensors (RGB) = red %d green %d blue %d\n\r",red,green,blue);}
void CmdAutoNavigate(void) { navigate_auto ^= 1; if(navigate_auto) { printf("rover autonomous control\n\r"); } else
                             { printf("rover manual control\n\r"); } }
void CmdClear(void) { TERMINAL_CLEAR; TERMINAL_MOVE_CURSOR_HOME; }
void CmdStartMav(void) { mavlink_flag = 1;}
void CmdStopMav(void) { mavlink_flag = 0;}  // default mavlink_flag value 0
void CmdTimestamp(void) {  get_timestamp(timestamp_holder); printf("%s\n\r", timestamp_holder);}
void CmdMillis(void) {  printf("%u\n\r", get_millis());}
void CmdFota(void){ fota_flag = 1; }

//place additional command functions here

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

/* Command Table Entries */

const commandStruct_t command[] =
{
    {"version", &CmdVersion, "Display firmware version"},
    {"blinkled", &CmdBlinkLed, "Blinks blue led"},
    {"ultrasonic_read", &CmdUltrasonicRead, "Read sensor range"},
    {"temp_read", &CmdTemperature, "read temperature of processor"},
    {"light_read", &CmdLight, "read ambient light level"},
    {"color_read", &CmdColor, "read RGB light levels"},
    {"auto_nav", &CmdAutoNavigate, "rover navigates autonomously"},
    {"clear", &CmdClear, "clears terminal"},
    {"timestamp", &CmdTimestamp, "Get current timestamp"},
    {"millis", &CmdMillis, "Get current timestamp(millis)"},
    {"start_mav", &CmdStartMav, "Start mavlink telemetry"},
    {"stop_mav", &CmdStopMav, "Stop mavlink telemetry"},
    /*insert additional command table entries here*/
    {"help", &CmdHelp, "Help menu."},
    {"fota_begin", &CmdFota, "Firmware update mode"},
    {"",0,""} //End of table indicator. MUST BE LAST!!!
};

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------


void gpio_init(void);
void systick_init(void);

//-------------------------------------------------------------------------------------------
//                                 FUNCTION DECLARATIONS
//-------------------------------------------------------------------------------------------

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
    /*place code here that you want run at startup*/

    clock_init();
    temp_init();
    gpio_init();
    serial_init();
    timer_init(timer0,no_interrupts);
    motor_init();
#ifdef _ULTRASONIC_
    ultrasonic_init();
#endif //_ULTRASONIC
    I2C1_init();
    misc_init();
    systick_init();
#ifdef _APDS9960_
    apds9960_init();
#endif //_APDS9960
    timestamp_init();
    //navigate_init();
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void gpio_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Configures all GPIO to lowest power stat (output low), takes IMU out of reset
//          enables the 5V converter, configures red, green and blue LED pins for output.
//
//@param:   void
//@return:  void
//@outcome: all non-driver GPIO configured.
/////////////////////////////////////////////////////////////////////////////////////////////

void gpio_init(void)
{
    // GPIO Port Configuration for lowest power configuration
    P1->OUT  = 0x00; P1->DIR  = 0xFF;
    P2->OUT  = 0x00; P2->DIR  = 0xFF;
    P3->OUT  = 0x00; P3->DIR  = 0xFF;
    P4->OUT  = 0x00; P4->DIR  = 0xFF;
    P5->OUT  = 0x00; P5->DIR  = 0xFF;
    P6->OUT  = 0x00; P6->DIR  = 0xFF;
    P7->OUT  = 0x00; P7->DIR  = 0xFF;
    P8->OUT  = 0x00; P8->DIR  = 0xFF;
    P9->OUT  = 0x00; P9->DIR  = 0xFF;
    P10->OUT = 0x00; P10->DIR = 0xFF;
    PJ->OUT  = 0x00; PJ->DIR  = 0xFF;

    //Take IMU out of Reset
    P6->DIR |= IMU_RESET;
    P6->OUT |= IMU_RESET;

    //Enable 5V Buck/Boost (Radio Power)
    _5V_ENABLE;                                             //Set P4.6 high

    // Configure RED LED Pin for output
    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;

    // Configure Green, Blue and Red LED Pin for output
    P2->DIR |= (BIT0 | BIT1 | BIT2);
    P2->OUT &= ~(BIT0 | BIT1 | BIT2);

}

/////////////////////////////////////////////////////////////////////////////////////////////
//void systick_init(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   configure the SysTick clock to overflow at 100 Hz.
//
//
//@param:   void
//@return:  void
//@outcome: SysTick_Handler() called at 100 Hz.
/////////////////////////////////////////////////////////////////////////////////////////////

void systick_init(void)
{
    // Enable SysTick Module
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk |
                     SysTick_CTRL_ENABLE_Msk;
    // Set SysTick period = 0x20000
    SysTick->LOAD =  SYSTICK_COUNT_VAL - 1;
    // Clear the SysTick current value register by writing
    // a dummy value
    SysTick->VAL = 0x01;
    // Enable SysTick interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

/*-------------------------------------------------------------------------------------------
//                                  COMMAND HANDLER START
//-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////////
//void CmdGet(void) -- NON-BLOCKING
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Handles user input received over serial port.  Reads characters from RX Buffer
//          and executes requested tasks.
//
//@param:   void
//@return:  void
//@outcome: user requested action is executed
/////////////////////////////////////////////////////////////////////////////////////////////

void CmdGet(void)
{
    static bool first_time = true;
    static char cmd[MAX_CMD_LENGTH] = "";                                           //command storage
    static uint8_t i = 0;                                               //command array index
    char _c = 0;                                                        //user character

    if(first_time)                                                      //if program is running for first time
    {
        TERMINAL_CLEAR;                                                 //clear serial terminal
        TERMINAL_MOVE_CURSOR_HOME;
        printf("ROVER V1.0: > ");                                       //print terminal entry marker
        first_time = false;
    }

    if(serial_rx_buffer_empty())                                        //if there is no data, exit.
    {
        return;                                                         //exit
    }

    _c = serial_read_byte();                                            //read a byte from serial buffer

    if(_c == ESC)                                                       //if arrow key detected
    {
        _c = serial_read_byte();                                        //get rid of [
        _c = serial_read_byte();                                        //get the arrow key

        switch(_c)                                                      //figure out which arrow key was pressed
        {
            case 'A': { motor_set_speed_desired(high); motor_set_direction_desired(forward); break;}      //drive forward
            case 'B': { motor_set_speed_desired(high); motor_set_direction_desired(reverse); break;}      //drive back
            case 'C': { motor_set_speed_desired(high); motor_set_direction_desired(right);   break;}      //drive right
            case 'D': { motor_set_speed_desired(high); motor_set_direction_desired(left);    break;}      //drive left
        }
    }
    else if(_c == SP)                                                   //if space bar was pressed
    {
        motor_set_speed_desired(zero);
        motor_set_direction_desired(stop);
    }
    else if(_c == CR)                                                   //if enter was pressed
    {
        if(i == 0)                                                      //if enter pressed, but no new command was entered
        {
            CmdHandler(cmd);                                            //try to simply execute the last command again
            printf("ROVER V1.0: > ");                                   //command was invalid, prompt user
        }
        else
        {
            *(cmd + i) = '\0';                                          //terminate string in null character
            i = 0;                                                      //set index back to zero
            printf("\n\r");
            CmdHandler(cmd);                                            //execute command
            printf("ROVER V1.0: > ");                                   //user input prompt
        }
    }
    else if(_c == BS)                                                   //if backspace was pressed
    {
        if(i > 0)
        {
            i--;                                                        //decrement index by one
        }
        serial_write_byte(_c,transmit);                                 //transmit the erasure
    }
    else
    {
        serial_write_byte(_c,transmit);                                 //echo back to user
        *(cmd + i) = _c;                                                //simply write the byte
        i = (i + 1) % MAX_CMD_LENGTH;                                   //increment index by one
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
//bool CmdHandler(char * cmd)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   compares command string against command list and executes the appropriate function
//          through function pointer.
//
//@param:   char *cmd -- pointer to command string.
//@return:  bool -- true if no command found.
//@outcome: command executed, or error message reported.
/////////////////////////////////////////////////////////////////////////////////////////////

bool CmdHandler(char * cmd)
{
    uint8_t num = 0;

    while(command[num].run > 0)
    {
        if(!strcmp(command[num].name, cmd))
        {
            printf("%s: > ",command[num].name);
            (*command[num].run)();

            return 0;
        }
        num++;
    }

    printf("\n\rERROR:  Command not recognized\n\r");

    return 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//void CmdHelp(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   prints out all command strings and their descriptions.
//
//
//@param:   void
//@return:  void
//@outcome: list of commands and their descriptions displayed
/////////////////////////////////////////////////////////////////////////////////////////////

void CmdHelp(void)
{
    uint8_t num = 0;

    printf("\n\r\n\r");

    while(command[num].run > 0)
    {
        printf("%-25s --  %-25s\n\r",command[num].name,command[num].help);
        num++;
    }

    printf("\n\r");
}

/*-------------------------------------------------------------------------------------------
//                                COMMAND HANDLER END
//-----------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////////////////////////////
//void SysTick_Handler(void)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   timed interrupt that occurs at defined PROGRAM_FREQ.  Default is 100 Hz.
//          CPU sleeps in between interrupts.
//
//@param:   void
//@return:  void
//@outcome: CPU wakes up, executes all necessary instructions and goes to sleep.
/////////////////////////////////////////////////////////////////////////////////////////////

void SysTick_Handler(void)
{

    static volatile uint8_t loop_counter1 = 0, loop_counter2 = 0;

/*---------------------------------------100 Hz Loop--------------------------------------------*/

    /*Place code here that you want to run at 100Hz */

    if(!mavlink_flag)
    {
        CmdGet();                                                     //get command input from user
    }

/*----------------------------------------10 Hz loop------------------------------------------*/

    if(loop_counter1 == TEN_LOOPS)
    {
        /*Place code here that you want to run at 10Hz */

        loop_counter1 = 0;

        if(!serial_tx_buffer_empty())                                 //if there is data to transmit
        {
            serial_transmit_buffer();                                 //flush the buffer
        }

        if(navigate_auto)
        {
            navigate_execute();
        }
        else
        {
            motor_controller();
            if(ultrasonic_object_near() && motor_get_direction() == forward)
            {
                motor_set_direction_desired(stop);
            }
        }


        clock_counter++;

#ifdef _IMU_
        if(mavlink_flag)
        {
            if (!populate_IMU_packet()) send_IMU_packet();
        }
#endif //_IMU_
    }
/*--------------------------------------10 Hz loop end------------------------------------------*/


/*----------------------------------------1 Hz loop---------------------------------------------*/

    if(loop_counter2 == ONE_HUNDRED_LOOPS)
    {

        /*Place code here that you want to run at 1Hz */
#ifdef _APDS9960_
        apds9960_IsDark();
#endif //_APDS9960_

        RED_LED_TOGGLE;                                // Toggle little Red LED, let's us know program is running

        if(mavlink_flag)
        {
            send_heartbeat(); // Send mavlink heartbeat packet every 1 second.
        }

        loop_counter2 = 0;
    }

/*--------------------------------------1 Hz loop end-------------------------------------------*/

/* Loop Counters */

    loop_counter1++;                                    //10 Hz loop counter
    loop_counter2++;                                    //1 Hz loop counter

#ifdef _ULTRASONIC_
        ultrasonic_sample(INCHES);
#endif //_ULTRASONIC

/*-------------------------------------100 Hz Loop end------------------------------------------*/
}

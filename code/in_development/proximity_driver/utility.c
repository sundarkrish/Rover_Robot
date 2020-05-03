
//-------------------------------------------------------------------------------------------
//                                       INCLUDES
//-------------------------------------------------------------------------------------------

#include "utility.h"

//-------------------------------------------------------------------------------------------
//                                        DEFINES
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//                                  EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//                              PRIVATE FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
//                                  FUNCTION DEFINITIONS
//-------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////
//long htoi(char *hexstr)
/////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   converts a string of hex digits into decimal
//
//
//@param:   char *hexstr -- pointer to string
//@return:  decimal equivalent
//@outcome: hex string converted to decimal
////////////////////////////////////////////////////////////////////////////////////////////

long htoi(uint8_t *hexstr)
{
    uint32_t result = 0;                                                        //decimal result
    uint8_t byte;                                                              //current byte we are converting

    //loop through string while(*hexstr)
    while(*hexstr)                                                          //while we haven't hit a null character
    {
        byte = *hexstr++;                                                   //get first character in hexstr, increment pointer

        if(byte >= '0' && byte <= '9') { byte = byte - '0';}                //if 0-9 entered, subtract '0' to get decimal eqv.
        else if(byte >= 'a' && byte <= 'f')  { byte = byte - 'a' + 10;}     //if a-f entered, subtract 'a' and add 10 to get decimal
        else if(byte >= 'A' && byte <= 'F')  { byte = byte - 'A' + 10;}     //if A-F entered, subtract 'A' and add 10 to get decimal
        else
        {
            printf("\n\r8051:    > VALUE ENTERED NOT IN HEX!\n\r");         //if character entered wasn't a hex value

            return -1;                                                      //return -1 aka error
        }

        //create decimal equivalent
        result = (result << NIBBLE) | (byte & LOW_NIBBLE);                  //create decimal equivalent
                                                                            //bit shift result by four to make room
    }                                                                       //write in new nibble
                                                                            //or these two together and write into result
    return result;
}

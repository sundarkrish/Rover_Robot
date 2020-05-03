/*---------------------------------------------------------------------------
*timestamp.c
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reuse: 
*
**--------------------------------------------------------------------------*/

#include <stdint.h>
#include <timestamp.h>

uint32_t clock_counter;



// Creates a delay of 3s

void timestamp_init()
{

    clock_counter = 0;

}


uint32_t get_millis()
{

	return clock_counter;

}

// Send a 15 byte character array as argument while calling.
// Else may lead to memory violations
void get_timestamp(char * timestamp)
{
	// Values are stored in ASCII equivalent decimal
	timestamp[0] = '[';
	timestamp[1] = ((clock_counter / 36000) / 10) + 48; // Hours tens place
	timestamp[2] = ((clock_counter / 36000) % 10) + 48; // Hours units place
	timestamp[3] = '.';
	timestamp[4] = (((clock_counter % 36000) / 600) / 10) + 48; // Minutes tens place
	timestamp[5] = (((clock_counter % 36000) / 600) % 10) + 48; // Minutes units place
	timestamp[6] = '.';
	timestamp[7] = ((((clock_counter % 36000) % 600) / 10) / 10) + 48; // Seconds tens place
	timestamp[8] = ((((clock_counter % 36000) % 600) / 10) % 10) + 48; // Seconds units place
	timestamp[9] = '.';
	timestamp[10] = (((clock_counter % 36000) % 600) % 10) + 48; // One tenth of a second
	timestamp[11] = ']';
	timestamp[12] = ':';
	timestamp[13] = '\0';


}





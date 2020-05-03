/*
 * circ_buff.h
 *
 *  Created on: Mar 8, 2020
 *      Author: Parker McDonnell
 */

#ifndef CIRC_BUFF_H_
#define CIRC_BUFF_H_

#include "msp.h"
#include "stdlib.h"
#include "stdbool.h"


//-------------------------------------------------------------------------------------------
//                                              DEFINES
//-------------------------------------------------------------------------------------------

#ifndef _MAX_BUFF_SIZE_
#define MAX_BUFF_SIZE 256         //size of circular buffer
#endif

#ifndef _NULL_
#define NULL 0
#endif

//-------------------------------------------------------------------------------------------
//                                          EXTERNAL VARIABLES
//-------------------------------------------------------------------------------------------


typedef struct
{
    uint8_t *buff;
    uint16_t head;
    uint16_t tail;
    uint16_t size;
    bool  overflow;
}   circ_buff_t;

//-------------------------------------------------------------------------------------------
//                                          FUNCTION PROTOTYPES
//-------------------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////////////////
//int circ_buff_put(circ_buff_t *ptr, uint8_t data)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Writes a byte to a circular buffer, checks if buffer is full before writing.
//
//
//@param:   circ_buff_t *ptr    -- pointer to circular buffer struct
//@param:   uint8_t data        -- data to be written into buffer
//@return:  0 if write successful, -1 if failure
//@outcome: if success a byte is written and the head is moved 1.  Else nothing is written.
//
////////////////////////////////////////////////////////////////////////////////////////////

int circ_buff_put(circ_buff_t *, uint8_t);

////////////////////////////////////////////////////////////////////////////////////////////
//int circ_buff_get(circ_buff_t *ptr, uint8_t *data)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Gets a byte from the circular buffer, and handles incrementing tail.
//          checks if buffer is empty.  Returns -1 if nothing exists in buffer.
//
//@param:   circ_buff_t *ptr -- pointer to circular buffer struct
//@param:   uint8_t *data    -- pointer location data will be written too
//@return:  1 if success, -1 if failure
//@outcome: a byte gets read from the buffer, or a -1 for failure is returned.
//
////////////////////////////////////////////////////////////////////////////////////////////

int circ_buff_get(circ_buff_t *, uint8_t *);

////////////////////////////////////////////////////////////////////////////////////////////
//bool circ_buff_full(circ_buff_t *ptr)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Determines if the circular buffer is full.  Returns 1 if full, 0 if not.
//
//
//@param:   circ_buff_t *ptr -- pointer to circular buffer struct
//@return:  bool -- true or false based on fullness
//@outcome: returns whether buffer is full or not.
//
////////////////////////////////////////////////////////////////////////////////////////////

bool circ_buff_full(circ_buff_t *);

////////////////////////////////////////////////////////////////////////////////////////////
//bool circ_buff_empty(circ_buff_t *ptr)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Checks if circular buffer is empty.  Returns true if yes, false if not.
//
//
//@param:   circ_buff_t *ptr    -- pointer to circ_buff_t struct
//@return:  bool -- true if buffer is empty, false if buffer has data
//@outcome: returns whether buffer is empty or not.
//
////////////////////////////////////////////////////////////////////////////////////////////

bool circ_buff_empty(circ_buff_t *);

////////////////////////////////////////////////////////////////////////////////////////////
//bool circ_buff_init(circ_buff_t *ptr,uint16_t size)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   Initializes a circular buffer structure, creates a buffer and points struct to
//          the buffer.  Returns whether malloc was successful or not
//
//
//@param:   circ_buff_t *ptr -- pointer to circular buffer struct
//@param:   uint116_t size  -- desired size of buffer
//@return:  true if successful, false if failure
//@outcome: circular buffer initialized
//
////////////////////////////////////////////////////////////////////////////////////////////

bool circ_buff_init(circ_buff_t *,uint16_t);

////////////////////////////////////////////////////////////////////////////////////////////
//int circ_buff_size(circ_buff_t *ptr)
////////////////////////////////////////////////////////////////////////////////////////////
//@brief:   calculates number of bytes present in circular buffer
//
//
//
//@param:   circ_buff_t *ptr -- pointer to circular buffer struct
//@return:  size in bytes
//@outcome: returns amount of memory used
//
////////////////////////////////////////////////////////////////////////////////////////////


int circ_buff_size(circ_buff_t *ptr);

#endif /* CIRC_BUFF_H_ */

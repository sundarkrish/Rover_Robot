/*---------------------------------------------------------------------------------------
*circ_buffer.c
*Parker McDonnell
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
**---------------------------------------------------------------------------------------
*@brief:        functions for creating and managing circular buffers.
*
*
*Code Use:      Leveraged some ideas from online journal on circular buffers:
*               https://embedjournal.com/implementing-circular-buffer-embedded-c/
*--------------------------------------------------------------------------------------*/
#include "circ_buff.h"

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


int circ_buff_put(circ_buff_t *ptr, uint8_t data)
{
    uint16_t head_next;     //location of next data write

    head_next = (ptr->head + 1) % MAX_BUFF_SIZE;        //the location or next write

    if(!circ_buff_full(ptr))            //check if buffer is full
    {
            ptr->buff[ptr->head] = data;
            ptr->head = head_next;                          //write data into the buffer
            ptr->overflow = false;
            return false;                                   //return 0 indicating success
    }
    else                //buffer is full.
    {
        ptr->overflow = true;
        return -1;       //buffer if full, return write failed
    }
}

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

int circ_buff_get(circ_buff_t *ptr, uint8_t *data)
{
    uint16_t tail_next;


    if(!circ_buff_empty(ptr))
    {
        tail_next = (ptr->tail + 1) % MAX_BUFF_SIZE;    //location of next read
        *data = ptr->buff[ptr->tail];                   //get the data from the buffer
        ptr->tail = tail_next;                          //set the tail to the next read location
        return true;                                    //return success
    }
    else
    {

        return 0;
    }

}

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

bool circ_buff_full(circ_buff_t *ptr)
{
    if(ptr->head + 1 == ptr->tail)      //if the next write will hit the tail
    {
        ptr->overflow = true;
        return true;                    //return buffer is full
    }

    ptr->overflow = false;

    return false;                       //otherwise the buffer is not full
}

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

bool circ_buff_empty(circ_buff_t *ptr)
{
    if(ptr->head == ptr->tail)
    {
        return true;       //buffer is empty
    }

    return false;           //buffer is not empty

}

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

bool circ_buff_init(circ_buff_t *ptr,uint16_t size)
{
    ptr->head = 0;              //set head to zero
    ptr->tail = 0;              //set tail to zero
    ptr->size = size;           //set buffer
    ptr->used = 0;              //set used bytes
    ptr->free = (size - 1);      //
    ptr->buff = NULL;           //set buffer pointer to null
    ptr->overflow = false;

    //now create a buffer
    ptr->buff = (uint8_t *)malloc(size); //create a buffer of specified size and point to it

    if(ptr->buff == NULL)   //if malloc() failed
    {
        return false;           //return failure
    }

    return true;               //return success
}

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


int circ_buff_used(circ_buff_t *ptr)
{
    if(!circ_buff_empty(ptr))                                               //if the buffer has data
    {
        if(ptr->head > ptr->tail)                                           //check if head is above tail
        {
            ptr->used = (ptr->head - ptr->tail);
            return ptr->used;                                              //simply take the difference between head and tail
        }
        else
        {
            ptr->used = (ptr->size + ptr->head - ptr->tail);
            return ptr->used;                                              //size is difference between head and tail + size.
        }
    }

    ptr->used = ptr->size + 1;                                             //buffer is empty, bytes = size + 1

    return ptr->used;

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

int circ_buff_space(circ_buff_t *ptr)
{
    return (ptr->size - 1 - circ_buff_used(ptr) );
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

void circ_buff_free(circ_buff_t *ptr)
{
    free(ptr);
}

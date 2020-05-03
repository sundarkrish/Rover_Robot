/*---------------------------------------------------------------------------
*mavlink_tx.c
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reuse:
*
**--------------------------------------------------------------------------*/

//---------------------------------------------------------------------------
//                              INCLUDES
//---------------------------------------------------------------------------

//libraries
#include "stdint.h"
#include "mavlink.h"
#include "serial.h"
#include "clock.h"

//Custom Libraries

//---------------------------------------------------------------------------
//                              DEFINES
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//                          EXTERNAL VARIABLES
//---------------------------------------------------------------------------

heartbeat_typedef heartbeat;
imu_report_typedef imu_report;
uint8_t frame[MAX_PACKET_LENGTH];
uint8_t seqno = 0;

void send_heartbeat(void);
void append_crc(uint8_t seed_type);

int main(void)
{

    clock_init();
    serial_init();
    //serial_interrupt_init();
    int j;

    // Populating heartbeat packet
    heartbeat.custom_mode = 0; // Not required .make zero
    heartbeat.type = MAV_TYPE_GROUND_ROVER;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = 64; // manual mode,active
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.mavlink_version = 3; // See mavlink_msg_heartbeat.h library of MAVlink v2/v1 for version#

    //



    while(1)
    {

        send_heartbeat();
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};

        for (j = 0; j < 60000; j++){};

        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};

        for (j = 0; j < 60000; j++){};for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};

        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};

        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};
        for (j = 0; j < 60000; j++){};


    }

    return 0;
}


// Stores the accumulated crc for the frame excluding the start byte and including the extra crc seed(@ frame[length + 6]) in
// frame[length] and frame[length + 1]

void append_crc(uint8_t seed_type)
{
    // Should start from 0xFFFF according to CRC-16 X.25 algorithm
    // Refer : http://docs.ros.org/jade/api/mavlink/html/include__v1_80_2checksum_8h_source.html
    uint16_t accum_crc = 0xFFFF;
    uint8_t tmp;
    uint8_t length = frame[1] + 6; // Includes the extra crc seed.stored at frame[length + 6]
    uint8_t i = 1;

    if (seed_type == 0)
    {

        frame[length] = MAVLINK_MSG_ID_HEARTBEAT_CRC;

    }
    else if (seed_type == 1)
    {

        frame[length] = MAVLINK_MSG_ID_SCALED_IMU_CRC;

    }

    while (i <= length)
    {

        tmp = frame[i] ^ (uint8_t)(accum_crc & 0xFF);
        tmp ^= (tmp<<4);
        accum_crc = (accum_crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
        i++;

    }

    frame[length] = accum_crc & 0xFF; // CRC Low byte
    frame[length + 1] = accum_crc >> 8; // CRC High byte



}

void send_heartbeat(void)
{

    frame[0] = 0xFE; // Start byte (magic number) for MAVlink version1
    frame[1] = MAVLINK_MSG_ID_HEARTBEAT_LEN;
    frame[2] = seqno++;
    frame[3] = 81; // System ID - specific for qcg don't change
    frame[4] = 50; // - specific for qcg don't change
    frame[5] = MAVLINK_MSG_ID_HEARTBEAT;
    frame[6] = 0; // heartbeat custom mode field
    frame[7] = 0;
    frame[8] = 0;
    frame[9] = 0;
    frame[10] = heartbeat.type;
    frame[11] = heartbeat.autopilot;
    frame[12] = heartbeat.base_mode;
    frame[13] = heartbeat.system_status;
    frame[14] = heartbeat.mavlink_version;
    append_crc(0); //  seed type 0 selected extra crc byte for heartbeat

    // Heartbeat frame ready
    // Send the heartbeat frame over serial
    uint8_t pos = 0;


    while (pos < (frame[1] + 8))
    {

        putchar(frame[pos]);
        //serial_transmit_buffer();
        pos++;
    }




}
/////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////
//@brief:
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////

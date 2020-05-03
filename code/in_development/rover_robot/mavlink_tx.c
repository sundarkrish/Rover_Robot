/*---------------------------------------------------------------------------
*mavlink_tx.c
*Sundar Krishnakumar
*ECEN 5613 -- Spring 2020 - Prof. McClure
*University of Colorado Boulder
*
*---------------------------------------------------------------------------
*Code Reuse: https://github.com/mavlink/c_library_v1(checksum calculation X.25 CRC-16)
*
**--------------------------------------------------------------------------*/

//---------------------------------------------------------------------------
//                              INCLUDES
//---------------------------------------------------------------------------

//libraries

#include "mavlink_tx.h"

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
char timestamp[15] = {0};

accel_data_typedef accel_data;
mag_data_typedef mag_data;
gyro_data_typedef gyro_data;
int8_t temperature_data;
int8_t status_accel;
int8_t status_gyro;

/////////////////////////////////////////////////////////////////////////////
//bool misc_init(void)
/////////////////////////////////////////////////////////////////////////////
//@brief: Inits the IMU module and populates the heartbeat packet.Must call this function before sending mavlink heartbeat packet.
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////
bool misc_init(void)
{

    bool error_status = 0;
    // Perform POST test.Init the IMU module


#ifdef _IMU_
    error_status = FXOS8700_POST_test();
    error_status = FXAS21002C_POST_test();
    error_status = IMU_init();
#endif //_IMU_


    // Populating heartbeat packet.Done only once.
    heartbeat.custom_mode = 0; // Not required .make zero
    heartbeat.type = MAV_TYPE_GROUND_ROVER;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = 64; // manual mode,active.See common.h
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.mavlink_version = 3; // See mavlink_msg_heartbeat.h library of MAVlink v2/v1 for version#

    return error_status; // 0 no error ; 1 error

}




/////////////////////////////////////////////////////////////////////////////
//void function(void)
/////////////////////////////////////////////////////////////////////////////
//@brief: Stores the accumulated crc for the frame excluding the start byte and including the extra crc seed(@ frame[length + 6]) in
//        frame[length] and frame[length + 1]
//
//
//@param:
//@return:
//@outcome:
/////////////////////////////////////////////////////////////////////////////

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
    else if (seed_type == 2)
    {

        frame[length] = MAVLINK_MSG_ID_COMMAND_ACK_CRC;

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


        serial_write_byte(frame[pos],no_transmit);

        pos++;
    }

    serial_transmit_buffer();


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
void send_command_ack(void)
{

    frame[0] = 0xFE; // Start byte (magic number) for MAVlink version1
    frame[1] = MAVLINK_MSG_ID_77_MIN_LEN;
    frame[2] = seqno++;
    frame[3] = 81; // System ID - specific for qcg don't change
    frame[4] = 50; // - specific for qcg don't change
    frame[5] = MAVLINK_MSG_ID_COMMAND_ACK;
    frame[6] = 128; // found by looking at qgc command_long packet
    frame[7] = 63; // command not supported
    frame[8] = 3;


    append_crc(2); //  seed type 0 selected extra crc byte for heartbeat

    // Heartbeat frame ready
    // Send the heartbeat frame over serial
    uint8_t pos = 0;


    while (pos < (frame[1] + 8))
    {


        serial_write_byte(frame[pos],no_transmit);
        pos++;
    }

    serial_transmit_buffer();


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
bool populate_IMU_packet(void)
{



    if (IMU_accelmag_read(&status_accel, &accel_data, &mag_data))
    {
        //printf("Error occurred while reading accelerometer data\n\r");
        //serial_transmit_buffer();
        return 1; // 1 -error ; 0 - no error
    }



    //printf("Accelerometer Status : %d\n\r", status_accel);
    //serial_transmit_buffer();
    //printf("[milliG] : accelX-axis : %d accelY-axis : %d accelZ-axis : %d\n\r", accel_data.x, accel_data.y, accel_data.z);
    //serial_transmit_buffer();
    //printf("[milligauss] : magX-axis : %d magY-axis : %d magZ-axis : %d\n\r", mag_data.x, mag_data.y, mag_data.z);
    //serial_transmit_buffer();


    if (IMU_gyrotemp_read(&status_gyro, &gyro_data, &temperature_data))
    {
        //printf("Error occurred while reading gyroscope data\n\r");
        //serial_transmit_buffer();
        return 1; // 1 -error ; 0 - no error
    }


    //printf("Gyroscope Status : %d\n\r", status_gyro);
    //serial_transmit_buffer();
    //printf("[mrad/s] : gyroX-axis : %d gyroY-axis : %d gyroZ-axis : %d\n\r", gyro_data.x, gyro_data.y, gyro_data.z);
    //serial_transmit_buffer();
    //printf("[degreeC] : Temperature : %d\n\r", temperature_data);
    //serial_transmit_buffer();


    // Populating IMU packet.Using scaled IMU packet structure (ID : 26) of mavlink
    imu_report.time_boot_ms = get_millis(); /*< [ms] Timestamp (time since system boot).Overflows to 0 at arounf 4 billion-uint32*/
    imu_report.xacc = accel_data.x; /*< [mG] X acceleration*/
    imu_report.yacc = accel_data.y; /*< [mG] Y acceleration*/
    imu_report.zacc = accel_data.z; /*< [mG] Z acceleration*/
    imu_report.xgyro = gyro_data.x; /*< [mrad/s] Angular speed around X axis*/
    imu_report.ygyro = gyro_data.y; /*< [mrad/s] Angular speed around Y axis*/
    imu_report.zgyro = gyro_data.z; /*< [mrad/s] Angular speed around Z axis*/
    imu_report.xmag = mag_data.x; /*< [mgauss] X Magnetic field*/
    imu_report.ymag = mag_data.y; /*< [mgauss] Y Magnetic field*/
    imu_report.zmag =  mag_data.z; /*< [mgauss] Z Magnetic field*/
    imu_report.temperature = (uint16_t)temperature_data; /*< [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).*/

    return 0; // no error

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
void send_IMU_packet(void)
{

/*    if (populate_IMU_packet()) // If error i.e = 1,do nothing return.Else transmit IMU packet
    {

        return 0;
    }*/

    frame[0] = 0xFE; // Start byte (magic number) for MAVlink version1
    frame[1] = MAVLINK_MSG_ID_SCALED_IMU_LEN;
    frame[2] = seqno++;
    frame[3] = 81; // System ID - specific for qcg don't change
    frame[4] = 50; // - specific for qcg don't change
    frame[5] = MAVLINK_MSG_ID_SCALED_IMU;
    frame[6] = imu_report.time_boot_ms & 0x000000FF; // payload starts here - 24bytes
    frame[7] = (imu_report.time_boot_ms & 0x0000FF00) >> 8;
    frame[8] = (imu_report.time_boot_ms & 0x00FF0000) >> 16;
    frame[9] = (imu_report.time_boot_ms & 0xFF000000) >> 24;
    frame[10] = imu_report.xacc & 0x00FF;
    frame[11] = (imu_report.xacc & 0xFF00) >> 8;
    frame[12] = imu_report.yacc & 0x00FF;
    frame[13] = (imu_report.yacc & 0xFF00) >> 8;
    frame[14] = imu_report.zacc & 0x00FF;
    frame[15] = (imu_report.zacc & 0xFF00) >> 8;
    frame[16] = imu_report.xgyro & 0x00FF;
    frame[17] = (imu_report.xgyro & 0xFF00) >> 8;
    frame[18] = imu_report.ygyro & 0x00FF;
    frame[19] = (imu_report.ygyro & 0xFF00) >> 8;
    frame[20] = imu_report.zgyro & 0x00FF;
    frame[21] = (imu_report.zgyro & 0xFF00) >> 8;
    frame[22] = imu_report.xmag & 0x00FF;
    frame[23] = (imu_report.xmag & 0xFF00) >> 8;
    frame[24] = imu_report.ymag & 0x00FF;
    frame[25] = (imu_report.ymag & 0xFF00) >> 8;
    frame[26] = imu_report.zmag & 0x00FF;
    frame[27] = (imu_report.zmag & 0xFF00) >> 8;
    frame[28] = imu_report.temperature & 0x00FF;
    frame[29] = (imu_report.temperature & 0xFF00) >> 8;

    append_crc(1); //  seed type 0 selected extra crc byte for heartbeat

    // IMU frame ready
    // Send the IMU frame over serial
    uint8_t pos = 0;


    while (pos < (frame[1] + 8))
    {


        serial_write_byte(frame[pos],no_transmit);
        pos++;
    }

    serial_transmit_buffer();



}


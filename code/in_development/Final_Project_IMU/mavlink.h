/** @file
 *  @brief MAVLink comm protocol built from common.xml
 *  @see http://mavlink.org
 *  Note : Mavlink v1 followed for simplicity.Mavlink v2 packet structure elements are complicated and
 *         some fields do not have proper MACROS to assign.
 */
#pragma once
#ifndef MAVLINK_H
#define MAVLINK_H

#define MAX_PACKET_LENGTH 263 //Using mavlink v1 packet format for simplicity

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_SCALED_IMU 26

#define MAVLINK_MSG_ID_HEARTBEAT_CRC 50
#define MAVLINK_MSG_ID_0_CRC 50
#define MAVLINK_MSG_ID_SCALED_IMU_CRC 170
#define MAVLINK_MSG_ID_26_CRC 170

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_SCALED_IMU_LEN 24

#define MAV_TYPE_GROUND_ROVER 10 /* Ground rover | */
#define MAV_AUTOPILOT_INVALID 8 /* No valid autopilot, e.g. a GCS or other MAVLink component | */
#define MAV_AUTOPILOT_PX4 12 /* PX4 Autopilot - http://px4.io/ | */


#define MAV_STATE_UNINIT 0 /* Uninitialized system, state is unknown. | */
#define MAV_STATE_BOOT 1 /* System is booting up. | */
#define MAV_STATE_CALIBRATING 2 /* System is calibrating and not flight-ready. | */
#define MAV_STATE_STANDBY 3 /* System is grounded and on standby. It can be launched any time. | */
#define MAV_STATE_ACTIVE 4/* System is active and might be already airborne. Motors are engaged. | */
#define MAV_STATE_CRITICAL 5 /* System is in a non-normal flight mode. It can however still navigate. | */
#define MAV_STATE_EMERGENCY 6 /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
#define MAV_STATE_POWEROFF 7 /* System just initialized its power-down sequence, will shut down now. | */
#define MAV_STATE_FLIGHT_TERMINATION 8 /* System is terminating itself. | */
#define MAV_STATE_ENUM_END 9 /*  | */


#define MAV_COMP_ID_LOG 155 /* Logging component. | */

typedef struct
{

    uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
    uint8_t type; /*<  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
    uint8_t autopilot; /*<  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
    uint8_t base_mode; /*<  System mode bitmap.*/
    uint8_t system_status; /*<  System status flag.*/
    uint8_t mavlink_version; /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/


} heartbeat_typedef;


typedef struct
{
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    int16_t xacc; /*< [mG] X acceleration*/
    int16_t yacc; /*< [mG] Y acceleration*/
    int16_t zacc; /*< [mG] Z acceleration*/
    int16_t xgyro; /*< [mrad/s] Angular speed around X axis*/
    int16_t ygyro; /*< [mrad/s] Angular speed around Y axis*/
    int16_t zgyro; /*< [mrad/s] Angular speed around Z axis*/
    int16_t xmag; /*< [mgauss] X Magnetic field*/
    int16_t ymag; /*< [mgauss] Y Magnetic field*/
    int16_t zmag; /*< [mgauss] Z Magnetic field*/
    int16_t temperature; /*< [cdegC] Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).*/

} imu_report_typedef;




#endif // MAVLINK_H

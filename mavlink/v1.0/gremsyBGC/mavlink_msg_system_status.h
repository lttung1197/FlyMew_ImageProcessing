// MESSAGE SYSTEM_STATUS PACKING

#define MAVLINK_MSG_ID_SYSTEM_STATUS 155

typedef struct __mavlink_system_status_t
{
 float battery_voltage; ///< current battery voltage
 uint8_t imu_calib; ///< imu calib status
 uint8_t sat_numbers; ///< number of sattelites if use GPS
 uint8_t status1; ///< additional status 1
 uint8_t status2; ///< additional status 2
} mavlink_system_status_t;

#define MAVLINK_MSG_ID_SYSTEM_STATUS_LEN 8
#define MAVLINK_MSG_ID_155_LEN 8



#define MAVLINK_MESSAGE_INFO_SYSTEM_STATUS { \
	"SYSTEM_STATUS", \
	5, \
	{  { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_system_status_t, battery_voltage) }, \
         { "imu_calib", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_system_status_t, imu_calib) }, \
         { "sat_numbers", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_system_status_t, sat_numbers) }, \
         { "status1", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_system_status_t, status1) }, \
         { "status2", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_system_status_t, status2) }, \
         } \
}


/**
 * @brief Pack a system_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param battery_voltage current battery voltage
 * @param imu_calib imu calib status
 * @param sat_numbers number of sattelites if use GPS
 * @param status1 additional status 1
 * @param status2 additional status 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_system_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float battery_voltage, uint8_t imu_calib, uint8_t sat_numbers, uint8_t status1, uint8_t status2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_float(buf, 0, battery_voltage);
	_mav_put_uint8_t(buf, 4, imu_calib);
	_mav_put_uint8_t(buf, 5, sat_numbers);
	_mav_put_uint8_t(buf, 6, status1);
	_mav_put_uint8_t(buf, 7, status2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 8);
#else
	mavlink_system_status_t packet;
	packet.battery_voltage = battery_voltage;
	packet.imu_calib = imu_calib;
	packet.sat_numbers = sat_numbers;
	packet.status1 = status1;
	packet.status2 = status2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_SYSTEM_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 8, 76);
}

/**
 * @brief Pack a system_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_voltage current battery voltage
 * @param imu_calib imu calib status
 * @param sat_numbers number of sattelites if use GPS
 * @param status1 additional status 1
 * @param status2 additional status 2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_system_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float battery_voltage,uint8_t imu_calib,uint8_t sat_numbers,uint8_t status1,uint8_t status2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_float(buf, 0, battery_voltage);
	_mav_put_uint8_t(buf, 4, imu_calib);
	_mav_put_uint8_t(buf, 5, sat_numbers);
	_mav_put_uint8_t(buf, 6, status1);
	_mav_put_uint8_t(buf, 7, status2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 8);
#else
	mavlink_system_status_t packet;
	packet.battery_voltage = battery_voltage;
	packet.imu_calib = imu_calib;
	packet.sat_numbers = sat_numbers;
	packet.status1 = status1;
	packet.status2 = status2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 8);
#endif

	msg->msgid = MAVLINK_MSG_ID_SYSTEM_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 8, 76);
}

/**
 * @brief Encode a system_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_status C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_system_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_status_t* system_status)
{
	return mavlink_msg_system_status_pack(system_id, component_id, msg, system_status->battery_voltage, system_status->imu_calib, system_status->sat_numbers, system_status->status1, system_status->status2);
}

/**
 * @brief Send a system_status message
 * @param chan MAVLink channel to send the message
 *
 * @param battery_voltage current battery voltage
 * @param imu_calib imu calib status
 * @param sat_numbers number of sattelites if use GPS
 * @param status1 additional status 1
 * @param status2 additional status 2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_system_status_send(mavlink_channel_t chan, float battery_voltage, uint8_t imu_calib, uint8_t sat_numbers, uint8_t status1, uint8_t status2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[8];
	_mav_put_float(buf, 0, battery_voltage);
	_mav_put_uint8_t(buf, 4, imu_calib);
	_mav_put_uint8_t(buf, 5, sat_numbers);
	_mav_put_uint8_t(buf, 6, status1);
	_mav_put_uint8_t(buf, 7, status2);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_STATUS, buf, 8, 76);
#else
	mavlink_system_status_t packet;
	packet.battery_voltage = battery_voltage;
	packet.imu_calib = imu_calib;
	packet.sat_numbers = sat_numbers;
	packet.status1 = status1;
	packet.status2 = status2;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_STATUS, (const char *)&packet, 8, 76);
#endif
}

#endif

// MESSAGE SYSTEM_STATUS UNPACKING


/**
 * @brief Get field battery_voltage from system_status message
 *
 * @return current battery voltage
 */
static __inline float mavlink_msg_system_status_get_battery_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field imu_calib from system_status message
 *
 * @return imu calib status
 */
static __inline uint8_t mavlink_msg_system_status_get_imu_calib(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field sat_numbers from system_status message
 *
 * @return number of sattelites if use GPS
 */
static __inline uint8_t mavlink_msg_system_status_get_sat_numbers(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field status1 from system_status message
 *
 * @return additional status 1
 */
static __inline uint8_t mavlink_msg_system_status_get_status1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field status2 from system_status message
 *
 * @return additional status 2
 */
static __inline uint8_t mavlink_msg_system_status_get_status2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Decode a system_status message into a struct
 *
 * @param msg The message to decode
 * @param system_status C-struct to decode the message contents into
 */
static __inline void mavlink_msg_system_status_decode(const mavlink_message_t* msg, mavlink_system_status_t* system_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	system_status->battery_voltage = mavlink_msg_system_status_get_battery_voltage(msg);
	system_status->imu_calib = mavlink_msg_system_status_get_imu_calib(msg);
	system_status->sat_numbers = mavlink_msg_system_status_get_sat_numbers(msg);
	system_status->status1 = mavlink_msg_system_status_get_status1(msg);
	system_status->status2 = mavlink_msg_system_status_get_status2(msg);
#else
	memcpy(system_status, _MAV_PAYLOAD(msg), 8);
#endif
}

// MESSAGE IMU_CALIB_REQUEST PACKING

#define MAVLINK_MSG_ID_IMU_CALIB_REQUEST 154

typedef struct __mavlink_imu_calib_request_t
{
 uint8_t calib_type; ///< imu calib type Gyro or Accelerometer
 uint8_t acc_calib_mode; ///< this argument is optional, only used when calib type is ACC
} mavlink_imu_calib_request_t;

#define MAVLINK_MSG_ID_IMU_CALIB_REQUEST_LEN 2
#define MAVLINK_MSG_ID_154_LEN 2



#define MAVLINK_MESSAGE_INFO_IMU_CALIB_REQUEST { \
	"IMU_CALIB_REQUEST", \
	2, \
	{  { "calib_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_imu_calib_request_t, calib_type) }, \
         { "acc_calib_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_imu_calib_request_t, acc_calib_mode) }, \
         } \
}


/**
 * @brief Pack a imu_calib_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param calib_type imu calib type Gyro or Accelerometer
 * @param acc_calib_mode this argument is optional, only used when calib type is ACC
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_imu_calib_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t calib_type, uint8_t acc_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, calib_type);
	_mav_put_uint8_t(buf, 1, acc_calib_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_imu_calib_request_t packet;
	packet.calib_type = calib_type;
	packet.acc_calib_mode = acc_calib_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMU_CALIB_REQUEST;
	return mavlink_finalize_message(msg, system_id, component_id, 2, 81);
}

/**
 * @brief Pack a imu_calib_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param calib_type imu calib type Gyro or Accelerometer
 * @param acc_calib_mode this argument is optional, only used when calib type is ACC
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_imu_calib_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t calib_type,uint8_t acc_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, calib_type);
	_mav_put_uint8_t(buf, 1, acc_calib_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_imu_calib_request_t packet;
	packet.calib_type = calib_type;
	packet.acc_calib_mode = acc_calib_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_IMU_CALIB_REQUEST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 81);
}

/**
 * @brief Encode a imu_calib_request struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu_calib_request C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_imu_calib_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_calib_request_t* imu_calib_request)
{
	return mavlink_msg_imu_calib_request_pack(system_id, component_id, msg, imu_calib_request->calib_type, imu_calib_request->acc_calib_mode);
}

/**
 * @brief Send a imu_calib_request message
 * @param chan MAVLink channel to send the message
 *
 * @param calib_type imu calib type Gyro or Accelerometer
 * @param acc_calib_mode this argument is optional, only used when calib type is ACC
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_imu_calib_request_send(mavlink_channel_t chan, uint8_t calib_type, uint8_t acc_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_uint8_t(buf, 0, calib_type);
	_mav_put_uint8_t(buf, 1, acc_calib_mode);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_CALIB_REQUEST, buf, 2, 81);
#else
	mavlink_imu_calib_request_t packet;
	packet.calib_type = calib_type;
	packet.acc_calib_mode = acc_calib_mode;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_CALIB_REQUEST, (const char *)&packet, 2, 81);
#endif
}

#endif

// MESSAGE IMU_CALIB_REQUEST UNPACKING


/**
 * @brief Get field calib_type from imu_calib_request message
 *
 * @return imu calib type Gyro or Accelerometer
 */
static __inline uint8_t mavlink_msg_imu_calib_request_get_calib_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field acc_calib_mode from imu_calib_request message
 *
 * @return this argument is optional, only used when calib type is ACC
 */
static __inline uint8_t mavlink_msg_imu_calib_request_get_acc_calib_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Decode a imu_calib_request message into a struct
 *
 * @param msg The message to decode
 * @param imu_calib_request C-struct to decode the message contents into
 */
static __inline void mavlink_msg_imu_calib_request_decode(const mavlink_message_t* msg, mavlink_imu_calib_request_t* imu_calib_request)
{
#if MAVLINK_NEED_BYTE_SWAP
	imu_calib_request->calib_type = mavlink_msg_imu_calib_request_get_calib_type(msg);
	imu_calib_request->acc_calib_mode = mavlink_msg_imu_calib_request_get_acc_calib_mode(msg);
#else
	memcpy(imu_calib_request, _MAV_PAYLOAD(msg), 2);
#endif
}

// MESSAGE MOTORS_CONTROL PACKING

#define MAVLINK_MSG_ID_MOTORS_CONTROL 161

typedef struct __mavlink_motors_control_t
{
 uint8_t command; ///< command ENABLE or DISABLE motor
} mavlink_motors_control_t;

#define MAVLINK_MSG_ID_MOTORS_CONTROL_LEN 1
#define MAVLINK_MSG_ID_161_LEN 1



#define MAVLINK_MESSAGE_INFO_MOTORS_CONTROL { \
	"MOTORS_CONTROL", \
	1, \
	{  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_motors_control_t, command) }, \
         } \
}


/**
 * @brief Pack a motors_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command command ENABLE or DISABLE motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_motors_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_motors_control_t packet;
	packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTORS_CONTROL;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 148);
}

/**
 * @brief Pack a motors_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param command command ENABLE or DISABLE motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_motors_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, command);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_motors_control_t packet;
	packet.command = command;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_MOTORS_CONTROL;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 148);
}

/**
 * @brief Encode a motors_control struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motors_control C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_motors_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motors_control_t* motors_control)
{
	return mavlink_msg_motors_control_pack(system_id, component_id, msg, motors_control->command);
}

/**
 * @brief Send a motors_control message
 * @param chan MAVLink channel to send the message
 *
 * @param command command ENABLE or DISABLE motor
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_motors_control_send(mavlink_channel_t chan, uint8_t command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, command);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTORS_CONTROL, buf, 1, 148);
#else
	mavlink_motors_control_t packet;
	packet.command = command;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTORS_CONTROL, (const char *)&packet, 1, 148);
#endif
}

#endif

// MESSAGE MOTORS_CONTROL UNPACKING


/**
 * @brief Get field command from motors_control message
 *
 * @return command ENABLE or DISABLE motor
 */
static __inline uint8_t mavlink_msg_motors_control_get_command(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a motors_control message into a struct
 *
 * @param msg The message to decode
 * @param motors_control C-struct to decode the message contents into
 */
static __inline void mavlink_msg_motors_control_decode(const mavlink_message_t* msg, mavlink_motors_control_t* motors_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	motors_control->command = mavlink_msg_motors_control_get_command(msg);
#else
	memcpy(motors_control, _MAV_PAYLOAD(msg), 1);
#endif
}

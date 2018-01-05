// MESSAGE KEYCODE_VALUE PACKING

#define MAVLINK_MSG_ID_KEYCODE_VALUE 160

typedef struct __mavlink_keycode_value_t
{
 uint32_t keycode; ///< keycode of device
 uint8_t device_name4; ///< name of device can be one of following values
} mavlink_keycode_value_t;

#define MAVLINK_MSG_ID_KEYCODE_VALUE_LEN 5
#define MAVLINK_MSG_ID_160_LEN 5



#define MAVLINK_MESSAGE_INFO_KEYCODE_VALUE { \
	"KEYCODE_VALUE", \
	2, \
	{  { "keycode", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_keycode_value_t, keycode) }, \
         { "device_name4", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_keycode_value_t, device_name4) }, \
         } \
}


/**
 * @brief Pack a keycode_value message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param device_name4 name of device can be one of following values
 * @param keycode keycode of device
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_keycode_value_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t device_name4, uint32_t keycode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_uint32_t(buf, 0, keycode);
	_mav_put_uint8_t(buf, 4, device_name4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 5);
#else
	mavlink_keycode_value_t packet;
	packet.keycode = keycode;
	packet.device_name4 = device_name4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 5);
#endif

	msg->msgid = MAVLINK_MSG_ID_KEYCODE_VALUE;
	return mavlink_finalize_message(msg, system_id, component_id, 5, 30);
}

/**
 * @brief Pack a keycode_value message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param device_name4 name of device can be one of following values
 * @param keycode keycode of device
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_keycode_value_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t device_name4,uint32_t keycode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_uint32_t(buf, 0, keycode);
	_mav_put_uint8_t(buf, 4, device_name4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 5);
#else
	mavlink_keycode_value_t packet;
	packet.keycode = keycode;
	packet.device_name4 = device_name4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 5);
#endif

	msg->msgid = MAVLINK_MSG_ID_KEYCODE_VALUE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 5, 30);
}

/**
 * @brief Encode a keycode_value struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param keycode_value C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_keycode_value_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_keycode_value_t* keycode_value)
{
	return mavlink_msg_keycode_value_pack(system_id, component_id, msg, keycode_value->device_name4, keycode_value->keycode);
}

/**
 * @brief Send a keycode_value message
 * @param chan MAVLink channel to send the message
 *
 * @param device_name4 name of device can be one of following values
 * @param keycode keycode of device
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_keycode_value_send(mavlink_channel_t chan, uint8_t device_name4, uint32_t keycode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_uint32_t(buf, 0, keycode);
	_mav_put_uint8_t(buf, 4, device_name4);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KEYCODE_VALUE, buf, 5, 30);
#else
	mavlink_keycode_value_t packet;
	packet.keycode = keycode;
	packet.device_name4 = device_name4;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_KEYCODE_VALUE, (const char *)&packet, 5, 30);
#endif
}

#endif

// MESSAGE KEYCODE_VALUE UNPACKING


/**
 * @brief Get field device_name4 from keycode_value message
 *
 * @return name of device can be one of following values
 */
static __inline uint8_t mavlink_msg_keycode_value_get_device_name4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field keycode from keycode_value message
 *
 * @return keycode of device
 */
static __inline uint32_t mavlink_msg_keycode_value_get_keycode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a keycode_value message into a struct
 *
 * @param msg The message to decode
 * @param keycode_value C-struct to decode the message contents into
 */
static __inline void mavlink_msg_keycode_value_decode(const mavlink_message_t* msg, mavlink_keycode_value_t* keycode_value)
{
#if MAVLINK_NEED_BYTE_SWAP
	keycode_value->keycode = mavlink_msg_keycode_value_get_keycode(msg);
	keycode_value->device_name4 = mavlink_msg_keycode_value_get_device_name4(msg);
#else
	memcpy(keycode_value, _MAV_PAYLOAD(msg), 5);
#endif
}

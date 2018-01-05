// MESSAGE UNIQUE_ID_REQUEST PACKING

#define MAVLINK_MSG_ID_UNIQUE_ID_REQUEST 157

typedef struct __mavlink_unique_id_request_t
{
 uint8_t device_name1; ///< name of device
} mavlink_unique_id_request_t;

#define MAVLINK_MSG_ID_UNIQUE_ID_REQUEST_LEN 1
#define MAVLINK_MSG_ID_157_LEN 1



#define MAVLINK_MESSAGE_INFO_UNIQUE_ID_REQUEST { \
	"UNIQUE_ID_REQUEST", \
	1, \
	{  { "device_name1", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_unique_id_request_t, device_name1) }, \
         } \
}


/**
 * @brief Pack a unique_id_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param device_name1 name of device
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_unique_id_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t device_name1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, device_name1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_unique_id_request_t packet;
	packet.device_name1 = device_name1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_UNIQUE_ID_REQUEST;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 205);
}

/**
 * @brief Pack a unique_id_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param device_name1 name of device
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_unique_id_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t device_name1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, device_name1);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_unique_id_request_t packet;
	packet.device_name1 = device_name1;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_UNIQUE_ID_REQUEST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 205);
}

/**
 * @brief Encode a unique_id_request struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param unique_id_request C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_unique_id_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_unique_id_request_t* unique_id_request)
{
	return mavlink_msg_unique_id_request_pack(system_id, component_id, msg, unique_id_request->device_name1);
}

/**
 * @brief Send a unique_id_request message
 * @param chan MAVLink channel to send the message
 *
 * @param device_name1 name of device
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_unique_id_request_send(mavlink_channel_t chan, uint8_t device_name1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, device_name1);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNIQUE_ID_REQUEST, buf, 1, 205);
#else
	mavlink_unique_id_request_t packet;
	packet.device_name1 = device_name1;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UNIQUE_ID_REQUEST, (const char *)&packet, 1, 205);
#endif
}

#endif

// MESSAGE UNIQUE_ID_REQUEST UNPACKING


/**
 * @brief Get field device_name1 from unique_id_request message
 *
 * @return name of device
 */
static __inline uint8_t mavlink_msg_unique_id_request_get_device_name1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a unique_id_request message into a struct
 *
 * @param msg The message to decode
 * @param unique_id_request C-struct to decode the message contents into
 */
static __inline void mavlink_msg_unique_id_request_decode(const mavlink_message_t* msg, mavlink_unique_id_request_t* unique_id_request)
{
#if MAVLINK_NEED_BYTE_SWAP
	unique_id_request->device_name1 = mavlink_msg_unique_id_request_get_device_name1(msg);
#else
	memcpy(unique_id_request, _MAV_PAYLOAD(msg), 1);
#endif
}

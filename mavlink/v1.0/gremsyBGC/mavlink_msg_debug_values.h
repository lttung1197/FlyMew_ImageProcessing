// MESSAGE DEBUG_VALUES PACKING

#define MAVLINK_MSG_ID_DEBUG_VALUES 156

typedef struct __mavlink_debug_values_t
{
 float debug1; ///< debug value 1
 float debug2; ///< debug value 2
 float debug3; ///< debug value 3
 float debug4; ///< debug value 4
 uint16_t debug5; ///< debug value 5
 uint16_t debug6; ///< debug value 6
 uint16_t debug7; ///< debug value 7
 uint16_t debug8; ///< debug value 8
} mavlink_debug_values_t;

#define MAVLINK_MSG_ID_DEBUG_VALUES_LEN 24
#define MAVLINK_MSG_ID_156_LEN 24



#define MAVLINK_MESSAGE_INFO_DEBUG_VALUES { \
	"DEBUG_VALUES", \
	8, \
	{  { "debug1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_debug_values_t, debug1) }, \
         { "debug2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_debug_values_t, debug2) }, \
         { "debug3", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_debug_values_t, debug3) }, \
         { "debug4", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_debug_values_t, debug4) }, \
         { "debug5", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_debug_values_t, debug5) }, \
         { "debug6", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_debug_values_t, debug6) }, \
         { "debug7", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_debug_values_t, debug7) }, \
         { "debug8", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_debug_values_t, debug8) }, \
         } \
}


/**
 * @brief Pack a debug_values message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param debug1 debug value 1
 * @param debug2 debug value 2
 * @param debug3 debug value 3
 * @param debug4 debug value 4
 * @param debug5 debug value 5
 * @param debug6 debug value 6
 * @param debug7 debug value 7
 * @param debug8 debug value 8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_debug_values_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float debug1, float debug2, float debug3, float debug4, uint16_t debug5, uint16_t debug6, uint16_t debug7, uint16_t debug8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, debug1);
	_mav_put_float(buf, 4, debug2);
	_mav_put_float(buf, 8, debug3);
	_mav_put_float(buf, 12, debug4);
	_mav_put_uint16_t(buf, 16, debug5);
	_mav_put_uint16_t(buf, 18, debug6);
	_mav_put_uint16_t(buf, 20, debug7);
	_mav_put_uint16_t(buf, 22, debug8);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 24);
#else
	mavlink_debug_values_t packet;
	packet.debug1 = debug1;
	packet.debug2 = debug2;
	packet.debug3 = debug3;
	packet.debug4 = debug4;
	packet.debug5 = debug5;
	packet.debug6 = debug6;
	packet.debug7 = debug7;
	packet.debug8 = debug8;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG_VALUES;
	return mavlink_finalize_message(msg, system_id, component_id, 24, 13);
}

/**
 * @brief Pack a debug_values message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param debug1 debug value 1
 * @param debug2 debug value 2
 * @param debug3 debug value 3
 * @param debug4 debug value 4
 * @param debug5 debug value 5
 * @param debug6 debug value 6
 * @param debug7 debug value 7
 * @param debug8 debug value 8
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_debug_values_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float debug1,float debug2,float debug3,float debug4,uint16_t debug5,uint16_t debug6,uint16_t debug7,uint16_t debug8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, debug1);
	_mav_put_float(buf, 4, debug2);
	_mav_put_float(buf, 8, debug3);
	_mav_put_float(buf, 12, debug4);
	_mav_put_uint16_t(buf, 16, debug5);
	_mav_put_uint16_t(buf, 18, debug6);
	_mav_put_uint16_t(buf, 20, debug7);
	_mav_put_uint16_t(buf, 22, debug8);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 24);
#else
	mavlink_debug_values_t packet;
	packet.debug1 = debug1;
	packet.debug2 = debug2;
	packet.debug3 = debug3;
	packet.debug4 = debug4;
	packet.debug5 = debug5;
	packet.debug6 = debug6;
	packet.debug7 = debug7;
	packet.debug8 = debug8;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 24);
#endif

	msg->msgid = MAVLINK_MSG_ID_DEBUG_VALUES;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 24, 13);
}

/**
 * @brief Encode a debug_values struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param debug_values C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_debug_values_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_debug_values_t* debug_values)
{
	return mavlink_msg_debug_values_pack(system_id, component_id, msg, debug_values->debug1, debug_values->debug2, debug_values->debug3, debug_values->debug4, debug_values->debug5, debug_values->debug6, debug_values->debug7, debug_values->debug8);
}

/**
 * @brief Send a debug_values message
 * @param chan MAVLink channel to send the message
 *
 * @param debug1 debug value 1
 * @param debug2 debug value 2
 * @param debug3 debug value 3
 * @param debug4 debug value 4
 * @param debug5 debug value 5
 * @param debug6 debug value 6
 * @param debug7 debug value 7
 * @param debug8 debug value 8
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_debug_values_send(mavlink_channel_t chan, float debug1, float debug2, float debug3, float debug4, uint16_t debug5, uint16_t debug6, uint16_t debug7, uint16_t debug8)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[24];
	_mav_put_float(buf, 0, debug1);
	_mav_put_float(buf, 4, debug2);
	_mav_put_float(buf, 8, debug3);
	_mav_put_float(buf, 12, debug4);
	_mav_put_uint16_t(buf, 16, debug5);
	_mav_put_uint16_t(buf, 18, debug6);
	_mav_put_uint16_t(buf, 20, debug7);
	_mav_put_uint16_t(buf, 22, debug8);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VALUES, buf, 24, 13);
#else
	mavlink_debug_values_t packet;
	packet.debug1 = debug1;
	packet.debug2 = debug2;
	packet.debug3 = debug3;
	packet.debug4 = debug4;
	packet.debug5 = debug5;
	packet.debug6 = debug6;
	packet.debug7 = debug7;
	packet.debug8 = debug8;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEBUG_VALUES, (const char *)&packet, 24, 13);
#endif
}

#endif

// MESSAGE DEBUG_VALUES UNPACKING


/**
 * @brief Get field debug1 from debug_values message
 *
 * @return debug value 1
 */
static __inline float mavlink_msg_debug_values_get_debug1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field debug2 from debug_values message
 *
 * @return debug value 2
 */
static __inline float mavlink_msg_debug_values_get_debug2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field debug3 from debug_values message
 *
 * @return debug value 3
 */
static __inline float mavlink_msg_debug_values_get_debug3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field debug4 from debug_values message
 *
 * @return debug value 4
 */
static __inline float mavlink_msg_debug_values_get_debug4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field debug5 from debug_values message
 *
 * @return debug value 5
 */
static __inline uint16_t mavlink_msg_debug_values_get_debug5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field debug6 from debug_values message
 *
 * @return debug value 6
 */
static __inline uint16_t mavlink_msg_debug_values_get_debug6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field debug7 from debug_values message
 *
 * @return debug value 7
 */
static __inline uint16_t mavlink_msg_debug_values_get_debug7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field debug8 from debug_values message
 *
 * @return debug value 8
 */
static __inline uint16_t mavlink_msg_debug_values_get_debug8(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Decode a debug_values message into a struct
 *
 * @param msg The message to decode
 * @param debug_values C-struct to decode the message contents into
 */
static __inline void mavlink_msg_debug_values_decode(const mavlink_message_t* msg, mavlink_debug_values_t* debug_values)
{
#if MAVLINK_NEED_BYTE_SWAP
	debug_values->debug1 = mavlink_msg_debug_values_get_debug1(msg);
	debug_values->debug2 = mavlink_msg_debug_values_get_debug2(msg);
	debug_values->debug3 = mavlink_msg_debug_values_get_debug3(msg);
	debug_values->debug4 = mavlink_msg_debug_values_get_debug4(msg);
	debug_values->debug5 = mavlink_msg_debug_values_get_debug5(msg);
	debug_values->debug6 = mavlink_msg_debug_values_get_debug6(msg);
	debug_values->debug7 = mavlink_msg_debug_values_get_debug7(msg);
	debug_values->debug8 = mavlink_msg_debug_values_get_debug8(msg);
#else
	memcpy(debug_values, _MAV_PAYLOAD(msg), 24);
#endif
}

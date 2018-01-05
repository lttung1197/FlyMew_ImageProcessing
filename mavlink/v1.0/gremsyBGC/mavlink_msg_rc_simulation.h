// MESSAGE RC_SIMULATION PACKING

#define MAVLINK_MSG_ID_RC_SIMULATION 153

typedef struct __mavlink_rc_simulation_t
{
 int16_t tilt; ///< tilt value in degree
 int16_t roll; ///< roll value in degree
 int16_t pan; ///< pan value in degree
} mavlink_rc_simulation_t;

#define MAVLINK_MSG_ID_RC_SIMULATION_LEN 6
#define MAVLINK_MSG_ID_153_LEN 6



#define MAVLINK_MESSAGE_INFO_RC_SIMULATION { \
	"RC_SIMULATION", \
	3, \
	{  { "tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_rc_simulation_t, tilt) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_rc_simulation_t, roll) }, \
         { "pan", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_rc_simulation_t, pan) }, \
         } \
}


/**
 * @brief Pack a rc_simulation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tilt tilt value in degree
 * @param roll roll value in degree
 * @param pan pan value in degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_rc_simulation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t tilt, int16_t roll, int16_t pan)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, tilt);
	_mav_put_int16_t(buf, 2, roll);
	_mav_put_int16_t(buf, 4, pan);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_rc_simulation_t packet;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pan = pan;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_SIMULATION;
	return mavlink_finalize_message(msg, system_id, component_id, 6, 194);
}

/**
 * @brief Pack a rc_simulation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param tilt tilt value in degree
 * @param roll roll value in degree
 * @param pan pan value in degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_rc_simulation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t tilt,int16_t roll,int16_t pan)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, tilt);
	_mav_put_int16_t(buf, 2, roll);
	_mav_put_int16_t(buf, 4, pan);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_rc_simulation_t packet;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pan = pan;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_RC_SIMULATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 194);
}

/**
 * @brief Encode a rc_simulation struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_simulation C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_rc_simulation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_simulation_t* rc_simulation)
{
	return mavlink_msg_rc_simulation_pack(system_id, component_id, msg, rc_simulation->tilt, rc_simulation->roll, rc_simulation->pan);
}

/**
 * @brief Send a rc_simulation message
 * @param chan MAVLink channel to send the message
 *
 * @param tilt tilt value in degree
 * @param roll roll value in degree
 * @param pan pan value in degree
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_rc_simulation_send(mavlink_channel_t chan, int16_t tilt, int16_t roll, int16_t pan)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, tilt);
	_mav_put_int16_t(buf, 2, roll);
	_mav_put_int16_t(buf, 4, pan);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_SIMULATION, buf, 6, 194);
#else
	mavlink_rc_simulation_t packet;
	packet.tilt = tilt;
	packet.roll = roll;
	packet.pan = pan;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RC_SIMULATION, (const char *)&packet, 6, 194);
#endif
}

#endif

// MESSAGE RC_SIMULATION UNPACKING


/**
 * @brief Get field tilt from rc_simulation message
 *
 * @return tilt value in degree
 */
static __inline int16_t mavlink_msg_rc_simulation_get_tilt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field roll from rc_simulation message
 *
 * @return roll value in degree
 */
static __inline int16_t mavlink_msg_rc_simulation_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field pan from rc_simulation message
 *
 * @return pan value in degree
 */
static __inline int16_t mavlink_msg_rc_simulation_get_pan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Decode a rc_simulation message into a struct
 *
 * @param msg The message to decode
 * @param rc_simulation C-struct to decode the message contents into
 */
static __inline void mavlink_msg_rc_simulation_decode(const mavlink_message_t* msg, mavlink_rc_simulation_t* rc_simulation)
{
#if MAVLINK_NEED_BYTE_SWAP
	rc_simulation->tilt = mavlink_msg_rc_simulation_get_tilt(msg);
	rc_simulation->roll = mavlink_msg_rc_simulation_get_roll(msg);
	rc_simulation->pan = mavlink_msg_rc_simulation_get_pan(msg);
#else
	memcpy(rc_simulation, _MAV_PAYLOAD(msg), 6);
#endif
}

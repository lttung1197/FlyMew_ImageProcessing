// MESSAGE PPM_CHAN_VALUES PACKING

#define MAVLINK_MSG_ID_PPM_CHAN_VALUES 151

typedef struct __mavlink_ppm_chan_values_t
{
 float tilt; ///< ppm tilt value
 float pan; ///< ppm pan value
 float roll; ///< ppm roll value
 float mode; ///< ppm mode value
} mavlink_ppm_chan_values_t;

#define MAVLINK_MSG_ID_PPM_CHAN_VALUES_LEN 16
#define MAVLINK_MSG_ID_151_LEN 16



#define MAVLINK_MESSAGE_INFO_PPM_CHAN_VALUES { \
	"PPM_CHAN_VALUES", \
	4, \
	{  { "tilt", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ppm_chan_values_t, tilt) }, \
         { "pan", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ppm_chan_values_t, pan) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ppm_chan_values_t, roll) }, \
         { "mode", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ppm_chan_values_t, mode) }, \
         } \
}


/**
 * @brief Pack a ppm_chan_values message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tilt ppm tilt value
 * @param pan ppm pan value
 * @param roll ppm roll value
 * @param mode ppm mode value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_ppm_chan_values_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float tilt, float pan, float roll, float mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_float(buf, 0, tilt);
	_mav_put_float(buf, 4, pan);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 16);
#else
	mavlink_ppm_chan_values_t packet;
	packet.tilt = tilt;
	packet.pan = pan;
	packet.roll = roll;
	packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 16);
#endif

	msg->msgid = MAVLINK_MSG_ID_PPM_CHAN_VALUES;
	return mavlink_finalize_message(msg, system_id, component_id, 16, 200);
}

/**
 * @brief Pack a ppm_chan_values message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param tilt ppm tilt value
 * @param pan ppm pan value
 * @param roll ppm roll value
 * @param mode ppm mode value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_ppm_chan_values_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float tilt,float pan,float roll,float mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_float(buf, 0, tilt);
	_mav_put_float(buf, 4, pan);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 16);
#else
	mavlink_ppm_chan_values_t packet;
	packet.tilt = tilt;
	packet.pan = pan;
	packet.roll = roll;
	packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 16);
#endif

	msg->msgid = MAVLINK_MSG_ID_PPM_CHAN_VALUES;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16, 200);
}

/**
 * @brief Encode a ppm_chan_values struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ppm_chan_values C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_ppm_chan_values_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ppm_chan_values_t* ppm_chan_values)
{
	return mavlink_msg_ppm_chan_values_pack(system_id, component_id, msg, ppm_chan_values->tilt, ppm_chan_values->pan, ppm_chan_values->roll, ppm_chan_values->mode);
}

/**
 * @brief Send a ppm_chan_values message
 * @param chan MAVLink channel to send the message
 *
 * @param tilt ppm tilt value
 * @param pan ppm pan value
 * @param roll ppm roll value
 * @param mode ppm mode value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_ppm_chan_values_send(mavlink_channel_t chan, float tilt, float pan, float roll, float mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_float(buf, 0, tilt);
	_mav_put_float(buf, 4, pan);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, mode);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PPM_CHAN_VALUES, buf, 16, 200);
#else
	mavlink_ppm_chan_values_t packet;
	packet.tilt = tilt;
	packet.pan = pan;
	packet.roll = roll;
	packet.mode = mode;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PPM_CHAN_VALUES, (const char *)&packet, 16, 200);
#endif
}

#endif

// MESSAGE PPM_CHAN_VALUES UNPACKING


/**
 * @brief Get field tilt from ppm_chan_values message
 *
 * @return ppm tilt value
 */
static __inline float mavlink_msg_ppm_chan_values_get_tilt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pan from ppm_chan_values message
 *
 * @return ppm pan value
 */
static __inline float mavlink_msg_ppm_chan_values_get_pan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field roll from ppm_chan_values message
 *
 * @return ppm roll value
 */
static __inline float mavlink_msg_ppm_chan_values_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field mode from ppm_chan_values message
 *
 * @return ppm mode value
 */
static __inline float mavlink_msg_ppm_chan_values_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a ppm_chan_values message into a struct
 *
 * @param msg The message to decode
 * @param ppm_chan_values C-struct to decode the message contents into
 */
static __inline void mavlink_msg_ppm_chan_values_decode(const mavlink_message_t* msg, mavlink_ppm_chan_values_t* ppm_chan_values)
{
#if MAVLINK_NEED_BYTE_SWAP
	ppm_chan_values->tilt = mavlink_msg_ppm_chan_values_get_tilt(msg);
	ppm_chan_values->pan = mavlink_msg_ppm_chan_values_get_pan(msg);
	ppm_chan_values->roll = mavlink_msg_ppm_chan_values_get_roll(msg);
	ppm_chan_values->mode = mavlink_msg_ppm_chan_values_get_mode(msg);
#else
	memcpy(ppm_chan_values, _MAV_PAYLOAD(msg), 16);
#endif
}

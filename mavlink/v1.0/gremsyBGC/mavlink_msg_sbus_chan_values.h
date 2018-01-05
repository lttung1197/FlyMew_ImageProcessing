// MESSAGE SBUS_CHAN_VALUES PACKING

#define MAVLINK_MSG_ID_SBUS_CHAN_VALUES 152

typedef struct __mavlink_sbus_chan_values_t
{
 int16_t ch1; ///< channel1 value
 int16_t ch2; ///< channel2 value
 int16_t ch3; ///< channel3 value
 int16_t ch4; ///< channel4 value
 int16_t ch5; ///< channel5 value
 int16_t ch6; ///< channel6 value
 int16_t ch7; ///< channel7 value
 int16_t ch8; ///< channel8 value
 int16_t ch9; ///< channel9 value
 int16_t ch10; ///< channel10 value
 int16_t ch11; ///< channel11 value
 int16_t ch12; ///< channel12 value
 int16_t ch13; ///< channel13 value
 int16_t ch14; ///< channel14 value
 int16_t ch15; ///< channel15 value
 int16_t ch16; ///< channel16 value
 int16_t ch17; ///< channel17 value
 int16_t ch18; ///< channel18 value
} mavlink_sbus_chan_values_t;

#define MAVLINK_MSG_ID_SBUS_CHAN_VALUES_LEN 36
#define MAVLINK_MSG_ID_152_LEN 36



#define MAVLINK_MESSAGE_INFO_SBUS_CHAN_VALUES { \
	"SBUS_CHAN_VALUES", \
	18, \
	{  { "ch1", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_sbus_chan_values_t, ch1) }, \
         { "ch2", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_sbus_chan_values_t, ch2) }, \
         { "ch3", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_sbus_chan_values_t, ch3) }, \
         { "ch4", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_sbus_chan_values_t, ch4) }, \
         { "ch5", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_sbus_chan_values_t, ch5) }, \
         { "ch6", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_sbus_chan_values_t, ch6) }, \
         { "ch7", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_sbus_chan_values_t, ch7) }, \
         { "ch8", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_sbus_chan_values_t, ch8) }, \
         { "ch9", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_sbus_chan_values_t, ch9) }, \
         { "ch10", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_sbus_chan_values_t, ch10) }, \
         { "ch11", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_sbus_chan_values_t, ch11) }, \
         { "ch12", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_sbus_chan_values_t, ch12) }, \
         { "ch13", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_sbus_chan_values_t, ch13) }, \
         { "ch14", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_sbus_chan_values_t, ch14) }, \
         { "ch15", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_sbus_chan_values_t, ch15) }, \
         { "ch16", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_sbus_chan_values_t, ch16) }, \
         { "ch17", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_sbus_chan_values_t, ch17) }, \
         { "ch18", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_sbus_chan_values_t, ch18) }, \
         } \
}


/**
 * @brief Pack a sbus_chan_values message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ch1 channel1 value
 * @param ch2 channel2 value
 * @param ch3 channel3 value
 * @param ch4 channel4 value
 * @param ch5 channel5 value
 * @param ch6 channel6 value
 * @param ch7 channel7 value
 * @param ch8 channel8 value
 * @param ch9 channel9 value
 * @param ch10 channel10 value
 * @param ch11 channel11 value
 * @param ch12 channel12 value
 * @param ch13 channel13 value
 * @param ch14 channel14 value
 * @param ch15 channel15 value
 * @param ch16 channel16 value
 * @param ch17 channel17 value
 * @param ch18 channel18 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_sbus_chan_values_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4, int16_t ch5, int16_t ch6, int16_t ch7, int16_t ch8, int16_t ch9, int16_t ch10, int16_t ch11, int16_t ch12, int16_t ch13, int16_t ch14, int16_t ch15, int16_t ch16, int16_t ch17, int16_t ch18)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_int16_t(buf, 0, ch1);
	_mav_put_int16_t(buf, 2, ch2);
	_mav_put_int16_t(buf, 4, ch3);
	_mav_put_int16_t(buf, 6, ch4);
	_mav_put_int16_t(buf, 8, ch5);
	_mav_put_int16_t(buf, 10, ch6);
	_mav_put_int16_t(buf, 12, ch7);
	_mav_put_int16_t(buf, 14, ch8);
	_mav_put_int16_t(buf, 16, ch9);
	_mav_put_int16_t(buf, 18, ch10);
	_mav_put_int16_t(buf, 20, ch11);
	_mav_put_int16_t(buf, 22, ch12);
	_mav_put_int16_t(buf, 24, ch13);
	_mav_put_int16_t(buf, 26, ch14);
	_mav_put_int16_t(buf, 28, ch15);
	_mav_put_int16_t(buf, 30, ch16);
	_mav_put_int16_t(buf, 32, ch17);
	_mav_put_int16_t(buf, 34, ch18);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_sbus_chan_values_t packet;
	packet.ch1 = ch1;
	packet.ch2 = ch2;
	packet.ch3 = ch3;
	packet.ch4 = ch4;
	packet.ch5 = ch5;
	packet.ch6 = ch6;
	packet.ch7 = ch7;
	packet.ch8 = ch8;
	packet.ch9 = ch9;
	packet.ch10 = ch10;
	packet.ch11 = ch11;
	packet.ch12 = ch12;
	packet.ch13 = ch13;
	packet.ch14 = ch14;
	packet.ch15 = ch15;
	packet.ch16 = ch16;
	packet.ch17 = ch17;
	packet.ch18 = ch18;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_SBUS_CHAN_VALUES;
	return mavlink_finalize_message(msg, system_id, component_id, 36, 134);
}

/**
 * @brief Pack a sbus_chan_values message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param ch1 channel1 value
 * @param ch2 channel2 value
 * @param ch3 channel3 value
 * @param ch4 channel4 value
 * @param ch5 channel5 value
 * @param ch6 channel6 value
 * @param ch7 channel7 value
 * @param ch8 channel8 value
 * @param ch9 channel9 value
 * @param ch10 channel10 value
 * @param ch11 channel11 value
 * @param ch12 channel12 value
 * @param ch13 channel13 value
 * @param ch14 channel14 value
 * @param ch15 channel15 value
 * @param ch16 channel16 value
 * @param ch17 channel17 value
 * @param ch18 channel18 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static __inline uint16_t mavlink_msg_sbus_chan_values_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t ch1,int16_t ch2,int16_t ch3,int16_t ch4,int16_t ch5,int16_t ch6,int16_t ch7,int16_t ch8,int16_t ch9,int16_t ch10,int16_t ch11,int16_t ch12,int16_t ch13,int16_t ch14,int16_t ch15,int16_t ch16,int16_t ch17,int16_t ch18)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_int16_t(buf, 0, ch1);
	_mav_put_int16_t(buf, 2, ch2);
	_mav_put_int16_t(buf, 4, ch3);
	_mav_put_int16_t(buf, 6, ch4);
	_mav_put_int16_t(buf, 8, ch5);
	_mav_put_int16_t(buf, 10, ch6);
	_mav_put_int16_t(buf, 12, ch7);
	_mav_put_int16_t(buf, 14, ch8);
	_mav_put_int16_t(buf, 16, ch9);
	_mav_put_int16_t(buf, 18, ch10);
	_mav_put_int16_t(buf, 20, ch11);
	_mav_put_int16_t(buf, 22, ch12);
	_mav_put_int16_t(buf, 24, ch13);
	_mav_put_int16_t(buf, 26, ch14);
	_mav_put_int16_t(buf, 28, ch15);
	_mav_put_int16_t(buf, 30, ch16);
	_mav_put_int16_t(buf, 32, ch17);
	_mav_put_int16_t(buf, 34, ch18);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 36);
#else
	mavlink_sbus_chan_values_t packet;
	packet.ch1 = ch1;
	packet.ch2 = ch2;
	packet.ch3 = ch3;
	packet.ch4 = ch4;
	packet.ch5 = ch5;
	packet.ch6 = ch6;
	packet.ch7 = ch7;
	packet.ch8 = ch8;
	packet.ch9 = ch9;
	packet.ch10 = ch10;
	packet.ch11 = ch11;
	packet.ch12 = ch12;
	packet.ch13 = ch13;
	packet.ch14 = ch14;
	packet.ch15 = ch15;
	packet.ch16 = ch16;
	packet.ch17 = ch17;
	packet.ch18 = ch18;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 36);
#endif

	msg->msgid = MAVLINK_MSG_ID_SBUS_CHAN_VALUES;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 36, 134);
}

/**
 * @brief Encode a sbus_chan_values struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbus_chan_values C-struct to read the message contents from
 */
static __inline uint16_t mavlink_msg_sbus_chan_values_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbus_chan_values_t* sbus_chan_values)
{
	return mavlink_msg_sbus_chan_values_pack(system_id, component_id, msg, sbus_chan_values->ch1, sbus_chan_values->ch2, sbus_chan_values->ch3, sbus_chan_values->ch4, sbus_chan_values->ch5, sbus_chan_values->ch6, sbus_chan_values->ch7, sbus_chan_values->ch8, sbus_chan_values->ch9, sbus_chan_values->ch10, sbus_chan_values->ch11, sbus_chan_values->ch12, sbus_chan_values->ch13, sbus_chan_values->ch14, sbus_chan_values->ch15, sbus_chan_values->ch16, sbus_chan_values->ch17, sbus_chan_values->ch18);
}

/**
 * @brief Send a sbus_chan_values message
 * @param chan MAVLink channel to send the message
 *
 * @param ch1 channel1 value
 * @param ch2 channel2 value
 * @param ch3 channel3 value
 * @param ch4 channel4 value
 * @param ch5 channel5 value
 * @param ch6 channel6 value
 * @param ch7 channel7 value
 * @param ch8 channel8 value
 * @param ch9 channel9 value
 * @param ch10 channel10 value
 * @param ch11 channel11 value
 * @param ch12 channel12 value
 * @param ch13 channel13 value
 * @param ch14 channel14 value
 * @param ch15 channel15 value
 * @param ch16 channel16 value
 * @param ch17 channel17 value
 * @param ch18 channel18 value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static __inline void mavlink_msg_sbus_chan_values_send(mavlink_channel_t chan, int16_t ch1, int16_t ch2, int16_t ch3, int16_t ch4, int16_t ch5, int16_t ch6, int16_t ch7, int16_t ch8, int16_t ch9, int16_t ch10, int16_t ch11, int16_t ch12, int16_t ch13, int16_t ch14, int16_t ch15, int16_t ch16, int16_t ch17, int16_t ch18)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[36];
	_mav_put_int16_t(buf, 0, ch1);
	_mav_put_int16_t(buf, 2, ch2);
	_mav_put_int16_t(buf, 4, ch3);
	_mav_put_int16_t(buf, 6, ch4);
	_mav_put_int16_t(buf, 8, ch5);
	_mav_put_int16_t(buf, 10, ch6);
	_mav_put_int16_t(buf, 12, ch7);
	_mav_put_int16_t(buf, 14, ch8);
	_mav_put_int16_t(buf, 16, ch9);
	_mav_put_int16_t(buf, 18, ch10);
	_mav_put_int16_t(buf, 20, ch11);
	_mav_put_int16_t(buf, 22, ch12);
	_mav_put_int16_t(buf, 24, ch13);
	_mav_put_int16_t(buf, 26, ch14);
	_mav_put_int16_t(buf, 28, ch15);
	_mav_put_int16_t(buf, 30, ch16);
	_mav_put_int16_t(buf, 32, ch17);
	_mav_put_int16_t(buf, 34, ch18);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBUS_CHAN_VALUES, buf, 36, 134);
#else
	mavlink_sbus_chan_values_t packet;
	packet.ch1 = ch1;
	packet.ch2 = ch2;
	packet.ch3 = ch3;
	packet.ch4 = ch4;
	packet.ch5 = ch5;
	packet.ch6 = ch6;
	packet.ch7 = ch7;
	packet.ch8 = ch8;
	packet.ch9 = ch9;
	packet.ch10 = ch10;
	packet.ch11 = ch11;
	packet.ch12 = ch12;
	packet.ch13 = ch13;
	packet.ch14 = ch14;
	packet.ch15 = ch15;
	packet.ch16 = ch16;
	packet.ch17 = ch17;
	packet.ch18 = ch18;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBUS_CHAN_VALUES, (const char *)&packet, 36, 134);
#endif
}

#endif

// MESSAGE SBUS_CHAN_VALUES UNPACKING


/**
 * @brief Get field ch1 from sbus_chan_values message
 *
 * @return channel1 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field ch2 from sbus_chan_values message
 *
 * @return channel2 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field ch3 from sbus_chan_values message
 *
 * @return channel3 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field ch4 from sbus_chan_values message
 *
 * @return channel4 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field ch5 from sbus_chan_values message
 *
 * @return channel5 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field ch6 from sbus_chan_values message
 *
 * @return channel6 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field ch7 from sbus_chan_values message
 *
 * @return channel7 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field ch8 from sbus_chan_values message
 *
 * @return channel8 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch8(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field ch9 from sbus_chan_values message
 *
 * @return channel9 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch9(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field ch10 from sbus_chan_values message
 *
 * @return channel10 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch10(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field ch11 from sbus_chan_values message
 *
 * @return channel11 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch11(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field ch12 from sbus_chan_values message
 *
 * @return channel12 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch12(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field ch13 from sbus_chan_values message
 *
 * @return channel13 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch13(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field ch14 from sbus_chan_values message
 *
 * @return channel14 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch14(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field ch15 from sbus_chan_values message
 *
 * @return channel15 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch15(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field ch16 from sbus_chan_values message
 *
 * @return channel16 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch16(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field ch17 from sbus_chan_values message
 *
 * @return channel17 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch17(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field ch18 from sbus_chan_values message
 *
 * @return channel18 value
 */
static __inline int16_t mavlink_msg_sbus_chan_values_get_ch18(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Decode a sbus_chan_values message into a struct
 *
 * @param msg The message to decode
 * @param sbus_chan_values C-struct to decode the message contents into
 */
static __inline void mavlink_msg_sbus_chan_values_decode(const mavlink_message_t* msg, mavlink_sbus_chan_values_t* sbus_chan_values)
{
#if MAVLINK_NEED_BYTE_SWAP
	sbus_chan_values->ch1 = mavlink_msg_sbus_chan_values_get_ch1(msg);
	sbus_chan_values->ch2 = mavlink_msg_sbus_chan_values_get_ch2(msg);
	sbus_chan_values->ch3 = mavlink_msg_sbus_chan_values_get_ch3(msg);
	sbus_chan_values->ch4 = mavlink_msg_sbus_chan_values_get_ch4(msg);
	sbus_chan_values->ch5 = mavlink_msg_sbus_chan_values_get_ch5(msg);
	sbus_chan_values->ch6 = mavlink_msg_sbus_chan_values_get_ch6(msg);
	sbus_chan_values->ch7 = mavlink_msg_sbus_chan_values_get_ch7(msg);
	sbus_chan_values->ch8 = mavlink_msg_sbus_chan_values_get_ch8(msg);
	sbus_chan_values->ch9 = mavlink_msg_sbus_chan_values_get_ch9(msg);
	sbus_chan_values->ch10 = mavlink_msg_sbus_chan_values_get_ch10(msg);
	sbus_chan_values->ch11 = mavlink_msg_sbus_chan_values_get_ch11(msg);
	sbus_chan_values->ch12 = mavlink_msg_sbus_chan_values_get_ch12(msg);
	sbus_chan_values->ch13 = mavlink_msg_sbus_chan_values_get_ch13(msg);
	sbus_chan_values->ch14 = mavlink_msg_sbus_chan_values_get_ch14(msg);
	sbus_chan_values->ch15 = mavlink_msg_sbus_chan_values_get_ch15(msg);
	sbus_chan_values->ch16 = mavlink_msg_sbus_chan_values_get_ch16(msg);
	sbus_chan_values->ch17 = mavlink_msg_sbus_chan_values_get_ch17(msg);
	sbus_chan_values->ch18 = mavlink_msg_sbus_chan_values_get_ch18(msg);
#else
	memcpy(sbus_chan_values, _MAV_PAYLOAD(msg), 36);
#endif
}

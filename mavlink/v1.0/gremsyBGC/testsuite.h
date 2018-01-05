/** @file
 *	@brief MAVLink comm protocol testsuite generated from gremsyBGC.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef GREMSYBGC_TESTSUITE_H
#define GREMSYBGC_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_gremsyBGC(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_gremsyBGC(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_ppm_chan_values(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ppm_chan_values_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	};
	mavlink_ppm_chan_values_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.tilt = packet_in.tilt;
        	packet1.pan = packet_in.pan;
        	packet1.roll = packet_in.roll;
        	packet1.mode = packet_in.mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ppm_chan_values_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ppm_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ppm_chan_values_pack(system_id, component_id, &msg , packet1.tilt , packet1.pan , packet1.roll , packet1.mode );
	mavlink_msg_ppm_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ppm_chan_values_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tilt , packet1.pan , packet1.roll , packet1.mode );
	mavlink_msg_ppm_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ppm_chan_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ppm_chan_values_send(MAVLINK_COMM_1 , packet1.tilt , packet1.pan , packet1.roll , packet1.mode );
	mavlink_msg_ppm_chan_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sbus_chan_values(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sbus_chan_values_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	18275,
	18379,
	18483,
	18587,
	18691,
	18795,
	18899,
	19003,
	};
	mavlink_sbus_chan_values_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.ch1 = packet_in.ch1;
        	packet1.ch2 = packet_in.ch2;
        	packet1.ch3 = packet_in.ch3;
        	packet1.ch4 = packet_in.ch4;
        	packet1.ch5 = packet_in.ch5;
        	packet1.ch6 = packet_in.ch6;
        	packet1.ch7 = packet_in.ch7;
        	packet1.ch8 = packet_in.ch8;
        	packet1.ch9 = packet_in.ch9;
        	packet1.ch10 = packet_in.ch10;
        	packet1.ch11 = packet_in.ch11;
        	packet1.ch12 = packet_in.ch12;
        	packet1.ch13 = packet_in.ch13;
        	packet1.ch14 = packet_in.ch14;
        	packet1.ch15 = packet_in.ch15;
        	packet1.ch16 = packet_in.ch16;
        	packet1.ch17 = packet_in.ch17;
        	packet1.ch18 = packet_in.ch18;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sbus_chan_values_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sbus_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sbus_chan_values_pack(system_id, component_id, &msg , packet1.ch1 , packet1.ch2 , packet1.ch3 , packet1.ch4 , packet1.ch5 , packet1.ch6 , packet1.ch7 , packet1.ch8 , packet1.ch9 , packet1.ch10 , packet1.ch11 , packet1.ch12 , packet1.ch13 , packet1.ch14 , packet1.ch15 , packet1.ch16 , packet1.ch17 , packet1.ch18 );
	mavlink_msg_sbus_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sbus_chan_values_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ch1 , packet1.ch2 , packet1.ch3 , packet1.ch4 , packet1.ch5 , packet1.ch6 , packet1.ch7 , packet1.ch8 , packet1.ch9 , packet1.ch10 , packet1.ch11 , packet1.ch12 , packet1.ch13 , packet1.ch14 , packet1.ch15 , packet1.ch16 , packet1.ch17 , packet1.ch18 );
	mavlink_msg_sbus_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sbus_chan_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sbus_chan_values_send(MAVLINK_COMM_1 , packet1.ch1 , packet1.ch2 , packet1.ch3 , packet1.ch4 , packet1.ch5 , packet1.ch6 , packet1.ch7 , packet1.ch8 , packet1.ch9 , packet1.ch10 , packet1.ch11 , packet1.ch12 , packet1.ch13 , packet1.ch14 , packet1.ch15 , packet1.ch16 , packet1.ch17 , packet1.ch18 );
	mavlink_msg_sbus_chan_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rc_simulation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_simulation_t packet_in = {
		17235,
	17339,
	17443,
	};
	mavlink_rc_simulation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.tilt = packet_in.tilt;
        	packet1.roll = packet_in.roll;
        	packet1.pan = packet_in.pan;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_simulation_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rc_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_simulation_pack(system_id, component_id, &msg , packet1.tilt , packet1.roll , packet1.pan );
	mavlink_msg_rc_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_simulation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tilt , packet1.roll , packet1.pan );
	mavlink_msg_rc_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rc_simulation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_simulation_send(MAVLINK_COMM_1 , packet1.tilt , packet1.roll , packet1.pan );
	mavlink_msg_rc_simulation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_imu_calib_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_imu_calib_request_t packet_in = {
		5,
	72,
	};
	mavlink_imu_calib_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.calib_type = packet_in.calib_type;
        	packet1.acc_calib_mode = packet_in.acc_calib_mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_calib_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_imu_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_calib_request_pack(system_id, component_id, &msg , packet1.calib_type , packet1.acc_calib_mode );
	mavlink_msg_imu_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_calib_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.calib_type , packet1.acc_calib_mode );
	mavlink_msg_imu_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_imu_calib_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_imu_calib_request_send(MAVLINK_COMM_1 , packet1.calib_type , packet1.acc_calib_mode );
	mavlink_msg_imu_calib_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_system_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_system_status_t packet_in = {
		17.0,
	17,
	84,
	151,
	218,
	};
	mavlink_system_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.battery_voltage = packet_in.battery_voltage;
        	packet1.imu_calib = packet_in.imu_calib;
        	packet1.sat_numbers = packet_in.sat_numbers;
        	packet1.status1 = packet_in.status1;
        	packet1.status2 = packet_in.status2;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_system_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_system_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_system_status_pack(system_id, component_id, &msg , packet1.battery_voltage , packet1.imu_calib , packet1.sat_numbers , packet1.status1 , packet1.status2 );
	mavlink_msg_system_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_system_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.battery_voltage , packet1.imu_calib , packet1.sat_numbers , packet1.status1 , packet1.status2 );
	mavlink_msg_system_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_system_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_system_status_send(MAVLINK_COMM_1 , packet1.battery_voltage , packet1.imu_calib , packet1.sat_numbers , packet1.status1 , packet1.status2 );
	mavlink_msg_system_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_debug_values(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_debug_values_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	18067,
	18171,
	18275,
	18379,
	};
	mavlink_debug_values_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.debug1 = packet_in.debug1;
        	packet1.debug2 = packet_in.debug2;
        	packet1.debug3 = packet_in.debug3;
        	packet1.debug4 = packet_in.debug4;
        	packet1.debug5 = packet_in.debug5;
        	packet1.debug6 = packet_in.debug6;
        	packet1.debug7 = packet_in.debug7;
        	packet1.debug8 = packet_in.debug8;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_values_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_debug_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_values_pack(system_id, component_id, &msg , packet1.debug1 , packet1.debug2 , packet1.debug3 , packet1.debug4 , packet1.debug5 , packet1.debug6 , packet1.debug7 , packet1.debug8 );
	mavlink_msg_debug_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_values_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.debug1 , packet1.debug2 , packet1.debug3 , packet1.debug4 , packet1.debug5 , packet1.debug6 , packet1.debug7 , packet1.debug8 );
	mavlink_msg_debug_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_debug_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_values_send(MAVLINK_COMM_1 , packet1.debug1 , packet1.debug2 , packet1.debug3 , packet1.debug4 , packet1.debug5 , packet1.debug6 , packet1.debug7 , packet1.debug8 );
	mavlink_msg_debug_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_unique_id_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_unique_id_request_t packet_in = {
		5,
	};
	mavlink_unique_id_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.device_name1 = packet_in.device_name1;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_unique_id_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_unique_id_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_unique_id_request_pack(system_id, component_id, &msg , packet1.device_name1 );
	mavlink_msg_unique_id_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_unique_id_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device_name1 );
	mavlink_msg_unique_id_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_unique_id_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_unique_id_request_send(MAVLINK_COMM_1 , packet1.device_name1 );
	mavlink_msg_unique_id_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_unique_id_values(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_unique_id_values_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	17651,
	17755,
	41,
	};
	mavlink_unique_id_values_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.unique_id_0 = packet_in.unique_id_0;
        	packet1.unique_id_1 = packet_in.unique_id_1;
        	packet1.unique_id_2 = packet_in.unique_id_2;
        	packet1.unique_id_3 = packet_in.unique_id_3;
        	packet1.unique_id_4 = packet_in.unique_id_4;
        	packet1.unique_id_5 = packet_in.unique_id_5;
        	packet1.device_name2 = packet_in.device_name2;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_unique_id_values_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_unique_id_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_unique_id_values_pack(system_id, component_id, &msg , packet1.device_name2 , packet1.unique_id_0 , packet1.unique_id_1 , packet1.unique_id_2 , packet1.unique_id_3 , packet1.unique_id_4 , packet1.unique_id_5 );
	mavlink_msg_unique_id_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_unique_id_values_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device_name2 , packet1.unique_id_0 , packet1.unique_id_1 , packet1.unique_id_2 , packet1.unique_id_3 , packet1.unique_id_4 , packet1.unique_id_5 );
	mavlink_msg_unique_id_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_unique_id_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_unique_id_values_send(MAVLINK_COMM_1 , packet1.device_name2 , packet1.unique_id_0 , packet1.unique_id_1 , packet1.unique_id_2 , packet1.unique_id_3 , packet1.unique_id_4 , packet1.unique_id_5 );
	mavlink_msg_unique_id_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_keycode_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_keycode_request_t packet_in = {
		5,
	};
	mavlink_keycode_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.device_name3 = packet_in.device_name3;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_keycode_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_keycode_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_keycode_request_pack(system_id, component_id, &msg , packet1.device_name3 );
	mavlink_msg_keycode_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_keycode_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device_name3 );
	mavlink_msg_keycode_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_keycode_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_keycode_request_send(MAVLINK_COMM_1 , packet1.device_name3 );
	mavlink_msg_keycode_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_keycode_value(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_keycode_value_t packet_in = {
		963497464,
	17,
	};
	mavlink_keycode_value_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.keycode = packet_in.keycode;
        	packet1.device_name4 = packet_in.device_name4;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_keycode_value_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_keycode_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_keycode_value_pack(system_id, component_id, &msg , packet1.device_name4 , packet1.keycode );
	mavlink_msg_keycode_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_keycode_value_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.device_name4 , packet1.keycode );
	mavlink_msg_keycode_value_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_keycode_value_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_keycode_value_send(MAVLINK_COMM_1 , packet1.device_name4 , packet1.keycode );
	mavlink_msg_keycode_value_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_motors_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_motors_control_t packet_in = {
		5,
	};
	mavlink_motors_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.command = packet_in.command;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motors_control_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_motors_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motors_control_pack(system_id, component_id, &msg , packet1.command );
	mavlink_msg_motors_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motors_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command );
	mavlink_msg_motors_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_motors_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_motors_control_send(MAVLINK_COMM_1 , packet1.command );
	mavlink_msg_motors_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gremsyBGC(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_ppm_chan_values(system_id, component_id, last_msg);
	mavlink_test_sbus_chan_values(system_id, component_id, last_msg);
	mavlink_test_rc_simulation(system_id, component_id, last_msg);
	mavlink_test_imu_calib_request(system_id, component_id, last_msg);
	mavlink_test_system_status(system_id, component_id, last_msg);
	mavlink_test_debug_values(system_id, component_id, last_msg);
	mavlink_test_unique_id_request(system_id, component_id, last_msg);
	mavlink_test_unique_id_values(system_id, component_id, last_msg);
	mavlink_test_keycode_request(system_id, component_id, last_msg);
	mavlink_test_keycode_value(system_id, component_id, last_msg);
	mavlink_test_motors_control(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // GREMSYBGC_TESTSUITE_H

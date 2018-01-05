#include <iostream>
#include "mavlink/v1.0/common/mavlink.h"

#define MY_MAVLINK_COMM_NUM_BUFFERS 16
#define PI (float)3.14159

using namespace std;

struct CopterData
{
	//float roll;/* unit: rad, -pi..+pi */
	//float pitch;/* unit: rad, -pi..+pi */
	//float yaw;/* unit: rad, -pi..+pi */
	//float rollSpeed;/* unit: rad/s */
	//float pitchSpeed;/* unit: rad/s */
	//float yawSpeed;/* unit: rad/s */
	mavlink_attitude_t attitude;
	float altitudeSonar;/* unit: m */
};

class MavlinkData
{
	public:
	MavlinkData();
	uint8_t my_mavlink_parse_char(uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
	void handleMessage(mavlink_message_t* msg);
	struct	CopterData getCopterData();
	
	private:
	mavlink_message_t mavlink_buffer[MY_MAVLINK_COMM_NUM_BUFFERS];
	mavlink_status_t mavlink_status[MY_MAVLINK_COMM_NUM_BUFFERS];
	struct CopterData copterData;
};
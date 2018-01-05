#include "mavlink/v1.0/ardupilotmega/mavlink.h"
#include "MavlinkData.h"

MavlinkData::MavlinkData() 
{
    copterData.attitude.roll = 0;
    copterData.attitude.pitch = 0;
    copterData.attitude.yaw = 0;
    copterData.attitude.rollspeed = 0;
    copterData.attitude.pitchspeed = 0;
    copterData.attitude.yawspeed = 0;
    copterData.altitudeSonar = 0;
}

uint8_t MavlinkData::my_mavlink_parse_char(uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
    uint8_t msg_received = mavlink_frame_char_buffer(mavlink_buffer, mavlink_status, c, r_message,
					r_mavlink_status);
    if (msg_received == MAVLINK_FRAMING_BAD_CRC) 
    {
	    // we got a bad CRC. Treat as a parse failure
	    mavlink_message_t* rxmsg = mavlink_buffer;
	    mavlink_status_t* status = mavlink_status;
	    //_mav_parse_error(status);
	    status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
	    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
	    if (c == MAVLINK_STX)
	    {
		    status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
		    rxmsg->len = 0;
		    mavlink_start_checksum(rxmsg);
	    }
	    return 0;
    }
    return msg_received;
}

void MavlinkData::handleMessage(mavlink_message_t* msg)
{
    switch (msg->msgid) 
    {
    	case MAVLINK_MSG_ID_ATTITUDE:
    		mavlink_msg_attitude_decode(msg, &copterData.attitude);
    		cout << "attitude msg:" << (copterData.attitude.roll*180/PI) << "," 
    			<< (copterData.attitude.pitch*180/PI) << ","
    			<< (copterData.attitude.yaw*180/PI) << endl;
    		break;
    	case 74: /* VFR HUD*/
            //copterData.altitudeSonar = mavlink_msg_vfr_hud_get_alt(msg);
            //cout << "sonar " << copterData.altitudeSonar << endl;
    		break;
        case MAVLINK_MSG_ID_RANGEFINDER:
            copterData.altitudeSonar = mavlink_msg_rangefinder_get_distance(msg);
            cout << "sonar " << copterData.altitudeSonar << endl;
            break;    
    	default:
    		break;
    }	
}

struct	CopterData MavlinkData::getCopterData()
{
	return copterData;
}

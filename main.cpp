/* main.cpp
 * Author: Support team - RYA 2017
 * Date: 21/9
*/

//#include <QUdpSocket>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>
#include <pthread.h>
#include <time.h>

//#include "imageprocess.h"
#include "Object.h"
#include "utility.h"
#include "TCPServer.h"
#include "pid.h"
#include "mavlink/v1.0/common/mavlink.h"

//#define __USE_TCP__
//#define __DEBUG__

/* Type mask for MSG_SET_ATTITUDE_TARGET: If any of these bits are set, the corresponding input should be ignored*/
#define ROLL_BIT 0X01
#define PITCH_BIT 0X02
#define YAW_BIT 0X04
#define THROTTLE_BIT 0X40
#define ATTITUDE_BIT 0X80

using namespace cv;
using namespace std;

bool calibrationMode = false;

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

const int FRAME_WIDTH = 213;
const int FRAME_HEIGHT = 160;
const float INV_SCALE_FACTOR = 640/FRAME_WIDTH;

const int MAX_NUM_OBJECTS = 5;
const int MIN_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/25;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

Mat dst, detected_edges;
Mat src, src_gray;
int edgeThresh = 1;
int ratio = 3;
int filter_kernel_size = 3;
int erode_kernel_size = 2;
int dilate_kernel_size = 4;
const char* window_name = "Edge Map";

PID pid_roll;
PID pid_pitch;
mavlink_set_attitude_target_t msg_target;
mavlink_message_t msg_mav;

#ifdef __USE_TCP__
#include "TCPServer.h"
TCPServer server;
#else
#include "UDPServer.h"
UDPServer server;
#endif
pthread_t serverThread;
MavlinkData mavlinkData;

std::string format(long num) {
  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(8) << num;
  return oss.str().insert(3, ",").insert(6, ",");
}

void createTrackbars(){
	
#ifdef __DEBUG__
    namedWindow(trackbarWindowName,0);
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX );
    createTrackbar( "Erode", trackbarWindowName, &erode_kernel_size, 8 );
    createTrackbar( "Dilate", trackbarWindowName, &dilate_kernel_size, 8 );    
    createTrackbar( "Filter", trackbarWindowName, &filter_kernel_size, 8 );        
#endif

}

void drawObject(vector<Object> theObjects,Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy){
    for(unsigned int i =0; i<theObjects.size(); i++){
        drawContours(frame,contours,i,theObjects.at(i).getColor(),3,8,hierarchy);
        circle(frame,Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),5,theObjects.at(i).getColor());
        putText(frame,Utility().intToString(theObjects.at(i).getXPos())+ " , " + Utility().intToString(theObjects.at(i).getYPos()),Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,theObjects.at(i).getColor());
        putText(frame,theObjects.at(i).getType(),Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-20),1,2,theObjects.at(i).getColor());
    }
}

void drawObject(vector<Object> theObjects,Mat &frame){

    for( unsigned int i =0; i<theObjects.size(); i++){

        circle(frame,Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),10,Scalar(0,0,255));
        putText(frame,Utility().intToString(theObjects.at(i).getXPos())+ " , " + Utility().intToString(theObjects.at(i).getYPos()),Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,Scalar(0,255,0));
        putText(frame,theObjects.at(i).getType(),Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-30),1,2,theObjects.at(i).getColor());
    }
}

void morphOps(Mat &thresh){

    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(2*erode_kernel_size + 1,2*erode_kernel_size + 1));
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(2*dilate_kernel_size + 1,2*dilate_kernel_size + 1));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);

    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);
}
void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed)
{
    vector <Object> objects;
    Mat temp;
    threshold.copyTo(temp);

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

    double refArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();

        if(numObjects<MAX_NUM_OBJECTS)
        {
            for (int index = 0; index >= 0; index = hierarchy[index][0])
            {
                Moments moment = moments((Mat)contours[index]);
                double area = moment.m00;

                if(area>MIN_OBJECT_AREA)
                {
                    Object object;

                    object.setXPos(moment.m10/area);
                    object.setYPos(moment.m01/area);

                    objects.push_back(object);

                    objectFound = true;

                }
                else objectFound = false;
            }

            if(objectFound ==true)
            {
#ifdef __DEBUG__
                drawObject(objects,cameraFeed);
#endif
            }
        }
#ifdef __DEBUG__        
        else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
#endif        
    }
}

void trackFilteredObject(Object theObject,Mat threshold,Mat HSV, Mat &cameraFeed){
    int max_area_idx = 0;
    int max_area = 0;
    //vector <Object> objects;
    Mat temp;
    threshold.copyTo(temp);

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
	float err_roll, err_pitch, u_roll, u_pitch;

    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
    //cout << contours.size() << endl;
	for (unsigned int i=0; i < contours.size(); i++) {
	    int area = contourArea(contours.at(i));
        if (area > max_area)
        {
            max_area = area;
            max_area_idx = i;
        }
        else
            continue;
    }
    if (max_area > MIN_OBJECT_AREA) 
    {
        uint32_t time;
        //RotatedRect rect = minAreaRect(contours.at(max_area_idx));
        
        // Get the moments
        //vector<Moments> mu(contours.size() );
        //for( int i = 0; i < contours.size(); i++ )
        //{ mu[i] = moments( contours[i], false ); }
        Moments mu;
        mu = moments( contours[max_area_idx], true );

        //  Get the mass centers:
        //vector<Point2f> mc( contours.size() );
        //for( int i = 0; i < contours.size(); i++ )
        //{ mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
        Point2f mc;
        mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 ); 

		//OBJECT FOUND
#ifdef __DEBUG__
        //putText(cameraFeed, "OBJECT", rect.center, FONT_HERSHEY_COMPLEX, 0.8, Scalar(255,255,255), 1);
        //drawContours(cameraFeed,contours,max_area_idx,Scalar(255,255,255),1);
        //circle(cameraFeed, rect.center, 5, Scalar(255,255,255));
        circle(cameraFeed, mc, 5, Scalar(255,255,255));
#endif
		//PID process
        //err_roll = rect.center.x - FRAME_WIDTH/2;
        //err_pitch = rect.center.y - FRAME_HEIGHT/2;
        err_roll = mc.x - FRAME_WIDTH/2;
        err_pitch = mc.y - FRAME_HEIGHT/2;
        //cout << err_roll << "," << err_pitch << endl;	
        time = Utility::millis();
        u_roll = pid_roll.pid_process(err_roll, time);
        u_pitch = pid_pitch.pid_process(err_pitch, time);        
        //cout << (int)err_roll << "," << (int)u_roll << "," << (int)err_pitch << "," << (int)u_pitch << endl;

		//setup package
		msg_target.time_boot_ms = time;
		msg_target.body_roll_rate = u_roll;
		msg_target.body_pitch_rate = u_pitch;
		msg_target.body_yaw_rate = 0;
		msg_target.thrust = 0.5;
		msg_target.type_mask = 0;
			
		mavlink_msg_set_attitude_target_encode(2, 1, &msg_mav, &msg_target);
			
		server.sendBuff((const char*) &msg_mav, sizeof(msg_mav));
	}
	else
	{
		//pid_roll.pid_reset();
		//pid_pitch.pid_reset();
	}
}

void* ServerTask(void *arg)
{	
	cout << "Server Task run" << endl;	
	server.start();
	server.closeServer();
}

int main()
{
    int num_frames = 0;
    time_t start,end;	
    time(&start);

    Mat cameraFeed;
    Mat threshold;
    Mat HSV,im_Grey;

    Object blue("blue"), yellow("yellow"), red("red"), green("green");

    Utility::initTime();

    if(calibrationMode){
        createTrackbars();
    }
    VideoCapture capture;
    capture.open(1);
    if (!capture.isOpened())
        capture.open(0);
            if (!capture.isOpened())
                return -1;
    capture.set(CV_CAP_PROP_FPS, 30);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	
	pid_roll.pid_set_k_params(1.5*INV_SCALE_FACTOR, 0*INV_SCALE_FACTOR, 0.1*INV_SCALE_FACTOR, 0.04, 1000);
	pid_pitch.pid_set_k_params(1.5*INV_SCALE_FACTOR, 0*INV_SCALE_FACTOR, 0.1*INV_SCALE_FACTOR, 0.04, 1000);

	server.setup(8000);
	server.attach(&mavlinkData);
	pthread_create(&serverThread,NULL,&ServerTask,NULL);

    while(1){
        capture.read(cameraFeed);
        src = cameraFeed;
        usleep(1000);
        while( !src.data )
        {
            capture.read(cameraFeed);
            src = cameraFeed;
            usleep(1000);
        }
        cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
        if(calibrationMode==true)
        {
            //medianBlur (cameraFeed, cameraFeed, 2*filter_kernel_size+1 );
            //GaussianBlur( cameraFeed, cameraFeed, Size( 2*filter_kernel_size+1, 2*filter_kernel_size+1 ), 0, 0 );
            cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
            inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
            morphOps(threshold);
			
#ifdef __DEBUG__			
            imshow(windowName2,threshold);
            imshow(windowName1,HSV);
#endif

            dst.create( src.size(), src.type() );
            cvtColor( src, src_gray, CV_BGR2GRAY );

            trackFilteredObject(threshold,HSV,cameraFeed);
        }
        else
        {
//            cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
//            inRange(HSV,blue.getHSVmin(),blue.getHSVmax(),threshold);
//            morphOps(threshold);
//            trackFilteredObject(blue,threshold,HSV,cameraFeed);

//            cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
//            inRange(HSV,yellow.getHSVmin(),yellow.getHSVmax(),threshold);
//            morphOps(threshold);
//            trackFilteredObject(yellow,threshold,HSV,cameraFeed);

            cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
            inRange(HSV,green.getHSVmin(),green.getHSVmax(),threshold);
            morphOps(threshold);
#ifdef __DEBUG__            
            imshow(windowName2,threshold);
#endif			
            //cvtColor(cameraFeed, im_Grey, COLOR_BGR2GRAY);
			//cv::threshold(im_Grey, threshold,125, 255, THRESH_BINARY); 
            trackFilteredObject(red,threshold,HSV,cameraFeed);

//            cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
//            inRange(HSV,green.getHSVmin(),green.getHSVmax(),threshold);
//            morphOps(threshold);
//            trackFilteredObject(green,threshold,HSV,cameraFeed);
        }
#ifdef __DEBUG__
        imshow(windowName,cameraFeed);
#endif
//        imshow(windowName1,HSV);
		++num_frames;
		time(&end);
		double seconds = difftime(end, start);
		if (seconds >= 1.0)
		{
		   cout << "Frame per second: " << num_frames << " Time: " << 
		   seconds << endl;
		   time(&start);
		   num_frames = 0; 
		}
#ifdef __DEBUG__		
        if (waitKey(10) == 'q')
            break;
#endif
    }
    return 0;
}

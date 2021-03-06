#include <iostream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_altitude.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

#define MIDDLE 0
#define RIGHT 1
#define LEFT 2
#define Z_SET 1.5

class Face_Follow
{
private:
	ros::NodeHandle node;
	ros::Subscriber faceside_sub;
	ros::Subscriber faceloc_sub;
	ros::Publisher cmd_pub;

	geometry_msgs::Twist cmd;
	int current_mode;
	float face_distance;
	geometry_msgs::Point face_center;

public:
	Face_Follow();
	~Face_Follow();
	void run(double freq);
	void iteration(const ros::TimerEvent& e);
	void facesideCallback(const std_msgs::UInt8 &msg);
	void facelocCallback(const geometry_msgs::Point &msg);
};

Face_Follow::Face_Follow()
{
	faceside_sub = node.subscribe("/face_side",1,&Face_Follow::facesideCallback,this);
	faceloc_sub = node.subscribe("/face_loc",1,&Face_Follow::facelocCallback,this);
	cmd_pub = node.advertise<geometry_msgs::Twist>("cmd_vel_ref", 1000);
}

Face_Follow::~Face_Follow()
{}

void Face_Follow::run(double freq)
{
	ros::NodeHandle node;
	/*code*/
	current_mode = MIDDLE;
	ros::Timer timer = node.createTimer(ros::Duration(1.0/freq), &Face_Follow::iteration, this);
	ros::spin();
}

void Face_Follow::iteration(const ros::TimerEvent& e)
{
	static float time_elapse = 0;
	float dt = e.current_real.toSec() - e.last_real.toSec();
	time_elapse += dt;
	/*flight control code*/

	if (fabs(face_center.y)>40)
	{
		cmd.linear.x = 0.0;
	    cmd.linear.y = 0.0;
	    cmd.linear.z = -face_center.y/100.0;
	    cmd.angular.x = 0.0;
	    cmd.angular.y = 0.0;
	    cmd.angular.z = 0.0;
	    ROS_INFO("altitude control~~~~~\n");
	}else if (fabs(face_center.x)>50)
	{
		cmd.linear.x = 0.0;
	    cmd.linear.y = 0.0;
	    cmd.linear.z = 0.0;
	    cmd.angular.x = 0.0;
	    cmd.angular.y = 0.0;
	    cmd.angular.z = face_center.x/100.0;
	    ROS_INFO("yaw control~~~~~\n");
	}else{
		switch(current_mode)
		{
			case MIDDLE:
				cmd.linear.x = (100.0-face_distance)*0.01;
	    		cmd.linear.y = 0.0;
	   			cmd.linear.z = 0.0;
	    		cmd.angular.x = 0.0;
	    		cmd.angular.y = 0.0;
	    		cmd.angular.z = 0.0;
			    ROS_INFO("distance control~~~~~\n");break;
			case RIGHT:
				cmd.linear.x = 0.0;
				cmd.linear.y = 1.0;
				cmd.linear.z = 0.0;
	    		cmd.angular.x = 0.0;
				cmd.angular.y = 0.0;
				cmd.angular.z = 0.0;break;
			case LEFT:;break;
			default:
				cmd.linear.x = 0.0;
	    		cmd.linear.y = -1.0;
	    		cmd.linear.z = 0.0;
	    		cmd.angular.x = 0.0;
	   		 	cmd.angular.y = 0.0;
	    		cmd.angular.z = 0.0;;break;
		}
	}
	
	cmd_pub.publish(cmd);
}

void Face_Follow::facesideCallback(const std_msgs::UInt8 &msg)
{
	current_mode = msg.data;
}

void Face_Follow::facelocCallback(const geometry_msgs::Point &msg)
{
	/*x, y is col&roll, z is the size of the face*/
	face_center.x = msg.x;
	face_center.y = msg.y;
	face_distance = msg.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "facefollow");
	Face_Follow ff;
	ff.run(50);
	return 0;
}
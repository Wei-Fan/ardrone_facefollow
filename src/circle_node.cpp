#include "ardrone_autonomy/Navdata.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <ardrone_facefollow/output2.h>
#include <ardrone_facefollow/faceangle.h>
#include <math.h>

using namespace std;

//process the input
ardrone_facefollow::output2 face_current_center;
//ardrone_facefollow::faceangle faceangle_value;
int dx=0;
int dy=0;
int level_x=0;
int level_y=0;
int fullwidth=0;
int faceangle_value;
void face_center_cb(const ardrone_facefollow::output2::ConstPtr& center){
     face_current_center=*center;
     dx = face_current_center.pose1.x/2 - face_current_center.pose2.x;
     dy = face_current_center.pose1.y/2 - face_current_center.pose2.y;
     level_x=face_current_center.pose3.x;
     level_y=face_current_center.pose3.y;
     fullwidth=face_current_center.pose1.x;
}
void face_angle_cb(const ardrone_facefollow::faceangle::ConstPtr& value){
     faceangle_value = value->value;
}

//controller
int main(int argc, char **argv){
	ros::init(argc, argv, "circle_node");
	ros::NodeHandle nh;
	std_msgs::String msg;
    std::stringstream ss;
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    double facesize_p=0.0;//Float32

    ros::Subscriber circle_sub=  nh.subscribe<ardrone_facefollow::output2>("/ardrone_facefollow/output2_msg", 20, face_center_cb);
    ros::Subscriber faceangle_sub=  nh.subscribe<ardrone_facefollow::faceangle>("/ardrone_facefollow/faceangle", 20, face_angle_cb);
    ros::Publisher cmd_pub= nh.advertise<geometry_msgs::Twist>("cmd_vel_ref", 1000);
    ros::Publisher takeoff_pub= nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
    ros::Publisher land_pub= nh.advertise<std_msgs::Empty>("/ardrone/land", 1);

    ros::Rate rate(20.0);

    while(ros::ok()){
    	ros::spinOnce();
    	//altitude control
    	if(abs(dy)<=10){
    		cmd.linear.z = 0.0;
    	}
    	else{
    		cmd.linear.z=0.2*double(-dy/abs(level_y))/0.7;//Float32//support max(cmd.linear.z)=0.7m/s
    	}
    	//distance control
    	facesize_p=(double(level_x/fullwidth)-0.25)/0.25;
    	if(abs(facesize_p)<=0.05){
    		cmd.linear.x = 0.0;
    	}
    	else{
    		cmd.linear.x =-0.2*facesize_p/0.1455381836;// max(cmd.linear.x)=0.1455381836m/s
    	}
    	//make sure the face is centered
    	//frontalface case
    	if(faceangle_value==0){//the value is unknown
    		ROS_INFO("Frontalface Detected!");
    		if(abs(dy)<=10){
    			cmd.linear.y = 0.0;
    		}
    		else{
    			cmd.linear.y=0.2*double(dx/abs(level_x))/0.1455381836;//Float32
    		}
    		cmd.angular.z=0.0;
    	}
    	else{
    		//leftsideface case
    		if(faceangle_value==1){
    			ROS_INFO("Leftsideface Detected!");
    			cmd.linear.y=1.0;//support max(cmd.linear.x)=0.1455381836m/s
    			if(abs(dx)<=10){
    				cmd.angular.z=0.0;
    			}
    			else{
    				cmd.angular.z=double(18*dx/fullwidth);//Float32//support max(cmd.angular.z= 100 deg/s)
    			}
    			
    		}
    		//rightsideface case
    		else{
    			if(faceangle_value==2){
    			     ROS_INFO("Rightsideface Detected!");
    			     cmd.linear.y=-1.0;
    			     if(abs(dx)<=10){
    				    cmd.angular.z=0.0;
    			     }
                 }
    			else{
    				cmd.angular.z=double(18*dx/fullwidth);//Float32
    			}
    		}
    	}
    	cmd_pub.publish(cmd);

    }
    return 0;
}
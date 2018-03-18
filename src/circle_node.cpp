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
//#include <fdetection/face_detection.h>
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
//ardrone_facefollow::faceangle faceangle_value;
std_msgs::UInt8 faceangle_value;
//int faceangle_int;
void face_center_cb(const ardrone_facefollow::output2::ConstPtr& center){
     face_current_center=*center;
     dx = face_current_center.pose1.x/2 - face_current_center.pose2.x;
     dy = face_current_center.pose1.y/2 - face_current_center.pose2.y;
     level_x=face_current_center.pose3.x;
     level_y=face_current_center.pose3.y;
     fullwidth=face_current_center.pose1.x;
     ROS_INFO("dx dy level_x level_y : %d  %d  %d  %d",dx,dy,level_x,level_y);
}
// void face_angle_cb(const ardrone_facefollow::faceangle::ConstPtr& value){
//      faceangle_value = *value;
//      faceangle_int = faceangle_value.value.data;
//      ROS_INFO("faceangle_value : %d",faceangle_int);
// }

void face_angle_cb(const std_msgs::UInt8 & value){
     faceangle_value.data = value.data;
     //faceangle_int = faceangle_value.data;
     //ROS_INFO("faceangle_value : %d",faceangle_value.data);
}

//controller
int main(int argc, char **argv){
    ros::init(argc, argv, "circle_node");
    ros::NodeHandle nh;
    //std_msgs::String msg;
    //std::stringstream ss;
    geometry_msgs::Twist cmd;
    double facesize_p=0.0;//Float32

    ros::Subscriber circle_sub=  nh.subscribe("ardrone_facefollow/output2_msg", 1000, face_center_cb);//<ardrone_facefollow::output2>
    ros::Subscriber faceangle_sub=  nh.subscribe("ardrone_facefollow/faceangle", 1000, face_angle_cb);//<ardrone_facefollow::faceangle>
    ros::Publisher cmd_pub= nh.advertise<geometry_msgs::Twist>("cmd_vel_ref", 1000);
    ros::Publisher takeoff_pub= nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
    ros::Publisher land_pub= nh.advertise<std_msgs::Empty>("/ardrone/land", 1);

    ros::Rate rate(20.0);

    while(ros::ok()){
        //ros::spinOnce();
        //ROS_INFO("faceangle_value : %d",faceangle_int);
        if ((level_x >0)&&(fullwidth>0)){

        	cmd.linear.x = 0.0;
		    cmd.linear.y = 0.0;
		    cmd.linear.z = 0.0;
		    cmd.angular.x = 0.0;
		    cmd.angular.y = 0.0;
		    cmd.angular.z = 0.0;
            
            if(abs(dy)<=10)
            {
                cmd.linear.z = 0.0;
            }else{
            	cmd.linear.z=0.7*double(dy/abs(level_y))/0.7;//Float32//support max(cmd.linear.z)=0.7m/s 
        	}
	        //distance control
	        facesize_p=(double(level_x/fullwidth)-0.1)/0.1;
	        ROS_INFO("facesize_p : %f",facesize_p);
	        if(fabs(facesize_p)<=0.01){
	            cmd.linear.x = 0.0;
	        }else{
	            cmd.linear.x =-double(level_x/fullwidth)/0.1455381836;// max(cmd.linear.x)=0.1455381836m/s
	        }
	        //make sure the face is centered
	        //frontalface case
	        /*if(faceangle_value.data==2)//faceangle_value.value==2)
	        {//the value is unknown
	            //ROS_INFO("Frontalface Detected!");
	            if(abs(dy)<=10){
	                cmd.linear.y = 0.0;
	            }
	            else{
	                cmd.linear.y=0.2*double(dx/abs(level_x))/0.1455381836;//Float32 level_x
	            }
	            cmd.angular.z=0.0;
	        }else if(faceangle_value.data==0)//faceangle_value.value==0)//leftsideface case
	        {
	            //ROS_INFO("Leftsideface Detected!");
	            cmd.linear.y=1.0;//support max(cmd.linear.x)=0.1455381836m/s
	            if(abs(dx)<=10){
	                cmd.angular.z=0.0;
	            }
	            else{
	                cmd.angular.z=double(18*dx/fullwidth);//Float32//support max(cmd.angular.z= 100 deg/s)
	            }
	            
	        }else if(faceangle_value.data==1)//faceangle_value.value==1)
	        {//rightsideface case
	             //ROS_INFO("Rightsideface Detected!");
	             cmd.linear.y=-1.0;
	             if(abs(dx)<=10){
	                cmd.angular.z=0.0;
	             }else{
	                cmd.angular.z=double(18*dx/fullwidth);//Float32
	            }
	        }*/
	        //altitude control
	        
	        cmd_pub.publish(cmd);
	    }
	    ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
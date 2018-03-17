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

class CircleNode
{
public:
	CircleNode(ros::NodeHandle& nh);
    void run(double frequency);
    void iteration(const ros::TimerEvent& e);
private:
	ros::Subscriber circle_sub;
	ros::Subscriber faceangle_sub;
    ros::Publisher cmd_pub;
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;

	void face_center_cb(const ardrone_facefollow::output2::ConstPtr& center);
	void face_angle_cb(const std_msgs::UInt8 & value);

	ardrone_facefollow::output2 face_current_center;
	//ardrone_facefollow::faceangle faceangle_value;
	int dx;
	int dy;
	int level_x;
	int level_y;
	int fullwidth;
    std_msgs::UInt8 faceangle_value;
	// ardrone_facefollow::faceangle faceangle_value;
    // int faceangle_int;

	geometry_msgs::Twist cmd;
    double facesize_p;//Float32
};

CircleNode::CircleNode(ros::NodeHandle& nh)
{
	circle_sub = nh.subscribe<ardrone_facefollow::output2>("/ardrone_facefollow/output2_msg", 20, &CircleNode::face_center_cb,this);
    faceangle_sub = nh.subscribe("/ardrone_facefollow/faceangle", 20, &CircleNode::face_angle_cb,this);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_ref", 1000);
    takeoff_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
    land_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);

    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    dx=0;
    dy=0;
    level_x=0;
    level_y=0;
    fullwidth=0;
    facesize_p=0.0;
}

void CircleNode::face_center_cb(const ardrone_facefollow::output2::ConstPtr& center){
     face_current_center=*center;
     dx = face_current_center.pose1.x/2 - face_current_center.pose2.x;
     dy = face_current_center.pose1.y/2 - face_current_center.pose2.y;
     level_x=face_current_center.pose3.x;
     level_y=face_current_center.pose3.y;
     fullwidth=face_current_center.pose1.x;
     ROS_INFO("face_center_cb : %d",level_x);
}

void CircleNode::face_angle_cb(const std_msgs::UInt8 & value){
     faceangle_value.data = value.data;
     //faceangle_int = faceangle_value.data;
     ROS_INFO("faceangle_value : %d",faceangle_value.data);
}

void CircleNode::run(double frequency)
{
    ros::NodeHandle node;
    ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &CircleNode::iteration, this);
    ros::spin();
}

void CircleNode::iteration(const ros::TimerEvent& e)
{
    if ((level_x >0)&&(fullwidth>0)){
        if(abs(dy)<=10){
            cmd.linear.z = 0.0;
        }else{
            cmd.linear.z=0.2*double(-dy/abs(level_y))/0.7;//Float32//support max(cmd.linear.z)=0.7m/s 
        }
        //distance control
        facesize_p=(double(level_x/fullwidth-0.25))/0.25;
        if(abs(facesize_p)<=0.05){
            cmd.linear.x = 0.0;
        }else{
            cmd.linear.x =-0.2*facesize_p/0.1455381836;// max(cmd.linear.x)=0.1455381836m/s
        }
        //make sure the face is centered
        //frontalface case
        if(faceangle_value.data==2)//faceangle_value.value==2)
        {//the value is unknown
            //ROS_INFO("Frontalface Detected!");
            if(abs(dy)<=10){
                cmd.linear.y = 0.0;
            }
            else{
                cmd.linear.y=0.2*double(dx/abs(100))/0.1455381836;//Float32 level_x
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
        }else{
            //ROS_INFO("Not Detected!");
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.x = 0.0;
            cmd.angular.y = 0.0;
            cmd.angular.z = 0.0;
        }
        //altitude control
        
        //cmd_pub.publish(cmd);
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "circle_node");
    ros::NodeHandle n;
	CircleNode c(n);
	c.run(50);
	return 0;
}
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
#include <queue>
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
    std_msgs::UInt8 faceangle_norm;//filter
    std::queue<std_msgs::UInt8> faceangle_queue;
    int faceangle_sum;
	// ardrone_facefollow::faceangle faceangle_value;
    // int faceangle_int;

	geometry_msgs::Twist cmd;
    double facesize_p;//Float32

    int count;
    bool faceside_lock;
    bool yaw_lock;
    bool seeYourFace;
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

    faceangle_norm.data = 1;
    faceangle_sum = 0;
    count = 0;
    faceside_lock = false;
    yaw_lock = false;
    seeYourFace = false;
}

void CircleNode::face_center_cb(const ardrone_facefollow::output2::ConstPtr& center){
     face_current_center=*center;
     dx = face_current_center.pose1.x/2 - face_current_center.pose2.x;
     dy = face_current_center.pose1.y/2 - face_current_center.pose2.y;
     level_x=face_current_center.pose3.x;
     level_y=face_current_center.pose3.y;
     fullwidth=face_current_center.pose1.x;
     seeYourFace = true;
     //ROS_INFO("face_center_cb : %d",level_x);
}

void CircleNode::face_angle_cb(const std_msgs::UInt8 & value){
     faceangle_value.data = value.data;
     //faceangle_int = faceangle_value.data;
     faceangle_queue.push(faceangle_value);
     faceangle_sum = faceangle_sum + faceangle_value.data;

     if (faceangle_queue.size() ==11)
     {
         faceangle_sum = faceangle_sum - faceangle_queue.front().data;
         faceangle_queue.pop();
         if (faceangle_sum < 6)
         {
            faceangle_norm.data = 0;
         } else if (faceangle_sum < 18)
         {
            faceangle_norm.data = 1;
         } else {
            faceangle_norm.data = 2;
         }
     }
     ROS_INFO("faceangle_norm : %d",faceangle_norm.data);
     // int left_count = 0;
     // int right_count = 0;
     // int front_count = 0;
     // for (int i = 0; i < faceangle_queue.size(); ++i)
     // {
     //     if (faceangle_queue(i).data == 0)
     //     {
     //         left_count++;
     //     } else if (faceangle_queue(i).data == 1)
     //     {
     //         front_count++;
     //     } else {
     //        right_count++;
     //     }
     // }
     // if (front_count > 2)
     // {
     //    faceangle_norm.data = 1;
     // } else if (right_count <= left_count)
     // {
     //    faceangle_norm.data = 0;
     // } else if (right_count > left_count)
     // {
     //    faceangle_norm.data = 2;
     // }
}

void CircleNode::run(double frequency)
{
    ros::NodeHandle node;
    ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &CircleNode::iteration, this);
    ros::spin();
}

void CircleNode::iteration(const ros::TimerEvent& e)
{
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    
    if (count == 10)
    {
        count = 0;
        faceside_lock = true;
    }

    if (seeYourFace)
    {
        if (!faceside_lock)//aim
        {
        
            //altitude control
            if(fabs(dy)<=30)
            {
                cmd.linear.z = 0.0;
            }else{
                cmd.linear.z=0.1*double(dy)/double(level_y);//Float32//support max(cmd.linear.z)=0.7m/s 
            }

            //distance control
            facesize_p=(double(level_x)/double(fullwidth)-0.09)/0.09;
            //ROS_INFO("facesize_p : %f",facesize_p);
            if(fabs(facesize_p)<=0.05){
                cmd.linear.x = 0.0;
            }else{
                cmd.linear.x = -0.6*facesize_p;// max(cmd.linear.x)=0.1455381836m/s
            }

            cmd.linear.y = 0.0025*dx;
            
            cmd_pub.publish(cmd);
            count++;

        } else {
            //make sure the face is centered
            //frontalface case
            if(faceangle_norm.data==1)//faceangle_value.value==2)
            {
                faceside_lock = false;
                ROS_INFO("~~~~~~~Frontalface Detected!");
                // if(abs(dy)<=10){
                //     cmd.linear.y = 0.0;
                // }
                // else{
                //     cmd.linear.y=0.2*double(dx/abs(100))/0.1455381836;//Float32 level_x
                // }
                // cmd.angular.z=0.0;
            }else if(faceangle_norm.data==0)//faceangle_value.value==0)//leftsideface case
            {
                if (fabs(dx)<30&&yaw_lock)
                {
                    yaw_lock = false;
                }
                ROS_INFO("~~~~~~~Leftsideface Detected!");
                if(dx>-150&&!yaw_lock){
                    cmd.linear.y=0.003*(dx+150);
                    ROS_INFO("vyl : %f", cmd.linear.y);
                }else{
                    if (!yaw_lock)
                    {
                        yaw_lock = true;
                    }
                    cmd.angular.z=-0.001*fabs(dx);//Float32//support max(cmd.angular.z= 100 deg/s)
                }
                
            }else if(faceangle_norm.data==2)//faceangle_value.value==1)
            {//rightsideface case
                if (fabs(dx)<30&&yaw_lock)
                {
                    yaw_lock = false;
                }
                ROS_INFO("~~~~~~~Rightsideface Detected!");
                if(dx<150&&!yaw_lock){
                    cmd.linear.y=0.003*(dx-150);
                    ROS_INFO("vyr : %f", cmd.linear.y);
                }
                else{
                    if (!yaw_lock)
                    {
                        yaw_lock = true;
                    }
                    cmd.angular.z=0.001*fabs(dx);//Float32//support max(cmd.angular.z= 100 deg/s)
                }
            }
            cmd_pub.publish(cmd);   
        }
    }
    seeYourFace = false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "circle_node");
    ros::NodeHandle n;
	CircleNode c(n);
	c.run(10);
	return 0;
}
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

#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

class Face_Follow
{
private:
	ros::NodeHandle node;


public:
	Face_Follow();
	~Face_Follow();
	void run(double freq);
	void iteration(const ros::TimerEvent& e);
	
};

Face_Follow::Face_Follow()
{

}

Face_Follow::~Face_Follow()
{}

void Face_Follow::run(double freq)
{
	ros::NodeHandle node;
	/*code*/

	ros::Timer timer = node.createTimer(ros::Duration(1.0/freq), &Face_Follow::iteration, this);
	ros::spin();
}

void Face_Follow::iteration(const ros::TimerEvent& e)
{
	static float time_elapse = 0;
	float dt = e.current_real.toSec() - e.last_real.toSec();
	time_elapse += dt;
	/*code*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ardrone_facefollow");
	Face_Follow ff;
	ff.run(50);
	return 0;
}
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h> 
//#include <drone_msgs/ThrustSettings.h>
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "test");
	ros::NodeHandle nh;
	//ros::Publisher pub = nh.advertise<drone_msgs::ThrustSettings>("drone_msgs/thrust_settings", 1000);

	//ROS_INFO_STREAM("Test: " << thrust[0]<< " " << thrust[1]<< " " <<  thrust[2] << " " <<  thrust[3]);
}

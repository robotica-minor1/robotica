#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h> 

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "flightcontroller");
	//flightcontroller fc;
	ros::NodeHandle nh;
	//ros::Subscriber sub = nh.subscribe("drone_flightcontrol/", 1000, &messageReceived);
	Eigen::Vector3f direction(4.0, 3.0, 0.0);
	//ROS_INFO_STREAM("Test" << direction.array().square());
	float t_total = rand();
	float thrust[4] = {t_total};
	ROS_INFO_STREAM("Test: " << thrust[0]<< " " << thrust[1]<< " " <<  thrust[2] << " " <<  thrust[3]);
}
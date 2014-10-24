#include <ros/ros.h>
#include "pid_controller.hpp"
#include "ThrustSettings"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "pid_controller");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("ros_1/cmd_vel", 1000, &messageReceived);
}

void messageReceived(const ThrustSettings msg) {
	ROS_INFO_STREAM("Received new settings: thrust[" << msg.thrust[0] << "," << msg.thrust[1] ","<< msg.thrust[2] << "," << msg.thrust[3] 
									<< "] \n thrustAngles[" << msg.thrustAngles[0] << "," << msg.thrustAngles[1] ","<< msg.thrustAngles[2] << "," << msg.thrustAngles[3] << "]");
									<< "] \n attitude[" << msg.attitude[0] << "," << msg.attitude[1] ","<< msg.attitude[2] << "]");
}
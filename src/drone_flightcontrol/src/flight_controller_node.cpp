#include <ros/ros.h>

#include "drone_msgs/ThrustSettings.h"
#include "flight_controller.hpp"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "flightcontroller");
	ros::NodeHandle nh; 
	ros::Publisher pub = nh.advertise<drone_msgs::ThrustSettings>("drone_msgs/thrust_settings", 1000);
	FlightController fc(pub);
}

#include <Eigen/Dense>
#include <ros/ros.h>
#include <math.h>
#include <drone_msgs/ThrustSettings.h>

#include "drone_constants.hpp"
#include "flightcontroller.hpp"

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "flightcontroller");
	ros::NodeHandle nh; 
	ros::Publisher pub = nh.advertise<drone_msgs::ThrustSettings>("drone_msgs/thrust_settings", 1000);
	Flightcontroller fc(pub);
}

void Flightcontroller::translate(Eigen::Vector3f direction) {
	drone_msgs::ThrustSettings new_msg;
	Eigen::Vector3f t = direction.array().square() * (cd * rho * s / 2);
	t(2) += w;
	float t_per_engine = t.norm()/4;
	new_msg.thrust = {t_per_engine, t_per_engine, t_per_engine, t_per_engine};
	float thrustAngle = atan(t(0)/t(2));
	new_msg.thrustAngles = {thrustAngle, thrustAngle, thrustAngle, thrustAngle};
	new_msg.attitude = {atan(t(1) / sqrt(pow(t(0), 2) + pow(t(2), 2))), 0, 0};
	publisher.publish(new_msg);
}
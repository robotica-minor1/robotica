#include <Eigen/Dense>
#include <ros/ros.h>
#include <math.h>
#include <drone_msgs/ThrustSettings.h>

#include "drone_constants.hpp"
#include "flight_controller.hpp"

drone_msgs::ThrustSettings FlightController::translate(Eigen::Vector3f direction) {
	drone_msgs::ThrustSettings new_msg;
	Eigen::Vector3f t = direction.array().square() * (CD * RHO * S / 2);
	t(2) += W;
	float t_per_engine = t.norm()/4;
	new_msg.thrust = {t_per_engine, t_per_engine, t_per_engine, t_per_engine};
	float thrustAngle = atan(t(0)/t(2));
	new_msg.thrustAngles = {thrustAngle, thrustAngle, thrustAngle, thrustAngle};
	new_msg.attitude = {atan(t(1) / sqrt(pow(t(0), 2) + pow(t(2), 2))), 0, 0};
	return new_msg;
}

void FlightController::roll() {

}
#include <Eigen/Dense>
#include <ros/ros.h>
#include <math.h>

#include "drone_constants.hpp"
#include "flightcontroller.hpp"
int main(int argc, char* argv[]) {
	ros::init(argc, argv, "flightcontroller");
}

void Flightcontroller::translate(Eigen::Vector3f direction) {
	Eigen::Vector3f t = direction.array().square() * (cd * rho * s / 2);
	t(2) += w;
	float t_per_engine = t.norm()/4;
	float thrust[4] = {t_per_engine, t_per_engine, t_per_engine, t_per_engine};
	float thrustAngle = atan(t(0)/t(2));
	float thrustAngles[4] = {thrustAngle, thrustAngle, thrustAngle, thrustAngle};
	float attitude[4] = {atan(t(1) / sqrt(pow(t(0), 2) + pow(t(2), 2))), 0, 0};
}
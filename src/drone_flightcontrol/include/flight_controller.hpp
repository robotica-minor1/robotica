#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_
#include <ros/ros.h>
#include <drone_msgs/ThrustSettings.h>
#include <Eigen/Dense>
class FlightController {
private:
	ros::Publisher publisher;
public: 
	FlightController(ros::Publisher pub);
	FlightController(){};
	drone_msgs::ThrustSettings translate(Eigen::Vector3f direction);
	void roll();
	//void rotate();
};
inline FlightController::FlightController(ros::Publisher pub) : publisher(pub){};
#endif //FLIGHT_CONTROLLER_HPP_

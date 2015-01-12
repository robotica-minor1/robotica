#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_
#include <Eigen/Dense>
#include <map>
#include <string> 

#include "drone.hpp"

class FlightController {
private:
	string navMode;
	float referenceThrust[4];
	Drone drone;
	Eigen::Vector3f direction;
	std::map<string,int> pidGains;
	Eigen::Vector3f landingPosition;
	fc_config config; 
	
	/*
		actuate takes reference thrust axis rotation and attitude, and makes it happen.
	*/
	void actuate(Eigen::Vector4f thrustPerEngine, Eigen::Vector4f axisRotation, Eigen::Vector4f referenceAttitude);
	void setThrust(Eigen::Vector4f thrustPerEngine);
	void updateReferenceThrust();
	void headingPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity);
	void rollPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity);
	void pitchPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity);
	void heightPID(Eigen::Vector3f absoluteDirection, Eigen::Vector3f differenceVelocity);
	
	
public: 
	/*
		setDirection Takes a reference direction and converts it to actuate settings. 
	*/
	setDirection(Eigen::Vector3f newDirection);
	FlightController();
	//setHoldPosition(Eigen::Vector3f newPosition);
};

void log(string message);
#endif //FLIGHT_CONTROLLER_HPP_
 
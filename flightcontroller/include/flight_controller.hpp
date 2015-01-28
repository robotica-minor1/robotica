#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_
#include <Eigen/Dense>
#include <map>
#include <string> 

#include "drone.hpp"

class FlightController {
private:
	std::string navMode;
	Eigen::Vector4f thrust;
	std::map<std::string, float> pidGains;
	Eigen::Vector3f direction;
	
	Eigen::Vector3f landingPosition;
	/*
		actuate takes reference thrust axis rotation and attitude, and makes it happen.
	*/
	void actuate(Eigen::Vector4f thrustPerEngine, Eigen::Vector4f axisRotation, Eigen::Vector4f referenceAttitude);
	void updateReferenceThrust(float gain, int signs[4]);
	void headingPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity);
	void rollPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity);
	void pitchPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity);
	void heightPID(Eigen::Vector3f absoluteDirection, Eigen::Vector3f differenceVelocity);
	void setHoldPosition(Eigen::Vector3f newPosition);
	void actuate();
	Eigen::Vector3f getDifferenceAttitude();
	Eigen::Vector3f getDifferenceRotationalVel();
	Eigen::Vector3f getDifferenceVel();
	void setReferenceVel(Eigen::Vector3f newRefSpeed);
	Eigen::Vector3f getAbsoluteDirection();
	void setReferencePosition(Eigen::Vector3f newPos);
	void hold();
	void land();

public: 
	/*
		setDirection Takes a reference direction and converts it to actuate settings. 
	*/
	void setDirection(Eigen::Vector3f newDirection);
	FlightController();
	void run();
	void setReferenceRotationalVel(Eigen::Vector3f newRefAtt);
	//setHoldPosition(Eigen::Vector3f newPosition);
};

void log(std::string message);
#endif //FLIGHT_CONTROLLER_HPP_
 
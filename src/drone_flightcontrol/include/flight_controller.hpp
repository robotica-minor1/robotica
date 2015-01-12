#ifndef FLIGHT_CONTROLLER_HPP_
#define FLIGHT_CONTROLLER_HPP_
#include <Eigen/Dense>
#include <map>
#include <string> 

class FlightController {
private:
	/*
		actuate takes reference thrust axis rotation and attitude, and makes it happen.
	*/
	void actuate(Eigen::Vector4f thrustPerEngine, Eigen::Vector4f axisRotation, Eigen::Vector4f referenceAttitude);
	Eigen::Vector3f direction;
	std::map<string,int> pidGains;
	Eigen::Vector3f landingPosition;
	fc_config config; 
	string navMode; 
	
public: 
	/*
		setDirection Takes a reference direction and converts it to actuate settings. 
	*/
	setDirection(Eigen::Vector3f newDirection);
	FlightController();
	//setHoldPosition(Eigen::Vector3f newPosition);
};
#endif //FLIGHT_CONTROLLER_HPP_
 
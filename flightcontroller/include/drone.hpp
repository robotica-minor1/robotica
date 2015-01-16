#ifndef DRONE_HPP_
#define DRONE_HPP_
#include <Eigen/Dense>
// #include "VarSpeedServo.h"
class Drone {
private:
	//servo settings
	int servoPins[4] = {3, 6, 9, 10};
	// VarSpeedServo servos[4];
	int servoSpeeds[4] = {servoMaxSpeed, servoMaxSpeed, servoMaxSpeed, servoMaxSpeed};
	int servoMinSpeed = 0; 
	int servoMaxSpeed = 127;
	int servoStartSpeed = 127;
	
	int servoMinAngle = 0;
	int servoMaxAngle = 90;
	int servoMinPulse = 1000;
	int servoMaxPulse = 2000;

	int servoOffset24 = 20;
	int servoOffset13 = 70;


	void setupServos();
	void updateReferenceThrust(float gain, int signs[4]);
public: 
	Drone();
	void setAxisRotation(Eigen::Vector4f axisAngles);
	void setThrust(Eigen::Vector4f thrust);
	//drag coefficient(assumed)
	float dragCoefficient = 0.5;

	//body surface area(assumed)
	float surfaceArea = 0.06;

	Eigen::Vector3f position;
	Eigen::Vector3f landingSpot;
	float distanceToLandingSpot;
	Eigen::Vector3f holdPosition;
	int motorRotationSigns[4] = {-1, 1, -1, 1};
	Eigen::Vector3f velocity;
	Eigen::Vector3f acceleration;

};

#endif //DRONE_HPP_
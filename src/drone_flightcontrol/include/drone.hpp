#ifndef DRONE_HPP_
#define DRONE_HPP_
#include <Eigen/dense>
#include "VarSpeedServo.h"
class Drone {
private:
	setupServos();
public: 
	Drone();
	void setAxisRotation(Eigen::Vector4f axisAngles);

	//drag coefficient(assumed)
	float dragCoefficient = 0.5;

	//body surface area(assumed)
	float surfaceArea = 0.06;

	//servo settings
	int servoPins[4] = {3, 6, 9, 10}
	VarSpeedServo servos[4];
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
};

#endif //DRONE_HPP_
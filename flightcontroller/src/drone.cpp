#include <cmath> 

#include "drone.hpp"
#include "arduino.hpp"

// Defined in imu.cpp
extern long micros();

arduino& Arduino = arduino::get();

Drone::Drone() {
	
}

Drone& Drone::get() {
    static Drone instance;
    return instance;
}

void Drone::setAxisRotation(Eigen::Vector4f axisAngles) {
    Arduino.set_servos(axisAngles, Eigen::Vector4f::Ones(1) * 40);
}

void Drone::setThrust(Eigen::Vector4f thrust) {
	//labels on the actual prop engines don't match the conventions 
	//previously used in the code, so convert it here.
	Eigen::Vector4f converted;
	converted[0] = thrust[2];
	converted[1] = thrust[3];
	converted[2] = thrust[1];
	converted[3] = thrust[0];
	Arduino.set_props(converted);
}


void Drone::setRetracts(bool up) {
    Arduino.set_retracts(up);
}

float Drone::getHeight() {
    float height = Arduino.poll_sonar() / 100.0f;

    float dt = (micros() - last_height_sample) / 1000000.0f;
    speed = (height - lastHeight) / dt;
    lastHeight = height;
    last_height_sample = micros();

    return height;
}

float Drone::getZSpeed() {
    return speed;
}

Eigen::Vector3f Drone::getPosition() {
	return Eigen::Vector3f::Zero(3);
}
#include <cmath> 

#include "drone.hpp"
#include "arduino.hpp"

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
	Arduino.set_props(thrust);
}


void Drone::setRetracts(bool up) {
    Arduino.set_retracts(up);
}

float Drone::getHeight() {
    return Arduino.poll_sonar() / 100.0f;
}

Eigen::Vector3f Drone::getPosition() {
	return Eigen::Vector3f::Zero(3);
}
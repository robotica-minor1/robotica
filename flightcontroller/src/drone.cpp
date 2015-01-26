#include <cmath> 

#include "drone.hpp"
#include "arduino.hpp"
Drone::Drone() {
	
}

Drone& Drone::get() {
    static Drone instance;
    return instance;
}

void Drone::setAxisRotation(Eigen::Vector4f axisAngles) {
    arduino::get().set_servos(axisAngles, Eigen::Vector4f::Ones(1) * 40);
}

void Drone::setThrust(Eigen::Vector4f thrust) {
	arduino::get().set_props(thrust);
}


void Drone::setRetracts(bool up) {
    arduino::get().set_retracts(up);
}

float Drone::getHeight() {
    return arduino::get().poll_sonar() / 100.0f;
}

Eigen::Vector3f Drone::getPosition() {
	return Eigen::Vector3f::Zero(3);
}
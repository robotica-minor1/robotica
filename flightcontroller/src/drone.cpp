#include <cmath> 

#include "drone.hpp"
#include "io.hpp"

// #include "VarSpeedServo.h"
Drone::Drone() {
	
}

Drone::~Drone() {

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

int Drone::getHeight() {
    return arduino::get().poll_sonar();
}
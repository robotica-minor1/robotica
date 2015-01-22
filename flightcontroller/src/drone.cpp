#include <cmath> 

#include "drone.hpp"
// #include "VarSpeedServo.h"
Drone::Drone() {
	
}

Drone& Drone::get() {
    static Drone instance;
    return instance;
}

void Drone::setAxisRotation(Eigen::Vector4f axisAngles) {

}
void Drone::setThrust(Eigen::Vector4f thrust) {
	
}

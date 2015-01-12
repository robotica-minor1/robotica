#include drone.hpp
Drone::Drone() {
	setupServos();
}

Drone::setupServos() {
	servos[0].write(servoMinPulse+((servoMinPulse/(servoMaxAngle))*servoOffset13), servoStartSpeed);
	servos[1].write(servoMinPulse+((servoMinPulse/(servoMaxAngle))*servoOffset24), servoStartSpeed); 
	servos[2].write(servoMinPulse+((servoMinPulse/(servoMaxAngle))*servoOffset13), servoStartSpeed);
	servos[3].write(servoMinPulse+((servoMinPulse/(servoMaxAngle))*servoOffset24), servoStartSpeed);
}

void Drone::setAxisRotation(Eigen::Vector4f axisAngles) {

	//dirty hack to prevent odd behavior by the servos
	for (int i = 0; i < 4; i++) {
		drone.servos[i].attach(drone.servopins[i], servoMinPulse, servoMaxPulse);
	}

	//compensate for the difference between desired angle and the angle 'known' by the servo
	axisAngles[0] = std::min(std::max(axisAngles[0], servoMinAngle-(servoMaxAngle-servoOffset13)), servoOffset13);
	axisAngles[2] = std::min(std::max(axisAngles[0], servoMinAngle-(servoMaxAngle-servoOffset13)), servoOffset13);

	axisAngles[1] = std::min(std::max(axisAngles[1], servoMinAngle-servoOffset24), servoMaxAngle-servoOffset24);
	axisAngles[3] = std::min(std::max(axisAngles[3], servoMinAngle-servoOffset24), servoMaxAngle-servoOffset24);

	//write the new angles to the servos
	servos[0].write(((axisAngles[0]*-1+servoOffset13)*2), servoSpeeds[0]);
	servos[2].write(((axisAngles[2]*-1+servoOffset13)*2), servoSpeeds[2]);
	
	servos[1].write(((axisAngles[1]*+servoOffset24)*2), servoSpeeds[1]);
	servos[3].write(((axisAngles[3]*+servoOffset24)*2), servoSpeeds[3]);
}




#include <Eigen/Dense>
#include <map>
#include <string> 
#include <cmath>
#include <iostream>

#include "fc_constants.hpp"
#include "fc_config.hpp"
#include "flight_controller.hpp"
#include "drone.hpp"
#include "imu.hpp"
void FlightController::run() {
	while(true) {
		Eigen::Vector3f diffAtt = getDifferenceAttitude();
	}
}

Eigen::Vector3f FlightController::getDifferenceAttitude() {
	return drone.referenceAttitude - imu::get().get_angles();
}

void FlightController::setReferenceAttitude(Eigen::Vector3f newRefAtt) {
	drone.referenceAttitude = newRefAtt;
}


FlightController::FlightController() {
	pidGains["Roll"] = 0.0;
	pidGains["Pitch"] = 0.0;
	pidGains["Heading"] = 0.0;
	pidGains["Height"] = 0.0;
	navMode = "Hold";
}

void FlightController::setHoldPosition(Eigen::Vector3f newPosition) {
	drone.holdPosition = newPosition; 
	if(drone.holdPosition[2] < fc_config::safetyHeight) {
		drone.holdPosition[2] = fc_config::safetyHeight; 
	}
}

void FlightController::navigate() {
/*	if (navMode == "Waypoint") {
		wayPointNavigation(); 
	} else if (navMode == "Direct") {
		directControl();
	} else if (navMode == "Follow" && target) {
		directControl();
	} else if (navMode == "Hold") {
		directControl();
	} else if (navMode == "Land") {
		directControl();
	} */
}

void FlightController::updateReferenceThrust(float gain, int signs[]) {
	for (int i = 0; i < 4; i++) {
		if (signs[i] != 1 && signs[i] != -1) {
			log("Incorrect signs for updateReferenceThrust!");
		}
	}

	for (int i = 0; i < 4; i++) {
		referenceThrust[i] *= (1 + gain * signs[i]);
	}
}

void FlightController::headingPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity){
 	//get PID values
	float KP = fc_config::pidHeading[0];
	float KD = fc_config::pidHeading[1];

	//calculate gain
	float gain = diffAtt[2] * KP + diffRotationalVelocity[2] * KD;

	gain *= fc_config::masterGain;
	pidGains["Heading"] = gain;

	//update reference thrust
	if(gain > 0 && diffRotationalVelocity[2] <= fc_config::maxYawRotationalVel) {
		log("Turn-R");
		updateReferenceThrust(gain, drone.motorRotationSigns);
	} else if (gain < 0 && diffRotationalVelocity[2] >= -fc_config::maxYawRotationalVel) {
		log("Turn-L");
		updateReferenceThrust(gain, drone.motorRotationSigns);
	} else {
		log("Heading out of bounds set by maxYawRotationalVel");
	}
}


void FlightController::rollPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity){
	//get PID values
	float KP = fc_config::pidRoll[0];
	float KD = fc_config::pidRoll[1];

	//calculate gain
	float gain = diffAtt[0] * KP + diffRotationalVelocity[0] * KD;

	gain *= fc_config::masterGain;
	pidGains["Roll"] = gain;
	
	//update reference thrust
	int signs[4] = {1, -1, -1, 1};
	if(gain > 0 && diffRotationalVelocity[0] <= fc_config::maxRollRotationalVel) {
		log("Roll-R");
		updateReferenceThrust(gain, signs);
	} else if (gain < 0 && diffRotationalVelocity[0] >= -fc_config::maxRollRotationalVel) {
		log("Roll-L");
		updateReferenceThrust(gain, signs);
	} else {
		log("Roll out of bounds set by maxRollRotationalVel");
	}
}


void FlightController::pitchPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity){
	//get PID values
	float KP = fc_config::pidPitch[0];
	float KD = fc_config::pidPitch[1];

	//calculate gain
	float gain = diffAtt[1] * KP + diffRotationalVelocity[1] * KD;

	gain *= fc_config::masterGain;
	pidGains["Pitch"] = gain;
	
	//update reference thrust
	int signs[4] = {-1, -1, 1, 1};
	if (gain > 0 && diffRotationalVelocity[1] <= fc_config::maxPitchRotationalVel) {
		log("Forward");
		updateReferenceThrust(gain, signs);
	} else if (gain < 0 && diffRotationalVelocity[1] >= -fc_config::maxPitchRotationalVel) {
		log("Backward");
		updateReferenceThrust(gain, signs);
	} else {
		log("Pitch out of bounds set by maxPitchRotationalVel");
	}
}

void FlightController::heightPID(Eigen::Vector3f absoluteDirection, Eigen::Vector3f differenceVelocity){
	//get PID values
	float KP = fc_config::pidHeight[0];
	float KD = fc_config::pidHeight[1];

	//calculate gain
	float gain = absoluteDirection[2] * KP + differenceVelocity[2] * KD;

	//check whether or not we're above the safety height
	if (drone.position[2] < fc_config::safetyHeight && drone.distanceToLandingSpot > fc_config::landingPrecision) {
		log("Below safety height!");
		gain = KP * (fc_config::safetyHeight - drone.position[2]);
	}

	gain *= fc_config::masterGain;
	pidGains["Height"] = gain;

	//update reference thrust
	int signs[4] = {1, 1, 1, 1};
	if (drone.velocity[2] <= -fc_config::maxDownSpeed || 
		drone.acceleration[2] <= -fc_config::maxDownAcceleration && 
		drone.velocity[2] < 0) {
		log("Moving down too fast");

		gain = fabs(gain);
		updateReferenceThrust(gain, signs);
	} else if (drone.velocity[2] <= fc_config::maxUpSpeed) {
		if (gain > 0) {
			log("Up");
			updateReferenceThrust(gain, signs);
		} else if (gain < 0 && drone.velocity[2] >= -fc_config::maxDownSpeed) {
			log("Down");
			updateReferenceThrust(gain, signs);
		}
	}

}

void log(std::string message) {
	// std::cout << message << std::endl; 
}

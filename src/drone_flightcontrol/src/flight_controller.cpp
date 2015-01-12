#include <Eigen/Dense>
#include <map>
#include <string> 
#include <cmath>

#include "fc_constants.hpp"
#include "fc_config.hpp"
#include "flight_controller.hpp"
#include "drone.hpp"

FlightController::FlightController() {
	pidGains["Roll"] = 0.0;
	pidGains["Pitch"] = 0.0;
	pidGains["Heading"] = 0.0;
	pidGains["Height"] = 0.0;
	navMode = "Hold";
}

FlightController::setHoldPosition(Eigen::Vector3f newPosition) {
	holdPosition = newPosition; 
	if(holdPosition[2] < config.safetyHeight) {
		holdPosition[2] = config.safetyHeight; 
	}
}

FlightController::navigate() {
	if (navMode == "Waypoint") {
		wayPointNavigation(); 
	} else if (navMode == "Direct") {
		directControl();
	} else if (navMode == "Follow" && target) {
		directControl();
	} else if (navMode == "Hold") {
		directControl();
	} else if (navMode == "Land") {
		directControl();
	} 
}
void Drone::updateReferenceThrust(float gain, int signs[4]) {
	for (int i = 0; i < 4; i++) {
		if (signs[i] != 1 && signs[i] != -1) {
			log("Incorrect signs for setReferenceThrust!");
		}
	}

	for (int i = 0; i < 4; i++) {
		referenceThrust[i] *= (1 + gain * signs[i]);
	}
}

 void Drone::headingPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity){
 	//get PID values
	float KP = config.pidHeading[0];
	float KD = config.pidHeading[1];

	//calculate gain
	float gain = diffAtt[2] * KP + diffRotationalVelocity[2] * KD;

	gain *= config.masterGain;
	pidGains["Heading"] = gain;

	//update reference thrust
	if(gain > 0 && diffRotationalVelocity[2] <= config.maxYawRotationalVel) {
		log("Turn-R");
		setReferenceThrust(gain, motorRotationSigns);
	} else if (gain < 0 && diffRotationalVelocity[2] >= -config.maxYawRotationalVel) {
		log("Turn-L");
		setReferenceThrust(gain, motorRotationSigns);
	} else {
		log("Heading out of bounds set by maxYawRotationalVel");
	}
}


void Drone::rollPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity){
	//get PID values
	float KP = config.pidRoll[0];
	float KD = config.pidRoll[1];

	//calculate gain
	float gain = diffAtt[0] * KP + diffRotationalVelocity[0] * KD;

	gain *= config.masterGain;
	pidGains["Roll"] = gain;
	
	//update reference thrust
	if(gain > 0 && diffRotationalVelocity[0] <= config.maxRollRotationalVel) {
		log("Roll-R");
		setReferenceThrust(gain, {1, -1, -1, 1});
	} else if (gain < 0 && diffRotationalVelocity[0] >= -config.maxRollRotationalVel) {
		log("Roll-L");
		setReferenceThrust(gain, {1, -1, -1, 1});
	} else {
		log("Roll out of bounds set by maxRollRotationalVel");
	}
}


void Drone::pitchPID(Eigen::Vector3f diffAtt, Eigen::Vector3f diffRotationalVelocity){
	//get PID values
	float KP = config.pidPitch[0];
	float KD = config.pidPitch[1];

	//calculate gain
	float gain = diffAtt[1] * KP + diffRotationalVelocity[1] * KD;

	gain *= config.masterGain;
	pidGains["Pitch"] = gain;
	
	//update reference thrust
	if(gain > 0 && diffRotationalVelocity[1] <= config.maxPitchRotationalVel) {
		log("Forward");
		setReferenceThrust(gain, {-1, -1, 1, 1});
	} else if (gain < 0 && diffRotationalVelocity[1] >= -config.maxPitchRotationalVel) {
		log("Backward");
		setReferenceThrust(gain, {-1, -1, 1, 1});
	} else {
		log("Pitch out of bounds set by maxPitchRotationalVel");
	}
}
void Drone::heightPID(Eigen::Vector3f absoluteDirection, Eigen::Vector3f differenceVelocity){
	//get PID values
	float KP = config.pidHeight[0];
	float KD = config.pidHeight[1];

	//calculate gain
	float gain = absoluteDirection[2] * KP + differenceVelocity[2] * KD;

	//check whether or not we're above the safety height
	if (drone.position[2] < config.safetyHeight && drone.distanceToLandingSpot > config.landingPrecision) {
		log("Below safety height!");
		gain = KP * (config.safetyHeight - drone.position[2]);
	}

	gain *= config.masterGain;
	pidGains["Height"] = gain;

	//update reference thrust
	if (drone.velocity[2] <= -config.maxDownSpeed || 
		drone.acceleration[2] <= -config.maxDownAcceleration && 
		drone.velocity < 0) {
		log("Moving down too fast");

		gain = fabs(gain);
		updateReferenceThrust(gain, {1, 1, 1, 1});
	} else if (drone.velocity[2] <= config.maxUpSpeed) {
		if (gain > 0) {
			log("Up");
			updateReferenceThrust(gain, {1, 1, 1, 1});
		} else if (gain < 0 && drone.velocity[2] => -config.maxDownSpeed) {
			log("Down");
			updateReferenceThrust(gain, {1, 1, 1, 1});
		}
	}

}

void log(string message) {
	std::cout << message << std::endl; 
}

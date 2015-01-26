#ifndef FC_CONFIG_HPP
#define FC_CONFIG_HPP
#include <cmath> 

namespace fc_config {
	/*
		Control settings
	*/
	//maximum speed while on manual control. [m/s]
	float maxSpeedInput; 
	
	//Maximum rotational velocity on yaw (manual control only) [Degrees/s]
	float maxYawRotation; 


	/*
		Navigation settings
	*/

	//True - land. False - hold. 
	// bool landAfterFlightpath;

	//When this close to target, target has been 'reached' [m]
	// float proximityTreshold;

	//maximum speed to descend while landing [m/s]
	// float landDescentSpeed;

	//while landing, drone is allowed to land this far from ideal landing spot, 
	//it will still attempt to get as close as possible. [m]
	// float landingPrecision;

	//Only look north. debug purposes
	// bool fixHeading;

	//prevent the drone from flying sideways
	// bool forceForwardFlight; 

	//drone may only move when its heading is between the target heading +- this precision. [degrees]
	// float safeHeadingPrecision;

	/*
		Safety settings
	*/

	//drone will attempt to stay above this height while in flight. [m]
	float safetyHeight = 3;

	//maximum allowed descent speed. [m/s]
	float maxDownSpeed = 1.5;

	//maximum allowed ascent speed. [m/s]
	float maxUpSpeed = 3;

	//maximum allowed downward acceleration. [m/s^2]
	float maxDownAcceleration = 1;

	//maximum allowed speed [m/s]
	float maxSpeed = 5;

	//Maximum allowable pitch. [radians]
	// float maxPitch;

	//Maximum allowable yaw. [radians]
	// float maxYaw;

	//maximum allowable rotational velocity [radians]
	float maxPitchRotationalVel = 20 * DEG_TO_RAD;
	float maxRollRotationalVel = 20 * DEG_TO_RAD;
	float maxYawRotationalVel = 120 * DEG_TO_RAD;
	//PID values. P and D only. 
	float pidHeading[2] = {0.1, 0.1};
	float pidRoll[2] = {0.15, 0.2};
	float pidPitch[2] = {0.15, 0.2};
	float pidHeight[2] = {0.16, 0.2};
	float masterGain = 1;
}

#endif //FC_CONFIG_HPP
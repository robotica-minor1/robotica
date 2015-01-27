#ifndef FC_CONFIG_HPP
#define FC_CONFIG_HPP
#include <cmath> 

namespace fc_config {
	/*
		Control settings
	*/
	//maximum speed while on manual control. [m/s]
	// const float maxSpeedInput; 
	
	//Maximum rotational velocity on yaw (manual control only) [Degrees/s]
	// const float maxYawRotation; 


	/*
		Navigation settings
	*/

	//True - land. False - hold. 
	// bool landAfterFlightpath;

	//When this close to target, target has been 'reached' [m]
	// const float proximityTreshold;

	//maximum speed to descend while landing [m/s]
	// const float landDescentSpeed;

	//while landing, drone is allowed to land this far from ideal landing spot, 
	//it will still attempt to get as close as possible. [m]
	const float landingPrecision = 1;

	//Only look north. debug purposes
	// bool fixHeading;

	//prevent the drone from flying sideways
	// bool forceForwardFlight; 

	//drone may only move when its heading is between the target heading +- this precision. [degrees]
	// const float safeHeadingPrecision;

	/*
		Safety settings
	*/

	//drone will attempt to stay above this height while in flight. [m]
	const float safetyHeight = 0.5;

	//maximum allowed descent speed. [m/s]
	const float maxDownSpeed = 1.5;

	//maximum allowed ascent speed. [m/s]
	const float maxUpSpeed = 3;

	//maximum allowed downward acceleration. [m/s^2]
	const float maxDownAcceleration = 1;

	//maximum allowed speed [m/s]
	const float maxSpeed = 5;

	//Maximum allowable pitch. [radians]
	// const float maxPitch;

	//Maximum allowable yaw. [radians]
	// const float maxYaw;

	//maximum allowable rotational velocity [radians]
	const float maxPitchRotationalVel = 20 * DEG_TO_RAD;
	const float maxRollRotationalVel = 20 * DEG_TO_RAD;
	const float maxYawRotationalVel = 120 * DEG_TO_RAD;
	//PID values. P and D only. 
	const float pidHeading[2] = {0.1, 0.01};
	const float pidRoll[2] = {0.15, 0.02};
	const float pidPitch[2] = {0.15, 0.02};
	const float pidHeight[2] = {0.16, 0.02};
	const float masterGain = 1;

	const float MAX_THRUST = 10200;
	const float MIN_THRUST = 700;

}

#endif //FC_CONFIG_HPP
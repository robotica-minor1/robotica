#ifndef FC_CONFIG_HPP
#define FC_CONFIG_HPP
#include <cmath> 

namespace fc_config {
	/*
		Control settings
	*/
	//maximum speed while on manual control. [m/s]
	static float maxSpeedInput; 
	
	//Maximum rotational velocity on yaw (manual control only) [Degrees/s]
	static float maxYawRotation; 


	/*
		Navigation settings
	*/

	//True - land. False - hold. 
	static bool landAfterFlightpath;

	//When this close to target, target has been 'reached' [m]
	static float proximityTreshold;

	//maximum speed to descend while landing [m/s]
	static float landDescentSpeed;

	//while landing, drone is allowed to land this far from ideal landing spot, 
	//it will still attempt to get as close as possible. [m]
	static float landingPrecision;

	//Only look north. debug purposes
	static bool fixHeading;

	//prevent the drone from flying sideways
	static bool forceForwardFlight; 

	//drone may only move when its heading is between the target heading +- this precision. [degrees]
	static float safeHeadingPrecision;

	/*
		Safety settings
	*/

	//drone will attempt to stay above this height while in flight. [m]
	static float safetyHeight = 3;

	//maximum allowed descent speed. [m/s]
	static float maxDownSpeed = 1.5;

	//maximum allowed ascent speed. [m/s]
	static float maxUpSpeed = 3;

	//maximum allowed downward acceleration. [m/s^2]
	static float maxDownAcceleration = 1;

	//maximum allowed speed [m/s]
	static float maxSpeed = 5;

	//Maximum allowable pitch. [radians]
	static float maxPitch;

	//Maximum allowable yaw. [radians]
	static float maxYaw;

	//maximum allowable rotational velocity [radians]
	static float maxPitchRotationalVel;
	static float maxRollRotationalVel;
	static float maxYawRotationalVel;
	//PID values. P and D only. 
	static float pidHeading[2];
	static float pidRoll[2];
	static float pidPitch[2];
	static float pidHeight[2];
	static float masterGain = 1;
}

#endif //FC_CONFIG_HPP
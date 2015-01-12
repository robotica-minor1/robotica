#ifndef FC_CONFIG_HPP
#define FC_CONFIG_HPP
#include <string>

class fc_config {
public:
	fc_config(std::string configname);
	

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
	bool landAfterFlightpath;

	//When this close to target, target has been 'reached' [m]
	float proximityTreshold;

	//maximum speed to descend while landing [m/s]
	float landDescentSpeed;

	//while landing, drone is allowed to land this far from ideal landing spot, 
	//it will still attempt to get as close as possible. [m]
	float landingPrecision;

	//Only look north. debug purposes
	bool fixHeading;

	//prevent the drone from flying sideways
	bool forceForwardFlight; 

	//drone may only move when its heading is between the target heading +- this precision. [degrees]
	float safeHeadingPrecision;

	/*
		Safety settings
	*/

	//drone will attempt to stay above this height while in flight. [m]
	float safetyHeight;

	//maximum allowed ascent speed. [m/s]
	float maxDownSpeed;

	//maximum allowed descent speed. [m/s]
	float maxUpspeed;

	//maximum allowed downward acceleration. [m/s^2]
	float maxDownAcceleration

	//maximum allowed speed [m/s]
	float maxSpeed;

	//Maximum allowable pitch. [radians]
	float maxPitch;

	//Maximum allowable yaw. [radians]
	float maxYaw;

	//maximum allowable rotational velocity [radians]
	float maxPitchRotationalVel;
	float maxRollRotationalVel;
	float maxYawRotationalVel;
	//PID values. P and D only. 
	float pidHeading[2];
	float pidRoll[2];
	float pidPitch[2];
	float pidHeight[2];


};




#endif //FC_CONFIG_HPP
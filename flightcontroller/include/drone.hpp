#ifndef DRONE_HPP_
#define DRONE_HPP_
#include <Eigen/Dense>

// Defined in imu.cpp
extern long micros();

struct HeightSample {
	long t;
	float h;

	HeightSample() : t(0), h(0) {}

	HeightSample(float h) : h(h) {
		t = micros();
	}
};

class Drone {
private:
	void updateReferenceThrust(float gain, int signs[4]);
	Drone();
	Drone(Drone const&) = delete;
    void operator=(Drone const&) = delete;
public: 
	void setAxisRotation(Eigen::Vector4f axisAngles);
	void setThrust(Eigen::Vector4f thrust);
	void setRetracts(bool up);
	Eigen::Vector3f getPosition(); 
	static Drone& get();
	float getHeight();
	float getZSpeed();

	float speed = 0;
	float lastHeight = 0;
	long last_height_sample = 0;

	//drag coefficient(assumed)
	float dragCoefficient = 0.5;
	//body surface area(assumed)
	float surfaceArea = 0.06;
	Eigen::Vector3f position;
	Eigen::Vector3f landingSpot;
	float distanceToLandingSpot;
	Eigen::Vector3f holdPosition;
	int motorRotationSigns[4] = {-1, 1, -1, 1};
	Eigen::Vector3f referenceAttitude;	
	Eigen::Vector3f referenceRotationalVel;	
	Eigen::Vector3f referenceVelocity;	
	Eigen::Vector3f referencePosition; 
	bool gearUp; 
	float gearRaiseHeight = 0.7;
	float gearLowerHeight = 0.5;

	float defaultThrust = 9500;
	Eigen::Vector4f t00 = Eigen::Vector4f(defaultThrust, defaultThrust, defaultThrust, defaultThrust);

	// Circular buffer of height samples
	HeightSample queue[30];
	int queueIdx = 0;
	int queueCount = 0;
};

#endif //DRONE_HPP_
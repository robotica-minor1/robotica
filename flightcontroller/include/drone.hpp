#ifndef DRONE_HPP_
#define DRONE_HPP_
#include <Eigen/Dense>

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
	static Drone& get();
	int getHeight();

	//drag coefficient(assumed)
	float dragCoefficient = 0.5;
	float defaultThrust = 9500;
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


};

#endif //DRONE_HPP_
#ifndef DRONE_HPP_
#define DRONE_HPP_
#include <Eigen/Dense>

class Drone {
private:
	void updateReferenceThrust(float gain, int signs[4]);
public: 
	Drone();
	~Drone();

	void setAxisRotation(Eigen::Vector4f axisAngles);
	void setThrust(Eigen::Vector4f thrust);
	void setRetracts(bool up);

	int getHeight();

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


};

#endif //DRONE_HPP_
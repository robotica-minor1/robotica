#ifndef FLIGHTCONTROLLER_HPP_
#define FLIGHTCONTROLLER_HPP_
#include <ros/ros.h>
class Flightcontroller {
private:
	ros::Publisher publisher;
public: 
	Flightcontroller(ros::Publisher pub);
	void translate(Eigen::Vector3f direction);
	//void rotate();
};
inline Flightcontroller::Flightcontroller(ros::Publisher pub) : publisher(pub){};
#endif //FLIGHTCONTROLLER_HPP_

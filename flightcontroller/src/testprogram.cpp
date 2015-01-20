#include "flight_controller.hpp"

int main(int argc, char* argv[]) {
	FlightController fc; 
	fc.setReferenceAttitude(Eigen::Vector3f(0,0,0));
}

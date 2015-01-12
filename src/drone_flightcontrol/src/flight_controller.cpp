#include <Eigen/Dense>
#include <map>
#include <string> 

#include "fc_constants.hpp"
#include "fc_config.hpp"
#include "flight_controller.hpp"

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

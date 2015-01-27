#include <cmath>
#include <iostream>

#include "fc_config.hpp"
#include "drone.hpp"
#include "arduino.hpp"

arduino& Arduino = arduino::get();

Drone::Drone() {
	
}

Drone& Drone::get() {
    static Drone instance;
    return instance;
}

void Drone::setAxisRotation(Eigen::Vector4f axisAngles) {
    Arduino.set_servos(axisAngles, Eigen::Vector4f::Ones(1) * 40);
}

void Drone::setThrust(Eigen::Vector4f thrust) {
	//labels on the actual prop engines don't match the conventions 
	//previously used in the code, so convert it here.
	Eigen::Vector4f converted;
	converted[0] = thrust[2];
	converted[1] = thrust[3];
	converted[2] = thrust[1];
	converted[3] = thrust[0];

    for (int i = 0; i < 4; i++) {
        if(converted[i] > fc_config::MAX_THRUST) {
            converted[i] = fc_config::MAX_THRUST;
        } else if (converted[i] < fc_config::MIN_THRUST) {
            converted[i] = fc_config::MIN_THRUST;
        }
    }

	Arduino.set_props(converted);
}


void Drone::setRetracts(bool up) {
    Arduino.set_retracts(up);
}

float Drone::getHeight() {
    float height = Arduino.poll_sonar() / 100.0f;

    queue[queueIdx] = HeightSample(height);
    queueIdx = (queueIdx + 1) % 30;
    queueCount = std::min(queueCount + 1, 30);

    return height;
}

float Drone::getZSpeed() {
    if (queueCount < 30) {
        return 0;
    } else {
        float speedSum = 0;

        for (int i = 0; i < 30 - 1; i++) {
            // Start of circular buffer (1 after last inserted item)
            int o = (queueIdx + i) % 10;

            float dt = (queue[o + 1].t - queue[o].t) / 1000000.0f;
            float dist = queue[o + 1].h - queue[o].h;

            speedSum += dist / dt;
        }

        return speedSum / 30.0f;
    }
}

Eigen::Vector3f Drone::getPosition() {
	return Eigen::Vector3f::Zero(3);
}
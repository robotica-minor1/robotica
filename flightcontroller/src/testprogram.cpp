#include "arduino.hpp"
#include <iostream>
#include "flight_controller.hpp"
#include "drone.hpp"
#include <Eigen/Dense>

int main(int argc, char* argv[]) {
    std::cout << arduino::get().poll_sonar() << std::endl;
    FlightController fc; 
    Eigen::Vector4f angles(0,0,0,0);
    Drone::get().setAxisRotation(angles);
    sleep(2);
    fc.run();

    return 0;
}

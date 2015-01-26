#include "arduino.hpp"
#include <iostream>
#include "flight_controller.hpp"

int main(int argc, char* argv[]) {
    std::cout << arduino::get().poll_sonar() << std::endl;
    FlightController fc; 
    fc.run();

    return 0;
}

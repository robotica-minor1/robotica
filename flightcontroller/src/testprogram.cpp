#include "arduino.hpp"
#include <iostream>

int main(int argc, char* argv[]) {
    std::cout << arduino::get().poll_sonar() << std::endl;

    return 0;
}

#include "imu.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char* argv[]) {
    setbuf(stdout, NULL);

    while (true) {
        auto acc = imu::get().get_pos();

        printf("%6.2f, %6.2f, %6.2f\r", acc[0], acc[1], acc[2]);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}

include_directories(../include ../lib)
add_executable(flightController.o flight_controller.cpp drone.cpp testprogram.cpp imu.cpp i2c.cpp)

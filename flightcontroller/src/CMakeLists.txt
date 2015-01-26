include_directories(../include ../lib)
add_executable(flightController testprogram.cpp imu.cpp i2c.cpp arduino.cpp flight_controller.cpp drone.cpp)
target_link_libraries(flightController pthread)
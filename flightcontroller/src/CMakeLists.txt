include_directories(../include ../lib)
add_executable(flightController.out testprogram.cpp imu.cpp i2c.cpp arduino.cpp flight_controller.cpp drone.cpp)
target_link_libraries(flightController.out pthread)

include_directories(../include ../lib)
add_executable(flightController.o testprogram.cpp imu.cpp i2c.cpp)
target_link_libraries(flightController.o pthread)
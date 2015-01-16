cmake_minimum_required(VERSION 2.8.3)
add_definitions (-std=c++11)
project(drone_flightcontrol)

add_subdirectory(src)
include_directories(lib include)


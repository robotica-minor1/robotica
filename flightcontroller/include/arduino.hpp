#ifndef ARDUINO_HPP
#define ARDUINO_HPP

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <Eigen/Dense>

class arduino {
public:
    static arduino& get();

    void set_servos(Eigen::Vector4f axes = Eigen::Vector4f::Zero(), Eigen::Vector4f speeds = Eigen::Vector4f::Zero());
    void set_props(Eigen::Vector4f pwms = Eigen::Vector4f::Zero());
    void set_retracts(bool up = false);

    int poll_sonar(); // Distance to ground in cm

private:
    int sockfd;

    arduino();
    ~arduino();

    void writeline(const std::string& msg);
    std::string readline();
    
    arduino(arduino const&) = delete;
    void operator=(arduino const&) = delete;
};

#endif
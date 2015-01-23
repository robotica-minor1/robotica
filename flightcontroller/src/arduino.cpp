#include "arduino.hpp"
#include <iostream>

arduino& arduino::get() {
    static arduino instance;
    return instance;
}

arduino::arduino() {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    struct hostent* server = gethostbyname("localhost");

    struct sockaddr_in serv_addr;
    bzero((char*) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char*) server->h_addr,  (char*) &serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(20000);

    if (connect(sockfd, (struct sockaddr*) &serv_addr,sizeof(serv_addr)) < 0) {
        std::cerr << "error: can't connect to serial proxy" << std::endl;
        return;
    }
}

arduino::~arduino() {
    close(sockfd);
}

void arduino::writeline(const std::string& msg) {
    std::string _msg = msg + "\n";
    write(sockfd, _msg.c_str(), _msg.size());
}

std::string arduino::readline() {
    char buffer[256];
    bzero(buffer, 256);
    
    int n = read(sockfd, buffer, 255);
    buffer[n] = 0;

    std::string res = buffer;
    res.erase(res.find_last_not_of(" \n\r\t") + 1);
    return res;
}

void arduino::set_servos(Eigen::Vector4f axes, Eigen::Vector4f speeds) {
    static char buf[256];

    sprintf(
        buf,
        "servos %d %d %d %d %d %d %d %d",
        (int) axes[0],
        (int) axes[1],
        (int) axes[2],
        (int) axes[3],
        (int) speeds[0],
        (int) speeds[1],
        (int) speeds[2],
        (int) speeds[3]
    );

    writeline(buf);
    readline();
}

void arduino::set_props(Eigen::Vector4f pwms) {
    static char buf[256];

    sprintf(
        buf,
        "propsraw %d %d %d %d",
        (int) pwms[0],
        (int) pwms[1],
        (int) pwms[2],
        (int) pwms[3]
    );

    writeline(buf);
    readline();
}

void arduino::set_retracts(bool up) {
    static char buf[256];

    sprintf(
        buf,
        "retracts %d",
        up ? 1 : 0
    );

    writeline(buf);
    readline();
}

int arduino::poll_sonar() {
    writeline("pollsonar");
    auto response = readline();

    int cm;
    sscanf(response.c_str(), "sonar %d", &cm);

    return cm;
}
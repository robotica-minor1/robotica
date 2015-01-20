#ifndef IMU_HPP
#define IMU_HPP

#include "kalman.hpp"
#include <stdint.h>
#include <Eigen/Dense>
/*
struct angles {
    float pitch, yaw, roll;

    angles(float pitch = 0, float yaw = 0, float roll = 0) : pitch(pitch), yaw(yaw), roll(roll) {};
};

struct vec3 {
    float x, y, z;

    vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};*/

// Yaw is unreliable in the first ~1000 ms
class imu {
public:
    static imu& get();

    void poll();

    Eigen::Vector3f get_angles() const;
    Eigen::Vector3f get_acceleration() const;
    Eigen::Vector3f get_speed() const;

    float get_temperature() const;

private:
    Eigen::Vector3f acc, speed, ang;

    Kalman kalmanX, kalmanY;

    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    int16_t tempRaw;

    bool calibrated;
    int start_t;
    double yawOffset;

    double gyroXangle, gyroYangle, gyroZangle;
    double compAngleX, compAngleY;
    double kalAngleX, kalAngleY;

    uint32_t timer;
    uint8_t i2cData[14];

    imu();

    imu(imu const&) {};
    void operator=(imu const&) {};
};

#endif
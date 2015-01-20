#ifndef IMU_HPP
#define IMU_HPP

#include "kalman.hpp"
#include <stdint.h>
#include <Eigen/Dense>


// Yaw is unreliable in the first ~1000 ms
class imu {
public:
    static imu& get();

    void poll();

    Eigen::Vector3f get_angles() const;
    Eigen::Vector3f get_acceleration() const;// TODO: Omzetten naar m/s^2 (op dit moment g verhouding)
    Eigen::Vector3f get_speed() const;// TODO: Omzetten naar m/s
    Eigen::Vector3f get_rotational_acceleration() const; 

    float get_temperature() const;

private:
    Eigen::Vector3f acc, speed, ang, rotationalAcc;

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
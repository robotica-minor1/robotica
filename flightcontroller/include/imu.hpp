#ifndef IMU_HPP
#define IMU_HPP

#include "kalman.hpp"
#include <stdint.h>
#include <thread>
#include <Eigen/Dense>
#include "fc_constants.hpp"

// Yaw and accelerometer are unreliable in the first ~1000 ms
class imu {
public:
    static imu& get();

    ~imu();

    void poll();

    Eigen::Vector3f get_angles() const;
    Eigen::Vector3f get_acceleration() const;
    Eigen::Vector3f get_speed() const;
    Eigen::Vector3f get_pos() const;
    Eigen::Vector3f get_rotational_acceleration() const; 
    Eigen::Vector3f get_rotational_velocity() const; 

    float get_temperature() const;

    bool is_updating() const;

private:
    Eigen::Vector3f acc = Eigen::Vector3f::Zero(3);
    Eigen::Vector3f speed = Eigen::Vector3f::Zero(3);
    Eigen::Vector3f ang = Eigen::Vector3f::Zero(3);
    Eigen::Vector3f rotationalAcc = Eigen::Vector3f::Zero(3);
    Eigen::Vector3f rotationalVel = Eigen::Vector3f::Zero(3);
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero(3);
    Eigen::Vector3f pos = Eigen::Vector3f::Zero(3);

    Kalman kalmanX, kalmanY;

    int16_t tempRaw;

    bool calibrated = false;
    int start_t;
    double yawOffset;

    Eigen::Vector3f accOffset = Eigen::Vector3f::Zero(3);

    double compAngleX, compAngleY;
    double kalAngleX, kalAngleY;

    uint32_t timer;
    uint8_t i2cData[14];

    std::thread update_thread;
    bool updating = true;

    imu();

    imu(imu const&) = delete;
    void operator=(imu const&) = delete;
};

#endif
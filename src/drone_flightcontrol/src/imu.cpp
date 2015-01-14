#include <Arduino.h>
#include <Wire.h>
#include <stdexcept>

#include "i2c.hpp"
#include "imu.hpp"
#include "kalman.hpp"

imu& imu::get() {
    static imu instance;
    return instance;
}

imu::imu() : calibrated(false), start_t(0), yawOffset(0) {
    Wire.begin();

    // Set I2C frequency to 400 kHz
    TWBR = ((F_CPU / 400000L) - 16) / 2;

    i2cData[0] = 7; // Set sample rate to 1000 Hz = 8000 Hz / (7 + 1)
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

    while (i2c::write(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2c::write(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

    while (i2c::read(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) {
        throw std::runtime_error("failed to detect IMU");
    }

    // Let sensor stabilise
    delay(100);

    // Set kalman and gyro start angle
    while (i2c::read(0x3B, i2cData, 6));
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];

    // Calculate roll/pitch using accelerometer, restricting roll to +-90 deg
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

    kalmanX.setAngle(roll);
    kalmanY.setAngle(pitch);

    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();
    start_t = millis();
}

void imu::poll() {
    // Update values
    while (i2c::read(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    double dt = (double)(micros() - timer) / 1000000;
    timer = micros();

    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

    double gyroXrate = gyroX / 131.0;
    double gyroYrate = gyroY / 131.0;
    double gyroZrate = gyroZ / 131.0;

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading

    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt - yawOffset * dt;

    // Determine yaw offset
    if (!calibrated && millis() - start_t >= 1000) {
        yawOffset = gyroZangle;
        calibrated = true;
        gyroZangle = 0;
    }

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
}

angles imu::get_angles() const {
    return angles(kalAngleX, kalAngleY, gyroZangle);
}

vec3 imu::get_acceleration() const {
    return vec3(accX, accY, accZ);
}

float imu::get_temperature() const {
    return (float) tempRaw / 340.0f + 36.53f;
}
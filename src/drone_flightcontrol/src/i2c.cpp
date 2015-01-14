#include <Arduino.h>
#include <Wire.h>
#include <stdexcept>
#include "i2c.hpp"

namespace i2c {
    const uint8_t IMU_ADDR = 0x68;
    const uint16_t TIMEOUT = 1000;

    uint8_t write(uint8_t registerAddress, uint8_t data, bool sendStop) {
        return write(registerAddress, &data, 1, sendStop);
    }

    uint8_t write(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
        Wire.beginTransmission(IMU_ADDR);
        Wire.write(registerAddress);
        Wire.write(data, length);

        uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
        if (rcode) {
            throw std::runtime_error("i2c write failure");
        }

        return rcode;
    }

    uint8_t read(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
        uint32_t timeOutTimer;
        Wire.beginTransmission(IMU_ADDR);
        Wire.write(registerAddress);
        uint8_t rcode = Wire.endTransmission(false); // Don't release the bus

        if (rcode) {
            throw std::runtime_error("i2c read failure");
            return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
        }

        Wire.requestFrom(IMU_ADDR, nbytes, (uint8_t) true); // Send a repeated start and then release the bus after reading
        for (uint8_t i = 0; i < nbytes; i++) {
            if (Wire.available())
                data[i] = Wire.read();
            else {
                timeOutTimer = micros();

                while (((micros() - timeOutTimer) < TIMEOUT) && !Wire.available());
                
                if (Wire.available())
                    data[i] = Wire.read();
                else {
                    throw std::runtime_error("i2c read failure");
                    return 5; // This error value is not already taken by endTransmission
                }
            }
        }

        return 0; // Success
    }
}
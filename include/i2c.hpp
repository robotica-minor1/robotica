#ifndef I2C_HPP
#define I2C_HPP

#include <stdint.h>

namespace i2c {
    uint8_t write(uint8_t registerAddress, uint8_t data, bool sendStop);
    uint8_t write(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop);

    uint8_t read(uint8_t registerAddress, uint8_t* data, uint8_t nbytes);
}

#endif
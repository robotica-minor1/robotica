#include "i2c.hpp"

#include <cstdio>
#include <string>
#include <stdexcept>

namespace i2c {
    static std::string exec(const std::string& cmd) {
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) throw std::runtime_error("can't open pipe to command");

        char buf[128];
        std::string result = "";

        while (!feof(pipe)) {
            if(fgets(buf, 128, pipe) != nullptr) {
                result += buf;
            }
        }

        pclose(pipe);

        return result;
    }

    static uint8_t _write(uint8_t addr, uint8_t reg, uint8_t val) {
        static char buf[128];
        sprintf(buf, "i2cset -y 1 0x%X 0x%X 0x%X", addr, reg, val);
        exec(buf);
    }

    static uint8_t _read(uint8_t addr, uint8_t reg) {
        static char buf[128];
        sprintf(buf, "i2cget -y 1 0x%X 0x%X", addr, reg);
        std::string res = exec(buf);

        int data;
        sscanf(res.c_str(), "0x%X", &data);

        return data;
    }

    uint8_t write(uint8_t reg, uint8_t data) {
        return write(reg, &data, 1);
    }

    uint8_t write(uint8_t reg, uint8_t* data, uint8_t length) {
        for (int i = 0; i < length; i++) {
            _write(0x68, reg, data[i]);
        }

        return 0;
    }

    uint8_t read(uint8_t reg, uint8_t* data, uint8_t length) {
        if (reg == 0x3B && length == 14) {
            std::string res = exec("i2cdump -r 0x3B-0x48 -y 1 0x68 b");
            res = res.substr(res.find_first_of('\n') + 1);

            std::string line1 = res.substr(4 + 33, res.find_first_of('\n') - 4 - 33);
            std::string line2 = res.substr(res.find_first_of('\n') + 1 + 4);

            int tmp[14];

            sscanf(line1.c_str(), "%x %x %x %x %x", &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4]);
            sscanf(line2.c_str(), "%x %x %x %x %x %x %x %x %x", &tmp[5], &tmp[6], &tmp[7], &tmp[8], &tmp[9], &tmp[10], &tmp[11], &tmp[12], &tmp[13]);

            for (int i = 0; i < 14; i++) {
                data[i] = tmp[i];
            }
        } else {
            for (int i = 0; i < length; i++) {
                data[i] = _read(0x68, reg);
            }
        }

        return 0;
    }
}
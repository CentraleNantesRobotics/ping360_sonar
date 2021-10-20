#pragma once

#include <stdint.h>

// port interface used by PingDevice class
class PingPort {
public:
    virtual int read(uint8_t* buffer, int nBytes) = 0;
    virtual int write(const uint8_t* data, int nBytes) = 0;
};

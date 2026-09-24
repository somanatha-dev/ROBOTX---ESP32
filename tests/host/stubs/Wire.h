#ifndef SIM_WIRE_H
#define SIM_WIRE_H

// Host stub for the ESP32 Wire library. A small fake bus: a few addresses
// ACK (see sim_hw.cpp), everything else NACKs. Nothing here moves a motor.

#include <stdint.h>
#include <stddef.h>

class TwoWire {
public:
    bool    begin(int sda, int scl, uint32_t freq);
    void    setTimeOut(uint16_t ms) { (void)ms; }
    void    beginTransmission(int address);
    size_t  write(uint8_t b);
    uint8_t endTransmission(bool sendStop = true);
    uint8_t requestFrom(int address, int quantity);
    int     available(void);
    int     read(void);
};

extern TwoWire Wire;

#endif

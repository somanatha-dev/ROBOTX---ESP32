#ifndef SIM_VL53L0X_H
#define SIM_VL53L0X_H

// Host stub for the Pololu VL53L0X library: just enough surface for
// rear_tof.cpp to compile. While TCA9548A_ADDRESS_CONFIRMED is 0 (the current
// configuration) the firmware never calls any of it.

#include <stdint.h>
#include "Wire.h"

// Stands in for the PATCHED library (rear_tof.cpp refuses to build without
// this marker). The guard itself is tested against the real library in
// tests/host/test_vl53_guard.cpp.
#define VL53L0X_ROVER_VCSEL_ZERO_GUARD 1

class VL53L0X {
public:
    enum regAddr {
        RESULT_INTERRUPT_STATUS = 0x13,
        RESULT_RANGE_STATUS     = 0x14
    };

    uint8_t last_status = 0;

    void     setBus(TwoWire *bus)                    { (void)bus; }
    void     setTimeout(uint16_t ms)                 { (void)ms; }
    bool     init(bool io2v8 = true)                 { (void)io2v8; return false; }
    bool     setMeasurementTimingBudget(uint32_t us) { (void)us; return true; }
    void     startContinuous(uint32_t periodMs = 0)  { (void)periodMs; }
    uint8_t  readReg(uint8_t reg)                    { (void)reg; return 0; }
    uint16_t readRangeContinuousMillimeters(void)    { return 65535; }
    uint16_t readRangeSingleMillimeters(void)        { return 65535; }
    bool     timeoutOccurred(void)                   { return true; }
};

#endif

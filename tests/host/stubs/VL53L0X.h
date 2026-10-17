#ifndef SIM_VL53L0X_H
#define SIM_VL53L0X_H

// Host stub for the Pololu VL53L0X library, used by the link simulator. Every
// call is answered by tests/host/sim_hw.cpp through the hooks below, by the
// sensor on whichever ONE TCA channel is open at that moment. With SIM_REAR_TOF
// unset the hooks keep the old inert answers -- init() fails, nothing is ready,
// every range read times out -- and since the simulated bus then has no
// TCA9548A at the configured address, the firmware never calls any of it.

#include <stdint.h>
#include "Wire.h"

// Stands in for the PATCHED library (rear_tof.cpp refuses to build without
// this marker). The guard itself is tested against the real library in
// tests/host/test_vl53_guard.cpp.
#define VL53L0X_ROVER_VCSEL_ZERO_GUARD 1

bool     simVl53Init(void);
uint8_t  simVl53ReadReg(uint8_t reg, uint8_t *status);
uint16_t simVl53ReadRange(uint8_t *status, bool *timedOut);

class VL53L0X {
public:
    enum regAddr {
        RESULT_INTERRUPT_STATUS = 0x13,
        RESULT_RANGE_STATUS     = 0x14
    };

    uint8_t last_status = 0;

    void     setBus(TwoWire *bus)                    { (void)bus; }
    void     setTimeout(uint16_t ms)                 { (void)ms; }
    bool     init(bool io2v8 = true)                 { (void)io2v8; return simVl53Init(); }
    bool     setMeasurementTimingBudget(uint32_t us) { (void)us; return true; }
    void     startContinuous(uint32_t periodMs = 0)  { (void)periodMs; }
    uint8_t  readReg(uint8_t reg)                    { return simVl53ReadReg(reg, &last_status); }
    uint16_t readRangeContinuousMillimeters(void)    { return simVl53ReadRange(&last_status, &didTimeout); }
    uint16_t readRangeSingleMillimeters(void)        { return readRangeContinuousMillimeters(); }

    bool timeoutOccurred(void)
    {
        bool t = didTimeout;
        didTimeout = false;
        return t;
    }

private:
    bool didTimeout = false;
};

#endif

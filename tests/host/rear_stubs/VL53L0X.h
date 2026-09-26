#ifndef SIM_REAR_VL53L0X_H
#define SIM_REAR_VL53L0X_H

// Scriptable host stub for the Pololu VL53L0X library, used ONLY by
// tests/host/test_rear_tof.cpp (this directory is first on that build's
// include path; the link simulator keeps using tests/host/stubs/VL53L0X.h).
//
// Every call is answered by the sensor on the ONE TCA channel that is open at
// that moment, exactly as on the rover where all three sit at 0x29. The test
// implements the three hooks below. `last_status` mirrors the real library:
// the endTransmission() code of the register-pointer write, 0 = success.

#include <stdint.h>
#include "Wire.h"

bool     simTofInit(uint8_t *status);
uint8_t  simTofReadReg(uint8_t reg, uint8_t *status);
uint16_t simTofReadRange(uint8_t *status, bool *timedOut);

// The timeout flag, made observable. Like the library's private did_timeout,
// it is per object, set only by a timed-out range read and cleared only by
// timeoutOccurred(). simTofReadRange() also sees the flag's value on entry,
// so the test can tell whether a STALE flag was present when a read started.
void     simTofFlagSet(void);
void     simTofFlagCleared(void);

class VL53L0X {
public:
    enum regAddr {
        RESULT_INTERRUPT_STATUS = 0x13,
        RESULT_RANGE_STATUS     = 0x14
    };

    uint8_t last_status = 0;

    void     setBus(TwoWire *bus)                    { (void)bus; }
    void     setTimeout(uint16_t ms)                 { (void)ms; }
    bool     init(bool io2v8 = true)                 { (void)io2v8; return simTofInit(&last_status); }
    bool     setMeasurementTimingBudget(uint32_t us) { (void)us; return true; }
    void     startContinuous(uint32_t periodMs = 0)  { (void)periodMs; }
    uint8_t  readReg(uint8_t reg)                    { return simTofReadReg(reg, &last_status); }
    uint16_t readRangeSingleMillimeters(void)        { return readRangeContinuousMillimeters(); }

    uint16_t readRangeContinuousMillimeters(void)
    {
        bool     was = didTimeout;
        uint16_t mm  = simTofReadRange(&last_status, &didTimeout);
        if (!was && didTimeout) simTofFlagSet();
        return mm;
    }

    bool timeoutOccurred(void)
    {
        bool t = didTimeout;
        didTimeout = false;
        if (t) simTofFlagCleared();
        return t;
    }

private:
    bool didTimeout = false;
};

#endif

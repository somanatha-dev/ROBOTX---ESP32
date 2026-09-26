// ============================================================================
//  FOCUSED HOST TESTS -- rear VL53L0X BUS_ERROR recovery
//
//  REAL firmware under test, compiled unchanged: rear_tof.cpp, safety.cpp,
//  tca9548a.cpp, rover_i2c.cpp and config.h. Simulated below them:
//
//    * the clock   -- a manual millisecond counter; step() advances 1 ms and
//                     calls safetyUpdate() once, as loop() does
//    * the I2C bus -- only the TCA9548A (TCA9548A_I2C_ADDRESS) answers; it
//                     keeps its control byte and can be forced to NACK
//    * 3x VL53L0X  -- scriptable (rear_stubs/VL53L0X.h); each answers ONLY
//                     while its channel is the one channel open, so any
//                     access without a correct select is counted as a bug
//
//  What this CANNOT show: real ESP32 Wire / IDF error codes and timing, real
//  VL53L0X init duration, or how the physical sensors behave after a fault.
// ============================================================================

#include <stdio.h>
#include <string.h>

#include "Arduino.h"
#include "Wire.h"
#include "VL53L0X.h"
#include "config.h"
#include "rover_i2c.h"
#include "tca9548a.h"
#include "rear_tof.h"
#include "safety.h"

// ---------------------------------------------------------------------------
// Arduino core: manual clock, inert GPIO
// ---------------------------------------------------------------------------

static uint32_t gNow = 0;

uint32_t millis(void)                         { return gNow; }
void     delay(uint32_t ms)                   { gNow += ms; }
void     delayMicroseconds(uint32_t us)       { (void)us; }
void     pinMode(uint8_t pin, uint8_t mode)   { (void)pin; (void)mode; }
void     digitalWrite(uint8_t pin, uint8_t v) { (void)pin; (void)v; }
int      digitalRead(uint8_t pin)             { (void)pin; return HIGH; }

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeoutUs)
{
    (void)pin; (void)state;
    unsigned long us = 5831;                    // ~100 cm, front sensors valid
    return (us > timeoutUs) ? 0 : us;
}

// ---------------------------------------------------------------------------
// I2C bus: the TCA9548A is the only device that answers on it
// ---------------------------------------------------------------------------

TwoWire Wire;

static bool    gTcaNack = false;
static uint8_t gTcaMask = 0;
static int     gTxAddr  = -1;
static bool    gTxWrote = false;
static uint8_t gTxByte  = 0;
static int     gRxAvail = 0;
static uint8_t gRxByte  = 0;

bool TwoWire::begin(int sda, int scl, uint32_t freq)
{
    (void)sda; (void)scl; (void)freq;
    return true;
}

void TwoWire::beginTransmission(int address)
{
    gTxAddr  = address;
    gTxWrote = false;
}

size_t TwoWire::write(uint8_t b)
{
    gTxByte  = b;
    gTxWrote = true;
    return 1;
}

uint8_t TwoWire::endTransmission(bool sendStop)
{
    (void)sendStop;
    if (gTxAddr != TCA9548A_I2C_ADDRESS || gTcaNack) {
        return 2;                                   // address NACK
    }
    if (gTxWrote) {
        gTcaMask = gTxByte;
    }
    return 0;
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
    if (address != TCA9548A_I2C_ADDRESS || gTcaNack || quantity < 1) {
        gRxAvail = 0;
        return 0;
    }
    gRxByte  = gTcaMask;
    gRxAvail = 1;
    return 1;
}

int TwoWire::available(void) { return gRxAvail; }

int TwoWire::read(void)
{
    if (gRxAvail <= 0) {
        return -1;
    }
    gRxAvail--;
    return gRxByte;
}

// ---------------------------------------------------------------------------
// Three scriptable VL53L0X, one per TCA channel
// ---------------------------------------------------------------------------

enum SimMode {
    M_VALID,            // ready, range status 11, returns mm
    M_OUT_OF_RANGE,     // ready, returns 8190
    M_TIMEOUT,          // ready, range read times out (65535)
    M_NOT_READY,        // answers, but no measurement ready
    M_ERR_READY,        // RESULT_INTERRUPT_STATUS pointer write fails
    M_ERR_STATUS,       // RESULT_RANGE_STATUS pointer write fails
    M_ERR_RANGE,        // readRangeContinuousMillimeters() last write fails
    M_ERR_RANGE_TO      // range read times out AND its last pointer write
                        // failed: last_status != 0 with the timeout flag set
};

#define SIM_MAX_INIT_LOG 512

struct SimTof {
    bool     initOk;
    SimMode  mode;
    uint16_t mm;
    uint8_t  errCode;
    int      initCalls;
    uint32_t initTimes[SIM_MAX_INIT_LOG];
};

static SimTof gSim[REAR_TOF_SENSOR_COUNT];
static int    gBadSelect = 0;   // VL53L0X accessed with != 1 channel open

// Timeout-flag bookkeeping across all three sensor objects.
static int    gFlagsSet     = 0;   // flag went false -> true
static int    gFlagsCleared = 0;   // timeoutOccurred() consumed a set flag
static int    gStaleAtRead  = 0;   // a range read STARTED with the flag set

void simTofFlagSet(void)     { gFlagsSet++; }
void simTofFlagCleared(void) { gFlagsCleared++; }
static int flagsPending(void) { return gFlagsSet - gFlagsCleared; }

static const uint8_t kChannelOf[REAR_TOF_SENSOR_COUNT] = {
    TCA_CH_REAR_TOF_0, TCA_CH_REAR_TOF_1, TCA_CH_REAR_TOF_2
};

// The sensor index whose channel is the ONE channel open, or -1.
static int openSensor(void)
{
    uint8_t m = gTcaMask;
    if (m == 0 || (m & (m - 1)) != 0) {
        return -1;
    }
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (m == (uint8_t)(1u << kChannelOf[i])) {
            return i;
        }
    }
    return -1;
}

bool simTofInit(uint8_t *status)
{
    int i = openSensor();
    if (i < 0) {
        gBadSelect++;
        *status = 2;
        return false;
    }
    SimTof &t = gSim[i];
    if (t.initCalls < SIM_MAX_INIT_LOG) {
        t.initTimes[t.initCalls] = gNow;
    }
    t.initCalls++;
    *status = t.initOk ? 0 : 2;
    return t.initOk;
}

uint8_t simTofReadReg(uint8_t reg, uint8_t *status)
{
    int i = openSensor();
    if (i < 0) {
        gBadSelect++;
        *status = 2;
        return 0xFF;
    }
    const SimTof &t = gSim[i];

    if (reg == VL53L0X::RESULT_INTERRUPT_STATUS) {
        if (t.mode == M_ERR_READY) { *status = t.errCode; return 0xFF; }
        *status = 0;
        return (t.mode == M_NOT_READY) ? 0x00 : 0x07;
    }
    if (reg == VL53L0X::RESULT_RANGE_STATUS) {
        if (t.mode == M_ERR_STATUS) { *status = t.errCode; return 0xFF; }
        *status = 0;
        return (uint8_t)(11 << 3);
    }
    *status = 0;
    return 0;
}

uint16_t simTofReadRange(uint8_t *status, bool *timedOut)
{
    int i = openSensor();
    if (i < 0) {
        gBadSelect++;
        *status = 2;
        return 0xFFFF;
    }
    const SimTof &t = gSim[i];

    if (*timedOut) {
        gStaleAtRead++;
    }

    *status = 0;
    switch (t.mode) {
        case M_ERR_RANGE:    *status = t.errCode; return 0xFFFF;
        case M_ERR_RANGE_TO: *status = t.errCode; *timedOut = true; return 65535;
        case M_TIMEOUT:      *timedOut = true;     return 65535;
        case M_OUT_OF_RANGE: return 8190;
        default:             return t.mm;
    }
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

static const char *gTest   = "";
static int         gChecks = 0;
static int         gFails  = 0;

#define CHECK(cond, msg)                                                      \
    do {                                                                      \
        gChecks++;                                                            \
        if (!(cond)) {                                                        \
            gFails++;                                                         \
            printf("  FAIL [%s] line %d: %s -- %s\n",                         \
                   gTest, __LINE__, #cond, msg);                              \
        }                                                                     \
    } while (0)

static void resetSim(void)
{
    gTcaNack      = false;
    gTcaMask      = 0;
    gBadSelect    = 0;
    gFlagsSet     = 0;
    gFlagsCleared = 0;
    gStaleAtRead  = 0;
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        gSim[i].initOk    = true;
        gSim[i].mode      = M_VALID;
        gSim[i].mm        = 1000;
        gSim[i].errCode   = 2;
        gSim[i].initCalls = 0;
    }
}

// The rear-relevant part of setup(), in setup()'s order.
static void boot(void)
{
    gNow = 10;
    roverI2cInit();
    (void)tcaInit();
    safetyInit();
}

static void step(void)
{
    gNow++;
    safetyUpdate();
}

static void run(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        step();
    }
}

template <typename F>
static bool runUntil(F done, uint32_t maxMs)
{
    for (uint32_t i = 0; i < maxMs; i++) {
        if (done()) {
            return true;
        }
        step();
    }
    return done();
}

static bool allValid(void)
{
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (!rearTofValid(i)) return false;
    }
    return true;
}

// Boot with three healthy sensors at 1000 mm and let every window fill.
static void bootAllValid(void)
{
    resetSim();
    boot();
    run(1500);
    CHECK(rearTofBackendAvailable(), "setup: backend did not come up");
    CHECK(allValid(), "setup: sensors not valid after boot");
}

// ---------------------------------------------------------------------------
// A. 50 consecutive sensor BUS_ERRORs -> demoted
// ---------------------------------------------------------------------------
static void testA(void)
{
    bootAllValid();
    gSim[2].mode = M_ERR_READY;

    CHECK(runUntil([] { return rearTofBusErrStreak(2) ==
                               REAR_TOF_BUS_ERROR_REINIT_STREAK - 1; }, 20000),
          "streak never reached threshold - 1");
    CHECK(rearTofInitialised(2), "demoted BEFORE the threshold");
    CHECK(rearTofBusErrTotal(2) == REAR_TOF_BUS_ERROR_REINIT_STREAK - 1, "total");

    CHECK(runUntil([] { return rearTofBusErrTotal(2) ==
                               REAR_TOF_BUS_ERROR_REINIT_STREAK; }, 1000),
          "threshold-th BUS_ERROR never happened");
    CHECK(!rearTofInitialised(2), "not demoted at the threshold");
    CHECK(rearTofBusErrStreak(2) == 0, "streak not reset by demotion");
    CHECK(rearTofStatusOf(2) == TOF_BUS_ERROR, "status should still say BUS_ERROR");
    CHECK(rearTofFailStreak(2) == REAR_TOF_BUS_ERROR_REINIT_STREAK,
          "failStreak semantics changed");
    CHECK(rearTofHealthOf(2) == HEALTH_FAULT, "demoted sensor not FAULT");
    CHECK(!rearTofValid(2), "demoted sensor still valid");
    CHECK(rearTofReinitCount(2) == 0, "re-init ran at the moment of demotion");

    CHECK(rearTofInitialised(0) && rearTofInitialised(1), "other sensors demoted");
    CHECK(rearTofValid(0) && rearTofValid(1), "other sensors disturbed");
}

// ---------------------------------------------------------------------------
// B. Demotion -> the existing 5 s re-init branch becomes reachable
// ---------------------------------------------------------------------------
static void testB(void)
{
    bootAllValid();
    const uint32_t bootInit = gSim[2].initTimes[0];

    gSim[2].mode = M_ERR_READY;
    CHECK(runUntil([] { return !rearTofInitialised(2); }, 20000), "never demoted");
    const uint32_t tDemote   = gNow;
    const int      callsHere = gSim[2].initCalls;

    CHECK(tDemote < bootInit + REAR_TOF_REINIT_AFTER_MS,
          "setup: demotion should come before the throttle expires");

    CHECK(runUntil([] { return rearTofReinitCount(2) == 1; }, 20000),
          "re-init branch never ran after demotion");
    CHECK(gSim[2].initCalls == callsHere + 1,
          "re-init did not reach sensor 2 on its own channel");

    const uint32_t tReinit = gSim[2].initTimes[callsHere];
    CHECK(tReinit - bootInit >= REAR_TOF_REINIT_AFTER_MS, "5 s throttle ignored");
    CHECK(tReinit - bootInit <  REAR_TOF_REINIT_AFTER_MS + 100,
          "re-init not taken on the first visit after the throttle");
    CHECK(rearTofLastReinitResult(2) == TOF_REINIT_OK, "result not OK");
    CHECK(rearTofInitialised(2), "successful re-init did not re-initialise");
    CHECK(rearTofStatusOf(2) == TOF_UNINITIALISED, "status after re-init");
    CHECK(!rearTofValid(2), "valid straight after re-init");
}

// ---------------------------------------------------------------------------
// C. Successful re-init -> valid ONLY after new valid readings
// ---------------------------------------------------------------------------
static void testC(void)
{
    bootAllValid();
    gSim[0].mode = M_ERR_READY;
    CHECK(runUntil([] { return !rearTofInitialised(0); }, 20000), "never demoted");

    gSim[0].mode = M_VALID;
    gSim[0].mm   = 700;
    CHECK(runUntil([] { return rearTofReinitCount(0) == 1; }, 20000), "no re-init");
    CHECK(rearTofLastReinitResult(0) == TOF_REINIT_OK, "re-init failed");

    CHECK(!rearTofValid(0), "valid right after re-init");
    CHECK(rearTofGoodSampleCount(0) == 0, "window not empty after re-init");
    CHECK(rearTofHealthOf(0) == HEALTH_FAULT, "healthy right after re-init");
    CHECK(rearTofFilteredMm(0) == REAR_TOF_INVALID_MM, "distance published");
    CHECK(safetyRearSensorFault(), "rear fault not reported");
    CHECK(safetyReverseBlocked(), "reverse open during recovery");

    int  earlyValid = 0;
    bool sawOne = false, sawTwo = false;
    for (int i = 0; i < 2000 && !rearTofValid(0); i++) {
        step();
        uint8_t g = rearTofGoodSampleCount(0);
        if (rearTofValid(0) && g < REAR_TOF_MIN_GOOD_SAMPLES) earlyValid++;
        if (!rearTofValid(0) && g == 1) sawOne = true;
        if (!rearTofValid(0) && g == 2) sawTwo = true;
        if (!rearTofValid(0) && !safetyReverseBlocked()) earlyValid++;
    }

    CHECK(rearTofValid(0), "never became valid on real data");
    CHECK(earlyValid == 0, "valid (or reverse open) before enough samples");
    CHECK(sawOne && sawTwo, "did not pass through 1 and 2 good samples invalid");
    CHECK(rearTofGoodSampleCount(0) == REAR_TOF_MIN_GOOD_SAMPLES, "count at validity");
    CHECK(rearTofFilteredMm(0) == 700, "filtered value not from new readings");
    CHECK(rearTofHealthOf(0) != HEALTH_OK, "OK health on a partial window");
}

// ---------------------------------------------------------------------------
// D. TIMEOUT does not increment the recovery counter, and breaks a run
// ---------------------------------------------------------------------------
static void testD(void)
{
    bootAllValid();
    gSim[0].mode = M_TIMEOUT;
    run(10000);

    CHECK(rearTofStatusOf(0) == TOF_TIMEOUT, "TIMEOUT not reported as TIMEOUT");
    CHECK(rearTofBusErrStreak(0) == 0, "TIMEOUT counted toward recovery");
    CHECK(rearTofBusErrTotal(0) == 0, "TIMEOUT counted as BUS_ERROR");
    CHECK(rearTofInitialised(0), "TIMEOUT caused a demotion");
    CHECK(rearTofReinitCount(0) == 0, "TIMEOUT caused a re-init");
    CHECK(rearTofFailStreak(0) >= 100, "TIMEOUT no longer counts in failStreak");

    gSim[0].mode = M_ERR_READY;
    CHECK(runUntil([] { return rearTofBusErrStreak(0) ==
                               REAR_TOF_BUS_ERROR_REINIT_STREAK - 1; }, 20000), "run 1");
    gSim[0].mode = M_TIMEOUT;
    CHECK(runUntil([] { return rearTofStatusOf(0) == TOF_TIMEOUT; }, 1000), "timeout");
    CHECK(rearTofBusErrStreak(0) == 0, "TIMEOUT did not break the BUS_ERROR run");
    gSim[0].mode = M_ERR_READY;
    CHECK(runUntil([] { return rearTofBusErrStreak(0) ==
                               REAR_TOF_BUS_ERROR_REINIT_STREAK - 1; }, 20000), "run 2");
    CHECK(rearTofInitialised(0), "demoted without CONSECUTIVE BUS_ERRORs");
}

// ---------------------------------------------------------------------------
// E. OUT_OF_RANGE does not increment the recovery counter, and breaks a run
// ---------------------------------------------------------------------------
static void testE(void)
{
    bootAllValid();
    gSim[1].mode = M_OUT_OF_RANGE;
    run(10000);

    CHECK(rearTofStatusOf(1) == TOF_OUT_OF_RANGE, "OUT_OF_RANGE not reported");
    CHECK(rearTofRawMm(1) == 8190, "raw 8190 no longer visible");
    CHECK(!rearTofValid(1), "OUT_OF_RANGE became a valid distance");
    CHECK(rearTofBusErrStreak(1) == 0, "OUT_OF_RANGE counted toward recovery");
    CHECK(rearTofBusErrTotal(1) == 0, "OUT_OF_RANGE counted as BUS_ERROR");
    CHECK(rearTofInitialised(1), "OUT_OF_RANGE caused a demotion");
    CHECK(rearTofReinitCount(1) == 0, "OUT_OF_RANGE caused a re-init");
    CHECK(gSim[1].initCalls == 1, "sensor re-initialised");

    gSim[1].mode = M_ERR_READY;
    CHECK(runUntil([] { return rearTofBusErrStreak(1) ==
                               REAR_TOF_BUS_ERROR_REINIT_STREAK - 1; }, 20000), "run 1");
    gSim[1].mode = M_OUT_OF_RANGE;
    CHECK(runUntil([] { return rearTofStatusOf(1) == TOF_OUT_OF_RANGE; }, 1000), "oor");
    CHECK(rearTofBusErrStreak(1) == 0, "OUT_OF_RANGE did not break the run");
    gSim[1].mode = M_ERR_READY;
    CHECK(runUntil([] { return rearTofBusErrStreak(1) ==
                               REAR_TOF_BUS_ERROR_REINIT_STREAK - 1; }, 20000), "run 2");
    CHECK(rearTofInitialised(1), "demoted without CONSECUTIVE BUS_ERRORs");
}

// ---------------------------------------------------------------------------
// F. Demotion and recovery do not clear the obstacle latch
// ---------------------------------------------------------------------------
static void testF(void)
{
    resetSim();
    gSim[1].mm = 100;                       // inside REAR_STOP_DISTANCE_MM
    boot();
    run(1500);
    CHECK(safetyRearSensorObstacle(1), "setup: latch not set");
    CHECK(safetyReverseBlocked(), "setup: reverse not blocked");

    int latchLost = 0;
    gSim[1].mode = M_ERR_READY;
    while (rearTofInitialised(1) && gNow < 30000) {
        step();
        if (!safetyRearSensorObstacle(1)) latchLost++;
    }
    CHECK(!rearTofInitialised(1), "never demoted");

    gSim[1].mode = M_VALID;
    gSim[1].mm   = 1000;                    // far: would CLEAR on valid data
    while (!rearTofValid(1) && gNow < 60000) {
        step();
        if (!safetyRearSensorObstacle(1)) latchLost++;
    }
    CHECK(rearTofReinitCount(1) >= 1, "no re-init happened");
    CHECK(rearTofValid(1), "never recovered");
    CHECK(latchLost == 0, "latch cleared by demotion / re-init / invalid data");
    CHECK(safetyRearSensorObstacle(1), "latch not held at first valid reading");

    // Only now, on genuinely new valid readings, does the unchanged rule apply.
    CHECK(runUntil([] { return !safetyRearSensorObstacle(1); }, 1000),
          "latch never released by new valid far readings");
}

// ---------------------------------------------------------------------------
// G. Demotion clears the old sample window
// ---------------------------------------------------------------------------
static void testG(void)
{
    resetSim();
    gSim[0].mm = 100;                       // pre-failure window: all 100 mm
    boot();
    run(1500);
    CHECK(rearTofGoodSampleCount(0) == REAR_TOF_SAMPLE_WINDOW, "setup: window");

    gSim[0].mode = M_ERR_READY;
    CHECK(runUntil([] { return !rearTofInitialised(0); }, 20000), "never demoted");
    CHECK(rearTofGoodSampleCount(0) == 0, "window not cleared");
    CHECK(rearTofFilteredMm(0) == REAR_TOF_INVALID_MM, "filtered value kept");
    CHECK(!rearTofValid(0), "valid after demotion");

    gSim[0].mode = M_VALID;
    gSim[0].mm   = 1500;
    CHECK(runUntil([] { return rearTofValid(0); }, 20000), "never recovered");
    CHECK(rearTofFilteredMm(0) == 1500, "pre-failure samples leaked into median");
}

// ---------------------------------------------------------------------------
// H. Repeated failures -> no retry storm
// ---------------------------------------------------------------------------
static void stormCase(bool initOk)
{
    bootAllValid();
    gSim[0].initOk = initOk;
    gSim[0].mode   = M_ERR_READY;
    const int firstRuntimeCall = gSim[0].initCalls;   // after the boot init

    run(60000);

    // Calls within 50 ms of each other are the retries inside ONE
    // initOneSelected() (delay(5) apart). Anything further apart is a new
    // runtime attempt.
    int      attempts = 0;
    uint32_t minGap   = 0xFFFFFFFFUL;
    uint32_t prevCall = gSim[0].initTimes[0];
    uint32_t prevStart = prevCall;
    for (int k = firstRuntimeCall; k < gSim[0].initCalls && k < SIM_MAX_INIT_LOG; k++) {
        uint32_t t = gSim[0].initTimes[k];
        if (t - prevCall > 50) {
            attempts++;
            if (t - prevStart < minGap) minGap = t - prevStart;
            prevStart = t;
        }
        prevCall = t;
    }

    CHECK(attempts == rearTofReinitCount(0), "reinit count does not match attempts");
    CHECK(minGap >= REAR_TOF_REINIT_AFTER_MS, "attempts closer than the throttle");
    CHECK(attempts <= (int)(60000 / REAR_TOF_REINIT_AFTER_MS) + 1, "retry storm");
    CHECK(attempts >= 8, "recovery stopped retrying");
    CHECK(gSim[0].initCalls - firstRuntimeCall <= attempts * REAR_TOF_INIT_ATTEMPTS,
          "more init() calls than attempts allow");
    CHECK(rearTofValid(1) && rearTofValid(2), "healthy sensors disturbed");
}

static void testH(void)
{
    stormCase(false);   // sensor dead: every re-init fails
    stormCase(true);    // re-init succeeds, reads keep failing: fastest cycle
}

// ---------------------------------------------------------------------------
// I. CRITICAL: backend up, then all sensors demoted -> reverse stays blocked
// ---------------------------------------------------------------------------
static void testI(void)
{
    bootAllValid();
    CHECK(!safetyReverseBlocked(), "setup: reverse gate should be open");

    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        gSim[i].initOk = false;
        gSim[i].mode   = M_ERR_READY;
    }

    int unavailable = 0, unguarded = 0;
    for (int ms = 0; ms < 30000; ms++) {
        step();
        if (!rearTofBackendAvailable() || !safetyRearSensingAvailable()) unavailable++;
        if (!allValid() && !safetyReverseBlocked()) unguarded++;
    }
    CHECK(unavailable == 0, "rear backend became unavailable at runtime");
    CHECK(unguarded == 0, "reverse open while a rear sensor was invalid");

    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        CHECK(!rearTofInitialised(i), "sensor not demoted");
        CHECK(rearTofReinitCount(i) >= 1, "no re-init attempted");
        CHECK(rearTofLastReinitResult(i) == TOF_REINIT_FAILED, "re-init result");
        CHECK(rearTofStatusOf(i) == TOF_INIT_FAILED, "status");
    }
    CHECK(rearTofBackendAvailable(), "backend unavailable");
    CHECK(safetyRearSensorFault(), "rear sensor fault not reported");
    CHECK(safetyReverseBlocked(), "reverse not blocked");
    CHECK(strcmp(safetyBlockReason(-100, -100), "REAR_SENSOR_FAULT") == 0,
          "block reason is not REAR_SENSOR_FAULT");

    int l = 1, r = 1;
    safetyGateMotion(-100, -100, &l, &r);
    CHECK(l == 0 && r == 0, "straight reverse not gated to zero");
    safetyGateMotion(100, -100, &l, &r);
    CHECK(r == 0, "reversing wheel of a spin turn not gated");
}

// ---------------------------------------------------------------------------
// J. BOOT POLICY: nothing ever came up -> existing behaviour unchanged
// ---------------------------------------------------------------------------
static void testJ(void)
{
    resetSim();
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) gSim[i].initOk = false;
    boot();

    const bool policy = (SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED != 0);

    CHECK(!rearTofBackendAvailable(), "backend available with no sensor up");
    CHECK(!safetyRearSensingAvailable(), "rear sensing available");
    CHECK(!safetyRearSensorFault(), "boot all-fail now reported as sensor fault");
    CHECK(safetyReverseBlocked() == policy, "boot-time reverse policy changed");
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        CHECK(rearTofStatusOf(i) == TOF_INIT_FAILED, "status");
    }

    int cameUp = 0;
    for (int ms = 0; ms < 20000; ms++) {
        step();
        if (rearTofBackendAvailable()) cameUp++;
    }
    CHECK(cameUp == 0, "backend came up with every init failing");
    CHECK(rearTofReinitCount(0) >= 3, "existing periodic re-init not running");
    CHECK(rearTofLastReinitResult(0) == TOF_REINIT_FAILED, "re-init result");
    CHECK(safetyReverseBlocked() == policy, "policy changed after retries");

    // Existing behaviour kept: a sensor that initialises later brings the
    // backend up -- and from then on it stays up (sticky).
    gSim[0].initOk = true;
    CHECK(runUntil([] { return rearTofBackendAvailable(); }, 10000),
          "late-initialising sensor no longer brings the backend up");
    gSim[0].initOk = false;
    gSim[0].mode   = M_ERR_READY;
    int dropped = 0;
    for (int ms = 0; ms < 15000; ms++) {
        step();
        if (!rearTofBackendAvailable()) dropped++;
    }
    CHECK(!rearTofInitialised(0), "late sensor not demoted");
    CHECK(dropped == 0, "backend dropped after it had come up");
    CHECK(safetyReverseBlocked(), "reverse not blocked after backend came up");
}

// ---------------------------------------------------------------------------
// K. TCA select failures: recorded, never counted toward sensor recovery
// ---------------------------------------------------------------------------
static void testK(void)
{
    bootAllValid();
    int calls[REAR_TOF_SENSOR_COUNT];
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) calls[i] = gSim[i].initCalls;

    gTcaNack = true;
    run(10000);

    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        CHECK(rearTofBusErrTotal(i) >= 100, "select failures not recorded");
        CHECK(rearTofLastBusErrSite(i) == TOF_BUSERR_TCA_SELECT, "site");
        CHECK(rearTofLastBusErrCode(i) == 2, "raw code not kept");
        CHECK(rearTofBusErrStreak(i) == 0, "select failure counted toward recovery");
        CHECK(rearTofInitialised(i), "select failure demoted a sensor");
        CHECK(rearTofReinitCount(i) == 0, "select failure triggered a re-init");
        CHECK(gSim[i].initCalls == calls[i], "sensor init() called");
        CHECK(rearTofStatusOf(i) == TOF_BUS_ERROR, "status");
        CHECK(rearTofFailStreak(i) >= 100, "failStreak behaviour changed");
        CHECK(rearTofHealthOf(i) == HEALTH_FAULT, "health behaviour changed");
    }
    CHECK(strcmp(tcaStatusName(), "BUS_ERROR") == 0, "TCA status");
    CHECK(rearTofBackendAvailable(), "backend dropped");
    CHECK(safetyReverseBlocked(), "reverse open with the mux failing");

    gTcaNack = false;
    run(1500);
    CHECK(allValid(), "sensors did not resume after the mux recovered");
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        CHECK(gSim[i].initCalls == calls[i], "mux recovery re-initialised a sensor");
    }

    // A select failure neither advances nor resets a sensor run in progress.
    gSim[1].mode = M_ERR_READY;
    CHECK(runUntil([] { return rearTofBusErrStreak(1) == 30; }, 20000), "run");
    gTcaNack = true;
    run(3000);
    CHECK(rearTofBusErrStreak(1) == 30, "select failures changed the sensor streak");
    gTcaNack = false;
    CHECK(runUntil([] { return !rearTofInitialised(1); }, 20000),
          "sensor run did not resume after the mux recovered");
}

// ---------------------------------------------------------------------------
// L. Diagnostics: site and raw code per failing transaction
// ---------------------------------------------------------------------------
static void testL(void)
{
    bootAllValid();
    gSim[0].mode = M_ERR_READY;  gSim[0].errCode = 2;
    gSim[1].mode = M_ERR_STATUS; gSim[1].errCode = 5;
    gSim[2].mode = M_ERR_RANGE;  gSim[2].errCode = 4;
    run(300);

    CHECK(rearTofLastBusErrSite(0) == TOF_BUSERR_READY_REGISTER, "site 0");
    CHECK(rearTofLastBusErrSite(1) == TOF_BUSERR_RANGE_STATUS,   "site 1");
    CHECK(rearTofLastBusErrSite(2) == TOF_BUSERR_RANGE_READ,     "site 2");
    CHECK(rearTofLastBusErrCode(0) == 2, "code 0");
    CHECK(rearTofLastBusErrCode(1) == 5, "code 1");
    CHECK(rearTofLastBusErrCode(2) == 4, "code 2");

    CHECK(runUntil([] { return !rearTofInitialised(0) && !rearTofInitialised(1) &&
                               !rearTofInitialised(2); }, 20000),
          "a BUS_ERROR site does not trigger recovery");

    CHECK(strcmp(rearTofBusErrSiteName(TOF_BUSERR_NONE), "NONE") == 0, "name");
    CHECK(strcmp(rearTofBusErrSiteName(TOF_BUSERR_TCA_SELECT), "TCA_SELECT") == 0, "name");
    CHECK(strcmp(rearTofBusErrSiteName(TOF_BUSERR_READY_REGISTER), "READY_REGISTER") == 0, "name");
    CHECK(strcmp(rearTofBusErrSiteName(TOF_BUSERR_RANGE_STATUS), "RANGE_STATUS") == 0, "name");
    CHECK(strcmp(rearTofBusErrSiteName(TOF_BUSERR_RANGE_READ), "RANGE_READ") == 0, "name");
    CHECK(strcmp(rearTofReinitResultName(TOF_REINIT_OK), "OK") == 0, "name");
    CHECK(strcmp(rearTofReinitResultName(TOF_REINIT_FAILED), "FAILED") == 0, "name");
}

// ---------------------------------------------------------------------------
// M. A stale timeout flag from an earlier read is cleared before the next read
//
// FIDELITY NOTE: the stub reproduces the OUTCOME of the library's rare path --
// readRangeContinuousMillimeters() returns 65535 with did_timeout set AND
// last_status != 0 -- but not the internal poll loop that produces it on real
// hardware. The flag itself behaves like the library's: per object, set only
// by a timed-out range read, cleared only by timeoutOccurred(), untouched by
// init(). This is a software regression test, not a hardware test.
// ---------------------------------------------------------------------------
static void testM(void)
{
    // 1. A previous read can leave a stale flag.
    bootAllValid();
    gSim[0].errCode = 5;
    gSim[0].mode    = M_ERR_RANGE_TO;
    CHECK(runUntil([] { return rearTofBusErrTotal(0) == 1; }, 500), "no BUS_ERROR");
    CHECK(rearTofStatusOf(0) == TOF_BUS_ERROR, "not classified BUS_ERROR");
    CHECK(rearTofLastBusErrSite(0) == TOF_BUSERR_RANGE_READ, "site");
    CHECK(rearTofLastBusErrCode(0) == 5, "code");
    CHECK(rearTofBusErrStreak(0) == 1, "streak");
    CHECK(flagsPending() == 1, "setup: the BUS_ERROR visit did not leave the flag set");

    // 2 + 3. Cleared before the next read; the good reading is VALID.
    gSim[0].mode = M_VALID;
    CHECK(runUntil([] { return rearTofStatusOf(0) != TOF_BUS_ERROR; }, 500), "no visit");
    CHECK(rearTofStatusOf(0) == TOF_VALID, "good reading reported as TIMEOUT");
    CHECK(gStaleAtRead == 0, "a range read started with a stale timeout flag");
    CHECK(flagsPending() == 0, "stale flag not cleared");
    CHECK(rearTofBusErrStreak(0) == 0, "answered visit did not reset the streak");

    // 4. A timeout in the CURRENT call is still TIMEOUT -- also straight
    //    after a stale flag was discarded.
    gSim[0].mode = M_ERR_RANGE_TO;
    CHECK(runUntil([] { return rearTofBusErrTotal(0) == 2; }, 500), "no BUS_ERROR");
    gSim[0].mode = M_TIMEOUT;
    CHECK(runUntil([] { return rearTofStatusOf(0) != TOF_BUS_ERROR; }, 500), "no visit");
    CHECK(rearTofStatusOf(0) == TOF_TIMEOUT, "current-call timeout lost");
    CHECK(rearTofBusErrTotal(0) == 2, "TIMEOUT counted as BUS_ERROR");
    CHECK(rearTofBusErrStreak(0) == 0, "TIMEOUT counted toward recovery");
    CHECK(flagsPending() == 0, "current-call flag not consumed by classification");

    // 5 + 6. BUS_ERROR classification and the recovery counter are unchanged:
    //        every one of these visits is a RANGE_READ BUS_ERROR, the streak
    //        climbs one per visit, and the threshold demotes as before.
    gSim[0].mode = M_ERR_RANGE_TO;
    int badVisits = 0;
    uint32_t lastTotal = rearTofBusErrTotal(0);
    while (rearTofInitialised(0) && gNow < 60000) {
        step();
        if (rearTofBusErrTotal(0) != lastTotal) {
            lastTotal = rearTofBusErrTotal(0);
            if (rearTofStatusOf(0) != TOF_BUS_ERROR ||
                rearTofLastBusErrSite(0) != TOF_BUSERR_RANGE_READ) badVisits++;
            if (rearTofInitialised(0) &&
                rearTofBusErrStreak(0) != (uint16_t)(lastTotal - 2)) badVisits++;
        }
    }
    CHECK(badVisits == 0, "BUS_ERROR classification or streak changed");
    CHECK(!rearTofInitialised(0), "never demoted");
    CHECK(rearTofBusErrTotal(0) == 2 + REAR_TOF_BUS_ERROR_REINIT_STREAK,
          "demoted at a different streak than the threshold");

    // The audit's case: the flag survives demotion + init() in the library.
    // The first good reading after recovery must still be VALID.
    CHECK(flagsPending() == 1, "setup: flag should still be set across demotion");
    gSim[0].mode = M_VALID;
    CHECK(runUntil([] { return rearTofReinitCount(0) == 1; }, 20000), "no re-init");
    CHECK(runUntil([] { return rearTofStatusOf(0) != TOF_UNINITIALISED; }, 500),
          "no visit after re-init");
    CHECK(rearTofStatusOf(0) == TOF_VALID, "first post-recovery reading not VALID");
    CHECK(gStaleAtRead == 0, "post-recovery read started with a stale flag");
    CHECK(runUntil([] { return rearTofValid(0); }, 2000), "never valid again");
}

// ---------------------------------------------------------------------------

typedef void (*TestFn)(void);

int main(void)
{
    static const struct { const char *name; TestFn fn; } kTests[] = {
        { "A  50 consecutive BUS_ERRORs demote",          testA },
        { "B  demotion reaches throttled re-init",        testB },
        { "C  valid only after new valid readings",       testC },
        { "D  TIMEOUT not counted, breaks a run",         testD },
        { "E  OUT_OF_RANGE not counted, breaks a run",    testE },
        { "F  recovery keeps the obstacle latch",         testF },
        { "G  demotion clears the sample window",         testG },
        { "H  no retry storm",                            testH },
        { "I  CRITICAL: all demoted, reverse blocked",    testI },
        { "J  boot policy unchanged",                     testJ },
        { "K  TCA select failures not counted",           testK },
        { "L  diagnostics: site + raw code",              testL },
        { "M  stale timeout flag cleared before read",    testM },
    };

    int failedTests = 0;
    for (size_t t = 0; t < sizeof(kTests) / sizeof(kTests[0]); t++) {
        gTest = kTests[t].name;
        int before = gFails;
        kTests[t].fn();
        CHECK(gBadSelect == 0, "VL53L0X accessed without exactly its channel open");
        bool ok = (gFails == before);
        if (!ok) failedTests++;
        printf("  %s  %s\n", ok ? "PASS" : "FAIL", kTests[t].name);
    }

    printf("=== rear ToF recovery: %d tests, %d failed, %d checks, %d check failures ===\n",
           (int)(sizeof(kTests) / sizeof(kTests[0])), failedTests, gChecks, gFails);
    return (gFails == 0) ? 0 : 1;
}

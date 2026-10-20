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
//
//  Built twice by tests/run_host_tests.py: with the shipped configuration
//  (REAR_TOF_RUNTIME_REINIT 0) and with -DREAR_TOF_RUNTIME_REINIT=1. Each test
//  declares which build it belongs to; `test_rear_tof <prefix>` runs only the
//  tests whose name starts with <prefix>, whatever the build (negative
//  controls).
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
static bool    gTcaNackDeselect = false;   // only writes of mask 0 NACK
static uint32_t gTcaDelayMs = 0;           // every TCA transaction takes this long
static uint8_t gTcaMask = 0;

// Every TCA control-byte write ATTEMPT (select or deselect), with its time.
#define TCA_LOG_MAX 4096
static uint32_t gTcaWriteTimes[TCA_LOG_MAX];
static int      gTcaWrites = 0;
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
    if (gTxAddr == TCA9548A_I2C_ADDRESS) {
        if (gTxWrote && gTcaWrites < TCA_LOG_MAX) {
            gTcaWriteTimes[gTcaWrites] = gNow;
        }
        if (gTxWrote) gTcaWrites++;
        gNow += gTcaDelayMs;                        // a slow (or hung) transaction
    }
    if (gTxAddr != TCA9548A_I2C_ADDRESS || gTcaNack) {
        return 2;                                   // address NACK
    }
    if (gTxWrote && gTxByte == 0 && gTcaNackDeselect) {
        return 2;                                   // deselect NACK only
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
    uint32_t initDelayMs;   // clock advance per init(): models hung transactions
    uint32_t readDelayMs;   // clock advance per register/range read (slow txn)
    bool     budgetOk;      // setMeasurementTimingBudget() result (false = the
                            // patched library refusing a zero VCSEL period)
    int      budgetCalls;
};

static SimTof gSim[REAR_TOF_SENSOR_COUNT];
static int    gBadSelect = 0;   // VL53L0X accessed with != 1 channel open
static uint32_t gMaxStepMs = 0; // longest single safetyUpdate() (see step())

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
    gNow += t.initDelayMs;
    *status = t.initOk ? 0 : 2;
    return t.initOk;
}

bool simTofSetBudget(void)
{
    int i = openSensor();
    if (i < 0) {
        gBadSelect++;
        return false;
    }
    gSim[i].budgetCalls++;
    return gSim[i].budgetOk;
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
    gNow += t.readDelayMs;

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
    gTcaNackDeselect = false;
    gTcaDelayMs   = 0;
    gTcaWrites    = 0;
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
        gSim[i].initDelayMs = 0;
        gSim[i].readDelayMs = 0;
        gSim[i].budgetOk    = true;
        gSim[i].budgetCalls = 0;
    }
    rearTofSetReinitPermitted(true);
    gMaxStepMs = 0;
}

// The rear-relevant part of setup(), in setup()'s order.
static void boot(uint32_t startMs = 10)
{
    gNow = startMs;
    roverI2cInit();
    (void)tcaInit();
    safetyInit();
}

// One loop() pass: 1 ms, then safetyUpdate(). The clock only moves inside
// safetyUpdate() if something in it waited -- an init() with initDelayMs
// advances it by exactly that much -- so gMaxStepMs is how long the longest
// single pass blocked the control loop.
static void step(void)
{
    gNow++;
    uint32_t before = gNow;
    safetyUpdate();
    if (gNow - before > gMaxStepMs) {
        gMaxStepMs = gNow - before;
    }
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

    // Since the rear I2C backoff, a failing select is NOT retried every 20 ms
    // poll any more: it is retried 1, 2, 4 s ... later. In 10 s from the first
    // failure that is attempts at +0, +1, +3 and +7 s -- exactly 4 -- and each
    // one enters a backoff. (Before the backoff: >= 100 per sensor.)
    uint32_t total = 0;
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) total += rearTofBusErrTotal(i);
    CHECK(total == 4, "select retried more often than the backoff allows");
    CHECK(rearTofI2cBackoffEntries() == total, "a select failure did not enter a backoff");

    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (rearTofBusErrTotal(i) > 0) {
            CHECK(rearTofLastBusErrSite(i) == TOF_BUSERR_TCA_SELECT, "site");
            CHECK(rearTofLastBusErrCode(i) == 2, "raw code not kept");
            CHECK(rearTofStatusOf(i) == TOF_BUS_ERROR, "status");
            CHECK(rearTofFailStreak(i) == rearTofBusErrTotal(i), "failStreak behaviour changed");
        }
        CHECK(rearTofStatusOf(i) != TOF_VALID, "a sensor still reports VALID with the mux failing");
        CHECK(rearTofBusErrStreak(i) == 0, "select failure counted toward recovery");
        CHECK(rearTofInitialised(i), "select failure demoted a sensor");
        CHECK(rearTofReinitCount(i) == 0, "select failure triggered a re-init");
        CHECK(gSim[i].initCalls == calls[i], "sensor init() called");
        CHECK(rearTofHealthOf(i) == HEALTH_FAULT, "health behaviour changed");
    }
    CHECK(strcmp(tcaStatusName(), "BUS_ERROR") == 0, "TCA status");
    CHECK(rearTofBackendAvailable(), "backend dropped");
    CHECK(safetyReverseBlocked(), "reverse open with the mux failing");

    // The next retry is at +15 s, so sensing resumes within one maximum
    // backoff of the mux recovering -- not on the very next poll.
    gTcaNack = false;
    CHECK(runUntil([] { return allValid(); }, REAR_I2C_BACKOFF_MAX_MS + 2000),
          "sensors did not resume after the mux recovered");
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

// ---------------------------------------------------------------------------
// N. A hung init() that FAILS (CH1, 2026-10-02): one attempt, then a safe fault
//
// initDelayMs models one hung I2C transaction (the ESP-IDF 1 s floor). Real
// init() hangs come from inside the library; only the elapsed time is modelled.
// ---------------------------------------------------------------------------
static void testN(void)
{
    resetSim();
    gSim[1].initOk      = false;
    gSim[1].initDelayMs = 1000;
    boot();

    CHECK(gSim[1].initCalls == 1, "slow failing attempt was retried at once");
    CHECK(gSim[1].budgetCalls == 0, "timing budget set after a failed init");
    CHECK(!rearTofInitialised(1), "slow sensor initialised");
    CHECK(rearTofStatusOf(1) == TOF_INIT_FAILED, "status");
    CHECK(gSim[0].initCalls == 1 && gSim[2].initCalls == 1, "healthy sensors re-tried");
    CHECK(gSim[0].budgetCalls == 1 && gSim[2].budgetCalls == 1, "healthy path changed");

    run(1500);
    CHECK(rearTofValid(0) && rearTofValid(2), "healthy sensors not valid");
    CHECK(!rearTofValid(1), "failed sensor valid");
    CHECK(safetyRearSensorFault(), "rear fault not reported");
    CHECK(safetyReverseBlocked(), "reverse open with a failed rear sensor");

    // Runtime: every throttled re-init is a single attempt as well.
    CHECK(runUntil([] { return rearTofReinitCount(1) == 2; }, 30000), "no re-init");
    CHECK(gSim[1].initCalls == 3, "runtime re-init retried a slow attempt");
    CHECK(rearTofLastReinitResult(1) == TOF_REINIT_FAILED, "re-init result");
    CHECK(safetyReverseBlocked(), "reverse open after failed re-inits");
}

// ---------------------------------------------------------------------------
// O. A hung init() that RETURNS TRUE: not trusted, not configured, not retried
// ---------------------------------------------------------------------------
static void testO(void)
{
    resetSim();
    gSim[1].initOk      = true;
    gSim[1].initDelayMs = 1000;
    boot();

    CHECK(gSim[1].initCalls == 1, "slow attempt was retried at once");
    CHECK(gSim[1].budgetCalls == 0, "slow init's sensor was configured anyway");
    CHECK(!rearTofInitialised(1), "slow init accepted");
    CHECK(rearTofStatusOf(1) == TOF_INIT_FAILED, "status");
    run(1500);
    CHECK(safetyReverseBlocked(), "reverse open with an untrusted sensor");

    // Recovery is kept: once init is fast again, the throttled re-init works.
    gSim[1].initDelayMs = 0;
    CHECK(runUntil([] { return rearTofValid(1); }, 20000), "never recovered");
    CHECK(rearTofLastReinitResult(1) == TOF_REINIT_OK, "re-init result");
    CHECK(gSim[1].budgetCalls == 1, "timing budget not set on the good init");
}

// ---------------------------------------------------------------------------
// P. Timing budget refused (patched library, invalid VCSEL period): INIT_FAILED
// ---------------------------------------------------------------------------
static void testP(void)
{
    resetSim();
    gSim[1].budgetOk = false;
    boot();

    CHECK(gSim[1].initCalls == REAR_TOF_INIT_ATTEMPTS, "fast failures not retried");
    CHECK(gSim[1].budgetCalls == REAR_TOF_INIT_ATTEMPTS, "budget not attempted each time");
    CHECK(!rearTofInitialised(1), "sensor with a refused budget initialised");
    CHECK(rearTofStatusOf(1) == TOF_INIT_FAILED, "status");
    CHECK(gSim[0].budgetCalls == 1 && gSim[2].budgetCalls == 1, "healthy path changed");

    run(1500);
    CHECK(rearTofValid(0) && rearTofValid(2), "healthy sensors disturbed");
    CHECK(safetyRearSensorFault(), "rear fault not reported");
    CHECK(safetyReverseBlocked(), "reverse open");
    int l = 1, r = 1;
    safetyGateMotion(-100, -100, &l, &r);
    CHECK(l == 0 && r == 0, "reverse not gated");
    safetyGateMotion(100, 100, &l, &r);
    CHECK(l == 100 && r == 100, "forward gated by a rear fault");
}

// ---------------------------------------------------------------------------
// Q. Runtime re-init only while permitted (no wheel driven); boot unaffected
// ---------------------------------------------------------------------------
static void testQ(void)
{
    resetSim();
    gSim[1].initOk = false;
    rearTofSetReinitPermitted(false);       // boot init must not depend on it
    boot();
    CHECK(gSim[0].initCalls == 1 && gSim[2].initCalls == 1, "boot init gated");
    CHECK(gSim[1].initCalls == REAR_TOF_INIT_ATTEMPTS, "boot init of sensor 1 gated");
    const int bootCalls = gSim[1].initCalls;

    run(20000);                             // wheels driven: not permitted
    CHECK(rearTofReinitCount(1) == 0, "re-init ran while not permitted");
    CHECK(gSim[1].initCalls == bootCalls, "init() called while not permitted");
    CHECK(safetyReverseBlocked(), "reverse open");
    CHECK(rearTofValid(0) && rearTofValid(2), "healthy sensors disturbed");

    gSim[1].initOk = true;
    rearTofSetReinitPermitted(true);        // stopped: the deferred re-init runs
    CHECK(runUntil([] { return rearTofReinitCount(1) == 1; }, 200),
          "deferred re-init did not run on the first permitted visit");
    CHECK(rearTofLastReinitResult(1) == TOF_REINIT_OK, "re-init result");
    CHECK(runUntil([] { return rearTofValid(1); }, 2000), "never recovered");
}

// ===========================================================================
// RX-01 -- runtime re-init OFF (the shipped configuration). A failed rear
// sensor's init() must never run from loop(), however long it would take.
// Each of these fails with -DREAR_TOF_RUNTIME_REINIT=1: the 5 s init() then
// shows up as a 5000 ms pass in gMaxStepMs.
// ===========================================================================

#define SLOW_INIT_MS 5000       // one hung init(), as long as the 2026-10-02 hang or longer

// Over `ms` of loop passes: did anything block, did reverse ever open while a
// rear sensor was invalid, and how many init() calls reached sensor `idx`?
static void runStationary(uint32_t ms, int idx, int *initCallsOut, int *unguardedOut)
{
    const int before = gSim[idx].initCalls;
    int unguarded = 0;
    rearTofSetReinitPermitted(true);            // wheels stopped: the old trigger
    for (uint32_t i = 0; i < ms; i++) {
        step();
        if (!allValid() && !safetyReverseBlocked()) unguarded++;
    }
    *initCallsOut = gSim[idx].initCalls - before;
    *unguardedOut = unguarded;
}

// ---------------------------------------------------------------------------
// R. Boot-failed sensor whose init() would now hang 5 s: the loop never waits
// ---------------------------------------------------------------------------
static void testR(void)
{
    resetSim();
    gSim[1].initOk = false;                     // dead at boot (fast failure)
    boot();
    CHECK(!rearTofInitialised(1), "setup: sensor 1 initialised");
    run(1500);
    CHECK(rearTofValid(0) && rearTofValid(2), "setup: healthy sensors not valid");

    gSim[1].initDelayMs = SLOW_INIT_MS;         // any further init() hangs 5 s
    gMaxStepMs = 0;
    int calls = 0, unguarded = 0;
    runStationary(60000, 1, &calls, &unguarded);

    CHECK(gMaxStepMs == 0, "a loop pass blocked: init() ran from safetyUpdate()");
    CHECK(calls == 0, "runtime init() reached the failed sensor");
    CHECK(rearTofReinitCount(1) == 0, "runtime re-init counted");
    CHECK(unguarded == 0, "reverse open while the rear sensor was failed");
    CHECK(rearTofStatusOf(1) == TOF_INIT_FAILED, "failure status lost");
    CHECK(rearTofValid(0) && rearTofValid(2), "healthy sensors disturbed");
    CHECK(safetyRearSensorFault(), "rear fault not reported");
    CHECK(strcmp(safetyBlockReason(-100, -100), "REAR_SENSOR_FAULT") == 0, "block reason");
    int l = 1, r = 1;
    safetyGateMotion(-100, -100, &l, &r);
    CHECK(l == 0 && r == 0, "reverse not gated");
    safetyGateMotion(100, 100, &l, &r);
    CHECK(l == 100 && r == 100, "forward gated by the rear fault");
}

// ---------------------------------------------------------------------------
// S. Sensor demoted at runtime (BUS_ERRORs), init() would hang 5 s: no wait,
//    no more polling of it, reverse stays blocked
// ---------------------------------------------------------------------------
static void testS(void)
{
    bootAllValid();
    gSim[1].mode        = M_ERR_READY;
    gSim[1].initOk      = false;
    gSim[1].initDelayMs = SLOW_INIT_MS;
    CHECK(runUntil([] { return !rearTofInitialised(1); }, 20000), "never demoted");
    const uint32_t errsAtDemotion = rearTofBusErrTotal(1);

    gMaxStepMs = 0;
    int calls = 0, unguarded = 0;
    runStationary(60000, 1, &calls, &unguarded);

    CHECK(gMaxStepMs == 0, "a loop pass blocked after demotion");
    CHECK(calls == 0, "runtime init() reached the demoted sensor");
    CHECK(rearTofReinitCount(1) == 0, "runtime re-init counted");
    CHECK(rearTofBusErrTotal(1) == errsAtDemotion, "demoted sensor still being polled");
    CHECK(rearTofStatusOf(1) == TOF_BUS_ERROR, "failure status lost");
    CHECK(unguarded == 0, "reverse open while the rear sensor was failed");
    CHECK(rearTofValid(0) && rearTofValid(2), "healthy sensors disturbed");
    CHECK(safetyReverseBlocked(), "reverse not blocked");
}

// ---------------------------------------------------------------------------
// T. All three demoted: backend stays up, reverse stays blocked, nothing waits
// ---------------------------------------------------------------------------
static void testT(void)
{
    bootAllValid();
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        gSim[i].initOk      = false;
        gSim[i].initDelayMs = SLOW_INIT_MS;
        gSim[i].mode        = M_ERR_READY;
    }
    gMaxStepMs = 0;
    int unavailable = 0, unguarded = 0;
    for (int ms = 0; ms < 60000; ms++) {
        step();
        if (!rearTofBackendAvailable() || !safetyRearSensingAvailable()) unavailable++;
        if (!allValid() && !safetyReverseBlocked()) unguarded++;
    }
    CHECK(gMaxStepMs == 0, "a loop pass blocked");
    CHECK(unavailable == 0, "rear backend became unavailable at runtime");
    CHECK(unguarded == 0, "reverse open while a rear sensor was invalid");
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        CHECK(!rearTofInitialised(i), "sensor not demoted");
        CHECK(gSim[i].initCalls == 1, "runtime init() called");
        CHECK(rearTofReinitCount(i) == 0, "runtime re-init counted");
    }
    CHECK(safetyRearSensorFault(), "rear sensor fault not reported");
    CHECK(strcmp(safetyBlockReason(-100, -100), "REAR_SENSOR_FAULT") == 0, "block reason");
}

// ---------------------------------------------------------------------------
// U. Boot policy: nothing came up at boot -> it never comes up later, and the
//    configured unconfigured-rear policy applies unchanged (see RX-05)
// ---------------------------------------------------------------------------
static void testU(void)
{
    resetSim();
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) gSim[i].initOk = false;
    boot();
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        gSim[i].initOk      = true;             // would now succeed...
        gSim[i].initDelayMs = SLOW_INIT_MS;     // ...after a 5 s hang
    }
    const bool policy = (SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED != 0);

    gMaxStepMs = 0;
    int cameUp = 0, policyChanged = 0;
    for (int ms = 0; ms < 30000; ms++) {
        step();
        if (rearTofBackendAvailable()) cameUp++;
        if (safetyReverseBlocked() != policy) policyChanged++;
    }
    CHECK(gMaxStepMs == 0, "a loop pass blocked");
    CHECK(cameUp == 0, "backend came up without a boot init");
    CHECK(policyChanged == 0, "unconfigured-rear reverse policy changed");
    CHECK(!safetyRearSensorFault(), "boot all-fail reported as a sensor fault");
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        CHECK(gSim[i].initCalls == REAR_TOF_INIT_ATTEMPTS, "runtime init() called");
        CHECK(rearTofStatusOf(i) == TOF_INIT_FAILED, "status");
    }
}

// ---------------------------------------------------------------------------
// V. Demotion keeps the obstacle latch and clears the sample window
// ---------------------------------------------------------------------------
static void testV(void)
{
    resetSim();
    gSim[1].mm = 100;                           // inside REAR_STOP_DISTANCE_MM
    boot();
    run(1500);
    CHECK(safetyRearSensorObstacle(1), "setup: latch not set");
    CHECK(rearTofGoodSampleCount(1) == REAR_TOF_SAMPLE_WINDOW, "setup: window");

    gSim[1].mode = M_ERR_READY;
    int latchLost = 0;
    while (rearTofInitialised(1) && gNow < 30000) {
        step();
        if (!safetyRearSensorObstacle(1)) latchLost++;
    }
    CHECK(!rearTofInitialised(1), "never demoted");
    CHECK(rearTofGoodSampleCount(1) == 0, "window not cleared");
    CHECK(rearTofFilteredMm(1) == REAR_TOF_INVALID_MM, "filtered value kept");

    gSim[1].mode = M_VALID;                     // sensor would answer again...
    gSim[1].mm   = 1000;                        // ...far away
    for (int ms = 0; ms < 30000; ms++) {
        step();
        if (!safetyRearSensorObstacle(1)) latchLost++;
    }
    CHECK(latchLost == 0, "latch cleared by demotion or invalid data");
    CHECK(!rearTofValid(1), "demoted sensor became valid without a boot");
    CHECK(safetyReverseBlocked(), "reverse open");
}

// ---------------------------------------------------------------------------
// W. Hung BOOT init (the 2026-10-02 case): one slow attempt at boot, and never
//    retried from the loop, whether init() reported failure or success
// ---------------------------------------------------------------------------
static void hungBootCase(bool initOk)
{
    resetSim();
    gSim[1].initOk      = initOk;
    gSim[1].initDelayMs = 1000;
    boot();
    CHECK(gSim[1].initCalls == 1, "slow boot attempt was retried at once");
    CHECK(!rearTofInitialised(1), "slow init accepted");
    CHECK(rearTofStatusOf(1) == TOF_INIT_FAILED, "status");

    gSim[1].initDelayMs = SLOW_INIT_MS;
    gMaxStepMs = 0;
    int calls = 0, unguarded = 0;
    runStationary(60000, 1, &calls, &unguarded);
    CHECK(gMaxStepMs == 0, "a loop pass blocked");
    CHECK(calls == 0, "runtime init() retried the hung sensor");
    CHECK(unguarded == 0, "reverse open with an untrusted sensor");
    CHECK(rearTofValid(0) && rearTofValid(2), "healthy sensors not valid");
}

static void testW(void)
{
    hungBootCase(false);
    hungBootCase(true);
}

enum Build { BOTH, REINIT_OFF, REINIT_ON };

// ===========================================================================
// REAR I2C BACKOFF (Phase 2)
// ===========================================================================

// Run until `n` more TCA writes have been attempted (or `maxMs` passes);
// return the time of the last one, or 0 if they never happened.
static uint32_t runUntilTcaWrites(int n, uint32_t maxMs)
{
    int target = gTcaWrites + n;
    for (uint32_t i = 0; i < maxMs && gTcaWrites < target; i++) step();
    return (gTcaWrites >= target && target - 1 < TCA_LOG_MAX) ? gTcaWriteTimes[target - 1] : 0;
}

// X. A persistently failing select backs off 1, 2, 4, 8, 16, 30, 30 s --
//    one attempt per interval, no rear I2C in between.
static void testX(void)
{
    bootAllValid();
    gTcaNack = true;
    uint32_t prev = runUntilTcaWrites(1, 1000);          // the first failing select
    CHECK(prev != 0, "setup: no select attempted");
    static const uint32_t kExpect[] = { 1000, 2000, 4000, 8000, 16000, 30000, 30000 };
    const size_t n = sizeof(kExpect) / sizeof(kExpect[0]);
    for (size_t k = 0; k < n; k++) {
        int before = gTcaWrites;
        uint32_t t = runUntilTcaWrites(1, kExpect[k] + 100);
        CHECK(t != 0, "no retry after the backoff interval");
        CHECK(gTcaWrites == before + 1, "more than one rear transaction per backoff interval");
        uint32_t gap = t - prev;
        char msg[96];
        snprintf(msg, sizeof(msg), "interval %u: %lu ms, expected %lu",
                 (unsigned)k, (unsigned long)gap, (unsigned long)kExpect[k]);
        CHECK(gap >= kExpect[k] && gap <= kExpect[k] + 2, msg);
        uint32_t next = (k + 1 < n) ? kExpect[k + 1] : REAR_I2C_BACKOFF_MAX_MS;
        CHECK(rearTofI2cBackoffIntervalMs() == next, "reported interval wrong");
        prev = t;
    }
    CHECK(rearTofI2cBackoffActive() && rearTofI2cBackoffRemainingMs() > 0, "not reported as backing off");
}

// Y. A failing DESELECT backs off the same way (the select succeeds).
static void testY(void)
{
    bootAllValid();
    gTcaNackDeselect = true;
    CHECK(runUntil([] { return rearTofI2cBackoffActive(); }, 200), "a failed deselect did not back off");
    CHECK(rearTofI2cBackoffEntries() == 1, "entries");
    int w = gTcaWrites;
    run(REAR_I2C_BACKOFF_INITIAL_MS - 50);
    CHECK(gTcaWrites == w, "rear I2C during the backoff");
    run(100);
    CHECK(gTcaWrites == w + 2, "retry is not exactly one select + one deselect");
    CHECK(rearTofI2cBackoffIntervalMs() == 2 * REAR_I2C_BACKOFF_INITIAL_MS, "did not double");
}

// Z. SLOW transactions back off even when they succeed: a VL53L0X read, and
//    a TCA select. Below the threshold nothing happens.
static void testZ(void)
{
    bootAllValid();
    gSim[1].readDelayMs = REAR_I2C_SLOW_MS + 150;       // succeeds, but slowly
    CHECK(runUntil([] { return rearTofI2cBackoffActive(); }, 1000), "slow VL53L0X read did not back off");
    int w = gTcaWrites;
    run(REAR_I2C_BACKOFF_INITIAL_MS - 50);
    CHECK(gTcaWrites == w, "rear I2C during the backoff (slow read)");
    gSim[1].readDelayMs = 0;
    CHECK(runUntil([] { return allValid(); }, REAR_I2C_BACKOFF_MAX_MS + 2000), "no recovery after the slow read cleared");

    bootAllValid();
    gTcaDelayMs = REAR_I2C_SLOW_MS + 150;               // every TCA txn slow, but OK
    CHECK(runUntil([] { return rearTofI2cBackoffActive(); }, 1000), "slow TCA select did not back off");
    gTcaDelayMs = 0;
    CHECK(runUntil([] { return allValid(); }, REAR_I2C_BACKOFF_MAX_MS + 2000), "no recovery after slow TCA cleared");

    // Control: one sensor read is up to three VL53L0X transactions (ready
    // register, range status, range). 20 ms each keeps the read step at
    // ~40 ms, under REAR_I2C_SLOW_MS: normal operation, no backoff.
    bootAllValid();
    gSim[0].readDelayMs = 20;
    run(3000);
    CHECK(rearTofI2cBackoffEntries() == 0, "backed off below the slow threshold");
}

// AA. Validity expires during a backoff on the normal schedule for EVERY
//     sensor (none is visited), and no stale distance is ever reported.
static void testAA(void)
{
    bootAllValid();
    gTcaNack = true;
    CHECK(runUntil([] { return rearTofI2cBackoffActive(); }, 200), "setup: no backoff");
    uint32_t tFault = gNow;
    uint32_t lastValidAge = 0;
    for (uint32_t ms = 0; ms < 5000; ms++) {
        step();
        for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
            if (rearTofValid(i) && gNow - tFault > lastValidAge) lastValidAge = gNow - tFault;
        }
    }
    CHECK(lastValidAge <= REAR_TOF_STALE_MS, "a sensor stayed valid past REAR_TOF_STALE_MS during the backoff");
    for (int i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        CHECK(!rearTofValid(i), "sensor still valid");
        CHECK(rearTofFilteredMm(i) <= 0, "a stale distance is still published");
        CHECK(rearTofHealthOf(i) == HEALTH_FAULT, "health");
    }
}

// AB. Reverse blocked throughout; the forward policy is unchanged.
static void testAB(void)
{
    bootAllValid();
    bool fwdBefore = safetyForwardBlocked();
    CHECK(!safetyReverseBlocked(), "setup: reverse blocked with a healthy rear");
    gTcaNack = true;
    run(REAR_TOF_STALE_MS + 100);
    CHECK(rearTofI2cBackoffActive(), "setup: no backoff");
    CHECK(safetyReverseBlocked(), "reverse open while backing off");
    CHECK(safetyRearSensorFault(), "rear not reported as a fault");
    CHECK(safetyForwardBlocked() == fwdBefore, "rear backoff changed the forward gate");
    run(20000);
    CHECK(safetyReverseBlocked(), "reverse reopened during a long backoff");
    CHECK(safetyForwardBlocked() == fwdBefore, "forward gate changed during a long backoff");
}

// AC. Recovery after the fault clears. A short healthy run keeps doubling; a
//     long one (HEALTHY_RESET) starts again at INITIAL.
static void testAC(void)
{
    bootAllValid();
    gTcaNack = true;
    CHECK(runUntil([] { return rearTofI2cBackoffIntervalMs() == 4000; }, 20000), "setup: backoff did not grow");
    gTcaNack = false;
    CHECK(runUntil([] { return allValid(); }, REAR_I2C_BACKOFF_MAX_MS + 2000), "no recovery");
    CHECK(!safetyReverseBlocked(), "reverse still blocked after recovery");

    run(2000);                                           // short healthy run
    gTcaNack = true;
    CHECK(runUntil([] { return rearTofI2cBackoffActive(); }, 200), "no backoff");
    CHECK(rearTofI2cBackoffIntervalMs() == 8000, "a short healthy run reset the backoff");

    // Recovered = the backoff has ended AND fresh readings made every sensor
    // valid again (right after a fault they are still valid from before it).
    gTcaNack = false;
    CHECK(runUntil([] { return !rearTofI2cBackoffActive() && allValid(); },
                   REAR_I2C_BACKOFF_MAX_MS + 2000), "no recovery 2");
    run(REAR_I2C_BACKOFF_HEALTHY_RESET_MS + 500);        // long healthy run
    gTcaNack = true;
    CHECK(runUntil([] { return rearTofI2cBackoffActive(); }, 200), "no backoff 2");
    CHECK(rearTofI2cBackoffIntervalMs() == REAR_I2C_BACKOFF_INITIAL_MS,
          "a long healthy run did not restart the backoff at INITIAL");
}

// AD. A VL53L0X that fails FAST keeps its per-sensor bound (demotion) and
//     does NOT back off the whole rear: the other two stay in service.
static void testAD(void)
{
    bootAllValid();
    gSim[2].mode = M_ERR_READY;
    // Checked at the moment of demotion (with runtime re-init built in, the
    // demoted sensor is legitimately re-initialised a few seconds later).
    CHECK(runUntil([] { return !rearTofInitialised(2); }, 6000),
          "failing sensor not demoted by its own bound");
    CHECK(rearTofI2cBackoffEntries() == 0, "a fast sensor failure backed off the whole rear");
    CHECK(rearTofValid(0) && rearTofValid(1), "healthy sensors blinded");
}

// AE. Wrap-safe: the same schedule across a millis() rollover.
static void testAE(void)
{
    resetSim();
    boot(0xFFFFFFFFUL - 4000UL);                         // wraps ~4 s in
    run(1500);
    CHECK(allValid(), "setup: not valid near the wrap");
    gTcaNack = true;
    uint32_t prev = runUntilTcaWrites(1, 1000);
    static const uint32_t kExpect[] = { 1000, 2000, 4000 };   // crosses the wrap
    for (size_t k = 0; k < 3; k++) {
        uint32_t t = runUntilTcaWrites(1, kExpect[k] + 100);
        uint32_t gap = t - prev;
        CHECK(gap >= kExpect[k] && gap <= kExpect[k] + 2, "wrong interval across the millis() wrap");
        prev = t;
    }
    gTcaNack = false;
    CHECK(runUntil([] { return allValid(); }, REAR_I2C_BACKOFF_MAX_MS + 2000), "no recovery across the wrap");
}

int main(int argc, char **argv)
{
    static const struct { const char *name; TestFn fn; Build build; } kTests[] = {
        { "A  50 consecutive BUS_ERRORs demote",          testA, BOTH },
        { "B  demotion reaches throttled re-init",        testB, REINIT_ON },
        { "C  valid only after new valid readings",       testC, REINIT_ON },
        { "D  TIMEOUT not counted, breaks a run",         testD, BOTH },
        { "E  OUT_OF_RANGE not counted, breaks a run",    testE, BOTH },
        { "F  recovery keeps the obstacle latch",         testF, REINIT_ON },
        { "G  demotion clears the sample window",         testG, REINIT_ON },
        { "H  no retry storm",                            testH, REINIT_ON },
        { "I  CRITICAL: all demoted, reverse blocked",    testI, REINIT_ON },
        { "J  boot policy unchanged",                     testJ, REINIT_ON },
        { "K  TCA select failures not counted",           testK, BOTH },
        { "L  diagnostics: site + raw code",              testL, BOTH },
        { "M  stale timeout flag cleared before read",    testM, REINIT_ON },
        { "N  hung init fails: one attempt, safe fault",  testN, REINIT_ON },
        { "O  hung init 'succeeds': rejected",            testO, REINIT_ON },
        { "P  refused timing budget: safe fault",         testP, BOTH },
        { "Q  re-init only while permitted",              testQ, REINIT_ON },
        { "R  RX-01: boot-failed, 5 s init never runs",   testR, REINIT_OFF },
        { "S  RX-01: demoted, 5 s init never runs",       testS, REINIT_OFF },
        { "T  RX-01: all demoted, reverse blocked",       testT, REINIT_OFF },
        { "U  RX-01: boot all-fail policy unchanged",     testU, REINIT_OFF },
        { "V  RX-01: demotion keeps latch, clears window", testV, REINIT_OFF },
        { "W  RX-01: hung boot init never retried",       testW, REINIT_OFF },
        { "X  backoff: failing select 1,2,4,8,16,30,30 s", testX, BOTH },
        { "Y  backoff: failing deselect",                 testY, BOTH },
        { "Z  backoff: slow VL53/TCA txn; control",       testZ, BOTH },
        { "AA backoff: validity expires, no stale data",  testAA, BOTH },
        { "AB backoff: reverse blocked, forward same",    testAB, BOTH },
        { "AC backoff: recovery + healthy-run reset",     testAC, BOTH },
        { "AD fast VL53 failure: per-sensor, no backoff", testAD, BOTH },
        { "AE backoff: millis() wraparound",              testAE, BOTH },
    };
    const Build thisBuild = REAR_TOF_RUNTIME_REINIT ? REINIT_ON : REINIT_OFF;
    const char *only = (argc > 1) ? argv[1] : NULL;

    printf("=== rear ToF build: REAR_TOF_RUNTIME_REINIT=%d%s%s ===\n",
           (int)REAR_TOF_RUNTIME_REINIT, only ? ", only " : "", only ? only : "");

    int ran = 0, failedTests = 0;
    for (size_t t = 0; t < sizeof(kTests) / sizeof(kTests[0]); t++) {
        if (only != NULL) {
            if (strncmp(kTests[t].name, only, strlen(only)) != 0) continue;
        } else if (kTests[t].build != BOTH && kTests[t].build != thisBuild) {
            continue;
        }
        gTest = kTests[t].name;
        int before = gFails;
        kTests[t].fn();
        CHECK(gBadSelect == 0, "VL53L0X accessed without exactly its channel open");
        bool ok = (gFails == before);
        if (!ok) failedTests++;
        ran++;
        printf("  %s  %s\n", ok ? "PASS" : "FAIL", kTests[t].name);
    }

    printf("=== rear ToF recovery: %d tests, %d failed, %d checks, %d check failures ===\n",
           ran, failedTests, gChecks, gFails);
    return (gFails == 0 && ran > 0) ? 0 : 1;
}

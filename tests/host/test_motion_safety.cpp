// ============================================================================
//  FOCUSED HOST TESTS -- motor stop ordering, reverse policy, diagnostics guard
//
//  REAL firmware under test: robot_controller.ino (setup/loop), comm.cpp,
//  protocol.cpp, motor.cpp, safety.cpp, rear_tof.cpp, tca9548a.cpp,
//  rover_i2c.cpp, gps.cpp and config.h. Simulated below them, in-process and
//  on a MANUAL clock (every loop() pass's delay(1) advances it 1 ms), so every
//  result is deterministic:
//
//    * stall guard -- its core-0 task ticks every STALL_GUARD_PERIOD_MS of
//                     simulated time (advance())
//    * GPIO        -- digitalWrite() on the eight L298N direction pins is
//                     recorded, in order, in one event log with the PCA9685
//    * PCA9685     -- pca9685.cpp replaced (as in sim_hw.cpp): SetDuty/AllOff
//                     recorded; a channel can be made to fail, which latches
//                     BUS_ERROR exactly as pca9685.cpp does
//    * UART        -- commands are queued into Serial; everything the firmware
//                     transmits is kept, so ACKs and READY/DIAG can be read
//    * HC-SR04     -- pulseIn() echoes gFrontCm
//    * I2C bus     -- only the TCA9548A answers (when present), at the
//                     configured address; 3 scriptable VL53L0X behind it
//
//  What this CANNOT show: real GPIO/I2C timing, L298N behaviour, or how long a
//  real stop takes. GPIO-first ORDER is proven here; physical stopping
//  performance needs a bench test with the bus held (SDA low) and a scope.
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

#include "Arduino.h"
#include "Wire.h"
#include "VL53L0X.h"
#include "pca9685.h"
#include "config.h"
#include "comm.h"
#include "motor.h"
#include "safety.h"
#include "rear_tof.h"
#include "protocol.h"
#include "stall_guard.h"

void setup(void);
void loop(void);

// ---------------------------------------------------------------------------
// Event log
// ---------------------------------------------------------------------------

enum EvKind { EV_PASS = 'M', EV_GPIO = 'G', EV_DUTY = 'P', EV_ALLOFF = 'A' };

struct Ev {
    char kind;
    int  a;       // pin / channel
    int  b;       // level / duty (-1: the write FAILED)
};

static std::vector<Ev> gLog;

static const uint8_t kMotorPins[8] = {
    M_FRONT_A_IN1, M_FRONT_A_IN2, M_FRONT_B_IN1, M_FRONT_B_IN2,
    M_REAR_A_IN1,  M_REAR_A_IN2,  M_REAR_B_IN1,  M_REAR_B_IN2
};
static const uint8_t kEnableCh[4] = {
    PCA_CH_FRONT_A_EN, PCA_CH_FRONT_B_EN, PCA_CH_REAR_A_EN, PCA_CH_REAR_B_EN
};

static int  gPin[64];             // current level of every GPIO
static int  gDuty[16];            // current PCA9685 duty per channel

static bool isMotorPin(int pin)
{
    for (int i = 0; i < 8; i++) if (kMotorPins[i] == pin) return true;
    return false;
}

static int enableIndexOfPin(int pin)       // motor channel 0..3 owning a pin
{
    for (int i = 0; i < 8; i++) if (kMotorPins[i] == pin) return i / 2;
    return -1;
}

// ---------------------------------------------------------------------------
// Arduino core (manual clock)
// ---------------------------------------------------------------------------

static uint32_t gNow    = 0;
static double   gFrontCm = 150.0;

// The loop-stall guard task, modelled: it runs on its own core, so it ticks
// every STALL_GUARD_PERIOD_MS of simulated time wherever the clock advances.
static void advance(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        gNow++;
        if ((gNow % STALL_GUARD_PERIOD_MS) == 0) (void)stallGuardTick(gNow);
    }
}

uint32_t millis(void)                         { return gNow; }
uint32_t micros(void)                         { return gNow * 1000UL; }
void     delay(uint32_t ms)                   { advance(ms); }
void     delayMicroseconds(uint32_t us)       { (void)us; }
void     pinMode(uint8_t pin, uint8_t mode)   { (void)pin; (void)mode; }
int      digitalRead(uint8_t pin)             { return gPin[pin & 63]; }

void digitalWrite(uint8_t pin, uint8_t v)
{
    gPin[pin & 63] = v ? HIGH : LOW;
    if (isMotorPin(pin)) {
        gLog.push_back(Ev{ EV_GPIO, pin, v ? HIGH : LOW });
    }
}

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeoutUs)
{
    (void)pin; (void)state;
    if (gFrontCm <= 0.0) return 0;
    unsigned long us = (unsigned long)(gFrontCm / 0.01715);
    return (us > timeoutUs) ? 0 : us;
}

// ---------------------------------------------------------------------------
// UART
// ---------------------------------------------------------------------------

SimSerial Serial;
static std::string gRx;
static size_t      gRxPos = 0;
static std::string gTx;

int SimSerial::available(void) { return (int)(gRx.size() - gRxPos); }
int SimSerial::read(void)      { return (gRxPos < gRx.size()) ? (uint8_t)gRx[gRxPos++] : -1; }

size_t SimSerial::write(const uint8_t *buf, size_t len)
{
    gTx.append((const char *)buf, len);
    return len;
}

// ---------------------------------------------------------------------------
// I2C: only the TCA9548A, and the VL53L0X behind it
// ---------------------------------------------------------------------------

TwoWire Wire;
static bool    gTcaPresent = true;
static uint8_t gTcaMask    = 0;
static int     gTxAddr     = -1;
static bool    gWrote      = false;
static uint8_t gLastByte   = 0;
static int     gRxAvail    = 0;
static uint8_t gRxByte     = 0;

bool TwoWire::begin(int sda, int scl, uint32_t f) { (void)sda; (void)scl; (void)f; return true; }
void TwoWire::beginTransmission(int address)       { gTxAddr = address; gWrote = false; }
size_t TwoWire::write(uint8_t b)                    { gLastByte = b; gWrote = true; return 1; }

uint8_t TwoWire::endTransmission(bool sendStop)
{
    (void)sendStop;
    if (!(gTcaPresent && gTxAddr == TCA9548A_I2C_ADDRESS)) return 2;
    if (gWrote) gTcaMask = gLastByte;
    return 0;
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
    if (!(gTcaPresent && address == TCA9548A_I2C_ADDRESS) || quantity < 1) {
        gRxAvail = 0;
        return 0;
    }
    gRxByte  = gTcaMask;
    gRxAvail = 1;
    return 1;
}

int TwoWire::available(void) { return gRxAvail; }
int TwoWire::read(void)      { if (gRxAvail <= 0) return -1; gRxAvail--; return gRxByte; }

struct RearSim {
    bool     initOk;
    bool     reads;        // false: every register access NACKs
    uint16_t mm;
    uint32_t initDelayMs;  // clock advance per init() (a hung init)
    int      initCalls;
};
static RearSim gRear[3];

static int openRearIndex(void)
{
    uint8_t m = gTcaMask;
    if (m == 0 || (m & (m - 1)) != 0) return -1;
    const uint8_t ch[3] = { TCA_CH_REAR_TOF_0, TCA_CH_REAR_TOF_1, TCA_CH_REAR_TOF_2 };
    for (int i = 0; i < 3; i++) if (m == (uint8_t)(1u << ch[i])) return i;
    return -1;
}

bool simVl53Init(void)
{
    int i = openRearIndex();
    if (i < 0) return false;
    gRear[i].initCalls++;
    advance(gRear[i].initDelayMs);
    return gRear[i].initOk;
}

uint8_t simVl53ReadReg(uint8_t reg, uint8_t *status)
{
    int i = openRearIndex();
    if (i < 0 || !gRear[i].reads) { *status = 2; return 0xFF; }
    *status = 0;
    if (reg == VL53L0X::RESULT_INTERRUPT_STATUS) return 0x07;
    if (reg == VL53L0X::RESULT_RANGE_STATUS)     return (uint8_t)(11 << 3);
    return 0;
}

uint16_t simVl53ReadRange(uint8_t *status, bool *timedOut)
{
    (void)timedOut;
    int i = openRearIndex();
    if (i < 0 || !gRear[i].reads) { *status = 2; return 0xFFFF; }
    *status = 0;
    return gRear[i].mm;
}

// ---------------------------------------------------------------------------
// PCA9685 (replaces pca9685.cpp; same ready / BUS_ERROR semantics)
// ---------------------------------------------------------------------------

static bool gPcaReady  = true;
static bool gPcaBusErr = false;
static int  gPcaFailCh = -1;      // SetDuty on this channel fails

bool        pca9685Init(void)             { return gPcaReady; }
bool        pca9685AddressConfirmed(void) { return true; }
uint8_t     pca9685Address(void)          { return 0x40; }
bool        pca9685Ready(void)            { return gPcaReady && !gPcaBusErr; }
const char *pca9685StatusName(void)       { return gPcaBusErr ? "BUS_ERROR" : (gPcaReady ? "OK" : "INIT_FAILED"); }

bool pca9685SetDuty(uint8_t ch, uint16_t duty)
{
    if (!pca9685Ready() || ch > 15) return false;     // no transaction attempted
    if ((int)ch == gPcaFailCh) {
        gLog.push_back(Ev{ EV_DUTY, ch, -1 });
        gPcaBusErr = true;
        return false;
    }
    gLog.push_back(Ev{ EV_DUTY, ch, (int)duty });
    gDuty[ch] = duty;
    return true;
}

bool pca9685AllOff(void)
{
    if (!gPcaReady) return false;
    gLog.push_back(Ev{ EV_ALLOFF, -1, 0 });
    for (int i = 0; i < 16; i++) gDuty[i] = 0;
    return true;
}

bool pca9685ProbeCandidate(uint8_t address, uint8_t *mode1Out, uint8_t *prescaleOut)
{
    (void)address;
    if (mode1Out)    *mode1Out    = 0;
    if (prescaleOut) *prescaleOut = 0;
    return false;
}

// Runtime recovery never succeeds here, so a latched BUS_ERROR stays latched
// (what A8 / B2 rely on). The real recovery is tested in test_pca_recovery.cpp.
bool        pca9685ServiceRecovery(void)  { return false; }
const char *pca9685RecoveryName(void)     { return gPcaBusErr ? "RETRYING" : "NONE"; }
const char *pca9685LastFaultName(void)    { return gPcaBusErr ? "RUNTIME_BUS" : "NONE"; }
uint16_t    pca9685RecoveryAttempts(void) { return 0; }
uint16_t    pca9685Recoveries(void)       { return 0; }
uint16_t    pca9685RuntimeBusErrors(void) { return gPcaBusErr ? 1 : 0; }
uint32_t    pca9685NextRetryInMs(void)    { return 0; }

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
            printf("  FAIL [%s] line %d: %s -- %s\n", gTest, __LINE__, #cond, msg); \
        }                                                                     \
    } while (0)

static int gSeq = 100;

static void pass(int n = 1)
{
    for (int i = 0; i < n; i++) {
        gLog.push_back(Ev{ EV_PASS, 0, 0 });
        loop();
    }
}

// Queue one COMMAND frame; returns its seq. `fields` is e.g. ",\"left\":120,\"right\":120".
static int send(const char *cmd, const char *fields = "")
{
    char payload[200];
    int seq = ++gSeq;
    snprintf(payload, sizeof(payload), "{\"type\":\"COMMAND\",\"seq\":%d,\"cmd\":\"%s\"%s}",
             seq, cmd, fields);
    char frame[220];
    snprintf(frame, sizeof(frame), "%s*%04X\n", payload,
             (unsigned)protoCrc16(payload, strlen(payload)));
    gRx += frame;
    return seq;
}

// The transmitted frame that answers `seq`, or "".
static std::string ackFor(int seq)
{
    char key[32];
    snprintf(key, sizeof(key), "\"seq\":%d,", seq);
    size_t start = 0;
    std::string found;
    while (start < gTx.size()) {
        size_t end = gTx.find('\n', start);
        if (end == std::string::npos) end = gTx.size();
        std::string line = gTx.substr(start, end - start);
        if ((line.find("\"type\":\"ACK\"") != std::string::npos ||
             line.find("\"type\":\"ERROR\"") != std::string::npos) &&
            line.find(key) != std::string::npos) {
            found = line;
        }
        start = end + 1;
    }
    return found;
}

static bool has(const std::string &s, const char *needle) { return s.find(needle) != std::string::npos; }

// Send, run a few passes, and return the answer.
static std::string command(const char *cmd, const char *fields = "")
{
    int seq = send(cmd, fields);
    pass(3);
    return ackFor(seq);
}

static bool motorPinsAllLow(void)
{
    for (int i = 0; i < 8; i++) if (gPin[kMotorPins[i]] != LOW) return false;
    return true;
}

static bool enablesAllZero(void)
{
    for (int i = 0; i < 4; i++) if (gDuty[kEnableCh[i]] != 0) return false;
    return true;
}

// Fresh firmware state. Fakes reset; setup(); then enough passes for the front
// and rear sensor windows to fill.
static void boot(bool tcaPresent = true, bool rearInitOk = true)
{
    gLog.clear(); gRx.clear(); gRxPos = 0; gTx.clear();
    for (int i = 0; i < 64; i++) gPin[i] = LOW;
    for (int i = 0; i < 16; i++) gDuty[i] = 0;
    gNow = 0; gFrontCm = 150.0;
    gTcaPresent = tcaPresent; gTcaMask = 0;
    gPcaReady = true; gPcaBusErr = false; gPcaFailCh = -1;
    for (int i = 0; i < 3; i++) gRear[i] = RearSim{ rearInitOk, true, 1000, 0, 0 };
    setup();
    pass(1500);
}

static void startDriving(int l, int r)
{
    char f[64];
    snprintf(f, sizeof(f), ",\"left\":%d,\"right\":%d", l, r);
    std::string a = command("DRIVE", f);
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    pass(20);
    CHECK(commAppliedLeft() == l && commAppliedRight() == r, "setup: not driving");
    CHECK(!motorPinsAllLow() && !enablesAllZero(), "setup: no motor output");
}

// Find the first pass, at or after event index `from`, that ends with every
// direction pin LOW, and check the GPIO-first rule inside it: at the first PCA
// operation of that pass, every direction pin that was HIGH when the pass began
// is already LOW. Also: no direction pin is driven HIGH in that pass.
static void checkStopPass(size_t from, const char *path)
{
    int level[64];
    for (int i = 0; i < 64; i++) level[i] = gPin[i];
    // Re-derive levels at `from` by replaying the log from the start.
    for (int i = 0; i < 64; i++) level[i] = LOW;
    for (size_t k = 0; k < from; k++) if (gLog[k].kind == EV_GPIO) level[gLog[k].a] = gLog[k].b;

    size_t k = from;
    bool found = false;
    while (k < gLog.size()) {
        // one pass: from a marker to the next
        size_t begin = k;
        size_t end = k + 1;
        while (end < gLog.size() && gLog[end].kind != EV_PASS) end++;

        int atStart[64];
        for (int i = 0; i < 64; i++) atStart[i] = level[i];
        int cur[64];
        for (int i = 0; i < 64; i++) cur[i] = level[i];
        bool sawPca = false, orderOk = true, wentHigh = false;
        for (size_t e = begin; e < end; e++) {
            const Ev &ev = gLog[e];
            if (ev.kind == EV_GPIO) {
                if (ev.b == HIGH) wentHigh = true;
                cur[ev.a] = ev.b;
            } else if ((ev.kind == EV_DUTY || ev.kind == EV_ALLOFF) && !sawPca) {
                sawPca = true;
                for (int p = 0; p < 8; p++) {
                    int pin = kMotorPins[p];
                    if (atStart[pin] == HIGH && cur[pin] != LOW) orderOk = false;
                }
            }
        }
        bool allLow = true;
        for (int p = 0; p < 8; p++) if (cur[kMotorPins[p]] != LOW) allLow = false;
        bool anyHighAtStart = false;
        for (int p = 0; p < 8; p++) if (atStart[kMotorPins[p]] == HIGH) anyHighAtStart = true;

        for (int i = 0; i < 64; i++) level[i] = cur[i];
        if (allLow && anyHighAtStart) {
            char msg[160];
            snprintf(msg, sizeof(msg), "%s: an I2C write came before the GPIO cutoff", path);
            CHECK(orderOk, msg);
            snprintf(msg, sizeof(msg), "%s: a direction pin was driven HIGH in the stop pass", path);
            CHECK(!wentHigh, msg);
            found = true;
            break;
        }
        k = end;
    }
    char msg[120];
    snprintf(msg, sizeof(msg), "%s: no pass stopped the motors", path);
    CHECK(found, msg);
}

// ===========================================================================
// A. GPIO-first ordering in every stop path
// ===========================================================================

static void testA_explicitStop(void)
{
    boot();
    startDriving(150, 150);
    size_t from = gLog.size();
    std::string a = command("STOP");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    checkStopPass(from, "STOP");
    CHECK(motorPinsAllLow() && enablesAllZero(), "not stopped after STOP");
}

static void testA_watchdog(void)
{
    boot();
    startDriving(150, 150);
    size_t from = gLog.size();
    pass(COMMAND_TIMEOUT_MS + 100);          // no command: the watchdog fires
    CHECK(commTimedOut(), "watchdog did not fire");
    checkStopPass(from, "watchdog");
    CHECK(motorPinsAllLow() && enablesAllZero(), "not stopped after watchdog");
}

static void testA_frontGate(void)
{
    boot();
    startDriving(150, 150);
    size_t from = gLog.size();
    gFrontCm = 20.0;                         // obstacle appears while driving
    pass(1000);
    CHECK(safetyObstacleDetected(), "front obstacle not detected");
    checkStopPass(from, "front obstacle gate");
    CHECK(motorPinsAllLow() && enablesAllZero(), "not stopped by the front gate");
    CHECK(commSafetyStopLatched(), "safety stop not latched");
}

static void testA_rearGate(void)
{
    boot();
    startDriving(-150, -150);                // reverse, rear valid and clear
    size_t from = gLog.size();
    gRear[0].mm = 100;                       // obstacle behind
    pass(1000);
    CHECK(safetyRearObstacleDetected(), "rear obstacle not detected");
    checkStopPass(from, "rear obstacle gate");
    CHECK(motorPinsAllLow() && enablesAllZero(), "not stopped by the rear gate");
}

static void testA_reset(void)
{
    boot();
    startDriving(150, 150);
    size_t from = gLog.size();
    std::string a = command("RESET");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    checkStopPass(from, "RESET");
    CHECK(motorPinsAllLow() && enablesAllZero(), "not stopped after RESET");
}

static void testA_motortestEnd(void)
{
    boot();
    std::string a = command("MOTORTEST", ",\"motor\":0,\"power\":150,\"ms\":500,\"onblocks\":true");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    pass(50);
    CHECK(commTestModeActive() && !motorPinsAllLow(), "setup: MOTORTEST not driving");
    size_t from = gLog.size();
    pass(600);                               // expires on its own
    CHECK(!commTestModeActive(), "MOTORTEST did not expire");
    checkStopPass(from, "MOTORTEST expiry");
    CHECK(motorPinsAllLow() && enablesAllZero(), "not stopped after MOTORTEST");
}

static void testA_driveToZero(void)
{
    boot();
    startDriving(150, 150);
    size_t from = gLog.size();
    std::string a = command("DRIVE", ",\"left\":0,\"right\":0");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    checkStopPass(from, "DRIVE 0/0");
    CHECK(motorPinsAllLow() && enablesAllZero(), "not stopped by DRIVE 0/0");
}

static void testA_pcaFailure(void)
{
    boot();
    startDriving(150, 150);
    size_t from = gLog.size();
    gPcaFailCh = kEnableCh[2];               // the bus starts failing
    pass(1);
    // Channels 0 and 1 were rewritten (driven) earlier in this very pass: they
    // must be cut by the motor layer now, not by the next pass.
    CHECK(motorPinsAllLow(), "channels written before the failed one were not cut in the failing pass");

    // The failing pass is a DRIVING pass (it legitimately rewrites directions
    // and duties before the failure), so the stop-pass rule does not apply to
    // it. What must hold: after the failed write, no further PCA operation and
    // no direction pin driven HIGH until every direction pin is LOW.
    int level[64];
    for (int i = 0; i < 64; i++) level[i] = LOW;
    size_t failAt = gLog.size();
    for (size_t e = 0; e < gLog.size(); e++) {
        if (e >= from && gLog[e].kind == EV_DUTY && gLog[e].b == -1) { failAt = e; break; }
        if (gLog[e].kind == EV_GPIO) level[gLog[e].a] = gLog[e].b;
    }
    CHECK(failAt < gLog.size(), "harness: the failed write was not logged");
    bool allLow = false, pcaBeforeCut = false, highAfterFail = false;
    for (size_t e = failAt + 1; e < gLog.size() && !allLow; e++) {
        const Ev &ev = gLog[e];
        if (ev.kind == EV_GPIO) {
            if (ev.b == HIGH) highAfterFail = true;
            level[ev.a] = ev.b;
            allLow = true;
            for (int p = 0; p < 8; p++) if (level[kMotorPins[p]] != LOW) allLow = false;
        } else if (ev.kind == EV_DUTY || ev.kind == EV_ALLOFF) {
            pcaBeforeCut = true;
        }
    }
    CHECK(allLow, "direction pins never all LOW after the failed write");
    CHECK(!pcaBeforeCut, "a PCA9685 operation ran after the failed write, before the cut");
    CHECK(!highAfterFail, "a direction pin was driven HIGH after the failed write");

    pass(4);
    CHECK(!motorDriveAvailable(), "PCA failure did not make drive unavailable");
    CHECK(motorPinsAllLow(), "direction pins not LOW after a PCA failure");
}

// ===========================================================================
// B. Direction changes never happen on a live enable
// ===========================================================================

static void testB_reversal(void)
{
    boot();
    startDriving(150, 150);
    size_t from = gLog.size();
    std::string a = command("DRIVE", ",\"left\":-150,\"right\":-150");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    // Every HIGH written after `from` must follow (a) a LOW cut of both of that
    // channel's pins and (b) a successful duty-0 write to that channel.
    bool cut[4] = { false, false, false, false };
    bool off[4] = { false, false, false, false };
    int  lowSeen[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    int  bad = 0;
    for (size_t e = from; e < gLog.size(); e++) {
        const Ev &ev = gLog[e];
        if (ev.kind == EV_GPIO) {
            int ch = enableIndexOfPin(ev.a);
            int idx = -1;
            for (int i = 0; i < 8; i++) if (kMotorPins[i] == ev.a) idx = i;
            if (ev.b == LOW) {
                lowSeen[idx] = 1;
                if (lowSeen[ch * 2] && lowSeen[ch * 2 + 1]) cut[ch] = true;
            } else if (!(cut[ch] && off[ch])) {
                bad++;
            }
        } else if (ev.kind == EV_DUTY && ev.b == 0) {
            for (int c = 0; c < 4; c++) if (kEnableCh[c] == ev.a) off[c] = true;
        }
    }
    CHECK(bad == 0, "a direction pin went HIGH before its channel was cut and its enable zeroed");
    CHECK(commAppliedLeft() == -150 && commAppliedRight() == -150, "reverse not applied");
}

static void testB_reversalEnableFails(void)
{
    boot();
    startDriving(150, 150);
    gPcaFailCh = kEnableCh[0];               // channel 0's enable cannot be zeroed
    size_t from = gLog.size();
    send("DRIVE", ",\"left\":-150,\"right\":-150");
    pass(1);
    int highs = 0;
    for (size_t e = from; e < gLog.size(); e++) {
        if (gLog[e].kind == EV_GPIO && gLog[e].b == HIGH) highs++;
    }
    CHECK(highs == 0, "a direction was set although the enable could not be zeroed");
    CHECK(motorPinsAllLow(), "channels not held stopped after the failed enable write");
}

// ===========================================================================
// C. Reverse policy (RX-05) and the existing gates, through the command path
// ===========================================================================

static void expectReverseGated(const char *reason)
{
    char need[96];
    snprintf(need, sizeof(need), "\"reason\":\"%s\"", reason);
    std::string a = command("DRIVE", ",\"left\":-120,\"right\":-120");
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, need), a.c_str());
    CHECK(has(a, "\"applied_left\":0,\"applied_right\":0"), a.c_str());
    a = command("DRIVE", ",\"left\":120,\"right\":-120");          // pivot
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, need), a.c_str());
    pass(20);
    CHECK(motorPinsAllLow() && enablesAllZero(), "reverse produced motor output");
    CHECK(safetyReverseBlocked(), "reverse not blocked");
    // Forward is unaffected by the rear: a clean forward DRIVE is accepted.
    a = command("DRIVE", ",\"left\":120,\"right\":120");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    command("STOP");
}

static void testC_tcaAbsent(void)
{
    boot(false, true);
    CHECK(!safetyRearSensingAvailable(), "rear available without a TCA");
    expectReverseGated("REAR_UNCONFIGURED");
}

static void testC_allInitFail(void)
{
    boot(true, false);
    CHECK(!safetyRearSensingAvailable(), "rear available with every init failed");
    expectReverseGated("REAR_UNCONFIGURED");
}

static void testC_invalidAfterBoot(void)
{
    boot();
    CHECK(!safetyReverseBlocked(), "setup: reverse blocked with valid rear sensing");
    gRear[1].reads = false;                  // ch1 stops answering after boot
    pass(1500);
    CHECK(safetyRearSensorFault(), "rear fault not reported");
    expectReverseGated("REAR_SENSOR_FAULT");
}

static void testC_validRear(void)
{
    boot();
    CHECK(safetyRearSensingAvailable() && !safetyRearSensorFault(), "setup: rear not valid");
    std::string a = command("DRIVE", ",\"left\":-120,\"right\":-120");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    CHECK(commAppliedLeft() == -120 && commAppliedRight() == -120, "valid rear did not permit reverse");
    command("STOP");
    gRear[2].mm = 100;                       // obstacle behind
    pass(1000);
    expectReverseGated("REAR_OBSTACLE");
}

static void testC_frontGateAndLatch(void)
{
    boot();
    gFrontCm = 20.0;
    pass(1000);
    std::string a = command("DRIVE", ",\"left\":120,\"right\":120");
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, "\"reason\":\"FRONT_OBSTACLE\""), a.c_str());
    CHECK(commSafetyStopLatched(), "safety stop not latched");
    command("STOP");
    CHECK(commSafetyStopLatched(), "STOP cleared the safety latch");
    a = command("RESET");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    CHECK(!commSafetyStopLatched(), "RESET did not clear the latch");
    gFrontCm = 150.0;
    pass(1000);
    a = command("DRIVE", ",\"left\":120,\"right\":120");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());
    command("STOP");
}

// C6. Rear I2C backoff at the command level: the mux fails while reversing ->
//     rear invalid -> reverse gated and the command discarded; forward is
//     unaffected; after the mux recovers, nothing moves until a NEW command.
static void testC_rearBackoffNoReplay(void)
{
    boot();
    startDriving(-150, -150);                // reverse, rear valid and clear
    gTcaPresent = false;                     // the mux stops answering
    pass(REAR_TOF_STALE_MS + 400);
    CHECK(rearTofI2cBackoffActive(), "mux failure did not back off the rear");
    CHECK(safetyReverseBlocked(), "reverse open with the rear backing off");
    CHECK(motorPinsAllLow() && enablesAllZero(), "still reversing with no rear sensing");
    CHECK(commDesiredLeft() == 0 && commDesiredRight() == 0, "the reverse command was kept");

    // Forward does not depend on the rear: still accepted during the backoff.
    std::string a = command("DRIVE", ",\"left\":120,\"right\":120");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());   // forward refused during a rear backoff
    command("STOP");

    gTcaPresent = true;                      // the mux recovers
    for (int i = 0; i < 40 && safetyReverseBlocked(); i++) pass(1000);
    CHECK(!safetyReverseBlocked(), "rear sensing did not resume after the mux recovered");
    CHECK(motorPinsAllLow() && enablesAllZero() && commAppliedLeft() == 0,
          "a stale command replayed after the rear recovered");
    a = command("DRIVE", ",\"left\":-120,\"right\":-120");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a.c_str());   // new reverse refused after recovery
    command("STOP");
}

// ===========================================================================
// D. TOFTEST is refused while the rover is under motion control
// ===========================================================================

static int totalRearInitCalls(void) { return gRear[0].initCalls + gRear[1].initCalls + gRear[2].initCalls; }

static void testD_toftest(void)
{
    boot(true, true);
    gRear[1].reads = false;                  // a failing sensor...
    pass(5000);                              // ...demoted after 50 BUS_ERRORs,
    CHECK(!rearTofInitialised(1), "setup: failing sensor not demoted");
    gRear[1].initOk = false;
    gRear[1].initDelayMs = 5000;             // ...so TOFTEST would init() it: a 5 s hang

    // 1. While the Pi is streaming STOPs (watchdog armed).
    command("STOP");
    int calls = totalRearInitCalls();
    uint32_t t0 = gNow;
    std::string a = command("TOFTEST");
    CHECK(has(a, "\"result\":\"REJECTED\"") && has(a, "REFUSED_MOTION_CONTROL_ACTIVE"), a.c_str());
    CHECK(totalRearInitCalls() == calls, "TOFTEST ran init() under motion control");
    CHECK(gNow - t0 <= 5, "TOFTEST blocked the loop under motion control");

    // 2. While driving.
    startDriving(120, 120);
    a = command("TOFTEST", ",\"sensor\":1");
    CHECK(has(a, "REFUSED_MOTION_CONTROL_ACTIVE"), a.c_str());
    CHECK(commAppliedLeft() == 120, "a refused TOFTEST stopped the drive");
    command("STOP");

    // 3. Field validation still comes first.
    a = command("TOFTEST", ",\"sensor\":3");
    CHECK(has(a, "\"reason\":\"OUT_OF_RANGE\""), a.c_str());

    // 4. Idle: no command for longer than the watchdog -> allowed (and it is
    //    the operator's choice to stall an idle controller).
    pass(COMMAND_TIMEOUT_MS + 100);
    CHECK(commTimedOut(), "setup: watchdog did not expire");
    a = command("TOFTEST", ",\"sensor\":0");
    CHECK(has(a, "\"result\":\"ACCEPTED\"") && has(a, "\"status\":\"VALID\""), a.c_str());
}

// ===========================================================================
// E. Firmware identity in READY and DIAG SYSTEM
// ===========================================================================

static void testE_identity(void)
{
    boot();
    char want[96];
    snprintf(want, sizeof(want), "\"fw_build\":\"%s\"", EXPECT_FW_BUILD);
    size_t ready = gTx.find("\"event\":\"READY\"");
    CHECK(ready != std::string::npos, "no READY");
    std::string readyLine = gTx.substr(ready, gTx.find('\n', ready) - ready);
    CHECK(has(readyLine, want), readyLine.c_str());
    CHECK(has(readyLine, "\"fw_build_utc\":\""), readyLine.c_str());
    CHECK(has(readyLine, "\"fw\":\"rover-esp32\""), "READY lost its fw field");

    gTx.clear();
    pass(1500);                               // >= one full DIAG rotation
    size_t sys = gTx.find("\"section\":\"SYSTEM\"");
    CHECK(sys != std::string::npos, "no DIAG SYSTEM");
    size_t lineStart = gTx.rfind('\n', sys);
    lineStart = (lineStart == std::string::npos) ? 0 : lineStart + 1;
    std::string sysLine = gTx.substr(lineStart, gTx.find('\n', sys) - lineStart);
    CHECK(has(sysLine, want), sysLine.c_str());
    CHECK(has(sysLine, "\"loop_max_ms\":"), "P1 loop timing missing from DIAG SYSTEM");
}

typedef void (*TestFn)(void);

int main(void)
{
    static const struct { const char *name; TestFn fn; } kTests[] = {
        { "A1 stop order: STOP",                       testA_explicitStop },
        { "A2 stop order: command watchdog",           testA_watchdog },
        { "A3 stop order: front obstacle gate",        testA_frontGate },
        { "A4 stop order: rear obstacle gate",         testA_rearGate },
        { "A5 stop order: RESET",                      testA_reset },
        { "A6 stop order: MOTORTEST expiry",           testA_motortestEnd },
        { "A7 stop order: DRIVE 0/0",                  testA_driveToZero },
        { "A8 stop order: PCA9685 bus error",          testA_pcaFailure },
        { "B1 reversal: cut, enable off, then dir",    testB_reversal },
        { "B2 reversal with failed enable: held",      testB_reversalEnableFails },
        { "C1 reverse: TCA absent at boot",            testC_tcaAbsent },
        { "C2 reverse: all rear inits fail",           testC_allInitFail },
        { "C3 reverse: rear invalid after boot",       testC_invalidAfterBoot },
        { "C4 reverse: valid rear, obstacle gates",    testC_validRear },
        { "C5 front gate + latch unchanged",           testC_frontGateAndLatch },
        { "C6 rear backoff: no replay, forward kept",  testC_rearBackoffNoReplay },
        { "D1 TOFTEST refused under motion control",   testD_toftest },
        { "E1 build identity in READY and DIAG",       testE_identity },
    };

    printf("=== motion safety: expected fw_build \"%s\" ===\n", EXPECT_FW_BUILD);
    int failedTests = 0;
    for (size_t t = 0; t < sizeof(kTests) / sizeof(kTests[0]); t++) {
        gTest = kTests[t].name;
        int before = gFails;
        kTests[t].fn();
        bool ok = (gFails == before);
        if (!ok) failedTests++;
        printf("  %s  %s\n", ok ? "PASS" : "FAIL", kTests[t].name);
    }
    printf("=== motion safety: %d tests, %d failed, %d checks, %d check failures ===\n",
           (int)(sizeof(kTests) / sizeof(kTests[0])), failedTests, gChecks, gFails);
    return (gFails == 0) ? 0 : 1;
}

// The real sketch: setup() and loop(), compiled unchanged.
#include "../../robot_controller.ino"

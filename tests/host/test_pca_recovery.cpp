// ============================================================================
//  FOCUSED HOST TESTS -- PCA9685 boot failure, bus errors and runtime recovery
//
//  REAL firmware under test: robot_controller.ino (setup/loop), comm.cpp,
//  protocol.cpp, motor.cpp, safety.cpp, rear_tof.cpp, tca9548a.cpp,
//  rover_i2c.cpp, gps.cpp, config.h -- AND pca9685.cpp, which every other host
//  suite replaces with a stub. Simulated below them, in-process, on a MANUAL
//  clock (every loop() pass's delay(1) advances it 1 ms):
//
//    * PCA9685   -- a register-level fake at PCA9685_I2C_ADDRESS: MODE1 (SLEEP,
//                   AI, RESTART), MODE2, PRESCALE (write-protected unless
//                   asleep, as on the part), LEDn and ALL_LED with the full-on /
//                   full-off bits, auto-increment. Faults: absent (address
//                   NACK), N failing data transactions, a chip that ignores
//                   SLEEP, and a hung transaction (clock +1000 ms, code 5 --
//                   what core 2.0.14 does on a hung bus).
//    * L298N     -- a wheel is ENERGISED when its PCA enable output is non-zero
//                   AND its IN1 != IN2. Checked after every direction-pin write,
//                   every PCA register write and every loop() pass.
//    * TCA9548A + three VL53L0X (all valid, 1000 mm), HC-SR04 (150 cm), UART.
//
//  What this CANNOT show: real I2C timing or electrical faults, real L298N or
//  PCA9685 behaviour, or that the rover recovers or moves. It proves the
//  firmware's ORDER and DECISIONS against a model of the part.
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "Arduino.h"
#include "Wire.h"
#include "VL53L0X.h"
#include "pca9685.h"
#include "config.h"
#include "comm.h"
#include "motor.h"
#include "protocol.h"
#include "stall_guard.h"

void setup(void);
void loop(void);

extern bool gStallGuardHostCreateFails;     // stall_guard.cpp, host builds only

static const uint8_t kMotorPins[8] = {
    M_FRONT_A_IN1, M_FRONT_A_IN2, M_FRONT_B_IN1, M_FRONT_B_IN2,
    M_REAR_A_IN1,  M_REAR_A_IN2,  M_REAR_B_IN1,  M_REAR_B_IN2
};
static const uint8_t kEnableCh[4] = {
    PCA_CH_FRONT_A_EN, PCA_CH_FRONT_B_EN, PCA_CH_REAR_A_EN, PCA_CH_REAR_B_EN
};

static uint32_t gNow = 0;
static int      gPin[64];

// ---------------------------------------------------------------------------
// Fake PCA9685
// ---------------------------------------------------------------------------

struct FakePca {
    bool    present;         // ACKs its address at all
    int     failDataTxns;    // next N transactions carrying data fail (code 4)
    bool    hang;            // every transaction: +1000 ms, then code 5
    bool    sleepStuck;      // the SLEEP bit cannot be set
    int     failAt;          // the failAt-th data transaction after arming fails (0: off)
    bool    failHangs;       // ...after hanging 1000 ms (code 5) instead of at once (code 4)
    int     armedTxns;       // data transactions counted since arming
    uint32_t slowProbeMs;    // next address probe takes this long, then SUCCEEDS (0: off)
    uint8_t reg[256];
    uint8_t ptr;
    int     hangs;
    int     probes;
};
static FakePca P;

// A command frame that "arrives on the UART" from INSIDE the next failing or
// hung PCA transaction, i.e. while the firmware is blocked in it. gInjectedAt
// is the arrival time: the START of a hang (worst case for latency).
static std::string gInjectFrame;
static uint32_t    gInjectedAt = 0;
static bool        gInjected   = false;

// Cutoff tripwire: once a PCA write has failed (gFailSeen), any further I2C
// transaction -- to ANY device -- while a direction pin is still HIGH counts as
// a violation. That is "the cut did not happen before the motor layer gave the
// bus back to the rest of the loop".
static bool gFailSeen      = false;
static int  gCutViolations = 0;

static void pcaPowerOn(void)
{
    memset(P.reg, 0, sizeof(P.reg));
    P.reg[PCA9685_REG_MODE1]    = 0x11;           // SLEEP | ALLCALL
    P.reg[PCA9685_REG_PRESCALE] = 0x1E;
    for (int ch = 0; ch < 16; ch++) {
        P.reg[PCA9685_REG_LED0_ON_L + 4 * ch + 3] = 0x10;   // LEDn_OFF_H full off
    }
    P.ptr = 0;
}

// PWM output of one channel, 0..4096 (4096 = full on).
static int pcaOut(int ch)
{
    if (P.reg[PCA9685_REG_MODE1] & PCA9685_MODE1_SLEEP) return 0;
    int b   = PCA9685_REG_LED0_ON_L + 4 * ch;
    int on  = P.reg[b]     | (P.reg[b + 1] << 8);
    int off = P.reg[b + 2] | (P.reg[b + 3] << 8);
    if (off & 0x1000) return 0;
    if (on  & 0x1000) return 4096;
    return (off - on) & 0x0FFF;
}

// "The chip was left running from before": awake, every enable FULL ON.
static void pcaWarmAllOn(void)
{
    P.reg[PCA9685_REG_MODE1] = 0x21;
    for (int ch = 0; ch < 16; ch++) {
        int b = PCA9685_REG_LED0_ON_L + 4 * ch;
        P.reg[b] = 0; P.reg[b + 1] = 0x10; P.reg[b + 2] = 0; P.reg[b + 3] = 0;
    }
}

// ---------------------------------------------------------------------------
// Energised-wheel monitor
// ---------------------------------------------------------------------------

static long gEnergisedScans = 0;     // scans that found ANY wheel energised

static bool wheelEnergised(int k)
{
    return pcaOut(kEnableCh[k]) > 0 && gPin[kMotorPins[2 * k]] != gPin[kMotorPins[2 * k + 1]];
}

static void scanEnergised(void)
{
    for (int k = 0; k < 4; k++) {
        if (wheelEnergised(k)) { gEnergisedScans++; return; }
    }
}

static int energisedWheels(void)
{
    int n = 0;
    for (int k = 0; k < 4; k++) if (wheelEnergised(k)) n++;
    return n;
}

static void pcaWrite(uint8_t r, uint8_t v)
{
    if (r == PCA9685_REG_MODE1) {
        if (P.sleepStuck) v &= (uint8_t)~PCA9685_MODE1_SLEEP;
        P.reg[r] = (uint8_t)(v & (uint8_t)~PCA9685_MODE1_RESTART);
    } else if (r == PCA9685_REG_PRESCALE) {
        if (P.reg[PCA9685_REG_MODE1] & PCA9685_MODE1_SLEEP) P.reg[r] = v;
    } else if (r >= PCA9685_REG_ALL_LED_ON_L && r <= PCA9685_REG_ALL_LED_ON_L + 3) {
        int k = r - PCA9685_REG_ALL_LED_ON_L;
        for (int ch = 0; ch < 16; ch++) P.reg[PCA9685_REG_LED0_ON_L + 4 * ch + k] = v;
    } else {
        P.reg[r] = v;
    }
    scanEnergised();
}

// ---------------------------------------------------------------------------
// Arduino core (manual clock)
// ---------------------------------------------------------------------------

// The loop-stall guard task, modelled: it runs on its own core, so it ticks
// every STALL_GUARD_PERIOD_MS of simulated time WHEREVER the clock advances --
// in loop()'s delay(1), and inside every simulated slow or hung transaction.
//
// Block tracking: during an advance() of more than one period (a blocking
// call), record when every direction pin was first LOW.
static bool     gInBlock     = false;
static uint32_t gBlockStartMs = 0;
static uint32_t gBlockCutMs   = 0;      // 0 = not cut during the block
static bool     gGuardTicks   = true;   // false: the guard task has stopped ticking
static bool     gForceHighInBlock = false;  // drive a direction pin HIGH at a block's start
static bool     gArmedInBlock     = false;  // stallGuardArmed() seen during the last block

static bool pinsAllLowNow(void);

static void advance(uint32_t ms)
{
    bool block = (ms > STALL_GUARD_PERIOD_MS);
    if (block) {
        gInBlock      = true;
        gBlockStartMs = gNow;
        gBlockCutMs   = 0;
        gArmedInBlock = stallGuardArmed();
        if (gForceHighInBlock) gPin[kMotorPins[0]] = HIGH;   // "output live"
    }
    for (uint32_t i = 0; i < ms; i++) {
        gNow++;
        if (gGuardTicks && !gStallGuardHostCreateFails && (gNow % STALL_GUARD_PERIOD_MS) == 0) {
            (void)stallGuardTick(gNow);
        }
        if (block && gBlockCutMs == 0 && pinsAllLowNow()) {
            gBlockCutMs = gNow;
        }
        scanEnergised();
    }
    if (block) gInBlock = false;
}

uint32_t millis(void)                         { return gNow; }
uint32_t micros(void)                         { return gNow * 1000UL; }
void     delay(uint32_t ms)                   { advance(ms); }
void     delayMicroseconds(uint32_t us)       { (void)us; }
void     pinMode(uint8_t pin, uint8_t mode)   { (void)pin; (void)mode; }
int      digitalRead(uint8_t pin)             { return gPin[pin & 63]; }

static int gHighWhileTripped = 0;       // a direction pin raised while tripped
static int gDirHighWrites    = 0;       // every direction-pin HIGH write

static bool isDirPin(uint8_t pin)
{
    for (int i = 0; i < 8; i++) if (kMotorPins[i] == pin) return true;
    return false;
}

void digitalWrite(uint8_t pin, uint8_t v)
{
    if (v && isDirPin(pin) && stallGuardTripped()) gHighWhileTripped++;
    if (v && isDirPin(pin)) gDirHighWrites++;
    gPin[pin & 63] = v ? HIGH : LOW;
    scanEnergised();
}

static bool pinsAllLowNow(void)
{
    for (int i = 0; i < 8; i++) if (gPin[kMotorPins[i]] != LOW) return false;
    return true;
}

// HC-SR04: an echo at gFrontCm; pulseIn() busy-waits for it, as on the ESP32,
// when gPulseBusy is set (the 20 ms worst case is what the guard must tolerate).
static double gFrontCm   = 150.0;
static bool   gPulseBusy = false;

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeoutUs)
{
    (void)pin; (void)state;
    unsigned long us = (unsigned long)(gFrontCm / 0.01715);
    if (us > timeoutUs) us = timeoutUs;
    if (gPulseBusy) advance((uint32_t)((us + 999) / 1000));
    return (us >= timeoutUs) ? 0 : us;
}

SimSerial Serial;
static std::string gRx;
static size_t      gRxPos = 0;
static std::string gTx;

// gRx is the UART driver's RX RING BUFFER: what Serial.available() counts.
//
// gFifo is the UART HARDWARE FIFO in front of it: bytes that have arrived on
// the pin but that the driver has not yet moved into the ring. On core 2.0.14
// at 115200 that move happens on RX-FIFO-full (120 bytes) or after 2 idle
// symbol times, so a whole in-flight frame can sit here, invisible to
// Serial.available() (uart_get_buffered_data_len() counts the ring only).
// Modelled as: flushed into the ring before each loop() pass.
static std::string gFifo;

static void fifoFlush(void)
{
    gRx += gFifo;
    gFifo.clear();
}

int SimSerial::available(void) { return (int)(gRx.size() - gRxPos); }
int SimSerial::read(void)      { return (gRxPos < gRx.size()) ? (uint8_t)gRx[gRxPos++] : -1; }

// Availability-report hook. Every TELEMETRY frame the firmware QUEUES with
// "motor_drive_available":true is counted and timestamped (the firmware takes
// its acknowledgement snapshot right after queuing it). At that instant a test
// can place a frame in the hardware FIFO (still arriving), or in the ring
// buffer (arrived, unread).
static int         gAvailReports       = 0;     // since the last watchReports()
static uint32_t    gFirstAvailReportMs = 0;
static std::string gFifoOnReport;               // -> FIFO when report N is queued
static int         gFifoOnReportN      = 0;
static std::string gRingOnSettleReport;         // -> ring at the first report >=
                                                //    ACK_SETTLE_MS after the first
static bool        gReportDelivered    = false;

size_t SimSerial::write(const uint8_t *buf, size_t len)
{
    gTx.append((const char *)buf, len);

    std::string s((const char *)buf, len);
    if (s.find("\"type\":\"TELEMETRY\"") != std::string::npos &&
        s.find("\"motor_drive_available\":true") != std::string::npos) {
        gAvailReports++;
        if (gAvailReports == 1) gFirstAvailReportMs = gNow;
        if (!gFifoOnReport.empty() && gAvailReports == gFifoOnReportN) {
            gFifo += gFifoOnReport;
            gFifoOnReport.clear();
            gReportDelivered = true;
        }
        if (!gRingOnSettleReport.empty() && gAvailReports > 1 &&
            gNow - gFirstAvailReportMs >= ACK_SETTLE_MS) {
            gRx += gRingOnSettleReport;
            gRingOnSettleReport.clear();
            gReportDelivered = true;
        }
    }
    return len;
}

// ---------------------------------------------------------------------------
// I2C: the fake PCA9685, the TCA9548A, and three VL53L0X behind it
// ---------------------------------------------------------------------------

TwoWire Wire;
static int                  gTxAddr = -1;
static std::vector<uint8_t> gTxBytes;
static uint8_t              gTcaMask = 0;
static std::vector<uint8_t> gRxBytes;

bool   TwoWire::begin(int sda, int scl, uint32_t f) { (void)sda; (void)scl; (void)f; return true; }
void   TwoWire::beginTransmission(int address)       { gTxAddr = address; gTxBytes.clear(); }
size_t TwoWire::write(uint8_t b)                     { gTxBytes.push_back(b); return 1; }

static bool directionPinsAllLow(void)
{
    for (int i = 0; i < 8; i++) if (gPin[kMotorPins[i]] != LOW) return false;
    return true;
}

static void cutTripwire(void)
{
    if (gFailSeen && !directionPinsAllLow()) gCutViolations++;
}

// The command "arrives" now, while the firmware is blocked in a transaction.
static void deliverInjected(uint32_t arrivedAt)
{
    if (gInjectFrame.empty()) return;
    gRx += gInjectFrame;
    gInjectFrame.clear();
    gInjectedAt = arrivedAt;
    gInjected   = true;
}

static void hangNow(void)
{
    uint32_t start = gNow;
    advance(1000);
    P.hangs++;
    deliverInjected(start);
}

// Hangs on the OTHER I2C users (each: the next N transactions block 1000 ms,
// then fail with code 5) -- the loop-stall guard must cover them all.
static int gTcaHangNext  = 0;
static int gBootTcaHang  = 0;           // consumed by boot(): hang during setup()
static bool gBootForceHigh = false;     // consumed by boot(): force a pin HIGH in that hang
static bool gBootGuardCreateFails = false;  // consumed by boot(): xTaskCreate... fails
static bool gBootGuardNoTicks     = false;  // consumed by boot(): task created, never ticks
static int gVl53HangNext = 0;
static int gGpsHangNext  = 0;
static int gGpsTxns      = 0;
static uint32_t gGpsDelayMs = 0;        // every GPS transaction takes this long (legit burst)

// A PCA data transaction that is SLOW but SUCCEEDS (the resume race).
static int      gSlowOkAt   = 0;        // the n-th data txn after arming
static int      gSlowOkTxns = 0;
static uint32_t gSlowOkMs   = 0;

uint8_t TwoWire::endTransmission(bool sendStop)
{
    (void)sendStop;
    cutTripwire();

    if (gTxAddr == TCA9548A_I2C_ADDRESS) {
        if (gTcaHangNext > 0) { uint32_t st = gNow; gTcaHangNext--; advance(1000); deliverInjected(st); return 5; }
        if (!gTxBytes.empty()) gTcaMask = gTxBytes.back();
        return 0;
    }
    if (gTxAddr == NEO9M_I2C_ADDRESS) {
        gGpsTxns++;
        if (gGpsHangNext > 0) { uint32_t st = gNow; gGpsHangNext--; advance(1000); deliverInjected(st); return 5; }
        advance(gGpsDelayMs);
        return 2;                            // GPS absent (NACK)
    }
    if (gTxAddr != PCA9685_I2C_ADDRESS) return 2;

    if (gSlowOkAt > 0 && !gTxBytes.empty() && ++gSlowOkTxns == gSlowOkAt) {
        gSlowOkAt = 0;
        advance(gSlowOkMs);                  // slow ... then it simply succeeds
    }

    if (P.hang)     { hangNow(); return 5; }
    if (!P.present) return 2;
    if (gTxBytes.empty()) {
        P.probes++;
        if (P.slowProbeMs > 0) {             // a slow but successful recovery probe
            uint32_t start = gNow;
            advance(P.slowProbeMs);
            P.slowProbeMs = 0;
            deliverInjected(start);
        }
        return 0;
    }
    if (P.failDataTxns > 0) { P.failDataTxns--; return 4; }
    if (P.failAt > 0 && ++P.armedTxns == P.failAt) {
        P.failAt  = 0;
        gFailSeen = true;
        if (P.failHangs) { hangNow(); return 5; }
        deliverInjected(gNow);
        return 4;
    }

    P.ptr = gTxBytes[0];
    for (size_t i = 1; i < gTxBytes.size(); i++) {
        pcaWrite(P.ptr, gTxBytes[i]);
        if (P.reg[PCA9685_REG_MODE1] & PCA9685_MODE1_AI) P.ptr++;
    }
    return 0;
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
    gRxBytes.clear();
    cutTripwire();
    if (quantity < 1) return 0;

    if (address == TCA9548A_I2C_ADDRESS) {
        gRxBytes.push_back(gTcaMask);
        return 1;
    }
    if (address != PCA9685_I2C_ADDRESS) return 0;
    if (P.hang)     { hangNow(); return 0; }
    if (!P.present) return 0;

    for (int i = 0; i < quantity; i++) {
        gRxBytes.push_back(P.reg[P.ptr]);
        if (P.reg[PCA9685_REG_MODE1] & PCA9685_MODE1_AI) P.ptr++;
    }
    return (uint8_t)quantity;
}

int TwoWire::available(void) { return (int)gRxBytes.size(); }
int TwoWire::read(void)
{
    if (gRxBytes.empty()) return -1;
    int b = gRxBytes.front();
    gRxBytes.erase(gRxBytes.begin());
    return b;
}

// Rear sensors: one per open TCA channel, always valid at 1000 mm.
static bool rearOpen(void) { uint8_t m = gTcaMask; return m != 0 && (m & (m - 1)) == 0; }
bool     simVl53Init(void)                               { return rearOpen(); }
uint8_t  simVl53ReadReg(uint8_t reg, uint8_t *status)
{
    if (!rearOpen()) { *status = 2; return 0xFF; }
    if (gVl53HangNext > 0) { uint32_t st = gNow; gVl53HangNext--; advance(1000); deliverInjected(st); *status = 5; return 0xFF; }
    *status = 0;
    if (reg == VL53L0X::RESULT_INTERRUPT_STATUS) return 0x07;
    if (reg == VL53L0X::RESULT_RANGE_STATUS)     return (uint8_t)(11 << 3);
    return 0;
}
uint16_t simVl53ReadRange(uint8_t *status, bool *timedOut)
{
    (void)timedOut;
    if (!rearOpen()) { *status = 2; return 0xFFFF; }
    *status = 0;
    return 1000;
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
            printf("  FAIL [%s] line %d: %s -- %s\n", gTest, __LINE__, #cond, \
                   std::string(msg).c_str());                                 \
        }                                                                     \
    } while (0)

static int      gSeq        = 100;
static uint32_t gLongestPass = 0;
static int      gSlowPasses  = 0;        // passes >= 1000 ms

static void pass(int n = 1)
{
    for (int i = 0; i < n; i++) {
        uint32_t t0 = gNow;
        fifoFlush();                         // the driver's ISR ran meanwhile
        loop();
        scanEnergised();
        uint32_t d = gNow - t0;
        if (d > gLongestPass) gLongestPass = d;
        if (d >= 1000) gSlowPasses++;
    }
}

static void passMs(uint32_t ms)
{
    uint32_t end = gNow + ms;
    while ((int32_t)(gNow - end) < 0) pass(1);
}

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

static std::vector<std::string> lines(void)
{
    std::vector<std::string> out;
    size_t start = 0;
    while (start < gTx.size()) {
        size_t end = gTx.find('\n', start);
        if (end == std::string::npos) break;
        out.push_back(gTx.substr(start, end - start));
        start = end + 1;
    }
    return out;
}

static bool has(const std::string &s, const char *needle) { return s.find(needle) != std::string::npos; }

static std::string ackFor(int seq)
{
    char key[32];
    snprintf(key, sizeof(key), "\"seq\":%d,", seq);
    std::string found;
    for (const std::string &l : lines()) {
        if ((has(l, "\"type\":\"ACK\"") || has(l, "\"type\":\"ERROR\"")) && has(l, key)) found = l;
    }
    return found;
}

static std::string command(const char *cmd, const char *fields = "")
{
    int seq = send(cmd, fields);
    pass(3);
    return ackFor(seq);
}

static std::string lastLine(const char *needle)
{
    std::string found;
    for (const std::string &l : lines()) if (has(l, needle)) found = l;
    return found;
}

// A fresh DIAG SYSTEM / TELEMETRY frame, produced after this call.
static std::string freshSystem(void)
{
    gTx.clear();
    passMs(1300);                     // >= one FRONT -> REAR -> SYSTEM rotation
    return lastLine("\"section\":\"SYSTEM\"");
}

static std::string freshTelemetry(void)
{
    gTx.clear();
    passMs(250);
    return lastLine("\"type\":\"TELEMETRY\"");
}

static long jsonInt(const std::string &l, const char *key)
{
    std::string k = std::string("\"") + key + "\":";
    size_t p = l.find(k);
    if (p == std::string::npos) return -999999;
    return strtol(l.c_str() + p + k.size(), NULL, 10);
}

static bool motorPinsAllLow(void)
{
    for (int i = 0; i < 8; i++) if (gPin[kMotorPins[i]] != LOW) return false;
    return true;
}

static bool enableOutputsAllOff(void)
{
    for (int k = 0; k < 4; k++) if (pcaOut(kEnableCh[k]) != 0) return false;
    return true;
}

// Fresh firmware state, then setup() and enough passes for the sensor windows.
static void boot(bool pcaPresent, int failDataTxns = 0, bool sleepStuck = false,
                 uint32_t startMs = 0)
{
    gRx.clear(); gRxPos = 0; gTx.clear();
    for (int i = 0; i < 64; i++) gPin[i] = LOW;
    gNow = startMs; gTcaMask = 0;
    gTcaHangNext = 0; gVl53HangNext = 0; gGpsHangNext = 0; gGpsTxns = 0; gGpsDelayMs = 0;
    gSlowOkAt = 0; gSlowOkTxns = 0; gSlowOkMs = 0;
    gFrontCm = 150.0; gPulseBusy = false;
    gGuardTicks = !gBootGuardNoTicks;            // the guard task, as asked
    gBootGuardNoTicks = false;
    gStallGuardHostCreateFails = gBootGuardCreateFails;
    gBootGuardCreateFails = false;
    gHighWhileTripped = 0; gDirHighWrites = 0; gInBlock = false; gBlockCutMs = 0;
    gForceHighInBlock = false; gArmedInBlock = false;
    gTcaHangNext = gBootTcaHang;                 // a hang INSIDE setup(), if asked
    gBootTcaHang = 0;
    gForceHighInBlock = gBootForceHigh;
    gBootForceHigh = false;
    pcaPowerOn();
    P.present = pcaPresent; P.failDataTxns = failDataTxns; P.hang = false;
    P.sleepStuck = sleepStuck; P.hangs = 0; P.probes = 0;
    P.failAt = 0; P.failHangs = false; P.armedTxns = 0; P.slowProbeMs = 0;
    gInjectFrame.clear(); gInjected = false; gInjectedAt = 0;
    gFifo.clear(); gFifoOnReport.clear(); gFifoOnReportN = 0;
    gRingOnSettleReport.clear(); gReportDelivered = false;
    gAvailReports = 0; gFirstAvailReportMs = 0;
    gFailSeen = false; gCutViolations = 0;
    gEnergisedScans = 0; gLongestPass = 0; gSlowPasses = 0;
    setup();
    pass(300);
}

// Fail the n-th PCA9685 data transaction from now (1 = the next one).
static void armFailure(int n, bool hangs)
{
    P.failAt    = n;
    P.failHangs = hangs;
    P.armedTxns = 0;
    gFailSeen   = false;
    gCutViolations = 0;
}

// A complete command frame, NOT queued: for delivery from inside a transaction.
static std::string frameFor(const char *cmd, const char *fields, int *seqOut)
{
    char payload[200];
    int seq = ++gSeq;
    snprintf(payload, sizeof(payload), "{\"type\":\"COMMAND\",\"seq\":%d,\"cmd\":\"%s\"%s}",
             seq, cmd, fields);
    char frame[220];
    snprintf(frame, sizeof(frame), "%s*%04X\n", payload,
             (unsigned)protoCrc16(payload, strlen(payload)));
    *seqOut = seq;
    return frame;
}

// Run passes until `seq` is answered (or `limitMs` elapses); returns the ACK.
static std::string awaitAck(int seq, uint32_t limitMs)
{
    uint32_t t0 = gNow;
    while (ackFor(seq).empty() && gNow - t0 < limitMs) pass(1);
    return ackFor(seq);
}

static void driveAccepted(int l, int r)
{
    char f[64];
    snprintf(f, sizeof(f), ",\"left\":%d,\"right\":%d", l, r);
    std::string a = command("DRIVE", f);
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
    pass(20);
    CHECK(energisedWheels() == 4, "setup: the DRIVE did not energise all four wheels");
}

// ===========================================================================
// 1. Healthy boot: the real driver's register sequence, no recovery activity
// ===========================================================================

static void test1_healthyBoot(void)
{
    boot(true);
    std::string ready = lastLine("\"event\":\"READY\"");
    CHECK(has(ready, "\"pca_status\":\"OK\"") && has(ready, "\"motor_drive_available\":true"), ready);
    CHECK(P.reg[PCA9685_REG_MODE1] == 0x21, "MODE1 is not AI|ALLCALL, awake");
    CHECK(P.reg[PCA9685_REG_PRESCALE] == 5, "PRESCALE is not 5 (1 kHz)");
    CHECK(P.reg[PCA9685_REG_MODE2] == PCA9685_MODE2_OUTDRV, "MODE2 is not totem-pole");
    CHECK(enableOutputsAllOff() && motorPinsAllLow(), "outputs not off after boot");
    CHECK(gEnergisedScans == 0, "a wheel was energised during boot");

    std::string sys = freshSystem();
    CHECK(has(sys, "\"pca_recovery\":\"NONE\"") && has(sys, "\"pca_last_fault\":\"NONE\""), sys);
    CHECK(jsonInt(sys, "pca_recovery_attempts") == 0 && jsonInt(sys, "pca_next_retry_ms") == 0, sys);

    // The real duty path: 150 -> 150 * 4095 / 255 = 2408 on all four enables.
    driveAccepted(150, 150);
    for (int k = 0; k < 4; k++) CHECK(pcaOut(kEnableCh[k]) == 2408, "duty is not 2408");
    command("STOP");
    pass(5);
    CHECK(enableOutputsAllOff() && motorPinsAllLow(), "STOP did not stop");
}

// ===========================================================================
// 2. Boot: PCA9685 absent (power-up race), appears later -> recovered
// ===========================================================================

static void test2_bootAbsentThenRecovers(void)
{
    boot(false);
    std::string ready = lastLine("\"event\":\"READY\"");
    CHECK(has(ready, "\"pca_status\":\"NOT_FOUND\"") && has(ready, "\"motor_drive_available\":false"), ready);

    std::string sys = freshSystem();
    CHECK(has(sys, "\"pca_status\":\"NOT_FOUND\"") && has(sys, "\"pca_recovery\":\"RETRYING\"") &&
          has(sys, "\"pca_last_fault\":\"BOOT_INIT\""), sys);

    // The Pi (or an operator) asks for motion while the drive is unavailable.
    int seq = send("DRIVE", ",\"left\":150,\"right\":150");
    pass(3);
    std::string a = ackFor(seq);
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, "\"reason\":\"MOTOR_PWM_UNAVAILABLE\""), a);
    std::string tel = freshTelemetry();
    CHECK(has(tel, "\"left_cmd\":0,\"right_cmd\":0"), "a DRIVE made while unavailable was stored: " + tel);
    CHECK(has(tel, "\"safety_stop\":false"), "nothing was interrupted, yet safety_stop latched: " + tel);

    // The chip comes up ~4 s after boot (as on 2026-10-02 16:21).
    P.present = true;
    passMs(8000);
    CHECK(motorDriveAvailable(), "the drive did not recover once the chip answered");
    CHECK(gEnergisedScans == 0, "a wheel was energised during or after recovery");
    CHECK(enableOutputsAllOff() && motorPinsAllLow(), "outputs not off after recovery");
    CHECK(commDesiredLeft() == 0 && commDesiredRight() == 0, "an old request survived the recovery");

    sys = freshSystem();
    CHECK(has(sys, "\"pca_status\":\"OK\"") && has(sys, "\"pca_recovery\":\"RECOVERED\"") &&
          has(sys, "\"pca_last_fault\":\"BOOT_INIT\""), sys);
    CHECK(jsonInt(sys, "pca_recoveries") == 1 && jsonInt(sys, "pca_recovery_attempts") >= 1, sys);
    tel = freshTelemetry();
    CHECK(has(tel, "\"motor_drive_available\":true") && has(tel, "\"left_applied\":0,\"right_applied\":0"), tel);

    // Motion needs a NEW command, and then it works through the real driver.
    driveAccepted(120, 120);
    command("STOP");
}

// ===========================================================================
// 3. Boot: chip answers but the bring-up fails part-way -> INIT_FAILED -> recovers
// ===========================================================================

static void test3_bootInitFailedThenRecovers(void)
{
    boot(true, /*failDataTxns*/ 1);
    std::string ready = lastLine("\"event\":\"READY\"");
    CHECK(has(ready, "\"pca_status\":\"INIT_FAILED\""), ready);
    CHECK(!motorDriveAvailable(), "drive available after a failed bring-up");

    passMs(3000);
    CHECK(motorDriveAvailable(), "the drive did not recover after a transient bring-up failure");
    CHECK(gEnergisedScans == 0, "a wheel was energised");
    std::string sys = freshSystem();
    CHECK(has(sys, "\"pca_recovery\":\"RECOVERED\"") && has(sys, "\"pca_last_fault\":\"BOOT_INIT\""), sys);
}

// ===========================================================================
// 4. A chip that does not hold its configuration is never declared ready
// ===========================================================================

static void test4_verifyRejectsBadConfig(void)
{
    boot(true, 0, /*sleepStuck*/ true);         // PRESCALE can never be written
    CHECK(!motorDriveAvailable(), "a chip at the wrong PWM frequency was declared ready");
    std::string ready = lastLine("\"event\":\"READY\"");
    CHECK(has(ready, "\"pca_status\":\"INIT_FAILED\""), ready);

    passMs(10000);
    CHECK(!motorDriveAvailable(), "a retry declared the misconfigured chip ready");
    CHECK(gEnergisedScans == 0 && enableOutputsAllOff(), "output while misconfigured");
    std::string sys = freshSystem();
    CHECK(has(sys, "\"pca_status\":\"INIT_FAILED\"") && has(sys, "\"pca_recovery\":\"RETRYING\""), sys);

    P.sleepStuck = false;                       // the part behaves again
    passMs(20000);
    CHECK(motorDriveAvailable(), "no recovery once the chip accepted its configuration");
    CHECK(P.reg[PCA9685_REG_PRESCALE] == 5, "recovered with the wrong prescale");
}

// ===========================================================================
// 5. Transient bus error WHILE DRIVING -> stop, latch, recover, stay stopped
// ===========================================================================

static void test5_transientWhileDriving(void)
{
    boot(true);
    driveAccepted(150, 150);

    P.failDataTxns = 1;                         // one failed enable write
    pass(1);
    CHECK(!motorDriveAvailable(), "the failed write did not make the drive unavailable");
    CHECK(motorPinsAllLow(), "direction inputs not LOW in the failing pass");
    CHECK(energisedWheels() == 0, "a wheel still energised after the failing pass");
    long afterFault = gEnergisedScans;

    pass(1);
    CHECK(commSafetyStopLatched(), "an interrupted motion did not latch safety_stop");
    CHECK(commDesiredLeft() == 0 && commDesiredRight() == 0, "the interrupted request was kept");
    std::string sys = freshSystem();
    CHECK(has(sys, "\"pca_last_fault\":\"RUNTIME_BUS\"") && jsonInt(sys, "pca_runtime_bus_errors") == 1, sys);

    passMs(5000);
    CHECK(motorDriveAvailable(), "no recovery from a single transient bus error");
    CHECK(gEnergisedScans == afterFault, "a wheel was energised after the fault (the old DRIVE resumed)");
    CHECK(enableOutputsAllOff() && motorPinsAllLow(), "outputs not off after recovery");
    CHECK(commSafetyStopLatched(), "recovery cleared the safety_stop latch");

    sys = freshSystem();
    CHECK(has(sys, "\"pca_status\":\"OK\"") && has(sys, "\"pca_recovery\":\"RECOVERED\"") &&
          has(sys, "\"pca_last_fault\":\"RUNTIME_BUS\""), sys);
    std::string tel = freshTelemetry();
    CHECK(has(tel, "\"safety_stop\":true") && has(tel, "\"left_applied\":0,\"right_applied\":0") &&
          has(tel, "\"motor_drive_available\":true"), tel);

    // Only an explicit RESET acknowledges it; motion then needs a new command.
    std::string a = command("RESET");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
    CHECK(!commSafetyStopLatched(), "RESET did not clear the latch");
    pass(50);
    CHECK(gEnergisedScans == afterFault, "RESET started a wheel");
    driveAccepted(120, 120);
    command("STOP");
}

// ===========================================================================
// 6. Warm chip holding every enable FULL ON when the drive is recovered
// ===========================================================================

static void test6_warmChipForcedOffFirst(void)
{
    boot(false);
    pass(10);
    // A chip that was running before (e.g. through an ESP32 reset) and now
    // answers with all sixteen outputs full on. Direction inputs stay LOW, so
    // nothing is energised; recovery must still clear the outputs.
    pcaWarmAllOn();
    P.present = true;
    passMs(5000);
    CHECK(motorDriveAvailable(), "no recovery");
    CHECK(enableOutputsAllOff(), "recovery left an enable output on");
    CHECK(gEnergisedScans == 0, "a wheel was energised during recovery");
}

// ===========================================================================
// 7. Persistent failure: backoff schedule, loop keeps running, then recovery
// ===========================================================================

static void test7_persistentBackoff(void)
{
    boot(true);
    P.present = false;                          // chip stops answering while idle
    pass(2);
    CHECK(!motorDriveAvailable(), "drive still available with the chip gone");
    CHECK(!commSafetyStopLatched(), "idle loss latched safety_stop (nothing was interrupted)");
    uint32_t t0 = gNow;

    gTx.clear();
    passMs(100000);
    // First attempt 1 s after the loss, then 2, 4, 8, 16 s, then every 30 s:
    // attempts at ~1, 3, 7, 15, 31, 61, 91 s inside 100 s.
    CHECK(pca9685RecoveryAttempts() == 7, "unexpected retry count in 100 s");
    CHECK(!motorDriveAvailable() && gEnergisedScans == 0, "output while the chip was gone");
    int telemetry = 0;
    for (const std::string &l : lines()) if (has(l, "\"type\":\"TELEMETRY\"")) telemetry++;
    CHECK(telemetry >= 480, "TELEMETRY stopped flowing while retrying");

    std::string sys = freshSystem();
    CHECK(has(sys, "\"pca_status\":\"NOT_FOUND\"") && has(sys, "\"pca_recovery\":\"RETRYING\"") &&
          has(sys, "\"pca_last_fault\":\"RUNTIME_BUS\""), sys);
    long next = jsonInt(sys, "pca_next_retry_ms");
    CHECK(next > 0 && next <= (long)PCA9685_RECOVERY_MAX_MS, sys);

    P.present = true;
    passMs(PCA9685_RECOVERY_MAX_MS + 100);
    CHECK(motorDriveAvailable(), "no recovery after a persistent fault cleared");
    CHECK(pca9685Recoveries() == 1 && pca9685RuntimeBusErrors() == 1, "counters wrong");
    CHECK(gNow - t0 < 135000, "recovery took longer than the backoff allows");
    CHECK(gEnergisedScans == 0, "a wheel was energised");
}

// ===========================================================================
// 8. Hung PCA9685 transactions: stalls are bounded, STOP is still answered
// ===========================================================================

static void test8_hungBusBounded(void)
{
    boot(true);
    P.hang = true;                              // every PCA transaction: 1 s, then fails
    gSlowPasses = 0;
    passMs(120000);
    CHECK(!motorDriveAvailable(), "drive available on a hung bus");
    // Fault detection (1) + the AllOff attempts before the first retry
    // (<= 2) + the 8 backoff retries inside 120 s.
    CHECK(P.hangs <= 12, "too many hung PCA transactions: the backoff is not bounding them");
    CHECK(gSlowPasses <= 12, "too many >= 1 s loop passes");
    CHECK(gEnergisedScans == 0 && motorPinsAllLow(), "output on a hung bus");

    // A STOP that ARRIVES WHILE the firmware is blocked in a hung transaction
    // (delivered from inside it) is answered as soon as that one transaction
    // returns: latency = one hang, measured from the hang's START. The hung
    // transactions here are the backoff retries, so each STOP waits for one.
    uint32_t worst = 0;
    for (int i = 0; i < 3; i++) {
        int seq;
        gInjectFrame = frameFor("STOP", "", &seq);
        gInjected = false;
        uint32_t t = gNow;
        while (!gInjected && gNow - t < PCA9685_RECOVERY_MAX_MS + 2000) pass(1);
        CHECK(gInjected, "setup: no hung transaction to deliver the STOP from");
        std::string a = awaitAck(seq, 5000);
        CHECK(has(a, "\"result\":\"ACCEPTED\""), "STOP sent during a hang not accepted: " + a);
        if (gNow - gInjectedAt > worst) worst = gNow - gInjectedAt;
    }
    CHECK(worst >= 1000, "harness: the STOP did not arrive inside the hang");
    CHECK(worst <= 1010, "STOP ACK delayed beyond the one hung transaction it arrived in");

    P.hang = false;
    passMs(PCA9685_RECOVERY_MAX_MS + 100);
    CHECK(motorDriveAvailable(), "no recovery after the bus came back");
    CHECK(gEnergisedScans == 0, "a wheel was energised");
}

// ===========================================================================
// 9. MOTORTEST interrupted by a bus error is cancelled, never resumed
// ===========================================================================

static void test9_motortestCancelled(void)
{
    boot(true);
    std::string a = command("MOTORTEST", ",\"motor\":0,\"power\":150,\"ms\":3000,\"onblocks\":true");
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
    pass(50);
    CHECK(commTestModeActive() && energisedWheels() == 1, "setup: MOTORTEST not driving");

    P.failDataTxns = 1;
    pass(2);
    CHECK(!commTestModeActive(), "MOTORTEST kept running after the drive was lost");
    std::string done = lastLine("\"event\":\"MOTORTEST_DONE\"");
    CHECK(has(done, "\"reason\":\"CANCELLED\""), done);
    CHECK(commSafetyStopLatched(), "interrupted MOTORTEST did not latch safety_stop");
    long afterFault = gEnergisedScans;

    passMs(5000);                               // longer than the test's 3000 ms
    CHECK(motorDriveAvailable(), "no recovery");
    CHECK(gEnergisedScans == afterFault, "the cancelled MOTORTEST resumed after recovery");
}

// ===========================================================================
// 10. Idle transient error: recovers, no latch, watchdog state untouched
// ===========================================================================

static void test10_idleTransient(void)
{
    boot(true);
    command("STOP");
    passMs(COMMAND_TIMEOUT_MS + 200);
    CHECK(commTimedOut(), "setup: watchdog did not time out");

    P.failDataTxns = 1;
    pass(3);
    CHECK(!motorDriveAvailable(), "the failed write did not make the drive unavailable");
    passMs(3000);
    CHECK(motorDriveAvailable(), "no recovery");
    CHECK(!commSafetyStopLatched(), "an idle loss latched safety_stop");
    CHECK(commTimedOut(), "recovery cleared the command-timeout latch");
    CHECK(gEnergisedScans == 0, "a wheel was energised");
}

// ===========================================================================
// 11. Cutoff: a failed enable write on ANY channel leaves every direction pin
//     LOW when the motor layer returns -- before any other I2C user runs
// ===========================================================================
//
// While driving 150/150 each channel k is written as: enable 0 (data
// transaction 2k+1), direction, duty (2k+2). Every position, fast and hung.

static void test11_cutoffDrive(void)
{
    for (int k = 0; k < 4; k++) {
        for (int step = 1; step <= 2; step++) {
            for (int hang = 0; hang <= 1; hang++) {
                char where[96];
                snprintf(where, sizeof(where), "ch%d %s write, %s", k,
                         step == 1 ? "enable-off" : "duty", hang ? "hung 1 s" : "fast error");
                int n = 2 * k + step;

                // (a) The motor-layer call itself.
                boot(true);
                driveAccepted(150, 150);
                armFailure(n, hang != 0);
                driveDifferential(150, 150);
                CHECK(gFailSeen, std::string("harness: no failure injected, ") + where);
                CHECK(directionPinsAllLow(), std::string("direction pin HIGH when driveDifferential returned: ") + where);
                CHECK(energisedWheels() == 0, std::string("a wheel energised after the failed write: ") + where);
                CHECK(!motorDriveAvailable(), std::string("drive still available: ") + where);

                // (b) The same inside loop(): no other I2C transaction (rear
                //     poll, GPS, ...) may run before the cut.
                boot(true);
                driveAccepted(150, 150);
                armFailure(n, hang != 0);
                pass(1);
                CHECK(gFailSeen, std::string("harness: no failure in the pass, ") + where);
                CHECK(gCutViolations == 0, std::string("I2C traffic while a direction pin was still HIGH: ") + where);
                CHECK(directionPinsAllLow(), std::string("direction pin HIGH at the end of the failing pass: ") + where);
                // Output was live at every one of these positions.
                pass(2);
                CHECK(commSafetyStopLatched(), std::string("live output lost, safety_stop not latched: ") + where);
            }
        }
    }
}

// MOTORTEST drives ONE raw channel; the others are written to 0 every pass.
// Motor 3: transactions 1..3 are the stop writes of channels 0..2 (with channel
// 3 still running from the previous pass), 4 and 5 are channel 3's own.
static void test12_cutoffMotortest(void)
{
    for (int n = 1; n <= 5; n++) {
        for (int hang = 0; hang <= 1; hang++) {
            char where[64];
            snprintf(where, sizeof(where), "MOTORTEST motor 3, txn %d, %s", n, hang ? "hung" : "fast");
            boot(true);
            std::string a = command("MOTORTEST", ",\"motor\":3,\"power\":150,\"ms\":3000,\"onblocks\":true");
            CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
            pass(20);
            CHECK(energisedWheels() == 1, std::string("setup: MOTORTEST not driving, ") + where);
            armFailure(n, hang != 0);
            motorDriveChannel(3, 150);
            CHECK(gFailSeen, std::string("harness: no failure injected, ") + where);
            CHECK(directionPinsAllLow(), std::string("direction pin HIGH when motorDriveChannel returned: ") + where);
            CHECK(energisedWheels() == 0, std::string("a wheel energised after the failed write: ") + where);
            // Channel 3 was running at every position (stop writes of 0..2
            // included), so the record must reach the latch.
            pass(2);
            CHECK(commSafetyStopLatched(), std::string("live output lost, safety_stop not latched: ") + where);
            CHECK(!commTestModeActive(), std::string("MOTORTEST not cancelled: ") + where);
        }
    }
}

// ===========================================================================
// 13. Latch evidence cannot be erased by a command arriving DURING the failure
// ===========================================================================

enum MotionKind { WHILE_DRIVING, WHILE_MOTORTEST };

static void latchCase(MotionKind kind, const char *cmd, const char *fields, bool hang)
{
    char where[96];
    snprintf(where, sizeof(where), "%s during a %s failure %s", cmd, hang ? "hung" : "fast",
             kind == WHILE_DRIVING ? "while driving" : "in MOTORTEST");

    boot(true);
    if (kind == WHILE_DRIVING) {
        driveAccepted(150, 150);
    } else {
        std::string a = command("MOTORTEST", ",\"motor\":0,\"power\":150,\"ms\":3000,\"onblocks\":true");
        CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
        pass(20);
    }
    CHECK(!commSafetyStopLatched(), std::string("setup: latch already set, ") + where);

    int seq;
    gInjectFrame = frameFor(cmd, fields, &seq);
    gInjected = false;
    armFailure(kind == WHILE_DRIVING ? 3 : 1, hang);   // channel 1 / channel 0 enable-off
    pass(1);
    CHECK(gFailSeen && gInjected, std::string("harness: failure or delivery missing, ") + where);
    CHECK(gCutViolations == 0 && directionPinsAllLow(), std::string("not cut in the failing pass, ") + where);
    long afterFault = gEnergisedScans;

    std::string ack = awaitAck(seq, 3000);
    CHECK(!ack.empty(), std::string("no answer to the command, ") + where);
    if (strcmp(cmd, "DRIVE") == 0) {
        CHECK(has(ack, "\"result\":\"GATED\"") && has(ack, "MOTOR_PWM_UNAVAILABLE"), ack);
    } else {
        CHECK(has(ack, "\"result\":\"ACCEPTED\""), ack);
    }
    if (hang && strcmp(cmd, "STOP") == 0) {
        CHECK(gNow - gInjectedAt <= 1010, std::string("STOP answered later than the hung write it arrived in"));
    }
    pass(3);
    CHECK(commSafetyStopLatched(), std::string("safety_stop NOT latched: the evidence was erased by ") + where);
    CHECK(!commTestModeActive(), std::string("MOTORTEST still running, ") + where);
    std::string tel = freshTelemetry();
    CHECK(has(tel, "\"safety_stop\":true"), std::string("TELEMETRY does not show the latch: ") + tel);

    // Recovery: nothing stored resumes, and the latch survives it.
    passMs(5000);
    CHECK(motorDriveAvailable(), std::string("no recovery, ") + where);
    CHECK(gEnergisedScans == afterFault, std::string("a wheel energised after the failure (old DRIVE / MOTORTEST resumed), ") + where);
    CHECK(commDesiredLeft() == 0 && commDesiredRight() == 0 && !commTestModeActive(),
          std::string("a stored request survived the recovery, ") + where);
    CHECK(commSafetyStopLatched(), std::string("recovery cleared the latch, ") + where);

    // Only RESET acknowledges it, and RESET commands no motion.
    std::string r = command("RESET");
    CHECK(has(r, "\"result\":\"ACCEPTED\"") && !commSafetyStopLatched(), r);
    pass(50);
    CHECK(gEnergisedScans == afterFault, std::string("RESET started a wheel, ") + where);
}

static void test13_latchEvidence(void)
{
    for (int hang = 0; hang <= 1; hang++) {
        latchCase(WHILE_DRIVING,   "DRIVE", ",\"left\":150,\"right\":150", hang != 0);
        latchCase(WHILE_DRIVING,   "STOP",  "",                            hang != 0);
        latchCase(WHILE_MOTORTEST, "PING",  "",                            hang != 0);
        latchCase(WHILE_MOTORTEST, "STOP",  "",                            hang != 0);
    }
}

// ===========================================================================
// 14. Watchdog still fires, and the drive stays cut, across a hung write
// ===========================================================================

static void test14_watchdogAcrossHungWrite(void)
{
    boot(true);
    int seq = send("DRIVE", ",\"left\":150,\"right\":150");
    pass(1);
    CHECK(has(ackFor(seq), "\"result\":\"ACCEPTED\""), ackFor(seq));
    uint32_t lastCmd = gNow;
    pass(500);
    CHECK(energisedWheels() == 4, "setup: not driving");

    armFailure(3, true);                         // channel 1 hangs 1 s, then fails
    pass(1);
    CHECK(directionPinsAllLow() && gCutViolations == 0, "not cut in the failing pass");
    long afterFault = gEnergisedScans;
    gTx.clear();
    while (!commTimedOut() && gNow - lastCmd < 10000) pass(1);
    CHECK(commTimedOut(), "watchdog never fired after the hung write");
    CHECK(gNow - lastCmd <= COMMAND_TIMEOUT_MS + 5, "watchdog fired late");
    CHECK(!lastLine("\"event\":\"COMMAND_TIMEOUT\"").empty(), "no COMMAND_TIMEOUT event");
    passMs(5000);
    CHECK(gEnergisedScans == afterFault && directionPinsAllLow(), "output after the hung write");
    CHECK(commSafetyStopLatched(), "interrupted output did not latch safety_stop");
}

// ===========================================================================
// Acknowledgement helpers
// ===========================================================================

static void waitRecovered(const char *where)
{
    uint32_t t0 = gNow;
    while (!motorDriveAvailable() && gNow - t0 < 40000) pass(1);
    CHECK(motorDriveAvailable(), std::string("no recovery, ") + where);
}

// After recovery, with the loss unacknowledged: the latch is still shown, and
// neither DRIVE nor MOTORTEST can move a wheel.
static void expectAckStillRequired(const char *where, long energisedBaseline)
{
    std::string tel = freshTelemetry();
    CHECK(has(tel, "\"safety_stop\":true") && has(tel, "\"motor_drive_available\":true"),
          std::string("after recovery, latch not shown: ") + where + " " + tel);
    std::string a = command("DRIVE", ",\"left\":150,\"right\":150");
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, "\"reason\":\"DRIVE_LOSS_UNACKNOWLEDGED\""),
          std::string("DRIVE not refused before acknowledgement, ") + where + " " + a);
    a = command("MOTORTEST", ",\"motor\":0,\"power\":150,\"ms\":500,\"onblocks\":true");
    CHECK(has(a, "\"result\":\"REJECTED\"") && has(a, "\"reason\":\"DRIVE_LOSS_UNACKNOWLEDGED\""),
          std::string("MOTORTEST not refused before acknowledgement, ") + where + " " + a);
    pass(50);
    CHECK(gEnergisedScans == energisedBaseline,
          std::string("a wheel moved before the acknowledgement, ") + where);
    CHECK(commSafetyStopLatched(), std::string("latch cleared by DRIVE/MOTORTEST, ") + where);
    CHECK(commDesiredLeft() == 0 && commDesiredRight() == 0 && !commTestModeActive(),
          std::string("a request was stored before the acknowledgement, ") + where);
}

// An eligible RESET (drive available, reported, and continuously so for the
// whole ACK_SETTLE_MS settle period, then reported again) clears it, still
// commands no motion, and a FRESH DRIVE then works.
static void acknowledgeThenDrive(const char *where, long energisedBaseline)
{
    passMs(ACK_SETTLE_MS + TELEMETRY_INTERVAL_MS + 50);
    std::string r = command("RESET");
    CHECK(has(r, "\"result\":\"ACCEPTED\"") && !commSafetyStopLatched(),
          std::string("eligible RESET did not clear the latch, ") + where + " " + r);
    pass(50);
    CHECK(gEnergisedScans == energisedBaseline, std::string("RESET started a wheel, ") + where);
    driveAccepted(120, 120);
    command("STOP");
}

// ===========================================================================
// 15. From REST: a failed non-zero DRIVE / MOTORTEST latches at EVERY write
//     position, fast and hung; a pure stop write does not
// ===========================================================================

static void fromRestCase(bool motortest, int n, bool hang, bool viaCommand)
{
    char where[96];
    snprintf(where, sizeof(where), "from rest, %s, txn %d, %s%s", motortest ? "MOTORTEST m3" : "DRIVE",
             n, hang ? "hung" : "fast", viaCommand ? ", via command" : "");

    boot(true);
    command("STOP");
    pass(50);
    CHECK(directionPinsAllLow() && energisedWheels() == 0 && !commSafetyStopLatched(),
          std::string("setup: not at rest, ") + where);

    armFailure(n, hang);
    if (viaCommand) {
        send(motortest ? "MOTORTEST" : "DRIVE",
             motortest ? ",\"motor\":3,\"power\":150,\"ms\":3000,\"onblocks\":true"
                       : ",\"left\":150,\"right\":150");
        pass(1);
        CHECK(gCutViolations == 0, std::string("I2C traffic while a direction pin was HIGH, ") + where);
    } else if (motortest) {
        motorDriveChannel(3, 150);
    } else {
        driveDifferential(150, 150);
    }
    CHECK(gFailSeen, std::string("harness: no failure injected, ") + where);
    CHECK(directionPinsAllLow(), std::string("direction pin HIGH on return, ") + where);
    CHECK(energisedWheels() == 0, std::string("a wheel energised after the failure, ") + where);
    long afterFault = gEnergisedScans;

    pass(3);
    CHECK(commSafetyStopLatched(), std::string("failed non-zero request not latched, ") + where);

    waitRecovered(where);
    expectAckStillRequired(where, afterFault);
    acknowledgeThenDrive(where, afterFault);
}

static void test15_fromRest(void)
{
    for (int hang = 0; hang <= 1; hang++) {
        // DRIVE 150/150 from rest: ch k enable-off = txn 2k+1, duty = 2k+2.
        for (int n = 1; n <= 8; n++) {
            fromRestCase(false, n, hang != 0, false);
            fromRestCase(false, n, hang != 0, true);
        }
        // MOTORTEST motor 3 from rest: txns 1..3 stop writes of ch 0..2,
        // 4 = ch3 enable-off, 5 = ch3 duty.
        for (int n = 1; n <= 5; n++) {
            fromRestCase(true, n, hang != 0, false);
            fromRestCase(true, n, hang != 0, true);
        }

        // A pure stop with nothing live: a failed write is a bus fault, not a
        // lost motion request -- no latch.
        boot(true);
        command("STOP");
        pass(50);
        armFailure(1, hang != 0);
        driveDifferential(0, 0);
        CHECK(gFailSeen && directionPinsAllLow(), "pure stop: harness / cut");
        pass(3);
        CHECK(!motorDriveAvailable() && !commSafetyStopLatched(),
              hang ? "pure stop write (hung) latched safety_stop" : "pure stop write latched safety_stop");
    }
}

// ===========================================================================
// 16. RESET delivered INSIDE the failing write cannot acknowledge the loss
// ===========================================================================

static void resetInsideFailureCase(MotionKind kind, bool hang)
{
    char where[96];
    snprintf(where, sizeof(where), "RESET inside a %s failure %s", hang ? "hung" : "fast",
             kind == WHILE_DRIVING ? "while driving" : "in MOTORTEST");

    boot(true);
    if (kind == WHILE_DRIVING) {
        driveAccepted(150, 150);
    } else {
        std::string a = command("MOTORTEST", ",\"motor\":0,\"power\":150,\"ms\":3000,\"onblocks\":true");
        CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
        pass(20);
    }

    int seq;
    gInjectFrame = frameFor("RESET", "", &seq);
    gInjected = false;
    armFailure(kind == WHILE_DRIVING ? 3 : 1, hang);
    pass(1);
    CHECK(gFailSeen && gInjected && directionPinsAllLow(), std::string("harness / cut, ") + where);
    long afterFault = gEnergisedScans;

    std::string r = awaitAck(seq, 3000);
    CHECK(has(r, "\"result\":\"ACCEPTED\""), std::string("RESET not answered as before, ") + where + " " + r);
    pass(2);
    CHECK(!motorDriveAvailable(), std::string("setup: drive already back, ") + where);
    CHECK(commSafetyStopLatched(), std::string("RESET while the drive was down cleared the latch, ") + where);

    waitRecovered(where);
    CHECK(commSafetyStopLatched(), std::string("latch not set after recovery, ") + where);
    CHECK(gEnergisedScans == afterFault, std::string("stored DRIVE / MOTORTEST resumed, ") + where);
    expectAckStillRequired(where, afterFault);
    acknowledgeThenDrive(where, afterFault);
}

static void test16_resetInsideFailure(void)
{
    for (int hang = 0; hang <= 1; hang++) {
        resetInsideFailureCase(WHILE_DRIVING,   hang != 0);
        resetInsideFailureCase(WHILE_MOTORTEST, hang != 0);
    }
}

// ===========================================================================
// 17. RESET that arrives DURING the (successful) recovery attempt, and is only
//     processed after the drive is back: it was sent before the recovery was
//     reported, so it cannot acknowledge the loss
// ===========================================================================

static void resetDuringRecoveryCase(uint32_t probeMs)
{
    char where[96];
    snprintf(where, sizeof(where), "RESET during a %lu ms successful recovery attempt", (unsigned long)probeMs);

    boot(true);
    driveAccepted(150, 150);
    armFailure(3, false);
    pass(3);
    CHECK(!motorDriveAvailable() && commSafetyStopLatched(), std::string("setup: no loss, ") + where);
    long afterFault = gEnergisedScans;

    int seq;
    gInjectFrame = frameFor("RESET", "", &seq);
    gInjected = false;
    P.slowProbeMs = probeMs;                 // the next recovery probe: slow, then OK
    gTx.clear();
    uint32_t t0 = gNow;
    while (!(gInjected && motorDriveAvailable()) && gNow - t0 < 40000) pass(1);
    CHECK(gInjected && motorDriveAvailable(), std::string("harness: RESET not delivered in a successful attempt, ") + where);

    std::string r = awaitAck(seq, 3000);
    CHECK(has(r, "\"result\":\"ACCEPTED\""), std::string("RESET not answered, ") + where + " " + r);
    if (probeMs >= TELEMETRY_INTERVAL_MS) {
        // The hard ordering: a TELEMETRY frame reporting the drive available
        // went out BEFORE this RESET was processed (but after it arrived).
        size_t availTel = gTx.find("\"motor_drive_available\":true");
        size_t ackPos   = gTx.find(r);
        CHECK(availTel != std::string::npos && ackPos != std::string::npos && availTel < ackPos,
              std::string("harness: no availability report between arrival and processing, ") + where);
    }
    pass(2);
    CHECK(commSafetyStopLatched(), std::string("a RESET sent before the recovery was reported acknowledged the loss, ") + where);
    CHECK(gEnergisedScans == afterFault, std::string("a wheel moved, ") + where);

    expectAckStillRequired(where, afterFault);
    acknowledgeThenDrive(where, afterFault);
}

static void test17_resetDuringRecovery(void)
{
    resetDuringRecoveryCase(5);                         // processed before any report
    resetDuringRecoveryCase(TELEMETRY_INTERVAL_MS + 100); // a report goes out first
}

// ===========================================================================
// 18. A SECOND outage while still unacknowledged (no new loss record: the
//     rover is at rest) voids the earlier "drive available" report: a RESET
//     sent during that outage cannot acknowledge
// ===========================================================================

static void test18_secondOutageVoidsReport(void)
{
    boot(true);
    driveAccepted(150, 150);
    armFailure(3, false);
    pass(3);                                       // loss 1: latched, unacknowledged
    waitRecovered("outage 1");
    std::string tel = freshTelemetry();            // drive reported available
    CHECK(has(tel, "\"motor_drive_available\":true") && has(tel, "\"safety_stop\":true"), tel);
    long afterFault = gEnergisedScans;

    // Outage 2, at rest: a pure-stop write fails -> no new loss record.
    armFailure(1, false);
    pass(3);
    CHECK(!motorDriveAvailable(), "setup: second outage did not happen");
    CHECK(commSafetyStopLatched(), "latch lost in the second outage");

    // A RESET sent during outage 2, received inside its (fast, successful)
    // recovery attempt and processed right after it.
    int seq;
    gInjectFrame = frameFor("RESET", "", &seq);
    gInjected = false;
    P.slowProbeMs = 5;
    uint32_t t0 = gNow;
    while (!(gInjected && motorDriveAvailable()) && gNow - t0 < 40000) pass(1);
    CHECK(gInjected && motorDriveAvailable(), "harness: RESET not delivered in the second recovery");
    std::string r = awaitAck(seq, 3000);
    CHECK(has(r, "\"result\":\"ACCEPTED\""), r);
    pass(2);
    CHECK(commSafetyStopLatched(), "a RESET sent during the second outage acknowledged the loss (stale report)");
    CHECK(gEnergisedScans == afterFault, "a wheel moved");

    expectAckStillRequired("second outage", afterFault);
    acknowledgeThenDrive("second outage", afterFault);
}

// ===========================================================================
// Acknowledgement-timing helpers (settle window)
// ===========================================================================

static void watchReports(void)
{
    gAvailReports       = 0;
    gFirstAvailReportMs = 0;
}

// A drive loss while driving: latched, drive down. Availability reports are
// watched from here, so report 1 is the first one after the recovery.
static long lossOutage(void)
{
    boot(true);
    driveAccepted(150, 150);
    armFailure(3, false);
    pass(3);
    CHECK(!motorDriveAvailable() && commSafetyStopLatched(), "setup: no drive loss");
    watchReports();
    return gEnergisedScans;
}

static void waitFirstReport(const char *where)
{
    uint32_t t0 = gNow;
    while (gAvailReports < 1 && gNow - t0 < 40000) pass(1);
    CHECK(gAvailReports >= 1, std::string("harness: no availability report, ") + where);
}

static void passUntil(uint32_t t)
{
    while ((int32_t)(gNow - t) < 0) pass(1);
}

// A second outage at rest: a pure-stop write fails (no new loss record).
static void pureStopOutage(const char *where)
{
    armFailure(1, false);
    pass(3);
    CHECK(gFailSeen && !motorDriveAvailable(), std::string("setup: second outage did not happen, ") + where);
}

// A complete RESET into the ring buffer; true if it acknowledged the loss.
static bool resetAcks(const char *where)
{
    std::string r = command("RESET");
    CHECK(has(r, "\"result\":\"ACCEPTED\""), std::string("RESET not answered, ") + where + " " + r);
    return !commSafetyStopLatched();
}

// ===========================================================================
// 19. A RESET still in the UART HARDWARE FIFO when an availability report is
//     queued (invisible to Serial.available()) cannot acknowledge
// ===========================================================================

static void test19_resetInFifoAtReport(void)
{
    for (int n = 1; n <= 3; n++) {                  // reports inside the settle period
        char where[96];
        snprintf(where, sizeof(where), "RESET in the HW FIFO when availability report %d is queued", n);
        long base = lossOutage();
        int seq;
        gFifoOnReport    = frameFor("RESET", "", &seq);
        gFifoOnReportN   = n;
        gReportDelivered = false;
        waitRecovered(where);
        uint32_t t0 = gNow;
        while (!gReportDelivered && gNow - t0 < 5000) pass(1);
        CHECK(gReportDelivered, std::string("harness: FIFO delivery missing, ") + where);
        std::string a = awaitAck(seq, 1000);
        CHECK(has(a, "\"result\":\"ACCEPTED\""), std::string("RESET not answered, ") + where + " " + a);
        pass(2);
        CHECK(commSafetyStopLatched(), std::string("acknowledged: ") + where);
        expectAckStillRequired(where, base);
        acknowledgeThenDrive(where, base);
    }
}

// ===========================================================================
// 20. A RESET arriving 20 / 100 / 150 ms after the first availability report
//     is QUEUED -- before the Pi can have it -- cannot acknowledge
// ===========================================================================

static void test20_resetJustAfterFirstReport(void)
{
    static const uint32_t kDelays[] = { 20, 100, 150 };
    for (size_t i = 0; i < sizeof(kDelays) / sizeof(kDelays[0]); i++) {
        char where[96];
        snprintf(where, sizeof(where), "RESET %lu ms after the first availability report", (unsigned long)kDelays[i]);
        long base = lossOutage();
        waitRecovered(where);
        waitFirstReport(where);
        passUntil(gFirstAvailReportMs + kDelays[i]);
        CHECK(!resetAcks(where), std::string("acknowledged: ") + where);
        CHECK(gEnergisedScans == base, std::string("a wheel moved, ") + where);
        expectAckStillRequired(where, base);
        acknowledgeThenDrive(where, base);
    }
}

// ===========================================================================
// 21. After the FULL settle period -- and only for bytes received after the
//     settle snapshot -- a RESET acknowledges
// ===========================================================================

static void test21_resetAfterSettle(void)
{
    const char *where = "settle period";
    long base = lossOutage();
    waitRecovered(where);
    waitFirstReport(where);
    uint32_t t1 = gFirstAvailReportMs;

    passUntil(t1 + ACK_SETTLE_MS - 50);
    CHECK(!resetAcks("50 ms before the settle period ends"), "acknowledged 50 ms before the settle period ended");

    // Bytes that are already in the ring buffer when the post-settle report
    // (the snapshot) is queued were received before it: not eligible.
    int seq;
    gRingOnSettleReport = frameFor("RESET", "", &seq);
    gReportDelivered    = false;
    uint32_t t0 = gNow;
    while (!gReportDelivered && gNow - t0 < 2 * ACK_SETTLE_MS) pass(1);
    CHECK(gReportDelivered, "harness: no post-settle report");
    std::string a = awaitAck(seq, 1000);
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
    pass(2);
    CHECK(commSafetyStopLatched(), "a RESET already in the ring buffer at the settle snapshot acknowledged");

    // Received after the snapshot: eligible. Still no motion until a DRIVE.
    pass(5);
    CHECK(resetAcks("after the settle snapshot"), "an eligible RESET after the full settle period did not acknowledge");
    pass(50);
    CHECK(gEnergisedScans == base, "the acknowledging RESET started a wheel");
    driveAccepted(120, 120);
    command("STOP");
}

// ===========================================================================
// 22. A new outage during the settle period voids it: the full period restarts
// ===========================================================================

static void test22_outageRestartsSettle(void)
{
    long base = lossOutage();
    waitRecovered("outage 1");
    waitFirstReport("outage 1");
    uint32_t t1 = gFirstAvailReportMs;
    passUntil(t1 + 600);                                // inside the settle period

    pureStopOutage("during settle");
    watchReports();
    waitRecovered("outage 2");
    waitFirstReport("outage 2");
    uint32_t t2 = gFirstAvailReportMs;
    CHECK(t2 - t1 >= ACK_SETTLE_MS, "harness: the first settle period would not have expired");

    passUntil(t2 + 100);
    CHECK(!resetAcks("100 ms after the second recovery"),
          "acknowledged 100 ms after a second recovery: the settle period did not restart");
    passUntil(t2 + ACK_SETTLE_MS - 50);
    CHECK(!resetAcks("50 ms before the restarted settle period ends"),
          "acknowledged before the restarted settle period ended");
    CHECK(gEnergisedScans == base, "a wheel moved");
    expectAckStillRequired("restarted settle", base);
    acknowledgeThenDrive("restarted settle", base);
}

// ===========================================================================
// 23. A RESET line whose first bytes were READ before the settle snapshot
//     stays ineligible when it is completed after it
// ===========================================================================

static void test23_partialLineAcrossSnapshot(void)
{
    long base = lossOutage();
    waitRecovered("partial line");
    waitFirstReport("partial line");
    uint32_t t1 = gFirstAvailReportMs;

    passUntil(t1 + ACK_SETTLE_MS - 30);
    int seq;
    std::string f = frameFor("RESET", "", &seq);
    size_t half = f.size() / 2;
    gRx += f.substr(0, half);
    pass(1);                                            // first half READ now
    passUntil(t1 + ACK_SETTLE_MS + TELEMETRY_INTERVAL_MS + 50);   // snapshot has gone out
    gRx += f.substr(half);
    std::string a = awaitAck(seq, 1000);
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
    pass(2);
    CHECK(commSafetyStopLatched(), "a RESET line begun before the snapshot acknowledged once completed after it");
    CHECK(gEnergisedScans == base, "a wheel moved");
    expectAckStillRequired("partial line", base);
    acknowledgeThenDrive("partial line", base);
}

// ===========================================================================
// 24. RESET followed by DRIVE in one batch
// ===========================================================================

static void test24_resetThenDriveBatch(void)
{
    long base = lossOutage();
    waitRecovered("batch");
    waitFirstReport("batch");

    // Ineligible RESET (inside the settle period): the DRIVE stays gated.
    passUntil(gFirstAvailReportMs + 100);
    int rs, ds;
    // Two statements: the seq order must match the byte order (an operand
    // order inside one expression is unspecified).
    std::string batch = frameFor("RESET", "", &rs);
    batch += frameFor("DRIVE", ",\"left\":150,\"right\":150", &ds);
    gRx += batch;
    pass(3);
    CHECK(has(ackFor(rs), "\"result\":\"ACCEPTED\""), ackFor(rs));
    CHECK(has(ackFor(ds), "\"result\":\"GATED\"") && has(ackFor(ds), "\"reason\":\"DRIVE_LOSS_UNACKNOWLEDGED\""),
          "DRIVE after an ineligible RESET in the same batch not gated: " + ackFor(ds));
    CHECK(commSafetyStopLatched(), "ineligible RESET in a batch acknowledged");
    passMs(200);
    CHECK(gEnergisedScans == base && energisedWheels() == 0, "motor output after an ineligible RESET + DRIVE batch");

    // Eligible RESET (after the settle period): the fresh DRIVE behind it runs.
    passMs(ACK_SETTLE_MS + TELEMETRY_INTERVAL_MS + 50);
    batch = frameFor("RESET", "", &rs);
    batch += frameFor("DRIVE", ",\"left\":150,\"right\":150", &ds);
    gRx += batch;
    pass(3);
    CHECK(has(ackFor(rs), "\"result\":\"ACCEPTED\"") && !commSafetyStopLatched(),
          "eligible RESET in a batch did not acknowledge");
    CHECK(has(ackFor(ds), "\"result\":\"ACCEPTED\""), "fresh DRIVE after an eligible RESET: " + ackFor(ds));
    pass(20);
    CHECK(energisedWheels() == 4, "fresh DRIVE after an eligible RESET did not drive");
    command("STOP");
}

// ===========================================================================
// 25. No stale eligibility across a failed recovery attempt, a slow successful
//     one and a second outage: a RESET line begun inside an OPEN window, then
//     interrupted by an outage, does not acknowledge when completed later
// ===========================================================================

static void test25_noStaleStateAcrossOutages(void)
{
    long base = lossOutage();
    waitRecovered("outage 1");
    waitFirstReport("outage 1");
    passUntil(gFirstAvailReportMs + ACK_SETTLE_MS + TELEMETRY_INTERVAL_MS + 50);   // window open

    int seq;
    std::string f = frameFor("RESET", "", &seq);
    size_t half = f.size() / 2;
    gRx += f.substr(0, half);
    pass(1);                                            // begun inside the open window

    pureStopOutage("second outage");
    P.present = false;                                  // the next attempt fails
    uint16_t a0 = pca9685RecoveryAttempts();
    uint32_t t0 = gNow;
    while (pca9685RecoveryAttempts() == a0 && gNow - t0 < 10000) pass(1);
    CHECK(pca9685RecoveryAttempts() > a0 && !motorDriveAvailable(), "harness: no failed recovery attempt");
    P.present     = true;
    P.slowProbeMs = 300;                                // then a slow, successful one
    watchReports();
    waitRecovered("slow recovery");
    waitFirstReport("slow recovery");
    passUntil(gFirstAvailReportMs + ACK_SETTLE_MS + TELEMETRY_INTERVAL_MS + 50);   // NEW window open

    gRx += f.substr(half);
    std::string a = awaitAck(seq, 1000);
    CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
    pass(2);
    CHECK(commSafetyStopLatched(),
          "a RESET line begun before the second outage acknowledged after the recovery (stale per-line state)");
    CHECK(gEnergisedScans == base, "a wheel moved");

    CHECK(resetAcks("fresh RESET in the new window"), "a fresh eligible RESET did not acknowledge");
    pass(50);
    CHECK(gEnergisedScans == base, "the acknowledging RESET started a wheel");
    driveAccepted(120, 120);
    command("STOP");
}

// ===========================================================================
// 26. An OPEN acknowledgement window is voided by an outage: after the next
//     recovery, a RESET needs a full new settle period again
// ===========================================================================

static void test26_openWindowVoidedByOutage(void)
{
    long base = lossOutage();
    waitRecovered("outage 1");
    waitFirstReport("outage 1");
    passUntil(gFirstAvailReportMs + ACK_SETTLE_MS + TELEMETRY_INTERVAL_MS + 50);   // window open
    CHECK(commSafetyStopLatched(), "setup: latch not held");

    pureStopOutage("with the window open");
    watchReports();
    waitRecovered("outage 2");
    waitFirstReport("outage 2");
    passUntil(gFirstAvailReportMs + 100);
    CHECK(!resetAcks("100 ms after recovering from an outage that hit an open window"),
          "an acknowledgement window opened before an outage was still open after it");
    passUntil(gFirstAvailReportMs + ACK_SETTLE_MS - 50);
    CHECK(!resetAcks("50 ms before the new settle period ends"),
          "acknowledged before the new settle period ended");
    CHECK(gEnergisedScans == base, "a wheel moved");
    expectAckStillRequired("open window voided", base);
    acknowledgeThenDrive("open window voided", base);
}

// ===========================================================================
// LOOP-STALL GUARD (Phase 2)
// ===========================================================================

// Keep a DRIVE alive (refreshed every 500 ms, as the Pi does) for `ms`, or
// until `stop()` is true.
template <typename F>
static void driveWhile(uint32_t ms, F stop, int l = 150, int r = 150)
{
    char f[64];
    snprintf(f, sizeof(f), ",\"left\":%d,\"right\":%d", l, r);
    uint32_t end = gNow + ms, next = gNow;
    while ((int32_t)(gNow - end) < 0 && !stop()) {
        if ((int32_t)(gNow - next) >= 0) { send("DRIVE", f); next = gNow + 500; }
        pass(1);
    }
}

static void driveFor(uint32_t ms) { driveWhile(ms, [] { return false; }); }

enum BlockKind { BLK_PCA, BLK_TCA, BLK_VL53, BLK_GPS };

static const char *blockName(BlockKind k)
{
    switch (k) {
        case BLK_PCA:  return "PCA9685 write";
        case BLK_TCA:  return "TCA9548A select";
        case BLK_VL53: return "VL53L0X read";
        default:       return "GPS transaction";
    }
}

// Drive, then make the chosen I2C user hang 1 s while the wheels are driven.
static void stallWhileDriving(BlockKind k)
{
    boot(true);
    driveAccepted(150, 150);
    switch (k) {
        case BLK_PCA:  armFailure(3, true);  break;   // ch1 enable-off: hangs, fails
        case BLK_TCA:  gTcaHangNext  = 1;    break;
        case BLK_VL53: gVl53HangNext = 1;    break;
        case BLK_GPS:  gGpsHangNext  = 1;    break;
    }
    // GPS is absent here and backs off (5 s doubling to GPS_BACKOFF_MAX_MS;
    // gps.cpp state also carries over between this harness's simulated boots),
    // so keep the drive alive until the hang has actually happened.
    driveWhile(GPS_BACKOFF_MAX_MS + 10000, [k] {
        return stallGuardTrips() > 0 ||
               (k == BLK_PCA  && gFailSeen)          ||
               (k == BLK_TCA  && gTcaHangNext  == 0) ||
               (k == BLK_VL53 && gVl53HangNext == 0) ||
               (k == BLK_GPS  && gGpsHangNext  == 0);
    });
}

// 27. A 1 s hang in ANY I2C user while driving: the guard cuts all eight
//     direction pins within T_STALL + one period of the hang starting.
static void test27_guardCutsEveryBlockingPath(void)
{
    static const BlockKind kKinds[] = { BLK_PCA, BLK_TCA, BLK_VL53, BLK_GPS };
    for (size_t i = 0; i < 4; i++) {
        std::string w = std::string("hung ") + blockName(kKinds[i]) + " while driving";
        stallWhileDriving(kKinds[i]);
        CHECK(stallGuardTrips() == 1, "guard did not trip: " + w);
        CHECK(gBlockCutMs != 0, "pins not cut during the hang: " + w);
        uint32_t latency = gBlockCutMs - gBlockStartMs;
        char msg[160];
        snprintf(msg, sizeof(msg), "cut %lu ms after the hang began (limit %lu): %s",
                 (unsigned long)latency,
                 (unsigned long)(STALL_GUARD_T_STALL_MS + STALL_GUARD_PERIOD_MS), w.c_str());
        CHECK(latency <= STALL_GUARD_T_STALL_MS + STALL_GUARD_PERIOD_MS, msg);
        CHECK(directionPinsAllLow() && energisedWheels() == 0, "output after the trip: " + w);
        CHECK(gHighWhileTripped == 0, "a direction pin raised while tripped: " + w);
        pass(3);
        CHECK(commSafetyStopLatched(), "trip not latched: " + w);
        CHECK(stallGuardTripped(), "trip not sticky: " + w);
    }
}

// 28. No false trip: a minute of driving with the ultrasonic busy-waiting
//     ~17 ms per ping (290 cm echo) and GPS transactions taking 16 ms.
static void test28_noFalseTrip(void)
{
    boot(true);
    gFrontCm   = 290.0;
    gPulseBusy = true;
    gGpsDelayMs = 16;
    pass(500);                                   // front windows refill at 290 cm
    driveAccepted(150, 150);
    gLongestPass = 0;
    // At least 60 s, and until GPS has hit the bus (16 ms each) at least
    // twice WHILE driving -- its backoff can be long in this harness.
    const int      gps0 = gGpsTxns;
    const uint32_t t0   = gNow;
    driveWhile(GPS_BACKOFF_MAX_MS + 70000, [gps0, t0] {
        return gNow - t0 >= 60000 && gGpsTxns - gps0 >= 2;
    });
    CHECK(stallGuardTrips() == 0, "the guard tripped on legitimate loop work");
    CHECK(energisedWheels() == 4, "driving stopped");
    CHECK(gGpsTxns - gps0 >= 2, "harness: GPS never hit the bus while driving");
    char msg[96];
    snprintf(msg, sizeof(msg), "longest pass %lu ms", (unsigned long)gLongestPass);
    CHECK(gLongestPass < STALL_GUARD_T_STALL_MS, msg);
    command("STOP");
}

// 29. Resume race: a PCA write is SLOW but SUCCEEDS (300 ms) at every write
//     position; the guard trips meanwhile; when the write returns, nothing
//     raises a direction pin again.
static void test29_resumeRace(void)
{
    for (int n = 1; n <= 8; n++) {
        char w[64];
        snprintf(w, sizeof(w), "slow successful write %d", n);
        boot(true);
        driveAccepted(150, 150);
        gSlowOkAt = n; gSlowOkTxns = 0; gSlowOkMs = 300;
        pass(1);
        CHECK(stallGuardTrips() == 1, std::string("guard did not trip: ") + w);
        CHECK(gHighWhileTripped == 0, std::string("a direction pin raised after the trip: ") + w);
        CHECK(directionPinsAllLow(), std::string("pins not LOW when the pass ended: ") + w);
        long base = gEnergisedScans;
        pass(200);
        CHECK(gEnergisedScans == base && gHighWhileTripped == 0, std::string("output after the trip: ") + w);
        CHECK(commSafetyStopLatched() && commDesiredLeft() == 0, std::string("not latched/discarded: ") + w);
        CHECK(motorDriveAvailable(), std::string("the drive itself is fine (slow, not failed): ") + w);
    }
}

// 30. The trip enters the Phase 1 acknowledgement flow: DRIVE, MOVE and
//     MOTORTEST refused; an early RESET does not clear it; an eligible RESET
//     after the settle window clears trip and latch; a fresh DRIVE drives.
static void test30_tripAckContract(void)
{
    stallWhileDriving(BLK_TCA);
    CHECK(stallGuardTripped(), "setup: no trip");
    long base = gEnergisedScans;
    pass(5);
    std::string a = command("MOVE", ",\"dir\":\"F\",\"speed\":150");
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, "DRIVE_LOSS_UNACKNOWLEDGED"), "MOVE not refused: " + a);
    CHECK(!resetAcks("RESET right after the trip"), "an early RESET acknowledged a guard trip");
    CHECK(stallGuardTripped(), "an early RESET cleared the trip");
    expectAckStillRequired("guard trip", base);
    CHECK(stallGuardTripped(), "DRIVE/MOTORTEST cleared the trip");
    std::string sys = freshSystem();
    CHECK(has(sys, "\"stall_guard\":\"TRIPPED\"") && jsonInt(sys, "stall_guard_trips") == 1, sys);
    acknowledgeThenDrive("guard trip", base);
    CHECK(!stallGuardTripped(), "eligible RESET did not clear the trip");
    sys = freshSystem();
    CHECK(has(sys, "\"stall_guard\":\"ARMED\""), sys);
}

// 31. A PCA write that HANGS AND FAILS while driving: guard trip + drive loss
//     + PCA recovery -- the recovered drive still does not move before the
//     acknowledgement.
static void test31_tripPlusRecovery(void)
{
    stallWhileDriving(BLK_PCA);
    CHECK(stallGuardTripped() && !motorDriveAvailable(), "setup: trip + PCA loss expected");
    long base = gEnergisedScans;
    waitRecovered("trip + PCA failure");
    CHECK(stallGuardTripped(), "PCA recovery cleared the guard trip");
    expectAckStillRequired("trip + PCA recovery", base);
    acknowledgeThenDrive("trip + PCA recovery", base);
}

// 32. Not armed during setup(): a 1 s hang inside setup() with a direction
//     pin HIGH does not trip; the guard is armed when setup() ends.
static void test32_notArmedInSetup(void)
{
    gBootTcaHang   = 1;                        // the TCA probe in setup() hangs 1 s ...
    gBootForceHigh = true;                     // ... with a direction pin HIGH
    boot(true);
    CHECK(!gArmedInBlock, "the guard was armed during setup()");
    CHECK(stallGuardTrips() == 0, "the guard tripped during setup()");
    CHECK(stallGuardArmed(), "the guard is not armed after setup()");
    // Control: the same forced block AFTER setup() does trip.
    gForceHighInBlock = true;
    advance(500);
    CHECK(stallGuardTrips() == 1 && directionPinsAllLow(), "control: an armed guard did not trip");
}

// 33. While tripped every tick re-asserts LOW, whatever drove a pin HIGH.
static void test33_reassertWhileTripped(void)
{
    stallWhileDriving(BLK_VL53);
    CHECK(stallGuardTripped(), "setup: no trip");
    for (int i = 0; i < 8; i++) {
        gPin[kMotorPins[i]] = HIGH;            // a rogue write outside the contract
        advance(STALL_GUARD_PERIOD_MS);
        CHECK(directionPinsAllLow(), "a tick while tripped did not re-assert LOW");
    }
}

// 34. Watchdog and telemetry after a trip: the 2 s watchdog still fires,
//     TELEMETRY shows the latch, DIAG SYSTEM the trip.
static void test34_watchdogTelemetryAfterTrip(void)
{
    boot(true);
    int seq = send("DRIVE", ",\"left\":150,\"right\":150");
    pass(1);
    CHECK(has(ackFor(seq), "\"result\":\"ACCEPTED\""), ackFor(seq));
    uint32_t lastCmd = gNow;
    pass(200);
    gTcaHangNext = 1;
    while (stallGuardTrips() == 0 && gNow - lastCmd < 1500) pass(1);
    CHECK(stallGuardTrips() == 1, "setup: no trip");
    gTx.clear();
    while (!commTimedOut() && gNow - lastCmd < 10000) pass(1);
    CHECK(commTimedOut(), "watchdog did not fire after a trip");
    CHECK(gNow - lastCmd <= COMMAND_TIMEOUT_MS + 5, "watchdog late after a trip");
    CHECK(!lastLine("\"event\":\"COMMAND_TIMEOUT\"").empty(), "no COMMAND_TIMEOUT event");
    std::string tel = freshTelemetry();
    CHECK(has(tel, "\"safety_stop\":true") && has(tel, "\"left_applied\":0,\"right_applied\":0"), tel);
    std::string sys = freshSystem();
    CHECK(has(sys, "\"stall_guard\":\"TRIPPED\""), sys);
}

// 35. No output, no trip: a hang while STOPPED is not a guard event.
static void test35_idleStallNoTrip(void)
{
    boot(true);
    command("STOP");
    pass(50);
    gTcaHangNext = 1;
    pass(100);
    CHECK(gTcaHangNext == 0, "setup: hang did not happen");
    CHECK(stallGuardTrips() == 0 && !stallGuardTripped(), "tripped with no output live");
    CHECK(!commSafetyStopLatched(), "latched with no output live");
}

// 36. millis() wraparound: no false trip across the wrap, and a hang after it
//     is still cut within the bound.
static void test36_wraparound(void)
{
    boot(true, 0, false, 0xFFFFFFFFUL - 3000UL);
    driveAccepted(150, 150);
    driveFor(6000);                            // crosses 0xFFFFFFFF -> 0
    CHECK(gNow < 0x80000000UL, "harness: clock did not wrap");
    CHECK(stallGuardTrips() == 0, "false trip across the millis() wrap");
    CHECK(energisedWheels() == 4, "driving stopped across the wrap");
    gTcaHangNext = 1;
    driveWhile(5000, [] { return gTcaHangNext == 0; });
    CHECK(stallGuardTrips() == 1, "no trip after the wrap");
    CHECK(gBlockCutMs - gBlockStartMs <= STALL_GUARD_T_STALL_MS + STALL_GUARD_PERIOD_MS,
          "cut too late after the wrap");
}

// 37. A STOP that arrives DURING the stall is processed before the motor step
//     of the next pass, so no direction write is ever attempted again -- the
//     guard's own evidence must still latch the drive loss.
static void test37_stopDuringStallStillLatches(void)
{
    static const BlockKind kKinds[] = { BLK_TCA, BLK_GPS };
    for (size_t i = 0; i < 2; i++) {
        std::string w = std::string("STOP delivered inside a hung ") + blockName(kKinds[i]);
        boot(true);
        driveAccepted(150, 150);
        // The STOP's seq must be newer than every DRIVE the wait below sends
        // (one per 500 ms for at most ~310 s), or it would be a stale replay.
        int base = gSeq;
        gSeq += 1000;
        int seq;
        gInjectFrame = frameFor("STOP", "", &seq);
        gSeq = base;
        gInjected = false;
        if (kKinds[i] == BLK_TCA) gTcaHangNext = 1; else gGpsHangNext = 1;
        driveWhile(GPS_BACKOFF_MAX_MS + 10000, [] { return gInjected; });
        CHECK(gInjected && stallGuardTrips() == 1, "harness: no trip / no delivery, " + w);
        gSeq = seq;                                // later frames continue after it
        std::string a = awaitAck(seq, 3000);
        CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
        pass(3);
        CHECK(commSafetyStopLatched(), "a guard trip was not latched: " + w);
        CHECK(stallGuardTripped(), "trip not sticky: " + w);
        std::string d = command("DRIVE", ",\"left\":150,\"right\":150");
        CHECK(has(d, "DRIVE_LOSS_UNACKNOWLEDGED"), "DRIVE not refused after the trip: " + d);
    }
}

// ===========================================================================
// LOOP-STALL GUARD LIVENESS (Phase 2, F1)
// ===========================================================================

static bool isNotRunning(void)
{
    return !stallGuardOperational() && !motorDriveAvailable() &&
           strcmp(stallGuardStateName(), "NOT_RUNNING") == 0 &&
           strcmp(motorDriveStatusName(), "STALL_GUARD_NOT_RUNNING") == 0;
}

// Every motion command refused as MOTOR_PWM_UNAVAILABLE; no direction pin raised.
static void expectMotionRefusedNoGuard(const char *where)
{
    int  highs = gDirHighWrites;
    long base  = gEnergisedScans;
    std::string a = command("DRIVE", ",\"left\":150,\"right\":150");
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, "\"reason\":\"MOTOR_PWM_UNAVAILABLE\""),
          std::string("DRIVE not refused, ") + where + " " + a);
    a = command("MOVE", ",\"dir\":\"F\",\"speed\":150");
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, "\"reason\":\"MOTOR_PWM_UNAVAILABLE\""),
          std::string("MOVE not refused, ") + where + " " + a);
    a = command("MOTORTEST", ",\"motor\":0,\"power\":150,\"ms\":500,\"onblocks\":true");
    CHECK(has(a, "\"result\":\"REJECTED\"") && has(a, "\"reason\":\"MOTOR_PWM_UNAVAILABLE\""),
          std::string("MOTORTEST not refused, ") + where + " " + a);
    pass(100);
    CHECK(gDirHighWrites == highs && gEnergisedScans == base && directionPinsAllLow(),
          std::string("a direction pin was raised, ") + where);
    CHECK(commDesiredLeft() == 0 && commDesiredRight() == 0 && !commTestModeActive(),
          std::string("a refused request was stored, ") + where);
}

// 38. xTaskCreatePinnedToCore() fails: never operational, never armed, the
//     drive is unavailable from READY on, and nothing can raise a pin.
static void test38_guardNotCreated(void)
{
    gBootGuardCreateFails = true;
    boot(true);
    CHECK(isNotRunning() && !stallGuardArmed(), "a guard that was never created is reported running");
    std::string ready = lastLine("\"event\":\"READY\"");
    CHECK(has(ready, "\"motor_drive_available\":false"), "READY: " + ready);
    std::string sys = freshSystem();
    CHECK(has(sys, "\"stall_guard\":\"NOT_RUNNING\"") &&
          has(sys, "\"motor_drive_status\":\"STALL_GUARD_NOT_RUNNING\""), sys);
    expectMotionRefusedNoGuard("guard never created");
    CHECK(!stallGuardSetDirection(kMotorPins[0], HIGH, kMotorPins[1], LOW) && directionPinsAllLow(),
          "stallGuardSetDirection raised a pin with no guard task");
}

// 39. Created, but no tick yet: NOT operational. stallGuardBegin()'s bounded
//     wait for the first tick ends; READY says unavailable. The first REAL
//     tick -- and nothing the loop does -- makes it operational.
static void test39_createdNoTickYet(void)
{
    gBootGuardNoTicks = true;
    boot(true);
    CHECK(stallGuardArmed(), "harness: a created guard is armed at the end of setup()");
    CHECK(isNotRunning(), "operational before the first tick");
    std::string ready = lastLine("\"event\":\"READY\"");
    CHECK(has(ready, "\"motor_drive_available\":false"), "READY before the first tick: " + ready);
    passMs(2000);                                // loop() checks it hundreds of times
    CHECK(isNotRunning(), "the loop's own checks made the guard look alive");
    expectMotionRefusedNoGuard("no tick yet");
    CHECK(commSafetyStopLatched(), "a non-zero request refused for a guard not running was not recorded");
    (void)stallGuardTick(gNow);                  // the task's first tick
    CHECK(stallGuardOperational() && motorDriveAvailable(), "the first tick did not make it operational");
    advance(STALL_GUARD_LIVENESS_MS + 1);        // ... and no other
    CHECK(isNotRunning(), "operational with no tick for longer than the liveness window");
}

// 40. Regular ticks: available on every pass of a drive; the window edge is
//     exact (LIVENESS ms after the last tick: live; one more: not); a tick
//     restores it; an idle gap records nothing.
static void test40_regularTicksAndWindow(void)
{
    boot(true);
    CHECK(has(lastLine("\"event\":\"READY\""), "\"motor_drive_available\":true"), "READY with the guard ticking");
    driveAccepted(150, 150);
    bool always = true;
    driveWhile(3000, [&always] { if (!motorDriveAvailable()) always = false; return !always; });
    CHECK(always, "drive unavailable on some pass while the guard ticks");
    CHECK(energisedWheels() == 4 && !commSafetyStopLatched(), "driving was disturbed");
    command("STOP");
    pass(50);

    while ((gNow % STALL_GUARD_PERIOD_MS) != 0) advance(1);   // a tick just ran
    gGuardTicks = false;
    advance(STALL_GUARD_LIVENESS_MS);
    CHECK(stallGuardOperational() && motorDriveAvailable(), "stale at exactly the liveness window");
    advance(1);
    CHECK(isNotRunning(), "live one ms past the liveness window");
    gGuardTicks = true;
    advance(STALL_GUARD_PERIOD_MS);
    CHECK(stallGuardOperational() && motorDriveAvailable(), "a new tick did not restore liveness");
    pass(50);
    CHECK(!commSafetyStopLatched(), "an idle liveness gap (nothing live, nothing asked) latched a loss");
}

// Drive (or MOTORTEST), then the guard task stops ticking. Returns the
// energised-scan baseline once all direction pins are LOW again.
static long livenessLossWhileDriving(bool motortest = false)
{
    boot(true);
    if (motortest) {
        std::string a = command("MOTORTEST", ",\"motor\":0,\"power\":150,\"ms\":3000,\"onblocks\":true");
        CHECK(has(a, "\"result\":\"ACCEPTED\""), a);
        pass(20);
        CHECK(energisedWheels() == 1 && commTestModeActive(), "setup: MOTORTEST not running");
    } else {
        driveAccepted(150, 150);
    }
    gGuardTicks = false;
    const uint32_t t0 = gNow;                    // the last tick was < 1 period ago
    while (!directionPinsAllLow() && gNow - t0 < 2000) pass(1);
    char msg[120];
    snprintf(msg, sizeof(msg), "pins cut %lu ms after the ticks stopped (limit %lu)",
             (unsigned long)(gNow - t0),
             (unsigned long)(STALL_GUARD_LIVENESS_MS + STALL_GUARD_PERIOD_MS));
    CHECK(directionPinsAllLow() && gNow - t0 <= STALL_GUARD_LIVENESS_MS + STALL_GUARD_PERIOD_MS, msg);
    pass(3);
    return gEnergisedScans;
}

// 41. Liveness lost during active motion (DRIVE and MOTORTEST): pins cut
//     through stopMotors() (enables zeroed too), drive loss latched, MOTORTEST
//     cancelled, reported as NOT_RUNNING. Not a stall trip.
static void test41_lossDuringMotion(void)
{
    for (int mt = 0; mt <= 1; mt++) {
        std::string w = mt ? "MOTORTEST" : "DRIVE";
        livenessLossWhileDriving(mt != 0);
        CHECK(isNotRunning(), "not reported NOT_RUNNING: " + w);
        CHECK(energisedWheels() == 0 && enableOutputsAllOff(), "output left live: " + w);
        CHECK(commSafetyStopLatched(), "liveness loss during motion not latched: " + w);
        CHECK(!commTestModeActive() && commDesiredLeft() == 0 && commDesiredRight() == 0,
              "motion request kept: " + w);
        CHECK(stallGuardTrips() == 0 && !stallGuardTripped(), "counted as a stall trip: " + w);
        std::string tel = freshTelemetry();
        CHECK(has(tel, "\"safety_stop\":true") && has(tel, "\"motor_drive_available\":false"), tel);
        std::string sys = freshSystem();
        CHECK(has(sys, "\"stall_guard\":\"NOT_RUNNING\"") &&
              has(sys, "\"motor_drive_status\":\"STALL_GUARD_NOT_RUNNING\""), sys);
    }
}

// 42. Stale at rest: no latch (nothing live). Every motion command refused,
//     no direction pin raised -- through the commands, the motor API and the
//     guard API -- and the refused non-zero request IS recorded as a loss.
//     Then the race: the guard stops while the first enable write of a DRIVE
//     from rest is slow (300 ms) -- the direction write after it is refused.
static void test42_refusedWhileStale(void)
{
    boot(true);
    command("STOP");
    pass(20);
    gGuardTicks = false;
    passMs(STALL_GUARD_LIVENESS_MS + STALL_GUARD_PERIOD_MS);
    CHECK(isNotRunning(), "setup: guard not stale");
    CHECK(!commSafetyStopLatched(), "a liveness loss at rest latched");
    expectMotionRefusedNoGuard("guard stale");
    CHECK(commSafetyStopLatched(), "a non-zero request refused for a stale guard was not recorded");
    int highs = gDirHighWrites;
    driveDifferential(150, 150);
    motorDriveChannel(0, 150);
    CHECK(!stallGuardSetDirection(kMotorPins[0], HIGH, kMotorPins[1], LOW), "guard API accepted a HIGH write");
    CHECK(gDirHighWrites == highs && directionPinsAllLow(), "a direction pin was raised with the guard stale");

    boot(true);
    pass(20);
    highs = gDirHighWrites;
    gSlowOkAt = 1; gSlowOkTxns = 0; gSlowOkMs = 300;
    gGuardTicks = false;                         // the guard stops as the DRIVE arrives
    int seq = send("DRIVE", ",\"left\":150,\"right\":150");
    pass(1);
    CHECK(has(ackFor(seq), "\"result\":\"ACCEPTED\""), "race setup (guard live on arrival): " + ackFor(seq));
    CHECK(gSlowOkAt == 0, "harness: the slow write did not happen");
    CHECK(gDirHighWrites == highs && directionPinsAllLow() && energisedWheels() == 0,
          "a direction pin was raised after the guard went stale mid-write");
    pass(3);
    CHECK(commSafetyStopLatched(), "the refused write was not recorded as a loss");
}

// 43. RESET while the guard is still stale: accepted, but no acknowledgement
//     and no drive -- however long after the loss it comes.
static void test43_resetWhileStale(void)
{
    long base = livenessLossWhileDriving();
    passMs(ACK_SETTLE_MS + TELEMETRY_INTERVAL_MS + 50);
    CHECK(!resetAcks("RESET with the guard stale"), "RESET acknowledged while the guard is stale");
    CHECK(isNotRunning(), "RESET restored the drive with the guard stale");
    std::string a = command("DRIVE", ",\"left\":150,\"right\":150");
    CHECK(has(a, "\"result\":\"GATED\"") && has(a, "\"reason\":\"MOTOR_PWM_UNAVAILABLE\""), a);
    std::string tel = freshTelemetry();
    CHECK(has(tel, "\"safety_stop\":true") && has(tel, "\"motor_drive_available\":false"), tel);
    CHECK(gEnergisedScans == base, "a wheel moved with the guard stale");
}

// 44. The guard ticks again: the drive is available but stays latched; an
//     immediate RESET is not an acknowledgement; only the established
//     settle-window RESET clears it; then a fresh DRIVE drives.
static void test44_resumeStaysLatched(void)
{
    long base = livenessLossWhileDriving();
    gGuardTicks = true;
    pass(STALL_GUARD_PERIOD_MS + 1);
    CHECK(motorDriveAvailable(), "drive not back once the guard ticks again");
    CHECK(commSafetyStopLatched(), "the guard resuming cleared the latch");
    CHECK(!resetAcks("RESET right after the guard resumed"), "acknowledged before the settle period");
    expectAckStillRequired("guard liveness restored", base);
    acknowledgeThenDrive("guard liveness restored", base);
}

// 45. No replay: requests made while stale, and refused after it resumed,
//     are not stored; the acknowledging RESET moves nothing; only a fresh
//     DRIVE moves the wheels.
static void test45_noReplayAfterAck(void)
{
    long base = livenessLossWhileDriving();
    command("DRIVE", ",\"left\":150,\"right\":150");
    command("MOVE", ",\"dir\":\"F\",\"speed\":150");
    gGuardTicks = true;
    pass(20);
    std::string a = command("DRIVE", ",\"left\":150,\"right\":150");
    CHECK(has(a, "DRIVE_LOSS_UNACKNOWLEDGED"), "DRIVE not refused before acknowledgement: " + a);
    passMs(ACK_SETTLE_MS + TELEMETRY_INTERVAL_MS + 50);
    CHECK(resetAcks("eligible RESET after resume"), "eligible RESET did not acknowledge");
    pass(500);
    CHECK(gEnergisedScans == base && commDesiredLeft() == 0 && commDesiredRight() == 0,
          "stale motion resumed after the acknowledgement");
    driveAccepted(120, 120);
    command("STOP");
}

// 46. millis() wraparound: no false staleness while ticking across the wrap;
//     the window edge is exact across it; a dead guard stays dead when the
//     clock wraps round to (or just before) its last stamp ~49.7 days on.
static void test46_livenessWraparound(void)
{
    boot(true, 0, false, 0xFFFFFFFFUL - 3000UL);
    driveAccepted(150, 150);
    bool always = true;
    driveWhile(6000, [&always] { if (!motorDriveAvailable()) always = false; return !always; });
    CHECK(gNow < 0x80000000UL, "harness: clock did not wrap");
    CHECK(always && energisedWheels() == 4 && !commSafetyStopLatched(), "false staleness across the wrap");
    command("STOP");

    boot(true, 0, false, 0xFFFFFFFFUL - 1000UL);
    const uint32_t kT = 0xFFFFFFFFUL - 25UL;     // a tick: T + LIVENESS is past the wrap
    CHECK((kT % STALL_GUARD_PERIOD_MS) == 0 && (uint32_t)(kT + STALL_GUARD_LIVENESS_MS) < 1000UL,
          "harness: tick constant");
    while (gNow != kT) advance(1);
    gGuardTicks = false;
    advance(STALL_GUARD_LIVENESS_MS);
    CHECK(gNow < 1000UL && stallGuardOperational(), "false staleness at the edge, across the wrap");
    advance(1);
    CHECK(isNotRunning(), "false liveness past the edge, across the wrap");
    gNow = kT + 5;                               // 2^32 + 5 ms after the last tick
    CHECK(isNotRunning(), "a dead guard came back when millis() wrapped round to its stamp");
    gNow = kT - 10;                              // 2^32 - 10 ms after it
    CHECK(isNotRunning(), "a dead guard came back just before millis() reached its stamp");
    gNow = 5000;
    gGuardTicks = true;
    advance(STALL_GUARD_PERIOD_MS);
    CHECK(stallGuardOperational(), "a real tick did not restore liveness");
}

typedef void (*TestFn)(void);

int main(void)
{
    static const struct { const char *name; TestFn fn; } kTests[] = {
        { "1  healthy boot: real register sequence",      test1_healthyBoot },
        { "2  boot NOT_FOUND -> recovered, no motion",    test2_bootAbsentThenRecovers },
        { "3  boot INIT_FAILED -> recovered",             test3_bootInitFailedThenRecovers },
        { "4  read-back rejects a misconfigured chip",    test4_verifyRejectsBadConfig },
        { "5  transient error while driving",             test5_transientWhileDriving },
        { "6  warm chip: outputs forced off first",       test6_warmChipForcedOffFirst },
        { "7  persistent error: backoff, then recovery",  test7_persistentBackoff },
        { "8  hung bus: stalls bounded, STOP mid-hang",   test8_hungBusBounded },
        { "9  MOTORTEST cancelled, never resumed",        test9_motortestCancelled },
        { "10 idle transient: no latch, watchdog kept",   test10_idleTransient },
        { "11 cutoff: every channel/step, fast + hung",   test11_cutoffDrive },
        { "12 cutoff: MOTORTEST path",                    test12_cutoffMotortest },
        { "13 latch evidence: DRIVE/STOP/PING mid-fail",  test13_latchEvidence },
        { "14 watchdog across a hung write",              test14_watchdogAcrossHungWrite },
        { "15 from rest: failed request latches, all txns", test15_fromRest },
        { "16 RESET inside the failure: not an ack",      test16_resetInsideFailure },
        { "17 RESET during recovery attempt: not an ack", test17_resetDuringRecovery },
        { "18 second outage voids the earlier report",    test18_secondOutageVoidsReport },
        { "19 RESET in HW FIFO at a report: not an ack",  test19_resetInFifoAtReport },
        { "20 RESET 20/100/150 ms after report: no ack",  test20_resetJustAfterFirstReport },
        { "21 RESET after the full settle period",        test21_resetAfterSettle },
        { "22 outage during settle restarts it",          test22_outageRestartsSettle },
        { "23 partial line across the snapshot",          test23_partialLineAcrossSnapshot },
        { "24 RESET + DRIVE batch, both cases",           test24_resetThenDriveBatch },
        { "25 no stale state across outages",             test25_noStaleStateAcrossOutages },
        { "26 outage voids an OPEN ack window",           test26_openWindowVoidedByOutage },
        { "27 guard: PCA/TCA/VL53/GPS hang cut in time",  test27_guardCutsEveryBlockingPath },
        { "28 guard: no false trip (busy-wait, GPS)",     test28_noFalseTrip },
        { "29 guard: resume race, every write position",  test29_resumeRace },
        { "30 guard: trip -> ack contract",               test30_tripAckContract },
        { "31 guard: trip + PCA recovery",                test31_tripPlusRecovery },
        { "32 guard: not armed during setup()",           test32_notArmedInSetup },
        { "33 guard: LOW re-asserted while tripped",      test33_reassertWhileTripped },
        { "34 guard: watchdog + telemetry after trip",    test34_watchdogTelemetryAfterTrip },
        { "35 guard: idle stall is not a trip",           test35_idleStallNoTrip },
        { "36 guard: millis() wraparound",                test36_wraparound },
        { "37 guard: STOP during the stall still latches", test37_stopDuringStallStillLatches },
        { "38 liveness: task not created",              test38_guardNotCreated },
        { "39 liveness: created, no tick yet",          test39_createdNoTickYet },
        { "40 liveness: regular ticks, exact window",   test40_regularTicksAndWindow },
        { "41 liveness: lost during DRIVE / MOTORTEST", test41_lossDuringMotion },
        { "42 liveness: stale -> no HIGH write, race",  test42_refusedWhileStale },
        { "43 liveness: RESET while stale",             test43_resetWhileStale },
        { "44 liveness: resumed -> latched until ack",  test44_resumeStaysLatched },
        { "45 liveness: no replay after the ack",       test45_noReplayAfterAck },
        { "46 liveness: millis() wraparound",           test46_livenessWraparound },
    };

    printf("=== PCA9685 recovery (real pca9685.cpp) ===\n");
    int failedTests = 0;
    for (size_t t = 0; t < sizeof(kTests) / sizeof(kTests[0]); t++) {
        gTest = kTests[t].name;
        int before = gFails;
        kTests[t].fn();
        bool ok = (gFails == before);
        if (!ok) failedTests++;
        printf("  %s  %s\n", ok ? "PASS" : "FAIL", kTests[t].name);
    }
    printf("=== PCA9685 recovery: %d tests, %d failed, %d checks, %d check failures ===\n",
           (int)(sizeof(kTests) / sizeof(kTests[0])), failedTests, gChecks, gFails);
    return (gFails == 0) ? 0 : 1;
}

// The real sketch: setup() and loop(), compiled unchanged.
#include "../../robot_controller.ino"

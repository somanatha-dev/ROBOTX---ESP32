// ============================================================================
//  FOCUSED HOST TEST -- VL53L0X library: invalid VCSEL period must not crash
//
//  The REAL installed Pololu VL53L0X library (VL53L0X.cpp, compiled unchanged
//  from the sketchbook's libraries/ folder) runs against a fake sensor on a
//  fake I2C bus. Nothing else of the firmware is involved.
//
//  The defect this guards (crash captured on the rover, 2026-10-02):
//  setMeasurementTimingBudget() divides by a macro period derived from the
//  FINAL_RANGE VCSEL period register. A failed I2C read returns 0xFF, and
//  0xFF (or 0x7F) decodes to a period of 0 -> IntegerDivideByZero. It is
//  reachable from our own call AND from inside init().
//
//  The rover's patched library returns false instead (marker
//  VL53L0X_ROVER_VCSEL_ZERO_GUARD). Against an unpatched library this test
//  dies with a divide-by-zero, which the harness reports as a failure.
//
//  What this CANNOT show: real ESP32 I2C timing, or how a real faulty sensor
//  behaves. The fake answers only what the library's init path polls for.
// ============================================================================

#include <stdio.h>
#include <string.h>

#include "Arduino.h"
#include "Wire.h"
#include <VL53L0X.h>

// ---------------------------------------------------------------------------
// Arduino core: a clock that advances on every read, so no poll loop can spin
// ---------------------------------------------------------------------------

static uint32_t gNow = 0;

uint32_t millis(void)                   { return gNow++; }
uint32_t micros(void)                   { return gNow * 1000UL; }
void     delay(uint32_t ms)             { gNow += ms; }
void     delayMicroseconds(uint32_t us) { (void)us; }

// ---------------------------------------------------------------------------
// Fake VL53L0X at 0x29: a flat register file plus the few registers the
// library polls. VCSEL period reads are scriptable.
// ---------------------------------------------------------------------------

enum VcselMode {
    V_VALID,        // FINAL_RANGE VCSEL register reads 4 (period 10 PCLKs)
    V_READ_FAILS,   // every read touching that register fails (no data)
    V_READS_0x7F    // the read succeeds but returns 0x7F (also decodes to 0)
};

static const uint8_t kAddr = 0x29;

static uint8_t   gRegs[256];
static VcselMode gVcsel = V_VALID;
static int       gFinalVcselReads = 0;
static int       gFinalTimeoutWrites = 0;

static uint8_t gTx[16];
static int     gTxLen  = 0;
static int     gTxAddr = -1;
static uint8_t gPtr    = 0;

static uint8_t gRx[16];
static int     gRxLen = 0;
static int     gRxPos = 0;

TwoWire Wire;

bool TwoWire::begin(int sda, int scl, uint32_t freq)
{
    (void)sda; (void)scl; (void)freq;
    return true;
}

void TwoWire::beginTransmission(int address)
{
    gTxAddr = address;
    gTxLen  = 0;
}

size_t TwoWire::write(uint8_t b)
{
    if (gTxLen < (int)sizeof(gTx)) {
        gTx[gTxLen++] = b;
    }
    return 1;
}

uint8_t TwoWire::endTransmission(bool sendStop)
{
    (void)sendStop;
    if (gTxAddr != kAddr) {
        return 2;                                   // address NACK
    }
    if (gTxLen >= 1) {
        gPtr = gTx[0];
    }
    for (int i = 1; i < gTxLen; i++) {
        gRegs[(uint8_t)(gPtr + i - 1)] = gTx[i];
    }
    // The timing budget is written with writeReg16Bit() (pointer + 2 bytes).
    // init()'s tuning table also writes 0x71 as a single raw byte -- that is
    // not a timing-budget write and is not counted.
    if (gTxLen == 3 && gPtr == VL53L0X::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI) {
        gFinalTimeoutWrites++;
    }
    return 0;
}

static uint8_t regValue(uint8_t reg)
{
    switch (reg) {
        case VL53L0X::IDENTIFICATION_MODEL_ID:          return 0xEE;
        case VL53L0X::RESULT_INTERRUPT_STATUS:          return 0x07;  // "ready"
        case 0x83:                                      return 0x01;  // SPAD info ready
        case VL53L0X::PRE_RANGE_CONFIG_VCSEL_PERIOD:    return 6;     // 14 PCLKs
        case VL53L0X::FINAL_RANGE_CONFIG_VCSEL_PERIOD:
            return (gVcsel == V_READS_0x7F) ? 0x7F : 4;               // 10 PCLKs
        default:                                        return gRegs[reg];
    }
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
    gRxLen = 0;
    gRxPos = 0;
    if (address != kAddr || quantity < 1 || quantity > (int)sizeof(gRx)) {
        return 0;
    }
    for (int i = 0; i < quantity; i++) {
        uint8_t reg = (uint8_t)(gPtr + i);
        if (reg == VL53L0X::FINAL_RANGE_CONFIG_VCSEL_PERIOD) {
            gFinalVcselReads++;
            if (gVcsel == V_READ_FAILS) {
                return 0;                           // nothing received
            }
        }
        gRx[i] = regValue(reg);
    }
    gRxLen = quantity;
    return (uint8_t)quantity;
}

int TwoWire::available(void) { return gRxLen - gRxPos; }

int TwoWire::read(void)
{
    if (gRxPos >= gRxLen) {
        return -1;                                  // as the ESP32 core does
    }
    return gRx[gRxPos++];
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

static void resetFake(VcselMode mode)
{
    memset(gRegs, 0, sizeof(gRegs));
    gRegs[VL53L0X::SYSTEM_SEQUENCE_CONFIG] = 0xE8;  // final range enabled
    gVcsel              = mode;
    gFinalVcselReads    = 0;
    gFinalTimeoutWrites = 0;
    gPtr                = 0;
    gRxLen = gRxPos = 0;
}

// ---------------------------------------------------------------------------
// 1-2. setMeasurementTimingBudget() with an invalid FINAL_RANGE VCSEL period
// ---------------------------------------------------------------------------
static void testBudgetReadFails(void)
{
    resetFake(V_READ_FAILS);
    VL53L0X dev;
    dev.setBus(&Wire);
    bool ok = dev.setMeasurementTimingBudget(33000);
    CHECK(gFinalVcselReads > 0, "test did not reach the VCSEL period read");
    CHECK(!ok, "accepted a timing budget computed from a failed read");
    CHECK(gFinalTimeoutWrites == 0, "wrote a final-range timeout from garbage");
}

static void testBudgetReads0x7F(void)
{
    resetFake(V_READS_0x7F);
    VL53L0X dev;
    dev.setBus(&Wire);
    bool ok = dev.setMeasurementTimingBudget(33000);
    CHECK(gFinalVcselReads > 0, "test did not reach the VCSEL period read");
    CHECK(!ok, "accepted a timing budget computed from VCSEL 0x7F");
    CHECK(gFinalTimeoutWrites == 0, "wrote a final-range timeout from garbage");
}

// ---------------------------------------------------------------------------
// 3. Positive control: a valid period is unaffected by the guard
// ---------------------------------------------------------------------------
static void testBudgetValid(void)
{
    resetFake(V_VALID);
    VL53L0X dev;
    dev.setBus(&Wire);
    bool ok = dev.setMeasurementTimingBudget(33000);
    CHECK(ok, "valid VCSEL period rejected");
    CHECK(gFinalTimeoutWrites == 1, "final-range timeout not written");
}

// ---------------------------------------------------------------------------
// 4-5. init() calls setMeasurementTimingBudget() internally -- the same
// division is reachable before control ever returns to rear_tof.cpp.
// ---------------------------------------------------------------------------
static void testInitReadFails(void)
{
    resetFake(V_READ_FAILS);
    VL53L0X dev;
    dev.setBus(&Wire);
    dev.setTimeout(100);
    (void)dev.init(true);                       // must RETURN, whatever it says
    CHECK(gFinalVcselReads > 0, "init() did not reach the VCSEL period read");
    CHECK(gFinalTimeoutWrites == 0, "init() wrote a final-range timeout from garbage");
}

static void testInitValid(void)
{
    resetFake(V_VALID);
    VL53L0X dev;
    dev.setBus(&Wire);
    dev.setTimeout(100);
    bool ok = dev.init(true);
    CHECK(ok, "fake sensor does not model a working init (test invalid)");
    CHECK(gFinalVcselReads > 0, "init() never read the VCSEL period");
    CHECK(gFinalTimeoutWrites >= 1, "init() did not set the timing budget");
}

typedef void (*TestFn)(void);

int main(void)
{
    static const struct { const char *name; TestFn fn; } kTests[] = {
        { "1  budget: VCSEL read fails -> false, no crash", testBudgetReadFails },
        { "2  budget: VCSEL reads 0x7F -> false, no crash", testBudgetReads0x7F },
        { "3  budget: valid VCSEL unchanged",               testBudgetValid     },
        { "4  init(): VCSEL read fails -> returns",         testInitReadFails   },
        { "5  init(): valid sensor initialises",            testInitValid       },
    };

    int failedTests = 0;
    for (size_t t = 0; t < sizeof(kTests) / sizeof(kTests[0]); t++) {
        gTest = kTests[t].name;
        int before = gFails;
        kTests[t].fn();
        bool ok = (gFails == before);
        if (!ok) failedTests++;
        printf("  %s  %s\n", ok ? "PASS" : "FAIL", kTests[t].name);
        fflush(stdout);
    }

    printf("=== VL53L0X VCSEL guard: %d tests, %d failed, %d checks, %d check failures ===\n",
           (int)(sizeof(kTests) / sizeof(kTests[0])), failedTests, gChecks, gFails);
    return (gFails == 0) ? 0 : 1;
}

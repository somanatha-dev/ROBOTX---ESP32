#include <Arduino.h>
#include <Wire.h>

#include "pca9685.h"
#include "rover_i2c.h"
#include "config.h"

// ============================================================================
//  PCA9685 DRIVER
// ============================================================================
//
// Minimal on purpose. This rover needs four DC duty cycles on ENA/ENB inputs;
// it does not need servo pulse-width helpers, phase-shifted channel offsets or
// an external-clock path. Every one of those is a place for a bug to live in a
// part that controls whether the motors are enabled.
//
// NO EXTERNAL LIBRARY. The register interface is four registers wide and the
// dependency would not earn its keep.
// ============================================================================

static bool gReady    = false;
static bool gBusError = false;
static bool gFound    = false;

// Runtime recovery bookkeeping (see "RUNTIME RECOVERY" below).
typedef enum {
    PCA_FAULT_NONE = 0,
    PCA_FAULT_BOOT_INIT,        // the boot bring-up did not complete
    PCA_FAULT_RUNTIME_BUS       // a write failed after the chip was ready
} PcaFault;

static PcaFault gFault           = PCA_FAULT_NONE;
static bool     gRetrying        = false;
static uint32_t gNextAttemptMs   = 0;
static uint32_t gRetryIntervalMs = 0;
static uint16_t gAttempts        = 0;
static uint16_t gRecoveries      = 0;
static uint16_t gRuntimeBusErrs  = 0;
static uint8_t  gExpectPrescale  = 0;


bool pca9685AddressConfirmed(void)
{
    return (PCA9685_ADDRESS_CONFIRMED != 0);
}

uint8_t pca9685Address(void)
{
    return (uint8_t)PCA9685_I2C_ADDRESS;
}

bool pca9685Ready(void)
{
    return gReady && !gBusError;
}

const char *pca9685StatusName(void)
{
    if (!pca9685AddressConfirmed()) return "ADDRESS_UNCONFIRMED";
    if (!gFound)                    return "NOT_FOUND";
    if (!gReady)                    return "INIT_FAILED";
    if (gBusError)                  return "BUS_ERROR";
    return "OK";
}


// ============================================================================
// LOW LEVEL
// ============================================================================

static bool wr(uint8_t reg, uint8_t value)
{
    if (!roverI2cWriteReg8(pca9685Address(), reg, value)) {
        gBusError = true;
        return false;
    }
    return true;
}

static bool rd(uint8_t reg, uint8_t *value)
{
    if (!roverI2cReadReg8(pca9685Address(), reg, value)) {
        gBusError = true;
        return false;
    }
    return true;
}

// A write to a chip that WAS ready failed. Counted once per loss of the drive:
// pca9685AllOff() keeps trying while gReady is still true, and a second
// failure there is the same fault, not a new one.
static void noteRuntimeBusError(void)
{
    if (gReady && !gBusError) {
        gFault = PCA_FAULT_RUNTIME_BUS;
        if (gRuntimeBusErrs < 0xFFFF) gRuntimeBusErrs++;
    }
    gBusError = true;
}

// ALL_LED full-off in one transaction. No readiness check: bring-up uses it
// before gReady is set.
static bool allOffTxn(void)
{
    Wire.beginTransmission(pca9685Address());
    Wire.write(PCA9685_REG_ALL_LED_ON_L);
    Wire.write(0x00);       // ALL_ON_L
    Wire.write(0x00);       // ALL_ON_H
    Wire.write(0x00);       // ALL_OFF_L
    Wire.write(0x10);       // ALL_OFF_H, bit 4 = full off

    return (Wire.endTransmission() == 0);
}


// ============================================================================
// PWM FREQUENCY
// ============================================================================
//
// prescale = round( osc / (4096 * freq) ) - 1
//
// The part REQUIRES sleep mode to change the prescaler -- the register is
// write-protected while the oscillator runs. So the sequence is: sleep, write
// prescale, wake, wait for the oscillator to stabilise, then RESTART to
// resume the PWM outputs that sleeping suspended.
//
// The datasheet gives 500 us as the oscillator stabilisation time. This runs
// once, at boot, before any motor can be enabled, so the delay is free.
// ============================================================================

static bool setPwmFrequency(uint32_t freqHz)
{
    // The part's hardware limits. Outside these the prescaler saturates and
    // the actual frequency silently stops matching the request, which on a
    // motor enable line means a speed nobody commanded.
    if (freqHz < 24UL)   freqHz = 24UL;
    if (freqHz > 1526UL) freqHz = 1526UL;

    uint32_t prescaleVal = (PCA9685_OSC_HZ / (4096UL * freqHz));
    if (prescaleVal == 0) {
        return false;
    }
    prescaleVal -= 1;

    if (prescaleVal < 3)   prescaleVal = 3;       // datasheet minimum
    if (prescaleVal > 255) prescaleVal = 255;

    gExpectPrescale = (uint8_t)prescaleVal;       // checked by verifyConfig()

    uint8_t mode1 = 0;
    if (!rd(PCA9685_REG_MODE1, &mode1)) {
        return false;
    }

    // Sleep: clear RESTART, set SLEEP.
    uint8_t sleepMode = (uint8_t)((mode1 & (uint8_t)~PCA9685_MODE1_RESTART)
                                  | PCA9685_MODE1_SLEEP);
    if (!wr(PCA9685_REG_MODE1, sleepMode))                  return false;
    if (!wr(PCA9685_REG_PRESCALE, (uint8_t)prescaleVal))    return false;

    // Wake, with auto-increment on so a 4-byte channel write is one burst.
    uint8_t wakeMode = (uint8_t)((sleepMode & (uint8_t)~PCA9685_MODE1_SLEEP)
                                 | PCA9685_MODE1_AI);
    if (!wr(PCA9685_REG_MODE1, wakeMode))                   return false;

    delayMicroseconds(500);                     // oscillator stabilisation

    // RESTART resumes the PWM channels that sleeping switched off.
    if (!wr(PCA9685_REG_MODE1,
            (uint8_t)(wakeMode | PCA9685_MODE1_RESTART)))   return false;

    return true;
}


// Read back what bring-up configured. A chip is only declared ready once it
// demonstrably holds the configuration: oscillator running (SLEEP clear) and
// the prescaler we wrote. Two reads, no writes.
static bool verifyConfig(void)
{
    uint8_t mode1    = 0;
    uint8_t prescale = 0;

    if (!rd(PCA9685_REG_MODE1, &mode1))       return false;
    if (!rd(PCA9685_REG_PRESCALE, &prescale)) return false;

    if ((mode1 & PCA9685_MODE1_SLEEP) != 0)   return false;
    if (prescale != gExpectPrescale)          return false;

    return true;
}


// ============================================================================
// BRING-UP  --  ORDERING IS SAFETY CRITICAL
// ============================================================================
//
// The one sequence used at boot AND by runtime recovery.
//
// Outputs are forced FULLY OFF before the frequency is set, and again after.
// The first one matters because the chip may already have been running from a
// previous boot with the ESP32 reset underneath it: a warm-reset ESP32 sees a
// PCA9685 that never lost power and may still be holding a motor enable high.
// The same holds for a recovery: after a failed write the enables are in an
// unknown state. Clearing the outputs is the very first thing we do to it.
//
// gReady is set only after EVERY step succeeded, including the read-back, so
// there is no window in which a half-configured chip counts as a speed path.
// ============================================================================

static bool bringUp(void)
{
    gReady    = false;
    gBusError = false;
    gFound    = false;

    if (!pca9685AddressConfirmed()) {
        // Deliberate no-op -- see the header. We do not probe a placeholder.
        return false;
    }

    if (!roverI2cProbe(pca9685Address())) {
        return false;
    }
    gFound = true;

    // ---- 1. Kill every output immediately, before anything else. ----
    // ALL_LED full-off bit, one register at a time.
    if (!wr(PCA9685_REG_ALL_LED_ON_L + 0, 0x00)) return false;   // ALL_ON_L
    if (!wr(PCA9685_REG_ALL_LED_ON_L + 1, 0x00)) return false;   // ALL_ON_H
    if (!wr(PCA9685_REG_ALL_LED_ON_L + 2, 0x00)) return false;   // ALL_OFF_L
    if (!wr(PCA9685_REG_ALL_LED_ON_L + 3, 0x10)) return false;   // ALL_OFF_H
                                                                 // bit4 = full off

    // ---- 2. Totem-pole outputs. ----
    // The L298N enable input is a logic input expecting a driven high, not an
    // open-drain line waiting for a pull-up that this rover may not have.
    if (!wr(PCA9685_REG_MODE2, PCA9685_MODE2_OUTDRV)) return false;

    // ---- 3. Frequency. ----
    if (!setPwmFrequency((uint32_t)PCA9685_PWM_FREQ_HZ)) return false;

    // ---- 4. The chip holds what we configured. ----
    if (!verifyConfig()) return false;

    // ---- 5. Explicit final all-off, as one burst. ----
    if (!allOffTxn()) {
        gBusError = true;
        return false;
    }

    gReady = true;
    return true;
}

bool pca9685Init(void)
{
    gFault          = PCA_FAULT_NONE;
    gRetrying       = false;
    gAttempts       = 0;
    gRecoveries     = 0;
    gRuntimeBusErrs = 0;

    bool ok = bringUp();
    if (!ok && pca9685AddressConfirmed()) {
        gFault = PCA_FAULT_BOOT_INIT;
    }
    return ok;
}


// ============================================================================
// RUNTIME RECOVERY
// ============================================================================
//
// Without this, ONE failed bring-up at boot (peripherals powering up a few
// seconds after the ESP32) or ONE failed write while running left the drive
// unavailable until the next reset -- the latches above were set in exactly
// one place and cleared nowhere else.
//
// While the drive is unavailable, the full bring-up() above is retried: first
// PCA9685_RECOVERY_INITIAL_MS after the loss, then with the interval doubling
// up to PCA9685_RECOVERY_MAX_MS. A success resets the schedule.
//
// WHAT RECOVERY DOES NOT DO, by construction:
//   * It cannot energise a motor. bring-up's first act is ALL outputs full
//     off, its last act is another full-off, and gReady only becomes true
//     after both. The direction inputs are not touched here at all; motor.cpp
//     holds them LOW for the whole time the drive is unavailable.
//   * It does not resume anything. The caller (loop()) discards every stored
//     motion request and cancels MOTORTEST while the drive is unavailable,
//     BEFORE calling this -- so a recovered drive starts at 0/0 and moves only
//     on a brand-new command.
//   * It touches no safety latch and no watchdog state.
//
// COST: one attempt is ~12 I2C transactions (a few ms) on a healthy bus. On a
// HUNG bus, the first transaction blocks for >= 1 s on this core (a floor in
// the core's I2C driver; config.h I2C_TIMEOUT_MS does not shorten it) and the
// attempt stops there; the backoff is what bounds how
// often that can happen. Every attempt happens with all wheels stopped.
// ============================================================================

bool pca9685ServiceRecovery(void)
{
    if (!pca9685AddressConfirmed() || pca9685Ready()) {
        gRetrying = false;
        return false;
    }

    uint32_t now = millis();

    if (!gRetrying) {
        // First pass after the loss: schedule, do not retry on the spot.
        gRetrying        = true;
        gRetryIntervalMs = PCA9685_RECOVERY_INITIAL_MS;
        gNextAttemptMs   = now + gRetryIntervalMs;
        return false;
    }

    // Signed comparison handles millis() rollover correctly.
    if ((int32_t)(now - gNextAttemptMs) < 0) {
        return false;
    }

    if (gAttempts < 0xFFFF) gAttempts++;

    if (bringUp()) {
        gRetrying = false;
        if (gRecoveries < 0xFFFF) gRecoveries++;
        return true;
    }

    gRetryIntervalMs = (gRetryIntervalMs >= PCA9685_RECOVERY_MAX_MS / 2)
                     ? PCA9685_RECOVERY_MAX_MS
                     : gRetryIntervalMs * 2;
    gNextAttemptMs   = millis() + gRetryIntervalMs;
    return false;
}

const char *pca9685RecoveryName(void)
{
    if (!pca9685AddressConfirmed()) return "NONE";
    if (pca9685Ready())             return (gRecoveries > 0) ? "RECOVERED" : "NONE";
    return "RETRYING";
}

const char *pca9685LastFaultName(void)
{
    switch (gFault) {
        case PCA_FAULT_BOOT_INIT:   return "BOOT_INIT";
        case PCA_FAULT_RUNTIME_BUS: return "RUNTIME_BUS";
        default:                    return "NONE";
    }
}

uint16_t pca9685RecoveryAttempts(void)  { return gAttempts;       }
uint16_t pca9685Recoveries(void)        { return gRecoveries;     }
uint16_t pca9685RuntimeBusErrors(void)  { return gRuntimeBusErrs; }

uint32_t pca9685NextRetryInMs(void)
{
    if (!gRetrying || pca9685Ready()) return 0;
    int32_t left = (int32_t)(gNextAttemptMs - millis());
    return (left > 0) ? (uint32_t)left : 0;
}


// ============================================================================
// OUTPUT
// ============================================================================

bool pca9685SetDuty(uint8_t channel, uint16_t duty)
{
    if (!pca9685Ready() || channel > 15) {
        return false;
    }

    if (duty > PCA9685_PWM_MAX) {
        duty = PCA9685_PWM_MAX;
    }

    const uint8_t base = (uint8_t)(PCA9685_REG_LED0_ON_L + 4 * channel);

    uint16_t on  = 0;
    uint16_t off = 0;

    if (duty == 0) {
        // FULL OFF (bit 12 of the OFF word). Not a zero-width pulse: the part
        // still emits a sliver for a 0-width pulse, and a sliver on an L298N
        // enable is a motor twitching instead of a motor stopped.
        off = 0x1000;
    } else if (duty >= PCA9685_PWM_MAX) {
        // FULL ON (bit 12 of the ON word).
        on = 0x1000;
    } else {
        on  = 0;
        off = duty;
    }

    // Auto-increment is enabled in MODE1, so this is one burst of four bytes
    // rather than four separate transactions. That matters: a torn write
    // between the ON and OFF words would produce a duty nobody commanded.
    Wire.beginTransmission(pca9685Address());
    Wire.write(base);
    Wire.write((uint8_t)(on  & 0xFF));
    Wire.write((uint8_t)(on  >> 8));
    Wire.write((uint8_t)(off & 0xFF));
    Wire.write((uint8_t)(off >> 8));

    if (Wire.endTransmission() != 0) {
        noteRuntimeBusError();
        return false;
    }

    return true;
}


bool pca9685AllOff(void)
{
    // Still attempted after a runtime BUS_ERROR (gReady stays true until the
    // first recovery attempt): this is the path a fault takes, and one more
    // try at killing the enables is worth it.
    if (!gReady) {
        return false;
    }

    // One transaction, all sixteen outputs dead. This is the path a fault
    // takes, so it is deliberately the shortest one available.
    if (!allOffTxn()) {
        noteRuntimeBusError();
        return false;
    }

    return true;
}


// ============================================================================
// COMMISSIONING PROBE
// ============================================================================
//
// READ ONLY. It writes nothing, so pointing it at the wrong address cannot
// disturb whatever actually lives there -- which is exactly what you want when
// you are working through a scan list one address at a time.
// ============================================================================

bool pca9685ProbeCandidate(uint8_t address, uint8_t *mode1Out, uint8_t *prescaleOut)
{
    if (mode1Out    != NULL) *mode1Out    = 0;
    if (prescaleOut != NULL) *prescaleOut = 0;

    if (!roverI2cProbe(address)) {
        return false;
    }

    uint8_t mode1 = 0;
    if (!roverI2cReadReg8(address, PCA9685_REG_MODE1, &mode1)) {
        return false;
    }

    uint8_t prescale = 0;
    if (!roverI2cReadReg8(address, PCA9685_REG_PRESCALE, &prescale)) {
        return false;
    }

    if (mode1Out    != NULL) *mode1Out    = mode1;
    if (prescaleOut != NULL) *prescaleOut = prescale;

    return true;
}

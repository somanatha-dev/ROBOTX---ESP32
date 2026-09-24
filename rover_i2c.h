#ifndef ROVER_I2C_H
#define ROVER_I2C_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ============================================================================
//  ROVER I2C BUS ABSTRACTION
// ============================================================================
//
// ONE place brings the bus up, and every I2C user on this rover goes through
// it. Before this module existed the firmware declared GPIO21/22 and then
// never called Wire.begin() at all, because no peripheral could be named.
// Four are named now: PCA9685, TCA9548A, three VL53L0X behind it, and the
// NEO-9M GPS.
//
// WHAT THIS MODULE DELIBERATELY DOES NOT DO
// -----------------------------------------
// It does not know any device's address except the two that are actually
// CONFIRMED (VL53L0X 0x29, NEO-9M 0x42). The TCA9548A and PCA9685 addresses
// are unconfirmed hardware facts and live behind their own _CONFIRMED flags in
// config.h. This module will happily probe an address you hand it -- that is
// what the scanner is for -- but it never picks one on your behalf.
// ============================================================================

// ---------------------------------------------------------------------------
// Bring up Wire on SDA=I2C_SDA_PIN, SCL=I2C_SCL_PIN at I2C_CLOCK_HZ.
// Safe to call once from setup(), before any other I2C user is initialised.
// Sets a per-transaction timeout so a slave holding SDA low cannot wedge the
// safety loop.
// ---------------------------------------------------------------------------
void roverI2cInit(void);

// True once roverI2cInit() has run successfully.
bool roverI2cReady(void);

// ---------------------------------------------------------------------------
// Probe one 7-bit address. Returns true if a device ACKed.
//
// This is an ADDRESS-ONLY transaction (start, address, stop) with no data
// byte, which is the standard non-destructive way to ask "is anything there?".
// Addresses outside I2C_SCAN_FIRST_ADDR..I2C_SCAN_LAST_ADDR are refused
// without being probed -- the reserved blocks below 0x08 and above 0x77 must
// never be poked.
// ---------------------------------------------------------------------------
bool roverI2cProbe(uint8_t address);

// ---------------------------------------------------------------------------
// Raw byte-level helpers. Every one returns success/failure; none of them
// invents a value on error. `*value` is left untouched when a read fails, so a
// caller that ignores the return code cannot silently consume a fabricated
// register.
// ---------------------------------------------------------------------------
bool roverI2cWriteByte(uint8_t address, uint8_t value);
bool roverI2cReadByte(uint8_t address, uint8_t *value);
bool roverI2cWriteReg8(uint8_t address, uint8_t reg, uint8_t value);
bool roverI2cReadReg8(uint8_t address, uint8_t reg, uint8_t *value);

// ---------------------------------------------------------------------------
// The Wire.endTransmission() code from the most recent transaction, and its
// name. 0 = success; 2 = address NACK (nothing there); 3 = data NACK;
// 4 = other; 5 = timeout.
// ---------------------------------------------------------------------------
uint8_t     roverI2cLastError(void);
const char *roverI2cErrorName(uint8_t err);

// ---------------------------------------------------------------------------
// PHYSICAL LINE LEVELS  --  read the pads, touch nothing.
//
// This is the measurement that separates "the bus is empty" from "the bus is
// stuck". It performs NO I2C transaction, changes NO pin mode, and writes to
// NO device. It only reads the GPIO input register, which on ESP32 reflects
// the pad level even while the pin is routed to the I2C peripheral.
//
// HOW TO READ THE RESULT. The esp32 core enables the ESP32's INTERNAL
// pull-ups on both lines inside Wire.begin() (i2cInit() sets sda_pullup_en and
// scl_pullup_en to GPIO_PULLUP_ENABLE). An idle I2C master releases both
// lines. So with nothing else on the bus, both lines MUST read HIGH.
//
// A line reading LOW therefore means something EXTERNAL is holding it down.
// That is a hardware fact, not a firmware state.
//
// Several samples are taken a few hundred microseconds apart so a line that is
// toggling can be told apart from one that is stuck.
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t sampleCount;      // how many samples were taken
    uint8_t sdaHighSamples;   // how many of them read HIGH
    uint8_t sclHighSamples;
    bool    sdaHigh;          // level on the final sample
    bool    sclHigh;
    bool    sdaStuckLow;      // LOW on EVERY sample
    bool    sclStuckLow;
} RoverI2cLineState;

void roverI2cSampleLines(RoverI2cLineState *out);

// A one-line reading of the levels above, for the diagnostic report.
// Never names a device and never blames a specific chip -- it describes the
// electrical state and what classes of cause produce it.
const char *roverI2cLineVerdict(const RoverI2cLineState *s);

// ---------------------------------------------------------------------------
// Scan the bus. Fills `found` with every address that ACKed, up to maxFound,
// and returns how many were stored.
//
// NOTE FOR COMMISSIONING: a scan tells you an address responded. It does NOT
// tell you which chip it is. 0x70..0x77 is shared territory between a TCA9548A
// and a fully-jumpered PCA9685, so use TCATEST/PCATEST to identify a candidate
// rather than assuming from the number.
// ---------------------------------------------------------------------------
uint8_t roverI2cScan(uint8_t *found, uint8_t maxFound);

// ---------------------------------------------------------------------------
// A HINT for what might live at an address, for human-readable scan output.
//
// Returns a confirmed identity ONLY for the two addresses this rover's
// hardware information actually confirms, plus whatever the operator has
// marked confirmed in config.h. Everything else comes back as a candidate
// string that reads like a guess because it IS a guess -- e.g. "?TCA9548A/
// PCA9685 range". Never treat this as identification.
// ---------------------------------------------------------------------------
const char *roverI2cAddressHint(uint8_t address);

#endif // ROVER_I2C_H

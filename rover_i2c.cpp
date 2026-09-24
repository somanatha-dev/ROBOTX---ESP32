#include <Arduino.h>
#include <Wire.h>

#include "rover_i2c.h"
#include "config.h"

// ============================================================================
//  I2C BUS -- SINGLE OWNER
// ============================================================================
//
// Wire.begin() is called EXACTLY ONCE, here. No other file calls it. If a
// second caller ever re-began the bus at a different clock or on different
// pins, every device on it would see a reset it did not expect, and the
// TCA9548A in particular would silently drop back to "no channel selected"
// mid-transaction.
// ============================================================================

static bool    gReady     = false;
static uint8_t gLastError = 0;


void roverI2cInit(void)
{
    // Wire.begin(sda, scl, frequency) is the esp32 core 2.x signature.
    // It returns false if the pins cannot be attached.
    gReady = Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, (uint32_t)I2C_CLOCK_HZ);

    // A slave that dies mid-byte holds SDA low forever. Without a timeout the
    // Wire driver waits for it indefinitely and takes the whole control loop
    // -- including the front ultrasonic safety gate -- down with it. This is
    // not a nicety; it is the difference between a failed sensor and a rover
    // that stops responding to STOP.
    Wire.setTimeOut((uint16_t)I2C_TIMEOUT_MS);

    gLastError = gReady ? 0 : 4;
}

bool roverI2cReady(void)
{
    return gReady;
}


// ============================================================================
// PROBE
// ============================================================================

bool roverI2cProbe(uint8_t address)
{
    if (!gReady) {
        gLastError = 4;
        return false;
    }

    // Reserved address blocks are never poked. 0x00-0x07 includes the general
    // call and START byte; 0x78-0x7F is reserved for 10-bit addressing.
    if (address < I2C_SCAN_FIRST_ADDR || address > I2C_SCAN_LAST_ADDR) {
        gLastError = 4;
        return false;
    }

    Wire.beginTransmission(address);
    gLastError = Wire.endTransmission();

    return (gLastError == 0);
}


// ============================================================================
// BYTE HELPERS
// ============================================================================
//
// Note what these do on failure: nothing. No output parameter is written, no
// default is substituted. A caller that reads a register and gets `false` back
// still holds whatever it had before, which is the only honest outcome -- the
// alternative is a zero that looks exactly like a real register value.
// ============================================================================

bool roverI2cWriteByte(uint8_t address, uint8_t value)
{
    if (!gReady) { gLastError = 4; return false; }

    Wire.beginTransmission(address);
    Wire.write(value);
    gLastError = Wire.endTransmission();

    return (gLastError == 0);
}

bool roverI2cReadByte(uint8_t address, uint8_t *value)
{
    if (!gReady || value == NULL) { gLastError = 4; return false; }

    uint8_t got = Wire.requestFrom((int)address, 1);
    if (got != 1 || Wire.available() < 1) {
        gLastError = 5;
        return false;
    }

    *value     = (uint8_t)Wire.read();
    gLastError = 0;
    return true;
}

bool roverI2cWriteReg8(uint8_t address, uint8_t reg, uint8_t value)
{
    if (!gReady) { gLastError = 4; return false; }

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    gLastError = Wire.endTransmission();

    return (gLastError == 0);
}

bool roverI2cReadReg8(uint8_t address, uint8_t reg, uint8_t *value)
{
    if (!gReady || value == NULL) { gLastError = 4; return false; }

    Wire.beginTransmission(address);
    Wire.write(reg);

    // Repeated START (endTransmission(false)) rather than a STOP, so no other
    // master can interleave between the register write and the read.
    gLastError = Wire.endTransmission(false);
    if (gLastError != 0) {
        return false;
    }

    uint8_t got = Wire.requestFrom((int)address, 1);
    if (got != 1 || Wire.available() < 1) {
        gLastError = 5;
        return false;
    }

    *value     = (uint8_t)Wire.read();
    gLastError = 0;
    return true;
}


// ============================================================================
// ERRORS
// ============================================================================

uint8_t roverI2cLastError(void)
{
    return gLastError;
}

const char *roverI2cErrorName(uint8_t err)
{
    switch (err) {
        case 0: return "OK";
        case 1: return "DATA_TOO_LONG";
        case 2: return "ADDR_NACK";      // nothing is at that address
        case 3: return "DATA_NACK";
        case 4: return "OTHER";
        case 5: return "TIMEOUT";
        default: return "UNKNOWN";
    }
}


// ============================================================================
// PHYSICAL LINE LEVELS
// ============================================================================
//
// WHY THIS IS SAFE TO RUN AT ANY TIME, WITH EVERYTHING POWERED:
//
//   * No I2C transaction is started. No START, no address, no data.
//   * No pinMode() call. The pads keep whatever configuration the I2C
//     peripheral gave them; we only read the GPIO input register, which on
//     ESP32 reflects the pad level regardless of which peripheral owns the
//     output path.
//   * Nothing is written to any device, at any address, ever.
//   * It cannot disturb a device mid-transfer, because it does not transfer.
//
// WHY THE READING IS MEANINGFUL: an idle I2C master releases both lines
// (they are open-drain), and the esp32 core enables the chip's internal
// pull-ups on both. So at idle, with nothing external attached, both lines
// read HIGH. Anything reading LOW is being pulled down from outside the ESP32.
// ============================================================================

#define ROVER_I2C_LINE_SAMPLES 16

void roverI2cSampleLines(RoverI2cLineState *out)
{
    if (out == NULL) {
        return;
    }

    out->sampleCount    = 0;
    out->sdaHighSamples = 0;
    out->sclHighSamples = 0;
    out->sdaHigh        = false;
    out->sclHigh        = false;

    for (uint8_t i = 0; i < ROVER_I2C_LINE_SAMPLES; i++) {
        bool sda = (digitalRead(I2C_SDA_PIN) == HIGH);
        bool scl = (digitalRead(I2C_SCL_PIN) == HIGH);

        if (sda) out->sdaHighSamples++;
        if (scl) out->sclHighSamples++;

        out->sdaHigh = sda;
        out->sclHigh = scl;
        out->sampleCount++;

        // Spread the samples over ~3 ms so a line that is being driven
        // periodically is distinguishable from one that is simply stuck.
        delayMicroseconds(200);
    }

    out->sdaStuckLow = (out->sdaHighSamples == 0);
    out->sclStuckLow = (out->sclHighSamples == 0);
}


const char *roverI2cLineVerdict(const RoverI2cLineState *s)
{
    if (s == NULL) {
        return "NO_DATA";
    }

    if (s->sdaStuckLow && s->sclStuckLow) {
        // Both rails down. A single fault that takes out both lines at once is
        // usually upstream of both: an unpowered peripheral clamping them
        // through its ESD diodes, or a common short.
        return "BOTH_LINES_STUCK_LOW - no I2C transaction can start. Typical "
               "causes: peripherals wired but UNPOWERED (their ESD diodes "
               "clamp both lines), or SDA and SCL shorted to GND";
    }

    if (s->sclStuckLow) {
        return "SCL_STUCK_LOW - the clock line is held down. No transaction "
               "can start. Typical causes: short to GND, an unpowered device "
               "on the line, or a slave stretching the clock forever";
    }

    if (s->sdaStuckLow) {
        return "SDA_STUCK_LOW - the data line is held down. Typical causes: "
               "short to GND, an unpowered device on the line, or a slave left "
               "mid-byte by a reset (classic hung-bus)";
    }

    if (s->sdaHighSamples < s->sampleCount || s->sclHighSamples < s->sampleCount) {
        return "LINES_TOGGLING - not stuck, but not idle either. Something is "
               "driving the bus, or the lines are floating and picking up "
               "noise";
    }

    return "BOTH_LINES_IDLE_HIGH - the bus is electrically healthy and idle. "
           "If a scan still finds nothing, the devices are not responding "
           "rather than the bus being stuck";
}


// ============================================================================
// SCAN
// ============================================================================

uint8_t roverI2cScan(uint8_t *found, uint8_t maxFound)
{
    if (!gReady || found == NULL || maxFound == 0) {
        return 0;
    }

    uint8_t n = 0;

    for (uint8_t a = I2C_SCAN_FIRST_ADDR; a <= I2C_SCAN_LAST_ADDR; a++) {
        if (roverI2cProbe(a)) {
            if (n < maxFound) {
                found[n] = a;
            }
            n++;
            if (n >= maxFound) {
                break;
            }
        }
    }

    gLastError = 0;
    return (n > maxFound) ? maxFound : n;
}


// ============================================================================
// ADDRESS HINTS  --  READ THE CAVEAT
// ============================================================================
//
// This function exists so a scan prints something a human can act on. It is
// NOT identification. Only two entries below are CONFIRMED facts about this
// rover's hardware (0x29 and 0x42); one more becomes confirmed as soon as the
// operator sets a _CONFIRMED flag in config.h. Every other return value is
// prefixed with '?' and names a RANGE the part could be in, which is a
// reminder that the number alone proves nothing.
//
// The awkward case is 0x70..0x77: that is the TCA9548A's entire address space
// AND a legal PCA9685 address (base 0x40 with all six jumpers bridged). A scan
// cannot tell them apart. TCATEST can, because a TCA9548A reads back the
// control register you just wrote to it and a PCA9685 does not.
// ============================================================================

const char *roverI2cAddressHint(uint8_t address)
{
    // ---- CONFIRMED by the operator's hardware information ----
    if (address == VL53L0X_I2C_ADDRESS) {
        // Only reachable with a TCA channel open. Seeing 0x29 on the MAIN bus
        // with all channels deselected would mean a VL53L0X is wired directly
        // to the bus, which contradicts the confirmed topology -- worth
        // knowing, hence the explicit wording.
        return "VL53L0X (0x29, confirmed) - expected only behind the TCA mux";
    }
    if (address == NEO9M_I2C_ADDRESS) {
        return "NEO-9M GPS (0x42, confirmed)";
    }

    // ---- Confirmed by the operator having set a flag in config.h ----
#if TCA9548A_ADDRESS_CONFIRMED
    if (address == TCA9548A_I2C_ADDRESS) {
        return "TCA9548A (confirmed in config.h)";
    }
#endif
#if PCA9685_ADDRESS_CONFIRMED
    if (address == PCA9685_I2C_ADDRESS) {
        return "PCA9685 (confirmed in config.h)";
    }
#endif

    // ---- Everything below is a CANDIDATE, not an identification ----
    if (address >= 0x70 && address <= 0x77) {
        return "?TCA9548A range - or a fully-jumpered PCA9685. Use TCATEST";
    }
    if (address >= 0x40 && address <= 0x7F) {
        return "?PCA9685 range (base 0x40 + A0..A5). Use PCATEST";
    }

    return "?unidentified";
}

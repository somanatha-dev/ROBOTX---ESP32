#include <Arduino.h>

#include "tca9548a.h"
#include "rover_i2c.h"
#include "config.h"

// ============================================================================
//  TCA9548A DRIVER
// ============================================================================
//
// The part has ONE register: an 8-bit control byte where bit N enables
// downstream channel N. You write it with a bare single-byte transaction (no
// register address, because there is only one register) and you read it back
// the same way.
//
// This driver keeps that surface as small as it can be, and refuses to widen
// it: there is no "open two channels" call, because on this rover two open
// channels means two VL53L0X at 0x29 shouting over each other.
// ============================================================================

static bool    gPresent        = false;
static int8_t  gCurrentChannel = -1;
static bool    gBusError       = false;


bool tcaAddressConfirmed(void)
{
    return (TCA9548A_ADDRESS_CONFIRMED != 0);
}

uint8_t tcaAddress(void)
{
    return (uint8_t)TCA9548A_I2C_ADDRESS;
}

bool tcaPresent(void)
{
    return gPresent;
}

const char *tcaStatusName(void)
{
    if (!tcaAddressConfirmed()) {
        // The honest answer. Not "not found" -- we never looked, because we
        // have no address we are entitled to look at.
        return "ADDRESS_UNCONFIRMED";
    }
    if (!gPresent) {
        return "NOT_FOUND";
    }
    if (gBusError) {
        return "BUS_ERROR";
    }
    return "OK";
}


// ============================================================================
// INIT
// ============================================================================

bool tcaInit(void)
{
    gPresent        = false;
    gCurrentChannel = -1;
    gBusError       = false;

    if (!tcaAddressConfirmed()) {
        // Deliberate no-op. Probing TCA9548A_I2C_ADDRESS here would be exactly
        // the silent guess this firmware refuses to make: the placeholder is
        // the part's power-on default, so it would very often ACK -- and a
        // coincidental ACK from some other chip would be read as confirmation.
        return false;
    }

    if (!roverI2cProbe(tcaAddress())) {
        return false;
    }

    gPresent = true;

    // Start from a known, quiet state: no downstream segment connected.
    return tcaDeselectAll();
}


// ============================================================================
// CHANNEL SELECTION
// ============================================================================

static bool writeMask(uint8_t mask)
{
    if (!tcaAddressConfirmed() || !gPresent) {
        return false;
    }

    if (!roverI2cWriteByte(tcaAddress(), mask)) {
        gBusError       = true;
        gCurrentChannel = -1;      // we no longer know what is connected
        return false;
    }

    gBusError = false;
    return true;
}


bool tcaSelectChannel(uint8_t channel)
{
    if (channel >= TCA9548A_CHANNEL_COUNT) {
        return false;
    }

    // EXCLUSIVE mask: exactly one bit set. This is the invariant that keeps
    // the three 0x29 sensors from colliding, and it is enforced by
    // construction rather than by a caller remembering to do the right thing.
    const uint8_t mask = (uint8_t)(1u << channel);

    if (!writeMask(mask)) {
        return false;
    }

    gCurrentChannel = (int8_t)channel;
    return true;
}


bool tcaDeselectAll(void)
{
    if (!writeMask(TCA9548A_DESELECT_MASK)) {
        return false;
    }

    gCurrentChannel = -1;
    return true;
}


int8_t tcaCurrentChannel(void)
{
    return gCurrentChannel;
}


bool tcaReadControl(uint8_t *maskOut)
{
    if (maskOut == NULL || !tcaAddressConfirmed() || !gPresent) {
        return false;
    }

    if (!roverI2cReadByte(tcaAddress(), maskOut)) {
        gBusError = true;
        return false;
    }

    gBusError = false;
    return true;
}


// ============================================================================
// COMMISSIONING PROBE
// ============================================================================
//
// This is the one function here that is allowed to touch an UNCONFIRMED
// address, because identifying that address is its entire job. It is still
// safe to run on a wrong guess: the worst case is writing one byte to some
// other chip's first register, and the only values it ever writes are a single
// channel bit and then 0x00.
//
// If you point it at something that is NOT a TCA9548A, the read-back will not
// match and it returns false. That negative result is just as useful as the
// positive one -- it rules an address out.
// ============================================================================

bool tcaProbeCandidate(uint8_t address, uint8_t testMask, uint8_t *readBackOut)
{
    if (readBackOut != NULL) {
        *readBackOut = 0;
    }

    if (!roverI2cProbe(address)) {
        return false;                       // nothing answers there at all
    }

    if (!roverI2cWriteByte(address, testMask)) {
        return false;
    }

    // The part needs no settling time, but a device that is NOT a mux may be
    // mid-operation. A short pause costs nothing during commissioning and
    // removes a class of false negative.
    delay(2);

    uint8_t back = 0;
    bool ok = roverI2cReadByte(address, &back);

    if (readBackOut != NULL) {
        *readBackOut = back;
    }

    // Always leave the candidate quiet, whatever it turned out to be. If it
    // IS the multiplexer, leaving a channel open would put a VL53L0X at 0x29
    // onto the main bus behind our back.
    (void)roverI2cWriteByte(address, TCA9548A_DESELECT_MASK);

    return (ok && back == testMask);
}

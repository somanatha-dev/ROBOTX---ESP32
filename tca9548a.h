#ifndef TCA9548A_H
#define TCA9548A_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
//  TCA9548A 8-CHANNEL I2C MULTIPLEXER
// ============================================================================
//
// WHY THIS PART IS HERE AT ALL
// ----------------------------
// The three rear VL53L0X all answer to address 0x29 and their addresses cannot
// be reassigned (that needs XSHUT lines, and this rover's XSHUT wiring is not
// confirmed). Three identical addresses cannot share a bus segment. The
// TCA9548A solves it by connecting exactly ONE downstream segment at a time.
//
// THE RULE THAT MATTERS: never have two channels open while addressing 0x29.
// Two VL53L0X would both ACK and both drive the bus, and the data you read
// back would be an electrical collision, not a measurement. Every function
// here writes an EXCLUSIVE channel mask (one bit, or zero bits) -- there is
// deliberately no API for opening several channels at once.
//
// THE ADDRESS IS NOT CONFIRMED
// ----------------------------
// See the I2C DEVICES section of config.h. While TCA9548A_ADDRESS_CONFIRMED is
// 0, every function here that would touch the bus REFUSES and returns false.
// The firmware does not talk to an address nobody verified.
// ============================================================================

// ---------------------------------------------------------------------------
// Call once from setup(), after roverI2cInit().
//
// When the address is confirmed this probes it and deselects all channels, so
// the bus starts in a known state. When it is not confirmed this does nothing
// at all except record why.
//
// Returns true only when the multiplexer was actually found and quiesced.
// ---------------------------------------------------------------------------
bool tcaInit(void);

// True when config.h says a human has confirmed the address.
bool tcaAddressConfirmed(void);

// The configured address. Meaningless unless tcaAddressConfirmed().
uint8_t tcaAddress(void);

// True when tcaInit() found the device and it has not failed since.
bool tcaPresent(void);

// Why the multiplexer is unusable, for telemetry:
//   "OK" / "ADDRESS_UNCONFIRMED" / "NOT_FOUND" / "BUS_ERROR"
const char *tcaStatusName(void);

// ---------------------------------------------------------------------------
// Select EXACTLY ONE channel, 0..TCA9548A_CHANNEL_COUNT-1.
// Every other channel is closed by the same write; there is no way for two to
// be open at once.
//
// Returns false if the address is unconfirmed, the channel is out of range, or
// the I2C write failed. A false return means YOU MUST NOT go on to address the
// downstream device -- you have no idea which segment is connected.
// ---------------------------------------------------------------------------
bool tcaSelectChannel(uint8_t channel);

// Close every channel. Leaves only the main bus segment live, which is the
// correct state before touching the PCA9685 or the GPS.
bool tcaDeselectAll(void);

// The channel currently believed to be selected, or -1 for none/unknown.
// This is the DRIVER'S BELIEF, not a read-back. Use tcaReadControl() if you
// need to know what the hardware actually holds.
int8_t tcaCurrentChannel(void);

// Read the control register back from the device.
bool tcaReadControl(uint8_t *maskOut);

// ---------------------------------------------------------------------------
// COMMISSIONING PROBE -- the whole reason the address question is answerable.
//
// Writes `testMask` to `address`, reads the control register back, and reports
// what came back. A TCA9548A returns exactly what you wrote. A PCA9685 sitting
// in the same 0x70..0x77 territory does not, because its single-byte read
// returns whatever its auto-incrementing register pointer is on, not a mirror
// of your write.
//
// Works on ANY address, including before TCA9548A_ADDRESS_CONFIRMED is set --
// that is its purpose. It restores the mask to 0x00 (all channels closed)
// before returning, whatever the outcome.
//
// Returns true only when the read-back matched the written mask.
// ---------------------------------------------------------------------------
bool tcaProbeCandidate(uint8_t address, uint8_t testMask, uint8_t *readBackOut);

#endif // TCA9548A_H

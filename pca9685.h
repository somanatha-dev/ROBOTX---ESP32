#ifndef PCA9685_H
#define PCA9685_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
//  PCA9685 16-CHANNEL PWM DRIVER  --  MOTOR ENABLE (ENA/ENB) SOURCE
// ============================================================================
//
// On this rover the PCA9685 is not driving servos. It drives the ENABLE inputs
// of the two L298N boards:
//
//     CH0 -> FRONT ENA      CH2 -> REAR ENA
//     CH1 -> FRONT ENB      CH3 -> REAR ENB
//
// That makes it a SAFETY-RELEVANT part, not a peripheral: its duty cycle is
// the rover's speed, and its outputs going to zero is the rover stopping.
//
// THE ADDRESS IS NOT CONFIRMED
// ----------------------------
// The PCA9685's address is its 0x40 base plus six solder jumpers A0..A5, so it
// can be anywhere in 0x40..0x7F. Nothing in this rover's confirmed hardware
// information states the jumper state.
//
// While PCA9685_ADDRESS_CONFIRMED is 0, this driver NEVER WRITES ANYTHING, and
// motor.cpp therefore refuses all motion. That is the correct failure: the
// enable inputs are fed by a chip we cannot identify, so commanding a
// direction would be commanding a motor through unverified hardware.
//
// Find the address with {"cmd":"I2CSCAN"} then {"cmd":"PCATEST","addr":0x??}.
// ============================================================================

// Register map, exposed so the diagnostics can report what they read.
#define PCA9685_REG_MODE1       0x00
#define PCA9685_REG_MODE2       0x01
#define PCA9685_REG_LED0_ON_L   0x06
#define PCA9685_REG_ALL_LED_ON_L 0xFA
#define PCA9685_REG_PRESCALE    0xFE

#define PCA9685_MODE1_RESTART   0x80
#define PCA9685_MODE1_AI        0x20    // register auto-increment
#define PCA9685_MODE1_SLEEP     0x10
#define PCA9685_MODE1_ALLCALL   0x01

#define PCA9685_MODE2_OUTDRV    0x04    // totem-pole outputs

// The part's internal oscillator, used to compute the prescaler.
#define PCA9685_OSC_HZ          25000000UL

// ---------------------------------------------------------------------------
// Bring the chip up: probe, reset, set PCA9685_PWM_FREQ_HZ, and force EVERY
// output fully off before anything else.
//
// Call once from setup(), after roverI2cInit() and with all TCA channels
// deselected (the PCA9685 is on the MAIN bus segment, not behind the mux).
//
// Returns false when the address is unconfirmed or the chip did not answer.
// A false return is not recoverable by retrying in a loop -- it is a wiring or
// configuration fact, and motor.cpp treats it as "no motion is possible".
// ---------------------------------------------------------------------------
bool pca9685Init(void);

// True when config.h says a human has confirmed the address.
bool pca9685AddressConfirmed(void);

// The configured address. Meaningless unless pca9685AddressConfirmed().
uint8_t pca9685Address(void);

// True only when the chip was found AND initialised AND has not errored since.
// motor.cpp gates all motion on this.
bool pca9685Ready(void);

// Why it is unusable, for telemetry:
//   "OK" / "ADDRESS_UNCONFIRMED" / "NOT_FOUND" / "INIT_FAILED" / "BUS_ERROR"
const char *pca9685StatusName(void);

// ---------------------------------------------------------------------------
// Set one channel's duty, 0..PCA9685_PWM_MAX (12-bit).
//
// 0 uses the part's FULL-OFF bit rather than a 0-width pulse, and
// PCA9685_PWM_MAX uses FULL-ON. Both matter here: a "0-width" pulse on a real
// PCA9685 still emits a sliver, and a sliver on an L298N enable pin is a motor
// twitching rather than a motor stopped.
//
// Returns false on any I2C failure, and latches a bus error so
// pca9685Ready() goes false -- a motor enable line we failed to write is a
// motor enable line in an unknown state.
// ---------------------------------------------------------------------------
bool pca9685SetDuty(uint8_t channel, uint16_t duty);

// ---------------------------------------------------------------------------
// Force ALL 16 outputs fully off in a single transaction, using the part's
// ALL_LED registers. This is the emergency path: one write, every enable dead.
// ---------------------------------------------------------------------------
bool pca9685AllOff(void);

// ---------------------------------------------------------------------------
// COMMISSIONING PROBE. Works on ANY address, including before the address is
// confirmed -- that is its purpose.
//
// Probes, then reads MODE1 and PRESCALE back. It writes NOTHING, so it cannot
// disturb whatever is actually at that address. A PCA9685 that has just been
// powered up reads MODE1 = 0x11 (SLEEP | ALLCALL) and PRESCALE = 0x1E.
//
// Returns true if the address ACKed and both registers were readable; the
// caller decides whether the values look like a PCA9685.
// ---------------------------------------------------------------------------
bool pca9685ProbeCandidate(uint8_t address, uint8_t *mode1Out, uint8_t *prescaleOut);

#endif // PCA9685_H

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
// Returns false when the address is unconfirmed, the chip did not answer, or
// it did not read back the configuration written to it. motor.cpp treats that
// as "no motion is possible" until pca9685ServiceRecovery() brings it back.
// ---------------------------------------------------------------------------
bool pca9685Init(void);

// ---------------------------------------------------------------------------
// RUNTIME RECOVERY. Call once per loop() pass, ONLY after every stored motion
// request has been discarded (see the .cpp). While the chip is not ready it
// re-runs the full boot bring-up -- outputs forced full-off first and last --
// on a backoff schedule (config.h PCA9685_RECOVERY_*). Returns true on the
// call that brought the chip back. A no-op while ready or unconfirmed.
// ---------------------------------------------------------------------------
bool pca9685ServiceRecovery(void);

// Recovery reporting, for DIAG SYSTEM:
//   pca9685RecoveryName()  "NONE"      ready and never recovered (or unconfirmed)
//                          "RETRYING"  not ready; bring-up is being retried
//                          "RECOVERED" ready again after >= 1 recovery this boot
//   pca9685LastFaultName() "NONE" / "BOOT_INIT" / "RUNTIME_BUS": what caused the
//                          most recent loss of the drive this boot
const char *pca9685RecoveryName(void);
const char *pca9685LastFaultName(void);
uint16_t    pca9685RecoveryAttempts(void);  // bring-up retries since boot
uint16_t    pca9685Recoveries(void);        // retries that succeeded
uint16_t    pca9685RuntimeBusErrors(void);  // losses caused by a failed write
uint32_t    pca9685NextRetryInMs(void);     // 0 when not retrying

// True when config.h says a human has confirmed the address.
bool pca9685AddressConfirmed(void);

// The configured address. Meaningless unless pca9685AddressConfirmed().
uint8_t pca9685Address(void);

// True only when the chip was found AND initialised AND has not errored since
// its last successful bring-up. motor.cpp gates all motion on this.
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
// motor enable line in an unknown state. Only a complete bring-up (boot, or
// pca9685ServiceRecovery()) clears it.
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

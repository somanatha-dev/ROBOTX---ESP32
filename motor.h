#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h"

#define MOTOR_CHANNEL_COUNT 4

// ============================================================================
//  MOTOR SUBSYSTEM  --  DIRECTION ON ESP32 GPIO, SPEED ON PCA9685
// ============================================================================
//
// CORRECTED ARCHITECTURE. Each of the four motor channels is a pair of L298N
// direction inputs driven by plain ESP32 GPIO, plus ONE PCA9685 PWM channel on
// that half-bridge pair's enable (ENA or ENB):
//
//   channel   IN pins (ESP32, digital)    enable (PCA9685 PWM)
//   FRONT_A   IN1=23  IN2=25              ENA = CH0
//   FRONT_B   IN3=26  IN4=27              ENB = CH1
//   REAR_A    IN1=32  IN2=33              ENA = CH2
//   REAR_B    IN3=16  IN4=17              ENB = CH3
//
// The IN pins are NEVER PWM'd. An earlier revision did PWM them, because at
// the time no enable pin was known to exist. With real ENA/ENB present that
// would multiply two duty cycles together and the commanded speed would mean
// nothing.
//
// NO MOTION WITHOUT A VERIFIED PCA9685. While PCA9685_ADDRESS_CONFIRMED is 0,
// or the chip did not answer, motorDriveAvailable() is false and every motion
// call holds all four channels stopped. That is not a degraded mode to work
// around -- the enable lines are fed by a chip nobody has identified, and
// commanding a direction into it would be commanding a motor through
// unverified hardware.
// ============================================================================

// Which physical side a motor channel belongs to.
// Values MUST match MOTOR_SIDE_LEFT / MOTOR_SIDE_RIGHT in config.h, which is
// where the actual per-channel assignment is configured.
typedef enum {
    SIDE_LEFT  = MOTOR_SIDE_LEFT,
    SIDE_RIGHT = MOTOR_SIDE_RIGHT
} MotorSide;

// ---------------------------------------------------------------------------
// Initialisation.
// Forces every L298N direction input LOW and every PCA9685 enable output fully
// off before anything else, so the rover cannot twitch during boot.
//
// Call once from setup(), AFTER roverI2cInit(). Safe to call when the PCA9685
// address is unconfirmed -- it still secures the direction pins.
// ---------------------------------------------------------------------------
void motorInit(void);

// ---------------------------------------------------------------------------
// True only when the speed path is real: the PCA9685 address is confirmed, the
// chip answered, and no bus error has been latched since.
//
// Telemetry publishes this, and the Pi should refuse to drive without it.
// ---------------------------------------------------------------------------
bool motorDriveAvailable(void);

// Why motion is unavailable, for telemetry. Mirrors pca9685StatusName():
//   "OK" / "ADDRESS_UNCONFIRMED" / "NOT_FOUND" / "INIT_FAILED" / "BUS_ERROR"
const char *motorDriveStatusName(void);

// ---------------------------------------------------------------------------
// Primary motion API. DIFFERENTIAL DRIVE -- there is no steering input.
//   leftSpeed / rightSpeed : -255 .. +255
//   positive = forward, negative = reverse, zero = stop (coast)
//
// Values are clamped. Magnitudes below MOTOR_MIN_EFFECTIVE_DUTY become 0.
// Side grouping and per-channel polarity come from config.h.
//
// UNCHANGED FROM THE PI'S POINT OF VIEW: the -255..+255 scale is exactly what
// it was. Only what happens below this function changed.
// ---------------------------------------------------------------------------
void driveDifferential(int leftSpeed, int rightSpeed);

// ---------------------------------------------------------------------------
// Immediate stop of all four channels: enables to zero FIRST, then direction
// pins low. Works even when the PCA9685 is unavailable -- the direction pins
// are still driven low, which is all this firmware can do in that case.
// ---------------------------------------------------------------------------
void stopMotors(void);

// ---------------------------------------------------------------------------
// Legacy arc helpers, kept so existing callers/protocol stay working.
// All are thin wrappers over driveDifferential(). They are NOT steering --
// they just set an inner/outer wheel speed ratio (MOTOR_TURN_INNER_PCT).
// ---------------------------------------------------------------------------
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);

// Inner-wheel speed for a legacy arc turn. Single source of truth for the
// ratio, shared by motor.cpp and comm.cpp so they cannot drift apart.
int  motorArcInnerSpeed(int speed);

// ---------------------------------------------------------------------------
// COMMISSIONING ONLY.
// Drives exactly ONE motor channel (0..MOTOR_CHANNEL_COUNT-1) and forces the
// other three to zero. Used to determine the left/right mapping and rotation
// polarity empirically. Run with the rover ON BLOCKS.
// Deliberately bypasses the side/invert mapping so you observe raw hardware.
// ---------------------------------------------------------------------------
void motorDriveChannel(int channelIndex, int speed);

// Human-readable channel name, e.g. "FRONT_A". Returns "?" if out of range.
const char *motorChannelName(int channelIndex);

// Configured side of a channel, as a string: "LEFT" / "RIGHT" / "?".
const char *motorChannelSideName(int channelIndex);

// The L298N direction GPIOs and the PCA9685 enable channel for a motor
// channel, so commissioning output can state the wiring it is about to drive
// instead of the operator having to cross-reference config.h.
// Return 0xFF for an out-of-range index.
uint8_t motorChannelIn1Pin(int channelIndex);
uint8_t motorChannelIn2Pin(int channelIndex);
uint8_t motorChannelEnableCh(int channelIndex);

// The value a requested speed ACTUALLY becomes after clamping and deadband.
// Lets MOTORTEST report honestly that e.g. power=20 was turned into 0, rather
// than acking a test that could never have spun the motor.
int motorEffectivePower(int requested);

// True only when a human has verified and committed the motor map in
// config.h. Reported in telemetry so the Pi never assumes it is trustworthy.
bool motorMapVerified(void);

#endif // MOTOR_H

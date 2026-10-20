#ifndef STALL_GUARD_H
#define STALL_GUARD_H

#include <Arduino.h>

// ============================================================================
//  LOOP-STALL GUARD  --  cuts the motor direction inputs while loop() is stuck
// ============================================================================
//
// THE PROBLEM. Every I2C transaction runs synchronously inside loop(). On core
// 2.0.14 a hung transaction blocks for >= 1 s whatever Wire.setTimeOut() says,
// and for that whole time the wheels keep their last direction and enable
// duty: STOP, the command watchdog and the safety gate all wait for it.
//
// THE GUARD. A FreeRTOS task pinned to CORE 0 (loop() runs on core 1) wakes
// every STALL_GUARD_PERIOD_MS. loop() refreshes a heartbeat at the start of
// every pass and at every loop-progress marker (comm.cpp commLoopBegin/Mark).
// While loop() is blocked inside an I2C transaction it is WAITING (on a
// FreeRTOS queue), not spinning, so the guard runs normally. If the heartbeat
// is older than STALL_GUARD_T_STALL_MS and any of the eight L298N direction
// inputs is HIGH, the guard TRIPS:
//   1. drives all eight direction inputs LOW -- direct GPIO register writes,
//      no I2C, no Wire lock, nothing that waits;
//   2. sets a sticky trip flag that refuses every later direction-HIGH write;
//   3. leaves drive-loss evidence for the motor layer, which comm.cpp turns
//      into safety_stop + DRIVE_LOSS_UNACKNOWLEDGED (the Phase 1 flow);
//   4. re-asserts LOW on every tick while tripped.
// Only an acknowledging RESET clears the trip (stallGuardClearTrip()).
//
// LIVENESS. Every tick stamps its time. stallGuardOperational() -- and with it
// the drive (motorDriveAvailable()) -- is true only once the task has been
// created AND has ticked within STALL_GUARD_LIVENESS_MS. A guard that never
// started, has not ticked yet, or stops ticking leaves the drive unavailable,
// refuses every direction-HIGH write, and the motor layer records any output
// live at that moment (or a non-zero request refused for it) as a drive loss.
// The check runs in loop(): if the guard stops AND loop() stalls at the same
// time, nothing in this firmware cuts the direction inputs.
//
// SYNCHRONISATION CONTRACT. Every direction-HIGH write in the firmware goes
// through stallGuardSetDirection(), which checks the trip flag and writes the
// pins inside the SAME cross-core critical section the guard trips in. So
// either the write lands first and the guard's cut overrides it, or the guard
// trips first and the write is refused -- a slow I2C write returning after a
// trip can never raise a pin. LOW writes need no coordination.
//
// WHAT THIS IS NOT. Both direction inputs LOW with the enable still HIGH is the
// L298N's "fast motor stop": the motor is braked and gets no drive, but the
// bridge stays powered and the PCA9685 enable keeps whatever duty it had (it
// cannot be written without I2C). This is a software brake on the same chip,
// not a hardware safety system: it does not survive an ESP32 reset, crash or
// brownout, and it removes no power. During a reset the ESP32 no longer
// controls the direction inputs: depending on the reset type they may float
// or keep their last output level until setup() drives them LOW (not
// verified for any reset type).
// STALL_GUARD_T_STALL_MS is a PROVISIONAL host-test value, not a validated
// physical requirement (config.h).
// ============================================================================

// setup(), before motorInit(): create the task (once). Not armed yet.
void stallGuardBegin(void);

// End of setup(): start watching. Boot work before this may block freely.
void stallGuardArm(void);

// loop() progress. Cheap; called by comm.cpp's loop-timing markers.
void stallGuardHeartbeat(void);

// One guard check at `nowMs` (the task calls this every period; host tests
// call it from their fake clock). Returns true on the tick that trips.
bool stallGuardTick(uint32_t nowMs);

// Drive one channel's pair of direction inputs (pinA <- levelA, pinB <- levelB)
// unless the guard has tripped or is not operational. Atomic with the guard's
// cut. Returns false, having written nothing, when it refuses.
bool stallGuardSetDirection(uint8_t pinA, uint8_t levelA, uint8_t pinB, uint8_t levelB);

// The trip produced evidence that output was live when it fired. Returns it
// and clears it (consume once). Read by the motor layer.
bool stallGuardConsumeTripEvidence(void);

// Acknowledging RESET only: re-allow direction-HIGH writes.
void stallGuardClearTrip(void);

// Task created and ticked within STALL_GUARD_LIVENESS_MS. loop() only.
bool        stallGuardOperational(void);
bool        stallGuardArmed(void);
bool        stallGuardTripped(void);
uint32_t    stallGuardTrips(void);         // trips since boot
const char *stallGuardStateName(void);     // NOT_RUNNING / DISARMED / ARMED / TRIPPED

#endif // STALL_GUARD_H

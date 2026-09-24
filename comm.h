#ifndef COMM_H
#define COMM_H

#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Serial link to the Raspberry Pi (UART0: GPIO1 TX / GPIO3 RX, GND common).
// Line protocol: one JSON object per line, terminated with '\n'.
//
// COMMANDS THE PI MAY SEND  --  UNCHANGED. Nothing below was removed,
// renamed, or given different semantics; the diagnostics block is additive.
//   {"cmd":"DRIVE","left":150,"right":150}   <- primary differential command
//   {"cmd":"STOP"}
//   {"cmd":"RESET"}                          <- clears the safety-stop latch
//   {"cmd":"PING"}
//   {"cmd":"MOVE","dir":"F","speed":150}     <- legacy, retained
//
// COMMISSIONING
//   {"cmd":"MOTORTEST","motor":0,"power":120,"ms":600,"onblocks":true}
//        Spins ONE motor channel with the obstacle gate bypassed. Requires
//        "onblocks":true while MOTORTEST_REQUIRE_ONBLOCKS is 1 in config.h --
//        THE ROVER MUST BE SECURED WITH ITS WHEELS OFF THE GROUND.
//
// DIAGNOSTICS  (DIAGNOSTICS_ENABLED in config.h; none can turn a wheel, and
//               none refreshes the movement watchdog)
//   {"cmd":"I2CSCAN"}                    <- list responding I2C addresses
//   {"cmd":"I2CSTATUS"}                  <- SDA/SCL pad levels, no bus traffic
//   {"cmd":"TCATEST","addr":"0x70"}      <- identify the TCA9548A
//   {"cmd":"PCATEST","addr":"0x40"}      <- probe a PCA9685 candidate
//   {"cmd":"TOFTEST","sensor":1}         <- read one/all rear VL53L0X
//   {"cmd":"HWREPORT"}                   <- dump config + what is unverified
// ---------------------------------------------------------------------------
void commInit(void);

// ---------------------------------------------------------------------------
// ONE-SHOT I2C bus scan, emitted at the end of setup() when I2C_BOOT_SCAN is 1.
//
// Prints exactly the same line as {"cmd":"I2CSCAN"}, with "trigger":"boot".
// Call it ONCE, from setup(). Never from loop(): a scan blocks the control
// loop for tens of milliseconds, and repeating that to re-answer a question
// whose answer has not changed would starve the safety gate.
//
// Compiles to a no-op when DIAGNOSTICS_ENABLED is 0.
// ---------------------------------------------------------------------------
void commBootI2cScan(void);

// ---------------------------------------------------------------------------
// Non-blocking. Call every pass of loop(). Consumes whatever bytes are
// available and dispatches any complete lines. Never blocks waiting for a
// line terminator.
// ---------------------------------------------------------------------------
void commPoll(void);

// ---------------------------------------------------------------------------
// Emit one telemetry line. Assembled into a single buffer and written once.
// ---------------------------------------------------------------------------
void sendTelemetry(void);

// ---------------------------------------------------------------------------
// Parse and act on a single already-terminated command line.
// Exposed mainly for testing; commPoll() calls it for you.
// ---------------------------------------------------------------------------
void handleCommand(const char *line);

// ---------------------------------------------------------------------------
// REQUESTED motion, as most recently accepted from the Pi. -255..+255.
// This is the raw intent. The safety gate is applied to it fresh on every
// pass of loop(), so it is NOT necessarily what the motors are doing.
// ---------------------------------------------------------------------------
int  commDesiredLeft(void);
int  commDesiredRight(void);

// Force the requested motion to zero. Used by the command failsafe.
void commClearDesired(void);

// ---------------------------------------------------------------------------
// Per-pass runtime update, called by loop() AFTER the safety gate has run.
// Records what was actually applied to the motors and why anything was
// clamped, then RECOMPUTES the reported robot state from live conditions.
//
// This is what stops the state machine getting stuck: the state is derived,
// never latched. The sticky record of "a safety stop happened" is the
// separate commSafetyStopLatched() flag.
// ---------------------------------------------------------------------------
void commUpdateRuntime(int appliedLeft, int appliedRight, const char *blockReason);

int         commAppliedLeft(void);
int         commAppliedRight(void);
const char *commBlockReason(void);

// ---------------------------------------------------------------------------
// COMMAND FAILSAFE.
//
// Milliseconds since the last well-formed movement command (DRIVE / MOVE /
// STOP). Malformed lines, unknown commands, over-long lines, PING, RESET and
// MOTORTEST do NOT refresh this, so serial noise can never keep the motors
// alive.
// ---------------------------------------------------------------------------
uint32_t commMsSinceLastCommand(void);

// True once any movement command has ever been accepted since boot.
bool commHasEverReceivedCommand(void);

// The failsafe latch. Set by loop() when the window expires; cleared by the
// next well-formed movement command.
void commSetTimedOut(bool timedOut);
bool commTimedOut(void);

// ---------------------------------------------------------------------------
// Robot state, owned here so there is exactly one definition.
// ---------------------------------------------------------------------------
void       commSetState(RobotState s);
RobotState commGetState(void);

// ---------------------------------------------------------------------------
// SAFETY-STOP LATCH.
//
// Held separately from RobotState because the state field is single-valued
// and is recomputed every pass -- a safety stop followed 2 s later by a
// command timeout would otherwise erase the fact that a safety stop ever
// happened, and the Pi would lose the reason the rover halted.
//
// The latch is cleared ONLY by a fully-unclamped accepted movement command or
// by RESET. It is never cleared merely because the obstacle went away.
// ---------------------------------------------------------------------------
void commLatchSafetyStop(void);
bool commSafetyStopLatched(void);

// ---------------------------------------------------------------------------
// MOTORTEST commissioning mode. Bounded in duration; expires by itself.
// ---------------------------------------------------------------------------
bool commTestModeActive(void);
int  commTestChannel(void);
int  commTestSpeed(void);

#endif // COMM_H

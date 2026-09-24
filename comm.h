#ifndef COMM_H
#define COMM_H

#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------------------
// Serial link to the Raspberry Pi (UART0: GPIO1 TX / GPIO3 RX, GND common).
//
// PROTOCOL v2 -- full specification in PROTOCOL.md. In short:
//
//   every frame   {flat JSON object}*CRC4\n     (CRC-16/CCITT-FALSE, 4 hex)
//   Pi -> ESP32   {"type":"COMMAND","seq":N,"cmd":"...",<fields>}
//   ESP32 -> Pi   type = ACK | ERROR | EVENT | TELEMETRY | DIAG
//
// COMMANDS
//   DRIVE      left, right             (-255..255, both required)
//   STOP
//   MOVE       dir F|B|L|R|S, speed    (0..255, required unless dir is S)
//   RESET                              clears the safety-stop latch
//   PING
//
// COMMISSIONING
//   MOTORTEST  motor 0..3, power -255..255, ms 1..MOTORTEST_MAX_MS,
//              onblocks (must be true while MOTORTEST_REQUIRE_ONBLOCKS is 1)
//              Spins ONE motor channel with the obstacle gate bypassed.
//              THE ROVER MUST BE SECURED WITH ITS WHEELS OFF THE GROUND.
//
// DIAGNOSTICS  (DIAGNOSTICS_ENABLED in config.h; none can turn a wheel, and
//               none refreshes the movement watchdog)
//   I2CSCAN, I2CSTATUS, TCATEST [addr], PCATEST [addr], TOFTEST [sensor],
//   HWREPORT                          -- results are carried in the ACK
//
// Every well-formed command gets exactly ONE ACK echoing its seq. A frame that
// cannot be accepted as a command gets one ERROR instead.
// ---------------------------------------------------------------------------
void commInit(void);

// ---------------------------------------------------------------------------
// Application-level READY event. Call once, at the end of setup().
//
// The ESP32 boot ROM prints plain text on UART0 before the application runs.
// Firmware cannot suppress that; it carries no valid CRC trailer, so the Pi
// discards it. READY is the first frame the application itself sends.
// ---------------------------------------------------------------------------
void commEmitReady(void);

// ---------------------------------------------------------------------------
// ONE-SHOT I2C bus scan, emitted at the end of setup() when I2C_BOOT_SCAN is 1,
// as an EVENT with "trigger":"BOOT". Never call it from loop(): a scan blocks
// the control loop for tens of milliseconds.
//
// Compiles to a no-op when DIAGNOSTICS_ENABLED is 0.
// ---------------------------------------------------------------------------
void commBootI2cScan(void);

// ---------------------------------------------------------------------------
// Non-blocking. Call every pass of loop(). Consumes whatever bytes are
// available and processes any complete frames. Never waits for a terminator.
// ---------------------------------------------------------------------------
void commPoll(void);

// ---------------------------------------------------------------------------
// Telemetry scheduler. Call every pass of loop(). Sends the compact TELEMETRY
// frame every TELEMETRY_INTERVAL_MS, and one DIAG section every
// TELEMETRY_DIAG_EVERY_N_FAST fast frames, half an interval later so the two
// never go out back to back.
// ---------------------------------------------------------------------------
void commServiceTelemetry(void);

// ---------------------------------------------------------------------------
// Process one received line (without its '\n'). commPoll() calls this; it is
// exposed so the host tests can drive the protocol directly.
// ---------------------------------------------------------------------------
void handleCommand(const char *line, size_t len);

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
// Per-pass runtime update, called by loop() AFTER the safety gate has run,
// with the GATED values -- what the safety and availability gates permitted.
// Records them, derives what the motor layer actually outputs from them
// (clamp + deadband, or zero with no drive), then RECOMPUTES the reported
// robot state from live conditions.
//
// This is what stops the state machine getting stuck: the state is derived,
// never latched. The sticky record of "a safety stop happened" is the
// separate commSafetyStopLatched() flag.
// ---------------------------------------------------------------------------
void commUpdateRuntime(int gatedLeft, int gatedRight, const char *blockReason);

int         commAppliedLeft(void);
int         commAppliedRight(void);
const char *commBlockReason(void);

// ---------------------------------------------------------------------------
// COMMAND FAILSAFE.
//
// Milliseconds since the last fully validated, accepted (or gated) movement
// command -- DRIVE / MOVE / STOP. Invalid frames, bad CRCs, rejected commands,
// duplicates, unknown commands, PING, RESET, MOTORTEST and diagnostics do NOT
// refresh this, so nothing but a real movement command keeps the motors alive.
// ---------------------------------------------------------------------------
uint32_t commMsSinceLastCommand(void);

// True once any movement command has ever been accepted since boot.
bool commHasEverReceivedCommand(void);

// The failsafe latch. Set by loop() when the window expires; cleared by the
// next accepted movement command. The false -> true transition emits one
// COMMAND_TIMEOUT event.
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

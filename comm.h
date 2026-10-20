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
// Loop timing, reported in DIAG SYSTEM (loop_max_ms, last_stall_*). loop()
// brackets each pass with commLoopBegin() / commLoopEnd() and calls
// commLoopMark(source) as each section ends. commLoopNested(source, ms)
// charges time a sub-step inside the current section measured itself (the
// rear ToF half of safetyUpdate()), so a stall is named after that sub-step
// and not after the whole section. Arithmetic only: nothing here blocks.
// ---------------------------------------------------------------------------
void commLoopBegin(void);
void commLoopNested(const char *source, uint32_t ms);
void commLoopMark(const char *source);
void commLoopEnd(void);

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
// The speed path is unavailable (no ready PCA9685). Called by loop() on every
// such pass, BEFORE any recovery attempt, so a drive that comes back can never
// act on a request made before it was lost:
//   * the stored request is zeroed and a running MOTORTEST is CANCELLED;
//   * a pending drive-loss record from the motor layer is turned into the
//     safety-stop latch and an acknowledgement requirement (see the
//     SAFETY-STOP LATCH note below);
//   * any earlier report of the drive being available is voided, so an
//     acknowledgement must follow a report made after it comes back.
// The watchdog and its timeout latch are not touched.
// ---------------------------------------------------------------------------
void commDiscardMotionNoDrive(void);

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
// It is SET by:
//   * a safety gate clamping a command (obstacle, sensor fault, or the
//     REAR_UNCONFIGURED policy) -- in loop() and in the command handler;
//   * a DRIVE LOSS: a PCA9685 enable write failing while output was live (any
//     direction input driven at that instant) or while a non-zero speed was
//     being written. The motor layer records that at the failure itself
//     (motorConsumeLostWhileLive()) and comm.cpp latches it before handling
//     any further command, runtime update or telemetry, so a command arriving
//     during a hung write cannot erase it. Runtime recovery of the PCA9685
//     never clears it.
//
// The latch is cleared ONLY by a fully-unclamped accepted movement command or
// by RESET. It is never cleared merely because the obstacle went away.
//
// A DRIVE LOSS additionally needs ACKNOWLEDGING. Until then the latch is held,
// DRIVE/MOVE are GATED and MOTORTEST REJECTED (reason
// DRIVE_LOSS_UNACKNOWLEDGED), and RESET -- still ACCEPTED, still stopping the
// motors and clearing the command timeout -- clears the latch only if the
// drive is available and the RESET line began arriving after the settle
// snapshot: a TELEMETRY report queued once the drive had been available, and
// reported available, continuously for ACK_SETTLE_MS (config.h). Any outage
// restarts that. A RESET sent while the drive was down, or within about
// ACK_SETTLE_MS of it coming back, acknowledges nothing.
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

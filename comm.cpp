#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>

#include "comm.h"
#include "motor.h"
#include "safety.h"
#include "rear_tof.h"
#include "rover_i2c.h"
#include "tca9548a.h"
#include "pca9685.h"
#include "config.h"

// ============================================================================
// STATE (single definition -- nothing else defines these)
// ============================================================================

static RobotState gState = STATE_BOOT;

// What the Pi asked for.
static int  gDesiredLeft  = 0;
static int  gDesiredRight = 0;

// What actually reached the motors after the safety gate.
static int  gAppliedLeft  = 0;
static int  gAppliedRight = 0;

// Why the gate clamped something, or "NONE".
static const char *gBlockReason = "NONE";

// Most recent command rejection, or "NONE". Sticky until the next rejection.
//
// This exists because STATE_INVALID_COMMAND cannot survive in gState: the
// state is recomputed from live conditions on every loop pass (~1 ms), so a
// transient "bad command" state would be overwritten long before the next
// telemetry line. Publishing the reason as its own sticky field is the honest
// way to tell the Pi that a line was refused.
static const char *gLastRejectReason = "NONE";

static uint32_t gLastCommandMs   = 0;
static bool     gEverReceivedCmd = false;
static bool     gTimedOut        = false;

// Latched safety stop. See the note in comm.h for why this is not just a
// RobotState value.
static bool     gSafetyStopLatched = false;

// Incoming line assembly.
static char     gLine[COMM_LINE_MAX];
static uint16_t gLineLen  = 0;
static bool     gOverflow = false;

// MOTORTEST.
static bool     gTestActive   = false;
static uint32_t gTestDeadline = 0;
static int      gTestChannel  = 0;
static int      gTestSpeed    = 0;


// ============================================================================
// STATE NAMES
// ============================================================================

const char *robotStateName(RobotState s)
{
    switch (s) {
        case STATE_BOOT:            return "BOOT";
        case STATE_IDLE:            return "IDLE";
        case STATE_STOPPED:         return "STOPPED";
        case STATE_MOVING_FORWARD:  return "MOVING_FORWARD";
        case STATE_MOVING_BACKWARD: return "MOVING_BACKWARD";
        case STATE_TURNING_LEFT:    return "TURNING_LEFT";
        case STATE_TURNING_RIGHT:   return "TURNING_RIGHT";
        case STATE_ROTATING:        return "ROTATING";
        case STATE_COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
        case STATE_SAFETY_STOP:     return "SAFETY_STOP";
        case STATE_SENSOR_FAULT:    return "SENSOR_FAULT";
        case STATE_MOTOR_TEST:      return "MOTOR_TEST";
        case STATE_INVALID_COMMAND: return "INVALID_COMMAND";
        case STATE_DRIVE_UNAVAILABLE: return "DRIVE_UNAVAILABLE";
        default:                    return "UNKNOWN";
    }
}


// ============================================================================
// MINIMAL PARSING HELPERS
// ============================================================================
//
// Deliberately no JSON library -- no new dependency, and the Pi's message set
// is small and fixed. These look for an exact "key": token and read the value
// that follows, so a substring cannot match by accident.
// ============================================================================

// Finds "<key>" then the ':' after it, returning a pointer just past the colon.
static const char *findValue(const char *line, const char *key)
{
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *p = strstr(line, pattern);
    if (p == NULL) {
        return NULL;
    }

    p += strlen(pattern);

    while (*p == ' ' || *p == '\t') p++;
    if (*p != ':') {
        return NULL;
    }
    p++;
    while (*p == ' ' || *p == '\t') p++;

    return p;
}

static bool parseInt(const char *line, const char *key, int *out)
{
    const char *p = findValue(line, key);
    if (p == NULL) {
        return false;
    }
    if (*p != '-' && *p != '+' && !isdigit((unsigned char)*p)) {
        return false;
    }
    *out = (int)strtol(p, NULL, 10);
    return true;
}

// True only when the field is present and literally `true`. A missing field,
// `false`, 0, "true" as a string, or anything else is NOT true -- this gates a
// command that spins a wheel, so it fails closed on anything ambiguous.
static bool boolFieldIsTrue(const char *line, const char *key)
{
    const char *p = findValue(line, key);
    if (p == NULL) {
        return false;
    }
    return (strncmp(p, "true", 4) == 0);
}

// Compares a string-valued field against an expected value, e.g.
// stringFieldIs(line, "cmd", "DRIVE").
static bool stringFieldIs(const char *line, const char *key, const char *expected)
{
    const char *p = findValue(line, key);
    if (p == NULL || *p != '"') {
        return false;
    }
    p++;

    size_t n = strlen(expected);
    if (strncmp(p, expected, n) != 0) {
        return false;
    }
    return p[n] == '"';
}


// ============================================================================
// ACK HELPERS
// ============================================================================

static void ackAccepted(const char *cmd, int left, int right)
{
    // A clean acceptance clears the sticky rejection, so "last_reject" reads
    // as "what went wrong since the last command that worked".
    gLastRejectReason = "NONE";

    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"ack\":\"%s\",\"accepted\":true,\"gated\":false,"
             "\"left\":%d,\"right\":%d}",
             cmd, left, right);
    Serial.println(buf);
}

// A command the gate clamped. "accepted":false is kept for backward
// compatibility with any Pi code that only checks that flag, and the gated
// values say exactly what the motors were actually given.
static void ackGated(const char *cmd, const char *reason,
                     int reqL, int reqR, int gotL, int gotR)
{
    char buf[192];
    snprintf(buf, sizeof(buf),
             "{\"ack\":\"%s\",\"accepted\":false,\"gated\":true,"
             "\"reason\":\"%s\",\"requested_left\":%d,\"requested_right\":%d,"
             "\"left\":%d,\"right\":%d}",
             cmd, reason, reqL, reqR, gotL, gotR);
    Serial.println(buf);
}

static void ackRejected(const char *cmd, const char *reason)
{
    gLastRejectReason = reason;

    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"ack\":\"%s\",\"accepted\":false,\"gated\":false,"
             "\"reason\":\"%s\"}",
             cmd, reason);
    Serial.println(buf);
}


// ============================================================================
// STATE DERIVED FROM A MOTION PAIR
// ============================================================================
//
// Differential drive: the state has to be inferred from the SIGNS and the
// RELATIVE MAGNITUDES of the two wheel speeds, because there is no steering
// input to read.
//
// Note the >= / <= comparisons. A pivot turn with one side held at zero --
// (0, 150) or (150, 0) -- is a turn, not a rotation-in-place.
// ============================================================================

static RobotState stateForMotion(int left, int right)
{
    if (left == 0 && right == 0)            return STATE_STOPPED;

    if (left >= 0 && right >= 0) {          // forward, arc, or forward pivot
        if (left < right)                   return STATE_TURNING_LEFT;
        if (left > right)                   return STATE_TURNING_RIGHT;
        return STATE_MOVING_FORWARD;
    }

    if (left <= 0 && right <= 0)            return STATE_MOVING_BACKWARD;

    // Opposite signs -> counter-rotating wheels -> rotating in place.
    return STATE_ROTATING;
}


// ============================================================================
// EXPLICIT STATE MACHINE
// ============================================================================
//
// The reported state is RECOMPUTED from live conditions every pass, in strict
// priority order. Nothing is latched here, which is what guarantees the rover
// cannot sit reporting SAFETY_STOP forever after the obstacle has gone.
//
//   1. MOTOR_TEST       - commissioning override is running
//   2. COMMAND_TIMEOUT  - failsafe latch is set (cleared by the next command)
//   3. SAFETY_STOP      - the gate is clamping RIGHT NOW because of a
//                         confirmed obstacle
//   4. SENSOR_FAULT     - the gate is clamping RIGHT NOW because sensing is
//                         not trustworthy
//   5. IDLE             - booted, never commanded
//   6. motion state     - derived from what actually reached the motors
//
// The STICKY "this rover performed a safety stop" record is the separate
// commSafetyStopLatched() flag, published as "safety_stop". It is intentional
// that it survives the obstacle going away; it is cleared by RESET or by a
// fully-unclamped accepted movement command.
// ============================================================================

static RobotState computeState(void)
{
    if (gTestActive) {
        return STATE_MOTOR_TEST;
    }

    // No speed path at all. Outranks everything below, because everything
    // below describes WHY a rover that could move is not moving -- and this
    // one is a rover that cannot move.
    if (!motorDriveAvailable()) {
        return STATE_DRIVE_UNAVAILABLE;
    }

    if (gTimedOut) {
        return STATE_COMMAND_TIMEOUT;
    }

    bool blockedNow = (strcmp(gBlockReason, "NONE") != 0);

    if (blockedNow) {
        if (safetyObstacleDetected() || safetyRearObstacleDetected()) {
            return STATE_SAFETY_STOP;
        }
        if (safetySensorFault() || safetyRearSensorFault()) {
            return STATE_SENSOR_FAULT;
        }
        // Blocked for a policy reason (e.g. rear sensing unconfigured and
        // reverse refused). Not an obstacle and not a fault.
        return STATE_SAFETY_STOP;
    }

    if (!gEverReceivedCmd) {
        return STATE_IDLE;
    }

    return stateForMotion(gAppliedLeft, gAppliedRight);
}

void commUpdateRuntime(int appliedLeft, int appliedRight, const char *blockReason)
{
    gAppliedLeft  = appliedLeft;
    gAppliedRight = appliedRight;
    gBlockReason  = (blockReason != NULL) ? blockReason : "NONE";

    gState = computeState();
}

int         commAppliedLeft(void)  { return gAppliedLeft;  }
int         commAppliedRight(void) { return gAppliedRight; }
const char *commBlockReason(void)  { return gBlockReason;  }


// ============================================================================
// APPLY AN ACCEPTED MOTION REQUEST
// ============================================================================
//
// The safety gate is applied HERE as well as in the main loop. Here it lets
// us send the Pi a meaningful, immediate answer; the loop check is the actual
// continuous authority that catches an obstacle appearing mid-manoeuvre.
//
// BOTH call safetyGateMotion() -- the single direction-aware gate in
// safety.cpp -- rather than each re-deriving "is this forward?". Two copies
// of that rule would eventually disagree, and the way they would disagree is
// one of them permitting motion the other considered unsafe.
//
// WATCHDOG POLICY: a WELL-FORMED movement command refreshes the failsafe
// timer even when the gate clamps it. The timer measures whether the Pi is
// still talking to us, and a Pi that is pushing into an obstacle is very much
// still talking. Malformed lines, unknown commands and over-long lines never
// refresh it.
// ============================================================================

static bool applyMotion(const char *cmdName, int left, int right)
{
    // A well-formed movement command: the link is alive.
    gLastCommandMs   = millis();
    gEverReceivedCmd = true;
    gTimedOut        = false;

    // Store the raw intent. The loop re-gates it every pass, so if the
    // obstacle clears while the Pi is still sending this command, motion
    // resumes -- which is correct, because the Pi is still asking for it. A
    // command the Pi has STOPPED sending is zeroed by the watchdog within
    // COMMAND_TIMEOUT_MS regardless.
    gDesiredLeft  = left;
    gDesiredRight = right;

    // ------------------------------------------------------------------
    // NO SPEED PATH. The PCA9685 drives every L298N enable input, so without
    // a verified one there is nothing to command.
    //
    // This is reported as GATED rather than REJECTED on purpose. The command
    // itself was perfectly well-formed, the link is alive, and the watchdog
    // above has already been refreshed -- so telling the Pi "bad command"
    // would send it looking in the wrong place. The reason string names the
    // actual problem, and the state becomes DRIVE_UNAVAILABLE.
    //
    // The stored intent is kept, not cleared, so that if the PCA9685 becomes
    // available the next loop pass acts on what the Pi is still asking for --
    // the same rule the obstacle gate follows.
    // ------------------------------------------------------------------
    if (!motorDriveAvailable()) {
        stopMotors();
        commUpdateRuntime(0, 0, "MOTOR_PWM_UNAVAILABLE");
        ackGated(cmdName, "MOTOR_PWM_UNAVAILABLE", left, right, 0, 0);
        return false;
    }

    int gl = 0;
    int gr = 0;
    safetyGateMotion(left, right, &gl, &gr);

    if (gl != left || gr != right) {
        const char *reason = safetyBlockReason(left, right);

        driveDifferential(gl, gr);

#if SAFETY_CLEAR_COMMAND_ON_BLOCK
        gDesiredLeft  = 0;
        gDesiredRight = 0;
        stopMotors();
        gl = 0;
        gr = 0;
#endif

        commLatchSafetyStop();
        commUpdateRuntime(gl, gr, reason);

        ackGated(cmdName, reason, left, right, gl, gr);
        return false;
    }

    // ------------------------------------------------------------------
    // Fully permitted. This is the ONLY place a latched safety stop is
    // released by a movement command, and it happens because a NEW command
    // arrived and passed the gate cleanly -- never because the obstacle
    // went away on its own.
    // ------------------------------------------------------------------
    gSafetyStopLatched = false;

    driveDifferential(gl, gr);
    commUpdateRuntime(gl, gr, "NONE");

    ackAccepted(cmdName, gl, gr);
    return true;
}


// ============================================================================
// COMMAND HANDLERS
// ============================================================================

// Explicit STOP. Zeroes the stored command, so the rover STAYS stopped: the
// main loop re-asserts gDesiredLeft/Right every pass, and both are now 0.
//
// The safety-stop latch is deliberately NOT cleared here. STOP means "hold
// still", not "the obstacle situation is resolved", and keeping the latch set
// preserves WHY the rover halted. RESET is the command that acknowledges it.
static void handleStop(void)
{
    stopMotors();

    gDesiredLeft  = 0;
    gDesiredRight = 0;

    gLastCommandMs   = millis();
    gEverReceivedCmd = true;
    gTimedOut        = false;

    commUpdateRuntime(0, 0, "NONE");

    // STOP is always accepted -- it can never be unsafe.
    ackAccepted("STOP", 0, 0);
}


static void handleDrive(const char *line)
{
    int left  = 0;
    int right = 0;

    bool haveLeft  = parseInt(line, "left",  &left);
    bool haveRight = parseInt(line, "right", &right);

    // Neither field present -> this is not a usable DRIVE. It must NOT
    // refresh the watchdog.
    if (!haveLeft && !haveRight) {
        // Rejected: does NOT refresh the watchdog, does NOT touch the motors,
        // does NOT touch the stored command. Reported via "last_reject".
        ackRejected("DRIVE", "MISSING_LEFT_RIGHT");
        return;
    }

    // A missing side defaults to 0 rather than to the previous value, so a
    // malformed command can never leave a wheel spinning.
    applyMotion("DRIVE", left, right);
}


// Legacy protocol, retained so existing Pi code keeps working unchanged:
//   {"cmd":"MOVE","dir":"F","speed":150}
// Directions are translated into differential pairs. This is NOT steering --
// L/R simply slow the inner wheels by MOTOR_TURN_INNER_PCT.
static void handleMove(const char *line)
{
    int speed = 150;
    parseInt(line, "speed", &speed);

    if (speed < 0)   speed = 0;
    if (speed > 255) speed = 255;

    int left, right;

    if (stringFieldIs(line, "dir", "F")) {
        left = speed;                       right = speed;
    } else if (stringFieldIs(line, "dir", "B")) {
        left = -speed;                      right = -speed;
    } else if (stringFieldIs(line, "dir", "L")) {
        left = motorArcInnerSpeed(speed);   right = speed;
    } else if (stringFieldIs(line, "dir", "R")) {
        left = speed;                       right = motorArcInnerSpeed(speed);
    } else if (stringFieldIs(line, "dir", "S")) {
        handleStop();
        return;
    } else {
        // Not a usable movement command -- does not refresh the watchdog.
        // An unrecognised direction is treated as "stop what you were doing"
        // rather than "carry on", which is the safe reading of a garbled
        // movement instruction.
        stopMotors();
        gDesiredLeft  = 0;
        gDesiredRight = 0;
        commUpdateRuntime(0, 0, "NONE");
        ackRejected("MOVE", "INVALID_DIRECTION");
        return;
    }

    applyMotion("MOVE", left, right);
}


// Commissioning only. Drives ONE raw motor channel for a bounded time so the
// left/right mapping and polarity can be established by observation.
// Deliberately bypasses the obstacle gate: the rover is expected to be ON
// BLOCKS. The duration is hard-capped so it cannot run away, and it does NOT
// refresh the movement watchdog.
//
// Preferred form:  {"cmd":"MOTORTEST","motor":0,"power":120,"ms":600}
// Also accepted :  {"cmd":"MOTORTEST","ch":0,"speed":120,"ms":600}
//
// "motor" is 0..3 and selects exactly ONE of the four motor channels. Every
// other channel is forced to zero for the duration, so only one wheel can
// ever move -- that is what makes the mapping observable.
static void handleMotorTest(const char *line)
{
    int motor = 0;
    int power = 120;
    int ms    = (int)MOTORTEST_DEFAULT_MS;

    // Accept the documented names first, then the older aliases.
    if (!parseInt(line, "motor", &motor)) {
        parseInt(line, "ch", &motor);
    }
    if (!parseInt(line, "power", &power)) {
        parseInt(line, "speed", &power);
    }
    parseInt(line, "ms", &ms);

    if (motor < 0 || motor >= MOTOR_CHANNEL_COUNT) {
        ackRejected("MOTORTEST", "BAD_MOTOR");
        return;
    }

    // ------------------------------------------------------------------
    // PHYSICAL INTERLOCK.
    //
    // MOTORTEST spins a wheel with the obstacle gate bypassed. On a rover
    // whose wheels are on the ground that is a machine driving itself across
    // the room. Require the caller to state, in the command, that the rover is
    // secured.
    //
    // This is not security -- anyone can type "onblocks":true. It is a guard
    // against the realistic failure, which is a half-remembered command pasted
    // from a notebook while the rover sits on the bench floor.
    // ------------------------------------------------------------------
#if MOTORTEST_REQUIRE_ONBLOCKS
    if (!boolFieldIsTrue(line, "onblocks")) {
        ackRejected("MOTORTEST",
                    "NEEDS_ONBLOCKS_TRUE_ROVER_MUST_BE_SECURED");
        return;
    }
#endif

    // No speed path means no test. Say so explicitly rather than running a
    // test that silently does nothing and leaves the operator concluding the
    // motor channel is dead.
    if (!motorDriveAvailable()) {
        ackRejected("MOTORTEST", "MOTOR_PWM_UNAVAILABLE");
        return;
    }

    if (power >  255) power =  255;
    if (power < -255) power = -255;

    if (ms < 0) ms = 0;
    if ((uint32_t)ms > MOTORTEST_MAX_MS) ms = (int)MOTORTEST_MAX_MS;

    // What the hardware will ACTUALLY receive after clamp + deadband. If this
    // comes back 0 for a non-zero request, the motor will not turn and the
    // operator needs to know that now -- not after concluding the channel is
    // dead and rewiring a working rover.
    int effective = motorEffectivePower(power);

    gTestChannel  = motor;
    gTestSpeed    = power;
    gTestDeadline = millis() + (uint32_t)ms;
    gTestActive   = true;

    gDesiredLeft  = 0;
    gDesiredRight = 0;

    commUpdateRuntime(0, 0, "NONE");    // recomputes to STATE_MOTOR_TEST

    // The ack states the WIRING it is about to drive, so the operator can
    // check it against the loom in front of them instead of cross-referencing
    // config.h. "configured_side" is the UNVERIFIED assumption under test --
    // it is printed so you can see what the firmware currently believes and
    // contradict it.
    char buf[320];
    snprintf(buf, sizeof(buf),
             "{\"ack\":\"MOTORTEST\",\"accepted\":true,\"motor\":%d,"
             "\"name\":\"%s\",\"in1_gpio\":%u,\"in2_gpio\":%u,"
             "\"enable_pca_ch\":%u,\"configured_side\":\"%s\","
             "\"side_verified\":%s,\"power\":%d,\"effective_power\":%d,"
             "\"below_deadband\":%s,\"ms\":%d}",
             motor, motorChannelName(motor),
             (unsigned)motorChannelIn1Pin(motor),
             (unsigned)motorChannelIn2Pin(motor),
             (unsigned)motorChannelEnableCh(motor),
             motorChannelSideName(motor),
             motorMapVerified() ? "true" : "false",
             power, effective,
             (power != 0 && effective == 0) ? "true" : "false", ms);
    Serial.println(buf);
}


// Clears a latched SAFETY_STOP and the COMMAND_TIMEOUT latch without
// commanding any motion. Motors stay stopped; a real movement command is
// still required to move.
static void handleReset(void)
{
    stopMotors();
    gDesiredLeft  = 0;
    gDesiredRight = 0;
    gTestActive   = false;

    // RESET is the explicit acknowledgement of a safety stop. It clears the
    // latch but commands NO motion -- the rover stays stopped until a real
    // movement command arrives, and if the obstacle is still there that
    // command will simply be clamped again.
    gSafetyStopLatched = false;
    gTimedOut          = false;

    // NOTE: does NOT refresh gLastCommandMs. RESET is not a movement command,
    // so it must not hold the failsafe open. gTimedOut will re-assert on the
    // next loop pass unless a real command arrives.
    commUpdateRuntime(0, 0, "NONE");

    Serial.println("{\"ack\":\"RESET\",\"accepted\":true}");
}


static void handlePing(void)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"ack\":\"PING\",\"accepted\":true,\"state\":\"%s\","
             "\"uptime_ms\":%lu}",
             robotStateName(gState), (unsigned long)millis());
    Serial.println(buf);
}


// ============================================================================
//  DIAGNOSTIC / COMMISSIONING COMMANDS
// ============================================================================
//
// These exist to answer the hardware questions this firmware refuses to guess
// at: which address is the multiplexer, which is the PWM driver, is each
// VL53L0X alive, and which physical position is each one.
//
// SAFETY PROPERTIES SHARED BY ALL OF THEM:
//   * None can turn a wheel. Not one of them touches the motor API.
//   * None refreshes the movement watchdog. A stream of diagnostics cannot
//     hold the failsafe open while the rover is meant to be stopping.
//   * All are bounded in time. The longest is the I2C scan, ~112 address
//     probes, a few milliseconds in total.
//   * None writes to an address the operator did not name in the command.
//
// Compiled out entirely by DIAGNOSTICS_ENABLED=0 in config.h.
// ============================================================================

#if DIAGNOSTICS_ENABLED

// Last scan result, remembered so I2CSTATUS can report it without re-running a
// scan. On a stuck bus a scan costs ~1 second PER ADDRESS, so re-running one
// just to fill in a status field would be a two-minute command.
static uint32_t gLastScanMs      = 0;
static uint8_t  gLastScanCount   = 0;
static bool     gLastScanDone    = false;
static uint32_t gLastScanPerAddr = 0;

// Parse an address that may be written as 0x70 or as 112. strtol with base 0
// handles both, and anything outside the probeable range is refused.
static bool parseAddress(const char *line, const char *key, uint8_t *out)
{
    const char *p = findValue(line, key);
    if (p == NULL) {
        return false;
    }
    if (*p != '-' && *p != '+' && !isdigit((unsigned char)*p)) {
        return false;
    }

    long v = strtol(p, NULL, 0);        // base 0 -> accepts 0x.. and decimal

    if (v < I2C_SCAN_FIRST_ADDR || v > I2C_SCAN_LAST_ADDR) {
        return false;
    }

    *out = (uint8_t)v;
    return true;
}


// ---------------------------------------------------------------------------
// I2C BUS SCAN  --  the one command that is meant to be run first.
//
// Emitted by BOTH {"cmd":"I2CSCAN"} and the one-shot boot scan, from this
// single function, so the two can never disagree about format or behaviour.
//
// WHAT IT DOES TO THE BUS: nothing. roverI2cProbe() sends START, the 7-bit
// address, and STOP. NO DATA BYTE IS EVER WRITTEN, to any address, responding
// or not. That is the standard non-destructive presence test and it cannot
// change the state of any device on the bus.
//
// The reserved blocks below 0x08 and above 0x77 are never touched.
//
// READ THE HINTS AS HINTS. A scan proves an address responded; it does not
// identify a chip. 0x70..0x77 is shared territory between a TCA9548A and a
// fully-jumpered PCA9685. The scanner does NOT assume 0x70 is a multiplexer
// or 0x40 is a PWM driver -- it reports what answered and says what the
// candidates are.
//
// EXPECT 0x29 TO BE ABSENT. The three VL53L0X sit behind the multiplexer and
// this scan sees only the main segment. Seeing 0x29 here would mean a sensor
// is wired directly to the main bus, contradicting the confirmed topology --
// which is itself worth knowing, so it is reported rather than filtered out.
// ---------------------------------------------------------------------------
static void emitI2cScan(const char *trigger)
{
    // ------------------------------------------------------------------
    // Ensure no multiplexer channel is open, so what we report is the MAIN
    // segment and not something downstream masquerading as a main-bus device.
    //
    // NOTE ON THE "NO WRITES" PROPERTY: closing the channels IS a one-byte
    // write, and it is the only write anywhere in this function. Today it does
    // not happen at all -- the multiplexer address is unconfirmed, so
    // tcaDeselectAll() returns false without touching the bus. It begins
    // happening only once YOU set TCA9548A_ADDRESS_CONFIRMED, at which point
    // writing 0x00 to a multiplexer you have identified is both safe and
    // necessary. Either way the result is reported below rather than hidden.
    // ------------------------------------------------------------------
    bool muxDeselected = tcaDeselectAll();

    uint32_t startMs = millis();

    uint8_t found[24];
    uint8_t n = roverI2cScan(found, (uint8_t)(sizeof(found) / sizeof(found[0])));

    uint32_t elapsedMs = millis() - startMs;

    // Remembered so I2CSTATUS can report them without re-running a scan that
    // may take a very long time on a stuck bus.
    gLastScanMs      = elapsedMs;
    gLastScanCount   = n;
    gLastScanDone    = true;
    gLastScanPerAddr = elapsedMs / (uint32_t)I2C_SCAN_ADDR_COUNT;

    // "state" is the field the Pi-side parser keys on. "event" is carried too
    // because every other asynchronous line this firmware emits uses it, and a
    // parser that switches on one should not have to special-case the other.
    Serial.printf("{\"state\":\"I2C_SCAN\",\"event\":\"I2CSCAN\","
                  "\"trigger\":\"%s\",\"sda\":%d,\"scl\":%d,\"clock_hz\":%lu,"
                  "\"scan_ms\":%lu,\"bus_writes\":%s,\"count\":%u,"
                  "\"devices\":[",
                  trigger,
                  (int)I2C_SDA_PIN, (int)I2C_SCL_PIN,
                  (unsigned long)I2C_CLOCK_HZ,
                  (unsigned long)elapsedMs,
                  muxDeselected ? "\"mux_deselect_only\"" : "\"none\"",
                  (unsigned)n);

    for (uint8_t i = 0; i < n; i++) {
        // "address" is the documented field name. "dec" and "hint" are extra,
        // never a substitute.
        Serial.printf("%s{\"address\":\"0x%02X\",\"dec\":%u,\"hint\":\"%s\"}",
                      (i ? "," : ""), found[i], (unsigned)found[i],
                      roverI2cAddressHint(found[i]));
    }

    Serial.printf("],\"tca_confirmed\":%s,\"pca_confirmed\":%s,"
                  "\"note\":\"hints are candidates, NOT identification. "
                  "0x70-0x77 could be either a TCA9548A or a fully-jumpered "
                  "PCA9685. 0x29 should NOT appear - the VL53L0X are behind "
                  "the mux.\"}\n",
                  tcaAddressConfirmed() ? "true" : "false",
                  pca9685AddressConfirmed() ? "true" : "false");
}


// One-shot boot scan. Called from setup(), never from loop().
void commBootI2cScan(void)
{
    if (!roverI2cReady()) {
        Serial.println("{\"state\":\"I2C_SCAN\",\"event\":\"I2CSCAN\","
                       "\"trigger\":\"boot\",\"error\":\"I2C_NOT_READY\","
                       "\"count\":0,\"devices\":[]}");
        return;
    }

    emitI2cScan("boot");
}


// ---------------------------------------------------------------------------
// {"cmd":"I2CSCAN"}
// ---------------------------------------------------------------------------
static void handleI2cScan(void)
{
    if (!roverI2cReady()) {
        ackRejected("I2CSCAN", "I2C_NOT_READY");
        return;
    }

#if I2C_SCAN_REQUIRE_STOPPED
    // A scan blocks the control loop for tens of milliseconds -- no ultrasonic
    // ping and no safety-gate re-evaluation for that window. Acceptable on a
    // stationary rover, not on a moving one.
    //
    // This cannot trigger today, because nothing can turn a wheel while the
    // PCA9685 address is unconfirmed. It is here so the rule still holds later.
    if (gAppliedLeft != 0 || gAppliedRight != 0) {
        ackRejected("I2CSCAN", "REFUSED_ROVER_IS_MOVING_SEND_STOP_FIRST");
        return;
    }
#endif

    emitI2cScan("command");
}


// ---------------------------------------------------------------------------
// {"cmd":"I2CSTATUS"}
//
// Reports the PHYSICAL state of the bus without using it.
//
// This is the command to run when a scan comes back empty, because it answers
// the question a scan cannot: is the bus idle and simply unpopulated, or is it
// electrically stuck?
//
// SAFETY. It performs no I2C transaction, changes no pin mode, writes to no
// device, assumes no address, and touches nothing motor-related. It reads two
// GPIO input registers sixteen times and prints the result. It is safe to run
// with everything powered, and it cannot disturb a device mid-transfer because
// it does not transfer.
//
// HOW TO READ IT. Wire.begin() enables the ESP32's internal pull-ups on both
// lines, and an idle master releases both. So both lines MUST read HIGH unless
// something external is holding one down.
// ---------------------------------------------------------------------------
static void handleI2cStatus(void)
{
    RoverI2cLineState lines;
    roverI2cSampleLines(&lines);

    bool eitherLow = (!lines.sdaHigh || !lines.sclHigh ||
                      lines.sdaStuckLow || lines.sclStuckLow);

    Serial.printf("{\"state\":\"I2C_STATUS\",\"event\":\"I2CSTATUS\""
                  ",\"i2c_initialized\":%s"
                  ",\"sda_pin\":%d,\"scl_pin\":%d,\"clock_hz\":%lu"
                  ",\"internal_pullups\":\"ENABLED_BY_CORE\"",
                  roverI2cReady() ? "true" : "false",
                  (int)I2C_SDA_PIN, (int)I2C_SCL_PIN,
                  (unsigned long)I2C_CLOCK_HZ);

    Serial.printf(",\"sda_level\":\"%s\",\"scl_level\":\"%s\""
                  ",\"sda_high_samples\":%u,\"scl_high_samples\":%u"
                  ",\"samples\":%u"
                  ",\"sda_stuck_low\":%s,\"scl_stuck_low\":%s"
                  ",\"either_line_low\":%s,\"bus_idle\":%s",
                  lines.sdaHigh ? "HIGH" : "LOW",
                  lines.sclHigh ? "HIGH" : "LOW",
                  (unsigned)lines.sdaHighSamples,
                  (unsigned)lines.sclHighSamples,
                  (unsigned)lines.sampleCount,
                  lines.sdaStuckLow ? "true" : "false",
                  lines.sclStuckLow ? "true" : "false",
                  eitherLow ? "true" : "false",
                  (!eitherLow) ? "true" : "false");

    // Last scan, recalled rather than re-run. On a stuck bus a fresh scan
    // costs about a second per address.
    if (gLastScanDone) {
        bool stuckTiming = (gLastScanPerAddr >= I2C_SCAN_STUCK_MS_PER_ADDR);

        Serial.printf(",\"last_scan\":{\"done\":true,\"count\":%u"
                      ",\"total_ms\":%lu,\"ms_per_address\":%lu"
                      ",\"addresses_probed\":%d,\"timing_indicates\":\"%s\"}",
                      (unsigned)gLastScanCount,
                      (unsigned long)gLastScanMs,
                      (unsigned long)gLastScanPerAddr,
                      (int)I2C_SCAN_ADDR_COUNT,
                      stuckTiming ? "STUCK_BUS" : "NORMAL_NACK_TIMING");
    } else {
        Serial.print(",\"last_scan\":{\"done\":false}");
    }

    Serial.printf(",\"verdict\":\"%s\"", roverI2cLineVerdict(&lines));

    Serial.println(",\"note\":\"An idle master releases both lines and the "
                   "core enables internal pull-ups, so both SHOULD read HIGH. "
                   "A LOW line is held down by something outside the ESP32. "
                   "This command performs no I2C transaction and writes to no "
                   "device.\"}");
}


// ---------------------------------------------------------------------------
// {"cmd":"TCATEST","addr":"0x70"}      -- test a candidate address
// {"cmd":"TCATEST"}                    -- test the configured address, and if
//                                         it is confirmed, exercise CH0/1/2
//
// THIS IS THE COMMAND THAT ANSWERS THE MULTIPLEXER ADDRESS QUESTION.
//
// It writes a channel mask and reads the control register back. A TCA9548A
// returns exactly what you wrote -- that is its only register. A PCA9685 at
// the same address does not, because a single-byte read from it returns
// whatever its register pointer is on, not a mirror of your write.
//
// It always leaves the candidate with all channels closed, whatever happens,
// so it cannot leave a VL53L0X hanging on the main bus.
// ---------------------------------------------------------------------------
static void handleTcaTest(const char *line)
{
    if (!roverI2cReady()) {
        ackRejected("TCATEST", "I2C_NOT_READY");
        return;
    }

    uint8_t addr = 0;
    bool haveAddr = parseAddress(line, "addr", &addr);

    if (!haveAddr) {
        if (!tcaAddressConfirmed()) {
            ackRejected("TCATEST", "NO_ADDR_GIVEN_AND_NONE_CONFIRMED");
            return;
        }
        addr = tcaAddress();
    }

    // 0x01 = channel 0 only. A single bit, so even if this IS the multiplexer
    // we open exactly one segment and never two.
    const uint8_t testMask = 0x01;

    // Probe BEFORE the read-back test, and capture both results into locals.
    // Doing these calls inside the printf argument list would leave their
    // relative order up to the compiler, and each one overwrites the shared
    // last-error state -- so the reported error could belong to either call.
    bool responded = roverI2cProbe(addr);

    uint8_t readBack = 0;
    bool looksLikeTca = tcaProbeCandidate(addr, testMask, &readBack);
    const char *errName = roverI2cErrorName(roverI2cLastError());

    Serial.printf("{\"event\":\"TCATEST\",\"addr\":\"0x%02X\","
                  "\"responded\":%s,\"wrote\":\"0x%02X\","
                  "\"read_back\":\"0x%02X\",\"looks_like_tca9548a\":%s,"
                  "\"i2c_error\":\"%s\"",
                  addr,
                  responded ? "true" : "false",
                  testMask, readBack,
                  looksLikeTca ? "true" : "false",
                  errName);

    if (looksLikeTca && !tcaAddressConfirmed()) {
        Serial.printf(",\"action\":\"set TCA9548A_I2C_ADDRESS to 0x%02X and "
                      "TCA9548A_ADDRESS_CONFIRMED to 1 in config.h, then "
                      "reflash\"", addr);
    } else if (!looksLikeTca) {
        Serial.print(",\"action\":\"read-back did not match - this address is "
                     "probably NOT a TCA9548A\"");
    }

    Serial.println("}");
}


// ---------------------------------------------------------------------------
// {"cmd":"PCATEST","addr":"0x40"}      -- probe a candidate address
// {"cmd":"PCATEST"}                    -- report the configured one
//
// READ ONLY. It reads MODE1 and PRESCALE and writes nothing, so pointing it at
// the wrong address cannot disturb whatever actually lives there.
//
// A PCA9685 that has just powered up and has NOT been initialised reads
// MODE1 = 0x11 (SLEEP | ALLCALL) and PRESCALE = 0x1E. One this firmware has
// already initialised reads MODE1 with SLEEP clear and AI set, and a PRESCALE
// matching PCA9685_PWM_FREQ_HZ. Neither is proof, and the report says so.
// ---------------------------------------------------------------------------
static void handlePcaTest(const char *line)
{
    if (!roverI2cReady()) {
        ackRejected("PCATEST", "I2C_NOT_READY");
        return;
    }

    uint8_t addr = 0;
    bool haveAddr = parseAddress(line, "addr", &addr);

    if (!haveAddr) {
        if (!pca9685AddressConfirmed()) {
            ackRejected("PCATEST", "NO_ADDR_GIVEN_AND_NONE_CONFIRMED");
            return;
        }
        addr = pca9685Address();
    }

    // The PCA9685 is on the MAIN bus segment. Close any mux channel first so
    // a downstream device cannot answer in its place.
    (void)tcaDeselectAll();

    uint8_t mode1 = 0;
    uint8_t prescale = 0;
    bool readable = pca9685ProbeCandidate(addr, &mode1, &prescale);
    const char *errName = roverI2cErrorName(roverI2cLastError());

    // What frequency that prescale corresponds to, so the operator can see at
    // a glance whether this chip is running at the configured rate.
    unsigned long impliedHz = 0;
    if (readable && prescale > 0) {
        impliedHz = (unsigned long)(PCA9685_OSC_HZ /
                                    (4096UL * ((unsigned long)prescale + 1UL)));
    }

    Serial.printf("{\"event\":\"PCATEST\",\"addr\":\"0x%02X\","
                  "\"responded\":%s,\"mode1\":\"0x%02X\","
                  "\"prescale\":\"0x%02X\",\"implied_freq_hz\":%lu,"
                  "\"configured_freq_hz\":%d,\"i2c_error\":\"%s\","
                  "\"status\":\"%s\"",
                  addr, readable ? "true" : "false",
                  mode1, prescale, impliedHz,
                  (int)PCA9685_PWM_FREQ_HZ,
                  errName,
                  pca9685StatusName());

    if (readable && !pca9685AddressConfirmed()) {
        Serial.printf(",\"action\":\"if this is the PCA9685, set "
                      "PCA9685_I2C_ADDRESS to 0x%02X and "
                      "PCA9685_ADDRESS_CONFIRMED to 1 in config.h, then "
                      "reflash\",\"caution\":\"a readable MODE1/PRESCALE pair "
                      "is consistent with a PCA9685 but does not prove one - "
                      "confirm against the board's solder jumpers\"", addr);
    }

    Serial.println("}");
}


// ---------------------------------------------------------------------------
// {"cmd":"TOFTEST"}            -- test all three rear sensors
// {"cmd":"TOFTEST","sensor":1} -- test just one
//
// Selects each sensor's TCA channel in turn and takes ONE fresh measurement.
// Blocking, bench use only -- the control loop uses the non-blocking
// round-robin instead.
//
// THIS IS HOW YOU RESOLVE THE ORIENTATION QUESTION. Put a hand ~200 mm behind
// ONE sensor, run TOFTEST, and note which index reports the short distance.
// Repeat for the other two. That tells you which TCA channel is physically
// left, centre and right -- which is the last unknown in the rear subsystem.
// ---------------------------------------------------------------------------
static void handleTofTest(const char *line)
{
    if (!tcaAddressConfirmed()) {
        ackRejected("TOFTEST", "TCA_ADDRESS_UNCONFIRMED");
        return;
    }
    if (!tcaPresent()) {
        ackRejected("TOFTEST", "TCA_NOT_FOUND");
        return;
    }

    int only = -1;
    if (parseInt(line, "sensor", &only)) {
        if (only < 0 || only >= REAR_TOF_SENSOR_COUNT) {
            ackRejected("TOFTEST", "BAD_SENSOR_INDEX");
            return;
        }
    }

    Serial.printf("{\"event\":\"TOFTEST\",\"tca_addr\":\"0x%02X\","
                  "\"orientation_verified\":%s,\"results\":[",
                  tcaAddress(),
                  rearTofOrientationVerified() ? "true" : "false");

    bool first = true;

    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (only >= 0 && (int)i != only) {
            continue;
        }

        uint16_t mm = 0;
        RearTofStatus st = rearTofTestOne(i, &mm);

        Serial.printf("%s{\"sensor\":%u,\"tca_channel\":%u,\"status\":\"%s\"",
                      first ? "" : ",", (unsigned)i,
                      (unsigned)rearTofChannelOf(i),
                      rearTofStatusName(st));

        // A distance is printed ONLY for a real measurement. OUT_OF_RANGE also
        // carries its raw number, explicitly labelled, so the operator can see
        // the ~8190 that means "nothing there" instead of wondering why the
        // field is missing.
        if (st == TOF_VALID) {
            Serial.printf(",\"mm\":%u", (unsigned)mm);
        } else if (st == TOF_OUT_OF_RANGE) {
            Serial.printf(",\"mm\":null,\"raw_mm_not_a_measurement\":%u",
                          (unsigned)mm);
        } else {
            Serial.print(",\"mm\":null");
        }

        Serial.print("}");
        first = false;
    }

    Serial.println("],\"note\":\"index is TCA channel order, NOT physical "
                   "left/centre/right\"}");
}


// ---------------------------------------------------------------------------
// {"cmd":"HWREPORT"}
//
// Dumps what the firmware believes about the hardware and, more usefully,
// what it does NOT know. One command to paste into a bug report or a
// commissioning log.
// ---------------------------------------------------------------------------
static void handleHwReport(void)
{
    Serial.printf("{\"event\":\"HWREPORT\""
                  ",\"core\":\"2.0.14\""
                  ",\"i2c\":{\"sda\":%d,\"scl\":%d,\"clock_hz\":%lu,"
                  "\"ready\":%s}",
                  (int)I2C_SDA_PIN, (int)I2C_SCL_PIN,
                  (unsigned long)I2C_CLOCK_HZ,
                  roverI2cReady() ? "true" : "false");

    Serial.printf(",\"tca9548a\":{\"address\":\"0x%02X\",\"confirmed\":%s,"
                  "\"present\":%s,\"status\":\"%s\","
                  "\"rear_channels\":[%d,%d,%d]}",
                  tcaAddress(),
                  tcaAddressConfirmed() ? "true" : "false",
                  tcaPresent() ? "true" : "false",
                  tcaStatusName(),
                  (int)TCA_CH_REAR_TOF_0, (int)TCA_CH_REAR_TOF_1,
                  (int)TCA_CH_REAR_TOF_2);

    Serial.printf(",\"pca9685\":{\"address\":\"0x%02X\",\"confirmed\":%s,"
                  "\"ready\":%s,\"status\":\"%s\",\"freq_hz\":%d,"
                  "\"enable_channels\":[%d,%d,%d,%d]}",
                  pca9685Address(),
                  pca9685AddressConfirmed() ? "true" : "false",
                  pca9685Ready() ? "true" : "false",
                  pca9685StatusName(),
                  (int)PCA9685_PWM_FREQ_HZ,
                  (int)PCA_CH_FRONT_A_EN, (int)PCA_CH_FRONT_B_EN,
                  (int)PCA_CH_REAR_A_EN,  (int)PCA_CH_REAR_B_EN);

    Serial.print(",\"motors\":[");
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        Serial.printf("%s{\"ch\":%d,\"name\":\"%s\",\"in1_gpio\":%u,"
                      "\"in2_gpio\":%u,\"enable_pca_ch\":%u,"
                      "\"configured_side\":\"%s\"}",
                      (i ? "," : ""), i, motorChannelName(i),
                      (unsigned)motorChannelIn1Pin(i),
                      (unsigned)motorChannelIn2Pin(i),
                      (unsigned)motorChannelEnableCh(i),
                      motorChannelSideName(i));
    }
    Serial.print("]");

    Serial.printf(",\"front_ultrasonic\":{\"a\":{\"trig\":%d,\"echo\":%d},"
                  "\"b\":{\"trig\":%d,\"echo\":%d}}",
                  (int)US_A_TRIG_PIN, (int)US_A_ECHO_PIN,
                  (int)US_B_TRIG_PIN, (int)US_B_ECHO_PIN);

    Serial.printf(",\"thresholds\":{\"front_stop_cm\":%d,\"front_clear_cm\":%d,"
                  "\"front_warn_cm\":%d,\"rear_stop_mm\":%d,"
                  "\"rear_clear_mm\":%d,\"rear_warn_mm\":%d,"
                  "\"rear_thresholds_are_commissioning_values\":true}",
                  (int)SAFETY_STOP_DISTANCE_CM, (int)SAFETY_CLEAR_DISTANCE_CM,
                  (int)SAFETY_WARN_DISTANCE_CM,
                  (int)REAR_STOP_DISTANCE_MM, (int)REAR_CLEAR_DISTANCE_MM,
                  (int)REAR_WARN_DISTANCE_MM);

    // The honest part. Everything a human still has to go and look at.
    Serial.print(",\"unverified\":[");
    bool first = true;
    #define UNV(cond, text) do { if (cond) { \
            Serial.printf("%s\"%s\"", first ? "" : ",", text); first = false; } \
        } while (0)

    UNV(!tcaAddressConfirmed(),        "TCA9548A I2C address");
    UNV(!pca9685AddressConfirmed(),    "PCA9685 I2C address");
    UNV(!motorMapVerified(),           "motor left/right side mapping");
    UNV(!motorMapVerified(),           "motor rotation polarity");
    UNV(!sensorMapVerified(),          "front ultrasonic left/right mapping");
    UNV(!rearTofOrientationVerified(), "rear VL53L0X physical left/centre/right");
    UNV(true,                          "rear obstacle thresholds (commissioning values)");
    UNV(true,                          "PCA9685 OE pin disposition");
    UNV(true,                          "TCA9548A RESET pin disposition");
    UNV(true,                          "ultrasonic distance calibration");
    #undef UNV
    Serial.println("]}");
}

#else  // !DIAGNOSTICS_ENABLED

// Diagnostics compiled out. The boot scan still exists, as a no-op, so setup()
// does not have to repeat the guard and cannot fall out of step with it.
void commBootI2cScan(void) { }

#endif // DIAGNOSTICS_ENABLED


// ============================================================================
// DISPATCH
// ============================================================================

void handleCommand(const char *line)
{
    if (line == NULL || line[0] == '\0') {
        return;
    }

    // Any recognised command cancels an in-progress motor test.
    if (gTestActive && !stringFieldIs(line, "cmd", "MOTORTEST")) {
        gTestActive = false;
        stopMotors();
    }

    if (stringFieldIs(line, "cmd", "STOP")) {
        handleStop();
    } else if (stringFieldIs(line, "cmd", "DRIVE")) {
        handleDrive(line);
    } else if (stringFieldIs(line, "cmd", "MOVE")) {
        handleMove(line);
    } else if (stringFieldIs(line, "cmd", "MOTORTEST")) {
        handleMotorTest(line);
    } else if (stringFieldIs(line, "cmd", "RESET")) {
        handleReset();
    } else if (stringFieldIs(line, "cmd", "PING")) {
        handlePing();
#if DIAGNOSTICS_ENABLED
    // Diagnostics. None of these can turn a wheel, and none refreshes the
    // movement watchdog -- a stream of them cannot hold the failsafe open.
    } else if (stringFieldIs(line, "cmd", "I2CSCAN")) {
        handleI2cScan();
    } else if (stringFieldIs(line, "cmd", "I2CSTATUS")) {
        handleI2cStatus();
    } else if (stringFieldIs(line, "cmd", "TCATEST")) {
        handleTcaTest(line);
    } else if (stringFieldIs(line, "cmd", "PCATEST")) {
        handlePcaTest(line);
    } else if (stringFieldIs(line, "cmd", "TOFTEST")) {
        handleTofTest(line);
    } else if (stringFieldIs(line, "cmd", "HWREPORT")) {
        handleHwReport();
#endif
    } else {
        // Unknown input does NOT refresh the command timeout. Serial noise
        // must never keep the failsafe alive while the motors hold speed.
        ackRejected("ERROR", "UNKNOWN_COMMAND");
    }
}


// ============================================================================
// NON-BLOCKING LINE READER
// ============================================================================
//
// Replaces Serial.readStringUntil('\n'), which blocks for up to 1000 ms on a
// partial line and would stall the safety loop.
// ============================================================================

void commPoll(void)
{
    // Expire a running motor test even if no bytes arrive.
    if (gTestActive && (int32_t)(millis() - gTestDeadline) >= 0) {
        gTestActive = false;
        stopMotors();
        commUpdateRuntime(0, 0, "NONE");
        Serial.println("{\"event\":\"MOTORTEST_DONE\"}");
    }

    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            if (gOverflow) {
                gOverflow = false;
                gLineLen  = 0;
                ackRejected("ERROR", "LINE_TOO_LONG");
                continue;
            }
            if (gLineLen > 0) {
                gLine[gLineLen] = '\0';
                handleCommand(gLine);
                gLineLen = 0;
            }
            continue;
        }

        if (gLineLen < (COMM_LINE_MAX - 1)) {
            gLine[gLineLen++] = c;
        } else {
            // Discard the rest of an over-long line rather than acting on a
            // truncated, possibly meaningless command.
            gOverflow = true;
            gLineLen  = 0;
        }
    }
}


// ============================================================================
// TELEMETRY
// ============================================================================
//
// One JSON object per line, assembled into a single buffer and written once.
// Forty separate Serial.print() calls cost far more than one write, and a
// partial line is impossible this way.
//
// FLOAT FORMATTING: distances are rendered with integer arithmetic, not
// "%f". This avoids depending on whether the toolchain links full or nano
// printf, which is a real difference between ESP-IDF configurations.
//
// NON-MEASUREMENTS are emitted as JSON null, never as -1 and never as a
// fabricated distance.
// ============================================================================

static char gTlmBuf[TELEMETRY_BUF_SIZE];
static int  gTlmPos      = 0;
static bool gTlmOverflow = false;

static void tlm(const char *fmt, ...)
{
    if (gTlmOverflow) return;

    int rem = (int)sizeof(gTlmBuf) - gTlmPos;
    if (rem <= 1) {
        gTlmOverflow = true;
        return;
    }

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(gTlmBuf + gTlmPos, (size_t)rem, fmt, ap);
    va_end(ap);

    if (n < 0 || n >= rem) {
        gTlmOverflow = true;
        return;
    }

    gTlmPos += n;
}

// A distance field, or JSON null when the value is not a real measurement.
static void tlmCm(const char *key, float cm)
{
    if (cm > 0.0f) {
        long t = (long)(cm * 10.0f + 0.5f);
        tlm(",\"%s\":%ld.%ld", key, t / 10L, t % 10L);
    } else {
        tlm(",\"%s\":null", key);
    }
}

// A millimetre field, or JSON null when the value is not a real measurement.
// Same contract as tlmCm(): REAR_TOF_INVALID_MM (negative) and anything else
// non-positive becomes null, never a number. There is no path here that can
// publish 0 mm, which would read as an obstacle touching the sensor.
static void tlmMm(const char *key, int mm)
{
    if (mm > 0) {
        tlm(",\"%s\":%d", key, mm);
    } else {
        tlm(",\"%s\":null", key);
    }
}

static void tlmBool(const char *key, bool v)
{
    tlm(",\"%s\":%s", key, v ? "true" : "false");
}

void sendTelemetry(void)
{
    gTlmPos      = 0;
    gTlmOverflow = false;
    gTlmBuf[0]   = '\0';

    // ---- State machine -----------------------------------------------------
    tlm("{\"state\":\"%s\"", robotStateName(gState));
    tlm(",\"block_reason\":\"%s\"", gBlockReason);
    tlm(",\"last_reject\":\"%s\"", gLastRejectReason);

    // ---- Commanded vs applied motion --------------------------------------
    // left_cmd / right_cmd are what the Pi asked for (unchanged field names).
    // left_applied / right_applied are what the motors actually received
    // after the safety gate -- these differ during a component block.
    tlm(",\"left_cmd\":%d,\"right_cmd\":%d", gDesiredLeft, gDesiredRight);
    tlm(",\"left_applied\":%d,\"right_applied\":%d", gAppliedLeft, gAppliedRight);

    // ---- FRONT ultrasonic: values, validity, health ------------------------
    // Side labels come from SENSOR_MAP_SWAP in config.h, which is UNVERIFIED:
    // see "sensor_map_verified" below before trusting which is which.
    // These ARE distance sensors, so centimetres are legitimate -- but they
    // are UNCALIBRATED centimetres. Validate against a tape measure.
    tlmCm("front_left_cm",  sensorFilteredCm(US_FRONT_LEFT));
    tlmCm("front_right_cm", sensorFilteredCm(US_FRONT_RIGHT));

    tlmBool("front_left_valid",  sensorValid(US_FRONT_LEFT));
    tlmBool("front_right_valid", sensorValid(US_FRONT_RIGHT));

    // Aggregate: is forward sensing trustworthy under the configured policy?
    // The inverse of the sensor-fault condition, NOT a simple AND/OR of the
    // two flags, so it always agrees with what the gate will do.
    tlmBool("front_valid", !safetySensorFault());

    tlm(",\"front_left_health\":\"%s\"",
        sensorHealthName(sensorHealthOf(US_FRONT_LEFT)));
    tlm(",\"front_right_health\":\"%s\"",
        sensorHealthName(sensorHealthOf(US_FRONT_RIGHT)));
    tlm(",\"sensor_health\":\"%s\"", sensorHealthName(sensorHealthWorst()));

    tlmCm("front_closest_cm", safetyClosestFrontCm());

    // ---- REAR time-of-flight sensors (3 x VL53L0X) -------------------------
    //
    // THREE PHYSICAL SENSORS, REPORTED INDEPENDENTLY. There is deliberately no
    // single "rear distance" field: the three sensors cover three different
    // cones with no guaranteed relationship, and collapsing them into one
    // number would throw away exactly the information the Pi needs to decide
    // WHICH WAY to turn out of trouble. rear_closest_mm exists as a
    // convenience, clearly labelled as the minimum of the three.
    //
    // A non-measurement is JSON null, never 0 and never a stale value. The
    // per-sensor status field says WHICH kind of non-measurement it was --
    // UNINITIALISED / INIT_FAILED / BUS_ERROR / TIMEOUT / OUT_OF_RANGE /
    // STALE -- because those are six different problems with six different
    // fixes, and "no reading" alone tells the operator nothing.
    tlm(",\"rear_sensor_count\":%d", (int)REAR_TOF_SENSOR_COUNT);
    tlm(",\"rear_backend\":\"%s\"", rearBackendName());
    tlmBool("rear_available", safetyRearSensingAvailable());
    tlmBool("rear_orientation_verified", rearTofOrientationVerified());

    // Individual readings. These field names are flat and indexed rather than
    // arrayed so a Pi-side parser can pick one out without positional
    // assumptions. Index N is TCA CHANNEL N -- it is NOT left/centre/right,
    // which is still unverified (see rear_orientation_verified above).
    tlmMm("rear_sensor_0_mm", safetyRearDistanceMm(REAR_TOF_0));
    tlmMm("rear_sensor_1_mm", safetyRearDistanceMm(REAR_TOF_1));
    tlmMm("rear_sensor_2_mm", safetyRearDistanceMm(REAR_TOF_2));

    tlmBool("rear_sensor_0_valid", safetyRearSensorValid(REAR_TOF_0));
    tlmBool("rear_sensor_1_valid", safetyRearSensorValid(REAR_TOF_1));
    tlmBool("rear_sensor_2_valid", safetyRearSensorValid(REAR_TOF_2));

    tlm(",\"rear_sensor_0_status\":\"%s\"",
        rearTofStatusName(rearTofStatusOf(REAR_TOF_0)));
    tlm(",\"rear_sensor_1_status\":\"%s\"",
        rearTofStatusName(rearTofStatusOf(REAR_TOF_1)));
    tlm(",\"rear_sensor_2_status\":\"%s\"",
        rearTofStatusName(rearTofStatusOf(REAR_TOF_2)));

    tlm(",\"rear_sensor_0_health\":\"%s\"",
        sensorHealthName(rearTofHealthOf(REAR_TOF_0)));
    tlm(",\"rear_sensor_1_health\":\"%s\"",
        sensorHealthName(rearTofHealthOf(REAR_TOF_1)));
    tlm(",\"rear_sensor_2_health\":\"%s\"",
        sensorHealthName(rearTofHealthOf(REAR_TOF_2)));

    // Per-sensor LATCHED obstacle flags, after hysteresis and confirmation.
    // These are what the reverse gate acts on -- not the raw distances above.
    tlmBool("rear_sensor_0_obstacle", safetyRearSensorObstacle(REAR_TOF_0));
    tlmBool("rear_sensor_1_obstacle", safetyRearSensorObstacle(REAR_TOF_1));
    tlmBool("rear_sensor_2_obstacle", safetyRearSensorObstacle(REAR_TOF_2));

    tlm(",\"rear_health\":\"%s\"", sensorHealthName(rearTofHealthWorst()));
    tlmMm("rear_closest_mm", safetyClosestRearMm());

    tlmBool("rear_obstacle",     safetyRearObstacleDetected());
    tlmBool("rear_warning",      safetyRearWarning());
    tlmBool("rear_sensor_fault", safetyRearSensorFault());
    tlmBool("reverse_guarded",   safetyRearSensingAvailable());

    // ---- I2C subsystem health ---------------------------------------------
    // The Pi needs to be able to see WHY the rover will not move or will not
    // reverse, without a serial console attached to the ESP32.
    tlmBool("i2c_ready", roverI2cReady());
    tlm(",\"tca_status\":\"%s\"", tcaStatusName());
    tlm(",\"pca_status\":\"%s\"", pca9685StatusName());
    tlmBool("tca_address_confirmed", tcaAddressConfirmed());
    tlmBool("pca_address_confirmed", pca9685AddressConfirmed());

    // ---- Motor drive availability -----------------------------------------
    // FALSE means no wheel can turn, whatever the Pi sends. The enable lines
    // are fed by the PCA9685, so without a verified PCA9685 there is no speed
    // path at all. The Pi should treat this exactly like a hardware fault.
    tlmBool("motor_drive_available", motorDriveAvailable());
    tlm(",\"motor_drive_status\":\"%s\"", motorDriveStatusName());

    // ---- Safety ------------------------------------------------------------
    tlmBool("obstacle",         safetyObstacleDetected());   // front, confirmed
    tlmBool("front_obstacle",   safetyObstacleDetected());   // explicit name
    tlmBool("warning",          safetyWarning());
    tlmBool("sensor_fault",     safetySensorFault());
    tlmBool("forward_blocked",  safetyForwardBlocked());
    tlmBool("reverse_blocked",  safetyReverseBlocked());

    // LATCHED. Stays true after a safety stop even if the obstacle vanishes
    // or the state moves on. Cleared only by RESET or a new fully-permitted
    // movement command -- never by the obstacle leaving.
    tlmBool("safety_stop", commSafetyStopLatched());

    // ---- Command failsafe --------------------------------------------------
    tlm(",\"command_age_ms\":%lu", (unsigned long)commMsSinceLastCommand());
    tlmBool("command_timeout",        commTimedOut());
    tlmBool("command_ever_received",  gEverReceivedCmd);

    // ---- Verification flags ------------------------------------------------
    // Both are false until a HUMAN confirms them on the real rover. The
    // firmware has no way to discover either by itself and will not claim to.
    tlmBool("motor_map_verified",  motorMapVerified());
    tlmBool("sensor_map_verified", sensorMapVerified());

    // ---- Debug / calibration aids -----------------------------------------
    // Raw = the single most recent sample, unfiltered. Useful for watching
    // dropout rate during calibration. Never use it for control.
    // Compiled out by TELEMETRY_INCLUDE_DEBUG=0 to halve the link budget.
#if TELEMETRY_INCLUDE_DEBUG
    tlmCm("front_left_raw_cm",  sensorRawCm(US_FRONT_LEFT));
    tlmCm("front_right_raw_cm", sensorRawCm(US_FRONT_RIGHT));

    tlm(",\"front_left_samples\":%u",  (unsigned)sensorGoodSampleCount(US_FRONT_LEFT));
    tlm(",\"front_right_samples\":%u", (unsigned)sensorGoodSampleCount(US_FRONT_RIGHT));

    tlm(",\"front_left_timeout_streak\":%u",
        (unsigned)sensorTimeoutStreak(US_FRONT_LEFT));
    tlm(",\"front_right_timeout_streak\":%u",
        (unsigned)sensorTimeoutStreak(US_FRONT_RIGHT));

    // Rear raw readings, unfiltered. Watch these during commissioning to see
    // the dropout rate and, on an OUT_OF_RANGE sensor, the ~8190 that the part
    // reports when it sees nothing. Never use them for control.
    tlmMm("rear_sensor_0_raw_mm", rearTofRawMm(REAR_TOF_0));
    tlmMm("rear_sensor_1_raw_mm", rearTofRawMm(REAR_TOF_1));
    tlmMm("rear_sensor_2_raw_mm", rearTofRawMm(REAR_TOF_2));

    tlm(",\"rear_fail_streak\":[%u,%u,%u]",
        (unsigned)rearTofFailStreak(REAR_TOF_0),
        (unsigned)rearTofFailStreak(REAR_TOF_1),
        (unsigned)rearTofFailStreak(REAR_TOF_2));

    // Which TCA channel each index sits on. Confirmed hardware, published so
    // the Pi's logs record the topology the firmware actually used.
    tlm(",\"rear_tca_channels\":[%u,%u,%u]",
        (unsigned)rearTofChannelOf(REAR_TOF_0),
        (unsigned)rearTofChannelOf(REAR_TOF_1),
        (unsigned)rearTofChannelOf(REAR_TOF_2));
#endif

    // ---- Backward compatibility -------------------------------------------
    // Field names published by the previous firmware, kept so an existing
    // Pi-side parser keeps working while it migrates. NOTHING here was
    // removed or repurposed -- the Pi contract is additive only.
    //
    // "rear_sensing" keeps its original NONE/PRESENT vocabulary and its
    // original meaning: "can this firmware read anything at the rear?". It now
    // reports PRESENT once the VL53L0X subsystem is live.
    //
    // The old "rear_ir_*" names are kept as aliases of the new rear fields.
    // They were introduced when the rear sensors were believed to be
    // presence-only IR detectors; they are VL53L0X rangefinders, so the
    // CLEAR/OBSTACLE vocabulary is now derived from the real latched obstacle
    // state rather than from a sensor that was never read. A sensor with no
    // trustworthy reading still reports "UNKNOWN", which is not "CLEAR".
    tlmCm("front_sensor_1_cm", sensorFilteredCm(US_FRONT_LEFT));
    tlmCm("front_sensor_2_cm", sensorFilteredCm(US_FRONT_RIGHT));
    tlm(",\"rear_sensing\":\"%s\"",
        safetyRearSensingAvailable() ? "PRESENT" : "NONE");

#if TELEMETRY_INCLUDE_LEGACY_REAR
    tlm(",\"rear_ir_count\":%d", (int)REAR_TOF_SENSOR_COUNT);
    tlm(",\"rear_ir_backend\":\"%s\"", rearBackendName());
    tlmBool("rear_ir_available", rearSensingUsable());

    tlm(",\"rear_ir\":[");
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        const char *st = "UNKNOWN";
        if (safetyRearSensorValid(i)) {
            st = safetyRearSensorObstacle(i) ? "OBSTACLE" : "CLEAR";
        }
        tlm("%s\"%s\"", (i ? "," : ""), st);
    }
    tlm("]");

    tlm(",\"rear_ir_valid\":[");
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        tlm("%s%s", (i ? "," : ""), safetyRearSensorValid(i) ? "true" : "false");
    }
    tlm("]");
#endif // TELEMETRY_INCLUDE_LEGACY_REAR

    tlm(",\"uptime_ms\":%lu", (unsigned long)millis());
    tlm("}");

    if (gTlmOverflow) {
        // Never emit a truncated object that would parse as valid-but-wrong.
        Serial.println("{\"event\":\"TELEMETRY_OVERFLOW\"}");
        return;
    }

    Serial.println(gTlmBuf);
}


// ============================================================================
// ACCESSORS
// ============================================================================

void commInit(void)
{
    gDesiredLeft       = 0;
    gDesiredRight      = 0;
    gAppliedLeft       = 0;
    gAppliedRight      = 0;
    gBlockReason       = "NONE";
    gLastRejectReason  = "NONE";
    gLineLen           = 0;
    gOverflow          = false;
    gTestActive        = false;
    gEverReceivedCmd   = false;
    gTimedOut          = false;
    gSafetyStopLatched = false;
    gLastCommandMs     = millis();
    gState             = STATE_IDLE;
}

void commLatchSafetyStop(void)   { gSafetyStopLatched = true;  }
bool commSafetyStopLatched(void) { return gSafetyStopLatched;  }

int commDesiredLeft(void)   { return gDesiredLeft;  }
int commDesiredRight(void)  { return gDesiredRight; }

void commClearDesired(void)
{
    gDesiredLeft  = 0;
    gDesiredRight = 0;
}

uint32_t commMsSinceLastCommand(void)
{
    return millis() - gLastCommandMs;
}

bool commHasEverReceivedCommand(void)
{
    return gEverReceivedCmd;
}

void commSetTimedOut(bool timedOut) { gTimedOut = timedOut; }
bool commTimedOut(void)             { return gTimedOut;     }

void commSetState(RobotState s) { gState = s; }
RobotState commGetState(void)   { return gState; }

bool commTestModeActive(void) { return gTestActive;  }
int  commTestChannel(void)    { return gTestChannel; }
int  commTestSpeed(void)      { return gTestSpeed;   }

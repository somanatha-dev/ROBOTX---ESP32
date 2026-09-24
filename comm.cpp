#include <Arduino.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "comm.h"
#include "protocol.h"
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

// What the safety and availability gates permitted.
static int  gGatedLeft    = 0;
static int  gGatedRight   = 0;

// What the motor layer actually outputs: the gated values after clamp and
// deadband, and zero when there is no verified speed path. This is the number
// published as "applied" -- never a value the hardware does not receive.
static int  gAppliedLeft  = 0;
static int  gAppliedRight = 0;

// Why the gate clamped something, or "NONE".
static const char *gBlockReason = "NONE";

// Most recent rejection reason (REJECTED ack or ERROR frame), or "NONE".
// Sticky until the next ACCEPTED command. The state field is recomputed every
// pass, so a transient "bad command" state could never survive to telemetry;
// this field is how the Pi sees that something was refused.
static const char *gLastRejectReason = "NONE";

static uint32_t gLastCommandMs   = 0;
static bool     gEverReceivedCmd = false;
static bool     gTimedOut        = false;

// Latched safety stop. See the note in comm.h for why this is not just a
// RobotState value.
static bool     gSafetyStopLatched = false;

// Incoming line assembly. No terminator is stored; lengths are passed.
static char     gLine[COMM_LINE_MAX];
static uint16_t gLineLen  = 0;
static bool     gOverflow = false;

// MOTORTEST.
static bool     gTestActive   = false;
static uint32_t gTestDeadline = 0;
static int      gTestChannel  = 0;
static int      gTestSpeed    = 0;
static uint16_t gTestSeq      = 0;

// The message being handled. Static rather than on the stack: it is ~600 bytes.
static ProtoMessage gMsg;
static uint16_t     gCurSeq = 0;
static const char  *gCurCmd = "";

// Duplicate detection: the seq of the most recent frame that received an ACK,
// and what that ACK said.
static bool         gHaveLastSeq = false;
static uint16_t     gLastSeq     = 0;
static char         gLastSeqCmd[PROTO_STR_MAX + 1] = "";
static const char  *gLastSeqResult = "NONE";

// Link statistics, published in the SYSTEM diagnostic section.
typedef struct {
    uint32_t rxOk;              // frames with a valid envelope (ACKed)
    uint32_t rxEmpty;           // blank lines (ignored)
    uint32_t rxBadFrame;        // INVALID_FRAME
    uint32_t rxBadCrc;          // INVALID_CRC
    uint32_t rxBadMessage;      // CRC valid, content invalid (ERROR sent)
    uint32_t rxTooLong;         // FRAME_TOO_LONG
    uint32_t rxRejected;        // ACK result REJECTED
    uint32_t rxDuplicates;      // ACK result DUPLICATE
    uint32_t rxStale;           // ACK reason STALE_SEQ
    uint32_t errorsSuppressed;  // ERROR frames dropped by the rate limit
    uint32_t txOverflows;       // outgoing frames that did not fit
} LinkStats;

static LinkStats gStats;

// Rate limit for ERROR frames that answer unidentifiable input (seq null).
// Line noise must not be able to fill the TX link with error reports.
static uint32_t gErrWindowStartMs = 0;
static uint8_t  gErrInWindow      = 0;

// Telemetry scheduling.
static uint32_t gLastFastMs      = 0;
static uint8_t  gFastCount       = 0;
static bool     gDiagPending     = false;
static uint32_t gDiagDueMs       = 0;
static uint8_t  gNextDiagSection = 0;


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
// OUTGOING FRAMES
// ============================================================================
//
// Every outgoing message is built in ONE static buffer and handed to the UART
// in a single write, so a frame is never interleaved with another and never
// sent half-built. protoWriterFinish() appends the CRC trailer. If a frame
// does not fit, a short ERROR frame goes out instead -- never a truncated one.
// ============================================================================

static char        gTx[COMM_TX_FRAME_MAX];
static ProtoWriter gW;
static long        gTxSeq = -1;     // seq the frame being built answers, or -1

static void txBegin(const char *type, long seq)
{
    gTxSeq = seq;
    protoWriterBegin(&gW, gTx, sizeof(gTx), type);
}

static void tx(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void tx(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    protoWriterAppendV(&gW, fmt, ap);
    va_end(ap);
}

static void txBool(const char *key, bool v)
{
    tx(",\"%s\":%s", key, v ? "true" : "false");
}

static void txStr(const char *key, const char *v)
{
    tx(",\"%s\":\"%s\"", key, v);
}

// "seq":N or "seq":null.
static void txSeq(long seq)
{
    if (seq < 0) {
        tx(",\"seq\":null");
    } else {
        tx(",\"seq\":%ld", seq);
    }
}

// A distance in cm with one decimal, or null when it is not a measurement.
// Integer arithmetic, so the output does not depend on printf float support.
static void txCmValue(float cm)
{
    if (cm > 0.0f) {
        long t = (long)(cm * 10.0f + 0.5f);
        tx("%ld.%ld", t / 10L, t % 10L);
    } else {
        tx("null");
    }
}

static void txCm(const char *key, float cm)
{
    tx(",\"%s\":", key);
    txCmValue(cm);
}

// A distance in mm, or null. There is no path that publishes 0 mm, which
// would read as an obstacle touching the sensor.
static void txMmValue(int mm)
{
    if (mm > 0) {
        tx("%d", mm);
    } else {
        tx("null");
    }
}

static void txMm(const char *key, int mm)
{
    tx(",\"%s\":", key);
    txMmValue(mm);
}

static void txOverflowError(long seq)
{
    // Built in its own small buffer: gTx is the thing that just overflowed.
    char        buf[112];
    ProtoWriter w;
    protoWriterBegin(&w, buf, sizeof(buf), "ERROR");
    if (seq < 0) {
        protoWriterAppend(&w, ",\"seq\":null");
    } else {
        protoWriterAppend(&w, ",\"seq\":%ld", seq);
    }
    protoWriterAppend(&w, ",\"reason\":\"TX_FRAME_OVERFLOW\",\"uptime_ms\":%lu",
                      (unsigned long)millis());
    size_t n = protoWriterFinish(&w);
    if (n > 0) {
        Serial.write((const uint8_t *)buf, n);
    }
}

static void txEnd(void)
{
    size_t n = protoWriterFinish(&gW);

    if (n == 0) {
        gStats.txOverflows++;
        txOverflowError(gTxSeq);
        return;
    }

    Serial.write((const uint8_t *)gTx, n);
}


// ============================================================================
// ERROR FRAMES
// ============================================================================
//
// ERROR answers a line that could not be accepted as a command at all: a bad
// frame, a bad CRC, malformed content, or a bad envelope (seq / type / cmd).
// seq is echoed only when it was itself valid; otherwise it is null.
// ============================================================================

static void emitError(const char *reason, long seq, const char *field)
{
    gLastRejectReason = reason;

    if (seq < 0) {
        uint32_t now = millis();
        if (now - gErrWindowStartMs >= 1000UL) {
            gErrWindowStartMs = now;
            gErrInWindow      = 0;
        }
        if (gErrInWindow >= COMM_ERROR_FRAMES_PER_SEC) {
            gStats.errorsSuppressed++;
            return;
        }
        gErrInWindow++;
    }

    txBegin("ERROR", seq);
    txSeq(seq);
    txStr("reason", reason);
    if (field != NULL) {
        txStr("field", field);
    }
    txEnd();
}


// ============================================================================
// ACK FRAMES
// ============================================================================
//
// Exactly one ACK per command frame that passed the envelope checks. Always:
//   "seq"    -- echoed
//   "cmd"    -- echoed
//   "result" -- ACCEPTED | GATED | REJECTED | DUPLICATE
//   "reason" -- NONE when ACCEPTED, otherwise the precise cause
// ============================================================================

static void ackBegin(const char *result, const char *reason)
{
    gLastSeqResult = result;

    if (strcmp(result, "ACCEPTED") == 0) {
        // A clean acceptance clears the sticky rejection, so "last_reject"
        // reads as "what went wrong since the last command that worked".
        gLastRejectReason = "NONE";
    } else if (strcmp(result, "REJECTED") == 0) {
        gLastRejectReason = reason;
        gStats.rxRejected++;
    }

    txBegin("ACK", (long)gCurSeq);
    tx(",\"seq\":%u", (unsigned)gCurSeq);
    txStr("cmd", gCurCmd);
    txStr("result", result);
    txStr("reason", reason);
}

static void ackReject(const char *reason, const char *field)
{
    ackBegin("REJECTED", reason);
    if (field != NULL) {
        txStr("field", field);
    }
    txEnd();
}

static void ackAcceptedPlain(void)
{
    ackBegin("ACCEPTED", "NONE");
    txEnd();
}


// ============================================================================
// FIELD VALIDATION
// ============================================================================
//
// Each helper either delivers a valid value or sends the REJECTED ack itself
// and returns false; the caller then simply returns. The order a handler calls
// them in is the order errors are reported in, which makes the reason for a
// frame with several faults deterministic:
//
//   1. unknown fields   2. required fields, in the documented order
// ============================================================================

static bool isEnvelopeKey(const char *key)
{
    return strcmp(key, "type") == 0 || strcmp(key, "seq") == 0 ||
           strcmp(key, "cmd") == 0;
}

// Every key must be an envelope key or one of `allowed`.
static bool onlyFields(const char *const *allowed, uint8_t allowedCount)
{
    for (uint8_t i = 0; i < gMsg.count; i++) {
        const char *key = gMsg.field[i].key;
        if (isEnvelopeKey(key)) {
            continue;
        }

        bool known = false;
        for (uint8_t k = 0; k < allowedCount; k++) {
            if (strcmp(key, allowed[k]) == 0) {
                known = true;
                break;
            }
        }
        if (!known) {
            ackReject("UNKNOWN_FIELD", key);
            return false;
        }
    }
    return true;
}

static bool noFields(void)
{
    return onlyFields(NULL, 0);
}

// Integer in [min, max]. Never clamps: out of range is a rejection.
static bool needInt(const char *key, long min, long max, int *out)
{
    const ProtoField *f = protoFindField(&gMsg, key);
    if (f == NULL) {
        ackReject("MISSING_FIELD", key);
        return false;
    }
    if (f->type != PV_INT) {
        ackReject("WRONG_TYPE", key);
        return false;
    }
    if (f->intOverflow || f->intValue < min || f->intValue > max) {
        ackReject("OUT_OF_RANGE", key);
        return false;
    }
    *out = (int)f->intValue;
    return true;
}

// As needInt(), but absence is fine and reported through *present.
static bool optInt(const char *key, long min, long max, int *out, bool *present)
{
    *present = (protoFindField(&gMsg, key) != NULL);
    if (!*present) {
        return true;
    }
    return needInt(key, min, max, out);
}

static bool needBool(const char *key, bool *out)
{
    const ProtoField *f = protoFindField(&gMsg, key);
    if (f == NULL) {
        ackReject("MISSING_FIELD", key);
        return false;
    }
    if (f->type != PV_BOOL) {
        ackReject("WRONG_TYPE", key);
        return false;
    }
    *out = f->boolValue;
    return true;
}

static bool needStr(const char *key, const char **out)
{
    const ProtoField *f = protoFindField(&gMsg, key);
    if (f == NULL) {
        ackReject("MISSING_FIELD", key);
        return false;
    }
    if (f->type != PV_STR) {
        ackReject("WRONG_TYPE", key);
        return false;
    }
    *out = f->str;
    return true;
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
//   1. MOTOR_TEST        - commissioning override is running
//   2. DRIVE_UNAVAILABLE - no verified speed path at all
//   3. COMMAND_TIMEOUT   - failsafe latch is set (cleared by the next command)
//   4. SAFETY_STOP       - the gate is clamping RIGHT NOW because of a
//                          confirmed obstacle
//   5. SENSOR_FAULT      - the gate is clamping RIGHT NOW because sensing is
//                          not trustworthy
//   6. IDLE              - booted, never commanded
//   7. motion state      - derived from what actually reaches the motors
//
// The STICKY "this rover performed a safety stop" record is the separate
// commSafetyStopLatched() flag, published as "safety_stop".
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

void commUpdateRuntime(int gatedLeft, int gatedRight, const char *blockReason)
{
    gGatedLeft  = gatedLeft;
    gGatedRight = gatedRight;

    // Exactly what driveDifferential() does with the gated values: clamp and
    // deadband -- and nothing at all without a verified PCA9685.
    bool drive    = motorDriveAvailable();
    gAppliedLeft  = drive ? motorEffectivePower(gatedLeft)  : 0;
    gAppliedRight = drive ? motorEffectivePower(gatedRight) : 0;

    gBlockReason  = (blockReason != NULL) ? blockReason : "NONE";

    gState = computeState();
}

int         commAppliedLeft(void)  { return gAppliedLeft;  }
int         commAppliedRight(void) { return gAppliedRight; }
const char *commBlockReason(void)  { return gBlockReason;  }


// ============================================================================
// MOTORTEST END
// ============================================================================

static void endMotorTest(const char *why)
{
    if (!gTestActive) {
        return;
    }

    gTestActive = false;
    stopMotors();
    commUpdateRuntime(0, 0, "NONE");

    txBegin("EVENT", (long)gTestSeq);
    txStr("event", "MOTORTEST_DONE");
    txSeq((long)gTestSeq);
    txStr("reason", why);
    tx(",\"uptime_ms\":%lu", (unsigned long)millis());
    txEnd();
}


// ============================================================================
// APPLY AN ACCEPTED MOTION REQUEST
// ============================================================================
//
// Reached ONLY after the frame, the envelope and every field have validated.
//
// The safety gate is applied HERE as well as in the main loop. Here it lets
// us send the Pi a meaningful, immediate answer; the loop check is the actual
// continuous authority that catches an obstacle appearing mid-manoeuvre.
//
// BOTH call safetyGateMotion() -- the single direction-aware gate in
// safety.cpp -- rather than each re-deriving "is this forward?".
//
// WATCHDOG POLICY (unchanged from the baseline): a validated movement command
// refreshes the failsafe timer even when the gate clamps it. The timer
// measures whether the Pi is still talking to us, and a Pi that is pushing
// into an obstacle is very much still talking. Nothing that failed validation
// can reach this function.
// ============================================================================

static void ackMotion(const char *result, const char *reason, int reqL, int reqR)
{
    ackBegin(result, reason);
    tx(",\"req_left\":%d,\"req_right\":%d", reqL, reqR);
    tx(",\"gated_left\":%d,\"gated_right\":%d", gGatedLeft, gGatedRight);
    tx(",\"applied_left\":%d,\"applied_right\":%d", gAppliedLeft, gAppliedRight);
    txEnd();
}

static void applyMotion(int left, int right)
{
    // A validated movement command: the link is alive.
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
    // a verified one there is nothing to command. Reported as GATED, not
    // REJECTED: the command itself was valid and the link is alive.
    //
    // The stored intent is kept, not cleared, so that if the PCA9685 becomes
    // available the next loop pass acts on what the Pi is still asking for --
    // the same rule the obstacle gate follows.
    // ------------------------------------------------------------------
    if (!motorDriveAvailable()) {
        stopMotors();
        commUpdateRuntime(0, 0, "MOTOR_PWM_UNAVAILABLE");
        ackMotion("GATED", "MOTOR_PWM_UNAVAILABLE", left, right);
        return;
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
        ackMotion("GATED", reason, left, right);
        return;
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
    ackMotion("ACCEPTED", "NONE", left, right);
}


// ============================================================================
// COMMAND HANDLERS
// ============================================================================

// Explicit STOP body, shared by STOP and MOVE dir=S. Zeroes the stored command
// so the rover STAYS stopped, and refreshes the watchdog: STOP is a movement
// command and is always accepted -- it can never be unsafe.
//
// The safety-stop latch is deliberately NOT cleared here. STOP means "hold
// still", not "the obstacle situation is resolved". RESET acknowledges it.
static void doStop(void)
{
    stopMotors();

    gDesiredLeft  = 0;
    gDesiredRight = 0;

    gLastCommandMs   = millis();
    gEverReceivedCmd = true;
    gTimedOut        = false;

    commUpdateRuntime(0, 0, "NONE");
    ackMotion("ACCEPTED", "NONE", 0, 0);
}

static void handleStop(void)
{
    if (!noFields()) return;
    doStop();
}


// {"cmd":"DRIVE","left":L,"right":R}  -- both required, -255..255.
static void handleDrive(void)
{
    static const char *const kFields[] = { "left", "right" };

    int left  = 0;
    int right = 0;

    if (!onlyFields(kFields, 2))                              return;
    if (!needInt("left",  -PWM_MAX_DUTY, PWM_MAX_DUTY, &left))  return;
    if (!needInt("right", -PWM_MAX_DUTY, PWM_MAX_DUTY, &right)) return;

    applyMotion(left, right);
}


// Legacy direction form: {"cmd":"MOVE","dir":"F","speed":150}
// Directions are translated into differential pairs. This is NOT steering --
// L/R simply slow the inner wheels by MOTOR_TURN_INNER_PCT.
static void handleMove(void)
{
    static const char *const kFields[] = { "dir", "speed" };

    const char *dir = NULL;

    if (!onlyFields(kFields, 2)) return;
    if (!needStr("dir", &dir))   return;

    bool isF = (strcmp(dir, "F") == 0);
    bool isB = (strcmp(dir, "B") == 0);
    bool isL = (strcmp(dir, "L") == 0);
    bool isR = (strcmp(dir, "R") == 0);
    bool isS = (strcmp(dir, "S") == 0);

    if (!(isF || isB || isL || isR || isS)) {
        // Unchanged from the baseline: an unrecognised direction is treated
        // as "stop what you were doing", the safe reading of a garbled
        // movement instruction. It does NOT refresh the watchdog.
        stopMotors();
        gDesiredLeft  = 0;
        gDesiredRight = 0;
        commUpdateRuntime(0, 0, "NONE");
        ackReject("INVALID_ARGUMENT", "dir");
        return;
    }

    int  speed = 0;
    bool haveSpeed = false;

    if (isS) {
        // speed is optional for a stop, but still validated if present.
        if (!optInt("speed", 0, PWM_MAX_DUTY, &speed, &haveSpeed)) return;
        doStop();
        return;
    }

    if (!needInt("speed", 0, PWM_MAX_DUTY, &speed)) return;

    int left  = speed;
    int right = speed;

    if (isB) {
        left  = -speed;
        right = -speed;
    } else if (isL) {
        left  = motorArcInnerSpeed(speed);
    } else if (isR) {
        right = motorArcInnerSpeed(speed);
    }

    applyMotion(left, right);
}


// Commissioning only. Drives ONE raw motor channel for a bounded time so the
// left/right mapping and polarity can be established by observation.
// Deliberately bypasses the obstacle gate: the rover is expected to be ON
// BLOCKS. It does NOT refresh the movement watchdog.
//
//   {"cmd":"MOTORTEST","motor":0,"power":120,"ms":600,"onblocks":true}
//
// Every field is required and range-checked; nothing is defaulted or clamped.
static bool validateMotorTest(int *motor, int *power, int *ms)
{
    static const char *const kFields[] = { "motor", "power", "ms", "onblocks" };

    if (!onlyFields(kFields, 4))                                      return false;
    if (!needInt("motor", 0, MOTOR_CHANNEL_COUNT - 1, motor))         return false;
    if (!needInt("power", -PWM_MAX_DUTY, PWM_MAX_DUTY, power))        return false;
    if (!needInt("ms", 1, (long)MOTORTEST_MAX_MS, ms))                return false;

    // ------------------------------------------------------------------
    // PHYSICAL INTERLOCK. MOTORTEST spins a wheel with the obstacle gate
    // bypassed. Require the caller to state, in the command, that the rover
    // is secured. Not security -- a guard against a half-remembered command
    // pasted while the rover sits on the floor.
    // ------------------------------------------------------------------
    bool onBlocks = false;
#if MOTORTEST_REQUIRE_ONBLOCKS
    if (!needBool("onblocks", &onBlocks))                             return false;
    if (!onBlocks) {
        ackReject("ONBLOCKS_REQUIRED", "onblocks");
        return false;
    }
#else
    if (protoFindField(&gMsg, "onblocks") != NULL &&
        !needBool("onblocks", &onBlocks))                             return false;
#endif

    // No speed path means no test. Say so explicitly rather than running a
    // test that silently does nothing and leaves the operator concluding the
    // motor channel is dead.
    if (!motorDriveAvailable()) {
        ackReject("MOTOR_PWM_UNAVAILABLE", NULL);
        return false;
    }

    return true;
}

static void handleMotorTest(void)
{
    int motor = 0;
    int power = 0;
    int ms    = 0;

    if (!validateMotorTest(&motor, &power, &ms)) {
        // A refused MOTORTEST never leaves an earlier one running.
        endMotorTest("CANCELLED");
        return;
    }

    // A new test supersedes a running one.
    endMotorTest("REPLACED");

    // What the hardware will ACTUALLY receive after deadband. If this comes
    // back 0 for a non-zero request, the motor will not turn and the operator
    // needs to know that now.
    int effective = motorEffectivePower(power);

    gTestChannel  = motor;
    gTestSpeed    = power;
    gTestSeq      = gCurSeq;
    gTestDeadline = millis() + (uint32_t)ms;
    gTestActive   = true;

    gDesiredLeft  = 0;
    gDesiredRight = 0;

    commUpdateRuntime(0, 0, "NONE");    // recomputes to STATE_MOTOR_TEST

    // The ack states the WIRING it is about to drive, so the operator can
    // check it against the loom instead of cross-referencing config.h.
    // "configured_side" is the UNVERIFIED assumption under test.
    ackBegin("ACCEPTED", "NONE");
    tx(",\"motor\":%d", motor);
    txStr("name", motorChannelName(motor));
    tx(",\"in1_gpio\":%u,\"in2_gpio\":%u,\"enable_pca_ch\":%u",
       (unsigned)motorChannelIn1Pin(motor),
       (unsigned)motorChannelIn2Pin(motor),
       (unsigned)motorChannelEnableCh(motor));
    txStr("configured_side", motorChannelSideName(motor));
    txBool("side_verified", motorMapVerified());
    tx(",\"power\":%d,\"effective_power\":%d", power, effective);
    txBool("below_deadband", power != 0 && effective == 0);
    tx(",\"ms\":%d", ms);
    txEnd();
}


// Clears a latched SAFETY_STOP and the COMMAND_TIMEOUT latch without
// commanding any motion. Motors stay stopped; a real movement command is
// still required to move.
static void handleReset(void)
{
    if (!noFields()) return;

    stopMotors();
    gDesiredLeft  = 0;
    gDesiredRight = 0;

    // RESET is the explicit acknowledgement of a safety stop. It clears the
    // latch but commands NO motion.
    gSafetyStopLatched = false;
    gTimedOut          = false;

    // NOTE: does NOT refresh gLastCommandMs. RESET is not a movement command,
    // so it must not hold the failsafe open. gTimedOut will re-assert on the
    // next loop pass unless a real command arrives.
    commUpdateRuntime(0, 0, "NONE");

    ackAcceptedPlain();
}


static void handlePing(void)
{
    if (!noFields()) return;

    ackBegin("ACCEPTED", "NONE");
    txStr("state", robotStateName(gState));
    tx(",\"uptime_ms\":%lu,\"proto\":%d", (unsigned long)millis(), PROTO_VERSION);
    txEnd();
}


// ============================================================================
//  DIAGNOSTIC / COMMISSIONING COMMANDS
// ============================================================================
//
// These exist to answer the hardware questions this firmware refuses to guess
// at. Results are carried in the command's ACK, so each still gets exactly one
// response.
//
// SAFETY PROPERTIES SHARED BY ALL OF THEM:
//   * None can turn a wheel. Not one of them touches the motor API.
//   * None refreshes the movement watchdog.
//   * None writes to an address the operator did not name in the command.
//
// Compiled out entirely by DIAGNOSTICS_ENABLED=0 in config.h.
// ============================================================================

#if DIAGNOSTICS_ENABLED

// Last scan result, remembered so I2CSTATUS can report it without re-running a
// scan. On a stuck bus a scan costs ~1 second PER ADDRESS.
static uint32_t gLastScanMs      = 0;
static uint8_t  gLastScanCount   = 0;
static bool     gLastScanDone    = false;
static uint32_t gLastScanPerAddr = 0;

// Most devices a scan lists. 16 keeps the worst-case frame inside
// COMM_TX_FRAME_MAX with the longest address hints.
#define I2C_SCAN_LIST_MAX 16


// ---------------------------------------------------------------------------
// I2C BUS SCAN body -- shared by the boot EVENT and the I2CSCAN ACK, so the
// two can never disagree about format.
//
// WHAT IT DOES TO THE BUS: roverI2cProbe() sends START, address, STOP. No data
// byte is written to any address. The one write is closing the multiplexer
// channels, and only once the TCA9548A address is confirmed.
//
// READ THE HINTS AS HINTS. 0x70..0x77 is shared territory between a TCA9548A
// and a fully-jumpered PCA9685. EXPECT 0x29 TO BE ABSENT -- the VL53L0X sit
// behind the multiplexer.
// ---------------------------------------------------------------------------
static void txI2cScanBody(const char *trigger)
{
    bool muxDeselected = tcaDeselectAll();

    uint32_t startMs = millis();

    uint8_t found[I2C_SCAN_LIST_MAX];
    uint8_t n = roverI2cScan(found, (uint8_t)I2C_SCAN_LIST_MAX);

    uint32_t elapsedMs = millis() - startMs;

    gLastScanMs      = elapsedMs;
    gLastScanCount   = n;
    gLastScanDone    = true;
    gLastScanPerAddr = elapsedMs / (uint32_t)I2C_SCAN_ADDR_COUNT;

    txStr("trigger", trigger);
    tx(",\"sda\":%d,\"scl\":%d,\"clock_hz\":%lu,\"scan_ms\":%lu",
       (int)I2C_SDA_PIN, (int)I2C_SCL_PIN,
       (unsigned long)I2C_CLOCK_HZ, (unsigned long)elapsedMs);
    txStr("bus_writes", muxDeselected ? "mux_deselect_only" : "none");
    tx(",\"count\":%u", (unsigned)n);
    txBool("list_full", n >= I2C_SCAN_LIST_MAX);

    tx(",\"devices\":[");
    for (uint8_t i = 0; i < n; i++) {
        tx("%s{\"address\":\"0x%02X\",\"dec\":%u,\"hint\":\"%s\"}",
           (i ? "," : ""), found[i], (unsigned)found[i],
           roverI2cAddressHint(found[i]));
    }
    tx("]");

    txBool("tca_confirmed", tcaAddressConfirmed());
    txBool("pca_confirmed", pca9685AddressConfirmed());
    txStr("note", "hints are candidates, NOT identification. 0x70-0x77 "
                  "could be either a TCA9548A or a fully-jumpered PCA9685. "
                  "0x29 should NOT appear - the VL53L0X are behind the mux.");
}


// One-shot boot scan. Called from setup(), never from loop().
void commBootI2cScan(void)
{
    txBegin("EVENT", -1);
    txStr("event", "I2CSCAN");
    txSeq(-1);

    if (!roverI2cReady()) {
        txStr("trigger", "BOOT");
        txStr("error", "I2C_NOT_READY");
        tx(",\"count\":0,\"devices\":[]");
    } else {
        txI2cScanBody("BOOT");
    }
    txEnd();
}


static void handleI2cScan(void)
{
    if (!noFields()) return;

    if (!roverI2cReady()) {
        ackReject("I2C_NOT_READY", NULL);
        return;
    }

#if I2C_SCAN_REQUIRE_STOPPED
    // A scan blocks the control loop for tens of milliseconds -- no ultrasonic
    // ping and no safety-gate re-evaluation for that window. Acceptable on a
    // stationary rover, not on a moving one.
    if (gAppliedLeft != 0 || gAppliedRight != 0) {
        ackReject("REFUSED_ROVER_IS_MOVING_SEND_STOP_FIRST", NULL);
        return;
    }
#endif

    ackBegin("ACCEPTED", "NONE");
    txI2cScanBody("COMMAND");
    txEnd();
}


// ---------------------------------------------------------------------------
// I2CSTATUS -- the PHYSICAL state of the bus, without using it. No I2C
// transaction, no pin mode change, no device written.
// ---------------------------------------------------------------------------
static void handleI2cStatus(void)
{
    if (!noFields()) return;

    RoverI2cLineState lines;
    roverI2cSampleLines(&lines);

    bool eitherLow = (!lines.sdaHigh || !lines.sclHigh ||
                      lines.sdaStuckLow || lines.sclStuckLow);

    ackBegin("ACCEPTED", "NONE");
    txBool("i2c_initialized", roverI2cReady());
    tx(",\"sda_pin\":%d,\"scl_pin\":%d,\"clock_hz\":%lu",
       (int)I2C_SDA_PIN, (int)I2C_SCL_PIN, (unsigned long)I2C_CLOCK_HZ);
    txStr("internal_pullups", "ENABLED_BY_CORE");
    txStr("sda_level", lines.sdaHigh ? "HIGH" : "LOW");
    txStr("scl_level", lines.sclHigh ? "HIGH" : "LOW");
    tx(",\"sda_high_samples\":%u,\"scl_high_samples\":%u,\"samples\":%u",
       (unsigned)lines.sdaHighSamples, (unsigned)lines.sclHighSamples,
       (unsigned)lines.sampleCount);
    txBool("sda_stuck_low", lines.sdaStuckLow);
    txBool("scl_stuck_low", lines.sclStuckLow);
    txBool("either_line_low", eitherLow);
    txBool("bus_idle", !eitherLow);

    // Last scan, recalled rather than re-run.
    if (gLastScanDone) {
        bool stuckTiming = (gLastScanPerAddr >= I2C_SCAN_STUCK_MS_PER_ADDR);
        tx(",\"last_scan\":{\"done\":true,\"count\":%u,\"total_ms\":%lu,"
           "\"ms_per_address\":%lu,\"addresses_probed\":%d,"
           "\"timing_indicates\":\"%s\"}",
           (unsigned)gLastScanCount, (unsigned long)gLastScanMs,
           (unsigned long)gLastScanPerAddr, (int)I2C_SCAN_ADDR_COUNT,
           stuckTiming ? "STUCK_BUS" : "NORMAL_NACK_TIMING");
    } else {
        tx(",\"last_scan\":{\"done\":false}");
    }

    txStr("verdict", roverI2cLineVerdict(&lines));
    txEnd();
}


// ---------------------------------------------------------------------------
// TCATEST [addr] -- write a channel mask and read the control register back.
// A TCA9548A returns exactly what was written; a PCA9685 does not. Always
// leaves the candidate with all channels closed.
//
// addr is a JSON integer (decimal), 8..119. Absent: the configured address,
// which is only allowed once it is confirmed.
// ---------------------------------------------------------------------------
static void handleTcaTest(void)
{
    static const char *const kFields[] = { "addr" };

    int  addrValue = 0;
    bool haveAddr  = false;

    if (!onlyFields(kFields, 1)) return;
    if (!optInt("addr", I2C_SCAN_FIRST_ADDR, I2C_SCAN_LAST_ADDR,
                &addrValue, &haveAddr)) return;

    if (!roverI2cReady()) {
        ackReject("I2C_NOT_READY", NULL);
        return;
    }

    uint8_t addr = (uint8_t)addrValue;
    if (!haveAddr) {
        if (!tcaAddressConfirmed()) {
            ackReject("NO_ADDR_GIVEN_AND_NONE_CONFIRMED", "addr");
            return;
        }
        addr = tcaAddress();
    }

    // 0x01 = channel 0 only: even if this IS the multiplexer we open exactly
    // one segment and never two.
    const uint8_t testMask = 0x01;

    // Probe BEFORE the read-back test and capture both results into locals;
    // each call overwrites the shared last-error state.
    bool responded = roverI2cProbe(addr);

    uint8_t readBack = 0;
    bool looksLikeTca = tcaProbeCandidate(addr, testMask, &readBack);
    const char *errName = roverI2cErrorName(roverI2cLastError());

    ackBegin("ACCEPTED", "NONE");
    tx(",\"addr\":%u,\"addr_hex\":\"0x%02X\"", (unsigned)addr, addr);
    txBool("responded", responded);
    tx(",\"wrote\":\"0x%02X\",\"read_back\":\"0x%02X\"", testMask, readBack);
    txBool("looks_like_tca9548a", looksLikeTca);
    txStr("i2c_error", errName);

    if (looksLikeTca && !tcaAddressConfirmed()) {
        tx(",\"action\":\"set TCA9548A_I2C_ADDRESS to 0x%02X and "
           "TCA9548A_ADDRESS_CONFIRMED to 1 in config.h, then reflash\"", addr);
    } else if (!looksLikeTca) {
        txStr("action", "read-back did not match - this address is probably "
                        "NOT a TCA9548A");
    }
    txEnd();
}


// ---------------------------------------------------------------------------
// PCATEST [addr] -- READ ONLY. Reads MODE1 and PRESCALE and writes nothing.
// ---------------------------------------------------------------------------
static void handlePcaTest(void)
{
    static const char *const kFields[] = { "addr" };

    int  addrValue = 0;
    bool haveAddr  = false;

    if (!onlyFields(kFields, 1)) return;
    if (!optInt("addr", I2C_SCAN_FIRST_ADDR, I2C_SCAN_LAST_ADDR,
                &addrValue, &haveAddr)) return;

    if (!roverI2cReady()) {
        ackReject("I2C_NOT_READY", NULL);
        return;
    }

    uint8_t addr = (uint8_t)addrValue;
    if (!haveAddr) {
        if (!pca9685AddressConfirmed()) {
            ackReject("NO_ADDR_GIVEN_AND_NONE_CONFIRMED", "addr");
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

    unsigned long impliedHz = 0;
    if (readable && prescale > 0) {
        impliedHz = (unsigned long)(PCA9685_OSC_HZ /
                                    (4096UL * ((unsigned long)prescale + 1UL)));
    }

    ackBegin("ACCEPTED", "NONE");
    tx(",\"addr\":%u,\"addr_hex\":\"0x%02X\"", (unsigned)addr, addr);
    txBool("responded", readable);
    tx(",\"mode1\":\"0x%02X\",\"prescale\":\"0x%02X\",\"implied_freq_hz\":%lu,"
       "\"configured_freq_hz\":%d",
       mode1, prescale, impliedHz, (int)PCA9685_PWM_FREQ_HZ);
    txStr("i2c_error", errName);
    txStr("status", pca9685StatusName());

    if (readable && !pca9685AddressConfirmed()) {
        tx(",\"action\":\"if this is the PCA9685, set PCA9685_I2C_ADDRESS to "
           "0x%02X and PCA9685_ADDRESS_CONFIRMED to 1 in config.h, then "
           "reflash\"", addr);
        txStr("caution", "a readable MODE1/PRESCALE pair is consistent with a "
                         "PCA9685 but does not prove one - confirm against "
                         "the board's solder jumpers");
    }
    txEnd();
}


// ---------------------------------------------------------------------------
// TOFTEST [sensor] -- one fresh measurement from one or all rear VL53L0X.
// Blocking, bench use only. Put a hand behind ONE sensor to resolve which TCA
// channel is physically left, centre and right.
// ---------------------------------------------------------------------------
static void handleTofTest(void)
{
    static const char *const kFields[] = { "sensor" };

    int  only = -1;
    bool haveSensor = false;

    if (!onlyFields(kFields, 1)) return;
    if (!optInt("sensor", 0, REAR_TOF_SENSOR_COUNT - 1, &only, &haveSensor)) return;
    if (!haveSensor) {
        only = -1;
    }

    if (!tcaAddressConfirmed()) {
        ackReject("TCA_ADDRESS_UNCONFIRMED", NULL);
        return;
    }
    if (!tcaPresent()) {
        ackReject("TCA_NOT_FOUND", NULL);
        return;
    }

    ackBegin("ACCEPTED", "NONE");
    tx(",\"tca_addr\":\"0x%02X\"", tcaAddress());
    txBool("orientation_verified", rearTofOrientationVerified());
    tx(",\"results\":[");

    bool first = true;

    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (only >= 0 && (int)i != only) {
            continue;
        }

        uint16_t mm = 0;
        RearTofStatus st = rearTofTestOne(i, &mm);

        tx("%s{\"sensor\":%u,\"tca_channel\":%u,\"status\":\"%s\"",
           first ? "" : ",", (unsigned)i,
           (unsigned)rearTofChannelOf(i), rearTofStatusName(st));

        // A distance is printed ONLY for a real measurement. OUT_OF_RANGE also
        // carries its raw number, explicitly labelled as not a measurement.
        if (st == TOF_VALID) {
            tx(",\"mm\":%u", (unsigned)mm);
        } else if (st == TOF_OUT_OF_RANGE) {
            tx(",\"mm\":null,\"raw_mm_not_a_measurement\":%u", (unsigned)mm);
        } else {
            tx(",\"mm\":null");
        }

        tx("}");
        first = false;
    }

    tx("]");
    txStr("note", "index is TCA channel order, NOT physical left/centre/right");
    txEnd();
}


// ---------------------------------------------------------------------------
// HWREPORT -- what the firmware believes about the hardware and, more
// usefully, what it does NOT know.
// ---------------------------------------------------------------------------
static void handleHwReport(void)
{
    if (!noFields()) return;

    ackBegin("ACCEPTED", "NONE");
    tx(",\"proto\":%d,\"core\":\"2.0.14\"", PROTO_VERSION);
    tx(",\"i2c\":{\"sda\":%d,\"scl\":%d,\"clock_hz\":%lu,\"ready\":%s}",
       (int)I2C_SDA_PIN, (int)I2C_SCL_PIN, (unsigned long)I2C_CLOCK_HZ,
       roverI2cReady() ? "true" : "false");

    tx(",\"tca9548a\":{\"address\":\"0x%02X\",\"confirmed\":%s,"
       "\"present\":%s,\"status\":\"%s\",\"rear_channels\":[%d,%d,%d]}",
       tcaAddress(),
       tcaAddressConfirmed() ? "true" : "false",
       tcaPresent() ? "true" : "false",
       tcaStatusName(),
       (int)TCA_CH_REAR_TOF_0, (int)TCA_CH_REAR_TOF_1, (int)TCA_CH_REAR_TOF_2);

    tx(",\"pca9685\":{\"address\":\"0x%02X\",\"confirmed\":%s,"
       "\"ready\":%s,\"status\":\"%s\",\"freq_hz\":%d,"
       "\"enable_channels\":[%d,%d,%d,%d]}",
       pca9685Address(),
       pca9685AddressConfirmed() ? "true" : "false",
       pca9685Ready() ? "true" : "false",
       pca9685StatusName(),
       (int)PCA9685_PWM_FREQ_HZ,
       (int)PCA_CH_FRONT_A_EN, (int)PCA_CH_FRONT_B_EN,
       (int)PCA_CH_REAR_A_EN,  (int)PCA_CH_REAR_B_EN);

    tx(",\"motors\":[");
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        tx("%s{\"ch\":%d,\"name\":\"%s\",\"in1_gpio\":%u,\"in2_gpio\":%u,"
           "\"enable_pca_ch\":%u,\"configured_side\":\"%s\"}",
           (i ? "," : ""), i, motorChannelName(i),
           (unsigned)motorChannelIn1Pin(i),
           (unsigned)motorChannelIn2Pin(i),
           (unsigned)motorChannelEnableCh(i),
           motorChannelSideName(i));
    }
    tx("]");

    tx(",\"front_ultrasonic\":{\"a\":{\"trig\":%d,\"echo\":%d},"
       "\"b\":{\"trig\":%d,\"echo\":%d}}",
       (int)US_A_TRIG_PIN, (int)US_A_ECHO_PIN,
       (int)US_B_TRIG_PIN, (int)US_B_ECHO_PIN);

    tx(",\"thresholds\":{\"front_stop_cm\":%d,\"front_clear_cm\":%d,"
       "\"front_warn_cm\":%d,\"rear_stop_mm\":%d,\"rear_clear_mm\":%d,"
       "\"rear_warn_mm\":%d,\"rear_thresholds_are_commissioning_values\":true}",
       (int)SAFETY_STOP_DISTANCE_CM, (int)SAFETY_CLEAR_DISTANCE_CM,
       (int)SAFETY_WARN_DISTANCE_CM,
       (int)REAR_STOP_DISTANCE_MM, (int)REAR_CLEAR_DISTANCE_MM,
       (int)REAR_WARN_DISTANCE_MM);

    // The honest part. Everything a human still has to go and look at.
    tx(",\"unverified\":[");
    bool first = true;
    #define UNV(cond, text) do { if (cond) { \
            tx("%s\"%s\"", first ? "" : ",", text); first = false; } \
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
    tx("]");
    txEnd();
}

#else  // !DIAGNOSTICS_ENABLED

void commBootI2cScan(void) { }

#endif // DIAGNOSTICS_ENABLED


// ============================================================================
// FRAME PROCESSING
// ============================================================================
//
//   1. Frame  : printable ASCII, "*XXXX" trailer, leading '{', CRC  -> ERROR
//   2. Syntax : flat JSON object as protocol.cpp defines it          -> ERROR
//   3. Envelope: seq (1..65535), type == "COMMAND", cmd (string)     -> ERROR
//   4. Sequence order: same seq       -> ACK DUPLICATE, no-op
//                      seq behind     -> ACK REJECTED STALE_SEQ, no-op
//   5. Command fields                          -> ACK REJECTED / GATED / ...
//
// Only a frame that clears all five can reach a handler, and only the
// movement handlers can refresh the watchdog.
//
// MOTORTEST cancellation keeps the baseline's conservative rule: any received
// line cancels a running test, EXCEPT a valid MOTORTEST (which replaces it)
// and a duplicate or stale frame (a replay must not change anything).
// ============================================================================

static void badEnvelope(const char *reason, long seq, const char *field)
{
    gStats.rxBadMessage++;
    endMotorTest("CANCELLED");
    emitError(reason, seq, field);
}

void handleCommand(const char *line, size_t len)
{
    ProtoStatus st = protoParseFrame(line, len, &gMsg);

    if (st == PROTO_EMPTY) {
        gStats.rxEmpty++;
        return;
    }

    if (st != PROTO_OK) {
        if (st == PROTO_BAD_FRAME)      gStats.rxBadFrame++;
        else if (st == PROTO_BAD_CRC)   gStats.rxBadCrc++;
        else                            gStats.rxBadMessage++;

        endMotorTest("CANCELLED");
        emitError(protoStatusReason(st), -1, NULL);
        return;
    }

    // ---- Envelope: seq ----
    const ProtoField *fSeq = protoFindField(&gMsg, "seq");
    if (fSeq == NULL) {
        badEnvelope("MISSING_FIELD", -1, "seq");
        return;
    }
    if (fSeq->type != PV_INT) {
        badEnvelope("WRONG_TYPE", -1, "seq");
        return;
    }
    if (fSeq->intOverflow ||
        fSeq->intValue < PROTO_SEQ_MIN || fSeq->intValue > PROTO_SEQ_MAX) {
        badEnvelope("INVALID_SEQUENCE", -1, "seq");
        return;
    }
    const long seq = (long)fSeq->intValue;

    // ---- Envelope: type ----
    const ProtoField *fType = protoFindField(&gMsg, "type");
    if (fType == NULL) {
        badEnvelope("MISSING_FIELD", seq, "type");
        return;
    }
    if (fType->type != PV_STR) {
        badEnvelope("WRONG_TYPE", seq, "type");
        return;
    }
    if (strcmp(fType->str, "COMMAND") != 0) {
        badEnvelope("INVALID_MESSAGE_TYPE", seq, "type");
        return;
    }

    // ---- Envelope: cmd ----
    const ProtoField *fCmd = protoFindField(&gMsg, "cmd");
    if (fCmd == NULL) {
        badEnvelope("MISSING_FIELD", seq, "cmd");
        return;
    }
    if (fCmd->type != PV_STR) {
        badEnvelope("WRONG_TYPE", seq, "cmd");
        return;
    }

    gCurSeq = (uint16_t)seq;
    gCurCmd = fCmd->str;

    // ---- Sequence order ----
    // A seq must ADVANCE relative to the last ACKed one. The same seq is a
    // duplicate (a retransmission); one that is behind is stale (a replay of
    // an older frame). Neither is executed, neither refreshes the watchdog,
    // and a running MOTORTEST is neither restarted nor cancelled.
    if (gHaveLastSeq) {
        uint16_t steps = protoSeqDistance(gLastSeq, gCurSeq);

        if (steps == 0) {
            gStats.rxDuplicates++;

            txBegin("ACK", seq);
            tx(",\"seq\":%u", (unsigned)gCurSeq);
            txStr("cmd", gCurCmd);
            txStr("result", "DUPLICATE");
            txStr("reason", "DUPLICATE_SEQ");
            txStr("original_cmd", gLastSeqCmd);
            txStr("original_result", gLastSeqResult);
            txEnd();
            return;
        }

        if (steps > PROTO_SEQ_WINDOW) {
            // Built by hand rather than through ackReject(): the recorded
            // result of the LAST seq must not be overwritten by a replay.
            gStats.rxStale++;
            gLastRejectReason = "STALE_SEQ";

            txBegin("ACK", seq);
            tx(",\"seq\":%u", (unsigned)gCurSeq);
            txStr("cmd", gCurCmd);
            txStr("result", "REJECTED");
            txStr("reason", "STALE_SEQ");
            tx(",\"last_seq\":%u", (unsigned)gLastSeq);
            txEnd();
            return;
        }
    }

    gHaveLastSeq   = true;
    gLastSeq       = gCurSeq;
    gLastSeqResult = "NONE";
    strncpy(gLastSeqCmd, gCurCmd, sizeof(gLastSeqCmd) - 1);
    gLastSeqCmd[sizeof(gLastSeqCmd) - 1] = '\0';

    gStats.rxOk++;

    if (strcmp(gCurCmd, "MOTORTEST") != 0) {
        endMotorTest("CANCELLED");
    }

    // ---- Dispatch ----
    if      (strcmp(gCurCmd, "STOP")      == 0) handleStop();
    else if (strcmp(gCurCmd, "DRIVE")     == 0) handleDrive();
    else if (strcmp(gCurCmd, "MOVE")      == 0) handleMove();
    else if (strcmp(gCurCmd, "MOTORTEST") == 0) handleMotorTest();
    else if (strcmp(gCurCmd, "RESET")     == 0) handleReset();
    else if (strcmp(gCurCmd, "PING")      == 0) handlePing();
#if DIAGNOSTICS_ENABLED
    else if (strcmp(gCurCmd, "I2CSCAN")   == 0) handleI2cScan();
    else if (strcmp(gCurCmd, "I2CSTATUS") == 0) handleI2cStatus();
    else if (strcmp(gCurCmd, "TCATEST")   == 0) handleTcaTest();
    else if (strcmp(gCurCmd, "PCATEST")   == 0) handlePcaTest();
    else if (strcmp(gCurCmd, "TOFTEST")   == 0) handleTofTest();
    else if (strcmp(gCurCmd, "HWREPORT")  == 0) handleHwReport();
#endif
    else {
        // A well-formed command the firmware does not know. It does NOT
        // refresh the watchdog.
        ackReject("UNKNOWN_COMMAND", "cmd");
    }
}


// ============================================================================
// NON-BLOCKING LINE READER
// ============================================================================
//
// Bytes accumulate until '\n'. A line longer than COMM_LINE_MAX is discarded
// in full -- never truncated into something that might parse -- and answered
// with one FRAME_TOO_LONG error when its terminator finally arrives.
// ============================================================================

void commPoll(void)
{
    // Expire a running motor test even if no bytes arrive.
    if (gTestActive && (int32_t)(millis() - gTestDeadline) >= 0) {
        endMotorTest("EXPIRED");
    }

    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\n') {
            if (gOverflow) {
                gOverflow = false;
                gLineLen  = 0;
                gStats.rxTooLong++;
                endMotorTest("CANCELLED");
                emitError("FRAME_TOO_LONG", -1, NULL);
                continue;
            }
            handleCommand(gLine, gLineLen);
            gLineLen = 0;
            continue;
        }

        if (gOverflow) {
            continue;                       // discarding an oversized line
        }

        if (gLineLen < COMM_LINE_MAX) {
            gLine[gLineLen++] = c;
        } else {
            gOverflow = true;
            gLineLen  = 0;
        }
    }
}


// ============================================================================
// READY / TIMEOUT EVENTS
// ============================================================================

void commEmitReady(void)
{
    txBegin("EVENT", -1);
    txStr("event", "READY");
    txSeq(-1);
    tx(",\"uptime_ms\":%lu,\"proto\":%d", (unsigned long)millis(), PROTO_VERSION);
    txStr("fw", "rover-esp32");
    txStr("core", "2.0.14");
    txStr("drive", "DIFFERENTIAL");
    tx(",\"seq_min\":%d,\"seq_max\":%ld,\"line_max\":%d",
       PROTO_SEQ_MIN, (long)PROTO_SEQ_MAX, (int)COMM_LINE_MAX);
    txBool("i2c_ready", roverI2cReady());
    txStr("tca_status", tcaStatusName());
    txStr("pca_status", pca9685StatusName());
    txBool("motor_drive_available", motorDriveAvailable());
    txStr("rear_backend", rearBackendName());
    txBool("rear_available", safetyRearSensingAvailable());
    tx(",\"rear_sensor_status\":[\"%s\",\"%s\",\"%s\"]",
       rearTofStatusName(rearTofStatusOf(REAR_TOF_0)),
       rearTofStatusName(rearTofStatusOf(REAR_TOF_1)),
       rearTofStatusName(rearTofStatusOf(REAR_TOF_2)));
    txEnd();
}

void commSetTimedOut(bool timedOut)
{
    // Announce the failsafe once, on the transition -- not on every pass.
    if (timedOut && !gTimedOut) {
        txBegin("EVENT", -1);
        txStr("event", "COMMAND_TIMEOUT");
        txSeq(-1);
        tx(",\"uptime_ms\":%lu,\"command_age_ms\":%lu,\"timeout_ms\":%lu",
           (unsigned long)millis(), (unsigned long)commMsSinceLastCommand(),
           (unsigned long)COMMAND_TIMEOUT_MS);
        txEnd();
    }
    gTimedOut = timedOut;
}


// ============================================================================
// TELEMETRY
// ============================================================================
//
// FAST (type TELEMETRY), every TELEMETRY_INTERVAL_MS: what the Pi needs to
// drive -- state, commanded vs applied motion, distances, gates, failsafe.
//
// DIAG (type DIAG), one section every TELEMETRY_DIAG_EVERY_N_FAST fast frames,
// rotating FRONT -> REAR -> SYSTEM: health, per-sensor detail, bus and
// configuration status, link statistics, calibration aids.
//
// Non-measurements are JSON null, never -1 and never a fabricated distance.
// ============================================================================

static void sendFastTelemetry(void)
{
    txBegin("TELEMETRY", -1);
    tx(",\"uptime_ms\":%lu", (unsigned long)millis());
    txStr("state", robotStateName(gState));
    txStr("block_reason", gBlockReason);
    if (gHaveLastSeq) {
        tx(",\"last_seq\":%u", (unsigned)gLastSeq);
    } else {
        tx(",\"last_seq\":null");
    }
    txStr("last_reject", gLastRejectReason);

    // left_cmd = what the Pi asked for; left_applied = what the motor layer
    // actually outputs. They differ when gated, below deadband, or with no
    // drive -- block_reason says which.
    tx(",\"left_cmd\":%d,\"right_cmd\":%d,\"left_applied\":%d,\"right_applied\":%d",
       gDesiredLeft, gDesiredRight, gAppliedLeft, gAppliedRight);

    // Side labels are UNVERIFIED (see DIAG FRONT "sensor_map_verified").
    txCm("front_left_cm",  sensorFilteredCm(US_FRONT_LEFT));
    txCm("front_right_cm", sensorFilteredCm(US_FRONT_RIGHT));
    txBool("front_valid",    !safetySensorFault());
    txBool("front_obstacle", safetyObstacleDetected());
    txBool("front_warning",  safetyWarning());

    // Index N is TCA CHANNEL N -- not left/centre/right.
    tx(",\"rear_mm\":[");
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        tx("%s", i ? "," : "");
        txMmValue(safetyRearDistanceMm(i));
    }
    tx("]");
    txBool("rear_available",    safetyRearSensingAvailable());
    txBool("rear_obstacle",     safetyRearObstacleDetected());
    txBool("rear_sensor_fault", safetyRearSensorFault());

    txBool("forward_blocked", safetyForwardBlocked());
    txBool("reverse_blocked", safetyReverseBlocked());

    // LATCHED. Cleared only by RESET or a new fully-permitted movement command.
    txBool("safety_stop", gSafetyStopLatched);

    tx(",\"command_age_ms\":%lu", (unsigned long)commMsSinceLastCommand());
    txBool("command_timeout", gTimedOut);
    txBool("motor_drive_available", motorDriveAvailable());
    txEnd();
}

static void txDiagFront(void)
{
    txBool("front_left_valid",  sensorValid(US_FRONT_LEFT));
    txBool("front_right_valid", sensorValid(US_FRONT_RIGHT));
    txStr("front_left_health",  sensorHealthName(sensorHealthOf(US_FRONT_LEFT)));
    txStr("front_right_health", sensorHealthName(sensorHealthOf(US_FRONT_RIGHT)));
    txStr("front_health",       sensorHealthName(sensorHealthWorst()));
    txCm("front_closest_cm", safetyClosestFrontCm());
    txBool("sensor_map_verified", sensorMapVerified());

#if TELEMETRY_INCLUDE_DEBUG
    // Raw = the single most recent sample, unfiltered. Never for control.
    txCm("front_left_raw_cm",  sensorRawCm(US_FRONT_LEFT));
    txCm("front_right_raw_cm", sensorRawCm(US_FRONT_RIGHT));
    tx(",\"front_left_samples\":%u,\"front_right_samples\":%u",
       (unsigned)sensorGoodSampleCount(US_FRONT_LEFT),
       (unsigned)sensorGoodSampleCount(US_FRONT_RIGHT));
    tx(",\"front_left_timeout_streak\":%u,\"front_right_timeout_streak\":%u",
       (unsigned)sensorTimeoutStreak(US_FRONT_LEFT),
       (unsigned)sensorTimeoutStreak(US_FRONT_RIGHT));
#endif
}

static void txDiagRear(void)
{
    txStr("rear_backend", rearBackendName());
    txBool("rear_orientation_verified", rearTofOrientationVerified());

    tx(",\"rear_sensor_valid\":[");
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        tx("%s%s", i ? "," : "", safetyRearSensorValid(i) ? "true" : "false");
    }
    tx("],\"rear_sensor_status\":[");
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        tx("%s\"%s\"", i ? "," : "", rearTofStatusName(rearTofStatusOf(i)));
    }
    tx("],\"rear_sensor_health\":[");
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        tx("%s\"%s\"", i ? "," : "", sensorHealthName(rearTofHealthOf(i)));
    }
    tx("],\"rear_sensor_obstacle\":[");
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        tx("%s%s", i ? "," : "", safetyRearSensorObstacle(i) ? "true" : "false");
    }
    tx("]");

    txStr("rear_health", sensorHealthName(rearTofHealthWorst()));
    txMm("rear_closest_mm", safetyClosestRearMm());
    txBool("rear_warning", safetyRearWarning());

#if TELEMETRY_INCLUDE_DEBUG
    tx(",\"rear_sensor_raw_mm\":[");
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        tx("%s", i ? "," : "");
        txMmValue(rearTofRawMm(i));
    }
    tx("],\"rear_fail_streak\":[%u,%u,%u],\"rear_tca_channels\":[%u,%u,%u]",
       (unsigned)rearTofFailStreak(REAR_TOF_0),
       (unsigned)rearTofFailStreak(REAR_TOF_1),
       (unsigned)rearTofFailStreak(REAR_TOF_2),
       (unsigned)rearTofChannelOf(REAR_TOF_0),
       (unsigned)rearTofChannelOf(REAR_TOF_1),
       (unsigned)rearTofChannelOf(REAR_TOF_2));
#endif
}

static void txDiagSystem(void)
{
    tx(",\"proto\":%d", PROTO_VERSION);
    txBool("i2c_ready", roverI2cReady());
    txStr("tca_status", tcaStatusName());
    txStr("pca_status", pca9685StatusName());
    txBool("tca_address_confirmed", tcaAddressConfirmed());
    txBool("pca_address_confirmed", pca9685AddressConfirmed());
    txStr("motor_drive_status", motorDriveStatusName());
    txBool("motor_map_verified", motorMapVerified());
    txBool("command_ever_received", gEverReceivedCmd);
    tx(",\"left_gated\":%d,\"right_gated\":%d", gGatedLeft, gGatedRight);

    tx(",\"link\":{\"rx_ok\":%lu,\"rx_empty\":%lu,\"rx_bad_frame\":%lu,"
       "\"rx_bad_crc\":%lu,\"rx_bad_message\":%lu,\"rx_too_long\":%lu,"
       "\"rx_rejected\":%lu,\"rx_duplicates\":%lu,\"rx_stale\":%lu,"
       "\"errors_suppressed\":%lu,\"tx_overflows\":%lu}",
       (unsigned long)gStats.rxOk, (unsigned long)gStats.rxEmpty,
       (unsigned long)gStats.rxBadFrame, (unsigned long)gStats.rxBadCrc,
       (unsigned long)gStats.rxBadMessage, (unsigned long)gStats.rxTooLong,
       (unsigned long)gStats.rxRejected, (unsigned long)gStats.rxDuplicates,
       (unsigned long)gStats.rxStale, (unsigned long)gStats.errorsSuppressed,
       (unsigned long)gStats.txOverflows);
}

#define DIAG_SECTION_COUNT 3

static void sendDiag(uint8_t section)
{
    static const char *const kNames[DIAG_SECTION_COUNT] = {
        "FRONT", "REAR", "SYSTEM"
    };

    txBegin("DIAG", -1);
    txStr("section", kNames[section]);
    tx(",\"uptime_ms\":%lu", (unsigned long)millis());

    switch (section) {
        case 0:  txDiagFront();  break;
        case 1:  txDiagRear();   break;
        default: txDiagSystem(); break;
    }
    txEnd();
}

void commServiceTelemetry(void)
{
    uint32_t now = millis();

    if (now - gLastFastMs >= TELEMETRY_INTERVAL_MS) {
        gLastFastMs = now;
        sendFastTelemetry();

        // Schedule the next DIAG section half an interval later, so it never
        // queues directly behind a fast frame.
        if (++gFastCount >= TELEMETRY_DIAG_EVERY_N_FAST) {
            gFastCount   = 0;
            gDiagPending = true;
            gDiagDueMs   = now + TELEMETRY_INTERVAL_MS / 2;
        }
    }

    if (gDiagPending && (int32_t)(now - gDiagDueMs) >= 0) {
        gDiagPending = false;
        sendDiag(gNextDiagSection);
        gNextDiagSection = (uint8_t)((gNextDiagSection + 1) % DIAG_SECTION_COUNT);
    }
}


// ============================================================================
// ACCESSORS
// ============================================================================

void commInit(void)
{
    gDesiredLeft       = 0;
    gDesiredRight      = 0;
    gGatedLeft         = 0;
    gGatedRight        = 0;
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
    gHaveLastSeq       = false;
    gLastSeq           = 0;
    gLastSeqResult     = "NONE";
    gLastSeqCmd[0]     = '\0';
    memset(&gStats, 0, sizeof(gStats));
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

bool commTimedOut(void)             { return gTimedOut;     }

void commSetState(RobotState s) { gState = s; }
RobotState commGetState(void)   { return gState; }

bool commTestModeActive(void) { return gTestActive;  }
int  commTestChannel(void)    { return gTestChannel; }
int  commTestSpeed(void)      { return gTestSpeed;   }

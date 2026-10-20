#include <Arduino.h>
#include "motor.h"
#include "pca9685.h"
#include "stall_guard.h"
#include "config.h"

// ============================================================================
//  CENTRALISED MOTOR CONFIGURATION / MAPPING
// ============================================================================
//
// THIS TABLE IS THE ONE PLACE the four motor channels are described. Pins,
// enable channels, side grouping and polarity all resolve here and nowhere
// else. Nothing below this table hard-codes a GPIO number.
//
//   channel   L298N direction inputs      L298N enable      configured from
//   -------   ------------------------    --------------    ---------------
//   FRONT_A   IN1 = GPIO23, IN2 = GPIO25  ENA = PCA CH0     config.h
//   FRONT_B   IN3 = GPIO26, IN4 = GPIO27  ENB = PCA CH1     config.h
//   REAR_A    IN1 = GPIO32, IN2 = GPIO33  ENA = PCA CH2     config.h
//   REAR_B    IN3 = GPIO16, IN4 = GPIO17  ENB = PCA CH3     config.h
//
// WHAT THE WIRING STILL DOES NOT TELL US
// --------------------------------------
//   SIDE   : is FRONT_A the front-LEFT wheel or the front-RIGHT? Nothing in
//            the pin numbering implies it. It depends on which driver output
//            terminal each motor was screwed into.
//   INVERT : does IN1 high / IN2 low spin that wheel forward or backward? That
//            depends on which way round the two motor leads went into the
//            terminal block.
//
// Both come from MOTOR_*_SIDE and MOTOR_*_INVERT in config.h, both are still
// PLACEHOLDERS, and telemetry reports "motor_map_verified":false until a human
// sets MOTOR_MAP_VERIFIED. Resolve them with MOTORTEST, rover ON BLOCKS.
//
// NOTE on ESP32-WROVER modules: GPIO16/17 are consumed by PSRAM and are NOT
// usable. This assumes an ESP32-WROOM-32 class module where 16/17 are free.
// If REAR_B never responds, check the module type before suspecting firmware.
// ============================================================================

typedef struct {
    const char *name;
    uint8_t     pinIn1;     // ESP32 GPIO -> L298N INx   (DIRECTION, digital)
    uint8_t     pinIn2;     // ESP32 GPIO -> L298N INy   (DIRECTION, digital)
    uint8_t     enableCh;   // PCA9685 channel -> L298N ENA/ENB  (SPEED, PWM)
    MotorSide   side;       // from config.h -- UNVERIFIED
    bool        invert;     // from config.h -- UNVERIFIED
} MotorChannel;

static MotorChannel gMotors[MOTOR_CHANNEL_COUNT] = {
    { "FRONT_A", M_FRONT_A_IN1, M_FRONT_A_IN2, PCA_CH_FRONT_A_EN,
      (MotorSide)MOTOR_FRONT_A_SIDE, (MOTOR_FRONT_A_INVERT != 0) },

    { "FRONT_B", M_FRONT_B_IN1, M_FRONT_B_IN2, PCA_CH_FRONT_B_EN,
      (MotorSide)MOTOR_FRONT_B_SIDE, (MOTOR_FRONT_B_INVERT != 0) },

    { "REAR_A",  M_REAR_A_IN1,  M_REAR_A_IN2,  PCA_CH_REAR_A_EN,
      (MotorSide)MOTOR_REAR_A_SIDE,  (MOTOR_REAR_A_INVERT != 0) },

    { "REAR_B",  M_REAR_B_IN1,  M_REAR_B_IN2,  PCA_CH_REAR_B_EN,
      (MotorSide)MOTOR_REAR_B_SIDE,  (MOTOR_REAR_B_INVERT != 0) }
};


// ============================================================================
// HELPERS
// ============================================================================

static int clampSpeed(int value)
{
    if (value >  PWM_MAX_DUTY) return  PWM_MAX_DUTY;
    if (value < -PWM_MAX_DUTY) return -PWM_MAX_DUTY;
    return value;
}

// Below the minimum effective duty a brushed motor stalls and buzzes rather
// than turning, which wastes current and sounds like a fault. Treat as stop.
static int applyDeadband(int value)
{
    if (value > 0 && value <  MOTOR_MIN_EFFECTIVE_DUTY) return 0;
    if (value < 0 && value > -MOTOR_MIN_EFFECTIVE_DUTY) return 0;
    return value;
}

// Map the -255..+255 API magnitude onto the PCA9685's 12-bit duty. The API
// scale is unchanged so the Pi protocol is untouched; only the hardware behind
// it is different.
static uint16_t speedToDuty12(int magnitude)
{
    if (magnitude <= 0) {
        return 0;
    }
    if (magnitude >= PWM_MAX_DUTY) {
        return (uint16_t)PCA9685_PWM_MAX;
    }

    return (uint16_t)(((uint32_t)magnitude * (uint32_t)PCA9685_PWM_MAX)
                      / (uint32_t)PWM_MAX_DUTY);
}


// ============================================================================
// ONE CHANNEL
// ============================================================================
//
// STOPPING IS GPIO FIRST (RX-02). The direction inputs are plain ESP32 GPIO
// and are written instantly; the enable is a PCA9685 PWM output reached over
// I2C, and one hung I2C transaction can take >= 1 s on this core (config.h
// I2C_TIMEOUT_MS). So whenever a channel is to stop -- or to reverse -- both of
// its direction inputs are driven LOW BEFORE any I2C write is attempted.
//
// L298N truth table (datasheet): IN1 == IN2 with the enable HIGH is "fast
// motor stop" (the bridge brakes the motor); with the enable LOW the motor
// free-runs. So between the GPIO cutoff and the enable write the motor brakes
// -- normally for well under a millisecond, and for as long as the I2C write
// takes if the bus is hung, which is exactly when stopping must not wait on
// it. Once the enable reaches 0 the channel coasts, the same end state as
// before. Direction is never changed on a live bridge: IN1/IN2 only ever go
// (H,L) or (L,H) -> (L,L), never straight to the opposite pair.
//
// DRIVING keeps the original order for a channel already turning that way:
//
//   1. ENABLE TO ZERO FIRST. Flipping direction on a LIVE bridge is how you
//      get a current reversal straight through a spinning motor -- hard on the
//      L298N, hard on the gearbox, and a supply-rail dip that can brown out
//      the ESP32 mid-safety-check.
//   2. SET DIRECTION. Plain digitalWrite. These pins are never PWM'd.
//   3. RAISE THE ENABLE to the commanded duty.
//
// A FAILED ENABLE WRITE, at any step of any channel, leaves every enable in an
// unknown state. applyChannelSpeeds() then drives ALL EIGHT direction inputs
// LOW before it returns -- including channels it already wrote earlier in the
// same call -- and writes nothing more. A failed PCA9685 write also latches
// BUS_ERROR in pca9685.cpp, so motorDriveAvailable() is false, and every later
// call goes through stopMotors(), until the chip is recovered
// (motorDriveRecoveryService()).
//
// The failure is also RECORDED here, at the instant it happens: if any
// direction input was driven at that moment, or the call was asking for any
// non-zero speed, motion was lost. comm.cpp turns that record into the
// safety_stop latch
// (motorConsumeLostWhileLive()). Recording it here, rather than inferring it
// later from what comm.cpp last commanded, means no command processed in
// between -- a DRIVE, STOP or PING arriving during a hung write -- can erase
// the evidence.
// ============================================================================

// The direction each channel's inputs were last set to: +1, -1 or 0 (both LOW).
static int8_t gDir[MOTOR_CHANNEL_COUNT] = { 0, 0, 0, 0 };

// A PCA9685 write failed while output was live, not yet consumed by comm.cpp.
static bool gLostWhileLive = false;

static int channelIndexOf(const MotorChannel *m)
{
    return (int)(m - gMotors);
}

static int signOf(int v)
{
    return (v > 0) - (v < 0);
}

// GPIO only: both direction inputs LOW. Never touches the I2C bus.
static void cutDirection(const MotorChannel *m)
{
    digitalWrite(m->pinIn1, LOW);
    digitalWrite(m->pinIn2, LOW);
    gDir[channelIndexOf(m)] = 0;
}

static void cutAllDirections(void)
{
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        cutDirection(&gMotors[i]);
    }
}

// Called the moment an enable write fails, BEFORE any direction is cut for
// it: was any direction input driven, i.e. could any wheel be turning?
static void noteWriteFailure(void)
{
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        if (gDir[i] != 0) {
            gLostWhileLive = true;
            return;
        }
    }
}

// The loop-stall guard is not operational: never started, or its task has
// stopped ticking (stallGuardOperational()). Nothing could then cut a stalled
// loop's output, so output live right now -- or a non-zero request being
// refused for it -- is a lost drive, recorded like a failed write. Called
// BEFORE anything is cut, so a STOP or RESET processed meanwhile cannot erase
// the evidence.
static void noteGuardDown(bool motionRequested)
{
    if (stallGuardOperational()) {
        return;
    }
    if (motionRequested) {
        gLostWhileLive = true;
        return;
    }
    noteWriteFailure();                 // any direction input driven?
}

// Returns false if a PCA9685 write for this channel failed (the failure has
// been recorded; the caller cuts every channel), or if the loop-stall guard
// has tripped and refused to raise its direction inputs.
static bool writeChannel(const MotorChannel *m, int speed)
{
    // ---- Stop: direction LOW first (GPIO), then the enable (I2C). ----
    if (speed == 0) {
        cutDirection(m);
        if (!pca9685SetDuty(m->enableCh, 0)) {
            noteWriteFailure();         // other channels may still be driven
            return false;
        }
        return true;
    }

    // ---- 1. Enable off. ----
    if (!pca9685SetDuty(m->enableCh, 0)) {
        noteWriteFailure();             // this channel may still be turning
        cutDirection(m);                // enable state unknown: hold it stopped
        return false;
    }

    // ---- 2. Direction. ----
    // THE ONLY direction-HIGH write in the firmware, and it goes through the
    // stall guard: checked against the trip flag and written inside the same
    // cross-core critical section the guard cuts in (stall_guard.h). If the
    // guard has tripped -- e.g. while step 1 above was a hung I2C write that
    // has only now returned -- nothing is raised.
    bool raised = (speed > 0)
                ? stallGuardSetDirection(m->pinIn1, HIGH, m->pinIn2, LOW)
                : stallGuardSetDirection(m->pinIn1, LOW,  m->pinIn2, HIGH);
    if (!raised) {
        cutDirection(m);
        return false;
    }
    gDir[channelIndexOf(m)] = (int8_t)signOf(speed);

    // ---- 3. Speed. ----
    int magnitude = (speed < 0) ? -speed : speed;
    if (!pca9685SetDuty(m->enableCh, speedToDuty12(magnitude))) {
        noteWriteFailure();             // the direction was just set: live
        return false;
    }
    return true;
}

// Apply one speed per channel (already clamped, deadbanded and polarity-
// mapped). Every channel that stops or reverses has its direction cut over
// GPIO in a first pass, BEFORE the first I2C write of the second pass.
//
// The FIRST failed write ends the call: all eight direction inputs are driven
// LOW right then -- channels written earlier in this call included -- and no
// further write is attempted. Nothing that runs after this function (the rear
// sensor poll, GPS, the next pass) can find a channel still driven.
//
// It is recorded as a drive loss (gLostWhileLive) if output was live at the
// failure (writeChannel() has already checked that) OR this call was asking
// for any non-zero speed: a motion request that failed is a loss whichever
// transaction failed, including the very first one from rest. A call whose
// targets are all zero, with nothing live, is a pure stop -- a failure there
// is a bus fault, not a lost motion request, and is not recorded.
static void applyChannelSpeeds(const int speeds[MOTOR_CHANNEL_COUNT])
{
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        if (signOf(speeds[i]) != gDir[i]) {
            cutDirection(&gMotors[i]);
        }
    }
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        if (!writeChannel(&gMotors[i], speeds[i])) {
            for (int k = 0; k < MOTOR_CHANNEL_COUNT; k++) {
                if (speeds[k] != 0) gLostWhileLive = true;
            }
            cutAllDirections();
            return;
        }
    }
}

bool motorConsumeLostWhileLive(void)
{
    // A loop-stall guard trip is a drive loss too: it only fires while a
    // direction input is HIGH (stall_guard.cpp).
    bool guardTrip = stallGuardConsumeTripEvidence();
    bool lost      = gLostWhileLive || guardTrip;
    gLostWhileLive = false;
    return lost;
}


// ============================================================================
// INITIALISATION  --  BOOT SAFETY CRITICAL
// ============================================================================
//
// Ordering:
//
//   1. pinMode(OUTPUT) + digitalWrite(LOW) on all eight direction inputs.
//      They are actively held low immediately; none is left floating.
//   2. pca9685Init(), which forces all sixteen PWM outputs fully off as its
//      very first act -- before setting the frequency, because on a warm ESP32
//      reset the PCA9685 may never have lost power and could still be holding
//      an enable line high from the previous run.
//   3. stopMotors() as an explicit belt-and-braces final zero.
//
// NOTE on ESP32 power-up (before setup() runs): these GPIOs are high-impedance
// inputs for the few hundred ms of bootloader time. If the driver boards do
// not have their own input pull-downs, fit 10k pull-downs to GND on all eight
// IN lines. Firmware cannot cover the window before it is running.
//
// The same applies to the PCA9685's outputs and its OE pin, which this rover
// has NOT confirmed. If OE is left floating rather than tied low, the outputs
// are undefined at power-on and no amount of firmware fixes it.
// ============================================================================

void motorInit(void)
{
    // Step 1 - hold every direction input low, right now.
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        pinMode(gMotors[i].pinIn1, OUTPUT);
        digitalWrite(gMotors[i].pinIn1, LOW);

        pinMode(gMotors[i].pinIn2, OUTPUT);
        digitalWrite(gMotors[i].pinIn2, LOW);
        gDir[i] = 0;
    }
    gLostWhileLive = false;             // no motion has been lost this boot

    // Step 2 - bring up the enable source. Returns false when the address is
    // unconfirmed or the chip is absent; that is reported, not worked around.
    (void)pca9685Init();

    // Step 3 - explicit stop of every channel.
    stopMotors();
}


bool motorDriveAvailable(void)
{
    // Fail closed: no loop-stall guard task ticking, no motion (a guard that
    // never started, or has stopped, could not cut anything).
    return pca9685Ready() && stallGuardOperational();
}

const char *motorDriveStatusName(void)
{
    if (!stallGuardOperational()) {
        return "STALL_GUARD_NOT_RUNNING";
    }
    return pca9685StatusName();
}

void motorStopUnavailable(bool motionRequested)
{
    noteGuardDown(motionRequested);
    stopMotors();
}

bool motorDriveRecoveryService(void)
{
    if (!pca9685ServiceRecovery()) {
        return false;
    }

    // Belt and braces, as in motorInit(): the bring-up already forced every
    // output full-off; this also re-zeroes each enable channel and re-asserts
    // every direction input LOW through the now-working chip.
    stopMotors();
    return true;
}


// ============================================================================
// DIFFERENTIAL DRIVE
// ============================================================================
//
// Differential drive rover. There is NO steering mechanism. All turning is
// produced by the difference between left and right wheel speeds:
//
//   FORWARD       left= 150  right= 150
//   ARC LEFT      left=  80  right= 150
//   ARC RIGHT     left= 150  right=  80
//   PIVOT LEFT    left=-100  right= 100
//   PIVOT RIGHT   left= 100  right=-100
//   STOP          left=   0  right=   0
//
// Both motors on a side are always driven identically.
// ============================================================================

void driveDifferential(int leftSpeed, int rightSpeed)
{
    // No verified enable source means no motion, full stop. Not a degraded
    // half-speed mode and not a best-effort attempt: the chip that gates every
    // half-bridge has not been identified.
    if (!motorDriveAvailable()) {
        motorStopUnavailable(leftSpeed != 0 || rightSpeed != 0);
        return;
    }

    leftSpeed  = applyDeadband(clampSpeed(leftSpeed));
    rightSpeed = applyDeadband(clampSpeed(rightSpeed));

    int speeds[MOTOR_CHANNEL_COUNT];
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        int speed = (gMotors[i].side == SIDE_LEFT) ? leftSpeed : rightSpeed;

        if (gMotors[i].invert) {
            speed = -speed;
        }

        speeds[i] = speed;
    }
    applyChannelSpeeds(speeds);
}


void stopMotors(void)
{
    // 0. Evidence before the cut: live output with no guard running is a loss.
    noteGuardDown(false);

    // 1. GPIO first: every direction input LOW, for all four channels, before
    //    anything touches the I2C bus (see "ONE CHANNEL" above). This part
    //    cannot be held up by a hung bus.
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        cutDirection(&gMotors[i]);
    }

    // 2. Then the enables, in ONE transaction where possible. pca9685AllOff()
    //    kills all sixteen outputs with a single write, which is materially
    //    faster than four separate channel writes.
    (void)pca9685AllOff();

    // 3. Belt and braces: also zero each channel, so a stop is correct even if
    //    the ALL_LED write above failed.
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        (void)pca9685SetDuty(gMotors[i].enableCh, 0);
    }
}


// ============================================================================
// LEGACY ARC HELPERS
// ============================================================================
//
// These are NOT steering. They just pick an inner-wheel speed. Kept because
// the legacy MOVE protocol uses the same ratio; the native DRIVE protocol
// bypasses them entirely and lets the Pi set both wheels explicitly.
// ============================================================================

int motorArcInnerSpeed(int speed)
{
    return (speed * MOTOR_TURN_INNER_PCT) / 100;
}

void moveForward(int speed)
{
    driveDifferential(speed, speed);
}

void moveBackward(int speed)
{
    driveDifferential(-speed, -speed);
}

void turnLeft(int speed)
{
    // Forward arc to the left: inner (left) side slowed, not reversed.
    driveDifferential(motorArcInnerSpeed(speed), speed);
}

void turnRight(int speed)
{
    driveDifferential(speed, motorArcInnerSpeed(speed));
}


// ============================================================================
// COMMISSIONING
// ============================================================================

void motorDriveChannel(int channelIndex, int speed)
{
    if (!motorDriveAvailable()) {
        motorStopUnavailable(speed != 0);
        return;
    }

    speed = applyDeadband(clampSpeed(speed));

    // Deliberately does NOT apply side/invert mapping. This drives the raw
    // hardware channel so you can observe its true behaviour and then fill in
    // MOTOR_*_SIDE / MOTOR_*_INVERT in config.h from observation.
    int speeds[MOTOR_CHANNEL_COUNT];
    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        speeds[i] = (i == channelIndex) ? speed : 0;
    }
    applyChannelSpeeds(speeds);
}

const char *motorChannelName(int channelIndex)
{
    if (channelIndex < 0 || channelIndex >= MOTOR_CHANNEL_COUNT) {
        return "?";
    }
    return gMotors[channelIndex].name;
}

const char *motorChannelSideName(int channelIndex)
{
    if (channelIndex < 0 || channelIndex >= MOTOR_CHANNEL_COUNT) {
        return "?";
    }
    return (gMotors[channelIndex].side == SIDE_LEFT) ? "LEFT" : "RIGHT";
}

uint8_t motorChannelIn1Pin(int channelIndex)
{
    if (channelIndex < 0 || channelIndex >= MOTOR_CHANNEL_COUNT) return 0xFF;
    return gMotors[channelIndex].pinIn1;
}

uint8_t motorChannelIn2Pin(int channelIndex)
{
    if (channelIndex < 0 || channelIndex >= MOTOR_CHANNEL_COUNT) return 0xFF;
    return gMotors[channelIndex].pinIn2;
}

uint8_t motorChannelEnableCh(int channelIndex)
{
    if (channelIndex < 0 || channelIndex >= MOTOR_CHANNEL_COUNT) return 0xFF;
    return gMotors[channelIndex].enableCh;
}

int motorEffectivePower(int requested)
{
    return applyDeadband(clampSpeed(requested));
}

bool motorMapVerified(void)
{
    return (MOTOR_MAP_VERIFIED != 0);
}

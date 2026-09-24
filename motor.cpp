#include <Arduino.h>
#include "motor.h"
#include "pca9685.h"
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
// SEQUENCE MATTERS, and it is the same every time:
//
//   1. ENABLE TO ZERO FIRST. The half-bridge is switched off before its
//      direction inputs change. Flipping direction on a LIVE bridge is how you
//      get a current reversal straight through a spinning motor -- hard on the
//      L298N, hard on the gearbox, and a supply-rail dip that can brown out
//      the ESP32 mid-safety-check.
//   2. SET DIRECTION. Plain digitalWrite. These pins are never PWM'd.
//   3. RAISE THE ENABLE to the commanded duty.
//
// For a stop, step 3 simply writes zero and both direction pins go low, which
// leaves the bridge coasting rather than braking -- same behaviour the
// previous firmware had.
//
// If the PCA9685 is not available this still drives the direction pins low and
// returns without pretending anything moved.
// ============================================================================

static void writeChannel(const MotorChannel *m, int speed)
{
    // ---- 1. Enable off. ----
    (void)pca9685SetDuty(m->enableCh, 0);

    // ---- 2. Direction. ----
    if (speed > 0) {
        digitalWrite(m->pinIn1, HIGH);
        digitalWrite(m->pinIn2, LOW);
    } else if (speed < 0) {
        digitalWrite(m->pinIn1, LOW);
        digitalWrite(m->pinIn2, HIGH);
    } else {
        digitalWrite(m->pinIn1, LOW);
        digitalWrite(m->pinIn2, LOW);
        return;                         // enable already zero -- done
    }

    // ---- 3. Speed. ----
    int magnitude = (speed < 0) ? -speed : speed;
    (void)pca9685SetDuty(m->enableCh, speedToDuty12(magnitude));
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
    }

    // Step 2 - bring up the enable source. Returns false when the address is
    // unconfirmed or the chip is absent; that is reported, not worked around.
    (void)pca9685Init();

    // Step 3 - explicit stop of every channel.
    stopMotors();
}


bool motorDriveAvailable(void)
{
    return pca9685Ready();
}

const char *motorDriveStatusName(void)
{
    return pca9685StatusName();
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
        stopMotors();
        return;
    }

    leftSpeed  = applyDeadband(clampSpeed(leftSpeed));
    rightSpeed = applyDeadband(clampSpeed(rightSpeed));

    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        int speed = (gMotors[i].side == SIDE_LEFT) ? leftSpeed : rightSpeed;

        if (gMotors[i].invert) {
            speed = -speed;
        }

        writeChannel(&gMotors[i], speed);
    }
}


void stopMotors(void)
{
    // Enables first, in ONE transaction where possible. pca9685AllOff() kills
    // all sixteen outputs with a single write, which is materially faster than
    // four separate channel writes on the emergency path.
    (void)pca9685AllOff();

    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        digitalWrite(gMotors[i].pinIn1, LOW);
        digitalWrite(gMotors[i].pinIn2, LOW);

        // Belt and braces: also zero the individual channel, so a stop is
        // correct even if the ALL_LED write above failed.
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
        stopMotors();
        return;
    }

    speed = applyDeadband(clampSpeed(speed));

    for (int i = 0; i < MOTOR_CHANNEL_COUNT; i++) {
        // Deliberately does NOT apply side/invert mapping. This drives the
        // raw hardware channel so you can observe its true behaviour and then
        // fill in MOTOR_*_SIDE / MOTOR_*_INVERT in config.h from observation.
        writeChannel(&gMotors[i], (i == channelIndex) ? speed : 0);
    }
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

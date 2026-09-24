// ============================================================================
//  ESP32 ROVER - LOW LEVEL MOTOR + SAFETY CONTROLLER
// ============================================================================
//
//  Board       : ESP32 Dev Module (ESP32-WROOM-32 class)
//  Core        : esp32 by Espressif, 2.0.14
//                No LEDC channel is used at all: motor speed now comes from
//                the PCA9685. If you ever add an LEDC user, this core takes
//                ledcSetup / ledcAttachPin / ledcWrite(channel, duty) -- the
//                core 3.x ledcAttach(pin,freq,res) form does not exist here.
//  Upload speed: 921600 (or 115200 if uploads are unreliable)
//
//  REQUIRED LIBRARY
//  ----------------
//  VL53L0X by Pololu (Library Manager, or github.com/pololu/vl53l0x-arduino).
//  It carries ST's device initialisation and tuning sequence, which is several
//  hundred register writes and is not something to reproduce by hand.
//
//  DRIVE TYPE  : DIFFERENTIAL. No steering mechanism exists. Every turn is
//                produced by the difference between left and right wheel
//                speeds. Nothing here assumes a steering angle.
//
//  RESPONSIBILITY SPLIT
//  --------------------
//  ESP32 (this firmware) : motor control, front ultrasonic sensing, immediate
//                          safety stop, command failsafe, telemetry.
//  Raspberry Pi          : camera / computer vision, navigation, mission
//                          logic. The Pi SENDS movement commands. It is not a
//                          safety authority.
//
//  The ESP32 is the FINAL SAFETY AUTHORITY. Every command from the Pi is
//  re-checked against the sensors on every pass of loop(), not just at the
//  moment it arrives.
//
//  SENSING
//  -------
//  FRONT : two HC-SR04 ultrasonic (GPIO13/14 and GPIO18/19), gating FORWARD.
//  REAR  : three VL53L0X time-of-flight rangefinders, all at I2C 0x29,
//          reached through a TCA9548A multiplexer on CH0/CH1/CH2, gating
//          REVERSE. Each is read with its own channel selected; no two are
//          ever addressed with more than one channel open.
//
//  MOTORS
//  ------
//  ESP32 GPIO -> L298N IN1..IN4 : DIRECTION ONLY, plain digitalWrite.
//  PCA9685 CH0..CH3 -> ENA/ENB  : SPEED, PWM.
//  The IN pins are never PWM'd. See the motor section of config.h.
//
//  KNOWN LIMITATIONS -- read these before trusting the rover
//  ---------------------------------------------------------
//  1. TWO I2C ADDRESSES ARE UNCONFIRMED and are not guessed anywhere:
//       * TCA9548A -- until TCA9548A_ADDRESS_CONFIRMED is 1, NO rear sensor
//         is read, every reading is TOF_UNINITIALISED, and reverse falls back
//         to the SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED policy.
//       * PCA9685  -- until PCA9685_ADDRESS_CONFIRMED is 1, NO WHEEL CAN
//         TURN. The enable inputs of both L298N boards are fed from this chip,
//         so without a verified one there is no speed path at all. Telemetry
//         reports "motor_drive_available":false and the state is
//         DRIVE_UNAVAILABLE.
//     Find both with {"cmd":"I2CSCAN"}, then {"cmd":"TCATEST"} / PCATEST.
//  2. MOTOR MAP UNVERIFIED. The left/right grouping and rotation polarity in
//     config.h are a placeholder assumption until confirmed with the
//     MOTORTEST command, rover ON BLOCKS. Telemetry reports
//     "motor_map_verified":false.
//  3. FRONT SENSOR SIDE MAP UNVERIFIED. The wiring tells us which GPIO pair
//     is which SENSOR, but not which sensor sits on the left or right of the
//     front assembly. Telemetry reports "sensor_map_verified":false. This
//     affects labels only -- either sensor alone blocks forward motion.
//  4. REAR SENSOR ORIENTATION UNVERIFIED. Indices 0/1/2 are TCA CHANNEL
//     numbers. Which one is physically left, centre or right is unknown --
//     resolve it with {"cmd":"TOFTEST"} and a hand behind one sensor.
//     Labels only: any one of the three blocks reverse.
//  5. REAR THRESHOLDS ARE COMMISSIONING VALUES. REAR_STOP_DISTANCE_MM and
//     friends are cautious starting points, NOT a measurement of this rover's
//     reverse stopping distance. Measure it before tightening them.
//  6. ULTRASONIC DISTANCES ARE UNCALIBRATED. The firmware reports what it
//     measures, using a fixed 343 m/s speed of sound. Real accuracy depends
//     on temperature, target surface and mounting angle, and has NOT been
//     verified against a tape measure. Compiling proves nothing about
//     accuracy -- run the calibration procedure before trusting a number.
// ============================================================================

#include <Arduino.h>

#include "config.h"
#include "comm.h"
#include "motor.h"
#include "safety.h"
#include "rover_i2c.h"
#include "tca9548a.h"
#include "pca9685.h"
#include "rear_tof.h"


void setup()
{
    // ------------------------------------------------------------------
    // 1. I2C FIRST.
    //    Both the motor enable source (PCA9685) and the rear sensors live on
    //    this bus, so it has to exist before either can be brought up. This
    //    is the only place Wire.begin() is called.
    // ------------------------------------------------------------------
    roverI2cInit();

    // ------------------------------------------------------------------
    // 2. MULTIPLEXER, before anything downstream of it.
    //    Leaves all channels deselected, so the main bus segment is clean
    //    when the PCA9685 is probed a moment later. A no-op (and a false
    //    return) while TCA9548A_ADDRESS_CONFIRMED is 0.
    // ------------------------------------------------------------------
    (void)tcaInit();

    // ------------------------------------------------------------------
    // 3. MOTORS, before anything that could take time.
    //    motorInit() drives all eight L298N direction inputs LOW, then brings
    //    up the PCA9685 whose very first act is forcing all sixteen PWM
    //    outputs fully off. The rover cannot twitch while we start up -- and
    //    on a WARM reset, where the PCA9685 never lost power and may still be
    //    holding an enable line high from the previous run, this is what
    //    clears it.
    // ------------------------------------------------------------------
    motorInit();
    stopMotors();

    // Both buffer calls MUST precede begin().
    //
    // TX: a worst-case telemetry line is ~2100 bytes; without the enlarged
    //     buffer the write blocks the control loop while it drains at 115200.
    // RX: the ring is filled by the UART ISR, so a blocking operation in
    //     loop() does not stop bytes arriving -- it stops them being consumed,
    //     and the ring then overflows silently. The 256-byte default is only
    //     22 ms of traffic at 115200, and a full bus scan blocks for ~25-45 ms.
    //     1024 bytes is ~89 ms of headroom, which covers it.
    Serial.setTxBufferSize(SERIAL_TX_BUFFER_BYTES);
    Serial.setRxBufferSize(SERIAL_RX_BUFFER_BYTES);
    Serial.begin(SERIAL_BAUD);

    // ------------------------------------------------------------------
    // 4. SENSORS.
    //    safetyInit() configures the front ultrasonic IO and then brings up
    //    the three rear VL53L0X, ONE AT A TIME behind its own TCA channel --
    //    they all answer at 0x29, so they can never be initialised together.
    // ------------------------------------------------------------------
    safetyInit();
    commInit();

    // Rover stays STOPPED after startup. STATE_IDLE means "booted, never
    // commanded". Nothing moves until a valid movement command arrives.
    commUpdateRuntime(0, 0, "NONE");

    // The READY event reports what actually came up, not what was hoped for.
    // If the Pi sees motor_drive_available:false here, the rover will not move
    // and no command will change that until the PCA9685 address is confirmed.
    // The boot ROM's plain-text output may precede it on the same UART.
    commEmitReady();

    // ------------------------------------------------------------------
    // 5. ONE-SHOT I2C BUS SCAN.
    //
    //    Last, so it prints AFTER the READY line and the operator sees the
    //    bus state in the same breath as the boot state.
    //
    //    Runs exactly once. It is deliberately NOT in loop(): a scan blocks
    //    for tens of milliseconds, and repeating that to re-answer a question
    //    whose answer has not changed would starve the safety gate for no
    //    information. Send {"cmd":"I2CSCAN"} when you want a fresh one.
    //
    //    Read-only: START + address + STOP per candidate, no data byte
    //    written to anything.
    // ------------------------------------------------------------------
#if I2C_BOOT_SCAN
    commBootI2cScan();
#endif
}


void loop()
{
    // ------------------------------------------------------------------
    // 1. COMMANDS (pre-sensor)
    //    Drain the UART before the sensor tick, which can block for up to
    //    one pulseIn timeout (~20 ms).
    // ------------------------------------------------------------------
    commPoll();

    // ------------------------------------------------------------------
    // 2. SENSORS
    //    Non-blocking scheduler: issues at most ONE front ping per
    //    US_PING_INTERVAL_MS, strictly alternating between the two
    //    HC-SR04 so they can never hear each other's burst.
    //
    //    It also collects at most ONE already-completed rear VL53L0X result
    //    per REAR_TOF_POLL_INTERVAL_MS, round-robin across the three sensors,
    //    each behind its own TCA channel. The sensors run in continuous mode,
    //    so this never waits for a measurement. It is a no-op while the
    //    TCA9548A address is unconfirmed.
    // ------------------------------------------------------------------
    safetyUpdate();

    // ------------------------------------------------------------------
    // 3. COMMANDS (post-sensor)
    //    Polling again immediately after the blocking window keeps the
    //    256-byte UART FIFO from overrunning at 115200 baud.
    // ------------------------------------------------------------------
    commPoll();

    // ------------------------------------------------------------------
    // 4. COMMISSIONING OVERRIDE
    //    MOTORTEST drives one raw channel for a bounded time. It expires by
    //    itself inside commPoll(). Rover must be ON BLOCKS.
    // ------------------------------------------------------------------
    if (commTestModeActive()) {
        motorDriveChannel(commTestChannel(), commTestSpeed());

        commServiceTelemetry();
        delay(1);
        return;
    }

    // ------------------------------------------------------------------
    // 5. COMMAND FAILSAFE
    //    Only a WELL-FORMED movement command (DRIVE / MOVE / STOP) refreshes
    //    the timer. Garbage, unknown commands, over-long lines, PING and
    //    RESET do not, so serial noise can never keep the rover running.
    //    On timeout: left = 0, right = 0.
    // ------------------------------------------------------------------
    if (commHasEverReceivedCommand() &&
        commMsSinceLastCommand() > COMMAND_TIMEOUT_MS) {

        commClearDesired();
        stopMotors();
        commSetTimedOut(true);
    }

    // ------------------------------------------------------------------
    // 6. CONTINUOUS SAFETY GATE  --  the final authority
    //
    //    Re-evaluated every pass, so an obstacle appearing partway through a
    //    manoeuvre stops the rover immediately rather than at the next
    //    command boundary.
    //
    //    The direction-aware decision lives in ONE place --
    //    safetyGateMotion() in safety.cpp -- which both this loop and the
    //    command handler call, so they cannot drift apart.
    //
    //    The gate works PER WHEEL, from the sign of that wheel's commanded
    //    speed: a wheel driving forward is gated on the front sensors, a
    //    wheel driving in reverse on the rear sensors. There is no steering
    //    angle involved anywhere.
    // ------------------------------------------------------------------
    int reqLeft  = commDesiredLeft();
    int reqRight = commDesiredRight();

    int appLeft  = 0;
    int appRight = 0;
    safetyGateMotion(reqLeft, reqRight, &appLeft, &appRight);

    const char *blockReason = "NONE";

    // ------------------------------------------------------------------
    // NO SPEED PATH -> NOTHING IS APPLIED.
    //
    // driveDifferential() already refuses to move without a verified PCA9685,
    // but the APPLIED values reported to the Pi must match reality too.
    // Reporting left_applied:150 while the wheels cannot turn would be the
    // firmware lying about its own output.
    // ------------------------------------------------------------------
    if (!motorDriveAvailable()) {
        appLeft     = 0;
        appRight    = 0;
        blockReason = "MOTOR_PWM_UNAVAILABLE";
    } else if (appLeft != reqLeft || appRight != reqRight) {
        blockReason = safetyBlockReason(reqLeft, reqRight);
        commLatchSafetyStop();

#if SAFETY_CLEAR_COMMAND_ON_BLOCK
        // Strict mode: a blocked command is discarded outright, so motion can
        // never resume without a brand-new command from the Pi.
        commClearDesired();
        appLeft  = 0;
        appRight = 0;
#endif
    }

    driveDifferential(appLeft, appRight);

    // Recompute the reported state from live conditions. Nothing is latched
    // here, which is why the rover cannot sit reporting SAFETY_STOP forever
    // after the obstacle has gone. The sticky record lives in the separate
    // "safety_stop" flag.
    commUpdateRuntime(appLeft, appRight, blockReason);

    // ------------------------------------------------------------------
    // 7. TELEMETRY  (fast TELEMETRY frame + rotating DIAG sections)
    // ------------------------------------------------------------------
    commServiceTelemetry();

    // ------------------------------------------------------------------
    // 8. YIELD
    //    Arduino-ESP32 runs loop() as a FreeRTOS task with no implicit yield.
    //    A completely tight loop starves the IDLE task and trips the task
    //    watchdog. delay(1) is a 1 ms vTaskDelay, not a busy wait: the loop
    //    still runs ~1000x per second, far faster than the 30 ms sensor tick
    //    or the serial byte rate, so command processing is unaffected.
    // ------------------------------------------------------------------
    delay(1);
}

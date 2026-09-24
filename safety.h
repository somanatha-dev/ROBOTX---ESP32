#ifndef SAFETY_H
#define SAFETY_H

#include <Arduino.h>
#include "config.h"
#include "rear_tof.h"

// ---------------------------------------------------------------------------
// FRONT ULTRASONIC sensor identifiers.
//
// EXACTLY TWO HC-SR04, BOTH mounted on the FRONT assembly. There is no rear
// ultrasonic sensor on this rover -- rear sensing is IR, see further down.
//
// These indices are SIDE labels (left/right of the front assembly), resolved
// from the raw A/B hardware channels through SENSOR_MAP_SWAP in config.h.
// That map is UNVERIFIED -- see sensorMapVerified().
// ---------------------------------------------------------------------------
#define US_FRONT_LEFT   0
#define US_FRONT_RIGHT  1

// Legacy aliases. Kept so older call sites keep compiling, but prefer the
// side-named constants above -- "1" and "2" say nothing about placement.
#define US_FRONT_1      US_FRONT_LEFT
#define US_FRONT_2      US_FRONT_RIGHT

// NOTE: SensorHealth now lives in config.h, beside RobotState, because both
// the front ultrasonic path and the rear time-of-flight path use it and
// neither module should have to include the other to name a health level.

// ---------------------------------------------------------------------------
// Configure sensor IO. Call once from setup(), AFTER roverI2cInit() and
// tcaInit() -- the rear sensors are reached over I2C through the multiplexer.
// ---------------------------------------------------------------------------
void safetyInit(void);

// ---------------------------------------------------------------------------
// Non-blocking. Call every pass of loop().
//
// FRONT: issues at most ONE ping per US_PING_INTERVAL_MS, strictly alternating
// between the two HC-SR04 so they are NEVER triggered together. Worst-case
// blocking per call is one pulseIn timeout (~20 ms).
//
// REAR: collects at most ONE already-completed VL53L0X result per
// REAR_TOF_POLL_INTERVAL_MS, round-robin across the three sensors. The sensors
// run in continuous mode, so this never waits for a measurement -- it costs a
// few short I2C transactions.
// ---------------------------------------------------------------------------
void safetyUpdate(void);

// ---------------------------------------------------------------------------
// FRONT per-sensor readings. sensorIndex is US_FRONT_LEFT or US_FRONT_RIGHT.
// ---------------------------------------------------------------------------

// Median-filtered distance in cm. Only meaningful when sensorValid() is true;
// returns a negative sentinel otherwise. Never publish this raw -- telemetry
// emits JSON null for non-measurements.
float   sensorFilteredCm(uint8_t sensorIndex);

// Most recent single raw sample in cm, or a negative value if that sample was
// rejected. Exposed for debugging/calibration only -- never for control.
float   sensorRawCm(uint8_t sensorIndex);

// True when the sensor has at least US_MIN_GOOD_SAMPLES good samples in its
// window AND has produced a good sample within US_STALE_MS.
bool    sensorValid(uint8_t sensorIndex);

// How many of the last US_SAMPLE_WINDOW samples were good. Debug aid.
uint8_t sensorGoodSampleCount(uint8_t sensorIndex);

// Health classification, and its name for telemetry.
SensorHealth sensorHealthOf(uint8_t sensorIndex);
const char  *sensorHealthName(SensorHealth h);

// Worst health across the two front sensors -- the single summary value that
// goes into telemetry's "sensor_health".
SensorHealth sensorHealthWorst(void);

// Count of consecutive bad (timeout / implausible) raw samples. Debug aid.
uint16_t sensorTimeoutStreak(uint8_t sensorIndex);

// False until a human has confirmed which physical side each sensor is on.
bool sensorMapVerified(void);

// ---------------------------------------------------------------------------
// REAR TIME-OF-FLIGHT SENSORS  (three VL53L0X behind a TCA9548A)
//
// MEASUREMENT lives in rear_tof.cpp. What lives HERE is the safety decision:
// the per-sensor obstacle LATCH, its hysteresis band and its confirmation
// counters -- built to exactly the same rules as the front ultrasonic latch,
// so there is one safety policy in this firmware rather than two.
//
// These ARE rangefinders and they DO report millimetres. The previous
// revision described the rear sensors as presence-only IR proximity detectors
// and refused to publish a distance for them, which was right for what was
// known then and is wrong now that the part is identified as a VL53L0X.
//
// While TCA9548A_ADDRESS_CONFIRMED is 0 there is no legitimate path to the
// sensors: every reading is TOF_UNINITIALISED, nothing is fabricated, and
// telemetry says so.
// ---------------------------------------------------------------------------

// True when at least one rear sensor currently holds a trustworthy reading.
bool rearSensingUsable(void);

// Backend name for telemetry: "TCA9548A_VL53L0X" / "UNCONFIGURED" /
// "TCA9548A_NOT_FOUND".
const char *rearBackendName(void);

// ---- Per-sensor accessors. index is REAR_TOF_0 / REAR_TOF_1 / REAR_TOF_2 ---

// Median-filtered rear distance in mm, or REAR_TOF_INVALID_MM when this sensor
// has no measurement. Never publish raw -- telemetry emits JSON null.
int  safetyRearDistanceMm(uint8_t index);

// True when this specific sensor has a trustworthy, fresh reading.
bool safetyRearSensorValid(uint8_t index);

// The LATCHED per-sensor obstacle flag, after hysteresis and confirmation.
// This is the value the reverse gate acts on -- not a bare distance compare.
bool safetyRearSensorObstacle(uint8_t index);

// Advisory per-sensor: inside REAR_WARN_DISTANCE_MM. Not acted upon.
bool safetyRearSensorWarning(uint8_t index);

// ---- Aggregates ------------------------------------------------------------

// True when at least one rear sensor holds a CONFIRMED obstacle latch.
// An invalid sensor never fabricates an obstacle -- and never clears one.
bool safetyRearObstacleDetected(void);

// Advisory only. True when any valid rear sensor is inside the warn distance.
bool safetyRearWarning(void);

// True when there is no trustworthy rear sensing under the configured policy
// (REAR_REQUIRE_ALL_SENSORS in config.h).
bool safetyRearSensorFault(void);

// True only if rear-facing sensing is actually usable by this firmware.
// NOTE: three rear VL53L0X physically EXIST regardless. This reports whether
// the firmware can currently READ them, which is a different question.
bool safetyRearSensingAvailable(void);

// Closest valid REAR reading in mm, or REAR_TOF_INVALID_MM if none is valid.
int safetyClosestRearMm(void);

// ---------------------------------------------------------------------------
// Safety assessment.
// ---------------------------------------------------------------------------

// True when at least one FRONT sensor holds a CONFIRMED obstacle latch.
// An invalid sensor never fabricates an obstacle -- and never clears one.
bool safetyObstacleDetected(void);

// Advisory only. True when a valid front sensor is inside
// SAFETY_WARN_DISTANCE_CM. The firmware does not act on this.
bool safetyWarning(void);

// True when there is no trustworthy FRONT sensing under the configured policy.
bool safetySensorFault(void);

// Final gate for any wheel being driven FORWARD.
bool safetyForwardBlocked(void);

// Final gate for any wheel being driven in REVERSE.
bool safetyReverseBlocked(void);

// ---------------------------------------------------------------------------
// DIRECTION-AWARE GATE  --  the single authority. The main loop and the
// command handler both call this, so they cannot drift apart.
//
// safetyGateMotion() clamps a requested (left,right) pair to what is actually
// permitted, per SAFETY_GATE_MODE in config.h. Outputs may be written even
// when nothing is blocked.
// ---------------------------------------------------------------------------
void safetyGateMotion(int leftCmd, int rightCmd, int *leftOut, int *rightOut);

// True when the requested motion passes the gate completely unchanged.
bool safetyMotionAllowed(int leftCmd, int rightCmd);

// Why safetyGateMotion() clamped something:
//   "NONE" / "FRONT_OBSTACLE" / "FRONT_SENSOR_FAULT" /
//   "REAR_OBSTACLE" / "REAR_SENSOR_FAULT" / "REAR_UNCONFIGURED"
const char *safetyBlockReason(int leftCmd, int rightCmd);

// Closest valid FRONT reading in cm, or a negative value if neither sensor is
// valid. Convenience for logging.
float safetyClosestFrontCm(void);

#endif // SAFETY_H

#ifndef REAR_TOF_H
#define REAR_TOF_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
//  REAR TIME-OF-FLIGHT SENSING  --  THREE VL53L0X BEHIND A TCA9548A
// ============================================================================
//
// LAYERING. This module is the MEASUREMENT layer and nothing else:
//
//     rear_tof.cpp  ->  mux channel, sensor I/O, plausibility, median filter,
//                       validity, health, per-read status
//     safety.cpp    ->  obstacle LATCH, hysteresis, confirmation counters,
//                       and the reverse gate
//
// The split is deliberate and mirrors the front ultrasonic path, where
// safety.cpp owns the latch too. A measurement module that also decided what
// counts as an obstacle would put the safety policy in two files.
//
// THE ADDRESSING RULE. All three sensors answer at 0x29. Every single access
// therefore selects the sensor's TCA channel FIRST, and only one channel is
// ever open. If a channel selection fails, the read is abandoned with
// TOF_BUS_ERROR -- it is NEVER retried against whatever segment happened to be
// connected, because that would silently read the wrong sensor.
//
// WHAT IS NEVER FABRICATED. A failed read of any kind yields no distance at
// all. There is no 0 mm, no last-known value dressed up as fresh, and no
// "assume clear". Every failure mode has its own RearTofStatus and telemetry
// publishes JSON null for the distance.
// ============================================================================

// ---------------------------------------------------------------------------
// Initialise all three sensors, one at a time, each behind its own TCA
// channel. Call once from setup(), after roverI2cInit() and tcaInit().
//
// THE THREE SENSORS ARE NEVER INITIALISED CONCURRENTLY. Each one is brought up
// with its channel selected and every other channel closed, because a VL53L0X
// init is a long sequence of writes to 0x29 and two listening at once would
// both receive them.
//
// Returns true if AT LEAST ONE sensor initialised. Per-sensor results are in
// rearTofStatusOf() -- check those, not just this. A partial success is a real
// outcome and the safety layer treats it according to REAR_REQUIRE_ALL_SENSORS.
// ---------------------------------------------------------------------------
bool rearTofInit(void);

// ---------------------------------------------------------------------------
// Non-blocking. Call every pass of loop().
//
// Services AT MOST ONE sensor per REAR_TOF_POLL_INTERVAL_MS, round-robin, so
// each sensor is revisited every 3 x that interval. The sensors run in
// CONTINUOUS mode and this only collects a result that is already ready, so a
// call costs a few short I2C transactions and never waits for a measurement.
// ---------------------------------------------------------------------------
void rearTofUpdate(void);

// ---------------------------------------------------------------------------
// True when the firmware is able to reach the rear sensors at all -- i.e. the
// TCA9548A address is confirmed, the multiplexer answered, and the rear
// backend CAME UP AT LEAST ONCE DURING THIS BOOT (some sensor initialised).
//
// It is deliberately NOT "at least one sensor is initialised right now". Once
// true it stays true until the next rearTofInit(), so a sensor that is later
// demoted or fails re-initialisation reads as an invalid sensor -- a
// REAR_SENSOR_FAULT that blocks reverse -- and never as "rear unconfigured",
// which would hand reverse to SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED.
//
// This is a question about the FIRMWARE, not the rover. Three VL53L0X are
// physically fitted either way.
// ---------------------------------------------------------------------------
bool rearTofBackendAvailable(void);

// True when at least one sensor currently holds a trustworthy reading.
bool rearTofAnyValid(void);

// Backend description for telemetry: "TCA9548A_VL53L0X" or "UNCONFIGURED".
const char *rearTofBackendName(void);

// ---------------------------------------------------------------------------
// PRIMARY READ API
//
// Returns true ONLY for a real, fresh, in-band measurement, and only then is
// distanceMm written. On false, distanceMm is left untouched -- a caller that
// ignores the return value cannot pick up a fabricated number.
//
// index is REAR_TOF_0 / REAR_TOF_1 / REAR_TOF_2 (TCA channel order).
// ---------------------------------------------------------------------------
bool rearTofRead(uint8_t index, uint16_t *distanceMm);

// ---------------------------------------------------------------------------
// Read all three at once. `distancesMm` and `statuses` must each have room for
// REAR_TOF_SENSOR_COUNT entries; either may be NULL if you do not want it.
//
// A slot whose status is not TOF_VALID has its distance slot left UNTOUCHED.
// Initialise your array before calling if you care what is in those slots.
//
// Returns the number of sensors that produced a valid measurement (0..3).
// ---------------------------------------------------------------------------
uint8_t rearTofReadAll(uint16_t *distancesMm, RearTofStatus *statuses);

// ---------------------------------------------------------------------------
// Per-sensor detail.
// ---------------------------------------------------------------------------

// Median-filtered distance, or REAR_TOF_INVALID_MM when there is no
// measurement. Never publish this raw -- telemetry emits JSON null instead.
int rearTofFilteredMm(uint8_t index);

// The single most recent raw reading, or REAR_TOF_INVALID_MM if that reading
// was rejected. Debug/calibration only, never for control.
int rearTofRawMm(uint8_t index);

// Outcome of the most recent read attempt. This is the field that
// distinguishes a timeout from an init failure from an out-of-range answer.
RearTofStatus rearTofStatusOf(uint8_t index);
const char   *rearTofStatusName(RearTofStatus s);

// True when this sensor has enough good samples in its window AND produced one
// within REAR_TOF_STALE_MS.
bool rearTofValid(uint8_t index);

// Health classification, same vocabulary as the front sensors.
SensorHealth rearTofHealthOf(uint8_t index);

// Worst health across all three -- the single summary for telemetry.
SensorHealth rearTofHealthWorst(void);

// Debug aids.
uint8_t  rearTofGoodSampleCount(uint8_t index);
uint16_t rearTofFailStreak(uint8_t index);

// The TCA channel this sensor index sits on. Confirmed hardware.
uint8_t rearTofChannelOf(uint8_t index);

// True while the sensor is initialised. False before boot init, after a
// failed init, and after a BUS_ERROR demotion until re-init succeeds.
bool rearTofInitialised(uint8_t index);

// ---------------------------------------------------------------------------
// BUS_ERROR DIAGNOSTICS
//
// Where the most recent BUS_ERROR happened, and the raw code the Wire driver
// returned for it (esp32 core 2.0.14 endTransmission(): 2 = NACK, 4 = other,
// 5 = timeout; 0 = no BUS_ERROR recorded yet).
//
// LIMITATION: for the VL53L0X sites the code is the Pololu library's
// last_status, which is ONLY the endTransmission() result of a register-pointer
// write. The data phase (requestFrom) is never reported. For RANGE_READ it is
// the LAST such write inside readRangeContinuousMillimeters(), normally the
// interrupt clear. For TCA_SELECT it is roverI2cLastError() after the mask
// write.
// ---------------------------------------------------------------------------
typedef enum {
    TOF_BUSERR_NONE = 0,
    TOF_BUSERR_TCA_SELECT,        // channel-select write to the TCA9548A
    TOF_BUSERR_READY_REGISTER,    // VL53L0X RESULT_INTERRUPT_STATUS read
    TOF_BUSERR_RANGE_STATUS,      // VL53L0X RESULT_RANGE_STATUS read
    TOF_BUSERR_RANGE_READ         // readRangeContinuousMillimeters()
} RearTofBusErrSite;

typedef enum {
    TOF_REINIT_NONE = 0,          // no runtime re-init attempted yet
    TOF_REINIT_OK,
    TOF_REINIT_FAILED
} RearTofReinitResult;

// Consecutive VL53L0X-transaction BUS_ERRORs (the recovery trigger). Does NOT
// count TCA_SELECT, TIMEOUT or OUT_OF_RANGE.
uint16_t            rearTofBusErrStreak(uint8_t index);
// Every BUS_ERROR since boot, TCA_SELECT included.
uint32_t            rearTofBusErrTotal(uint8_t index);
uint8_t             rearTofLastBusErrCode(uint8_t index);
RearTofBusErrSite   rearTofLastBusErrSite(uint8_t index);
const char         *rearTofBusErrSiteName(RearTofBusErrSite site);
// Runtime re-init attempts made by rearTofUpdate() (boot init not counted).
uint16_t            rearTofReinitCount(uint8_t index);
RearTofReinitResult rearTofLastReinitResult(uint8_t index);
const char         *rearTofReinitResultName(RearTofReinitResult r);

// False until a human confirms which sensor is physically left/centre/right.
bool rearTofOrientationVerified(void);

// ---------------------------------------------------------------------------
// COMMISSIONING: test ONE sensor, right now, synchronously.
//
// Selects its channel, takes a single fresh measurement and reports what came
// back. Blocks for up to REAR_TOF_IO_TIMEOUT_MS, so it is for the bench only
// -- rearTofUpdate() is what the control loop uses.
//
// distanceMm is written only when the return value is TOF_VALID.
// ---------------------------------------------------------------------------
RearTofStatus rearTofTestOne(uint8_t index, uint16_t *distanceMm);

#endif // REAR_TOF_H

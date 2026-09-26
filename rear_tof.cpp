#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "rear_tof.h"
#include "tca9548a.h"
#include "rover_i2c.h"
#include "config.h"

// ============================================================================
//  REAR VL53L0X SUBSYSTEM
// ============================================================================
//
// EXTERNAL DEPENDENCY: the Pololu VL53L0X Arduino library (MIT-licensed),
// which carries ST's device initialisation and tuning sequence. That sequence
// is several hundred register writes long and is NOT something to reproduce
// from memory -- getting one value wrong produces a sensor that initialises
// cleanly and ranges wrongly, which is the worst possible failure for a part
// whose job is stopping the rover.
//
//     Install: Arduino IDE -> Library Manager -> "VL53L0X" by Pololu
//     Or:      https://github.com/pololu/vl53l0x-arduino
//
// -------------------------------------------------------------------------
// THE ONE RULE
// -------------------------------------------------------------------------
// All three sensors are at 0x29. EVERY access selects the sensor's own TCA
// channel first, and the multiplexer API only ever opens one channel. If the
// channel selection fails we do NOT fall through to the transaction -- we do
// not know which segment is connected, so any answer would be from an unknown
// sensor. That case returns TOF_BUS_ERROR and no distance.
// -------------------------------------------------------------------------
// NOTHING IS FABRICATED
// -------------------------------------------------------------------------
// There are six distinct ways to not get a measurement and each keeps its own
// identity all the way out to telemetry (see RearTofStatus in config.h). None
// of them becomes 0 mm, none becomes "clear", and none silently reuses the
// last good value. REAR_TOF_INVALID_MM is negative precisely so that a bug
// that leaked it into a distance comparison would be obvious rather than
// looking like an obstacle 0 mm away.
// ============================================================================


// ---------------------------------------------------------------------------
// Per-sensor record. The filtering mirrors the front ultrasonic path: rolling
// window, median, validity from good-sample count plus freshness.
// ---------------------------------------------------------------------------
typedef struct {
    VL53L0X       dev;
    uint8_t       channel;          // TCA channel. Confirmed hardware.

    bool          initialised;
    RearTofStatus status;

    int           samples[REAR_TOF_SAMPLE_WINDOW];   // REAR_TOF_INVALID_MM = bad
    uint8_t       writeIndex;

    int           filteredMm;
    int           lastRawMm;
    bool          valid;
    uint32_t      lastGoodMs;
    uint8_t       goodCount;

    uint16_t      failStreak;       // consecutive non-measurements
    uint32_t      lastInitAttemptMs;
    uint8_t       lastRangeStatus;  // raw 4-bit code from register 0x14

    // BUS_ERROR recovery and diagnostics. See rearTofUpdate().
    uint16_t      busErrStreak;     // consecutive VL53L0X-transaction BUS_ERRORs
    uint32_t      busErrTotal;      // every BUS_ERROR, TCA_SELECT included
    uint8_t       lastBusErrCode;   // raw Wire code, see rear_tof.h
    uint8_t       lastBusErrSite;   // RearTofBusErrSite
    uint16_t      reinitCount;      // runtime re-init attempts
    uint8_t       lastReinitResult; // RearTofReinitResult
} RearTofSensor;

static RearTofSensor gTof[REAR_TOF_SENSOR_COUNT];

static uint8_t  gActive      = 0;
static uint32_t gNextPollMs  = 0;
static bool     gBackendUp   = false;


// ============================================================================
// SMALL HELPERS
// ============================================================================

static int windowMedian(const RearTofSensor *s, uint8_t *goodOut)
{
    int sorted[REAR_TOF_SAMPLE_WINDOW];
    uint8_t n = 0;

    for (uint8_t i = 0; i < REAR_TOF_SAMPLE_WINDOW; i++) {
        if (s->samples[i] > 0) {
            sorted[n++] = s->samples[i];
        }
    }

    *goodOut = n;

    if (n == 0) {
        return REAR_TOF_INVALID_MM;
    }

    for (uint8_t i = 1; i < n; i++) {
        int key = sorted[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    if (n & 1) {
        return sorted[n / 2];
    }
    return (sorted[n / 2 - 1] + sorted[n / 2]) / 2;
}


// Record a NON-measurement. Note what this does NOT do: it does not write a
// sample, it does not touch filteredMm, and it does not clear validity by
// itself. Validity is re-derived below from the window and freshness, so a
// sensor that starts failing decays out of validity on its own schedule
// instead of flipping the instant one read fails.
static void recordFailure(RearTofSensor *s, RearTofStatus why)
{
    s->status    = why;
    s->lastRawMm = REAR_TOF_INVALID_MM;

    if (s->failStreak < 0xFFFF) {
        s->failStreak++;
    }
}


// A BUS_ERROR, with the evidence kept: which transaction failed and the raw
// code the Wire driver returned for it. The status and fail streak are set by
// recordFailure() exactly as for any other BUS_ERROR. The recovery streak is
// NOT touched here -- rearTofUpdate() owns it, because a TCA select failure is
// recorded here too and must never count toward re-initialising a sensor.
static void recordBusError(RearTofSensor *s, RearTofBusErrSite site, uint8_t code)
{
    recordFailure(s, TOF_BUS_ERROR);

    s->lastBusErrSite = (uint8_t)site;
    s->lastBusErrCode = code;
    if (s->busErrTotal < 0xFFFFFFFFUL) {
        s->busErrTotal++;
    }
}


// Re-derive validity and the filtered value. Called after EVERY serviced tick,
// success or failure, so a sensor cannot keep a stale "valid" flag alive.
static void refreshValidity(RearTofSensor *s)
{
    uint8_t good = 0;
    int median = windowMedian(s, &good);
    s->goodCount = good;

    bool fresh = (millis() - s->lastGoodMs) <= REAR_TOF_STALE_MS;

    if (good >= REAR_TOF_MIN_GOOD_SAMPLES && median > 0 && fresh) {
        s->filteredMm = median;
        s->valid      = true;
    } else {
        s->valid = false;
        if (!fresh) {
            // Gone quiet. Stop publishing a number that is no longer being
            // re-measured -- a stale distance is worse than no distance,
            // because it reads as a live one.
            s->filteredMm = REAR_TOF_INVALID_MM;
            if (s->initialised && s->status == TOF_VALID) {
                s->status = TOF_STALE;
            }
        }
    }
}


// ============================================================================
// INITIALISATION OF ONE SENSOR
// ============================================================================
//
// The caller must already have selected this sensor's channel. That is not an
// optimisation -- it is how the "one channel at a time" invariant is kept
// visible at the call site instead of buried in here.
// ============================================================================

static bool initOneSelected(RearTofSensor *s)
{
    s->lastInitAttemptMs = millis();

    s->dev.setBus(&Wire);
    s->dev.setTimeout(REAR_TOF_IO_TIMEOUT_MS);

    for (uint8_t attempt = 0; attempt < REAR_TOF_INIT_ATTEMPTS; attempt++) {
        // A VL53L0X coming off a cold power rail sometimes needs a second
        // attempt. Retrying is honest -- it is the same question asked again,
        // not a different answer substituted.
        if (s->dev.init(REAR_TOF_IO_2V8 != 0)) {

            if (!s->dev.setMeasurementTimingBudget(REAR_TOF_TIMING_BUDGET_US)) {
                // The part came up but would not accept its timing budget.
                // Treat that as an init failure rather than running with a
                // budget we did not choose.
                continue;
            }

            // Continuous mode. This is what makes rearTofUpdate() cheap: the
            // sensor ranges on its own schedule and we collect results that
            // are already waiting, instead of starting a measurement and
            // blocking ~30 ms for it inside the safety loop.
            s->dev.startContinuous(REAR_TOF_CONTINUOUS_PERIOD_MS);

            s->initialised = true;
            s->status      = TOF_UNINITIALISED;   // no reading taken YET
            s->failStreak  = 0;
            s->lastGoodMs  = millis();            // grace period before stale
            return true;
        }

        delay(5);
    }

    s->initialised = false;
    s->status      = TOF_INIT_FAILED;
    return false;
}


// ============================================================================
// PUBLIC: INITIALISATION
// ============================================================================

bool rearTofInit(void)
{
    gActive     = 0;
    gNextPollMs = millis();
    gBackendUp  = false;

    static const uint8_t channelOf[REAR_TOF_SENSOR_COUNT] = {
        TCA_CH_REAR_TOF_0,
        TCA_CH_REAR_TOF_1,
        TCA_CH_REAR_TOF_2
    };

    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        RearTofSensor *s = &gTof[i];

        s->channel           = channelOf[i];
        s->initialised       = false;
        s->status            = TOF_UNINITIALISED;
        s->writeIndex        = 0;
        s->filteredMm        = REAR_TOF_INVALID_MM;
        s->lastRawMm         = REAR_TOF_INVALID_MM;
        s->valid             = false;
        s->lastGoodMs        = millis();
        s->goodCount         = 0;
        s->failStreak        = 0;
        s->lastInitAttemptMs = 0;
        s->lastRangeStatus   = 0;
        s->busErrStreak      = 0;
        s->busErrTotal       = 0;
        s->lastBusErrCode    = 0;
        s->lastBusErrSite    = TOF_BUSERR_NONE;
        s->reinitCount       = 0;
        s->lastReinitResult  = TOF_REINIT_NONE;

        for (uint8_t k = 0; k < REAR_TOF_SAMPLE_WINDOW; k++) {
            s->samples[k] = REAR_TOF_INVALID_MM;
        }
    }

    // No multiplexer means no legitimate path to a sensor. Every sensor stays
    // TOF_UNINITIALISED, which telemetry reports plainly. We do not probe 0x29
    // on the bare bus "just in case" -- the confirmed topology says the
    // sensors are behind the mux, and finding something at 0x29 without one
    // would mean the wiring is not what was described.
    if (!tcaAddressConfirmed() || !tcaPresent()) {
        return false;
    }

    uint8_t up = 0;

    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        RearTofSensor *s = &gTof[i];

        // ONE CHANNEL AT A TIME. This is the line that makes three sensors at
        // the same address workable, and it is why initialisation is a loop of
        // select-then-init rather than a batch.
        if (!tcaSelectChannel(s->channel)) {
            s->status         = TOF_BUS_ERROR;
            s->lastBusErrSite = TOF_BUSERR_TCA_SELECT;    // diagnostics only
            s->lastBusErrCode = roverI2cLastError();
            s->busErrTotal++;
            continue;
        }

        if (initOneSelected(s)) {
            up++;
        }
    }

    // Leave the bus on the main segment so the PCA9685 and the GPS are
    // reachable and no 0x29 device is hanging off it.
    (void)tcaDeselectAll();

    gBackendUp = (up > 0);
    return gBackendUp;
}


// ============================================================================
// PUBLIC: SCHEDULED UPDATE
// ============================================================================

// Collect one result from an already-selected, already-initialised sensor.
// Returns true when this visit ended in a VL53L0X-transaction BUS_ERROR, and
// false for every outcome where the sensor answered (not ready, TIMEOUT,
// OUT_OF_RANGE, VALID).
static bool serviceSelected(RearTofSensor *s)
{
    // Is a measurement ready? In continuous mode the part raises an interrupt
    // status bit when it has one. Polling that bit is a 1-byte read; calling
    // the library's read function without checking would spin until the next
    // measurement completed, inside the safety loop.
    uint8_t ready = s->dev.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);

    if (s->dev.last_status != 0) {
        recordBusError(s, TOF_BUSERR_READY_REGISTER, s->dev.last_status);
        return true;
    }

    if ((ready & 0x07) == 0) {
        // Simply not ready yet. This is NOT a failure and must not count
        // toward the fail streak: the sensor is working exactly as configured,
        // we just arrived early. Staleness is what catches a sensor that
        // stops producing results altogether.
        return false;
    }

    // The 4-bit range status lives in bits 3..6 of RESULT_RANGE_STATUS and is
    // latched until the next measurement, so it is read BEFORE the range read
    // clears the interrupt.
    uint8_t rawStatus = s->dev.readReg(VL53L0X::RESULT_RANGE_STATUS);
    if (s->dev.last_status != 0) {
        recordBusError(s, TOF_BUSERR_RANGE_STATUS, s->dev.last_status);
        return true;
    }
    s->lastRangeStatus = (uint8_t)((rawStatus >> 3) & 0x0F);

    // Discard a timeout flag left over from an EARLIER call. The library
    // clears it only in timeoutOccurred(), which the BUS_ERROR return below
    // skips, and init() never clears it -- so a stale flag would turn this
    // call's good reading into a false TIMEOUT. The flag checked after the
    // read then describes this call only.
    (void)s->dev.timeoutOccurred();
    uint16_t mm = s->dev.readRangeContinuousMillimeters();

    if (s->dev.last_status != 0) {
        recordBusError(s, TOF_BUSERR_RANGE_READ, s->dev.last_status);
        return true;
    }

    if (s->dev.timeoutOccurred() || mm == REAR_TOF_TIMEOUT_SENTINEL) {
        // The sensor was addressed but produced nothing in time. Distinct from
        // a bus error (we reached it) and from out-of-range (it answered).
        recordFailure(s, TOF_TIMEOUT);
        return false;
    }

#if REAR_TOF_REQUIRE_RANGE_STATUS_VALID
    if (s->lastRangeStatus != REAR_TOF_RANGE_STATUS_VALID_CODE) {
        // The sensor itself says this measurement did not meet its own
        // quality criteria. Reported as out-of-range rather than as an error:
        // the hardware is fine, the measurement is not usable.
        recordFailure(s, TOF_OUT_OF_RANGE);
        return false;
    }
#endif

    if (mm < REAR_TOF_MIN_VALID_MM || mm > REAR_TOF_MAX_VALID_MM) {
        // The normal "nothing within range" answer -- a VL53L0X reports around
        // 8190 mm when it sees nothing. That is NOT an 8.19 m measurement, and
        // it is NOT an obstacle. It is also NOT an error, which is why it does
        // not get turned into one.
        recordFailure(s, TOF_OUT_OF_RANGE);
        s->lastRawMm = (int)mm;    // keep it visible for debugging
        return false;
    }

    // ---- A real measurement. ----
    s->lastRawMm = (int)mm;
    s->failStreak = 0;
    s->status     = TOF_VALID;

    s->samples[s->writeIndex] = (int)mm;
    s->writeIndex++;
    if (s->writeIndex >= REAR_TOF_SAMPLE_WINDOW) {
        s->writeIndex = 0;
    }

    s->lastGoodMs = millis();
    return false;
}


#if REAR_TOF_REINIT_AFTER_MS > 0
// Hand a sensor that initialised, and has since failed on the bus
// REAR_TOF_BUS_ERROR_REINIT_STREAK visits in a row, back to the throttled
// re-init branch in rearTofUpdate(). Without this, `initialised` stays true
// forever and that branch is unreachable for it.
//
// The sample window is emptied so a later recovery cannot be filtered together
// with readings from before the failure: after a successful re-init the sensor
// needs REAR_TOF_MIN_GOOD_SAMPLES NEW readings before it is valid again.
//
// What this deliberately leaves alone: the obstacle latch (safety.cpp owns it
// and an invalid sensor never changes it), status (still the BUS_ERROR that
// caused this, until the re-init attempt reports), failStreak and health.
static void demoteForReinit(RearTofSensor *s)
{
    s->initialised  = false;
    s->busErrStreak = 0;

    for (uint8_t k = 0; k < REAR_TOF_SAMPLE_WINDOW; k++) {
        s->samples[k] = REAR_TOF_INVALID_MM;
    }
    s->writeIndex = 0;
    s->goodCount  = 0;
    s->valid      = false;
    s->filteredMm = REAR_TOF_INVALID_MM;
}
#endif


void rearTofUpdate(void)
{
    if (!tcaAddressConfirmed() || !tcaPresent()) {
        return;                     // nothing legitimate to talk to
    }

    // Signed comparison handles millis() rollover correctly.
    if ((int32_t)(millis() - gNextPollMs) < 0) {
        return;
    }

    RearTofSensor *s = &gTof[gActive];

    gActive     = (uint8_t)((gActive + 1) % REAR_TOF_SENSOR_COUNT);
    gNextPollMs = millis() + REAR_TOF_POLL_INTERVAL_MS;

    if (!tcaSelectChannel(s->channel)) {
        // We do not know which segment is connected. Abandoning is the only
        // safe option -- reading 0x29 now could be any of the three sensors.
        //
        // Recorded for diagnosis, but it neither advances nor resets the
        // sensor's BUS_ERROR recovery streak: this failure is upstream of the
        // sensor, and re-initialising the VL53L0X cannot repair it.
        recordBusError(s, TOF_BUSERR_TCA_SELECT, roverI2cLastError());
        refreshValidity(s);
        return;
    }

    if (s->initialised) {
        // Consecutive VL53L0X-transaction BUS_ERRORs only. Any visit where
        // the sensor answered -- not ready, TIMEOUT, OUT_OF_RANGE, VALID --
        // breaks the run.
        if (serviceSelected(s)) {
            if (s->busErrStreak < 0xFFFF) {
                s->busErrStreak++;
            }
        } else {
            s->busErrStreak = 0;
        }

#if REAR_TOF_REINIT_AFTER_MS > 0
        if (s->busErrStreak >= REAR_TOF_BUS_ERROR_REINIT_STREAK) {
            // Nothing is re-initialised HERE. The next visits take the
            // throttled branch below, at most once per
            // REAR_TOF_REINIT_AFTER_MS, with this channel selected first.
            demoteForReinit(s);
        }
#endif
    } else {
#if REAR_TOF_REINIT_AFTER_MS > 0
        // A dead or demoted sensor is retried periodically in case its failure
        // was a transient power or bus event. This retries the HARDWARE; it
        // never invents a reading. A successful retry does NOT make the sensor
        // valid -- it still needs fresh readings to fill its empty window.
        if ((millis() - s->lastInitAttemptMs) >= REAR_TOF_REINIT_AFTER_MS) {
            bool ok = initOneSelected(s);

            if (s->reinitCount < 0xFFFF) {
                s->reinitCount++;
            }
            s->lastReinitResult = ok ? TOF_REINIT_OK : TOF_REINIT_FAILED;
        }
#endif
    }

    // Back to the main segment. Keeping the default "no channel open" means
    // any code path that forgets to select cannot accidentally succeed against
    // whichever sensor happened to still be connected.
    (void)tcaDeselectAll();

    refreshValidity(s);

    // STICKY FOR THE REST OF THIS BOOT. "Backend up" means the rear backend
    // came up at least once since rearTofInit() -- NOT "some sensor is
    // initialised right now". It can become true here (a sensor that failed
    // at boot initialises later), but runtime demotion or a failed re-init
    // never makes it false again.
    //
    // This is a SAFETY rule. rearTofBackendAvailable() going false makes
    // safetyReverseBlocked() skip the rear sensors and latches entirely and
    // fall back to SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED. Losing sensors
    // at runtime must read as REAR_SENSOR_FAULT (reverse blocked), never as an
    // unconfigured rear. Only rearTofInit() clears this flag.
    bool anyUp = false;
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (gTof[i].initialised) anyUp = true;
    }
    gBackendUp = gBackendUp || anyUp;
}


// ============================================================================
// PUBLIC: READ API
// ============================================================================

bool rearTofRead(uint8_t index, uint16_t *distanceMm)
{
    if (index >= REAR_TOF_SENSOR_COUNT || distanceMm == NULL) {
        return false;
    }

    const RearTofSensor *s = &gTof[index];

    if (!s->valid || s->filteredMm <= 0) {
        // Deliberately leaves *distanceMm untouched. A caller that ignores the
        // return value keeps whatever it had rather than picking up a zero
        // that would read as an obstacle pressed against the sensor.
        return false;
    }

    *distanceMm = (uint16_t)s->filteredMm;
    return true;
}


uint8_t rearTofReadAll(uint16_t *distancesMm, RearTofStatus *statuses)
{
    uint8_t validCount = 0;

    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (statuses != NULL) {
            statuses[i] = gTof[i].status;
        }

        uint16_t mm = 0;
        if (rearTofRead(i, &mm)) {
            if (distancesMm != NULL) {
                distancesMm[i] = mm;
            }
            validCount++;
        }
        // No else. An invalid slot is left exactly as the caller supplied it.
    }

    return validCount;
}


// ============================================================================
// PUBLIC: ACCESSORS
// ============================================================================

bool rearTofBackendAvailable(void)
{
    return tcaAddressConfirmed() && tcaPresent() && gBackendUp;
}

bool rearTofAnyValid(void)
{
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (gTof[i].valid) return true;
    }
    return false;
}

const char *rearTofBackendName(void)
{
    if (!tcaAddressConfirmed()) {
        return "UNCONFIGURED";
    }
    if (!tcaPresent()) {
        return "TCA9548A_NOT_FOUND";
    }
    return "TCA9548A_VL53L0X";
}

int rearTofFilteredMm(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return REAR_TOF_INVALID_MM;
    return gTof[index].filteredMm;
}

int rearTofRawMm(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return REAR_TOF_INVALID_MM;
    return gTof[index].lastRawMm;
}

RearTofStatus rearTofStatusOf(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return TOF_UNINITIALISED;
    return gTof[index].status;
}

const char *rearTofStatusName(RearTofStatus s)
{
    switch (s) {
        case TOF_UNINITIALISED: return "UNINITIALISED";
        case TOF_INIT_FAILED:   return "INIT_FAILED";
        case TOF_BUS_ERROR:     return "BUS_ERROR";
        case TOF_TIMEOUT:       return "TIMEOUT";
        case TOF_OUT_OF_RANGE:  return "OUT_OF_RANGE";
        case TOF_STALE:         return "STALE";
        case TOF_VALID:         return "VALID";
        default:                return "UNKNOWN";
    }
}

bool rearTofValid(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return false;
    return gTof[index].valid;
}

SensorHealth rearTofHealthOf(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) {
        return HEALTH_FAULT;
    }

    const RearTofSensor *s = &gTof[index];

    // Never initialised, or initialisation failed: not a degraded sensor, an
    // absent one.
    if (!s->initialised) {
        return HEALTH_FAULT;
    }

    bool fresh = (millis() - s->lastGoodMs) <= REAR_TOF_STALE_MS;

    if (!fresh ||
        s->goodCount == 0 ||
        s->failStreak >= REAR_TOF_HEALTH_FAULT_STREAK) {
        return HEALTH_FAULT;
    }

    if (s->goodCount < REAR_TOF_SAMPLE_WINDOW ||
        s->failStreak >= REAR_TOF_HEALTH_DEGRADED_STREAK ||
        !s->valid) {
        return HEALTH_DEGRADED;
    }

    return HEALTH_OK;
}

SensorHealth rearTofHealthWorst(void)
{
    SensorHealth worst = HEALTH_OK;

    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        SensorHealth h = rearTofHealthOf(i);
        if (h > worst) {
            worst = h;              // enum ordered OK < DEGRADED < FAULT
        }
    }

    return worst;
}

uint8_t rearTofGoodSampleCount(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return 0;
    return gTof[index].goodCount;
}

uint16_t rearTofFailStreak(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return 0;
    return gTof[index].failStreak;
}

uint8_t rearTofChannelOf(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return 0xFF;
    return gTof[index].channel;
}

bool rearTofOrientationVerified(void)
{
    return (REAR_TOF_ORIENTATION_VERIFIED != 0);
}

bool rearTofInitialised(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return false;
    return gTof[index].initialised;
}

uint16_t rearTofBusErrStreak(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return 0;
    return gTof[index].busErrStreak;
}

uint32_t rearTofBusErrTotal(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return 0;
    return gTof[index].busErrTotal;
}

uint8_t rearTofLastBusErrCode(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return 0;
    return gTof[index].lastBusErrCode;
}

RearTofBusErrSite rearTofLastBusErrSite(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return TOF_BUSERR_NONE;
    return (RearTofBusErrSite)gTof[index].lastBusErrSite;
}

const char *rearTofBusErrSiteName(RearTofBusErrSite site)
{
    switch (site) {
        case TOF_BUSERR_NONE:           return "NONE";
        case TOF_BUSERR_TCA_SELECT:     return "TCA_SELECT";
        case TOF_BUSERR_READY_REGISTER: return "READY_REGISTER";
        case TOF_BUSERR_RANGE_STATUS:   return "RANGE_STATUS";
        case TOF_BUSERR_RANGE_READ:     return "RANGE_READ";
        default:                        return "UNKNOWN";
    }
}

uint16_t rearTofReinitCount(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return 0;
    return gTof[index].reinitCount;
}

RearTofReinitResult rearTofLastReinitResult(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return TOF_REINIT_NONE;
    return (RearTofReinitResult)gTof[index].lastReinitResult;
}

const char *rearTofReinitResultName(RearTofReinitResult r)
{
    switch (r) {
        case TOF_REINIT_NONE:   return "NONE";
        case TOF_REINIT_OK:     return "OK";
        case TOF_REINIT_FAILED: return "FAILED";
        default:                return "UNKNOWN";
    }
}


// ============================================================================
// COMMISSIONING: SINGLE-SENSOR TEST
// ============================================================================
//
// Synchronous and blocking, for bench use only. It exists so you can answer
// "is sensor 1 alive, and which physical position is it?" by waving a hand in
// front of one sensor and reading one number back -- which is also how you
// resolve REAR_TOF_ORIENTATION_VERIFIED.
// ============================================================================

RearTofStatus rearTofTestOne(uint8_t index, uint16_t *distanceMm)
{
    if (index >= REAR_TOF_SENSOR_COUNT) {
        return TOF_UNINITIALISED;
    }

    if (!tcaAddressConfirmed() || !tcaPresent()) {
        return TOF_UNINITIALISED;
    }

    RearTofSensor *s = &gTof[index];

    if (!tcaSelectChannel(s->channel)) {
        return TOF_BUS_ERROR;
    }

    RearTofStatus result;

    if (!s->initialised && !initOneSelected(s)) {
        result = TOF_INIT_FAILED;
    } else {
        // A single fresh measurement. readRangeSingleMillimeters() stops
        // continuous mode implicitly for this one shot; continuous is
        // restarted below so the control loop's round-robin keeps working.
        uint16_t mm = s->dev.readRangeSingleMillimeters();

        if (s->dev.last_status != 0) {
            result = TOF_BUS_ERROR;
        } else if (s->dev.timeoutOccurred() || mm == REAR_TOF_TIMEOUT_SENTINEL) {
            result = TOF_TIMEOUT;
        } else if (mm < REAR_TOF_MIN_VALID_MM || mm > REAR_TOF_MAX_VALID_MM) {
            result = TOF_OUT_OF_RANGE;
            if (distanceMm != NULL) {
                // Reported so you can SEE the ~8190 that means "nothing
                // there". The status is what says not to trust it as a range.
                *distanceMm = mm;
            }
        } else {
            result = TOF_VALID;
            if (distanceMm != NULL) {
                *distanceMm = mm;
            }
        }

        s->dev.startContinuous(REAR_TOF_CONTINUOUS_PERIOD_MS);
    }

    (void)tcaDeselectAll();

    return result;
}

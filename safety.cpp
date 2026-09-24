#include <Arduino.h>
#include "safety.h"
#include "rear_tof.h"
#include "config.h"

// ============================================================================
//  SENSING OVERVIEW -- READ THIS BEFORE TRUSTING ANY AUTONOMY
// ============================================================================
//
// FRONT  : TWO HC-SR04 ultrasonic sensors.
//            A : TRIG GPIO13 / ECHO GPIO14 (through a resistive divider)
//            B : TRIG GPIO18 / ECHO GPIO19 (through a resistive divider)
//          Fully implemented below. These are true ranging sensors and do
//          report centimetres -- UNCALIBRATED centimetres, using a fixed
//          343 m/s speed of sound. Accuracy must be validated with a tape
//          measure; the firmware only guarantees that the numbers are
//          physically derived, not that they are correct.
//
// REAR   : THREE VL53L0X time-of-flight rangefinders, all at I2C address
//          0x29, reached through a TCA9548A multiplexer on CH0/CH1/CH2.
//
//          Measurement lives in rear_tof.cpp. What lives HERE is the safety
//          decision -- the per-sensor obstacle latch, its hysteresis band and
//          its confirmation counters, built to the SAME rules as the front
//          ultrasonic latch below. One safety policy, applied twice, rather
//          than two policies that will eventually disagree.
//
//          TERMINOLOGY CORRECTION: an earlier revision of this file called the
//          rear sensors "IR proximity" detectors and refused to publish any
//          distance for them. That was correct for what was known then. The
//          part is now identified as a VL53L0X, which is an infrared device
//          but a TRUE RANGEFINDER -- it measures time of flight and reports
//          millimetres. Rear distances are published because they are
//          genuinely measured.
//
//          STILL UNCONFIRMED: the TCA9548A's I2C address. While
//          TCA9548A_ADDRESS_CONFIRMED is 0 there is no legitimate path to the
//          sensors, every reading stays TOF_UNINITIALISED, and reverse is
//          governed by SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED in
//          config.h. Nothing is fabricated in the meantime.
// ============================================================================


// ============================================================================
//  SENSOR-INVALID SAFETY POLICY
// ============================================================================
//
// A timeout from an HC-SR04 means "I heard nothing back". That is NOT the
// same as "the path is clear" and it is NOT the same as "there is an
// obstacle". Common causes of a genuine timeout are a soft/angled surface
// that scatters the burst, or a target beyond range -- but a dead sensor and
// a cut wire look identical.
//
// So the policy is:
//
//   INVALID SAMPLE   -> discarded. Never counted as an obstacle, never
//                       counted as clear. It simply does not vote.
//
//   SENSOR VALID     -> >= US_MIN_GOOD_SAMPLES good samples in the rolling
//                       window AND a good sample within the last US_STALE_MS.
//
//   OBSTACLE         -> at least one VALID sensor holds a confirmed latch.
//
//   BOTH INVALID     -> SENSOR FAULT. Forward motion is REFUSED. This is the
//                       failsafe that stops a blind rover driving on forever
//                       when its sensors die.
//
//   ONE VALID        -> depends on SAFETY_REQUIRE_BOTH_SENSORS in config.h.
//                       Default 1: still a fault, because the dead sensor's
//                       cone is unknown space.
//
// Note the deliberate asymmetry: we fail toward "stop". A sensor that cannot
// be trusted blocks motion; it never authorises it.
//
// THE SAME RULE APPLIES TO THE REAR VL53L0X SENSORS, in the same words: a
// timeout, a bus error or an out-of-range answer is NOT "the path behind me is
// clear". None of them can clear a confirmed rear obstacle latch, and the
// rear equivalent of "both invalid" -- governed by REAR_REQUIRE_ALL_SENSORS --
// refuses REVERSE exactly as the front equivalent refuses FORWARD.
// ============================================================================


// ============================================================================
//  HYSTERESIS  --  WHY THERE IS NO PLAIN "distance < threshold" ANYWHERE
// ============================================================================
//
// A single threshold turns any reading that dithers around it into a stream
// of STOP/GO/STOP/GO transitions. With an ultrasonic sensor, whose readings
// legitimately jitter by several centimetres, that is guaranteed to happen.
//
// So each sensor carries a LATCHED obstacle flag driven by two DIFFERENT
// distances and two consecutive-sample counters:
//
//        0            STOP(40)        CLEAR(65)      WARN(90)
//        |---------------|---------------|--------------|----------->
//        |   latch SET   |   dead band   |  latch CLEAR |  advisory
//                        |<- hysteresis->|
//
//   * filtered <= STOP  for SAFETY_OBSTACLE_CONFIRM ticks  -> latch SET
//   * filtered >= CLEAR for SAFETY_CLEAR_CONFIRM    ticks  -> latch CLEARED
//   * anywhere BETWEEN them                                -> latch UNCHANGED
//
// The dead band is the whole point: inside it nothing changes state, so noise
// in that region cannot produce a transition at all.
//
// Both counters require CONSECUTIVE agreement and are reset by any sample
// that disagrees, so a single outlier can neither trigger nor release a stop.
// ============================================================================

typedef struct {
    uint8_t  trigPin;
    uint8_t  echoPin;
    const char *name;

    float    samples[US_SAMPLE_WINDOW];   // rolling window, US_INVALID_CM = bad
    uint8_t  writeIndex;

    float    filteredCm;
    float    lastRawCm;
    bool     valid;
    uint32_t lastGoodMs;
    uint8_t  goodCount;
    uint8_t  spikeCount;

    // ---- hysteresis state ----
    bool     obstacle;        // LATCHED. Only the counters below change it.
    uint8_t  nearCount;       // consecutive ticks with filtered <= STOP
    uint8_t  farCount;        // consecutive ticks with filtered >= CLEAR
    bool     warning;         // advisory: filtered <= WARN

    // ---- health ----
    uint16_t timeoutStreak;   // consecutive bad raw samples
} UltrasonicSensor;

static UltrasonicSensor gSensors[US_SENSOR_COUNT];

static uint8_t  gActiveSensor = 0;
static uint32_t gNextPingMs   = 0;
static bool     gObstacle     = false;
static bool     gWarning      = false;


// ============================================================================
// ONE RAW FRONT MEASUREMENT
// ============================================================================
//
// Returns distance in cm, or US_INVALID_CM if the echo timed out or the
// result is outside the physically plausible band.
//
// Blocking time is bounded by US_ECHO_TIMEOUT_US (~20 ms worst case, far less
// in normal operation since pulseIn returns as soon as the echo arrives).
// The main loop polls the serial port on both sides of this call.
// ============================================================================

static float pingRaw(UltrasonicSensor *s)
{
    // Clean low period before the trigger burst.
    digitalWrite(s->trigPin, LOW);
    delayMicroseconds(4);

    // HC-SR04 requires a 10 us high trigger.
    digitalWrite(s->trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(s->trigPin, LOW);

    unsigned long duration = pulseIn(s->echoPin, HIGH, US_ECHO_TIMEOUT_US);

    // pulseIn returns 0 on timeout. TIMEOUT IS *INVALID*, NOT "OBSTACLE" and
    // NOT "CLEAR". It is never turned into a fake distance.
    if (duration == 0) {
        return US_INVALID_CM;
    }

    // speed of sound 343 m/s = 0.0343 cm/us; halve it for the round trip.
    float cm = (float)duration * 0.01715f;

    if (cm < US_MIN_VALID_CM || cm > US_MAX_VALID_CM) {
        return US_INVALID_CM;
    }

    return cm;
}


// ============================================================================
// MEDIAN OF THE GOOD SAMPLES IN THE WINDOW
// ============================================================================

static float windowMedian(const UltrasonicSensor *s, uint8_t *goodOut)
{
    float sorted[US_SAMPLE_WINDOW];
    uint8_t n = 0;

    for (uint8_t i = 0; i < US_SAMPLE_WINDOW; i++) {
        if (s->samples[i] > 0.0f) {
            sorted[n++] = s->samples[i];
        }
    }

    *goodOut = n;

    if (n == 0) {
        return US_INVALID_CM;
    }

    // Insertion sort -- n is at most US_SAMPLE_WINDOW.
    for (uint8_t i = 1; i < n; i++) {
        float key = sorted[i];
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
    return (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5f;
}


// ============================================================================
// SPIKE GATE
// ============================================================================
//
// The symptom this exists for: a reading jumping from ~10 cm to 100+ cm in
// one sample. That is almost always a missed echo followed by a late or
// reflected one, not the obstacle teleporting away.
//
// The gate is ASYMMETRIC, on purpose:
//
//   * A sample claiming something is much FARTHER than we currently believe
//     is treated as suspect and ignored until US_SPIKE_CONFIRM consecutive
//     samples agree. Being slow to believe "the path cleared" is safe.
//
//   * A sample claiming something is CLOSER is ACCEPTED IMMEDIATELY, always.
//     Being slow to believe "something is in front of me" is not safe.
// ============================================================================

static bool sampleIsSuspect(UltrasonicSensor *s, float cm)
{
    if (!s->valid || s->filteredCm <= 0.0f) {
        return false;                    // nothing to compare against yet
    }

    if (cm <= s->filteredCm) {
        return false;                    // closer -> always believed
    }

    return (cm - s->filteredCm) > US_MAX_JUMP_CM;
}


// ============================================================================
// HYSTERESIS + CONFIRMATION, RUN ONCE PER SERVICED TICK
// ============================================================================

static void updateHysteresis(UltrasonicSensor *s)
{
    // ------------------------------------------------------------------
    // NO TRUSTWORTHY READING.
    //
    // This is the single most important branch in the file. An invalid
    // sensor must NOT be able to clear an obstacle latch, because "no echo
    // came back" is not evidence that the path ahead is clear -- it is the
    // absence of evidence either way, and it looks identical to a cut wire.
    //
    // So: both counters reset (no partial progress toward a decision), the
    // warning flag drops (it is advisory and we have nothing to advise on),
    // and the LATCH IS LEFT EXACTLY AS IT WAS. An obstacle confirmed before
    // the sensor died stays confirmed until a valid sensor says otherwise.
    // ------------------------------------------------------------------
    if (!s->valid || s->filteredCm <= 0.0f) {
        s->nearCount = 0;
        s->farCount  = 0;
        s->warning   = false;
        return;                                  // latch deliberately untouched
    }

    if (s->filteredCm <= SAFETY_STOP_DISTANCE_CM) {
        s->farCount = 0;
        if (s->nearCount < 255) s->nearCount++;
    } else if (s->filteredCm >= SAFETY_CLEAR_DISTANCE_CM) {
        s->nearCount = 0;
        if (s->farCount < 255) s->farCount++;
    } else {
        // Inside the hysteresis dead band: neither confirming nor denying.
        // Reset both runs so a transition always needs a fresh consecutive
        // run that starts on the correct side of the band.
        s->nearCount = 0;
        s->farCount  = 0;
    }

    if (s->nearCount >= SAFETY_OBSTACLE_CONFIRM) {
        s->obstacle = true;
    } else if (s->farCount >= SAFETY_CLEAR_CONFIRM) {
        s->obstacle = false;
    }

    s->warning = (s->filteredCm <= SAFETY_WARN_DISTANCE_CM);
}


static void serviceSensor(UltrasonicSensor *s)
{
    float cm = pingRaw(s);

    s->lastRawCm = cm;

    // Health accounting. A run of bad samples is what distinguishes "this
    // sensor is broken" from "one echo got scattered".
    if (cm > 0.0f) {
        s->timeoutStreak = 0;
    } else if (s->timeoutStreak < 0xFFFF) {
        s->timeoutStreak++;
    }

    bool accept = true;

    if (cm > 0.0f && sampleIsSuspect(s, cm)) {
        s->spikeCount++;
        if (s->spikeCount < US_SPIKE_CONFIRM) {
            // Not yet confirmed -- drop this sample. The window is left
            // untouched so the previous filtered value stands.
            accept = false;
        } else {
            // Confirmed by repetition: accept it and let the median migrate.
            s->spikeCount = 0;
        }
    } else {
        s->spikeCount = 0;
    }

    if (accept) {
        s->samples[s->writeIndex] = cm;
        s->writeIndex++;
        if (s->writeIndex >= US_SAMPLE_WINDOW) {
            s->writeIndex = 0;
        }

        if (cm > 0.0f) {
            s->lastGoodMs = millis();
        }
    }

    // Validity is re-evaluated on EVERY tick, whether or not the sample was
    // accepted. Without this, a sensor stuck emitting nothing but suspect
    // samples would keep its last "valid" flag forever and never be reported
    // as faulty.
    uint8_t good = 0;
    float median = windowMedian(s, &good);
    s->goodCount = good;

    bool fresh = (millis() - s->lastGoodMs) <= US_STALE_MS;

    if (good >= US_MIN_GOOD_SAMPLES && median > 0.0f && fresh) {
        s->filteredCm = median;
        s->valid = true;
    } else {
        s->valid = false;
        if (!fresh) {
            // Sensor has gone quiet. Do not keep publishing a stale number.
            s->filteredCm = US_INVALID_CM;
        }
    }

    // Hysteresis runs on the freshly-updated filtered value, every tick,
    // including ticks where the sample was rejected or the sensor is invalid.
    updateHysteresis(s);
}


// ============================================================================
// ============================================================================
//  REAR OBSTACLE LATCH  --  THREE VL53L0X, SAME HYSTERESIS RULES AS THE FRONT
// ============================================================================
//
// This is deliberately a near-mirror of updateHysteresis() above, in
// millimetres instead of centimetres. It is NOT a simplified version: the
// rear gate is the only thing standing between a reversing rover and whatever
// is behind it, and a weaker rule here would be a weaker rule where it counts.
//
//        0          STOP(250)      CLEAR(400)     WARN(600)
//        |--------------|--------------|-------------|--------->
//        |  latch SET   |  dead band   | latch CLEAR |  advisory
//                       |<-hysteresis->|
//
//   * filtered <= STOP  for REAR_OBSTACLE_CONFIRM ticks  -> latch SET
//   * filtered >= CLEAR for REAR_CLEAR_CONFIRM    ticks  -> latch CLEARED
//   * anywhere BETWEEN them                              -> latch UNCHANGED
//   * NO TRUSTWORTHY READING                             -> latch UNCHANGED,
//                                                           both counters reset
//
// That last line is the one that matters most. A VL53L0X that times out, errors
// on the bus, or reports out-of-range has told us NOTHING about what is behind
// the rover. It must not be able to release a stop that a working sensor set.
//
// WHY THE THRESHOLDS ARE PROVISIONAL: see the commissioning note beside
// REAR_STOP_DISTANCE_MM in config.h. They are cautious starting values, not a
// measurement of this rover's reverse stopping distance.
// ============================================================================

typedef struct {
    bool     obstacle;        // LATCHED. Only the counters below change it.
    uint8_t  nearCount;       // consecutive ticks with filtered <= STOP
    uint8_t  farCount;        // consecutive ticks with filtered >= CLEAR
    bool     warning;         // advisory: filtered <= WARN
} RearLatch;

static RearLatch gRearLatch[REAR_TOF_SENSOR_COUNT];

static bool gRearObstacle = false;
static bool gRearWarning  = false;


static void rearLatchInit(void)
{
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        // Boot state: NO obstacle latched, but also no validity yet, so the
        // sensor-fault path is what holds reverse until real measurements
        // arrive. Starting latched would be a fabricated obstacle.
        gRearLatch[i].obstacle  = false;
        gRearLatch[i].nearCount = 0;
        gRearLatch[i].farCount  = 0;
        gRearLatch[i].warning   = false;
    }

    gRearObstacle = false;
    gRearWarning  = false;
}


static void rearLatchUpdateOne(uint8_t i)
{
    RearLatch *L = &gRearLatch[i];

    int mm = rearTofFilteredMm(i);

    // ------------------------------------------------------------------
    // NO TRUSTWORTHY READING.
    //
    // Same reasoning as the front, and the same consequence: both counters
    // reset so no partial progress toward a decision survives, the advisory
    // warning drops because we have nothing to advise on, and THE LATCH IS
    // LEFT EXACTLY AS IT WAS.
    //
    // An out-of-range answer takes this branch too. "I see nothing within
    // 2 metres" is genuinely not a distance, and while it strongly suggests a
    // clear path, letting it drive the CLEAR counter would mean a sensor that
    // has been knocked out of alignment -- and so sees nothing, forever --
    // would silently authorise reversing.
    // ------------------------------------------------------------------
    if (!rearTofValid(i) || mm <= 0) {
        L->nearCount = 0;
        L->farCount  = 0;
        L->warning   = false;
        return;                                  // latch deliberately untouched
    }

    if (mm <= REAR_STOP_DISTANCE_MM) {
        L->farCount = 0;
        if (L->nearCount < 255) L->nearCount++;
    } else if (mm >= REAR_CLEAR_DISTANCE_MM) {
        L->nearCount = 0;
        if (L->farCount < 255) L->farCount++;
    } else {
        // Inside the hysteresis dead band: neither confirming nor denying.
        L->nearCount = 0;
        L->farCount  = 0;
    }

    if (L->nearCount >= REAR_OBSTACLE_CONFIRM) {
        L->obstacle = true;
    } else if (L->farCount >= REAR_CLEAR_CONFIRM) {
        L->obstacle = false;
    }

    L->warning = (mm <= REAR_WARN_DISTANCE_MM);
}


static void rearLatchUpdate(void)
{
    bool obstacle = false;
    bool warning  = false;

    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        rearLatchUpdateOne(i);

        if (gRearLatch[i].obstacle) obstacle = true;
        if (gRearLatch[i].warning)  warning  = true;
    }

    // OR across the three, never averaged and never compared against each
    // other. They cover different cones with no guaranteed relationship, so
    // each is judged on its own evidence and any one of them is enough to stop
    // the rover reversing.
    gRearObstacle = obstacle;
    gRearWarning  = warning;
}


// ============================================================================
// REAR ACCESSORS
// ============================================================================

bool rearSensingUsable(void)
{
    return rearTofAnyValid();
}

const char *rearBackendName(void)
{
    return rearTofBackendName();
}

int safetyRearDistanceMm(uint8_t index)
{
    return rearTofFilteredMm(index);
}

bool safetyRearSensorValid(uint8_t index)
{
    return rearTofValid(index);
}

bool safetyRearSensorObstacle(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return false;
    return gRearLatch[index].obstacle;
}

bool safetyRearSensorWarning(uint8_t index)
{
    if (index >= REAR_TOF_SENSOR_COUNT) return false;
    return gRearLatch[index].warning;
}

// An invalid sensor never fabricates an obstacle, and never clears one.
bool safetyRearObstacleDetected(void)
{
    return gRearObstacle;
}

bool safetyRearWarning(void)
{
    return gRearWarning;
}

bool safetyRearSensorFault(void)
{
    if (!rearTofBackendAvailable()) {
        // No backend at all. Reported as "unconfigured" rather than as a
        // sensor fault -- see safetyBlockReason(). The distinction matters:
        // one is a rover with broken sensors, the other is a firmware that
        // has not been told an address.
        return false;
    }

#if REAR_REQUIRE_ALL_SENSORS
    // Conservative default, matching SAFETY_REQUIRE_BOTH_SENSORS at the front.
    // A faulty rear sensor is not a rear sensor reporting "clear"; the cone it
    // was covering is unknown space, and we refuse to reverse into it.
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (!rearTofValid(i)) return true;
    }
    return false;
#else
    // DEGRADED operation: fault only when NOTHING trustworthy looks backward.
    // Accepts a real blind spot across the other cones -- see config.h.
    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (rearTofValid(i)) return false;
    }
    return true;
#endif
}

bool safetyRearSensingAvailable(void)
{
    return rearTofBackendAvailable();
}

int safetyClosestRearMm(void)
{
    int closest = REAR_TOF_INVALID_MM;

    for (uint8_t i = 0; i < REAR_TOF_SENSOR_COUNT; i++) {
        if (!rearTofValid(i)) {
            continue;
        }
        int mm = rearTofFilteredMm(i);
        if (mm <= 0) {
            continue;
        }
        if (closest <= 0 || mm < closest) {
            closest = mm;
        }
    }

    return closest;
}


// ============================================================================
// INITIALISATION
// ============================================================================

void safetyInit(void)
{
    // ------------------------------------------------------------------
    // Apply the front sensor side map from config.h: hardware channel
    // A/B -> side. This is a LABELLING decision only. Both sensors are
    // treated independently by the obstacle logic regardless of which way
    // round this resolves, so a wrong map mislabels but cannot endanger.
    // ------------------------------------------------------------------
#if SENSOR_MAP_SWAP
    gSensors[US_FRONT_LEFT ].trigPin = US_B_TRIG_PIN;
    gSensors[US_FRONT_LEFT ].echoPin = US_B_ECHO_PIN;
    gSensors[US_FRONT_RIGHT].trigPin = US_A_TRIG_PIN;
    gSensors[US_FRONT_RIGHT].echoPin = US_A_ECHO_PIN;
#else
    gSensors[US_FRONT_LEFT ].trigPin = US_A_TRIG_PIN;
    gSensors[US_FRONT_LEFT ].echoPin = US_A_ECHO_PIN;
    gSensors[US_FRONT_RIGHT].trigPin = US_B_TRIG_PIN;
    gSensors[US_FRONT_RIGHT].echoPin = US_B_ECHO_PIN;
#endif

    gSensors[US_FRONT_LEFT ].name = "FRONT_LEFT";
    gSensors[US_FRONT_RIGHT].name = "FRONT_RIGHT";

    for (uint8_t i = 0; i < US_SENSOR_COUNT; i++) {
        UltrasonicSensor *s = &gSensors[i];

        pinMode(s->trigPin, OUTPUT);
        digitalWrite(s->trigPin, LOW);

        // ECHO is INPUT only. The external resistive divider must bring the
        // HC-SR04's 5V echo down to ~3.3V; these GPIOs are not 5V tolerant.
        // Never use INPUT_PULLUP here -- it would fight the divider and skew
        // the threshold.
        pinMode(s->echoPin, INPUT);

        for (uint8_t k = 0; k < US_SAMPLE_WINDOW; k++) {
            s->samples[k] = US_INVALID_CM;
        }

        s->writeIndex    = 0;
        s->filteredCm    = US_INVALID_CM;
        s->lastRawCm     = US_INVALID_CM;
        s->valid         = false;
        s->lastGoodMs    = millis();
        s->goodCount     = 0;
        s->spikeCount    = 0;

        // Boot state: NO obstacle latched, but also no validity, so nothing
        // can move forward until real samples arrive. We do not start latched
        // because that would be a fabricated obstacle -- the sensor-fault
        // path is what blocks motion until the sensors prove themselves.
        s->obstacle      = false;
        s->nearCount     = 0;
        s->farCount      = 0;
        s->warning       = false;
        s->timeoutStreak = 0;
    }

    // Rear measurement layer. Brings up all three VL53L0X, one at a time,
    // each behind its own TCA channel. Returns false when the multiplexer
    // address is unconfirmed or no sensor answered -- which is reported, not
    // worked around.
    (void)rearTofInit();

    // Rear safety latches. Reset regardless of whether the sensors came up:
    // a latch left over from a previous run would be an obstacle nobody
    // measured.
    rearLatchInit();

    gActiveSensor = 0;
    gNextPingMs   = millis();
    gObstacle     = false;
    gWarning      = false;
}


// ============================================================================
// SCHEDULED UPDATE
// ============================================================================

void safetyUpdate(void)
{
    // ------------------------------------------------------------------
    // REAR first, and on its OWN schedule.
    //
    // Both of these calls are cheap and non-blocking: rearTofUpdate() collects
    // at most one already-completed VL53L0X result per tick, and the latch
    // update is arithmetic on values that are already in RAM.
    //
    // The rear path runs on EVERY pass, not only on a front-ping tick, and
    // that is deliberate: the front ping below can early-return for up to
    // US_PING_INTERVAL_MS, and the rear gate must not inherit the front
    // sensor's cadence.
    // ------------------------------------------------------------------
    rearTofUpdate();
    rearLatchUpdate();

    // Signed comparison handles millis() rollover correctly.
    if ((int32_t)(millis() - gNextPingMs) < 0) {
        return;
    }

    // Exactly ONE sensor per tick, strictly alternating. The two HC-SR04 are
    // NEVER triggered simultaneously and never within US_PING_INTERVAL_MS of
    // each other -- that is what causes cross-talk, where one sensor hears
    // the other's burst and reports a constant bogus distance.
    serviceSensor(&gSensors[gActiveSensor]);

    gActiveSensor = (gActiveSensor + 1) % US_SENSOR_COUNT;
    gNextPingMs   = millis() + US_PING_INTERVAL_MS;

    // ------------------------------------------------------------------
    // Aggregate the per-sensor LATCHES. Note this reads s->obstacle, the
    // confirmed hysteresis output -- NOT a bare distance comparison.
    //
    // The two sensors are aggregated with OR and are never averaged or
    // compared against each other. They are not mounted parallel and have
    // no guaranteed relationship, so each one's cone is judged entirely on
    // its own evidence, and either one is enough to stop the rover.
    // ------------------------------------------------------------------
    bool obstacle = false;
    bool warning  = false;

    for (uint8_t i = 0; i < US_SENSOR_COUNT; i++) {
        if (gSensors[i].obstacle) obstacle = true;
        if (gSensors[i].warning)  warning  = true;
    }

    gObstacle = obstacle;
    gWarning  = warning;
}


// ============================================================================
// FRONT ACCESSORS
// ============================================================================

float sensorFilteredCm(uint8_t sensorIndex)
{
    if (sensorIndex >= US_SENSOR_COUNT) return US_INVALID_CM;
    return gSensors[sensorIndex].filteredCm;
}

float sensorRawCm(uint8_t sensorIndex)
{
    if (sensorIndex >= US_SENSOR_COUNT) return US_INVALID_CM;
    return gSensors[sensorIndex].lastRawCm;
}

bool sensorValid(uint8_t sensorIndex)
{
    if (sensorIndex >= US_SENSOR_COUNT) return false;
    return gSensors[sensorIndex].valid;
}

uint8_t sensorGoodSampleCount(uint8_t sensorIndex)
{
    if (sensorIndex >= US_SENSOR_COUNT) return 0;
    return gSensors[sensorIndex].goodCount;
}

uint16_t sensorTimeoutStreak(uint8_t sensorIndex)
{
    if (sensorIndex >= US_SENSOR_COUNT) return 0;
    return gSensors[sensorIndex].timeoutStreak;
}


// ============================================================================
// HEALTH
// ============================================================================
//
// "Repeatedly timing out" is explicitly NOT healthy. A sensor reports OK only
// when every slot in its window holds a good sample and it is not currently
// in a run of failures.
// ============================================================================

SensorHealth sensorHealthOf(uint8_t sensorIndex)
{
    if (sensorIndex >= US_SENSOR_COUNT) {
        return HEALTH_FAULT;
    }

    const UltrasonicSensor *s = &gSensors[sensorIndex];

    bool fresh = (millis() - s->lastGoodMs) <= US_STALE_MS;

    // Dead, unplugged, or never worked since boot.
    if (!fresh ||
        s->goodCount == 0 ||
        s->timeoutStreak >= US_HEALTH_FAULT_STREAK) {
        return HEALTH_FAULT;
    }

    // Working, but losing samples -- report it rather than hiding it.
    if (s->goodCount < US_SAMPLE_WINDOW ||
        s->timeoutStreak >= US_HEALTH_DEGRADED_STREAK ||
        !s->valid) {
        return HEALTH_DEGRADED;
    }

    return HEALTH_OK;
}

const char *sensorHealthName(SensorHealth h)
{
    switch (h) {
        case HEALTH_OK:       return "OK";
        case HEALTH_DEGRADED: return "DEGRADED";
        case HEALTH_FAULT:    return "FAULT";
        default:              return "UNKNOWN";
    }
}

SensorHealth sensorHealthWorst(void)
{
    SensorHealth worst = HEALTH_OK;

    for (uint8_t i = 0; i < US_SENSOR_COUNT; i++) {
        SensorHealth h = sensorHealthOf(i);
        if (h > worst) {
            worst = h;      // enum is ordered OK < DEGRADED < FAULT
        }
    }

    return worst;
}

bool sensorMapVerified(void)
{
    return (SENSOR_MAP_VERIFIED != 0);
}


// ============================================================================
// SAFETY ASSESSMENT
// ============================================================================

bool safetyObstacleDetected(void)
{
    return gObstacle;
}

bool safetyWarning(void)
{
    return gWarning;
}

bool safetySensorFault(void)
{
#if SAFETY_REQUIRE_BOTH_SENSORS
    // Conservative default. A faulty sensor is not a sensor reporting "clear",
    // so the cone it was covering counts as unknown, and we refuse to drive
    // forward into unknown space.
    return !(gSensors[US_FRONT_LEFT].valid && gSensors[US_FRONT_RIGHT].valid);
#else
    // DEGRADED single-sensor operation. Fault only when we have NOTHING
    // trustworthy looking forward. Accepts a real blind spot -- see config.h.
    return !(gSensors[US_FRONT_LEFT].valid || gSensors[US_FRONT_RIGHT].valid);
#endif
}

bool safetyForwardBlocked(void)
{
    return safetyObstacleDetected() || safetySensorFault();
}

// ---------------------------------------------------------------------------
// REVERSE GATE.
//
// Note this is a RUNTIME test, not a compile-time one. The previous revision
// selected between "real sensors" and "policy default" with #if, because
// whether a backend existed was a build-time fact. It is now a runtime fact:
// the multiplexer address can be confirmed in config.h and the hardware still
// fail to answer, and in that case the rover must fall back to the operator's
// explicit unguarded-reverse decision rather than to whatever the compiler
// happened to bake in.
// ---------------------------------------------------------------------------
bool safetyReverseBlocked(void)
{
    if (safetyRearSensingAvailable()) {
        // Real sensors, real gate.
        return safetyRearObstacleDetected() || safetyRearSensorFault();
    }

    // No usable rear sensing. Whether reverse is permitted is an explicit
    // operator decision in config.h, not a silent default.
    return (SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED != 0);
}


// ============================================================================
// DIRECTION-AWARE GATE
// ============================================================================
//
// The rover is DIFFERENTIAL DRIVE: it has no steering linkage, and every turn
// is produced purely by the difference between left and right wheel speeds.
// So "which way am I going?" is derived PER WHEEL from the sign of that
// wheel's commanded speed. There is no direction field and no steering angle
// anywhere in this logic.
//
//   wheel cmd > 0  ->  that wheel is driving FORWARD  -> front-guarded
//   wheel cmd < 0  ->  that wheel is driving REVERSE  -> rear-guarded
//   wheel cmd == 0 ->  ungated
//
// Worked examples with a confirmed FRONT obstacle, in COMPONENT mode:
//
//   ( 150,  150)  straight forward  -> (   0,   0)   both clamped
//   (  80,  150)  forward arc       -> (   0,   0)   both clamped
//   (-100,  100)  pivot left        -> (-100,   0)   only the forward wheel
//   (-150, -150)  straight reverse  -> (-150,-150)   untouched
//
// In WHOLE mode any clamp collapses the whole command to (0,0). That is the
// stricter choice, and a defensible one: a rover pivoting on the spot still
// sweeps its front corners through space and can strike something ahead.
// Select it with SAFETY_GATE_MODE in config.h.
//
// Both FRONT sensors gate ALL forward wheels. We do not try to gate a left
// turn on only the left sensor: the sensors are not mounted parallel, their
// cones overlap by an unknown amount, and the rover's body is wider than
// either cone. Attributing a turn to one sensor would be a guess dressed up
// as logic, and the failure mode is driving into something.
// ============================================================================

void safetyGateMotion(int leftCmd, int rightCmd, int *leftOut, int *rightOut)
{
    int l = leftCmd;
    int r = rightCmd;

    if (safetyForwardBlocked()) {
        if (l > 0) l = 0;
        if (r > 0) r = 0;
    }

    if (safetyReverseBlocked()) {
        if (l < 0) l = 0;
        if (r < 0) r = 0;
    }

#if SAFETY_GATE_MODE == SAFETY_GATE_MODE_WHOLE
    if (l != leftCmd || r != rightCmd) {
        l = 0;
        r = 0;
    }
#endif

    if (leftOut)  *leftOut  = l;
    if (rightOut) *rightOut = r;
}

bool safetyMotionAllowed(int leftCmd, int rightCmd)
{
    int l = 0;
    int r = 0;
    safetyGateMotion(leftCmd, rightCmd, &l, &r);
    return (l == leftCmd && r == rightCmd);
}

const char *safetyBlockReason(int leftCmd, int rightCmd)
{
    if (safetyMotionAllowed(leftCmd, rightCmd)) {
        return "NONE";
    }

    bool wantsForward = (leftCmd > 0 || rightCmd > 0);
    bool wantsReverse = (leftCmd < 0 || rightCmd < 0);

    // Obstacle takes priority over fault: it is the more specific, more
    // actionable fact. Front takes priority over rear because the front
    // sensors are the ones that actually work today.
    if (wantsForward && safetyObstacleDetected()) return "FRONT_OBSTACLE";
    if (wantsForward && safetySensorFault())      return "FRONT_SENSOR_FAULT";

    if (wantsReverse) {
        if (safetyRearSensingAvailable()) {
            if (safetyRearObstacleDetected()) return "REAR_OBSTACLE";
            if (safetyRearSensorFault())      return "REAR_SENSOR_FAULT";
        } else {
            return "REAR_UNCONFIGURED";
        }
    }

    return "BLOCKED";
}

float safetyClosestFrontCm(void)
{
    float closest = US_INVALID_CM;

    for (uint8_t i = 0; i < US_SENSOR_COUNT; i++) {
        if (!gSensors[i].valid || gSensors[i].filteredCm <= 0.0f) {
            continue;
        }
        if (closest <= 0.0f || gSensors[i].filteredCm < closest) {
            closest = gSensors[i].filteredCm;
        }
    }

    return closest;
}

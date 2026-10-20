#include <Arduino.h>
#include <atomic>

#include "stall_guard.h"
#include "config.h"

// See stall_guard.h for what this does and, as importantly, what it does not.

// ============================================================================
// PLATFORM: critical section and raw direction-pin access
// ============================================================================
//
// ESP32: a portMUX spinlock, which also works ACROSS cores (the guard runs on
// core 0, loop() on core 1). Inside it, only single writes to the GPIO
// write-1-to-set / write-1-to-clear registers and reads of the output latch:
// nothing that can wait, log or take another lock.
//
// Host: single-threaded tests; the "concurrency" is the fake clock calling
// stallGuardTick() from inside a simulated hung transaction. GPIO goes through
// the harness's digitalWrite()/digitalRead().
// ============================================================================

#ifdef ARDUINO_ARCH_ESP32

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"

static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
#define GUARD_LOCK()   portENTER_CRITICAL(&gMux)
#define GUARD_UNLOCK() portEXIT_CRITICAL(&gMux)

static inline void rawWrite(uint8_t pin, uint8_t level)
{
    if (pin < 32) {
        if (level) GPIO.out_w1ts = (1UL << pin);
        else       GPIO.out_w1tc = (1UL << pin);
    } else {
        if (level) GPIO.out1_w1ts.val = (1UL << (pin - 32));
        else       GPIO.out1_w1tc.val = (1UL << (pin - 32));
    }
}

static inline bool rawIsHigh(uint8_t pin)
{
    return (pin < 32) ? (((GPIO.out >> pin) & 1UL) != 0)
                      : (((GPIO.out1.val >> (pin - 32)) & 1UL) != 0);
}

#else

#define GUARD_LOCK()   do { } while (0)
#define GUARD_UNLOCK() do { } while (0)

static inline void rawWrite(uint8_t pin, uint8_t level) { digitalWrite(pin, level ? HIGH : LOW); }
static inline bool rawIsHigh(uint8_t pin)               { return digitalRead(pin) == HIGH; }

// Host tests only: model xTaskCreatePinnedToCore() failing in stallGuardBegin().
bool gStallGuardHostCreateFails = false;

#endif


// ============================================================================
// STATE
// ============================================================================

static const uint8_t kDirPins[8] = {
    M_FRONT_A_IN1, M_FRONT_A_IN2, M_FRONT_B_IN1, M_FRONT_B_IN2,
    M_REAR_A_IN1,  M_REAR_A_IN2,  M_REAR_B_IN1,  M_REAR_B_IN2
};

static std::atomic<uint32_t> gBeatMs(0);
static std::atomic<bool>     gArmed(false);
static std::atomic<bool>     gCreated(false);   // the task was created
static std::atomic<uint32_t> gTripCount(0);

// Liveness: written only by the tick path (stallGuardTick()), and reset by
// stallGuardBegin() before the task exists.
static std::atomic<bool>     gTicked(false);    // at least one tick since stallGuardBegin()
static std::atomic<uint32_t> gTickMs(0);        // nowMs of the latest tick

// loop() side only (stallGuardOperational()): the tick stamp the guard was
// last found stale at. It can only make the guard read MORE stale, never
// fresh: it keeps a dead task dead even when, ~49.7 days on, millis() wraps
// round to a value close to its last stamp again.
static bool     gStaleSeen   = false;
static uint32_t gStaleTickMs = 0;

// Written only inside GUARD_LOCK.
static volatile bool gTripped     = false;  // sticky: refuses direction-HIGH writes
static volatile bool gTripEvidence = false; // not yet consumed by the motor layer

static bool anyDirectionHigh(void)
{
    for (int i = 0; i < 8; i++) {
        if (rawIsHigh(kDirPins[i])) return true;
    }
    return false;
}

static void cutAllDirections(void)
{
    for (int i = 0; i < 8; i++) {
        rawWrite(kDirPins[i], LOW);
    }
}


// ============================================================================
// TASK (ESP32 only)
// ============================================================================

#ifdef ARDUINO_ARCH_ESP32

static void guardTask(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(STALL_GUARD_PERIOD_MS));
        (void)stallGuardTick(millis());
    }
}

#endif

void stallGuardBegin(void)
{
    // Boot state: disarmed, not tripped (what a cold boot's zeroed RAM gives;
    // explicit so a re-run of setup() -- host tests -- starts clean too).
    GUARD_LOCK();
    gTripped      = false;
    gTripEvidence = false;
    GUARD_UNLOCK();
    gArmed.store(false);
    gTripCount.store(0);
    gBeatMs.store(millis());

    // Not operational until the task's FIRST tick, whatever creation returns.
    gTicked.store(false);
    gTickMs.store(0);
    gStaleSeen   = false;
    gStaleTickMs = 0;

#ifdef ARDUINO_ARCH_ESP32
    // The task itself is created exactly once.
    static bool started = false;
    if (started) {
        return;
    }
    started = true;

    // Core 0, above loop() (priority 1 on core 1) and the idle tasks, below
    // the system's esp_timer (22) and IPC (24) tasks. Each tick is a few
    // register reads; the stack is generous for that.
    BaseType_t ok = xTaskCreatePinnedToCore(guardTask, "stallGuard",
                                            STALL_GUARD_TASK_STACK_BYTES, NULL,
                                            STALL_GUARD_TASK_PRIORITY, NULL, 0);
    gCreated.store(ok == pdPASS);
#else
    gCreated.store(!gStallGuardHostCreateFails);
#endif

    // Wait, bounded, for the task's first REAL tick, so the drive state that
    // READY reports is the guard's actual one. delay() yields to the task.
    if (gCreated.load()) {
        uint32_t t0 = millis();
        while (!gTicked.load() && (uint32_t)(millis() - t0) < STALL_GUARD_LIVENESS_MS) {
            delay(1);
        }
    }
}

void stallGuardArm(void)
{
    gBeatMs.store(millis());
    gArmed.store(gCreated.load());
}

void stallGuardHeartbeat(void)
{
    gBeatMs.store(millis());
}


// ============================================================================
// THE CHECK
// ============================================================================

bool stallGuardTick(uint32_t nowMs)
{
    // Liveness, armed or not: this call IS the guard running. The stamp is
    // stored before the flag, so a reader that sees the flag sees a stamp.
    gTickMs.store(nowMs);
    gTicked.store(true);

    if (!gArmed.load()) {
        return false;
    }

    bool newTrip = false;

    GUARD_LOCK();
    if (!gTripped) {
        // Signed age: loop() may store a heartbeat newer than nowMs between
        // the two reads (it runs on the other core). That must read as "fresh",
        // never as a 4-billion-ms-old heartbeat. Wrap-safe for any age < 2^31.
        int32_t age = (int32_t)(nowMs - gBeatMs.load());
        if (age > (int32_t)STALL_GUARD_T_STALL_MS && anyDirectionHigh()) {
            gTripped      = true;
            gTripEvidence = true;
            newTrip       = true;
        }
    }
    if (gTripped) {
        cutAllDirections();              // and again on every tick while tripped
    }
    GUARD_UNLOCK();

    if (newTrip) {
        gTripCount.fetch_add(1);
    }
    return newTrip;
}

bool stallGuardSetDirection(uint8_t pinA, uint8_t levelA, uint8_t pinB, uint8_t levelB)
{
    bool written = false;

    // A guard that is not ticking could not cut this write if loop() then
    // stalls: refuse it. Checked outside the critical section (it reads the
    // clock); a guard that stops in the microseconds between is the drive-
    // availability check's job, on the next call.
    bool live = stallGuardOperational();

    GUARD_LOCK();
    if (!gTripped && live) {
        rawWrite(pinA, levelA);
        rawWrite(pinB, levelB);
        written = true;
    }
    GUARD_UNLOCK();

    return written;
}

bool stallGuardConsumeTripEvidence(void)
{
    bool evidence;

    GUARD_LOCK();
    evidence      = gTripEvidence;
    gTripEvidence = false;
    GUARD_UNLOCK();

    return evidence;
}

void stallGuardClearTrip(void)
{
    GUARD_LOCK();
    gTripped = false;
    GUARD_UNLOCK();

    // A fresh start: the loop that acknowledged is plainly making progress.
    gBeatMs.store(millis());
}

bool stallGuardOperational(void)
{
    if (!gCreated.load() || !gTicked.load()) {
        return false;                    // no task, or no tick from it yet
    }

    // Clock first, then the stamp: a tick landing in between reads as a few
    // ms in the FUTURE (age < 0), never as old. Signed, so a stamp on the
    // far side of the millis() wrap is still the right few ms away.
    uint32_t now  = millis();
    uint32_t tick = gTickMs.load();

    if (gStaleSeen && tick == gStaleTickMs) {
        return false;                    // no tick since it was found stale
    }

    int32_t age = (int32_t)(now - tick);
    if (age <= (int32_t)STALL_GUARD_LIVENESS_MS && age >= -(int32_t)STALL_GUARD_LIVENESS_MS) {
        return true;
    }

    gStaleSeen   = true;
    gStaleTickMs = tick;
    return false;
}

bool stallGuardArmed(void)       { return gArmed.load(); }
uint32_t stallGuardTrips(void)   { return gTripCount.load(); }

bool stallGuardTripped(void)
{
    bool tripped;
    GUARD_LOCK();
    tripped = gTripped;
    GUARD_UNLOCK();
    return tripped;
}

const char *stallGuardStateName(void)
{
    if (!stallGuardOperational()) return "NOT_RUNNING";
    if (stallGuardTripped())      return "TRIPPED";
    if (!stallGuardArmed())       return "DISARMED";
    return "ARMED";
}

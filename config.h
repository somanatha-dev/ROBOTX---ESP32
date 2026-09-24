#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// ROVER CONFIGURATION  --  ALL HARDWARE-SPECIFIC CONSTANTS LIVE HERE
//
// Target      : ESP32-WROOM-32
// Arduino core: esp32 2.0.14  <-- if anything here ever uses LEDC, the API is
//                                 ledcSetup/ledcAttachPin/ledcWrite(ch,duty).
//                                 DO NOT use core 3.x ledcAttach(pin,f,r);
//                                 it does not exist on 2.0.14.
//                                 As of the PCA9685 correction, the motor
//                                 subsystem uses NO LEDC channel at all --
//                                 motor speed comes from the PCA9685.
//
// Role split:
//   ESP32 = low-level motor control, front ultrasonic sensing, rear
//           time-of-flight sensing, immediate safety stop, command failsafe,
//           telemetry.
//   Pi    = camera / CV / navigation / mission logic. The Pi sends movement
//           commands; it is NOT a safety authority. The ESP32 has final say.
//
// DRIVE TYPE: DIFFERENTIAL. There is no steering mechanism of any kind.
// Every turn is produced by the difference between left and right wheel
// speeds. Nothing in this firmware may assume a steering angle exists.
// ============================================================================


// ============================================================================
// PIN MAP  --  CONFIRMED PHYSICAL WIRING. DO NOT REASSIGN.
// ============================================================================

// ---- Motor drivers ---------------------------------------------------------
// Two dual-channel drivers = FOUR independent motor channels.
// Each channel is a pair of direction/PWM inputs.
//
//   FRONT driver : IN1=23  IN2=25  IN3=26  IN4=27
//   REAR  driver : IN1=32  IN2=33  IN3=16  IN4=17
//
// NOTE: on ESP32-WROVER modules GPIO16/17 are consumed by PSRAM and are NOT
// usable. This code assumes an ESP32-WROOM-32 class module where 16/17 are
// free. If your rear IN3/IN4 channel never responds, check your module type
// first before suspecting the firmware.

#define M_FRONT_A_IN1   23
#define M_FRONT_A_IN2   25
#define M_FRONT_B_IN1   26
#define M_FRONT_B_IN2   27

#define M_REAR_A_IN1    32
#define M_REAR_A_IN2    33
#define M_REAR_B_IN1    16
#define M_REAR_B_IN2    17

// ---- Front ultrasonic ------------------------------------------------------
// EXACTLY TWO HC-SR04, BOTH FRONT-MOUNTED. There is no rear ultrasonic sensor
// on this rover; rear sensing is done with IR (see the REAR IR section).
//
// ECHO pins are fed through an external resistive divider (5V -> ~3.3V).
// GPIO14 and GPIO19 are normal 3.3V-tolerant IO. They are NOT 5V tolerant,
// so the divider is mandatory, not optional.

#define US_A_TRIG_PIN   13      // FRONT sensor "A"
#define US_A_ECHO_PIN   14
#define US_B_TRIG_PIN   18      // FRONT sensor "B"
#define US_B_ECHO_PIN   19

// Legacy aliases, kept so nothing that referenced the old names breaks.
#define US1_TRIG_PIN    US_A_TRIG_PIN
#define US1_ECHO_PIN    US_A_ECHO_PIN
#define US2_TRIG_PIN    US_B_TRIG_PIN
#define US2_ECHO_PIN    US_B_ECHO_PIN

// ---- I2C bus ---------------------------------------------------------------
// CONFIRMED WIRING: GPIO21 = SDA, GPIO22 = SCL.
//
// The bus IS now brought up (roverI2cInit() -> Wire.begin(21, 22)). Four
// devices are named on it: the PCA9685 motor-enable PWM driver, the TCA9548A
// rear-sensor multiplexer, three VL53L0X behind that multiplexer, and the
// NEO-9M GPS. See the I2C DEVICES section below for which of their addresses
// are confirmed and which are not.
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22

// Bus clock. 100 kHz standard mode is the conservative choice and is what the
// VL53L0X, TCA9548A and PCA9685 all support unconditionally. Raise to 400000
// only after the bus is proven and you have checked the pull-up values.
#define I2C_CLOCK_HZ    100000UL

// Per-transaction timeout. Without this a stuck slave holding SDA low blocks
// Wire forever and takes the safety loop down with it.
#define I2C_TIMEOUT_MS  50UL

// ---- Reserved / not driven by this firmware --------------------------------
// Listed for documentation only. This firmware never calls pinMode() on any
// of these, so they cannot be accidentally configured as outputs.
//
//   Encoders : GPIO34, GPIO35, GPIO36, GPIO39
//              These are INPUT-ONLY pins. They have NO internal pull-ups and
//              cannot be driven as outputs, which is why leaving them
//              untouched is safe. No encoder feature is implemented.
//   Pi UART  : GPIO1 (TX0), GPIO3 (RX0)    -- owned by Serial.

#define ENCODER_1_PIN   34
#define ENCODER_2_PIN   35
#define ENCODER_3_PIN   36
#define ENCODER_4_PIN   39


// ============================================================================
//  I 2 C   D E V I C E S
// ============================================================================
//
// FOUR device types share GPIO21/22:
//
//        ESP32 ---- SDA/SCL ----+---- PCA9685      (addr: NOT CONFIRMED)
//                               |
//                               +---- TCA9548A     (addr: NOT CONFIRMED)
//                               |       |
//                               |       +-- CH0 -> VL53L0X #0  @ 0x29
//                               |       +-- CH1 -> VL53L0X #1  @ 0x29
//                               |       +-- CH2 -> VL53L0X #2  @ 0x29
//                               |
//                               +---- NEO-9M GPS   @ 0x42  (not driven here)
//
// All three VL53L0X answer to the SAME address, 0x29. Three identical
// addresses cannot coexist on one bus segment, which is exactly why the
// TCA9548A is there: it connects one downstream segment at a time. Every
// access to a 0x29 device MUST therefore be preceded by a channel selection,
// and no two channels may ever be open together.
//
// TWO ADDRESSES ARE UNKNOWN AND ARE NOT GUESSED ANYWHERE IN THIS FIRMWARE.
// Each has an explicit _CONFIRMED flag. While a flag is 0 the corresponding
// subsystem refuses to operate and says so in telemetry, rather than talking
// to an address nobody verified. Find them with:  {"cmd":"I2CSCAN"}
// ============================================================================

// ---- TCA9548A 8-channel I2C multiplexer ------------------------------------
//
//   *** THE ADDRESS BELOW IS A PLACEHOLDER. IT IS NOT A MEASUREMENT. ***
//
// The TCA9548A address is set by its A0/A1/A2 pins and, for this part, lies
// somewhere in 0x70..0x77. NOTHING in the confirmed hardware information says
// which. The value below is the part's power-on-default (all address pins
// low) and is provided ONLY so the constant has a syntactically valid value;
// it is INERT until TCA9548A_ADDRESS_CONFIRMED is set to 1 by a human.
//
// TO COMMISSION:
//   1. {"cmd":"I2CSCAN"}                    -- list every responding address
//   2. {"cmd":"TCATEST","addr":<candidate>} -- prove it is really a TCA9548A
//      (writes a channel mask and reads the control register back; a PCA9685
//      sitting at the same address will NOT read back what you wrote)
//   3. set TCA9548A_I2C_ADDRESS below, set TCA9548A_ADDRESS_CONFIRMED to 1
//
// CAUTION: a PCA9685 with all six address jumpers bridged also lands in
// 0x70..0x77, so a bare scan can be ambiguous. Step 2 is what disambiguates.
#define TCA9548A_I2C_ADDRESS        0x70
#define TCA9548A_ADDRESS_CONFIRMED  0       // <-- SET TO 1 ONLY AFTER TCATEST

// Channel assignment. CONFIRMED by the operator.
// These are ELECTRICAL channel indices. They say nothing about which sensor is
// physically left, centre or right at the rear -- see REAR_TOF_ORIENTATION
// _VERIFIED below.
#define TCA_CH_REAR_TOF_0           0
#define TCA_CH_REAR_TOF_1           1
#define TCA_CH_REAR_TOF_2           2

// Channel count of the part, and the "nothing selected" mask.
#define TCA9548A_CHANNEL_COUNT      8
#define TCA9548A_DESELECT_MASK      0x00

// TCA9548A RESET pin disposition is NOT CONFIRMED. This firmware therefore
// never drives a reset line; bus recovery is done by re-selecting the channel.
// If you later confirm RESET is on a GPIO, add it here -- do not assume it.

// ---- PCA9685 16-channel PWM driver -----------------------------------------
//
//   *** THE ADDRESS BELOW IS A PLACEHOLDER. IT IS NOT A MEASUREMENT. ***
//
// Same rule as the TCA9548A. The PCA9685 base address is 0x40 with six solder
// jumpers A0..A5, giving 0x40..0x7F. The confirmed hardware information does
// not state the jumper state, so the address is UNKNOWN.
//
// Until PCA9685_ADDRESS_CONFIRMED is 1 the motor enable outputs are NEVER
// written and ALL MOTION IS REFUSED. That is deliberate: with ENA/ENB fed from
// this chip, driving the direction pins without a verified enable source would
// be commanding a motor through hardware nobody has identified.
#define PCA9685_I2C_ADDRESS         0x40
#define PCA9685_ADDRESS_CONFIRMED   0       // <-- SET TO 1 ONLY AFTER PCATEST

// PWM frequency presented to the L298N ENA/ENB inputs.
//
//   *** COMMISSIONING VALUE -- NOT DERIVED FROM CONFIRMED HARDWARE. ***
//
// The PCA9685 can produce 24 Hz .. 1526 Hz. 1000 Hz is the conventional choice
// for an L298N (its switching loss climbs steeply above a few kHz, and it is
// above the audible whine of lower frequencies). No document states what this
// rover's drivers actually want. Validate on blocks before trusting it.
#define PCA9685_PWM_FREQ_HZ         1000

// PCA9685 resolution is fixed at 12 bits by the part.
#define PCA9685_PWM_MAX             4095

// Motor enable channels. CONFIRMED by the operator.
//   FRONT L298N: ENA <- CH0 (gates IN1/IN2), ENB <- CH1 (gates IN3/IN4)
//   REAR  L298N: ENA <- CH2 (gates IN1/IN2), ENB <- CH3 (gates IN3/IN4)
#define PCA_CH_FRONT_A_EN           0
#define PCA_CH_FRONT_B_EN           1
#define PCA_CH_REAR_A_EN            2
#define PCA_CH_REAR_B_EN            3

// PCA9685 OE (output enable) pin disposition is NOT CONFIRMED. This firmware
// never drives an OE line. If OE is floating rather than tied low, the outputs
// are undefined and NOTHING here can fix that -- it is a wiring question.

// ---- NEO-9M GPS ------------------------------------------------------------
// CONFIRMED at 0x42. No driver is implemented (out of scope for this work).
// Declared so the I2C scanner can name it instead of reporting an unknown.
#define NEO9M_I2C_ADDRESS           0x42

// ---- VL53L0X ---------------------------------------------------------------
// CONFIRMED: all three answer at 0x29, the part's power-on default. This
// firmware does NOT reassign their addresses -- there is no confirmed XSHUT
// wiring, so address reassignment is not possible and is not attempted.
#define VL53L0X_I2C_ADDRESS         0x29


// ============================================================================
//  M O T O R   M A P   --   *** UNVERIFIED. YOU MUST CONFIRM THIS. ***
// ============================================================================
//
// THIS IS THE SINGLE PLACE where motor grouping (which channel is LEFT and
// which is RIGHT) and polarity (which way a channel spins) are configured.
// motor.cpp reads these and nothing else. Change them here, nowhere else.
//
// A GPIO list CANNOT tell us two things:
//
//   (1) SIDE   - is FRONT_A the front-LEFT wheel or the front-RIGHT wheel?
//                Nothing in the pin numbering implies this. It depends on
//                which driver output terminal each motor was screwed into.
//
//   (2) INVERT - does driving IN1 with IN2 low spin that wheel FORWARD or
//                BACKWARD? That depends on which way round the two motor
//                leads went into the terminal block. A swapped pair reverses
//                the wheel.
//
// That is 2 side-assignments x 2^4 polarities = 64 possible mappings. The
// values below are a PLACEHOLDER ASSUMPTION, not a measurement:
//     "A" channels = LEFT, "B" channels = RIGHT, no inversion.
//
// Resolve it with the MOTORTEST command, rover ON BLOCKS. Then edit the table
// below and set MOTOR_MAP_VERIFIED to 1.
//
// Until MOTOR_MAP_VERIFIED is 1, telemetry reports "motor_map_verified":false
// so the Raspberry Pi can refuse to run autonomously on an unverified rover.
// ============================================================================

#define MOTOR_SIDE_LEFT     0
#define MOTOR_SIDE_RIGHT    1

#define MOTOR_MAP_VERIFIED  0

//                          <-- VERIFY  (MOTOR_SIDE_LEFT / MOTOR_SIDE_RIGHT)
#define MOTOR_FRONT_A_SIDE  MOTOR_SIDE_LEFT
#define MOTOR_FRONT_B_SIDE  MOTOR_SIDE_RIGHT
#define MOTOR_REAR_A_SIDE   MOTOR_SIDE_LEFT
#define MOTOR_REAR_B_SIDE   MOTOR_SIDE_RIGHT

//                          <-- VERIFY  (0 = as wired, 1 = reverse this wheel)
#define MOTOR_FRONT_A_INVERT 0
#define MOTOR_FRONT_B_INVERT 0
#define MOTOR_REAR_A_INVERT  0
#define MOTOR_REAR_B_INVERT  0


// ============================================================================
//  F R O N T   S E N S O R   S I D E   M A P  -- *** UNVERIFIED ***
// ============================================================================
//
// The confirmed wiring tells us which GPIO pair belongs to which SENSOR.
// It does NOT tell us which sensor sits on which SIDE of the front assembly.
//
//     A (TRIG 13 / ECHO 14)  ->  FRONT LEFT      (assumed)
//     B (TRIG 18 / ECHO 19)  ->  FRONT RIGHT     (assumed)
//
// TO VERIFY: put a hand in front of ONE sensor only and watch telemetry.
// If front_right_cm drops when you cover the left-hand module, the map is
// backwards -- set SENSOR_MAP_SWAP to 1. Then set SENSOR_MAP_VERIFIED to 1.
//
// SAFETY NOTE: this mapping affects LABELS ONLY. The obstacle logic treats
// both sensors independently and blocks forward motion if EITHER sees
// something, so a swapped map cannot create a safety hole.
// ============================================================================

#define SENSOR_MAP_VERIFIED 0
#define SENSOR_MAP_SWAP     0


// ============================================================================
// MOTOR DRIVE ARCHITECTURE  --  CORRECTED TO MATCH THE CONFIRMED HARDWARE
// ============================================================================
//
// WHAT CHANGED, AND WHY
// ---------------------
// An earlier revision of this firmware PWM'd the eight L298N IN pins directly
// and stated "no ENA/ENB pin is invented anywhere". That premise was correct
// for the information available at the time and is now SUPERSEDED: the
// confirmed wiring DOES have ENA/ENB, driven from PCA9685 CH0..CH3.
//
// So the roles are now, exactly:
//
//     ESP32 GPIO  ->  L298N IN1..IN4   =  DIRECTION ONLY (digitalWrite)
//     PCA9685 CH  ->  L298N ENA/ENB    =  SPEED (PWM)
//
// The IN pins are NEVER PWM'd. Doing so while a separate enable pin exists
// multiplies two independent duty cycles together and makes the commanded
// speed meaningless.
//
// CONSEQUENCE: no LEDC channel is used by the motor subsystem any more. All
// eight are free. (If you ever do add an LEDC user, this core is esp32 2.0.14:
// use ledcSetup / ledcAttachPin / ledcWrite(channel, duty). The core 3.x
// ledcAttach(pin, freq, res) form does not exist here.)
//
// COMMANDED-SPEED SCALE
// ---------------------
// The Pi-facing API is unchanged: -255..+255. The sign selects the direction
// pin pattern; the magnitude is scaled to the PCA9685's 12-bit duty. Keeping
// 255 as the API maximum means the DRIVE protocol and every stored command
// value stay byte-for-byte what the Pi already sends.
#define PWM_MAX_DUTY        255

// Below this magnitude a brushed motor typically just buzzes without turning.
// Commands between 1 and this value are treated as stop, not as creep.
#define MOTOR_MIN_EFFECTIVE_DUTY 40

// Inner-wheel percentage used by the legacy arc helpers (turnLeft/turnRight
// and MOVE dir=L/R). Defined once here so the two call sites cannot drift.
// The native DRIVE protocol does not use this -- the Pi sets both wheels
// explicitly, which is the preferred interface.
#define MOTOR_TURN_INNER_PCT 55


// ============================================================================
// ULTRASONIC PROCESSING  (FRONT ONLY -- exactly two HC-SR04)
// ============================================================================

// Physical plausibility gate. Anything outside this band is discarded as a
// bad sample rather than being reported as a distance.
#define US_MIN_VALID_CM     2.0f
#define US_MAX_VALID_CM     300.0f

// pulseIn() timeout. Derived from US_MAX_VALID_CM:
//   300 cm round trip = 300 / 0.01715 = ~17500 us. 20000 us adds margin.
// This is also the WORST-CASE blocking time of one ping (~20 ms). Only one
// ping is issued per scheduler tick. The main loop calls commPoll() both
// BEFORE and AFTER safetyUpdate() so this window cannot stall command intake.
#define US_ECHO_TIMEOUT_US  20000UL

// Rolling median window per sensor.
#define US_SAMPLE_WINDOW    5
// A sensor is only declared VALID when at least this many of the last
// US_SAMPLE_WINDOW samples were good. This is what rejects isolated dropouts.
#define US_MIN_GOOD_SAMPLES 3

// Ping scheduling. Sensors are triggered STRICTLY ALTERNATELY, never
// together, to prevent cross-talk (sensor B hearing sensor A's burst).
//   30 ms between pings  ->  60 ms between pings of the SAME sensor,
//   which is the HC-SR04 datasheet-recommended minimum cycle.
// An earlier firmware used 5 ms, which is far too short: the previous burst
// is still echoing around the room when the next one fires. That is the
// classic cause of stuck readings and 10 cm -> 100 cm jumps.
#define US_PING_INTERVAL_MS 30

// If a sensor produces no valid reading for this long it is considered FAULTY
// (disconnected, dead, wiring fault) rather than "clear".
#define US_STALE_MS         1000UL

// Spike gate. A sample claiming to be much FARTHER than the current filtered
// value must be confirmed by US_SPIKE_CONFIRM consecutive samples before it
// is believed. Samples claiming to be CLOSER are ALWAYS accepted immediately
// -- failing toward "obstacle" is the safe direction.
#define US_MAX_JUMP_CM      60.0f
#define US_SPIKE_CONFIRM    2

#define US_SENSOR_COUNT     2
#define US_INVALID_CM       (-1.0f)     // internal sentinel, never published

// ---- Sensor health ---------------------------------------------------------
// "Repeatedly timing out" must NOT be reported as healthy. Health is derived
// from the run of consecutive bad (timeout / implausible) raw samples:
//
//   < DEGRADED_STREAK  and full window good  -> HEALTH_OK
//   >= DEGRADED_STREAK or partial window     -> HEALTH_DEGRADED
//   >= FAULT_STREAK    or stale              -> HEALTH_FAULT
//
// At 2 sensors x US_PING_INTERVAL_MS, one sensor is serviced every 60 ms, so
// a streak of 3 is ~180 ms and a streak of 8 is ~480 ms.
#define US_HEALTH_DEGRADED_STREAK 3
#define US_HEALTH_FAULT_STREAK    8


// ============================================================================
//  R E A R   T I M E - O F - F L I G H T   S E N S I N G   (3 x VL53L0X)
// ============================================================================
//
// CONFIRMED HARDWARE:
//   * THREE VL53L0X time-of-flight rangefinders, rear-mounted.
//   * All three answer at I2C address 0x29 (the part's power-on default).
//   * They are reached through a TCA9548A multiplexer:
//         TCA CH0 -> sensor 0     TCA CH1 -> sensor 1     TCA CH2 -> sensor 2
//
// TERMINOLOGY CORRECTION. An earlier revision of this firmware described the
// rear sensors as "IR PROXIMITY" sensors and refused, correctly, to publish a
// distance for them. A VL53L0X is an infrared (940 nm) device, which is where
// that description came from -- but it is a true RANGEFINDER. It measures
// time of flight and reports MILLIMETRES. So this firmware now does publish
// rear distances, because they are genuinely measured, and the old
// presence-only vocabulary has been retired.
//
// WHAT IS STILL UNKNOWN:
//   * The TCA9548A's I2C address -- see the I2C DEVICES section above. While
//     TCA9548A_ADDRESS_CONFIRMED is 0, NO rear sensor is ever accessed and
//     every reading reports TOF_UNINITIALISED. Nothing is fabricated.
//   * Which sensor is physically LEFT, CENTRE or RIGHT at the rear. Indices
//     0/1/2 are TCA channel numbers and nothing more.
//   * The correct obstacle thresholds for this rover's reverse stopping
//     distance -- the values below are COMMISSIONING VALUES.
// ============================================================================

#define REAR_TOF_SENSOR_COUNT       3

// Sensor indices. These are ELECTRICAL positions (TCA channel order), NOT
// physical left/centre/right. Do not rename them to sides until verified.
#define REAR_TOF_0                  0
#define REAR_TOF_1                  1
#define REAR_TOF_2                  2

// Is the physical left/centre/right arrangement confirmed? NO.
// This is a LABELLING flag only. Like the front sensors, any one of the three
// asserting an obstacle blocks reverse, so a wrong orientation map mislabels
// telemetry but cannot open a safety hole.
#define REAR_TOF_ORIENTATION_VERIFIED 0

// Derived: can the firmware actually reach the rear sensors? This is purely a
// question of whether the multiplexer address has been confirmed by a human.
#if TCA9548A_ADDRESS_CONFIRMED
  #define REAR_TOF_BACKEND_AVAILABLE  1
#else
  #define REAR_TOF_BACKEND_AVAILABLE  0
#endif

// ---- Measurement plausibility band -----------------------------------------
// Readings outside this band are DISCARDED as non-measurements, exactly as the
// HC-SR04 path does. They are never turned into a distance.
//
//   30 mm  : below the VL53L0X's dependable near limit; closer than this the
//            part reports erratically rather than usefully.
//   2000 mm: the part's optimistic maximum in default (long-ish) mode. Beyond
//            this it reports ~8190 mm to mean "nothing seen", which is an
//            OUT_OF_RANGE status, NOT a 8.19 m measurement and NOT an obstacle.
#define REAR_TOF_MIN_VALID_MM       30
#define REAR_TOF_MAX_VALID_MM       2000

// The library returns 65535 on I/O timeout. Named so no magic number appears
// in the driver.
#define REAR_TOF_TIMEOUT_SENTINEL   65535

// I/O timeout handed to the VL53L0X driver, per transaction.
#define REAR_TOF_IO_TIMEOUT_MS      100

// Measurement timing budget. Longer = more accurate and longer range, but
// slower. 33 ms is ST's default. Marked as a tuning value, not a hardware fact.
#define REAR_TOF_TIMING_BUDGET_US   33000UL

// VL53L0X I/O voltage mode, passed to the driver's init().
//
//   *** NOT CONFIRMED. *** The supply and level-shifting arrangement of these
//   three sensors is not in the confirmed hardware information.
//
// 1 = 2.8 V I/O (the driver's default, and what almost every breakout board
//     with an onboard regulator + level shifter wants)
// 0 = leave the part's 1.8 V-referenced default
//
// Left at the driver default. If a sensor initialises but ranges nonsensically
// at all distances, this is one of the first things to try flipping -- and
// then check the board's actual regulator before believing the flip.
#define REAR_TOF_IO_2V8             1

// ---- Range status ----------------------------------------------------------
// The VL53L0X publishes a 4-bit range status in bits 3..6 of register 0x14.
// ST's API defines 11 as "range valid"; the other codes report specific
// failures (signal too weak, sigma too high, phase inconsistent, and so on).
//
// This firmware ALWAYS reads and publishes the raw code so you can see why a
// reading was poor. Whether it is also used as a validity gate is a choice:
//
// 0 (default) : validity is decided by the plausibility band
//               (REAR_TOF_MIN/MAX_VALID_MM), exactly as the HC-SR04 path does.
//               The status code is reported but does not veto.
// 1           : additionally REQUIRE status == 11. Stricter, and it will
//               reject more readings -- including some usable ones at long
//               range. Turn this on only after watching the codes your
//               sensors actually produce in their real mounting.
#define REAR_TOF_REQUIRE_RANGE_STATUS_VALID 0
#define REAR_TOF_RANGE_STATUS_VALID_CODE    11

// Continuous-mode inter-measurement period. The three sensors are serviced
// round-robin, one per REAR_TOF_POLL_INTERVAL_MS tick, so each individual
// sensor is revisited every 3 x that. The period below must be <= that figure
// or we would poll faster than the sensor produces data.
#define REAR_TOF_POLL_INTERVAL_MS   20
#define REAR_TOF_CONTINUOUS_PERIOD_MS 50UL

// A sensor with no good reading for this long is FAULTY, not "clear".
#define REAR_TOF_STALE_MS           600UL

// Health streaks, mirroring the HC-SR04 policy. At 3 sensors x 20 ms one
// sensor is serviced every 60 ms, so 3 is ~180 ms and 8 is ~480 ms.
#define REAR_TOF_HEALTH_DEGRADED_STREAK 3
#define REAR_TOF_HEALTH_FAULT_STREAK    8

// How many times initialisation is retried at boot before a sensor is declared
// dead. A VL53L0X occasionally needs a second attempt after a cold power rail.
#define REAR_TOF_INIT_ATTEMPTS      3

// Runtime re-initialisation. A sensor that has been faulty for this long is
// re-initialised once, in case it was a transient bus or power event. Set to 0
// to disable. This never fabricates a reading -- it only retries the hardware.
#define REAR_TOF_REINIT_AFTER_MS    5000UL

// ---- Rear safety thresholds ------------------------------------------------
//
//   *** COMMISSIONING VALUES. NOT MEASURED ON THIS ROVER. ***
//
// These mirror the front thresholds exactly in structure -- the same
// three-distance hysteresis design, just in millimetres because that is what
// the VL53L0X natively reports:
//
//        0          STOP(250)      CLEAR(400)     WARN(600)
//        |--------------|--------------|-------------|--------->
//        |  latch SET   |  dead band   | latch CLEAR |  advisory
//                       |<-hysteresis->|
//
// WHY THESE NUMBERS ARE PROVISIONAL: the correct STOP distance is the rover's
// actual reverse stopping distance at its actual reverse speed, plus margin.
// Nobody has measured that. 250 mm is a deliberately cautious starting point
// for a rover that has never reversed under guard before, NOT a result.
//
// COMMISSION BY: on blocks, drive reverse at your normal speed into a
// measured approach, record the distance travelled after the STOP latch sets,
// then set STOP to at least twice that. Only then tighten.
//
// REQUIRED ORDERING:  STOP < CLEAR <= WARN   (enforced below)
#define REAR_STOP_DISTANCE_MM       250
#define REAR_CLEAR_DISTANCE_MM      400
#define REAR_WARN_DISTANCE_MM       600

// Consecutive-sample confirmation on the filtered value. Asymmetric for the
// same reason as the front: quick to declare an obstacle, slow to release one.
#define REAR_OBSTACLE_CONFIRM       2
#define REAR_CLEAR_CONFIRM          4

// Median window per rear sensor, and the minimum good samples for validity.
#define REAR_TOF_SAMPLE_WINDOW      5
#define REAR_TOF_MIN_GOOD_SAMPLES   3

// Internal sentinel for "not a measurement". NEVER published: telemetry emits
// JSON null instead. It is 0 nowhere, deliberately -- 0 mm would read as a
// real, extremely close obstacle.
#define REAR_TOF_INVALID_MM         (-1)

// ---- Rear sensor-invalid policy --------------------------------------------
// Same question as SAFETY_REQUIRE_BOTH_SENSORS at the front, same answer.
//
// 1 (default) : reverse requires ALL THREE rear sensors trustworthy. A dead
//               sensor's cone is unknown space, and reversing into unknown
//               space is the exact failure this subsystem exists to prevent.
// 0           : DEGRADED operation -- reverse allowed while at least one rear
//               sensor is trustworthy. That is a real blind spot across the
//               other two cones. A conscious decision, hence not the default.
#define REAR_REQUIRE_ALL_SENSORS    1

// REVERSE POLICY WHILE REAR SENSING IS UNAVAILABLE
// ------------------------------------------------
// "Unavailable" now means one specific thing: TCA9548A_ADDRESS_CONFIRMED is 0,
// so the firmware cannot legitimately reach the sensors.
//
// 0 : reverse is PERMITTED but UNGUARDED. Telemetry states this plainly via
//     "reverse_guarded":false. Preserves the ability to back away from
//     whatever tripped the front safety stop.
// 1 : reverse is REFUSED entirely. Safer; the rover cannot retreat under its
//     own power.
//
// LEFT AT 0 -- the pre-existing operator decision is preserved unchanged.
// Once you confirm the TCA address this setting stops mattering, because the
// real sensors take over the gate.
#define SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED 0

// ---- Compile-time sanity on the rear thresholds ----------------------------
static_assert(REAR_STOP_DISTANCE_MM < REAR_CLEAR_DISTANCE_MM,
              "REAR_CLEAR_DISTANCE_MM must be GREATER than "
              "REAR_STOP_DISTANCE_MM or there is no hysteresis band.");
static_assert(REAR_CLEAR_DISTANCE_MM <= REAR_WARN_DISTANCE_MM,
              "REAR_WARN_DISTANCE_MM must be >= REAR_CLEAR_DISTANCE_MM.");
static_assert(REAR_STOP_DISTANCE_MM > REAR_TOF_MIN_VALID_MM,
              "REAR_STOP_DISTANCE_MM is below the sensor's minimum range, so "
              "the stop latch could never be set by a valid reading.");
static_assert(REAR_WARN_DISTANCE_MM < REAR_TOF_MAX_VALID_MM,
              "REAR_WARN_DISTANCE_MM is beyond the accepted sensor range.");
static_assert(REAR_TOF_MIN_GOOD_SAMPLES <= REAR_TOF_SAMPLE_WINDOW,
              "REAR_TOF_MIN_GOOD_SAMPLES cannot exceed REAR_TOF_SAMPLE_WINDOW.");
static_assert(REAR_TOF_CONTINUOUS_PERIOD_MS <=
              (REAR_TOF_POLL_INTERVAL_MS * REAR_TOF_SENSOR_COUNT),
              "Continuous period is longer than the round-robin revisit "
              "interval; sensors would be polled before new data exists.");


// ============================================================================
// SAFETY DISTANCES  --  THREE SEPARATE THRESHOLDS, WITH HYSTERESIS
// ============================================================================
//
// These are deliberately CONSERVATIVE starting values. Tighten them only
// after you have measured real stopping distance at your real drive speed.
// All values are uncalibrated centimetres as reported by the sensors.
//
//   WARN  (90) : advisory only. Telemetry raises "warning":true so the Pi can
//                slow down or re-plan. The ESP32 does NOT act on this.
//   STOP  (40) : hard limit. Inside this, forward motion is cut immediately.
//   CLEAR (65) : the obstacle latch only releases once readings come back out
//                past THIS distance -- which is further away than STOP.
//
// The 40 -> 65 cm gap IS the hysteresis band. Inside it, nothing changes:
// an obstacle stays an obstacle and a clear path stays clear.
//
// REQUIRED ORDERING:  STOP  <  CLEAR  <=  WARN   (enforced below)
#define SAFETY_WARN_DISTANCE_CM  90.0f
#define SAFETY_CLEAR_DISTANCE_CM 65.0f
#define SAFETY_STOP_DISTANCE_CM  40.0f

// Consecutive-sample confirmation, applied to the MEDIAN-FILTERED value, on
// top of the median filter -- not instead of it.
//
// Note the deliberate asymmetry. It takes 2 confirmations to declare an
// obstacle (fast, ~120 ms) but 4 to release one (slow, ~240 ms). Being quick
// to stop is safe; being quick to believe the path cleared is not.
#define SAFETY_OBSTACLE_CONFIRM 2
#define SAFETY_CLEAR_CONFIRM    4

// Compile-time sanity. A typo that inverted these would silently destroy the
// hysteresis, so refuse to build instead.
static_assert(SAFETY_STOP_DISTANCE_CM < SAFETY_CLEAR_DISTANCE_CM,
              "SAFETY_CLEAR_DISTANCE_CM must be GREATER than "
              "SAFETY_STOP_DISTANCE_CM or there is no hysteresis band.");
static_assert(SAFETY_CLEAR_DISTANCE_CM <= SAFETY_WARN_DISTANCE_CM,
              "SAFETY_WARN_DISTANCE_CM must be >= SAFETY_CLEAR_DISTANCE_CM.");
static_assert(SAFETY_STOP_DISTANCE_CM > US_MIN_VALID_CM,
              "SAFETY_STOP_DISTANCE_CM is below the sensor's minimum range.");
static_assert(SAFETY_WARN_DISTANCE_CM < US_MAX_VALID_CM,
              "SAFETY_WARN_DISTANCE_CM is beyond the accepted sensor range.");
static_assert(US_MIN_GOOD_SAMPLES <= US_SAMPLE_WINDOW,
              "US_MIN_GOOD_SAMPLES cannot exceed US_SAMPLE_WINDOW.");

// SENSOR-INVALID POLICY
// ---------------------
// A sensor that is faulty is NOT a sensor that is reporting "clear". Set to 1
// (the conservative default) forward motion requires BOTH front sensors to be
// trustworthy, so a dead sensor can never be mistaken for a confirmed-clear
// path in the part of the field of view it was supposed to be covering.
//
// Set to 0 to allow DEGRADED single-sensor operation instead -- partial
// coverage rather than refusing to move. That is a real reduction in safety
// and should be a conscious decision, which is why it is not the default.
#define SAFETY_REQUIRE_BOTH_SENSORS 1


// ============================================================================
// SAFETY GATE MODE
// ============================================================================
//
// COMPONENT (default): gate each wheel independently against the direction
//   that wheel is actually being driven.
//     * a wheel commanded FORWARD is zeroed when front sensing blocks
//     * a wheel commanded REVERSE is zeroed when rear sensing blocks
//     * a wheel commanded to zero is unaffected
//   This is the literal reading of "block the forward component". It means a
//   pivot (-100,+100) facing a front obstacle becomes (-100, 0) rather than
//   being refused outright, so the rover can still extract itself.
//
// WHOLE: if ANY component would be clamped, the entire command becomes (0,0).
//   Strictly more conservative. Choose this if you would rather the rover
//   refuse a mixed-sign manoeuvre than execute a partially-gated version of
//   it -- relevant because a rover pivoting on the spot still sweeps its
//   front corners through space.
#define SAFETY_GATE_MODE_COMPONENT 0
#define SAFETY_GATE_MODE_WHOLE     1

#define SAFETY_GATE_MODE           SAFETY_GATE_MODE_COMPONENT

// When a gate clamps a command, should the STORED command also be zeroed?
//   0 (default): no. The stored command stays, and the gate is re-applied on
//                every pass. If the obstacle clears while the Pi is still
//                actively sending that command, motion resumes -- which is
//                correct, because the Pi is still asking for it. A command
//                the Pi has STOPPED sending is zeroed by the watchdog within
//                COMMAND_TIMEOUT_MS regardless.
//   1          : yes. Motion can never resume without a brand-new command.
//                Note this also defeats COMPONENT gating, because the stored
//                command becomes (0,0) after the first blocked pass.
#define SAFETY_CLEAR_COMMAND_ON_BLOCK 0


// ============================================================================
// COMMAND / TELEMETRY TIMING
// ============================================================================

// Failsafe. If no ACCEPTED movement command arrives within this window the
// motors are zeroed and state becomes COMMAND_TIMEOUT.
// Only an accepted DRIVE / MOVE / STOP refreshes this. Malformed lines,
// unknown commands, rejected commands, RESET and PING do NOT.
#define COMMAND_TIMEOUT_MS      2000UL

#define TELEMETRY_INTERVAL_MS   200UL
#define SERIAL_BAUD             115200
#define COMM_LINE_MAX           160

// Telemetry is assembled into one buffer and written in a single call, rather
// than through ~40 separate Serial.print() calls. Must be large enough for
// the longest possible line; overflow is detected and reported as
// {"event":"TELEMETRY_OVERFLOW"}, never silently truncated into invalid JSON.
//
// MEASURED worst case, counting every field at its maximum width with the
// debug and legacy blocks both compiled in:
//
//     main block           1410 bytes
//     legacy rear_ir block  357 bytes   (TELEMETRY_INCLUDE_LEGACY_REAR)
//     debug block           308 bytes   (TELEMETRY_INCLUDE_DEBUG)
//     ------------------------------
//     worst case           ~2100 bytes + NUL
//
// The longest single field is "last_reject", whose longest value is the
// 41-character NEEDS_ONBLOCKS_TRUE_ROVER_MUST_BE_SECURED.
//
// 2560 leaves ~450 bytes of headroom. Do NOT shrink this -- overflow is only
// detectable at runtime and it costs you the entire telemetry line.
// (The previous 1536 was sized for the 1181-byte pre-rear-ToF line.)
#define TELEMETRY_BUF_SIZE      2560

// Enlarged UART TX buffer so a full telemetry line is handed to the UART
// driver and drained by its ISR, instead of blocking the control loop while it
// clocks out at 115200 baud.
//
// THIS MUST BE LARGER THAN TELEMETRY_BUF_SIZE. If a line does not fit, the
// write blocks until the ISR has drained enough room -- up to ~180 ms with the
// control loop, and therefore the safety gate, stopped dead. The previous 2048
// was sized for the old 1181-byte line and is no longer enough.
#define SERIAL_TX_BUFFER_BYTES  3072

// Enlarged UART RX ring buffer.
//
// WHY THIS IS NOT OPTIONAL ONCE A SCANNER EXISTS. The RX ring is filled by the
// UART ISR, so a blocking operation in loop() does not stop bytes arriving --
// but it does stop them being CONSUMED, and the ring overflows silently when
// it fills. The core default is 256 bytes, which at 115200 baud (10 bits per
// byte) is only 22 ms of traffic.
//
// A full 0x08..0x77 bus scan blocks for roughly 25-45 ms. If the Pi happened
// to be streaming commands at full rate across that window, the 256-byte
// default would overflow and a command would be lost with no error anywhere.
//
// 1024 bytes is ~89 ms of headroom at 115200, which covers the scan with room
// to spare. Must precede Serial.begin().
#define SERIAL_RX_BUFFER_BYTES  1024

static_assert(SERIAL_TX_BUFFER_BYTES > TELEMETRY_BUF_SIZE,
              "SERIAL_TX_BUFFER_BYTES must exceed TELEMETRY_BUF_SIZE, or a "
              "full telemetry line blocks the control loop while it drains.");

// LINK BUDGET, and how to halve it.
// -----------------------------------
//   *** READ THIS BEFORE THE FIRST BENCH RUN. ***
//
// The rear ToF block roughly doubled the telemetry line, and the link budget
// is now genuinely tight. At 115200 baud (10 bits per byte with start/stop),
// against a TELEMETRY_INTERVAL_MS of 200 ms:
//
//   debug ON,  legacy ON   ~2075 B  ->  180 ms  ->  90 % utilisation
//   debug OFF, legacy ON   ~1767 B  ->  154 ms  ->  77 %
//   debug ON,  legacy OFF  ~1718 B  ->  149 ms  ->  75 %
//   debug OFF, legacy OFF  ~1410 B  ->  123 ms  ->  61 %
//
// Those are WORST cases -- a typical line, with real distances and short
// status strings, is a few hundred bytes shorter. But at 90 % worst case
// there is very little room left for command acks, and a burst of acks during
// commissioning WILL queue behind a telemetry line.
//
// THE DEFAULTS BELOW ARE DELIBERATELY LEFT AS THEY WERE. Both blocks stay on,
// and TELEMETRY_INTERVAL_MS stays at 200 ms, because dropping a published
// field or changing the telemetry cadence would be a change to the contract
// the Pi already relies on -- not something to do silently as a side effect of
// adding rear sensing. Choose one of these deliberately once you have seen the
// real traffic:
//
//   * set TELEMETRY_INCLUDE_DEBUG to 0        (once sensors are commissioned)
//   * set TELEMETRY_INCLUDE_LEGACY_REAR to 0  (once the Pi reads rear_sensor_*)
//   * raise TELEMETRY_INTERVAL_MS to 250-300
//
// Do NOT raise the baud rate: it is part of the Pi contract.
//
// TELEMETRY_INCLUDE_DEBUG drops the calibration block -- raw samples,
// good-sample counts, timeout streaks, rear raw readings and fail streaks.
// All safety-relevant fields are kept either way.
//
// If you need more headroom than that, raise TELEMETRY_INTERVAL_MS rather
// than raising the baud rate -- the baud rate is part of the Pi contract.
#define TELEMETRY_INCLUDE_DEBUG 1

// The legacy rear_ir_* fields, kept so an existing Pi-side parser sees no
// change. They are aliases of the new rear_sensor_* fields, derived from the
// same latched state. Set to 0 once the Pi reads rear_sensor_0_mm and friends
// -- that is ~357 bytes off every line.
#define TELEMETRY_INCLUDE_LEGACY_REAR 1

// Bounded duration for the MOTORTEST commissioning command.
#define MOTORTEST_DEFAULT_MS    600UL
#define MOTORTEST_MAX_MS        3000UL

// ============================================================================
// MOTORTEST PHYSICAL INTERLOCK
// ============================================================================
//
// MOTORTEST spins a wheel with the obstacle gate bypassed. On a rover with its
// wheels on the ground that is a machine driving itself across the room.
//
// With this set to 1 (the default) MOTORTEST is REFUSED unless the command
// carries an explicit acknowledgement that the rover is secured:
//
//     {"cmd":"MOTORTEST","motor":0,"power":120,"ms":600,"onblocks":true}
//
// This is not security -- anyone can type "onblocks":true. It is an interlock
// against the far more likely failure, which is a half-remembered command
// pasted from a notebook while the rover sits on the bench floor.
//
// Set to 0 only if you have some other guarantee the wheels are off the ground.
#define MOTORTEST_REQUIRE_ONBLOCKS 1

// ============================================================================
// DIAGNOSTIC / COMMISSIONING COMMANDS
// ============================================================================
//
// These exist to answer the hardware questions this firmware refuses to guess
// at. All of them are READ-ONLY with respect to motion: none of them can turn
// a wheel. See comm.cpp for the full command list.
//
// Set to 0 to compile them out entirely once the rover is commissioned.
#define DIAGNOSTICS_ENABLED     1

// I2C scan bounds. 0x08..0x77 is the full 7-bit addressable range excluding
// the reserved low and high blocks, which must never be probed.
#define I2C_SCAN_FIRST_ADDR     0x08
#define I2C_SCAN_LAST_ADDR      0x77

// How many addresses a full scan probes. Used to derive per-address timing,
// which is the single most diagnostic number a scan produces:
//
//   HEALTHY bus, no device at an address -> the master clocks out the address
//     and reads a NACK. Nine clocks at 100 kHz is ~90 us, so the whole
//     112-address sweep finishes in WELL UNDER 100 ms.
//
//   STUCK bus (a line held low) -> no transaction can even start, so every
//     probe runs to the driver's timeout instead. ESP-IDF's I2C driver floors
//     that wait at a one-second "command alive" interval, which the
//     Wire.setTimeOut() value below does NOT shorten. 112 addresses x ~1 s is
//     roughly two minutes.
//
// So per-address time of ~1 ms means "empty bus" and ~1000 ms means "stuck
// bus". They are three orders of magnitude apart and cannot be confused.
#define I2C_SCAN_ADDR_COUNT     (I2C_SCAN_LAST_ADDR - I2C_SCAN_FIRST_ADDR + 1)

// Above this per-address time a scan is reporting a STUCK bus, not an empty
// one. Deliberately well below 1000 ms and well above a healthy NACK.
#define I2C_SCAN_STUCK_MS_PER_ADDR 100

// ---- Boot scan --------------------------------------------------------------
// Run the I2C scan ONCE, at the end of setup(), and print the same line the
// {"cmd":"I2CSCAN"} command prints.
//
// ONCE. Not periodically, and never from loop(). A scan blocks the control
// loop for tens of milliseconds, and a loop that repeatedly stops the safety
// gate to re-answer a question whose answer has not changed is a bug, not a
// diagnostic. If you want a fresh scan, send the command.
//
// Set to 0 to boot silently.
#define I2C_BOOT_SCAN           1

// Refuse an I2C scan while the motors are actually being driven.
//
// A scan blocks the control loop for tens of milliseconds, which means no
// HC-SR04 ping and no safety-gate re-evaluation for that window. That is fine
// on a stationary rover and is not fine on a moving one. Today the motors
// cannot move at all (see PCA9685_ADDRESS_CONFIRMED), so this never triggers
// -- it is here so that it still holds once they can.
#define I2C_SCAN_REQUIRE_STOPPED 1


// ============================================================================
// ROBOT STATE
// ============================================================================
//
// The reported state is RECOMPUTED every pass from live conditions (see
// commComputeState() in comm.cpp). It is not a latched variable, so it can
// never get stuck reporting SAFETY_STOP long after the obstacle has gone.
//
// The STICKY record of "a safety stop happened" is a SEPARATE latched boolean
// published as "safety_stop", cleared only by RESET or a new accepted
// movement command.
// ============================================================================

// ---------------------------------------------------------------------------
// SHARED SENSOR TYPES
//
// These live here, beside RobotState, because BOTH the front ultrasonic path
// (safety.cpp) and the rear time-of-flight path (rear_tof.cpp) use them, and
// neither module should have to include the other just to name a health level.
// ---------------------------------------------------------------------------

// Sensor health. A sensor that is repeatedly failing is NEVER reported as
// healthy, because "I got nothing back" is not a measurement.
// The ordering OK < DEGRADED < FAULT is relied upon by the "worst health"
// aggregates -- do not reorder.
typedef enum {
    HEALTH_OK = 0,      // full window of good samples, fresh
    HEALTH_DEGRADED,    // usable, but dropping samples
    HEALTH_FAULT        // stale / repeatedly failing / never worked
} SensorHealth;

// ---------------------------------------------------------------------------
// Outcome of the most recent attempt to read one rear VL53L0X.
//
// THE WHOLE POINT OF THIS ENUM is that these are five genuinely different
// things and the firmware must never collapse them into one. In particular,
// none of them is ever turned into a distance of 0 mm, and only TOF_VALID
// carries a number at all:
//
//   TOF_UNINITIALISED - no backend (TCA address unconfirmed), or never polled
//   TOF_INIT_FAILED   - the sensor did not come up; it is not present or dead
//   TOF_BUS_ERROR     - the mux select or the I2C transaction itself failed
//   TOF_TIMEOUT       - the sensor was addressed but produced no result in time
//   TOF_OUT_OF_RANGE  - the sensor answered, but outside the plausible band
//                       (this is the normal "nothing within range" answer --
//                        it is NOT an obstacle and NOT an error)
//   TOF_STALE         - no good reading within REAR_TOF_STALE_MS
//   TOF_VALID         - a real measurement, inside the plausible band
// ---------------------------------------------------------------------------
typedef enum {
    TOF_UNINITIALISED = 0,
    TOF_INIT_FAILED,
    TOF_BUS_ERROR,
    TOF_TIMEOUT,
    TOF_OUT_OF_RANGE,
    TOF_STALE,
    TOF_VALID
} RearTofStatus;

typedef enum {
    STATE_BOOT = 0,
    STATE_IDLE,             // booted, no command ever received, motors stopped
    STATE_STOPPED,          // commanded to zero
    STATE_MOVING_FORWARD,
    STATE_MOVING_BACKWARD,
    STATE_TURNING_LEFT,
    STATE_TURNING_RIGHT,
    STATE_ROTATING,
    STATE_COMMAND_TIMEOUT,  // failsafe tripped
    STATE_SAFETY_STOP,      // obstacle condition currently blocking motion
    STATE_SENSOR_FAULT,     // forward refused, no trustworthy front sensor
    STATE_MOTOR_TEST,       // commissioning mode
    STATE_INVALID_COMMAND,

    // No speed path exists: the PCA9685 that drives every L298N enable input
    // is unconfirmed, absent, or has errored. NO WHEEL CAN TURN in this state,
    // whatever the Pi sends. It is a hardware/configuration fact, not a
    // transient condition, and it outranks every other state except an
    // in-progress MOTORTEST.
    //
    // ADDED, not substituted -- every pre-existing state name is unchanged, so
    // a Pi-side parser that does not know this value sees a new string rather
    // than a changed one.
    STATE_DRIVE_UNAVAILABLE
} RobotState;

const char *robotStateName(RobotState s);

#endif // CONFIG_H

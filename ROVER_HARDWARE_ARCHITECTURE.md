# ROVER — HARDWARE ARCHITECTURE (DEFINITIVE)

**Status: DOCUMENTATION ONLY. No firmware file was created or modified.**
**Date of this revision: 2026-09-22**

Source of truth for this document: wiring facts confirmed directly by the person who
wired the rover. Where this document and the firmware disagree, **the firmware is
wrong and this document is right** — see Section H, which lists those disagreements
without fixing any of them.

Supersedes the hardware-fact sections of `HARDWARE_REAR_IR_IDENTIFICATION.md` (that
file remains valid as a record of what the *repository* contained before the wiring
was confirmed, and its Section 3 candidate lists are now obsolete).

**Reading rule used throughout:** anything not stated by the wirer is marked
`NOT CONFIRMED` and is left blank. Nothing here is inferred from part convention,
typical wiring, or datasheet defaults.

---

## A. DEFINITIVE CURRENT ROVER ARCHITECTURE

```
                        ┌──────────────────────────────┐
                        │      RASPBERRY PI 5          │
                        │  camera / CV / navigation    │
                        │  (NOT a safety authority)    │
                        └──────────┬───────────────────┘
                                   │ UART  115200 8N1
                    Pi TX ─────────┼──────────► GPIO3  (RX0)
                    Pi RX ◄────────┼────────── GPIO1  (TX0)
                    GND  ──────────┴────────── GND  (common, confirmed)
                                   │
                        ┌──────────▼───────────────────┐
                        │   ESP32-WROOM-32             │
                        │   motor control, sensing,    │
                        │   safety stop, telemetry     │
                        └─┬──────┬──────┬──────┬───────┘
                          │      │      │      │
        ┌─────────────────┘      │      │      └──────────────────┐
        │ 8 x digital IN         │      │ 4 x input-only          │
        │                        │      │                         │
        ▼                        │      ▼                         ▼
┌───────────────────┐            │  ┌────────────┐     ┌────────────────────┐
│  L298N  FRONT     │            │  │ ENCODERS   │     │ HC-SR04  FRONT x2  │
│  IN1 IN2 IN3 IN4  │            │  │ GPIO34,35  │     │ A: TRIG13 ECHO14   │
│  = 23  25  26  27 │            │  │ GPIO36,39  │     │ B: TRIG18 ECHO19   │
│  ENA ◄── PCA CH0  │            │  │ (4 signals,│     │ ECHO via resistive │
│  ENB ◄── PCA CH1  │            │  │  wheel map │     │ divider 5V→3V3     │
│  OUT→ 2 motors    │            │  │  NOT CONF.)│     │ (front only — there│
└───────────────────┘            │  └────────────┘     │  is no rear sonar) │
┌───────────────────┐            │                     └────────────────────┘
│  L298N  REAR      │            │
│  IN1 IN2 IN3 IN4  │            │  I2C bus:  GPIO21 SDA / GPIO22 SCL
│  = 32  33  16  17 │            │            (single shared bus)
│  ENA ◄── PCA CH2  │            ▼
│  ENB ◄── PCA CH3  │   ═══════════════════════════════════════════
│  OUT→ 2 motors    │      ║              ║                  ║
└───────────────────┘      ▼              ▼                  ▼
                     ┌──────────┐  ┌─────────────┐   ┌──────────────┐
                     │ PCA9685  │  │  TCA9548A   │   │   NEO-9M     │
                     │ 16ch PWM │  │  I2C mux    │   │   GPS  0x42  │
                     │ addr: ?  │  │  addr: ?    │   │  (I2C, not   │
                     │ CH0..CH3 │  │ CH0 CH1 CH2 │   │   UART)      │
                     │ → L298N  │  │  ▼   ▼   ▼  │   └──────────────┘
                     │   ENA/ENB│  │ IR  IR  IR  │
                     └──────────┘  │0x29 0x29 0x29│
                                   └─────────────┘

              ┌───────────────────────────────────────────────┐
              │  3 x VL53L0X — PHYSICALLY PRESENT, REAR-      │
              │  MOUNTED. I2C TOPOLOGY / ADDRESS / CHANNEL    │
              │  NOT YET CONFIRMED. Shown deliberately        │
              │  DETACHED: no connection to the bus above is  │
              │  claimed. See Section D.                      │
              └───────────────────────────────────────────────┘

        POWER (confirmed portions only — see Section E)
        BATTERY 5.0 V – 7.2 V ──┬── XO4016  5V ──► Raspberry Pi 5
                                └── LM2596S 5V ──► ESP32
        (a 2nd LM2596S exists on the rover; its input and output are NOT CONFIRMED)
        (how motors / L298N / PCA9685 / TCA9548A / sensors are fed is NOT CONFIRMED)
```

**Drive type: DIFFERENTIAL.** No steering mechanism of any kind exists. Every turn is
produced by a left/right speed difference.

---

## B. COMPLETE ESP32 GPIO TABLE

Every pin that is wired, reserved, or unavailable. "Confirmed by" = confirmed by the
wirer (W) or a platform property of ESP32-WROOM-32 (P).

### B1 — Wired and in use

| GPIO | Direction | Connects to | Function | Conf. |
|---|---|---|---|---|
| 1 | OUT (UART0 TX) | Pi RX | Serial telemetry → Pi, 115200 | W |
| 3 | IN (UART0 RX) | Pi TX | Serial commands ← Pi, 115200 | W |
| 13 | OUT | HC-SR04 **A** TRIG | Front ultrasonic A trigger | W |
| 14 | IN | HC-SR04 **A** ECHO | Front ultrasonic A echo, **via resistive divider 5 V→3.3 V** | W |
| 16 | OUT | L298N **REAR** IN3 | Rear driver, channel B direction | W |
| 17 | OUT | L298N **REAR** IN4 | Rear driver, channel B direction | W |
| 18 | OUT | HC-SR04 **B** TRIG | Front ultrasonic B trigger | W |
| 19 | IN | HC-SR04 **B** ECHO | Front ultrasonic B echo, **via resistive divider 5 V→3.3 V** | W |
| 21 | I/O open-drain | I2C **SDA** | Shared I2C bus: PCA9685 + TCA9548A + NEO-9M | W |
| 22 | OUT open-drain | I2C **SCL** | Shared I2C bus clock | W |
| 23 | OUT | L298N **FRONT** IN1 | Front driver, channel A direction | W |
| 25 | OUT | L298N **FRONT** IN2 | Front driver, channel A direction | W |
| 26 | OUT | L298N **FRONT** IN3 | Front driver, channel B direction | W |
| 27 | OUT | L298N **FRONT** IN4 | Front driver, channel B direction | W |
| 32 | OUT | L298N **REAR** IN1 | Rear driver, channel A direction | W |
| 33 | OUT | L298N **REAR** IN2 | Rear driver, channel A direction | W |
| 34 | **IN ONLY** | Encoder signal | Encoder input 1 — wheel/phase assignment NOT CONFIRMED | W |
| 35 | **IN ONLY** | Encoder signal | Encoder input 2 — wheel/phase assignment NOT CONFIRMED | W |
| 36 | **IN ONLY** | Encoder signal | Encoder input 3 — wheel/phase assignment NOT CONFIRMED | W |
| 39 | **IN ONLY** | Encoder signal | Encoder input 4 — wheel/phase assignment NOT CONFIRMED | W |

**Wired GPIO count: 20.**

### B2 — Free (nothing wired to them)

| GPIO | Free? | Caveat (platform fact) | Conf. |
|---|---|---|---|
| 0 | free | Strapping: LOW at boot = download mode. Avoid. | P |
| 2 | free | Strapping (boot mode); onboard LED on many dev boards. | P |
| 4 | free | **Cleanest free pin.** No strapping function. ADC2. | P |
| 5 | free | Strapping: must be HIGH at boot; emits PWM briefly at boot. | P |
| 12 | free | Strapping (MTDI): an external **pull-up prevents boot**. Avoid. | P |
| 15 | free | Strapping (MTDO); internal pull-up; LOW at boot silences boot log. | P |

### B3 — Not usable

| GPIO | Reason | Conf. |
|---|---|---|
| 6, 7, 8, 9, 10, 11 | SPI flash on ESP32-WROOM-32. **Never use.** | P |
| 20, 24, 28, 29, 30, 31 | Not bonded out on this module. | P |
| 37, 38 | Not bonded out on WROOM-32. | P |

### B4 — Notes that constrain future work

- **GPIO34/35/36/39 are input-only and have no internal pull-ups.** They can read
  encoders but cannot drive anything. Any encoder needing a pull-up needs an external
  resistor.
- **All four encoder inputs are ADC1 channels**, so they remain readable with WiFi
  active — relevant only if an analog signal is ever routed there.
- **No GPIO is assigned to the rear IR subsystem**, and none is needed: the rear IR
  sensors reach the ESP32 entirely over the shared I2C bus via the TCA9548A.
- **No GPIO is assigned to the VL53L0X sensors**, because their topology is unconfirmed.
- **ESP32 GPIO is not 5 V tolerant.** Every 5 V signal reaching a GPIO needs a divider
  or shifter. Only the two HC-SR04 ECHO lines are confirmed to have one.

---

## C. I2C TOPOLOGY

One bus. One pair of pins. Three devices directly on it, three more behind the mux.

```
 ESP32-WROOM-32
   GPIO21 ── SDA ──┬──────────────┬──────────────┬─────────────────────────────┐
   GPIO22 ── SCL ──┼──────────────┼──────────────┼─────────────────────────────┤
                   │              │              │                             │
      pull-ups: NOT CONFIRMED     │              │                             │
      (location/value unknown)    │              │                             │
                   │              │              │                             │
         ┌─────────▼────────┐ ┌───▼───────────┐ ┌▼─────────────┐   (nothing else
         │    PCA9685       │ │   TCA9548A    │ │   NEO-9M     │    confirmed on
         │  16-ch PWM/servo │ │  1-to-8 I2C   │ │   GPS        │    this bus)
         │                  │ │  bus switch   │ │              │
         │ ADDRESS: ???     │ │ ADDRESS: ???  │ │ ADDR: 0x42   │
         │ NOT CONFIRMED    │ │ NOT CONFIRMED │ │ CONFIRMED    │
         ├──────────────────┤ ├───────────────┤ └──────────────┘
         │ CH0 → FRONT ENA  │ │               │
         │ CH1 → FRONT ENB  │ │  RESET pin:   │
         │ CH2 → REAR  ENA  │ │  NOT CONFIRMED│
         │ CH3 → REAR  ENB  │ │  A0/A1/A2:    │
         │ CH4..CH15 unused │ │  NOT CONFIRMED│
         │ OE pin: NOT CONF.│ │               │
         │ V+ rail: NOT CONF│ │  ┌────────────┴──────────────────┐
         └──────────────────┘ └──┤ downstream channel pairs      │
                                 │ SD0/SC0, SD1/SC1, SD2/SC2 ... │
                                 └──┬──────────┬──────────┬──────┘
                                    │          │          │
                            ┌───────▼──┐ ┌─────▼────┐ ┌───▼──────┐
                            │  CH0     │ │  CH1     │ │  CH2     │
                            │ IR sensor│ │ IR sensor│ │ IR sensor│
                            │   0x29   │ │   0x29   │ │   0x29   │
                            │ CONFIRMED│ │ CONFIRMED│ │ CONFIRMED│
                            └──────────┘ └──────────┘ └──────────┘
                              CH3..CH7: nothing confirmed attached
```

### C1 — Why the mux is mandatory here

All three rear IR sensors answer to the **same address 0x29**. Three identical
addresses cannot coexist on one bus segment. The TCA9548A resolves this by connecting
exactly one downstream segment at a time, so each 0x29 device is reachable only while
its channel is selected. Any driver must therefore **select the channel, then talk,
then (optionally) deselect** — never address 0x29 with more than one channel open.

### C2 — What is NOT confirmed about the bus

| Item | Status |
|---|---|
| PCA9685 7-bit address (A0–A5 solder jumpers) | **NOT CONFIRMED — do not guess** |
| TCA9548A 7-bit address (A0/A1/A2 pins) | **NOT CONFIRMED — do not guess** |
| TCA9548A RESET pin (tied high / GPIO / floating) | **NOT CONFIRMED** |
| PCA9685 OE pin (tied low / GPIO / floating) | **NOT CONFIRMED** |
| Bus pull-up resistors: present? where? what value? | **NOT CONFIRMED** |
| Bus voltage level (3.3 V vs 5 V) at each device | **NOT CONFIRMED** |
| Whether any level shifter sits on the bus | **NOT CONFIRMED** |
| Bus speed the wiring/cable length will tolerate | **NOT CONFIRMED** |
| Contents of TCA9548A channels CH3–CH7 | **NOT CONFIRMED** |
| Whether the three VL53L0X are on this bus at all | **NOT CONFIRMED — see Section D** |
| IR sensor part number / model | **NOT CONFIRMED** |
| Which IR sensor is rear-left / rear-centre / rear-right | **NOT CONFIRMED** |

### C3 — Address-collision audit of the confirmed bus

| Address | Device(s) | Segment | Collision? |
|---|---|---|---|
| 0x42 | NEO-9M | main bus | No — sole occupant of 0x42 |
| 0x29 | IR sensor | TCA CH0 | No — isolated by mux |
| 0x29 | IR sensor | TCA CH1 | No — isolated by mux |
| 0x29 | IR sensor | TCA CH2 | No — isolated by mux |
| ??? | PCA9685 | main bus | **Cannot be audited — address unknown** |
| ??? | TCA9548A | main bus | **Cannot be audited — address unknown** |

The two unknown addresses must be confirmed before anyone can state that the main bus
is collision-free. This is a blocking item, not a formality.

---

## D. VL53L0X — PHYSICALLY PRESENT, TOPOLOGY NOT CONFIRMED

> ### THREE (3) VL53L0X SENSORS
> ### STATUS: **physically present, I2C topology / address / channel not yet confirmed**
>
> Confirmed: three VL53L0X units are physically mounted on the **rear** of the rover.
>
> Not confirmed, and **not to be assumed**:
>
> | Question | Status |
> |---|---|
> | Are they connected to the I2C bus at all? | **NOT CONFIRMED** |
> | Are they behind the TCA9548A, or direct on the main bus? | **NOT CONFIRMED** |
> | If behind the mux — which channels? | **NOT CONFIRMED** |
> | What 7-bit address does each one respond at? | **NOT CONFIRMED** |
> | Is each XSHUT pin wired to a GPIO, tied high, or floating? | **NOT CONFIRMED** |
> | Is address reassignment at boot being used? | **NOT CONFIRMED** |
> | Which unit is rear-left / rear-centre / rear-right? | **NOT CONFIRMED** |
> | What supplies them, and at what voltage? | **NOT CONFIRMED** |
>
> No VL53L0X appears anywhere in Section A or C as a connected device. That omission
> is deliberate and must stay until the wirer confirms the topology.

### D1 — Unresolved identity question (raised, not answered)

The confirmed facts contain an ambiguity that must be settled before either driver is
written:

- Three rear sensors are confirmed as **"IR sensors at 0x29"** on TCA CH0/CH1/CH2.
- Three **VL53L0X** are confirmed as physically mounted on the **rear**.
- A VL53L0X is itself an infrared (940 nm) time-of-flight sensor, and **0x29 is its
  factory address**.

This is consistent with **two mutually exclusive** readings:

| Reading | Consequence |
|---|---|
| **(i)** The "three rear IR sensors at 0x29" and the "three VL53L0X" are the **same three devices**, described twice | The rover has 3 rear rangefinders. Section C is complete. VL53L0X topology is already known (CH0/CH1/CH2). |
| **(ii)** They are **six separate devices** — 3 simple IR modules plus 3 VL53L0X | The rover has 6 rear sensors. Three more devices must be located on the bus, and their channels/addresses are entirely unknown. |

**This document does not choose between them.** It is the first question in Section G,
because every rear-sensing decision downstream — sensor count, whether readings are
millimetre distances or on/off presence, how many mux channels are occupied, what the
telemetry schema must carry — changes completely depending on the answer.

---

## E. POWER ARCHITECTURE — CONFIRMED CONNECTIONS ONLY

```
 ┌───────────────────────────┐
 │  BATTERY                  │
 │  5.0 V – 7.2 V            │   ← confirmed operating range
 │  chemistry: NOT CONFIRMED │
 │  capacity:  NOT CONFIRMED │
 │  fusing:    NOT CONFIRMED │
 └─────┬─────────────────────┘
       │
       ├──────────────► ┌──────────────────┐        ┌──────────────────────┐
       │   (confirmed)  │ XO4016  converter│───5V──►│  RASPBERRY PI 5      │
       │                │ output 5 V       │        │  current draw: N/C   │
       │                └──────────────────┘        └──────────────────────┘
       │
       ├──────────────► ┌──────────────────┐        ┌──────────────────────┐
       │   (confirmed)  │ LM2596S buck  #1 │───5V──►│  ESP32-WROOM-32      │
       │                │ output 5 V       │        │  (via 5V/VIN pin —   │
       │                └──────────────────┘        │   NOT CONFIRMED)     │
       │                                            └──────────────────────┘
       │
       └─ ??? ─────────► ┌─────────────────┐
           NOT CONFIRMED │ LM2596S buck #2 │──── ??? ───► NOT CONFIRMED
                         │ present on rover│
                         │ in/out: unknown │
                         └─────────────────┘

 NOT CONFIRMED — deliberately left blank, do not infer:
   • How the L298N motor supply (12V/VMS terminal) is fed
   • How the L298N logic supply (5V terminal / onboard regulator jumper) is fed
   • Whether the L298N onboard 5 V regulator jumper is fitted or removed
   • What powers the PCA9685 (VCC logic rail) and its V+ (servo/output rail)
   • What powers the TCA9548A
   • What powers the three rear IR sensors
   • What powers the three VL53L0X
   • What powers the two HC-SR04 (they are 5 V parts; source unconfirmed)
   • What powers the NEO-9M
   • What powers the encoders
   • Whether all grounds are common beyond the confirmed Pi↔ESP32 GND
   • Presence/rating of any fuse, main switch, e-stop, or reverse-polarity protection
   • Whether the battery feeds the motors directly or through a converter
```

### E1 — Confirmed power facts, stated plainly

| Rail | Source | Feeds | Status |
|---|---|---|---|
| Battery | — | 5.0 V – 7.2 V range | **CONFIRMED** |
| 5 V (XO4016) | Battery | Raspberry Pi 5 | **CONFIRMED** |
| 5 V (LM2596S #1) | Battery | ESP32 | **CONFIRMED** |
| LM2596S #2 | ? | ? | present, otherwise **NOT CONFIRMED** |
| Everything else | ? | ? | **NOT CONFIRMED** |

### E2 — Two electrical concerns raised by the confirmed numbers

These are observations about the confirmed facts, not changes and not assumptions.
They are listed because both are cheap to check and expensive to discover in the field.

1. **Buck dropout at the bottom of the battery range.** The LM2596S is a step-down
   regulator. It cannot produce 5 V out from 5 V in — it needs input above output by
   its dropout margin (roughly 1.5 V for an LM2596 at load, so ~6.5 V in for a stable
   5 V out). The confirmed battery range **starts at 5.0 V**. Across the lower part of
   that range the ESP32's 5 V rail will sag with the battery rather than regulate.
   Whether that matters depends on the ESP32 board's own regulator headroom, which is
   not confirmed. The same question applies to the XO4016 feeding the Pi 5, which is
   the more sensitive load. **This needs a measurement, not a guess** — see Section G.

2. **Raspberry Pi 5 current demand.** A Pi 5 can demand several amps at 5 V, and
   brownouts on a Pi 5 are silent and look like software faults. The XO4016's current
   rating and the battery's ability to supply it at 5.0 V are both unconfirmed.

Neither concern is a reason to change anything yet. Both are reasons not to write
power-dependent safety logic until Section G's power questions are answered.

---

## F. CONFIRMED vs UNKNOWN

### F1 — CONFIRMED

| # | Fact |
|---|---|
| 1 | MCU is ESP32-WROOM-32 class (GPIO16/17 are driven, so not WROVER) |
| 2 | Pi 5 ↔ ESP32 over UART: ESP32 GPIO1/TX0 → Pi RX, GPIO3/RX0 ← Pi TX, GND common |
| 3 | Two L298N dual H-bridge modules — four motor channels total |
| 4 | FRONT L298N: IN1=GPIO23, IN2=GPIO25, IN3=GPIO26, IN4=GPIO27 |
| 5 | REAR L298N: IN1=GPIO32, IN2=GPIO33, IN3=GPIO16, IN4=GPIO17 |
| 6 | FRONT L298N ENA ← PCA9685 CH0; ENB ← PCA9685 CH1 |
| 7 | REAR L298N ENA ← PCA9685 CH2; ENB ← PCA9685 CH3 |
| 8 | Drive is differential; no steering mechanism exists |
| 9 | Four encoder signals on GPIO34, 35, 36, 39 (all input-only) |
| 10 | HC-SR04 #1: TRIG=GPIO13, ECHO=GPIO14 through a voltage divider |
| 11 | HC-SR04 #2: TRIG=GPIO18, ECHO=GPIO19 through a voltage divider |
| 12 | Both HC-SR04 are front-mounted; there is no rear ultrasonic sensor |
| 13 | I2C bus is GPIO21 = SDA, GPIO22 = SCL — one shared bus |
| 14 | A PCA9685 is present on that bus |
| 15 | A TCA9548A is present on that bus |
| 16 | Three rear IR sensors exist, all at address 0x29 |
| 17 | Those three IR sensors sit behind the TCA9548A on CH0, CH1, CH2 (one each) |
| 18 | NEO-9M GPS is on the I2C bus at address **0x42** (I2C, not UART) |
| 19 | Three VL53L0X are physically mounted on the rear of the rover |
| 20 | Battery operating range is 5.0 V – 7.2 V |
| 21 | Raspberry Pi 5 is powered from an XO4016 5 V converter |
| 22 | ESP32 is powered from an LM2596S 5 V buck converter |
| 23 | Two LM2596S buck converters are physically present on the rover |

### F2 — UNKNOWN

| # | Unknown | Blocks |
|---|---|---|
| 1 | PCA9685 7-bit I2C address | PCA9685 motor PWM firmware |
| 2 | TCA9548A 7-bit I2C address | rear IR firmware, VL53L0X firmware |
| 3 | PCA9685 OE pin disposition | PCA9685 motor PWM firmware (fail-safe behaviour) |
| 4 | PCA9685 PWM frequency the ENA/ENB inputs need | PCA9685 motor PWM firmware |
| 5 | PCA9685 V+ rail and logic VCC source | power integration |
| 6 | TCA9548A RESET pin disposition | rear IR firmware (bus recovery) |
| 7 | I2C pull-up presence, location and value | every I2C driver |
| 8 | I2C bus voltage / level shifting per device | every I2C driver (**damage risk**) |
| 9 | Rear IR sensor part number and output semantics | rear IR firmware |
| 10 | Rear IR left/centre/right → CH0/CH1/CH2 mapping | rear IR firmware |
| 11 | Whether the 3 IR @0x29 and the 3 VL53L0X are the same devices | **all rear sensing** |
| 12 | VL53L0X bus attachment, channels, addresses | VL53L0X firmware |
| 13 | VL53L0X XSHUT wiring | VL53L0X firmware (address reassignment) |
| 14 | TCA9548A channels CH3–CH7 contents | rear sensing scope |
| 15 | NEO-9M I2C protocol expectation (DDC/UBX register behaviour) | NEO-9M firmware |
| 16 | NEO-9M power source and antenna/backup state | NEO-9M firmware |
| 17 | Motor side map: which channel is LEFT, which is RIGHT | motor firmware correctness |
| 18 | Motor polarity per channel (4 independent inversions) | motor firmware correctness |
| 19 | Which encoder input belongs to which wheel; A/B phase pairing | odometry |
| 20 | Encoder type, voltage, pull-up requirement | odometry (**damage risk**) |
| 21 | Front HC-SR04 A/B → left/right mapping | telemetry labelling only |
| 22 | L298N motor-supply (VMS) source | power integration |
| 23 | L298N logic-supply source and 5 V regulator jumper state | power integration |
| 24 | LM2596S #2 input and output | power integration |
| 25 | Battery chemistry, capacity, cell count, cutoff voltage | power/safety integration |
| 26 | Fuse, main switch, e-stop, reverse-polarity protection | power/safety integration |
| 27 | Common ground topology beyond Pi↔ESP32 | every subsystem |
| 28 | Battery voltage sensing: does any ADC divider exist? | power/safety integration |
| 29 | Measured rail behaviour at battery = 5.0 V | power/safety integration |

**Count: 23 confirmed facts, 29 open unknowns.**

---

## G. REMAINING QUESTIONS BEFORE IMPLEMENTATION

Grouped by the firmware each one blocks. Everything here is blocking: each item, if
guessed, produces firmware that compiles and runs while silently doing the wrong thing.

### G1 — Rear IR firmware

1. Are the "three rear IR sensors at 0x29" and the "three VL53L0X" the **same three
   physical devices**, or six separate sensors? *(Answer this first — it defines the
   subsystem.)*
2. What is the TCA9548A's 7-bit I2C address (A0/A1/A2 pin states)?
3. What is the rear IR sensor's exact part number / module marking?
4. Is the reading a **distance value** or an **on/off presence** signal?
5. Which mux channel — CH0, CH1, CH2 — is the rear-**left**, rear-**centre**, and
   rear-**right** sensor, viewed from behind the rover?
6. Is the TCA9548A RESET pin tied high, driven by a GPIO, or left floating?
7. What supplies the IR sensors, at what voltage, and is there any level shifting
   between them and the bus?

### G2 — VL53L0X firmware

8. Are the three VL53L0X electrically connected to the I2C bus at all?
9. If yes: behind the TCA9548A, or directly on the main bus?
10. If behind the mux: which channel is each one on?
11. What address does each VL53L0X respond at, as currently wired?
12. Is each XSHUT pin wired to an ESP32 GPIO (which?), tied high, or left floating?
    *(Without XSHUT control, multiple VL53L0X on one segment cannot be separated.)*
13. Which unit is rear-left / rear-centre / rear-right?
14. What supplies them and at what voltage?

### G3 — NEO-9M firmware

15. Confirm the NEO-9M is wired **only** by I2C — no UART line to any ESP32 GPIO?
16. Is the module's I2C (DDC) interface enabled in its current configuration, and is
    that configuration saved to flash/battery-backed RAM?
17. Does the Pi also need GPS data, and if so does it arrive via ESP32 telemetry or a
    separate link?
18. What supplies the NEO-9M, at what voltage, and is there a level shifter on its
    bus lines?
19. Is an antenna connected, and is the backup battery present (cold vs warm start)?

### G4 — PCA9685 motor PWM firmware

20. What is the PCA9685's 7-bit I2C address (A0–A5 solder jumper state)?
21. Is the OE pin tied low, driven by an ESP32 GPIO (which?), or left floating?
    *(OE is the only hardware path to a guaranteed all-outputs-off motor stop.)*
22. What PWM frequency should the ENA/ENB outputs run at?
23. What powers the PCA9685's logic VCC and its V+ output rail?
24. Are the L298N ENA/ENB jumpers **removed**? *(If a jumper is still fitted, the PCA
    output is shorted to the onboard 5 V rail and speed control will not work — and
    the PCA output may be damaged.)*
25. Motor side map: which of FRONT-A, FRONT-B, REAR-A, REAR-B drives which wheel?
26. Motor polarity: for each of the four channels, does IN1-high/IN2-low spin that
    wheel forward or backward?

### G5 — Complete power / safety integration

27. What feeds each L298N's motor supply terminal — battery directly, or a converter?
28. What feeds each L298N's logic supply, and is the onboard 5 V regulator jumper
    fitted or removed?
29. What is LM2596S #2's input and its output, and what does it feed?
30. Is there a single common ground across battery, both bucks, XO4016, both L298N,
    ESP32, Pi, PCA9685, TCA9548A and all sensors?
31. Is there a fuse, a main power switch, or an e-stop — and what is the fuse rating?
32. Battery chemistry, cell count, capacity, and the minimum safe cell/pack voltage?
33. Can the ESP32 measure battery voltage — is there a divider to any ADC pin? *(No
    GPIO is currently assigned to one; without it, no low-battery safety logic can be
    written.)*
34. Measured at battery = 5.0 V: what does the ESP32 5 V rail actually read, and what
    does the Pi 5 rail read? *(See Section E2.)*
35. What is the measured stall current of one motor, and the total for four?
36. Does the motor supply share the battery terminals with the Pi/ESP32 converters —
    i.e. can a motor current spike brown out the Pi or the ESP32?

---

## H. FIRMWARE ↔ HARDWARE CONFLICTS (recorded, not fixed)

The confirmed wiring contradicts the firmware in two places. **No file was changed.**
Both are recorded here so they are not rediscovered later as bugs.

### H1 — The firmware has no PCA9685 and PWMs the IN pins directly

`config.h:169-196` states: *"We PWM the driver IN pins directly (sign-magnitude drive)
rather than using a separate enable pin... No ENA/ENB pin is invented anywhere in this
firmware. The confirmed wiring lists eight IN pins and nothing else."*

That premise is now **superseded**. The wiring does have ENA/ENB, on PCA9685 CH0–CH3.

Consequences to handle when firmware work is authorised:

- All eight IN pins are currently bound to LEDC PWM channels 0–7. With real ENA/ENB
  present, the IN pins should be plain direction outputs and the speed should come
  from the PCA9685.
- All eight LEDC channels are currently consumed. Moving speed control to the PCA9685
  frees all eight.
- Present behaviour with ENA/ENB unconnected-or-jumpered is **not** a safe assumption:
  if the ENA/ENB jumpers are removed and the PCA9685 is never initialised, its outputs
  are undefined at power-on and the motors may not respond at all — or may respond in
  a way nobody has characterised. This is why G4 Q21 and Q24 are blocking.

### H2 — The firmware never starts the I2C bus

`config.h:72-80` declares GPIO21/22 but deliberately never calls `Wire.begin()`,
because at the time no I2C peripheral could be named. Four are now named: PCA9685,
TCA9548A, three IR sensors at 0x29, and the NEO-9M at 0x42. The bus must be brought
up before any of the four firmware tasks in Section G can proceed — but not before the
addresses in G1 Q2, G4 Q20 and the pull-up/level questions are answered.

### H3 — Current safe operating state is unchanged

Rear sensing is still unimplemented; `reverse_guarded` still reports `false`; reverse
is permitted but **unguarded**. **Do not reverse the rover autonomously.** Nothing in
this document changes that, and nothing should until G1 and G2 are answered.

---

## NEXT HARDWARE QUESTIONS

Only questions that cannot be answered from anything currently known. Ordered so the
earliest answers unblock the most.

**Identity — answer first, it reshapes everything below**

1. Are the three "rear IR sensors at 0x29" and the three VL53L0X the **same three
   devices**, or six separate sensors on the rover?

**Addresses — three unknowns, none guessable**

2. PCA9685 7-bit I2C address (A0–A5 jumper state)?
3. TCA9548A 7-bit I2C address (A0/A1/A2 pin state)?
4. What address does each VL53L0X currently respond at?

**VL53L0X topology**

5. Are the VL53L0X on the I2C bus at all — and behind the TCA9548A or direct?
6. If behind the mux: which channel each?
7. Is each XSHUT wired to an ESP32 GPIO (which), tied high, or floating?

**Control pins that decide fail-safe behaviour**

8. PCA9685 OE: tied low, driven by a GPIO (which), or floating?
9. TCA9548A RESET: tied high, driven by a GPIO (which), or floating?
10. Are the L298N ENA/ENB jumpers removed, or still fitted?

**Bus electrical**

11. Where are the I2C pull-ups, and what value?
12. What voltage does each I2C device run at, and is there a level shifter anywhere on
    the bus?

**Sensor mapping**

13. Rear IR: which of CH0 / CH1 / CH2 is left, centre, right (viewed from behind)?
14. Rear IR part number, and does it report distance or on/off?
15. VL53L0X: which unit is left, centre, right?

**Motors**

16. Which wheel does each of FRONT-A, FRONT-B, REAR-A, REAR-B drive?
17. For each of those four: does IN1-high / IN2-low spin it forward or backward?
18. What PWM frequency do the ENA/ENB inputs want?

**Encoders**

19. Which encoder input (GPIO34/35/36/39) belongs to which wheel, and which pairs are
    A/B phases of the same encoder?
20. Encoder output voltage, and do they need external pull-ups? *(GPIO34/35/36/39 have
    none internally.)*

**Power**

21. What feeds each L298N's motor-supply terminal?
22. What feeds each L298N's logic supply, and is the onboard 5 V regulator jumper
    fitted?
23. LM2596S #2 — input, output, and what does it power?
24. What powers the PCA9685 (VCC and V+), the TCA9548A, the IR sensors, the VL53L0X,
    the HC-SR04 pair, the NEO-9M, and the encoders?
25. Is there one common ground across every board and the battery?
26. Fuse / main switch / e-stop — present? rating?
27. Battery chemistry, cell count, capacity, and minimum safe voltage?
28. Is there a voltage divider from the battery to any ESP32 ADC pin? *(Currently none
    is wired — without one, no low-battery safety logic is possible.)*
29. Measured with the battery at 5.0 V: what do the ESP32 and Pi 5 V rails actually
    read?
30. Measured stall current of one motor, and of all four together — and can a motor
    current spike brown out the Pi or the ESP32?

**NEO-9M**

31. Is the NEO-9M wired **only** by I2C, with no UART line to any ESP32 GPIO?
32. Is its I2C (DDC) port enabled and saved in its current configuration?
33. Antenna connected? Backup battery present?

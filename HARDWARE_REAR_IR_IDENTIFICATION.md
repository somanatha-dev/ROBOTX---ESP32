# REAR IR / MULTIPLEXER — HARDWARE IDENTIFICATION REPORT

> ## ⚠️ SUPERSEDED — 2026-09-22
>
> The wiring has since been confirmed by the person who built the rover. **Read
> [ROVER_HARDWARE_ARCHITECTURE.md](ROVER_HARDWARE_ARCHITECTURE.md) instead.**
>
> Specifically obsolete in this file:
> - **Section 2** — the mux is now confirmed to be a **TCA9548A**, I2C, with the three
>   rear IR sensors on **CH0 / CH1 / CH2**, all at address **0x29**. Items 1, 2, 4, 6
>   and 11 of the unknown-list are answered.
> - **Section 3** — the candidate lists are obsolete. TCA9548A is no longer a
>   "candidate"; it is the confirmed part. Nothing else in those tables is on the rover.
> - **Section 4b** — GPIO21/22 are no longer "declared but unused" wiring: four I2C
>   peripherals are now confirmed on that bus (PCA9685, TCA9548A, three IR sensors,
>   NEO-9M GPS at 0x42).
> - **Section 5** — the select-line/GPIO-headroom analysis no longer applies. The rear
>   IR subsystem needs **zero** additional GPIOs; it reaches the ESP32 over I2C.
> - **Sections 6, 7 and the summary** — replaced by Sections G and "NEXT HARDWARE
>   QUESTIONS" of the architecture document.
>
> Still accurate here: the ESP32-WROOM-32 / core 2.0.14 platform notes, the front
> HC-SR04 details, the LEDC channel accounting, and the standing warning that reverse
> is **unguarded** until a rear driver exists.
>
> Retained as the record of what the repository contained *before* the wiring was
> confirmed.

**Status: INVESTIGATION ONLY. No firmware was written or modified.**

Scope of the search: every file in this project (`robot_controller.ino`, `config.h`,
`comm.h`, `comm.cpp`, `motor.h`, `motor.cpp`, `safety.h`, `safety.cpp`), the parent
`Arduino/` tree (`libraries/`, `sketch_jun29a/`), and the surrounding `Documents/`
tree for any related sketch, doc or config file.

Search results for provenance:

| Source of information | Present? | Notes |
|---|---|---|
| Source code (rear IR driver) | **NO** | No mux code, no `Wire.h`, no `Wire.begin()`, no `analogRead()`, no I2C address literal anywhere in the project |
| Comments | **PARTIAL** | Only statements of what is *unknown*; see CONFIRMED section for the few facts |
| README / docs | **NONE** | No README, no docs folder, no datasheet, no notes file |
| Wiring / config files | **NONE** | `config.h` is the only config, and its rear IR section is explicitly a "not implemented" block |
| Previous configuration | **NONE** | No backup, no `.bak`, no commented-out old pin map, no legacy rear IR defines |
| Platform / build files | **NONE** | No `platformio.ini`, no `sketch.json`, no `boards.txt`, no `.vscode/`, no build artifacts |
| Git history | **NOT AVAILABLE** | This directory is not a git repository (no `.git/`). No history to inspect |
| Serial / debug output | **NO DATA** | Firmware *emits* `"rear_ir_backend":"UNCONFIGURED"` and `"rear_ir":["UNKNOWN",...]`; no captured logs exist in the repo |
| Pin definitions | **NONE FOR REAR IR** | No `REAR_IR_*_PIN`, no `MUX_*` pin macro exists |
| Board definitions | **PARTIAL** | Board family is stated in comments; see CONFIRMED |

Other sketches found nearby are unrelated: `Arduino/sketch_jun29a/` is a
`LiquidCrystal_I2C` "hello world" LCD demo (address `0x27`) — **that address belongs
to an LCD tutorial sketch and has nothing to do with this rover. Do not reuse it.**

---

## 1. CONFIRMED HARDWARE

Everything below is definitively established from this repository.

### Controller / platform
| Fact | Value | Evidence |
|---|---|---|
| MCU module | ESP32-WROOM-32 class (**not** WROVER — WROVER's GPIO16/17 are used by PSRAM and this firmware drives them) | `config.h:9`, `config.h:38-41`, `robot_controller.ino:5` |
| Arduino core | esp32 by Espressif, **2.0.14** (LEDC API `ledcSetup`/`ledcAttachPin`/`ledcWrite`) | `config.h:10-13`, `robot_controller.ino:6-8` |
| Serial link | UART0, **115200 baud**, GPIO1 TX / GPIO3 RX, to a Raspberry Pi | `config.h:90`, `config.h:455`, `comm.h:8` |
| Drive type | **Differential**, no steering mechanism | `config.h:21-23` |

### Rear IR subsystem — what IS confirmed
| Fact | Value | Evidence |
|---|---|---|
| Number of rear IR sensors | **THREE** | `config.h:273-274` ("PHYSICAL FACT (from the operator)"), `REAR_IR_SENSOR_COUNT 3` at `config.h:311` |
| Mounting location | Rear of the rover | `config.h:273-274` |
| Connection topology | The three sensors are connected **through a multiplexer** | `config.h:274`, `safety.cpp:18`, `robot_controller.ino:30` |
| There is **no** rear ultrasonic sensor | Rear sensing is IR only | `config.h:54-55`, `safety.h:10-11` |
| Current firmware state | `REAR_IR_BACKEND == REAR_IR_BACKEND_NONE`; all three report `IR_UNKNOWN`; `reverse_guarded:false`; reverse is permitted but **unguarded** | `config.h:307`, `config.h:347`, `safety.cpp:418-429`, `comm.cpp:753-771` |

**Important caveat on the three facts above:** the sensor count, the rear mounting and
the existence of a multiplexer are recorded in this repo as *operator-reported*
statements (`config.h:273` says so explicitly). They are not derived from any wiring
file, schematic or driver code. They are the strongest information available, but they
are a verbal report, not a measurement.

### Rear IR — firmware policy (a code decision, NOT a hardware measurement)
The firmware treats the rear sensors as **proximity/presence** devices and refuses to
publish any centimetre value for them (`safety.h:36-46`, `config.h:296-298`,
`comm.cpp:746-752`). This is a deliberate safety stance chosen because the sensor type
is unknown — **it is not evidence about what the sensors actually are.** Listed here
only so it is not mistaken for a confirmed hardware fact.

### I2C pins — confirmed as *pins*, NOT as the mux connection
`config.h:72-80` declares **GPIO21 = SDA, GPIO22 = SCL** under the heading "CONFIRMED
WIRING". What is confirmed is that these two pins are the designated I2C pins for this
build. The firmware **never calls `Wire.begin()`** and there is **no evidence anywhere
in the repo that the rear IR multiplexer is attached to them** — `config.h:75-78`
states the opposite: the mux is unidentified, so no bus was initialised for it.

---

## 2. UNKNOWN HARDWARE

Every item below is unknown and **cannot** be determined from this repository.
Source for the unknown-list itself: `config.h:276-294`, `safety.cpp:18-43`,
`safety.h:96-105`, `robot_controller.ino:29-37`.

| # | Item | Status |
|---|---|---|
| 1 | **Multiplexer part number** | UNKNOWN. No part number appears anywhere in the project |
| 2 | **Mux type** | UNKNOWN. Not established whether it is an I2C device/expander or an analog/digital select-line mux. `config.h:303-305` reserves *both* backend constants (`REAR_IR_BACKEND_I2C_MUX`, `REAR_IR_BACKEND_SELECT_MUX`) precisely because the type is undecided |
| 3 | **Mux control pins** | UNKNOWN. If select-line type: which ESP32 GPIOs drive S0/S1/S2(/S3) is unknown. If I2C type: the 7-bit address is unknown. No such macro exists in `config.h` |
| 4 | **Common signal pin (SIG / Z / COM)** | UNKNOWN. Which ESP32 GPIO reads the mux common line is not defined anywhere |
| 5 | **Enable pin (EN / E / INH)** | UNKNOWN. Whether EN is hard-tied (to GND) or driven by a GPIO is unknown — `config.h:283` flags exactly this |
| 6 | **Rear sensor channels** | UNKNOWN. Which mux channel number each of the three sensors occupies is unknown. `REAR_IR_1/2/3` (`config.h:324-326`) are positional *labels only* and carry no channel or left/centre/right meaning (`config.h:320-323`) |
| 7 | **Sensor model** | UNKNOWN. No IR sensor part number, module name or manufacturer appears anywhere in the project |
| 8 | **Sensor output type** | UNKNOWN. Digital (open-collector DO from an onboard comparator + pot) vs analog (Vout vs reflectance/distance) is unresolved — `config.h:285-287` |
| 9 | **Sensor supply voltage** | UNKNOWN. 3.3 V vs 5 V not recorded — `config.h:288` |
| 10 | **Level shifting** | UNKNOWN. Whether any level shifter or divider sits between the sensors/mux and the ESP32 is not recorded — `config.h:288`. (For contrast, the *front* ultrasonic echo dividers **are** documented, at `config.h:57-59` — so the absence of a rear note is a real gap, not an oversight in documentation style) |
| 11 | **ESP32 pins involved** | UNKNOWN. **Zero** GPIOs are currently assigned to the rear IR subsystem or its mux. No pin is reserved, claimed or commented for it |

---

## 3. POSSIBLE HARDWARE — ALL UNCONFIRMED

> **These are candidates only. NOTHING here is evidence. None of it may be implemented,
> and no value below may be copied into firmware.** Listed solely to make the questions
> in Section 6 easier to answer by eye.

### 3a. Candidates named in this repository as *examples* (still UNCONFIRMED)
`config.h:279-280` and `config.h:287` name these purely as illustrations of the two
possible architectures. **They are not a shortlist derived from the hardware.**

| Candidate | Type | Status |
|---|---|---|
| TCA9548A | I2C bus multiplexer, 8-channel | **UNCONFIRMED — example only** |
| PCA9548A | I2C bus multiplexer, 8-channel | **UNCONFIRMED — example only** |
| PCF8574 | I2C 8-bit I/O expander (not strictly a mux) | **UNCONFIRMED — example only** |
| CD74HC4067 | 16-channel analog/digital mux, 4 select lines + EN | **UNCONFIRMED — example only** |
| CD74HC4051 | 8-channel analog/digital mux, 3 select lines + EN | **UNCONFIRMED — example only** |
| Sharp GP2Y0A-series | Analog IR distance sensor | **UNCONFIRMED — example only** |

### 3b. Other commonly-seen parts in this class of build (general knowledge, NOT from this repo)
Included only so the part marking on the physical board is easier to recognise.

| Candidate | Type | Status |
|---|---|---|
| 74HC4051 / 74HC4052 / 74HC4053 | 8ch / dual-4ch / triple-2ch analog mux | **UNCONFIRMED** |
| 74HC151 / 74HC4511-class digital selectors | Digital-only mux | **UNCONFIRMED** |
| PCF8575 | I2C 16-bit I/O expander | **UNCONFIRMED** |
| MCP23017 / MCP23008 | I2C I/O expander | **UNCONFIRMED** |
| ADS1115 | I2C 4-channel ADC (often mistaken for a "mux" in hobby builds) | **UNCONFIRMED** |
| TCRT5000-based module | Digital reflectance/obstacle module, pot + comparator, typically 3–4 pins | **UNCONFIRMED** |
| FC-51 / "IR obstacle avoidance module" | Digital obstacle module, pot, 3 pins (VCC/GND/OUT) | **UNCONFIRMED** |
| E18-D80NK | Digital IR proximity switch, 3 wires, NPN open-collector | **UNCONFIRMED** |

**Architectural note (affects which questions matter most):** if the sensors turn out to
be *digital* 3-pin modules, a "multiplexer" in this build could equally well be an I/O
expander or even a passive breakout board rather than a true mux. The friend's answer to
Q1 and Q2 in Section 6 settles this and nothing in the repo can.

---

## 4. EXISTING GPIO USAGE

Complete table of every ESP32 GPIO touched, declared or reserved by this firmware.

### 4a. Actively driven / read by the firmware

| GPIO | Purpose | Mode | Defined at | Used at |
|---|---|---|---|---|
| 1 | UART0 TX → Raspberry Pi | Serial (owned by `Serial.begin`) | `config.h:90`, `comm.h:8` | `robot_controller.ino:74` |
| 3 | UART0 RX ← Raspberry Pi | Serial (owned by `Serial.begin`) | `config.h:90`, `comm.h:8` | `robot_controller.ino:74` |
| 13 | Front ultrasonic **A** TRIG | OUTPUT | `config.h:61` | `safety.cpp:539`, `safety.cpp:157-163` |
| 14 | Front ultrasonic **A** ECHO (via external resistive divider 5 V→3.3 V) | INPUT | `config.h:62` | `safety.cpp:546` |
| 16 | Motor REAR driver IN3 — channel `REAR_B_IN1` | OUTPUT + LEDC ch 6 | `config.h:50` | `motor.cpp:48-50`, `motor.cpp:126-145` |
| 17 | Motor REAR driver IN4 — channel `REAR_B_IN2` | OUTPUT + LEDC ch 7 | `config.h:51` | `motor.cpp:48-50`, `motor.cpp:126-145` |
| 18 | Front ultrasonic **B** TRIG | OUTPUT | `config.h:63` | `safety.cpp:539`, `safety.cpp:157-163` |
| 19 | Front ultrasonic **B** ECHO (via external resistive divider 5 V→3.3 V) | INPUT | `config.h:64` | `safety.cpp:546` |
| 23 | Motor FRONT driver IN1 — channel `FRONT_A_IN1` | OUTPUT + LEDC ch 0 | `config.h:43` | `motor.cpp:36-38`, `motor.cpp:126-145` |
| 25 | Motor FRONT driver IN2 — channel `FRONT_A_IN2` | OUTPUT + LEDC ch 1 | `config.h:44` | `motor.cpp:36-38`, `motor.cpp:126-145` |
| 26 | Motor FRONT driver IN3 — channel `FRONT_B_IN1` | OUTPUT + LEDC ch 2 | `config.h:45` | `motor.cpp:40-42`, `motor.cpp:126-145` |
| 27 | Motor FRONT driver IN4 — channel `FRONT_B_IN2` | OUTPUT + LEDC ch 3 | `config.h:46` | `motor.cpp:40-42`, `motor.cpp:126-145` |
| 32 | Motor REAR driver IN1 — channel `REAR_A_IN1` | OUTPUT + LEDC ch 4 | `config.h:48` | `motor.cpp:44-46`, `motor.cpp:126-145` |
| 33 | Motor REAR driver IN2 — channel `REAR_A_IN2` | OUTPUT + LEDC ch 5 | `config.h:49` | `motor.cpp:44-46`, `motor.cpp:126-145` |

PWM: all **eight** LEDC channels 0–7 (high-speed group) are consumed by the motors
(`config.h:189-196`), at 1 kHz / 8-bit (`config.h:181-184`).

### 4b. Declared in `config.h` but NEVER driven by the firmware

| GPIO | Declared as | Status | Defined at |
|---|---|---|---|
| 21 | `I2C_SDA_PIN` | Declared "confirmed wiring"; **no `Wire.begin()` is ever called**. No I2C peripheral is initialised or addressed | `config.h:79` |
| 22 | `I2C_SCL_PIN` | Same as above | `config.h:80` |
| 34 | `ENCODER_1_PIN` | Documentation only. Input-only pin. No `pinMode()`, no encoder feature implemented | `config.h:92` |
| 35 | `ENCODER_2_PIN` | Same as above | `config.h:93` |
| 36 | `ENCODER_3_PIN` | Same as above | `config.h:94` |
| 39 | `ENCODER_4_PIN` | Same as above | `config.h:95` |

Verified by search: `pinMode()` is called only in `motor.cpp:126,129` and
`safety.cpp:539,546`. No other GPIO is configured anywhere. There is no status LED, no
`Serial1`/`Serial2`, no `attachInterrupt`, no `analogRead`, no servo, no DAC.

---

## 5. CONFLICT CHECK — WHAT IS ACTUALLY FREE

**Hard-blocked (in use — must not be reused for rear IR):**
`1, 3, 13, 14, 16, 17, 18, 19, 23, 25, 26, 27, 32, 33`

**Reserved by declaration but electrically untouched (needs an owner decision):**
`21, 22, 34, 35, 36, 39`

**Not bonded out / reserved by the module itself:** GPIO6–11 are the SPI flash pins on
ESP32-WROOM-32 and must never be used. GPIO20, 24, 28–31 are not available on this
module. *(Platform fact about ESP32-WROOM-32, not a repository fact.)*

### Candidate pins for a rear IR / mux implementation

| GPIO | Free? | Caveats (ESP32 platform facts — not from this repo) |
|---|---|---|
| **4** | **YES — cleanest choice** | No strapping function, full-featured I/O. ADC2 channel (see WiFi note below) |
| **2** | Yes, with care | Strapping pin (boot mode); onboard LED on many dev boards. Must not be held HIGH by an external pull-up at boot |
| **5** | Yes, with care | Strapping pin; must be HIGH at boot. Outputs a PWM signal briefly at boot |
| **15** | Yes, with care | Strapping pin (MTDO); has an internal pull-up. Holding it LOW at boot silences the boot log |
| **12** | **Avoid if possible** | Strapping pin (MTDI) — an external **pull-up on GPIO12 prevents the board from booting** (it selects flash voltage). A poor choice for a mux select line that may idle high |
| **0** | **Avoid** | Strapping pin — held LOW at boot puts the ESP32 into download mode. Pulling it low from external hardware breaks normal boot |
| **21, 22** | Yes, if the mux is I2C | Already the designated I2C pins. **Only** appropriate if the mux is genuinely confirmed to be an I2C device. Would require adding `Wire.begin(21, 22)`, which the firmware deliberately does not do today |
| **34, 35, 36, 39** | Yes — **INPUT ONLY** | Cannot drive mux select lines. Can only *read*. No internal pull-ups (an open-collector digital sensor would need an external pull-up). All four are ADC1 channels, so any of them can read an analog mux common line even with WiFi active |

### Sizing the problem against what is available

| Scenario (UNCONFIRMED) | GPIOs needed | Fits? |
|---|---|---|
| I2C mux/expander on the existing bus | 0 new (uses 21/22) | Yes, comfortably |
| 8-channel select mux (e.g. 3 select + 1 common, EN tied) | 4 | Tight but feasible: e.g. 3 outputs from {4, 2, 5, 15} + 1 input from {34, 35, 36, 39} |
| 16-channel select mux (4 select + 1 common, EN tied) | 5 | Feasible only by using strapping pins; needs care |
| Select mux with EN also driven | +1 | Adds pressure; prefer EN hard-tied |
| Three sensors wired direct, no mux at all | 3 inputs | Trivially fits (34/35/36/39 are input-only but sufficient for reading) |

**Two additional conflict notes:**
1. **ADC2 vs WiFi.** If the sensors are *analog* and the common line lands on an ADC2
   pin (0, 2, 4, 12, 13, 14, 15, 25, 26, 27), `analogRead()` will fail whenever WiFi is
   active. This firmware does not currently use WiFi, but if analog sensing is
   confirmed, route the common line to an **ADC1** pin (32–39) instead. Of those, only
   34/35/36/39 are free here — 32 and 33 are motor pins.
2. **LEDC channels 0–7 are fully consumed** by the motors. This matters only if a
   candidate solution needed hardware PWM; a mux does not.

**Conclusion: there is enough GPIO headroom for any of the likely architectures. Pin
availability is not the blocker — hardware identification is.**

---

## 6. INFORMATION REQUIRED FROM HARDWARE OWNER

Minimum set of questions. Each is answerable by *looking at the physical robot* — no
measurement equipment, no datasheet lookup, no disassembly beyond reading a chip top.

**About the multiplexer board**

1. **What is printed on the multiplexer chip or board?** Read the text on top of the
   black chip, and any silkscreen label on the little PCB it sits on. Send the exact
   text, or a close-up photo. *(e.g. "CD74HC4067", "TCA9548A", "HW-478")*
2. **How many pins are labelled S0, S1, S2, S3 on that board — and are any labelled
   SDA and SCL?**
   - If you see **SDA/SCL** → it is an I2C device. Go to Q3.
   - If you see **S0/S1/S2(/S3)** → it is a select-line mux. Go to Q4.
3. *(I2C only)* **Are there any tiny solder jumpers or pads labelled A0, A1, A2 — and
   are any of them bridged with solder?** A photo of that corner of the board is
   enough.
4. **For every wire leaving the multiplexer board and going to the ESP32: which pad
   does it leave from, and which ESP32 pin number does it land on?**
   Please give it as a plain list, one per line, e.g.:
   ```
   S0  -> ESP32 pin labelled D4
   S1  -> ESP32 pin labelled D2
   S2  -> ESP32 pin labelled D15
   SIG -> ESP32 pin labelled D34
   EN  -> GND
   VCC -> 3V3
   GND -> GND
   ```
   The ESP32 pin numbers are printed on the dev board next to each header pin.
5. **Is the EN (or E, or INH) pad connected to GND, left unconnected, or run to an
   ESP32 pin?**
6. **Which numbered channels are the three rear IR sensors plugged into?** The channel
   pads are usually labelled C0–C15 or Y0–Y7. Please give all three, e.g. "C0, C1, C2"
   — and say which one is on the left, centre and right of the rear when facing the
   back of the robot.

**About the IR sensors themselves**

7. **How many wires come out of each rear IR sensor, and is there a small blue
   adjustment screw (a potentiometer) on the sensor board?**
   - 3 wires + a blue screw → almost certainly a digital on/off module.
   - 3 wires, no screw, larger lens assembly → likely analog.
8. **What is printed on the rear IR sensor module?** Silkscreen text or chip marking —
   or a photo of one sensor from the component side.
9. **Is the sensors' red/VCC wire going to the ESP32's 3V3 pin or its 5V/VIN pin?**
10. **Is there any extra small board between the sensors/mux and the ESP32?** Typically
    a board with pads labelled HV/LV, or a row of paired pins — that would be a level
    shifter. Yes/no, and a photo if yes.

**One optional but very valuable answer**

11. **Where did the mux and sensor parts come from?** A product page link or order
    listing from the kit/shop resolves Q1, Q7, Q8 and Q9 in one go.

---

## 7. SAFE NEXT STEP

**Do not write any rear IR or multiplexer code yet.** The information required before a
single line is written is:

**Blocking — code cannot be written correctly without these (from Section 6):**

1. **Mux part number and type** (Q1, Q2) — this selects between
   `REAR_IR_BACKEND_I2C_MUX` and `REAR_IR_BACKEND_SELECT_MUX` in `config.h:303-307`.
   Guessing here produces firmware that compiles, runs, and reports sensor states it
   never measured.
2. **The complete pin-to-pin wiring list** (Q4) — select pins S0/S1/S2(/S3), the common
   SIG/Z pin, and EN. *Or*, if I2C: confirmation that it is on GPIO21/22 plus the 7-bit
   address including the A0/A1/A2 jumper state (Q3).
3. **EN pin disposition** (Q5) — a mux with EN left floating reads garbage.
4. **Channel-to-sensor map** (Q6) — without it, a detected obstacle cannot be attributed
   to the correct sensor position.
5. **Sensor output type: digital or analog** (Q7, Q8) — this decides `digitalRead()` vs
   `analogRead()`, and if analog, forces the common line onto an **ADC1** pin (32–39).
6. **Sensor supply voltage and any level shifting** (Q9, Q10) — **this one is an
   electrical-damage risk, not just a correctness risk.** GPIO pins on the ESP32 are
   **not 5 V tolerant**. If the sensors run at 5 V with no shifter, connecting their
   output to a GPIO can damage the ESP32. Confirm this *before powering the rear
   subsystem*, not after.

**Non-blocking, can proceed in parallel:** nothing in the existing firmware needs to
change first. `safety.cpp:382-394` already documents the three-step integration path,
the `gRearIr[]` state machine and debounce constants already exist
(`config.h:331-334`), and `safetyReverseBlocked()` plus every telemetry field are
already wired to the API. **The driver is the only missing piece, and it is missing
because the hardware is unidentified — not because the software is incomplete.**

**Verification step once the answers arrive:** before trusting any driver, use the
existing telemetry stream. With the backend enabled, `rear_ir` must change from
`["UNKNOWN","UNKNOWN","UNKNOWN"]` to real per-sensor values as a hand is placed behind
**one** sensor at a time, and `reverse_guarded` must become `true`. If a sensor never
changes, the channel map is wrong — fix the map, do not loosen the debounce.

**Current safe operating state, until then:** reverse is permitted but **unguarded**
(`SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED 0`, `config.h:347`), and telemetry states
this honestly via `"reverse_guarded":false`. **Do not reverse the rover autonomously.**
If the rover must run autonomously before the rear IR is identified, the deliberate
alternative already exists: set that flag to `1` to refuse reverse entirely — that is
an operator decision, documented at `config.h:336-347`, and it is the only rear-IR
related change that is safe to make without any new hardware information.

---

## SUMMARY FOR THE HARDWARE OWNER

> Hey — I've gone through the whole rover firmware looking for anything that identifies
> the rear IR setup. Here's where things stand.
>
> **What the code already knows for certain:** there are three IR sensors at the rear,
> they go through a multiplexer, and the board is an ESP32-WROOM-32 talking to the Pi
> over serial at 115200. The front pair of HC-SR04 ultrasonics is fully working.
>
> **What's genuinely missing:** literally everything about the mux and the IR sensors.
> No part numbers, no pin assignments, no channel map, no address, nowhere in the
> project — no wiring file, no notes, no git history to dig through. The previous
> author left the rear IR driver deliberately unwritten rather than guess, which was the
> right call: a guessed pin map would give us firmware that *reports* rear obstacles
> without ever actually measuring one.
>
> **Practical consequence right now:** reverse has no obstacle protection. The rover
> will happily back into things. Telemetry admits this (`reverse_guarded: false`), so
> please don't let it reverse on its own until this is sorted.
>
> **Good news:** there are enough free ESP32 pins for whichever setup it turns out to
> be, so nothing needs rewiring. I just need to know what's actually there.
>
> **Could you look at the robot and tell me:**
>
> 1. What's printed on the multiplexer chip/board? (photo of the chip top is perfect)
> 2. Does that board have pins marked **SDA/SCL**, or **S0/S1/S2/S3**?
> 3. For each wire from the mux to the ESP32 — which pad it comes from, and which ESP32
>    pin number it goes to. (The pin numbers are printed on the ESP32 board.)
> 4. Is the **EN** pad on the mux wired to GND, or to a pin, or left alone?
> 5. Which channel numbers (C0, C1, C2… or Y0, Y1…) are the three rear sensors plugged
>    into — and which is left, centre and right?
> 6. Do the IR sensors have a little **blue adjustment screw** on them? How many wires
>    each?
> 7. What's printed on the IR sensor modules? (photo works)
> 8. Do the sensors get power from the ESP32's **3V3** pin or the **5V/VIN** pin?
> 9. Is there any small extra board between the sensors and the ESP32 (a level shifter —
>    usually has HV/LV markings)?
>
> **Photos of the rear of the robot, the mux board and one sensor would answer most of
> this faster than typing.** And if you still have the shop link or order for the mux
> and sensors, that alone covers questions 1, 6, 7 and 8.
>
> **One safety thing to confirm before anything else:** if those sensors run on 5 V and
> there's no level shifter, wiring their output straight to the ESP32 can damage it —
> ESP32 pins aren't 5 V tolerant. So question 8 and 9 are the ones I'd like answered
> first, even before the rest.
>
> Once I have these I can write the driver properly and reverse gets its safety
> protection back.

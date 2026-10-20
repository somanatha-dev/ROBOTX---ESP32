# Pi 5 ⇄ ESP32 Link Protocol — v2

Specification of the UART protocol between the Raspberry Pi 5 (high-level controller) and
the ESP32 (low-level hardware controller), as implemented in `protocol.cpp` and
`comm.cpp`. A Python reference implementation is in `tests/rover_link.py`.

Protocol v2 **replaces** v1 (bare JSON lines, `strstr` parsing, no CRC). It is not
backward compatible: see [Changes from v1](#14-changes-from-v1).

---

## 1. Physical layer

| Item | Value |
|---|---|
| ESP32 port | UART0 (`Serial`), GPIO1 = TX → Pi RX, GPIO3 = RX ← Pi TX, common GND |
| Settings | 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control |
| Byte time | 10 bits per byte → 86.8 µs per byte, 11 520 bytes/s maximum |
| ESP32 RX ring | 1024 bytes · TX ring 3072 bytes |

UART0 is also the ESP32 boot ROM's console. **The ROM prints plain text at every reset,
before the firmware runs; firmware cannot suppress it.** That text never carries a valid
CRC trailer, so the Pi discards it (see §12).

## 2. Frame format

Every message in both directions is one line:

```
FRAME   = PAYLOAD "*" CRC4 LF
PAYLOAD = one flat JSON object, printable ASCII (0x20–0x7E), starting "{" ending "}"
CRC4    = 4 hex digits (ESP32 sends upper case; either case accepted on receive)
LF      = 0x0A          (one CR, 0x0D, immediately before LF is tolerated and ignored)
```

Example (Pi → ESP32):

```
{"type":"COMMAND","seq":42,"cmd":"DRIVE","left":150,"right":150}*E0DC
```

Receive rules (ESP32; the Pi should apply the same):

| Condition | Result |
|---|---|
| Empty line (or CR only) | Ignored. Sending a lone LF is a safe way to resynchronise. |
| More than **160 bytes** before LF | The entire line is discarded, never truncated. One `ERROR FRAME_TOO_LONG` is sent when its LF arrives. |
| Any byte outside 0x20–0x7E (CR before LF excepted) | `ERROR INVALID_FRAME` |
| Does not end in `*` + 4 hex digits | `ERROR INVALID_FRAME` |
| First byte is not `{` (garbage before a frame) | `ERROR INVALID_FRAME`. The line is never searched for an embedded command. |
| CRC mismatch | `ERROR INVALID_CRC` |

Partial frames are accumulated across reads for as long as they take to arrive. Two
frames written back to back are processed in order.

## 3. CRC

| Parameter | Value |
|---|---|
| Algorithm | CRC-16/CCITT-FALSE |
| Polynomial | 0x1021, processed MSB first |
| Initial value | 0xFFFF |
| Input / output reflection | none |
| Final XOR | 0x0000 |
| Check value | CRC(`"123456789"`) = `29B1` |
| Bytes covered | The PAYLOAD exactly as transmitted, from `{` through `}` inclusive. Not the `*`, the CRC digits, CR or LF. |
| Text form | 4 hex digits, most significant nibble first (`0x0A3F` → `0A3F`) |
| On mismatch | The frame is discarded, **nothing** in it is acted on, and `ERROR INVALID_CRC` with `"seq":null` is returned. The seq inside a corrupt frame is never trusted. |

```python
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
```

## 4. Payload grammar (strict)

Payloads are a deliberately small subset of JSON:

| Element | Accepted | Rejected |
|---|---|---|
| Object | one flat object, at most **10** keys, no duplicate keys | nesting, arrays, trailing comma, anything after `}` |
| Key | `"` + 1–16 chars of `[A-Za-z0-9_]` + `"` | other characters, empty key |
| String | 0–32 printable chars, no `"` and no `\` | escapes of any kind |
| Integer | `-?(0\|[1-9][0-9]*)` | `150abc`, `1.5`, `1e3`, `01`, `-`, `0x10`, `+5` |
| Literals | `true`, `false`, `null` | anything else |
| Whitespace | space / tab between tokens | — |

Integers outside the int32 range are well-formed JSON; they are rejected later as
`OUT_OF_RANGE`, **never wrapped or clamped**.

## 5. Message types

Every frame's first key is `"type"`. The Pi dispatches on `type` alone.

| type | Direction | Purpose |
|---|---|---|
| `COMMAND` | Pi → ESP32 | the only type the ESP32 accepts |
| `ACK` | ESP32 → Pi | exactly one per command frame that passed the envelope checks (§7) |
| `ERROR` | ESP32 → Pi | a line that could not be accepted as a command at all |
| `EVENT` | ESP32 → Pi | something happened: `READY`, `COMMAND_TIMEOUT`, `MOTORTEST_DONE`, boot `I2CSCAN` |
| `TELEMETRY` | ESP32 → Pi | compact operational state, every 200 ms |
| `DIAG` | ESP32 → Pi | detailed diagnostics, one rotating section every 400 ms |
| `GPS` | ESP32 → Pi | latest GPS solution and GPS link health, every 1000 ms (§11) |

## 6. Commands (Pi → ESP32)

Envelope, required on every command:

| Key | Type | Rule |
|---|---|---|
| `type` | string | must be `"COMMAND"` |
| `seq` | integer | 1 … 65535 (§8) |
| `cmd` | string | command name, case-sensitive |

Command fields. **All listed fields are required unless marked optional. Any other key is
rejected with `UNKNOWN_FIELD`.** Ranges are inclusive, and values outside them are rejected.

| cmd | Fields | Effect | Refreshes watchdog |
|---|---|---|---|
| `DRIVE` | `left` int −255…255, `right` int −255…255 | differential drive request | **yes** (result ACCEPTED or GATED) |
| `STOP` | — | zero both sides, hold stopped | **yes** |
| `MOVE` | `dir` `"F"`/`"B"`/`"L"`/`"R"`/`"S"`; `speed` int 0…255 (optional only for `"S"`) | F=(s,s) B=(−s,−s) L=(55%·s,s) R=(s,55%·s) S=STOP | **yes**, except an invalid `dir` |
| `RESET` | — | clear the safety-stop and timeout latches; no motion (a drive-loss latch only once the recovered drive has been reported available for `ACK_SETTLE_MS`, §9) | no |
| `PING` | — | liveness; ACK carries `state`, `uptime_ms`, `proto` | no |
| `MOTORTEST` | `motor` 0…3, `power` −255…255, `ms` 1…3000, `onblocks` bool (must be `true`) | spin ONE raw channel, safety gate bypassed. **Rover on blocks.** | no |
| `I2CSCAN` | — | scan 0x08–0x77; refused while the wheels are driven | no |
| `I2CSTATUS` | — | SDA/SCL pad levels, no bus traffic | no |
| `TCATEST` | `addr` int 8…119 (optional; the configured address once it is confirmed) | TCA9548A read-back test | no |
| `PCATEST` | `addr` int 8…119 (optional, as above) | read-only PCA9685 probe | no |
| `TOFTEST` | `sensor` int 0…2 (optional; all three if absent) | one blocking VL53L0X read each, plus a full library `init()` of a sensor that is not initialised (seconds on a faulty one). **Refused** `REFUSED_MOTION_CONTROL_ACTIVE` while a MOTORTEST runs, a wheel is commanded or driven, or a movement command (STOP included) arrived within the 2 s watchdog window; send it once the Pi has stopped commanding and `command_timeout` is set | no |
| `HWREPORT` | — | configuration and the list of what is unverified | no |

Diagnostic results are carried **inside the ACK**, so every command still gets exactly one
response.

## 7. Validation order and responses

Checks run in this order. The first failure decides the response, so a frame with several
faults always yields the same reason.

| # | Check | Failure response |
|---|---|---|
| 1 | Line length ≤ 160 bytes | `ERROR FRAME_TOO_LONG`, seq null |
| 2 | Printable ASCII, `*XXXX` trailer, leading `{` | `ERROR INVALID_FRAME`, seq null |
| 3 | CRC | `ERROR INVALID_CRC`, seq null |
| 4 | Grammar (§4) | `ERROR INVALID_MESSAGE` / `MALFORMED_NUMBER` / `DUPLICATE_FIELD` / `TOO_MANY_FIELDS` / `FIELD_TOO_LONG`, seq null |
| 5 | `seq` present, integer, 1…65535 | `ERROR MISSING_FIELD` / `WRONG_TYPE` / `INVALID_SEQUENCE` (field `seq`), seq null |
| 6 | `type` present, string, `"COMMAND"` | `ERROR MISSING_FIELD` / `WRONG_TYPE` / `INVALID_MESSAGE_TYPE` (field `type`), **seq echoed** |
| 7 | `cmd` present, string | `ERROR MISSING_FIELD` / `WRONG_TYPE` (field `cmd`), **seq echoed** |
| 8 | Sequence order (§8) | `ACK DUPLICATE` or `ACK REJECTED STALE_SEQ`, not executed |
| 9 | Command known | `ACK REJECTED UNKNOWN_COMMAND` |
| 10 | Unknown keys, then required fields **in the order listed in §6** | `ACK REJECTED UNKNOWN_FIELD` / `MISSING_FIELD` / `WRONG_TYPE` / `OUT_OF_RANGE` / `INVALID_ARGUMENT`, with `field` |
| 11 | Command-specific preconditions | `ACK REJECTED <reason>` (e.g. `MOTOR_PWM_UNAVAILABLE`, `ONBLOCKS_REQUIRED`, `I2C_NOT_READY`, `TCA_ADDRESS_UNCONFIRMED`) |
| 12 | Executed | `ACK ACCEPTED`, or `ACK GATED` when the safety or availability gate limited the output |

**The response rule for the Pi:** after sending seq N, expect exactly one `ACK` or `ERROR`
carrying `"seq":N`. If it never arrives, the frame was corrupted or garbled beyond the
point where seq could be trusted (the ESP32 then sends an `ERROR` with `"seq":null`).

ERROR frames with `"seq":null` are rate-limited to **10 per second**, so line noise cannot
flood the TX link. Suppressed errors are counted in `DIAG SYSTEM link.errors_suppressed`.

## 8. Sequence numbers

| Rule | Value |
|---|---|
| Range | 1 … 65535. **0 is invalid** (`INVALID_SEQUENCE`). |
| Pi behaviour | increment by 1 per command; after 65535 comes 1 |
| Order | `steps = (seq − last_seq) mod 65535`, where `last_seq` is the seq of the most recent frame that received an `ACK` |
| `steps == 0` | **duplicate**: `ACK result DUPLICATE`, `reason DUPLICATE_SEQ`, plus `original_cmd` / `original_result`. Not executed. |
| `1 ≤ steps ≤ 32767` | new: processed normally |
| `steps > 32767` | **stale** (an older frame replayed): `ACK result REJECTED`, `reason STALE_SEQ`, plus `last_seq`. Not executed. |
| What updates `last_seq` | only frames answered by an `ACK` other than DUPLICATE or STALE, whether ACCEPTED, GATED or REJECTED. ERROR frames never update it. |
| After ESP32 reboot | `last_seq` is forgotten (`TELEMETRY "last_seq":null`), so the next seq is always accepted. The Pi detects the reboot from `EVENT READY` or `uptime_ms` going backwards. |
| After Pi restart | read `last_seq` from any `TELEMETRY` frame and continue from `last_seq + 1`. On `STALE_SEQ`, set seq to the ACK's `last_seq + 1`. |

A seq identifies **one frame's content**. A corrected command needs a new seq; reusing the
seq of a REJECTED command returns DUPLICATE.

Duplicates and stale frames are never executed. They do not refresh the watchdog, and
they neither restart nor cancel a running `MOTORTEST`. For DRIVE/STOP a retransmission is
therefore idempotent: the original already took effect.

## 9. ACK

```
{"type":"ACK","seq":42,"cmd":"DRIVE","result":"GATED","reason":"MOTOR_PWM_UNAVAILABLE",
 "req_left":150,"req_right":150,"gated_left":0,"gated_right":0,
 "applied_left":0,"applied_right":0}*XXXX
```

| Key | Always | Meaning |
|---|---|---|
| `seq`, `cmd` | yes | echoed from the command |
| `result` | yes | `ACCEPTED` · `GATED` (valid and executed, but output limited by safety/availability) · `REJECTED` · `DUPLICATE` |
| `reason` | yes | `NONE` when ACCEPTED, otherwise the precise cause |
| `field` | on field errors | the offending key |

For DRIVE, MOVE and STOP the ACK also carries three pairs of values:

| Pair | Meaning |
|---|---|
| `req_left/right` | what the command asked for |
| `gated_left/right` | what the safety and availability gates permitted |
| `applied_left/right` | what the motor layer **actually outputs**: the gated value after clamp and the ±40 deadband, and 0 when there is no verified PCA9685 |

Since out-of-range values are rejected, `applied` can only differ from `gated` through the
deadband (|v| < 40 → 0) or through drive being unavailable.

GATED reasons: `MOTOR_PWM_UNAVAILABLE`, `DRIVE_LOSS_UNACKNOWLEDGED`, `FRONT_OBSTACLE`,
`FRONT_SENSOR_FAULT`, `REAR_OBSTACLE`, `REAR_SENSOR_FAULT`, `REAR_UNCONFIGURED`.

`MOTOR_PWM_UNAVAILABLE`: no ready PCA9685. The command is **discarded** (TELEMETRY
`left_cmd`/`right_cmd` read 0), whatever `SAFETY_CLEAR_COMMAND_ON_BLOCK` says. The firmware
retries the PCA9685 bring-up at runtime (DIAG SYSTEM `pca_recovery`), and a drive that comes
back starts at 0/0: motion needs a new command sent after `motor_drive_available` is true
again. A running MOTORTEST is cancelled (`MOTORTEST_DONE` `CANCELLED`) when the drive is
lost.

A **loop-stall guard trip** (§12a) is also a drive loss, handled exactly like the one below.

When a PCA9685 enable write fails, the motor layer drives **all eight** L298N direction
inputs LOW before it returns, including channels it had already written in that same call;
no further write is attempted. If any direction input was driven at the instant of the
failure, or the failing call was writing any non-zero speed (a DRIVE or MOTORTEST that
failed on its very first write, from rest, included), that is a **drive loss**: `safety_stop`
is latched, as for any other stop the controller makes on its own. A failure while only
stopping, with nothing driven, is not. The loss is recorded at the failure itself and
latched before the next command, runtime update or TELEMETRY frame, so a DRIVE, STOP, PING
or RESET arriving while the write was failing or hung cannot erase it.

A drive loss must be **acknowledged**. PCA9685 recovery never clears it. Until it is
acknowledged, `safety_stop` stays `true` (also after recovery), DRIVE and MOVE are `GATED`
`DRIVE_LOSS_UNACKNOWLEDGED` (discarded, `left_cmd` 0) and MOTORTEST is `REJECTED`
`DRIVE_LOSS_UNACKNOWLEDGED`. Only a RESET acknowledges it, and only after a **settle
period**: the first TELEMETRY frame with `motor_drive_available:true` after the outage starts
a clock; once the drive has stayed available — and reported available — for
`ACK_SETTLE_MS` (`config.h`, 1000 ms), the next such frame opens the acknowledgement window.
A RESET acknowledges only if the first byte of its line reached the ESP32's UART receive
buffer **after** that window-opening frame was queued, and the drive is still available.
Any outage, or a new drive loss, closes the window and restarts the clock. Any other RESET
is still `ACCEPTED` — it stops the motors and clears `command_timeout` as always — but leaves
`safety_stop` `true`; TELEMETRY, not the ACK, shows whether the controller is clear. So a
RESET sent while the drive was down, or within about a second of it coming back (including
one that arrives during a recovery attempt and is processed just after it), acknowledges
nothing: the operator RESETs again once TELEMETRY has shown the drive back for a second.
Motion then still needs a new DRIVE.

Why the settle period: the ESP32 sees only when a frame is *queued* and when a byte reaches
its UART receive buffer. The Pi has a report ~50–130 ms after it is queued, and the bytes of
a frame still arriving can sit in the UART hardware FIFO, which `Serial.available()` does
not count, for the frame's whole length (core 2.0.14 at 115200: FIFO-full threshold 120
bytes, RX timeout 2 symbols; ~5–10 ms). A RESET sent before the Pi saw the recovery reaches
the receive buffer well before the window opens and is excluded. **Assumption, not proven:**
the Pi's delay from receiving a report to its RESET reaching the ESP32 is under about
`ACK_SETTLE_MS` minus 200 ms.

These are host-tested orderings (the host simulator models the hardware FIFO and the report
queue); none of it has been validated on the rover. How long a hung I2C write takes on the
rover (≥ 1 s on core 2.0.14) is unchanged.

`REAR_UNCONFIGURED`: rear sensing never came up this boot (TCA9548A address unconfirmed, the
TCA9548A did not answer, or no rear sensor initialised). Any reverse wheel is then refused
(`config.h` `SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED 1`); before this firmware revision it
was permitted unguarded.

## 10. ERROR and EVENT

```
{"type":"ERROR","seq":null,"reason":"INVALID_CRC"}*XXXX
{"type":"ERROR","seq":17,"reason":"INVALID_MESSAGE_TYPE","field":"type"}*XXXX
{"type":"ERROR","seq":null,"reason":"TX_FRAME_OVERFLOW","uptime_ms":1234}*XXXX
```

`TX_FRAME_OVERFLOW` means an outgoing frame did not fit the 2560-byte buffer. It carries the
seq when the lost frame was an ACK, and replaces that frame; a truncated frame is never sent.

| EVENT | When | Extra keys |
|---|---|---|
| `READY` | once, at the end of `setup()` | `proto`, `fw` (always `"rover-esp32"`, the firmware family), `fw_build`, `fw_build_utc`, `core`, `seq_min`, `seq_max`, `line_max`, hardware status |

`fw_build` identifies the build: the 12-hex-digit source revision, with `-dirty` when the
tree did not match it, written by `tools/gen_fw_build_id.py` before the build (or passed as
`-DFW_BUILD_ID`); `"unversioned"` for a build nobody stamped, `"nogit"` when the generator
found no git revision. `fw_build_utc` is when the stamp was made (`"unknown"` when unstamped).
Both also appear in DIAG SYSTEM, so the build can be read at any time, not only at boot.
| `I2CSCAN` | once at boot (`trigger":"BOOT"`) | scan result |
| `COMMAND_TIMEOUT` | watchdog trips (false → true transition; again after a RESET) | `command_age_ms`, `timeout_ms` |
| `MOTORTEST_DONE` | a test ends | `seq` of the MOTORTEST, `reason` `EXPIRED`/`CANCELLED`/`REPLACED` |

Every EVENT carries `seq` (null except `MOTORTEST_DONE`) and `uptime_ms`.

## 11. Telemetry

### TELEMETRY (fast): every 200 ms

```
{"type":"TELEMETRY","uptime_ms":1400,"state":"DRIVE_UNAVAILABLE","block_reason":"MOTOR_PWM_UNAVAILABLE",
 "last_seq":1,"last_reject":"NONE","left_cmd":0,"right_cmd":0,"left_applied":0,"right_applied":0,
 "front_left_cm":150.0,"front_right_cm":150.0,"front_valid":true,"front_obstacle":false,
 "front_warning":false,"rear_mm":[null,null,null],"rear_available":false,"rear_obstacle":false,
 "rear_sensor_fault":false,"forward_blocked":false,"reverse_blocked":false,"safety_stop":false,
 "command_age_ms":1356,"command_timeout":false,"motor_drive_available":false}*XXXX
```

| Key | Meaning |
|---|---|
| `state`, `block_reason` | recomputed state; why the gate is clamping (or `NONE`) |
| `last_seq`, `last_reject` | last ACKed seq (null after boot); last rejection reason |
| `left_cmd`/`right_cmd` | the stored request the controller is acting on: what the Pi last requested, or 0 once it was discarded (STOP, watchdog, RESET, a GATED command under `SAFETY_CLEAR_COMMAND_ON_BLOCK 1`, and always for `MOTOR_PWM_UNAVAILABLE`) |
| `left_applied`/`right_applied` | what the motor layer outputs (§9) |
| `front_*_cm` | filtered distance, one decimal, **uncalibrated**; `null` when not a valid measurement |
| `front_valid`, `front_obstacle`, `front_warning` | front sensing trustworthy / latched obstacle / advisory |
| `rear_mm` | three VL53L0X by **TCA channel** (not left/centre/right), `null` when invalid |
| `rear_available`, `rear_obstacle`, `rear_sensor_fault` | rear backend reachable / latched obstacle / policy fault |
| `forward_blocked`, `reverse_blocked`, `safety_stop` | gates; latched safety stop (set by a gate clamp, or by a drive loss; cleared only by RESET or a new fully permitted command — a drive loss only by a RESET received once the recovered drive has been reported available for `ACK_SETTLE_MS`, §9) |
| `command_age_ms`, `command_timeout` | watchdog age; latch |
| `motor_drive_available` | false means no wheel can turn, whatever is sent |

### DIAG: one section every 400 ms, 100 ms after a fast frame, rotating FRONT → REAR → SYSTEM

| Section | Keys |
|---|---|
| `FRONT` | `front_left_valid`, `front_right_valid`, `front_left_health`, `front_right_health`, `front_health`, `front_closest_cm`, `sensor_map_verified`; debug: `front_*_raw_cm`, `front_*_samples`, `front_*_timeout_streak` |
| `REAR` | `rear_backend`, `rear_orientation_verified`, `rear_positions` (physical position per index, e.g. `["CENTRE","LEFT","RIGHT"]`; `null` while unverified), arrays `rear_sensor_valid` / `_status` / `_health` / `_obstacle`, `rear_health`, `rear_closest_mm`, `rear_warning`, BUS_ERROR record per sensor: `rear_bus_err_streak` (consecutive visits ending in a VL53L0X-transaction BUS_ERROR; excludes `TCA_SELECT`; reset by any visit where the sensor answered and by demotion; reaching 50 demotes the sensor: it is no longer polled and stays failed until the next boot), `rear_bus_err_total` (every BUS_ERROR since boot, `TCA_SELECT` included), `rear_bus_err_code` (raw Wire `endTransmission()` code, 0 = none yet: for `TCA_SELECT` the mask write, for `READY_REGISTER` / `RANGE_STATUS` that register's pointer write, for `RANGE_READ` the LAST write inside the library's range read — normally the interrupt clear, not the range data itself; read-phase failures are never reported, and a failed 16-bit range data read surfaces as `TIMEOUT`), `rear_bus_err_site` (site of the latest BUS_ERROR: `NONE`/`TCA_SELECT`/`READY_REGISTER`/`RANGE_STATUS`/`RANGE_READ`), `rear_reinit_count` (runtime re-init attempts; always 0 in the shipped firmware, where `config.h` `REAR_TOF_RUNTIME_REINIT` is 0 because a re-init from `loop()` can stall it for seconds — boot init and `TOFTEST` are never counted), `rear_reinit_result` (latest such attempt: `NONE`/`OK`/`FAILED`), rear I2C backoff (§12a): `rear_i2c_backoff_ms` (ms until the next rear transaction; 0 = not backing off), `rear_i2c_backoffs` (backoffs since boot); debug: `rear_sensor_raw_mm`, `rear_fail_streak`, `rear_tca_channels` |
| `SYSTEM` | `proto`, `fw_build`, `fw_build_utc` (as in READY), `i2c_ready`, `tca_status`, `pca_status`, `tca_address_confirmed`, `pca_address_confirmed`, `motor_drive_status` (as `pca_status`, or `STALL_GUARD_NOT_RUNNING` when the loop-stall guard task could not be started, has not ticked yet, or has stopped ticking — the drive is then unavailable), loop-stall guard (§12a): `stall_guard` (`NOT_RUNNING` / `DISARMED` (during `setup()`) / `ARMED` / `TRIPPED`), `stall_guard_trips` (since boot), PCA9685 runtime recovery: `pca_recovery` (`NONE` = ready and never recovered this boot, or address unconfirmed; `RETRYING` = not ready, the full bring-up is being retried — first 1 s after the loss, then every 2, 4, 8, 16 s, then every 30 s; `RECOVERED` = ready again after at least one recovery this boot), `pca_last_fault` (cause of the most recent loss this boot: `NONE` / `BOOT_INIT` / `RUNTIME_BUS`), `pca_recovery_attempts`, `pca_recoveries`, `pca_runtime_bus_errors` (losses caused by a failed write), `pca_next_retry_ms` (0 when not retrying) — while retrying, `pca_status` is the outcome of the latest attempt (`NOT_FOUND` / `INIT_FAILED`, or `BUS_ERROR` before the first one), `motor_map_verified`, `command_ever_received`, `left_gated`, `right_gated`, loop timing (all since boot, ms of `millis()`): `loop_max_ms` (longest single `loop()` pass, excluding its trailing 1 ms yield), `loop_max_source` (slowest section of that pass), `last_stall_ms` / `last_stall_source` / `last_stall_uptime_ms` (latest pass ≥ `LOOP_STALL_THRESHOLD_MS` = 50 ms, its slowest section, and when it ended; `0`, `"NONE"`, `0` until one happens), `stall_count` (passes ≥ 50 ms); sections: `COMM_RX` (UART reading and every command handler, diagnostics included), `REAR_TOF`, `FRONT_US`, `MOTORTEST`, `MOTOR` (watchdog, gate, motor writes, PCA9685 recovery attempts), `TELEMETRY`, `GPS`, `NONE`, `link{rx_ok, rx_empty, rx_bad_frame, rx_bad_crc, rx_bad_message, rx_too_long, rx_rejected, rx_duplicates, rx_stale, errors_suppressed, tx_overflows}` |

Debug keys are controlled by `TELEMETRY_INCLUDE_DEBUG` in `config.h`.

### GPS: every 1000 ms

Sent every `GPS_FRAME_INTERVAL_MS` whatever the GPS state, so `NO_FIX` / `STALE` /
`BACKOFF` stay visible. Never in the same loop pass as a TELEMETRY or DIAG frame (it goes
out on the next pass). **Reporting only:** nothing on the ESP32 — safety, motion,
watchdog — reads GPS state. Source: the u-blox receiver at I2C 0x42, polled for UBX-NAV-PVT
about once per second by `gps.cpp`; no configuration is ever written to it.

```
{"type":"GPS","uptime_ms":6252,"gps_status":"OK","fix_type":3,"fix_ok":true,"siv":18,
 "lat_e7":128998934,"lon_e7":775193730,"alt_msl_mm":861690,"hacc_mm":1500,"vacc_mm":2500,
 "speed_mm_s":20,"head_mot_e5":34358000,"pdop_e2":120,"itow_ms":123456000,"age_ms":180,
 "polls":8,"pvt_ok":6,"poll_timeouts":2,"bus_errors":0,"ff_chunks":0,"max_service_us":9810,
 "latency_ms":40}*XXXX
```

All values are integers or `null`. About 360–440 bytes.

| Key | Meaning |
|---|---|
| `gps_status` | first that applies: `NOT_STARTED` (I2C not ready at boot, bus never touched) · `BACKOFF` (a GPS attempt failed; no GPS bus traffic for 5 s, doubling on each consecutive failure up to 300 s; a successful PVT resets it to 5 s) · `NOT_DETECTED` (no NAV-PVT ever received) · `STALE` (none for ≥ 3000 ms) · `NO_FIX` (`fix_ok` false, or `fix_type` not 2/3/4) · `OK` |
| `fix_type`, `fix_ok`, `siv` | from the latest NAV-PVT: fixType (0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time only), flags.gnssFixOK, satellites used. `null` only if no PVT was ever received — so they remain visible in `NO_FIX` and `STALE` |
| `lat_e7`, `lon_e7` | degrees × 1e-7. **`null` unless `gps_status` is `OK`** |
| `alt_msl_mm` | height above mean sea level, mm. `null` unless `OK` |
| `hacc_mm`, `vacc_mm` | horizontal / vertical accuracy estimate, mm. `null` unless `OK` |
| `speed_mm_s` | 2-D ground speed, mm/s. `null` unless `OK` |
| `head_mot_e5` | heading of motion, degrees × 1e-5. Meaningless at low speed. `null` unless `OK` |
| `pdop_e2` | position DOP × 100. `null` unless `OK` |
| `itow_ms` | GPS time of week of the latest PVT, ms. `null` if none ever |
| `age_ms` | ms since the latest PVT was received. `null` if none ever |
| `polls` | NAV-PVT poll requests sent since boot |
| `pvt_ok` | NAV-PVT solutions received since boot |
| `poll_timeouts` | polls with no PVT within 500 ms. A poll request whose own write failed also lands here: the library call used to send it cannot report a failed write |
| `bus_errors` | failed GPS attempts: an I2C transaction that failed, or a single GPS service call longer than 100 ms (a hung transaction; on this ESP32 core one can last ≥ 1 s despite the 50 ms Wire timeout). Each starts a backoff |
| `ff_chunks` | receive chunks that were entirely 0xFF (module's count said data, stream had none) |
| `max_service_us` | longest single GPS service call since boot, µs — the GPS's worst contribution to one loop pass |
| `latency_ms` | poll request → PVT parsed, latest; 0 if none |

## 12. Boot, READY and resynchronisation

1. On reset the ROM prints plain text (e.g. `rst:0x1 (POWERON_RESET),boot:0x13 …`).
   The Pi discards every line that fails the CRC check.
2. The firmware sends `EVENT READY`, then `EVENT I2CSCAN` (boot scan), then begins
   telemetry.
3. **Pi startup procedure:**
   - send a lone `\n`, which clears any partial line in the ESP32's buffer;
   - wait for any valid frame;
   - take `last_seq` from TELEMETRY and continue from `last_seq + 1`, or from 1 if it is null;
   - send `PING`.

## 12a. Loop-stall guard and rear I2C backoff

Every I2C transaction runs synchronously inside `loop()`, and on core 2.0.14 a hung one blocks
for ≥ 1 s whatever `Wire.setTimeOut()` says. For that time STOP, the command watchdog and the
safety gate all wait.

**Loop-stall guard.** A FreeRTOS task on core 0 (`loop()` runs on core 1) checks every
`STALL_GUARD_PERIOD_MS` (10 ms) that `loop()` has made progress — it refreshes a heartbeat at the
start of each pass and at each loop-timing marker. If none has happened for more than
`STALL_GUARD_T_STALL_MS` (100 ms) **and** any motor direction input is HIGH, the guard drives all
eight direction inputs LOW through the GPIO registers (no I2C), latches a drive loss (§9:
`safety_stop`, `DRIVE_LOSS_UNACKNOWLEDGED`, the settle-window RESET, then a new DRIVE) and refuses
every later direction-HIGH write until that acknowledgement — so a slow I2C write that returns
afterwards cannot restart a wheel. It is armed only once `setup()` has finished. While a hang
lasts, the cut happens within 100 + 10 ms in the host model; STOP and the watchdog themselves
still run only when the transaction returns.

**Guard liveness.** The guard task stamps every tick. `motor_drive_available` is true only while
the task exists and its latest tick is at most `STALL_GUARD_LIVENESS_MS` (50 ms) old; otherwise
`stall_guard` is `NOT_RUNNING`, `motor_drive_status` is `STALL_GUARD_NOT_RUNNING`, motion commands
are GATED/REJECTED `MOTOR_PWM_UNAVAILABLE` and no direction input can be raised. If output was live
when this is found, or a non-zero motion request is refused for it, that is a drive loss (§9), and
a guard that ticks again needs the same settle-window RESET and a new DRIVE. The check runs in
`loop()`: a guard that stops while `loop()` is also stalled is not caught by this firmware.

Limits, stated plainly: the guard **brakes** (L298N inputs both LOW, the PCA9685 enable keeps its
last duty until `loop()` can write it); it removes no bridge or motor power; it is software on the
same chip and does nothing if the ESP32 resets, crashes or browns out. During a reset the ESP32 no
longer controls the direction inputs: depending on the reset type they may float or keep their
last output level until `setup()` drives them LOW (not verified for any reset type).
**100 ms is a provisional host-test value, not a validated stop-latency requirement** — measure
`loop_max_ms` on the rover first. Host-tested only.

**Rear I2C backoff.** A failed TCA9548A select or deselect, or any rear step slower than
`REAR_I2C_SLOW_MS` (50 ms: the select, the VL53L0X read sequence, the deselect) suspends **all**
rear I2C for 1 s, doubling per further fault to 30 s; after 30 s of clean visits the next fault
starts again at 1 s. While suspended no rear transaction is attempted, every sensor's validity
still expires after `REAR_TOF_STALE_MS`, the rear reads `REAR_SENSOR_FAULT`, reverse is blocked and
no stale distance is published. Forward motion does not use the rear. A VL53L0X that fails *fast*
keeps its per-sensor bound instead (demoted after 50 failing visits). The backoff limits how often
a hung transaction can stall `loop()`; it does not shorten one.

## 13. Watchdog interaction

The 2-second movement failsafe (`COMMAND_TIMEOUT_MS`) is unchanged. It is refreshed
**only** by a DRIVE, MOVE or STOP that passed every check in §7 and was executed with result
ACCEPTED or GATED. GATED refreshes it as in v1: the Pi is alive and talking.

It is **not** refreshed by any of these:

- invalid frames or bad CRCs;
- malformed content, or a bad seq, type or cmd;
- unknown commands or rejected commands;
- duplicate or stale seqs;
- PING, RESET, MOTORTEST or any diagnostic command.

When it trips, both sides are zeroed and `EVENT COMMAND_TIMEOUT` is sent once.

A running MOTORTEST is cancelled by any received non-empty line **except** a valid new
MOTORTEST (which replaces it) and a duplicate or stale frame.

## 14. Changes from v1

| v1 | v2 |
|---|---|
| bare JSON line | `PAYLOAD*CRC4` + LF |
| no message type; telemetry identified by `"state"` | `"type"` on every frame |
| no seq | `seq` required; echoed in every ACK |
| `{"ack":"DRIVE","accepted":…}` | `{"type":"ACK","seq":…,"cmd":"DRIVE","result":…,"reason":…}` |
| DRIVE with one side missing → other side 0 | `MISSING_FIELD` |
| out-of-range → clamped to ±255 | `OUT_OF_RANGE` |
| `150abc` → 150 | `MALFORMED_NUMBER` |
| MOVE without speed → 150 | `MISSING_FIELD` (except `dir:"S"`) |
| MOTORTEST aliases `ch`/`speed`, defaults, clamping | `motor`/`power`/`ms`/`onblocks` all required, range-checked |
| TCATEST/PCATEST `addr` | integer only (e.g. `112`) |
| single ~2.1 KB telemetry line every 200 ms | TELEMETRY + rotating DIAG |
| `left_applied` = gated value | `left_applied` = actual motor output; gated value in `DIAG SYSTEM left_gated` |
| `obstacle`, `sensor_fault`, `sensor_health`, `warning`, `reverse_guarded` | `front_obstacle`, `!front_valid`, `front_health`, `front_warning`, `rear_available` |
| `rear_sensor_N_*` flat keys | `rear_mm` (fast) and `rear_sensor_*` arrays (DIAG REAR) |
| legacy `front_sensor_1_cm`, `rear_sensing`, `rear_ir*` | removed (duplicates of the fields above) |
| `{"event":"TELEMETRY_OVERFLOW"}` | `ERROR TX_FRAME_OVERFLOW` |

## 15. Bandwidth (measured)

Measured on the host simulator, which runs the real formatting code; 10 bits per byte at
115200 baud.

| Frame | Bytes | Wire time |
|---|---|---|
| TELEMETRY, typical | 565 | 49.0 ms (24.5 % of its 200 ms slot) |
| TELEMETRY, worst case (every field at maximum width) | 621 | 53.9 ms (27.0 %) |
| DIAG FRONT / REAR / SYSTEM | 391 / 735–776 / 857–901 (measured; REAR with the rear I2C backoff keys, SYSTEM with the loop-timing, build-identity, PCA9685-recovery and loop-stall guard keys) | 33.9 / 63.8–67.4 / 74.4–78.2 ms |
| DIAG REAR, worst case — **computed upper bound, not a measured frame**: every field at the widest value its type allows in the serialization code (`comm.cpp` `txDiagRear()`), the rear I2C backoff keys included | 924 | 80.2 ms |
| ACK DRIVE (−255/−255) | 195 | 16.9 ms |
| ACK PING / STOP | 136 / 174 | 11.8 / 15.1 ms |
| ERROR (seq null) | 58 | 5.0 ms |
| ACK HWREPORT (largest diagnostic) | 1417 | 123 ms |
| ACK I2CSCAN, 3 devices / 16 devices (estimate) | 679 / ~1940 | 59 / ~168 ms |
| Command DRIVE / longest command (MOTORTEST) | 70–75 / 103 | 6.1–6.5 / 8.9 ms |

Steady-state ESP32 → Pi load: fast ≈ 2825 B/s + DIAG ≈ 1530 B/s ≈ **38 %** of the link
before the loop-timing, build-identity, PCA9685-recovery and loop-stall guard keys, which grew
each DIAG SYSTEM from ~675 to 857–901 B (one every 1.2 s), and the rear I2C backoff keys, which
added 46 B to each DIAG REAR. The host simulator now measures **41.9–44.9 %** including test
command traffic (range over five full host runs; it varies run to run), against ≈ **92 %** for
the v1 telemetry line.

**ACK latency, estimated for a physical 115200 link in normal operation** (DRIVE
command, 70 B up and 195 B back):

| Component | Typical | Worst (normal operation) |
|---|---|---|
| command on the wire | 6 ms | 6.5 ms |
| wait for `commPoll()` (a front ping busy-waits up to 20 ms) | 1–2 ms | 20 ms |
| queued behind a frame already in the TX ring (fast and DIAG are never back to back) | 0 ms | 54 ms |
| ACK on the wire | 17 ms | 17 ms |
| **total** | **≈ 25 ms** | **≈ 97 ms** |

Diagnostic commands (HWREPORT, I2CSCAN) add their own frame time. These are estimates;
the physical number comes from `tests/test_link.py --port …` (it reports round-trip times).

## 16. Testing

```
python tests/run_host_tests.py              # unit tests + simulator suites (no hardware)
python tests/test_link.py --port /dev/ttyAMA0   # from the Pi 5, against the real ESP32
```

The simulator compiles the real sketch with only the Arduino core, Wire, the VL53L0X
library and the PCA9685 driver stubbed. The real PCA9685 driver and its runtime recovery
are covered separately by `tests/host/test_pca_recovery.cpp`, against a register-level fake
of the part. The link suite refuses to run against real hardware
that reports `motor_drive_available:true` unless `--allow-motion` is given.

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include "gps.h"
#include "rover_i2c.h"
#include "config.h"

// ============================================================================
//  GPS -- bounded NAV-PVT polling. See gps.h for what is and is not called.
// ============================================================================
//
// STATE MACHINE (one bus step per gpsUpdate() call at most)
//
//   IDLE    every GPS_POLL_PERIOD_MS: getPVT(0) -> one short write -> WAIT
//   WAIT    at most one receive pass per GPS_SERVICE_INTERVAL_MS
//             PVT parsed                    -> copy, flushPVT(), IDLE
//             GPS_RESPONSE_TIMEOUT_MS gone  -> poll timeout, IDLE
//   any failed transaction, or any          -> bus error, BACKOFF: no GPS
//   gpsUpdate() call > GPS_SLOW_CALL_US        bus traffic for 5 s, doubling
//                                              per consecutive failure up to
//                                              GPS_BACKOFF_MAX_MS, then IDLE.
//                                              A good PVT resets it to 5 s.
//
// RECEIVE PASS (the timing-critical part)
//   1. read the 16-bit byte count from registers 0xFD/0xFE
//   2. zero -> done
//   3. read at most GPS_MAX_BYTES_PER_PASS bytes from the stream register, in
//      GPS_I2C_CHUNK_BYTES transactions, feeding each byte to the library
//   4. stop on the first failed transaction, or on a chunk that is entirely
//      0xFF (the module's "no data" filler: the count was wrong)
//   Whatever is left of a backlog waits for the next pass. There is no loop
//   here that runs until the module is empty.
// ============================================================================

typedef enum { GPS_PHASE_IDLE, GPS_PHASE_WAIT } GpsPhase;

static SFE_UBLOX_GNSS gnss;

// Our own receive packet for process(). With requested class/ID 0/0 the
// library only writes into it in fallback paths, bounded by its packetCfg
// payload size, which begin() sets to MAX_PAYLOAD_SIZE -- hence this size.
static uint8_t   gRxPayload[MAX_PAYLOAD_SIZE];
static ubxPacket gRxPkt = { 0, 0, 0, 0, 0, gRxPayload, 0, 0,
                            SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
                            SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED };

static bool     gStarted        = false;
static GpsPhase gPhase          = GPS_PHASE_IDLE;
static bool     gEverPolled     = false;
static uint32_t gPollSentMs     = 0;
static uint32_t gLastPassMs     = 0;
static bool     gBackoff        = false;
static uint32_t gBackoffStartMs = 0;
static uint32_t gBackoffCurMs   = GPS_FAIL_BACKOFF_MS;  // length of the active backoff
static uint32_t gNextBackoffMs  = GPS_FAIL_BACKOFF_MS;  // length of the next one

static bool     gHaveFix = false;
static GpsFix   gFix;

static uint32_t gPollsSent     = 0;
static uint32_t gPvtOk         = 0;
static uint32_t gPollTimeouts  = 0;
static uint32_t gBusErrors     = 0;
static uint32_t gBytesRead     = 0;
static uint32_t gFfChunks      = 0;
static uint32_t gMaxServiceUs  = 0;
static uint32_t gLastLatencyMs = 0;


// ============================================================================
// BOUNDED RECEIVE
// ============================================================================

// One capped pass. Returns false if any I2C transaction failed.
static bool receivePass(void)
{
    // Byte count: pointer write to 0xFD, repeated start, read 0xFD and 0xFE.
    // The pointer then sits on the stream register 0xFF for the reads below.
    Wire.beginTransmission(NEO9M_I2C_ADDRESS);
    Wire.write((uint8_t)0xFD);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom((int)NEO9M_I2C_ADDRESS, 2) != 2 || Wire.available() < 2) {
        return false;
    }
    uint8_t  msb   = (uint8_t)Wire.read();
    uint8_t  lsb   = (uint8_t)Wire.read();
    uint16_t avail = (uint16_t)(((uint16_t)msb << 8) | lsb) & 0x7FFF;

    if (avail == 0) {
        return true;
    }

    // THE HARD CAP. However large the count, this pass reads no more.
    uint16_t budget = (avail < GPS_MAX_BYTES_PER_PASS) ? avail
                                                       : (uint16_t)GPS_MAX_BYTES_PER_PASS;

    while (budget > 0) {
        uint8_t n = (budget < GPS_I2C_CHUNK_BYTES) ? (uint8_t)budget
                                                   : (uint8_t)GPS_I2C_CHUNK_BYTES;
        uint8_t chunk[GPS_I2C_CHUNK_BYTES];

        if (Wire.requestFrom((int)NEO9M_I2C_ADDRESS, (int)n) != n) {
            return false;
        }
        bool allFf = true;
        for (uint8_t i = 0; i < n; i++) {
            int b = Wire.read();
            if (b < 0) {
                return false;
            }
            chunk[i] = (uint8_t)b;
            if (chunk[i] != 0xFF) {
                allFf = false;
            }
        }
        gBytesRead += n;

        if (allFf) {
            // Nothing real in the stream; stop instead of reading more filler.
            gFfChunks++;
            return true;
        }
        for (uint8_t i = 0; i < n; i++) {
            gnss.process(chunk[i], &gRxPkt, 0, 0);
        }
        budget -= n;
    }
    return true;
}

// A complete NAV-PVT has been parsed since the last flushPVT()?
static bool pvtArrived(void)
{
    return gnss.packetUBXNAVPVT != NULL &&
           gnss.packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all;
}

// Copy straight from the library's storage -- never through the getters,
// which re-poll -- then mark it consumed.
static void takePvt(uint32_t now)
{
    const UBX_NAV_PVT_data_t &d = gnss.packetUBXNAVPVT->data;

    gFix.fixOk     = d.flags.bits.gnssFixOK != 0;
    gFix.fixType   = d.fixType;
    gFix.numSV     = d.numSV;
    gFix.latE7     = d.lat;
    gFix.lonE7     = d.lon;
    gFix.hMslMm    = d.hMSL;
    gFix.hAccMm    = d.hAcc;
    gFix.vAccMm    = d.vAcc;
    gFix.gSpeedMmS = d.gSpeed;
    gFix.headMotE5 = d.headMot;
    gFix.pDopE2    = d.pDOP;
    gFix.iTowMs    = d.iTOW;
    gFix.rxMs      = now;
    gHaveFix       = true;

    gnss.flushPVT();
}

static void enterBackoff(uint32_t now)
{
    gBusErrors++;
    gBackoff        = true;
    gBackoffStartMs = now;
    gBackoffCurMs   = gNextBackoffMs;
    gNextBackoffMs  = (gNextBackoffMs >= GPS_BACKOFF_MAX_MS / 2) ? GPS_BACKOFF_MAX_MS
                                                                 : gNextBackoffMs * 2;
    gPhase          = GPS_PHASE_IDLE;
}


// ============================================================================
// STATE MACHINE
// ============================================================================

static void serviceOnce(uint32_t now)
{
    if (gBackoff) {
        if (now - gBackoffStartMs < gBackoffCurMs) {
            return;                                 // NO bus traffic
        }
        gBackoff    = false;
        gEverPolled = false;                        // poll again straight away
    }

    if (gPhase == GPS_PHASE_IDLE) {
        if (gEverPolled && now - gPollSentMs < GPS_POLL_PERIOD_MS) {
            return;
        }
        gnss.flushPVT();            // only a PVT parsed AFTER this request counts
        (void)gnss.getPVT(0);       // one poll request, no wait (see gps.h)
        gPollsSent++;
        gEverPolled = true;
        gPollSentMs = now;
        gLastPassMs = now;
        gPhase      = GPS_PHASE_WAIT;
        return;
    }

    // WAIT
    if (now - gPollSentMs >= GPS_RESPONSE_TIMEOUT_MS) {
        gPollTimeouts++;
        gPhase = GPS_PHASE_IDLE;
        return;
    }
    if (now - gLastPassMs < GPS_SERVICE_INTERVAL_MS) {
        return;
    }
    gLastPassMs = now;

    if (!receivePass()) {
        enterBackoff(now);
        return;
    }
    if (pvtArrived()) {
        takePvt(now);
        gPvtOk++;
        gNextBackoffMs = GPS_FAIL_BACKOFF_MS;       // healthy again: next backoff starts at 5 s
        gLastLatencyMs = now - gPollSentMs;
        gPhase         = GPS_PHASE_IDLE;
    }
}


// ============================================================================
// PUBLIC
// ============================================================================

void gpsInit(void)
{
    if (!roverI2cReady()) {
        gStarted = false;
        return;
    }

    // maxWait 0: nothing is waited for. The CFG-PRT poll replies (if any) are
    // drained later and ignored by the parser. Return value is meaningless at
    // maxWait 0 -- detection is "a NAV-PVT arrived".
    (void)gnss.begin(Wire, NEO9M_I2C_ADDRESS, 0);

    gStarted = true;
    gPhase   = GPS_PHASE_IDLE;
}

void gpsUpdate(void)
{
    if (!gStarted) {
        return;
    }
    bool     wasBackoff = gBackoff;
    uint32_t t0 = micros();
    serviceOnce(millis());
    uint32_t dt = micros() - t0;
    if (dt > gMaxServiceUs) {
        gMaxServiceUs = dt;
    }

    // A hung transaction can outlast I2C_TIMEOUT_MS (see config.h), and a hung
    // getPVT(0) write is otherwise invisible. Either way the attempt failed:
    // back off, timed from the END of the stall.
    if (dt > GPS_SLOW_CALL_US) {
        if (gBackoff && !wasBackoff) {
            gBackoffStartMs = millis();             // this call already failed
        } else if (!gBackoff) {
            enterBackoff(millis());
        }
    }
}

GpsStatus gpsStatus(void)
{
    if (!gStarted) {
        return GPS_NOT_STARTED;
    }
    uint32_t now = millis();
    if (gBackoff && now - gBackoffStartMs < gBackoffCurMs) {
        return GPS_BACKOFF;
    }
    if (!gHaveFix) {
        return GPS_NOT_DETECTED;
    }
    if (now - gFix.rxMs >= GPS_STALE_MS) {
        return GPS_STALE;
    }
    if (!gFix.fixOk || gFix.fixType < 2 || gFix.fixType > 4) {
        return GPS_NO_FIX;
    }
    return GPS_OK;
}

const char *gpsStatusName(GpsStatus s)
{
    switch (s) {
        case GPS_NOT_STARTED:  return "NOT_STARTED";
        case GPS_NOT_DETECTED: return "NOT_DETECTED";
        case GPS_BACKOFF:      return "BACKOFF";
        case GPS_NO_FIX:       return "NO_FIX";
        case GPS_STALE:        return "STALE";
        case GPS_OK:           return "OK";
    }
    return "UNKNOWN";
}

bool gpsLatest(GpsFix *out)
{
    if (!gHaveFix || out == NULL) {
        return false;
    }
    *out = gFix;
    return true;
}

uint32_t gpsAgeMs(void)
{
    return gHaveFix ? (millis() - gFix.rxMs) : UINT32_MAX;
}

uint32_t gpsPollsSent(void)     { return gPollsSent; }
uint32_t gpsPvtOk(void)         { return gPvtOk; }
uint32_t gpsPollTimeouts(void)  { return gPollTimeouts; }
uint32_t gpsBusErrors(void)     { return gBusErrors; }
uint32_t gpsBytesRead(void)     { return gBytesRead; }
uint32_t gpsFfChunks(void)      { return gFfChunks; }
uint32_t gpsMaxServiceUs(void)  { return gMaxServiceUs; }
uint32_t gpsLastLatencyMs(void) { return gLastLatencyMs; }


// ============================================================================
// "GPS" FRAME FIELDS
// ============================================================================
//
// fix_type / fix_ok / siv / itow_ms / age_ms describe the latest PVT and are
// null only if none was ever received -- so NO_FIX and STALE stay visible.
// The position group is null unless the status is OK: a stale or no-fix
// position is never published as if it were a measurement.
// ============================================================================

void gpsWriteFrameFields(ProtoWriter *w)
{
    GpsStatus st   = gpsStatus();
    GpsFix    f;
    bool      have = gpsLatest(&f);
    bool      ok   = (st == GPS_OK);

    protoWriterAppend(w, ",\"gps_status\":\"%s\"", gpsStatusName(st));

    if (have) {
        protoWriterAppend(w, ",\"fix_type\":%u,\"fix_ok\":%s,\"siv\":%u",
                          (unsigned)f.fixType, f.fixOk ? "true" : "false",
                          (unsigned)f.numSV);
    } else {
        protoWriterAppend(w, ",\"fix_type\":null,\"fix_ok\":null,\"siv\":null");
    }

    if (ok) {
        protoWriterAppend(w, ",\"lat_e7\":%ld,\"lon_e7\":%ld,\"alt_msl_mm\":%ld"
                             ",\"hacc_mm\":%lu,\"vacc_mm\":%lu,\"speed_mm_s\":%ld"
                             ",\"head_mot_e5\":%ld,\"pdop_e2\":%u",
                          (long)f.latE7, (long)f.lonE7, (long)f.hMslMm,
                          (unsigned long)f.hAccMm, (unsigned long)f.vAccMm,
                          (long)f.gSpeedMmS, (long)f.headMotE5, (unsigned)f.pDopE2);
    } else {
        protoWriterAppend(w, ",\"lat_e7\":null,\"lon_e7\":null,\"alt_msl_mm\":null"
                             ",\"hacc_mm\":null,\"vacc_mm\":null,\"speed_mm_s\":null"
                             ",\"head_mot_e5\":null,\"pdop_e2\":null");
    }

    if (have) {
        protoWriterAppend(w, ",\"itow_ms\":%lu,\"age_ms\":%lu",
                          (unsigned long)f.iTowMs, (unsigned long)gpsAgeMs());
    } else {
        protoWriterAppend(w, ",\"itow_ms\":null,\"age_ms\":null");
    }

    protoWriterAppend(w, ",\"polls\":%lu,\"pvt_ok\":%lu,\"poll_timeouts\":%lu"
                         ",\"bus_errors\":%lu,\"ff_chunks\":%lu"
                         ",\"max_service_us\":%lu,\"latency_ms\":%lu",
                      (unsigned long)gPollsSent, (unsigned long)gPvtOk,
                      (unsigned long)gPollTimeouts, (unsigned long)gBusErrors,
                      (unsigned long)gFfChunks, (unsigned long)gMaxServiceUs,
                      (unsigned long)gLastLatencyMs);
}

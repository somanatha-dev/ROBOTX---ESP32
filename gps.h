#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include "config.h"
#include "protocol.h"

// ============================================================================
//  GPS  --  u-blox receiver at NEO9M_I2C_ADDRESS (0x42), read over I2C
// ============================================================================
//
// REPORTING ONLY. Nothing in safety, motion, the watchdog or the command path
// reads anything from this module. A dead, absent or lying GPS degrades to a
// status string in the "GPS" frame, never to a blocked rover.
//
// REUSE. The UBX NAV-PVT decoding is the SparkFun u-blox GNSS library's
// (v2.2.29, as used by the standalone GPS.ino that produced the earlier real
// fixes). Only three library calls are used:
//
//   begin(Wire, 0x42, 0)   maxWait 0: address check + three UBX-CFG-PRT POLL
//                          requests, waits for nothing. Its return value is
//                          meaningless with maxWait 0 and is ignored.
//   getPVT(0)              sends ONE UBX-NAV-PVT poll request and returns at
//                          once. Its return value is false whether or not the
//                          write succeeded, so a failed request shows up as a
//                          poll timeout (or as a failed count read on the next
//                          pass), not as a bus error.
//   process(byte, ...)     the library's byte parser; a complete NAV-PVT lands
//                          in gnss.packetUBXNAVPVT->data
//
// WHAT IS NEVER CALLED, AND WHY
//   * Wire.begin()                  -- rover_i2c.cpp owns the bus.
//   * getPVT(n > 0), checkUblox()   -- the library's receive loop drains the
//                                      module's ENTIRE backlog in one call with
//                                      no time limit (checkUbloxI2C).
//   * getLatitude() and friends     -- each silently re-polls for up to
//                                      1100 ms once its field has been read.
//   * any set...() / save / reset   -- no configuration is written, ever.
//
// Instead, gps.cpp reads the DDC stream itself with a HARD cap of
// GPS_MAX_BYTES_PER_PASS bytes per pass and feeds each byte to process().
// ============================================================================

typedef enum {
    GPS_NOT_STARTED,    // I2C not ready at boot; the bus is never touched
    GPS_NOT_DETECTED,   // no NAV-PVT has ever been received
    GPS_BACKOFF,        // an attempt failed; no GPS traffic for 5 s, doubling up to GPS_BACKOFF_MAX_MS
    GPS_NO_FIX,         // PVT received, but gnssFixOK is false or fixType is not 2/3/4
    GPS_STALE,          // a PVT was received, but none for GPS_STALE_MS
    GPS_OK
} GpsStatus;

// One NAV-PVT solution, integer units exactly as u-blox reports them.
typedef struct {
    bool     fixOk;         // flags.gnssFixOK
    uint8_t  fixType;       // 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
    uint8_t  numSV;         // satellites used
    int32_t  latE7;         // deg * 1e-7
    int32_t  lonE7;         // deg * 1e-7
    int32_t  hMslMm;        // height above mean sea level, mm
    uint32_t hAccMm;        // horizontal accuracy estimate, mm
    uint32_t vAccMm;        // vertical accuracy estimate, mm
    int32_t  gSpeedMmS;     // ground speed, mm/s
    int32_t  headMotE5;     // heading of motion, deg * 1e-5 (meaningless when slow)
    uint16_t pDopE2;        // position DOP * 0.01
    uint32_t iTowMs;        // GPS time of week, ms
    uint32_t rxMs;          // millis() when this solution was parsed
} GpsFix;

// Call once from setup(), after roverI2cInit() and after safetyInit().
// Never calls Wire.begin().
void gpsInit(void);

// Call every pass of loop(). Bounded: at most one bus step per call (one poll
// request OR one capped receive pass), and nothing at all during backoff.
void gpsUpdate(void);

GpsStatus   gpsStatus(void);
const char *gpsStatusName(GpsStatus s);

// The most recent solution. Returns false (and leaves *out untouched) if no
// PVT has ever been received. Check gpsStatus() before trusting it.
bool gpsLatest(GpsFix *out);

// ms since the most recent PVT was parsed, or UINT32_MAX if never.
uint32_t gpsAgeMs(void);

// ---- diagnostics (since boot) ----------------------------------------------
uint32_t gpsPollsSent(void);       // NAV-PVT poll requests sent
uint32_t gpsPvtOk(void);           // NAV-PVT solutions received
uint32_t gpsPollTimeouts(void);    // polls with no PVT within GPS_RESPONSE_TIMEOUT_MS
uint32_t gpsBusErrors(void);       // failed attempts: an I2C error, or a call > GPS_SLOW_CALL_US (each starts a backoff)
uint32_t gpsBytesRead(void);       // stream bytes read from the module
uint32_t gpsFfChunks(void);        // all-0xFF chunks (count said data, stream had none)
uint32_t gpsMaxServiceUs(void);    // longest single gpsUpdate() call, micros()
uint32_t gpsLastLatencyMs(void);   // poll request -> PVT parsed, latest; 0 if none

// Appends the "GPS" frame's fields (everything after "type") to a frame that
// the caller began with protoWriterBegin(..., "GPS") and will finish.
// Integers only. Position fields are null unless gpsStatus() is GPS_OK.
void gpsWriteFrameFields(ProtoWriter *w);

#endif // GPS_H

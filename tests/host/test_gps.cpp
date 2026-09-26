// ============================================================================
//  FOCUSED HOST TESTS -- GPS bounded polling (gps.cpp)
//
//  REAL firmware under test, compiled unchanged: gps.cpp, rover_i2c.cpp,
//  protocol.cpp and config.h. Simulated below them:
//
//    * the clock   -- manual. step() advances 1 ms and calls gpsUpdate() once,
//                     as loop() does. micros() also advances by a modelled I2C
//                     cost of 90 us per byte on the wire (9 clocks at 100 kHz,
//                     address byte included), so gpsMaxServiceUs() reports the
//                     modelled bus time of the worst single gpsUpdate() call.
//    * the I2C bus -- one fake u-blox DDC port at NEO9M_I2C_ADDRESS: count
//                     registers 0xFD/0xFE, stream register 0xFF (0xFF filler
//                     when empty), answers a NAV-PVT poll with a scripted PVT
//                     after a scripted delay; can NACK / fail on demand, lie
//                     about its byte count, and hold an NMEA-like backlog
//    * the library -- tests/host/stubs/SparkFun_u-blox_GNSS_Arduino_Library.h
//                     (the allowed subset only: a forbidden call won't compile)
//
//  gps.cpp keeps its state in file statics, so the tests run as ONE timeline
//  in a fixed order; each test starts from the state the previous one left
//  and checks deltas of the cumulative counters.
//
//  What this CANNOT show: real ESP32 Wire/IDF timing and error codes, the
//  real module's response latency or output configuration, or the real
//  library's parser (the stub's parser is a minimal re-implementation).
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <deque>

#include "Arduino.h"
#include "Wire.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include "config.h"
#include "rover_i2c.h"
#include "protocol.h"
#include "gps.h"

// ---------------------------------------------------------------------------
// Arduino core: manual clock, inert GPIO
// ---------------------------------------------------------------------------

static uint32_t gNowMs = 0;
static uint32_t gNowUs = 0;

uint32_t millis(void)                         { return gNowMs; }
uint32_t micros(void)                         { return gNowUs; }
void     delay(uint32_t ms)                   { gNowMs += ms; gNowUs += ms * 1000u; }
void     delayMicroseconds(uint32_t us)       { gNowUs += us; }
void     pinMode(uint8_t pin, uint8_t mode)   { (void)pin; (void)mode; }
void     digitalWrite(uint8_t pin, uint8_t v) { (void)pin; (void)v; }
int      digitalRead(uint8_t pin)             { (void)pin; return HIGH; }
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeoutUs)
{
    (void)pin; (void)state; (void)timeoutUs;
    return 0;
}

// ---------------------------------------------------------------------------
// Scripted PVT
// ---------------------------------------------------------------------------

struct Pvt {
    uint32_t iTow;
    uint8_t  fixType;
    bool     fixOk;
    uint8_t  numSV;
    int32_t  lon, lat, hMsl;
    uint32_t hAcc, vAcc;
    int32_t  gSpeed, headMot;
    uint16_t pDop;
};

static const Pvt kFix3D = { 123456000u, 3, true, 18, 775193730, 128998934, 861690,
                            1500u, 2500u, 20, 34358000, 120 };

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// A complete, checksummed UBX-NAV-PVT frame (100 bytes).
static void buildPvtFrame(const Pvt &s, std::deque<uint8_t> &out)
{
    uint8_t pl[92];
    memset(pl, 0, sizeof(pl));
    put32(&pl[0], s.iTow);
    pl[20] = s.fixType;
    pl[21] = s.fixOk ? 0x01 : 0x00;
    pl[23] = s.numSV;
    put32(&pl[24], (uint32_t)s.lon);
    put32(&pl[28], (uint32_t)s.lat);
    put32(&pl[36], (uint32_t)s.hMsl);
    put32(&pl[40], s.hAcc);
    put32(&pl[44], s.vAcc);
    put32(&pl[60], (uint32_t)s.gSpeed);
    put32(&pl[64], (uint32_t)s.headMot);
    pl[76] = (uint8_t)s.pDop; pl[77] = (uint8_t)(s.pDop >> 8);

    uint8_t hdr[4] = { 0x01, 0x07, 92, 0 };
    uint8_t a = 0, b = 0;
    out.push_back(0xB5);
    out.push_back(0x62);
    for (int i = 0; i < 4; i++)  { out.push_back(hdr[i]); a += hdr[i]; b += a; }
    for (int i = 0; i < 92; i++) { out.push_back(pl[i]);  a += pl[i];  b += a; }
    out.push_back(a);
    out.push_back(b);
}

// ---------------------------------------------------------------------------
// Fake u-blox DDC port on the I2C bus
// ---------------------------------------------------------------------------

TwoWire Wire;

enum { PTR_COUNT, PTR_STREAM };

struct FakeGps {
    bool     present;         // ACKs its address
    int      failIn;          // -1 = never; N = the (N+1)th transaction from now fails
    int      countOverride;   // -1 = report the real backlog; else this raw 16-bit value
    bool     respond;         // answer NAV-PVT polls
    uint32_t respDelayMs;
    Pvt      pvt;

    std::deque<uint8_t> fifo; // module output buffer
    bool     pending;
    uint32_t pendingAtMs;
    int      ptr;

    // observation
    long     transactions;    // every START on the bus to this device
    int      navPvtPolls;
    int      cfgPrtPolls;
    int      probes;          // address-only transactions
    int      otherWrites;     // ANY other write -- must stay 0 (no config)
    int      passBytes;       // stream bytes read since the last count read
    int      maxPassBytes;
    int      maxChunk;
    long     streamBytes;
    long     streamReads;     // stream (0xFF register) read transactions

    // hung transactions (the ESP-IDF 4.4 driver waits >= 1000 ms for a
    // completion interrupt that never comes, whatever Wire.setTimeOut says)
    int      hangReadIn;      // -1 = never; N = the (N+1)th requestFrom from now hangs
    uint32_t hangMs;          // how long a hung transaction blocks
    bool     hangSucceeds;    // after the delay: complete normally (true) or fail (false)
    bool     hangPoll;        // the next NAV-PVT poll write hangs hangMs, then fails
    int      hungPolls;
};

static FakeGps G;

static int     gTxAddr = -1;
static uint8_t gTxBuf[64];
static int     gTxLen  = 0;
// = ESP32 core 2.0.14 Wire I2C_BUFFER_LENGTH. The real requestFrom() does NOT
// bound-check against it (a larger read overflows the heap); here a larger
// request fails instead, so an oversize chunk can never pass these tests.
#define SIM_WIRE_BUFFER_LENGTH 128
static_assert(GPS_I2C_CHUNK_BYTES <= SIM_WIRE_BUFFER_LENGTH,
              "GPS_I2C_CHUNK_BYTES exceeds the ESP32 Wire buffer (unchecked overflow on hardware)");
static_assert(GPS_I2C_CHUNK_BYTES <= GPS_MAX_BYTES_PER_PASS,
              "a chunk larger than the per-pass cap is meaningless");
static uint8_t gRxBuf[SIM_WIRE_BUFFER_LENGTH];
static int     gRxLen  = 0;
static int     gRxPos  = 0;

static void busCost(int bytes) { gNowUs += (uint32_t)(bytes + 1) * 90u; }

static bool failNow(void)
{
    if (G.failIn < 0) {
        return false;
    }
    if (G.failIn == 0) {
        G.failIn = -1;
        return true;
    }
    G.failIn--;
    return false;
}

static void blockFor(uint32_t ms) { gNowMs += ms; gNowUs += ms * 1000u; }

static bool hangReadNow(void)
{
    if (G.hangReadIn < 0) {
        return false;
    }
    if (G.hangReadIn == 0) {
        G.hangReadIn = -1;
        blockFor(G.hangMs);
        return true;
    }
    G.hangReadIn--;
    return false;
}

static void releasePending(void)
{
    if (G.pending && (int32_t)(gNowMs - G.pendingAtMs) >= 0) {
        G.pending = false;
        buildPvtFrame(G.pvt, G.fifo);
    }
}

bool TwoWire::begin(int sda, int scl, uint32_t freq)
{
    (void)sda; (void)scl; (void)freq;
    return true;
}

void TwoWire::beginTransmission(int address)
{
    gTxAddr = address;
    gTxLen  = 0;
}

size_t TwoWire::write(uint8_t b)
{
    if (gTxLen < (int)sizeof(gTxBuf)) {
        gTxBuf[gTxLen++] = b;
    }
    return 1;
}

uint8_t TwoWire::endTransmission(bool sendStop)
{
    (void)sendStop;
    busCost(gTxLen);
    if (gTxAddr != NEO9M_I2C_ADDRESS) {
        return 2;
    }
    G.transactions++;
    if (!G.present) {
        return 2;
    }
    if (failNow()) {
        return 4;
    }

    if (gTxLen == 0) {
        G.probes++;
    } else if (gTxLen == 1 && gTxBuf[0] == 0xFD) {
        G.ptr = PTR_COUNT;
    } else if (gTxLen == 8 && gTxBuf[0] == 0xB5 && gTxBuf[1] == 0x62 &&
               gTxBuf[2] == 0x01 && gTxBuf[3] == 0x07 && gTxBuf[4] == 0 && gTxBuf[5] == 0) {
        G.navPvtPolls++;
        if (G.hangPoll) {                           // write hangs, then fails (5 = timeout)
            G.hangPoll = false;
            G.hungPolls++;
            blockFor(G.hangMs);
            return 5;
        }
        if (G.respond) {
            G.pending     = true;
            G.pendingAtMs = gNowMs + G.respDelayMs;
        }
    } else if (gTxLen == 9 && gTxBuf[0] == 0xB5 && gTxBuf[1] == 0x62 &&
               gTxBuf[2] == 0x06 && gTxBuf[3] == 0x00 && gTxBuf[4] == 1 && gTxBuf[5] == 0) {
        G.cfgPrtPolls++;
    } else {
        G.otherWrites++;
    }
    return 0;
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
    gRxLen = 0;
    gRxPos = 0;
    busCost(quantity);
    if (address != NEO9M_I2C_ADDRESS || quantity < 1 || quantity > (int)sizeof(gRxBuf)) {
        return 0;
    }
    G.transactions++;
    if (!G.present || failNow()) {
        return 0;
    }
    if (hangReadNow() && !G.hangSucceeds) {
        return 0;
    }
    releasePending();

    if (G.ptr == PTR_COUNT) {
        uint16_t c = (G.countOverride >= 0) ? (uint16_t)G.countOverride
                                            : (uint16_t)(G.fifo.size() > 0x7FFF ? 0x7FFF : G.fifo.size());
        gRxBuf[0] = (uint8_t)(c >> 8);
        gRxBuf[1] = (uint8_t)c;
        gRxLen    = (quantity < 2) ? quantity : 2;
        G.ptr     = PTR_STREAM;
        G.passBytes = 0;
        return (uint8_t)gRxLen;
    }

    for (int i = 0; i < quantity; i++) {
        if (G.fifo.empty()) {
            gRxBuf[i] = 0xFF;
        } else {
            gRxBuf[i] = G.fifo.front();
            G.fifo.pop_front();
        }
    }
    gRxLen = quantity;
    G.streamBytes += quantity;
    G.streamReads++;
    G.passBytes   += quantity;
    if (G.passBytes > G.maxPassBytes) G.maxPassBytes = G.passBytes;
    if (quantity > G.maxChunk)        G.maxChunk     = quantity;
    return (uint8_t)quantity;
}

int TwoWire::available(void) { return gRxLen - gRxPos; }

int TwoWire::read(void)
{
    if (gRxPos >= gRxLen) {
        return -1;
    }
    return gRxBuf[gRxPos++];
}

// ---------------------------------------------------------------------------
// Test plumbing
// ---------------------------------------------------------------------------

static const char *gTest   = "";
static int         gChecks = 0;
static int         gFails  = 0;

#define CHECK(cond, msg)                                                      \
    do {                                                                      \
        gChecks++;                                                            \
        if (!(cond)) {                                                        \
            gFails++;                                                         \
            printf("  FAIL [%s] line %d: %s\n", gTest, __LINE__, msg);        \
        }                                                                     \
    } while (0)

static void step(void)
{
    gNowMs++;
    gNowUs += 1000u;
    gpsUpdate();
}

static void run(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        step();
    }
}

template <typename F>
static bool runUntil(F done, uint32_t maxMs)
{
    for (uint32_t i = 0; i < maxMs; i++) {
        if (done()) {
            return true;
        }
        step();
    }
    return done();
}

static char   gFrame[1024];
static size_t gFrameLen = 0;

// Builds the frame exactly as comm.cpp does (minus uptime_ms, added there) and
// checks its CRC trailer.
static void buildFrame(void)
{
    ProtoWriter w;
    protoWriterBegin(&w, gFrame, sizeof(gFrame), "GPS");
    gpsWriteFrameFields(&w);
    gFrameLen = protoWriterFinish(&w);
    CHECK(gFrameLen > 0, "GPS frame did not fit");
    if (gFrameLen == 0) {
        gFrame[0] = '\0';
        return;
    }
    gFrame[gFrameLen] = '\0';

    const char *star = strrchr(gFrame, '*');
    CHECK(star != NULL && gFrame[gFrameLen - 1] == '\n', "frame trailer missing");
    if (star != NULL) {
        char want[8];
        snprintf(want, sizeof(want), "%04X", (unsigned)protoCrc16(gFrame, (size_t)(star - gFrame)));
        CHECK(strncmp(star + 1, want, 4) == 0, "frame CRC wrong");
        // Integers only: no '.' anywhere in the payload.
        CHECK(memchr(gFrame, '.', (size_t)(star - gFrame)) == NULL, "non-integer value in frame");
    }
    CHECK(strncmp(gFrame, "{\"type\":\"GPS\"", 13) == 0, "frame type is not GPS");
}

static bool has(const char *needle) { return strstr(gFrame, needle) != NULL; }

static void checkPositionNull(void)
{
    CHECK(has("\"lat_e7\":null,\"lon_e7\":null,\"alt_msl_mm\":null,\"hacc_mm\":null,"
              "\"vacc_mm\":null,\"speed_mm_s\":null,\"head_mot_e5\":null,\"pdop_e2\":null"),
          "position fields not all null");
}

// Waits for the next PVT to be consumed. Returns true if one was.
static bool waitPvt(uint32_t maxMs)
{
    uint32_t before = gpsPvtOk();
    return runUntil([before] { return gpsPvtOk() > before; }, maxMs);
}

// ---------------------------------------------------------------------------
// Tests (ONE timeline -- order matters)
// ---------------------------------------------------------------------------

static void testNotStarted(void)
{
    gTest = "NOT_STARTED";
    gpsInit();                                          // I2C not up yet
    CHECK(gpsStatus() == GPS_NOT_STARTED, "status");
    long tx = G.transactions;
    run(3000);
    CHECK(G.transactions == tx, "bus traffic while NOT_STARTED");
    CHECK(sfeStubLog().beginCalls == 0, "begin() called without I2C");
    buildFrame();
    CHECK(has("\"gps_status\":\"NOT_STARTED\""), "frame status");
    CHECK(has("\"fix_type\":null,\"fix_ok\":null,\"siv\":null"), "fix fields not null");
    checkPositionNull();
    CHECK(has("\"itow_ms\":null,\"age_ms\":null"), "time fields not null");
}

static void testInit(void)
{
    gTest = "init";
    roverI2cInit();
    CHECK(roverI2cReady(), "roverI2cInit failed");
    long tx = G.transactions;
    gpsInit();
    CHECK(sfeStubLog().beginCalls == 1, "begin() not called once");
    CHECK(sfeStubLog().beginMaxWait == 0, "begin() maxWait must be 0");
    CHECK(G.cfgPrtPolls == 3, "expected exactly 3 CFG-PRT polls from begin()");
    CHECK(G.transactions - tx == 6, "begin(): expected 3 probes + 3 poll writes");
    CHECK(G.otherWrites == 0, "a non-poll write was sent");
    CHECK(gpsStatus() == GPS_NOT_DETECTED, "status after init");
}

static void testResponseTimeout(void)
{
    gTest = "response timeout";
    G.respond = false;
    uint32_t polls0 = gpsPollsSent(), to0 = gpsPollTimeouts();

    step();                                             // IDLE -> REQUEST
    CHECK(gpsPollsSent() == polls0 + 1, "no poll request on first update");
    CHECK(G.navPvtPolls == (int)(polls0 + 1), "poll not on the bus");
    CHECK(sfeStubLog().getPvtMaxWaitMax == 0, "getPVT() called with maxWait > 0");

    run(GPS_RESPONSE_TIMEOUT_MS - 1);
    CHECK(gpsPollTimeouts() == to0, "timed out early");
    CHECK(gpsStatus() == GPS_NOT_DETECTED, "status while waiting");
    run(1);
    CHECK(gpsPollTimeouts() == to0 + 1, "no timeout at 500 ms");
    CHECK(gpsStatus() == GPS_NOT_DETECTED, "status after timeout");
    CHECK(gpsBusErrors() == 0, "timeout counted as bus error");

    // Back in IDLE: no traffic until the 1 s poll period is up.
    long tx = G.transactions;
    run(GPS_POLL_PERIOD_MS - GPS_RESPONSE_TIMEOUT_MS - 1);
    CHECK(G.transactions == tx, "bus traffic in IDLE");
    CHECK(gpsPollsSent() == polls0 + 1, "polled before the period");
    run(1);
    CHECK(gpsPollsSent() == polls0 + 2, "no poll after 1000 ms");

    // Receive passes during WAIT: at most one per GPS_SERVICE_INTERVAL_MS.
    // Each empty pass is 2 transactions (count pointer write + 2-byte read).
    tx = G.transactions;
    run(GPS_RESPONSE_TIMEOUT_MS);
    long maxPasses = (long)(GPS_RESPONSE_TIMEOUT_MS / GPS_SERVICE_INTERVAL_MS);
    CHECK((G.transactions - tx) <= 2 * maxPasses, "more than one pass per 20 ms");
    CHECK((G.transactions - tx) >= 2 * (maxPasses - 1), "passes not happening");

    buildFrame();
    CHECK(has("\"gps_status\":\"NOT_DETECTED\""), "frame status");
    CHECK(has("\"fix_type\":null"), "fix_type should be null before any PVT");
    checkPositionNull();
    CHECK(has("\"poll_timeouts\":2"), "poll_timeouts in frame");
}

static void testSuccessfulPvt(void)
{
    gTest = "successful PVT";
    G.respond     = true;
    G.respDelayMs = 30;
    G.pvt         = kFix3D;
    int flush0    = sfeStubLog().flushCalls;

    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT consumed");
    CHECK(gpsStatus() == GPS_OK, "status not OK");
    CHECK(sfeStubLog().flushCalls > flush0, "flushPVT() not called after copy");
    CHECK(gpsLastLatencyMs() >= 30 && gpsLastLatencyMs() < GPS_RESPONSE_TIMEOUT_MS, "latency");
    CHECK(gpsAgeMs() == 0, "age should be 0 right after parse");

    GpsFix f;
    CHECK(gpsLatest(&f), "gpsLatest false");
    CHECK(f.fixOk && f.fixType == 3 && f.numSV == 18, "fix/type/siv");
    CHECK(f.latE7 == 128998934 && f.lonE7 == 775193730, "lat/lon");
    CHECK(f.hMslMm == 861690 && f.hAccMm == 1500 && f.vAccMm == 2500, "alt/acc");
    CHECK(f.gSpeedMmS == 20 && f.headMotE5 == 34358000 && f.pDopE2 == 120, "speed/head/pdop");
    CHECK(f.iTowMs == 123456000u && f.rxMs == gNowMs, "itow/rxMs");

    // Steady state: one poll and one PVT per second, status OK throughout.
    uint32_t polls0 = gpsPollsSent(), ok0 = gpsPvtOk();
    bool alwaysOk = true;
    for (int i = 0; i < 5000; i++) {
        step();
        if (gpsStatus() != GPS_OK) alwaysOk = false;
    }
    CHECK(alwaysOk, "status left OK during steady polling");
    CHECK(gpsPollsSent() - polls0 == 5, "not 1 poll per second");
    CHECK(gpsPvtOk() - ok0 == 5, "not 1 PVT per second");
}

static void testValidFrame(void)
{
    gTest = "valid GPS frame";
    CHECK(gpsStatus() == GPS_OK, "precondition: OK");
    buildFrame();
    CHECK(has("\"gps_status\":\"OK\",\"fix_type\":3,\"fix_ok\":true,\"siv\":18,"
              "\"lat_e7\":128998934,\"lon_e7\":775193730,\"alt_msl_mm\":861690,"
              "\"hacc_mm\":1500,\"vacc_mm\":2500,\"speed_mm_s\":20,"
              "\"head_mot_e5\":34358000,\"pdop_e2\":120,\"itow_ms\":123456000,\"age_ms\":"),
          "frame body");
    const char *keys[] = { "polls", "pvt_ok", "poll_timeouts", "bus_errors", "ff_chunks",
                           "max_service_us", "latency_ms" };
    for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
        char k[32];
        snprintf(k, sizeof(k), "\"%s\":", keys[i]);
        CHECK(has(k), keys[i]);
    }
    CHECK(gFrameLen < 600, "frame larger than expected");
    printf("  GPS frame (%u bytes, uptime_ms is added by comm.cpp): %s",
           (unsigned)gFrameLen, gFrame);
}

static void testStale(void)
{
    gTest = "stale";
    G.respond = false;
    // Let the current cycle finish so the last PVT is well defined.
    run(GPS_POLL_PERIOD_MS);
    GpsFix f;
    CHECK(gpsLatest(&f), "no fix to go stale");
    uint32_t last = f.rxMs;

    CHECK(runUntil([] { return gpsStatus() != GPS_OK; }, 10000), "never left OK");
    CHECK(gpsStatus() == GPS_STALE, "left OK but not to STALE");
    CHECK(gNowMs - last == GPS_STALE_MS, "STALE not at exactly 3000 ms");
    CHECK(gpsPollTimeouts() >= 2, "polls should have timed out meanwhile");

    buildFrame();
    CHECK(has("\"gps_status\":\"STALE\",\"fix_type\":3,\"fix_ok\":true,\"siv\":18"),
          "STALE frame should keep the last fix type/siv");
    checkPositionNull();
    CHECK(has("\"age_ms\":3000"), "age_ms");
}

static void testNoFix(void)
{
    gTest = "NO_FIX";
    G.respond = true;

    Pvt nofix = kFix3D;
    nofix.fixType = 0; nofix.fixOk = false; nofix.numSV = 2;
    G.pvt = nofix;
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT");
    CHECK(gpsStatus() == GPS_NO_FIX, "fixType 0 not NO_FIX");
    buildFrame();
    CHECK(has("\"gps_status\":\"NO_FIX\",\"fix_type\":0,\"fix_ok\":false,\"siv\":2"),
          "NO_FIX fix fields");
    checkPositionNull();
    CHECK(has("\"itow_ms\":123456000,\"age_ms\":"), "NO_FIX should still carry itow/age");

    Pvt p = kFix3D; p.fixOk = false;                    // 3D but not gnssFixOK
    G.pvt = p;
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT");
    CHECK(gpsStatus() == GPS_NO_FIX, "fixOk false not NO_FIX");

    p = kFix3D; p.fixType = 5;                          // time only
    G.pvt = p;
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT");
    CHECK(gpsStatus() == GPS_NO_FIX, "fixType 5 not NO_FIX");

    p = kFix3D; p.fixType = 1;                          // dead reckoning only
    G.pvt = p;
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT");
    CHECK(gpsStatus() == GPS_NO_FIX, "fixType 1 not NO_FIX");

    p = kFix3D; p.fixType = 2;                          // 2D counts
    G.pvt = p;
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT");
    CHECK(gpsStatus() == GPS_OK, "fixType 2 not OK");

    p = kFix3D; p.fixType = 4;                          // GNSS + DR counts
    G.pvt = p;
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT");
    CHECK(gpsStatus() == GPS_OK, "fixType 4 not OK");

    G.pvt = kFix3D;
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT");
    CHECK(gpsStatus() == GPS_OK, "back to OK");
}

// A backoff of exactly lenMs starting at startMs: BACKOFF status and zero GPS
// bus traffic until lenMs, then traffic resumes and the status leaves BACKOFF.
static void checkBackoffLen(uint32_t startMs, long txAtStart, uint32_t lenMs)
{
    CHECK(gpsStatus() == GPS_BACKOFF, "not BACKOFF after failure");
    buildFrame();
    CHECK(has("\"gps_status\":\"BACKOFF\""), "frame status");
    checkPositionNull();

    bool quiet = true, backoff = true;
    while (gNowMs - startMs < lenMs - 1) {
        step();
        if (G.transactions != txAtStart) quiet = false;
        if (gpsStatus() != GPS_BACKOFF) backoff = false;
    }
    CHECK(quiet, "GPS bus traffic during backoff");
    CHECK(backoff, "status left BACKOFF early");

    step();                                             // lenMs: resume
    CHECK(gNowMs - startMs == lenMs, "timing bookkeeping");
    CHECK(G.transactions > txAtStart, "no bus traffic after backoff");
    CHECK(gpsStatus() != GPS_BACKOFF, "still BACKOFF after the backoff length");
}

static void checkBackoff(uint32_t failedAtMs, long txAtFail)
{
    checkBackoffLen(failedAtMs, txAtFail, GPS_FAIL_BACKOFF_MS);
}

static void testBusFailureBackoff(void)
{
    gTest = "bus failure -> backoff";
    G.respond = true;
    G.pvt     = kFix3D;

    // (a) The count read fails. Get to WAIT first, then fail the next START.
    uint32_t polls0 = gpsPollsSent();
    CHECK(runUntil([polls0] { return gpsPollsSent() > polls0; }, 2000), "no poll");
    uint32_t err0 = gpsBusErrors();
    G.failIn = 0;
    CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 100), "failure not detected");
    CHECK(gpsBusErrors() == err0 + 1, "bus_errors not +1");
    checkBackoff(gNowMs, G.transactions);
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT after backoff");
    CHECK(gpsStatus() == GPS_OK, "not OK after recovery");

    // (b) A stream chunk read fails mid-pass: the pass stops at once.
    polls0 = gpsPollsSent();
    CHECK(runUntil([polls0] { return gpsPollsSent() > polls0; }, 2000), "no poll");
    G.respDelayMs = 0;
    G.fifo.clear();
    for (int i = 0; i < 200; i++) G.fifo.push_back('A');
    err0 = gpsBusErrors();
    long read0 = G.streamBytes;
    G.failIn = 2;                                       // count ptr, count read, CHUNK 1 fails
    CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 100), "chunk failure not detected");
    CHECK(G.streamBytes == read0, "bytes consumed from a failed chunk");
    checkBackoff(gNowMs, G.transactions);
    G.fifo.clear();
    G.respDelayMs = 30;

    // (c) The device NACKs its address entirely.
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT");
    polls0 = gpsPollsSent();
    CHECK(runUntil([polls0] { return gpsPollsSent() > polls0; }, 2000), "no poll");
    err0 = gpsBusErrors();
    G.present = false;
    CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 100), "NACK not detected");
    long tx = G.transactions;
    run(GPS_FAIL_BACKOFF_MS - 1);
    CHECK(G.transactions == tx, "traffic during backoff (absent device)");
    G.present = true;
    CHECK(waitPvt(GPS_FAIL_BACKOFF_MS + 2 * GPS_POLL_PERIOD_MS), "no recovery after NACK");
    CHECK(gpsStatus() == GPS_OK, "not OK after device returned");
}

static void testReceiveCap(void)
{
    gTest = "128-byte cap";
    G.respond     = true;
    G.pvt         = kFix3D;
    G.respDelayMs = 0;
    G.maxPassBytes = 0;
    G.maxChunk     = 0;

    // 1000 bytes of NMEA-like backlog ahead of the PVT reply.
    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "precondition PVT");
    G.fifo.clear();
    const char *nmea = "$GNGGA,123519.00,1253.99360,N,07731.16238,E,1,18,0.8,861.7,M,,,,*47\r\n";
    while (G.fifo.size() < 1000) {
        for (const char *c = nmea; *c; c++) G.fifo.push_back((uint8_t)*c);
    }
    size_t backlog = G.fifo.size();

    uint32_t polls0 = gpsPollsSent();
    CHECK(runUntil([polls0] { return gpsPollsSent() > polls0; }, 2000), "no poll");
    long read0 = G.streamBytes;
    uint32_t ok0 = gpsPvtOk();

    long reads0 = G.streamReads;
    run(GPS_SERVICE_INTERVAL_MS);                       // exactly one pass
    CHECK(G.streamBytes - read0 == GPS_MAX_BYTES_PER_PASS, "first pass not exactly 128 bytes");
    CHECK(G.maxPassBytes == GPS_MAX_BYTES_PER_PASS, "a pass exceeded 128 bytes");
    CHECK(G.maxChunk == GPS_I2C_CHUNK_BYTES, "chunk size not GPS_I2C_CHUNK_BYTES");
    // A full pass = count read + ceil(128 / chunk) stream reads. With 128-byte
    // chunks that is ONE stream read (2 bus transactions instead of 5).
    CHECK(G.streamReads - reads0 ==
          (GPS_MAX_BYTES_PER_PASS + GPS_I2C_CHUNK_BYTES - 1) / GPS_I2C_CHUNK_BYTES,
          "wrong number of stream reads in a full pass");
    CHECK(G.streamReads - reads0 == 1, "a full 128-byte pass is not a single read");
    CHECK(G.fifo.size() == backlog + 100 - GPS_MAX_BYTES_PER_PASS, "backlog not reduced by 128");

    // The backlog drains 128 bytes per pass, then the PVT behind it is parsed.
    CHECK(waitPvt(GPS_RESPONSE_TIMEOUT_MS), "PVT behind the backlog lost");
    CHECK(gpsPvtOk() == ok0 + 1, "PVT count");
    CHECK(G.maxPassBytes <= GPS_MAX_BYTES_PER_PASS, "cap exceeded while draining");
    CHECK(gpsStatus() == GPS_OK, "status");

    // Modelled worst service time (wire bytes x 90 us only -- the model has
    // NO per-transaction overhead, which is what 128-byte chunks cut on real
    // hardware): count (2+3) + ceil(128/chunk) x (1+chunk) bytes. 128-byte
    // chunks: ~12.1 ms; 32-byte chunks: ~12.3 ms. Must stay under 13 ms.
    printf("  max_service_us (modelled bus time) = %lu\n", (unsigned long)gpsMaxServiceUs());
    CHECK(gpsMaxServiceUs() > 11000 && gpsMaxServiceUs() < 13000, "service time outside the modelled bound");
}

static void testAllFfChunk(void)
{
    gTest = "all-0xFF chunk";
    G.respond = false;
    G.fifo.clear();
    run(GPS_POLL_PERIOD_MS);                            // let a fresh poll start
    uint32_t polls0 = gpsPollsSent();
    CHECK(runUntil([polls0] { return gpsPollsSent() > polls0; }, 2000), "no poll");

    // The module claims 32767 bytes but its stream is empty (0xFF filler).
    G.countOverride = 0xFFFF;                           // bit 15 masked -> 0x7FFF
    uint32_t ff0 = gpsFfChunks(), err0 = gpsBusErrors();
    long read0 = G.streamBytes;
    run(GPS_SERVICE_INTERVAL_MS);                       // one pass
    CHECK(gpsFfChunks() == ff0 + 1, "all-0xFF chunk not counted");
    CHECK(G.streamBytes - read0 == GPS_I2C_CHUNK_BYTES, "pass did not stop after one 0xFF chunk");
    CHECK(gpsBusErrors() == err0, "0xFF chunk treated as bus error");

    // Every later pass in this WAIT also stops after one chunk.
    read0 = G.streamBytes;
    uint32_t passes0 = gpsFfChunks();
    run(GPS_RESPONSE_TIMEOUT_MS);
    uint32_t passes = gpsFfChunks() - passes0;
    CHECK(passes > 0 && G.streamBytes - read0 == (long)passes * GPS_I2C_CHUNK_BYTES,
          "a lying count made a pass read more than one chunk");

    buildFrame();
    CHECK(has("\"ff_chunks\":"), "ff_chunks field");
    G.countOverride = -1;
}

// ---------------------------------------------------------------------------
// Hung transactions and exponential backoff
//
// On the real ESP32 (core 2.0.14 / ESP-IDF 4.4.6) a transaction whose
// completion interrupt never arrives blocks >= 1000 ms whatever
// Wire.setTimeOut() says. gps.cpp cannot shorten that; these tests prove it
// is detected, costs ONE stall, and backs off exponentially.
// ---------------------------------------------------------------------------

// Healthy PVT first, so the next backoff is the 5 s first step.
static void getHealthy(void)
{
    G.present = true; G.failIn = -1; G.hangReadIn = -1; G.hangPoll = false;
    G.countOverride = -1; G.respond = true; G.respDelayMs = 30; G.pvt = kFix3D;
    G.fifo.clear();
    CHECK(waitPvt(GPS_BACKOFF_MAX_MS + 2 * GPS_POLL_PERIOD_MS), "precondition: no PVT");
}

static bool waitPoll(uint32_t maxMs)
{
    uint32_t p0 = gpsPollsSent();
    return runUntil([p0] { return gpsPollsSent() > p0; }, maxMs);
}

static void testHungRead(void)
{
    gTest = "1000 ms hung read";
    getHealthy();
    CHECK(waitPoll(2 * GPS_POLL_PERIOD_MS), "no poll");
    uint32_t err0 = gpsBusErrors();
    long     tx0  = G.transactions;
    uint32_t t0   = gNowMs;
    G.hangReadIn = 0; G.hangMs = 1000; G.hangSucceeds = false;  // the count read hangs, then fails

    CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 100), "hung read not detected");
    CHECK(gNowMs - t0 >= 1000, "clock did not advance through the hang");
    CHECK(gpsMaxServiceUs() >= 1000000u, "1 s stall not measured");
    CHECK(gpsBusErrors() == err0 + 1, "one hang counted more than once");
    CHECK(G.transactions - tx0 == 2, "stalled pass went on past the hung read");
    // timed from the END of the stall, so the full 5 s of silence follows it
    checkBackoffLen(gNowMs, G.transactions, GPS_FAIL_BACKOFF_MS);
}

static void testHungPollWrite(void)
{
    gTest = "hung poll-request write";
    getHealthy();
    uint32_t err0 = gpsBusErrors(), polls0 = gpsPollsSent();
    G.hangPoll = true; G.hangMs = 1000;         // getPVT(0) ignores the failure

    CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 2 * GPS_POLL_PERIOD_MS),
          "hung poll write not detected (it is invisible except by its duration)");
    CHECK(G.hungPolls == 1 && gpsPollsSent() == polls0 + 1, "the slow call was not the poll");
    CHECK(gpsBusErrors() == err0 + 1, "counted more than once");
    CHECK(gpsStatus() == GPS_BACKOFF, "not BACKOFF");
    // No count read may follow: on a dead bus that would be a SECOND ~1 s stall.
    checkBackoffLen(gNowMs, G.transactions, GPS_FAIL_BACKOFF_MS);
}

static void testSlowCallTimeout(void)
{
    gTest = "slow call (timeout) -> BACKOFF";
    getHealthy();

    // 80 ms stretched but successful: under GPS_SLOW_CALL_US, not a failure.
    CHECK(waitPoll(2 * GPS_POLL_PERIOD_MS), "no poll");
    uint32_t err0 = gpsBusErrors();
    G.hangReadIn = 0; G.hangMs = 80; G.hangSucceeds = true;
    run(GPS_SERVICE_INTERVAL_MS);
    CHECK(G.hangReadIn == -1, "80 ms delay not injected");
    CHECK(gpsBusErrors() == err0 && gpsStatus() != GPS_BACKOFF, "80 ms call treated as failure");

    // 150 ms, still successful: over the limit -> failed attempt -> BACKOFF.
    CHECK(waitPoll(2 * GPS_POLL_PERIOD_MS), "no poll");
    err0 = gpsBusErrors();
    G.hangReadIn = 0; G.hangMs = 150; G.hangSucceeds = true;
    CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 100), "150 ms call not treated as failure");
    CHECK(gpsBusErrors() == err0 + 1, "counted more than once");
    checkBackoffLen(gNowMs, G.transactions, GPS_FAIL_BACKOFF_MS);
    G.hangSucceeds = false;
}

static void testBackoffProgression(void)
{
    gTest = "exponential backoff progression";
    getHealthy();
    G.present = false;                          // every attempt now fails fast (NACK)

    uint32_t err0 = gpsBusErrors();
    CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 2 * GPS_POLL_PERIOD_MS), "no failure");

    const uint32_t expect[] = { 5000, 10000, 20000, 40000, 80000, 160000, 300000, 300000 };
    for (size_t i = 0; i < sizeof(expect) / sizeof(expect[0]); i++) {
        uint32_t f = gNowMs;
        checkBackoffLen(f, G.transactions, expect[i]);   // silent + BACKOFF for exactly expect[i]
        err0 = gpsBusErrors();
        CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 100), "no failure after backoff");
        // backoff, then the poll write, then the 20 ms pass that fails
        CHECK(gNowMs - f == expect[i] + GPS_SERVICE_INTERVAL_MS, "backoff length wrong");
    }
    printf("  backoff windows verified: 5 10 20 40 80 160 300 300 s\n");
}

static void testBackoffReset(void)
{
    gTest = "successful PVT resets backoff";
    CHECK(gpsStatus() == GPS_BACKOFF, "precondition: in the capped backoff");
    uint32_t f = gNowMs;
    G.present = true; G.respond = true; G.respDelayMs = 30; G.pvt = kFix3D; G.fifo.clear();
    checkBackoffLen(f, G.transactions, GPS_BACKOFF_MAX_MS);   // still honours the 300 s window

    CHECK(waitPvt(2 * GPS_POLL_PERIOD_MS), "no PVT after the device returned");
    CHECK(gpsStatus() == GPS_OK, "not OK after recovery");

    CHECK(waitPoll(2 * GPS_POLL_PERIOD_MS), "no poll");
    uint32_t err0 = gpsBusErrors();
    G.failIn = 0;
    CHECK(runUntil([err0] { return gpsBusErrors() > err0; }, 100), "failure not detected");
    checkBackoff(gNowMs, G.transactions);       // back to exactly 5 s
}

// With 128-byte chunks a pass is ONE chunk, so the hard cap -- not the 0xFF
// stop -- ends it. What the 0xFF stop still guarantees: filler never reaches
// the parser, so it cannot corrupt a UBX frame that is half received.
static void testFfFillerNotParsed(void)
{
    gTest = "0xFF filler kept out of the parser";
    getHealthy();
    G.respond = false;                                  // reply injected by hand below
    CHECK(waitPoll(2 * GPS_POLL_PERIOD_MS), "no poll");
    std::deque<uint8_t> frame;
    buildPvtFrame(kFix3D, frame);                       // 100 bytes
    uint32_t ok0 = gpsPvtOk();

    for (int i = 0; i < 50; i++) { G.fifo.push_back(frame.front()); frame.pop_front(); }
    run(GPS_SERVICE_INTERVAL_MS);                       // pass 1: first half of the reply
    CHECK(G.fifo.empty(), "first half not read");

    G.countOverride = 0x7FFF;                           // pass 2: count lies, stream is filler
    uint32_t ff0 = gpsFfChunks();
    run(GPS_SERVICE_INTERVAL_MS);
    CHECK(gpsFfChunks() == ff0 + 1, "filler chunk not detected");

    G.countOverride = -1;                               // pass 3: rest of the reply
    while (!frame.empty()) { G.fifo.push_back(frame.front()); frame.pop_front(); }
    run(GPS_SERVICE_INTERVAL_MS);
    CHECK(gpsPvtOk() == ok0 + 1, "PVT lost: 0xFF filler was fed into a half-received frame");
    G.respond = true;
}

static void testNoConfigWritten(void)
{
    gTest = "no configuration written";
    CHECK(G.otherWrites == 0, "a write other than a NAV-PVT/CFG-PRT poll or count pointer");
    CHECK(G.cfgPrtPolls == 3, "CFG-PRT polls outside begin()");
    CHECK(sfeStubLog().getPvtMaxWaitMax == 0, "getPVT() ever called with maxWait > 0");
    CHECK(sfeStubLog().beginCalls == 1, "begin() called more than once");
    CHECK(sfeStubLog().badChecksums == 0, "parser saw corrupt UBX frames");
}

int main(void)
{
    G.present       = true;
    G.failIn        = -1;
    G.countOverride = -1;
    G.respond       = true;
    G.respDelayMs   = 30;
    G.pvt           = kFix3D;
    G.ptr           = PTR_STREAM;
    G.hangReadIn    = -1;
    G.hangMs        = 1000;
    G.hangSucceeds  = false;
    G.hangPoll      = false;

    struct { const char *name; void (*fn)(void); } tests[] = {
        { "NOT_STARTED: no bus traffic, null frame",               testNotStarted },
        { "init: begin(maxWait 0), 3 CFG-PRT polls only",          testInit },
        { "response timeout -> NOT_DETECTED, pass pacing",         testResponseTimeout },
        { "successful PVT, 1 Hz steady state",                     testSuccessfulPvt },
        { "valid GPS frame contents",                              testValidFrame },
        { "stale after 3000 ms",                                   testStale },
        { "NO_FIX fields and fixType/fixOk rules",                 testNoFix },
        { "bus failure -> bus error + 5000 ms silent backoff",     testBusFailureBackoff },
        { "128-byte maximum receive cap",                          testReceiveCap },
        { "all-0xFF chunk stops the pass",                         testAllFfChunk },
        { "1000 ms hung read: one stall, then silent backoff",     testHungRead },
        { "hung poll write: detected, no second stall",            testHungPollWrite },
        { "slow call (timeout) -> BACKOFF; 80 ms is not",          testSlowCallTimeout },
        { "exponential backoff 5 -> 300 s",                        testBackoffProgression },
        { "successful PVT resets backoff to 5 s",                  testBackoffReset },
        { "0xFF filler never reaches the parser mid-frame",        testFfFillerNotParsed },
        { "no configuration written, getPVT maxWait always 0",     testNoConfigWritten },
    };

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = gFails;
        tests[i].fn();
        printf("%s  %s\n", gFails == before ? "PASS" : "FAIL", tests[i].name);
    }
    printf("GPS TESTS: %d checks, %d failed\n", gChecks, gFails);
    return gFails ? 1 : 0;
}

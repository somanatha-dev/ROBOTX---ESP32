#ifndef SIM_SPARKFUN_UBLOX_GNSS_H
#define SIM_SPARKFUN_UBLOX_GNSS_H

// ============================================================================
//  HOST STUB -- SparkFun u-blox GNSS Arduino Library (v2.2.29 API subset)
//
//  ONLY the calls gps.cpp is allowed to make exist here: begin(), getPVT(),
//  process(), flushPVT() and the public packetUBXNAVPVT pointer. Anything
//  else -- getLatitude(), checkUblox(), setI2COutput(), setAutoPVT(),
//  saveConfiguration(), ... -- is deliberately absent, so a forbidden call
//  FAILS TO COMPILE on the host.
//
//  Behaviour mirrors the real library where gps.cpp depends on it:
//    begin(port, addr, maxWait)  address check, then three 9-byte UBX-CFG-PRT
//                                poll writes (real: isConnected() x3)
//    getPVT(maxWait)             allocates packetUBXNAVPVT, writes ONE 8-byte
//                                UBX-NAV-PVT poll; returns false at maxWait 0
//    process(byte, ...)          UBX framing + Fletcher checksum; a valid
//                                NAV-PVT fills packetUBXNAVPVT->data and sets
//                                every moduleQueried bit (real: .cpp:3353-3392)
//  Every call is logged in sfeStubLog() so tests can assert on it.
// ============================================================================

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "Wire.h"

#define MAX_PAYLOAD_SIZE 276

typedef enum {
    SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
    SFE_UBLOX_PACKET_VALIDITY_VALID,
    SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
    SFE_UBLOX_PACKET_NOTACKNOWLEDGED
} sfe_ublox_packet_validity_e;

struct ubxPacket {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint16_t counter;
    uint16_t startingSpot;
    uint8_t *payload;
    uint8_t  checksumA;
    uint8_t  checksumB;
    sfe_ublox_packet_validity_e valid;
    sfe_ublox_packet_validity_e classAndIDmatch;
};

typedef struct {
    uint32_t iTOW;
    uint8_t  fixType;
    union { uint8_t all; struct { uint8_t gnssFixOK : 1; uint8_t rest : 7; } bits; } flags;
    uint8_t  numSV;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t  gSpeed;
    int32_t  headMot;
    uint16_t pDOP;
} UBX_NAV_PVT_data_t;

typedef struct {
    union { uint32_t all; struct { uint32_t all : 1; uint32_t rest : 31; } bits; } moduleQueried1;
    union { uint32_t all; } moduleQueried2;
} UBX_NAV_PVT_moduleQueried_t;

typedef struct {
    UBX_NAV_PVT_data_t          data;
    UBX_NAV_PVT_moduleQueried_t moduleQueried;
} UBX_NAV_PVT_t;

// ---- call log (one instance shared by every translation unit) --------------
struct SfeStubLog {
    int      beginCalls;
    uint16_t beginMaxWait;
    int      getPvtCalls;
    uint16_t getPvtMaxWaitMax;      // largest maxWait ever passed to getPVT()
    int      flushCalls;
    int      pvtParsed;
    int      badChecksums;
};
inline SfeStubLog &sfeStubLog(void) { static SfeStubLog l; return l; }

class SFE_UBLOX_GNSS {
public:
    UBX_NAV_PVT_t *packetUBXNAVPVT;

    SFE_UBLOX_GNSS() : packetUBXNAVPVT(NULL), _port(NULL), _addr(0x42), _state(0),
                       _cls(0), _id(0), _len(0), _count(0), _ckA(0), _ckB(0) {}

    bool begin(TwoWire &port, uint8_t addr = 0x42, uint16_t maxWait = 1100,
               bool assumeSuccess = false)
    {
        (void)assumeSuccess;
        _port = &port;
        _addr = addr;
        sfeStubLog().beginCalls++;
        sfeStubLog().beginMaxWait = maxWait;
        for (int attempt = 0; attempt < 3; attempt++) {
            _port->beginTransmission(_addr);
            if (_port->endTransmission() != 0) {
                continue;                               // no ACK: next attempt
            }
            static const uint8_t cfgPrtPoll[9] =
                { 0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x00, 0x07, 0x21 };
            sendRaw(cfgPrtPoll, sizeof(cfgPrtPoll));
        }
        return false;                                   // maxWait 0: never DATA_RECEIVED
    }

    bool getPVT(uint16_t maxWait = 1100)
    {
        sfeStubLog().getPvtCalls++;
        if (maxWait > sfeStubLog().getPvtMaxWaitMax) {
            sfeStubLog().getPvtMaxWaitMax = maxWait;
        }
        if (packetUBXNAVPVT == NULL) {
            packetUBXNAVPVT = new UBX_NAV_PVT_t();
            memset(packetUBXNAVPVT, 0, sizeof(*packetUBXNAVPVT));
        }
        static const uint8_t pvtPoll[8] = { 0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19 };
        sendRaw(pvtPoll, sizeof(pvtPoll));
        return false;
    }

    void flushPVT(void)
    {
        sfeStubLog().flushCalls++;
        if (packetUBXNAVPVT == NULL) {
            return;
        }
        packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0;
        packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
    }

    void process(uint8_t b, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
    {
        (void)incomingUBX; (void)requestedClass; (void)requestedID;
        switch (_state) {
            case 0: if (b == 0xB5) _state = 1; return;
            case 1: _state = (b == 0x62) ? 2 : 0; return;
            case 2: _cls = b; _ckA = b; _ckB = _ckA; _state = 3; return;
            case 3: _id = b; ck(b); _state = 4; return;
            case 4: _len = b; ck(b); _state = 5; return;
            case 5:
                _len |= (uint16_t)b << 8; ck(b); _count = 0;
                _state = (_len == 0) ? 7 : (_len > sizeof(_buf) ? 0 : 6);
                return;
            case 6: _buf[_count++] = b; ck(b); if (_count >= _len) _state = 7; return;
            case 7:
                if (b != _ckA) { sfeStubLog().badChecksums++; _state = 0; return; }
                _state = 8; return;
            case 8:
                _state = 0;
                if (b != _ckB) { sfeStubLog().badChecksums++; return; }
                if (_cls == 0x01 && _id == 0x07 && _len == 92 && packetUBXNAVPVT != NULL) {
                    storePvt();
                }
                return;
        }
        _state = 0;
    }

private:
    TwoWire *_port;
    uint8_t  _addr;
    int      _state;
    uint8_t  _cls, _id;
    uint16_t _len, _count;
    uint8_t  _ckA, _ckB;
    uint8_t  _buf[100];

    void ck(uint8_t b) { _ckA = (uint8_t)(_ckA + b); _ckB = (uint8_t)(_ckB + _ckA); }

    void sendRaw(const uint8_t *p, size_t n)
    {
        _port->beginTransmission(_addr);
        for (size_t i = 0; i < n; i++) {
            _port->write(p[i]);
        }
        (void)_port->endTransmission();
    }

    uint32_t u32(int o) const
    {
        return (uint32_t)_buf[o] | ((uint32_t)_buf[o + 1] << 8) |
               ((uint32_t)_buf[o + 2] << 16) | ((uint32_t)_buf[o + 3] << 24);
    }

    void storePvt(void)
    {
        UBX_NAV_PVT_data_t &d = packetUBXNAVPVT->data;
        d.iTOW      = u32(0);
        d.fixType   = _buf[20];
        d.flags.all = _buf[21];
        d.numSV     = _buf[23];
        d.lon       = (int32_t)u32(24);
        d.lat       = (int32_t)u32(28);
        d.height    = (int32_t)u32(32);
        d.hMSL      = (int32_t)u32(36);
        d.hAcc      = u32(40);
        d.vAcc      = u32(44);
        d.gSpeed    = (int32_t)u32(60);
        d.headMot   = (int32_t)u32(64);
        d.pDOP      = (uint16_t)(_buf[76] | (_buf[77] << 8));
        packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;
        sfeStubLog().pvtParsed++;
    }
};

#endif

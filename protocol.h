#ifndef PROTOCOL_H
#define PROTOCOL_H

// ============================================================================
//  PI <-> ESP32 LINK PROTOCOL v2  --  FRAMING, CRC, STRICT PARSER, WRITER
// ============================================================================
//
// Pure C++ with no Arduino dependency, so the host-side tests compile this
// file unchanged. The full specification is in PROTOCOL.md; the short form:
//
//   FRAME   := PAYLOAD '*' CRC4 '\n'          (one optional '\r' before '\n')
//   PAYLOAD := one flat JSON object, printable ASCII only, '{' ... '}'
//   CRC4    := CRC-16/CCITT-FALSE of the PAYLOAD bytes exactly as sent,
//              as 4 hex digits, most significant nibble first
//
// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0x0000.
// Check value: CRC("123456789") = 0x29B1.
//
// The parser accepts only what the protocol defines -- integers, short
// strings, true/false/null -- and rejects everything else rather than
// guessing: fractions, exponents, leading zeros, trailing junk after a
// number, escapes, nesting, duplicate keys.
// ============================================================================

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>

#define PROTO_VERSION        2

// Sequence numbers carried by every Pi command. 0 is never valid. The Pi
// counts 1, 2, ... 65535, 1, 2, ... A seq must ADVANCE relative to the last
// one the ESP32 acknowledged: by 1..PROTO_SEQ_WINDOW steps (circularly) is
// new, 0 steps is a duplicate, anything else is stale.
#define PROTO_SEQ_MIN        1
#define PROTO_SEQ_MAX        65535
#define PROTO_SEQ_WINDOW     32767

#define PROTO_MAX_FIELDS     10     // keys per object
#define PROTO_KEY_MAX        16     // characters per key
#define PROTO_STR_MAX        32     // characters per string value
#define PROTO_TRAILER_LEN    5      // '*' + 4 hex digits


// ---------------------------------------------------------------------------
// CRC-16/CCITT-FALSE
// ---------------------------------------------------------------------------
uint16_t protoCrc16Update(uint16_t crc, uint8_t byte);
uint16_t protoCrc16(const char *data, size_t len);


// ---------------------------------------------------------------------------
// Parsed message
// ---------------------------------------------------------------------------
typedef enum {
    PV_INT = 0,
    PV_STR,
    PV_BOOL,
    PV_NULL
} ProtoValueType;

typedef struct {
    char           key[PROTO_KEY_MAX + 1];
    ProtoValueType type;
    int32_t        intValue;        // valid when type == PV_INT && !intOverflow
    bool           intOverflow;     // a well-formed integer outside int32 range
    bool           boolValue;       // valid when type == PV_BOOL
    char           str[PROTO_STR_MAX + 1];  // valid when type == PV_STR
} ProtoField;

typedef struct {
    uint8_t    count;
    ProtoField field[PROTO_MAX_FIELDS];
} ProtoMessage;

typedef enum {
    PROTO_OK = 0,
    PROTO_EMPTY,            // blank line -- ignored, usable to resynchronise
    PROTO_BAD_FRAME,        // illegal byte, bad/missing "*XXXX", no leading '{'
    PROTO_BAD_CRC,          // well-formed trailer, wrong checksum
    PROTO_BAD_SYNTAX,       // not a flat JSON object as this protocol defines it
    PROTO_BAD_NUMBER,       // malformed integer literal (150abc, 1.5, 01, 1e3, -)
    PROTO_DUPLICATE_FIELD,  // the same key twice
    PROTO_TOO_MANY_FIELDS,  // more than PROTO_MAX_FIELDS keys
    PROTO_FIELD_TOO_LONG    // key or string value longer than its limit
} ProtoStatus;

// The wire reason string for a status, e.g. PROTO_BAD_CRC -> "INVALID_CRC".
const char *protoStatusReason(ProtoStatus s);

// Validate and parse one received line. `line` excludes the '\n' terminator;
// a single trailing '\r' is tolerated. `out` is written only on PROTO_OK.
ProtoStatus protoParseFrame(const char *line, size_t len, ProtoMessage *out);

// The field with this key, or NULL.
const ProtoField *protoFindField(const ProtoMessage *msg, const char *key);

// Steps forward from `from` to `to` in the circular sequence 1..65535.
// 0 = same seq (duplicate); 1..PROTO_SEQ_WINDOW = newer; larger = stale.
// Both arguments must be in PROTO_SEQ_MIN..PROTO_SEQ_MAX.
uint16_t protoSeqDistance(uint16_t from, uint16_t to);


// ---------------------------------------------------------------------------
// Frame writer
//
// Builds one outgoing frame in a caller-supplied buffer:
//
//     protoWriterBegin(&w, buf, sizeof(buf), "ACK");   //  {"type":"ACK"
//     protoWriterAppend(&w, ",\"seq\":%u", seq);        //  ,"seq":7
//     n = protoWriterFinish(&w);                        //  }*1A2B\n
//
// Room for the closing brace, trailer and newline is always reserved, so a
// frame is either complete and correct or protoWriterFinish() returns 0. It
// never produces a truncated frame.
// ---------------------------------------------------------------------------
typedef struct {
    char   *buf;
    size_t  cap;
    size_t  len;
    bool    overflow;
} ProtoWriter;

void   protoWriterBegin(ProtoWriter *w, char *buf, size_t cap, const char *type);
void   protoWriterAppend(ProtoWriter *w, const char *fmt, ...)
           __attribute__((format(printf, 2, 3)));
void   protoWriterAppendV(ProtoWriter *w, const char *fmt, va_list ap);

// Closes the object, appends "*XXXX\n". Returns the frame length in bytes
// (including the '\n'), or 0 if the frame did not fit.
size_t protoWriterFinish(ProtoWriter *w);

#endif // PROTOCOL_H

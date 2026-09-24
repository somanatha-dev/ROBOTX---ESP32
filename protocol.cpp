#include <string.h>
#include <stdio.h>

#include "protocol.h"

// ============================================================================
// CRC-16/CCITT-FALSE  (poly 0x1021, init 0xFFFF, MSB first, no final XOR)
// ============================================================================

uint16_t protoCrc16Update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (uint16_t)((crc << 1) ^ 0x1021);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t protoCrc16(const char *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = protoCrc16Update(crc, (uint8_t)data[i]);
    }
    return crc;
}


// ============================================================================
// REASON STRINGS
// ============================================================================

const char *protoStatusReason(ProtoStatus s)
{
    switch (s) {
        case PROTO_OK:              return "NONE";
        case PROTO_EMPTY:           return "EMPTY";
        case PROTO_BAD_FRAME:       return "INVALID_FRAME";
        case PROTO_BAD_CRC:         return "INVALID_CRC";
        case PROTO_BAD_SYNTAX:      return "INVALID_MESSAGE";
        case PROTO_BAD_NUMBER:      return "MALFORMED_NUMBER";
        case PROTO_DUPLICATE_FIELD: return "DUPLICATE_FIELD";
        case PROTO_TOO_MANY_FIELDS: return "TOO_MANY_FIELDS";
        case PROTO_FIELD_TOO_LONG:  return "FIELD_TOO_LONG";
        default:                    return "INVALID_MESSAGE";
    }
}


// ============================================================================
// TOKENIZER
// ============================================================================
//
// A cursor over the payload. Every helper either consumes exactly the token it
// recognises or returns an error; none of them skips ahead looking for
// something that might match, which is what made the previous strstr parser
// accept "left":150abc as 150.
// ============================================================================

typedef struct {
    const char *p;
    const char *end;
} Cursor;

static bool isDigit(char c)  { return c >= '0' && c <= '9'; }

static bool isKeyChar(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           isDigit(c) || c == '_';
}

static int hexValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static void skipWs(Cursor *c)
{
    while (c->p < c->end && (*c->p == ' ' || *c->p == '\t')) {
        c->p++;
    }
}

// True when the cursor sits on something that may legally follow a value.
static bool atValueEnd(const Cursor *c)
{
    if (c->p >= c->end) return true;
    char ch = *c->p;
    return ch == ' ' || ch == '\t' || ch == ',' || ch == '}';
}

static ProtoStatus parseKey(Cursor *c, char *out)
{
    if (c->p >= c->end || *c->p != '"') return PROTO_BAD_SYNTAX;
    c->p++;

    size_t n = 0;
    while (c->p < c->end && *c->p != '"') {
        if (!isKeyChar(*c->p))  return PROTO_BAD_SYNTAX;
        if (n >= PROTO_KEY_MAX) return PROTO_FIELD_TOO_LONG;
        out[n++] = *c->p++;
    }
    if (c->p >= c->end || n == 0) return PROTO_BAD_SYNTAX;

    c->p++;                                   // closing quote
    out[n] = '\0';
    return PROTO_OK;
}

// String values: printable ASCII except '"' and '\'. No escapes exist in this
// protocol, so a backslash is an error rather than something to interpret.
static ProtoStatus parseString(Cursor *c, char *out)
{
    c->p++;                                   // opening quote (checked by caller)

    size_t n = 0;
    while (c->p < c->end && *c->p != '"') {
        if (*c->p == '\\')      return PROTO_BAD_SYNTAX;
        if (n >= PROTO_STR_MAX) return PROTO_FIELD_TOO_LONG;
        out[n++] = *c->p++;
    }
    if (c->p >= c->end) return PROTO_BAD_SYNTAX;

    c->p++;                                   // closing quote
    out[n] = '\0';
    return PROTO_OK;
}

// JSON integer: -?(0|[1-9][0-9]*), and it must be followed by whitespace, ','
// or '}'. A well-formed integer that does not fit int32 is NOT a syntax error
// -- it is flagged, so the command layer can reject it as OUT_OF_RANGE.
static ProtoStatus parseInteger(Cursor *c, ProtoField *f)
{
    bool negative = false;
    if (*c->p == '-') {
        negative = true;
        c->p++;
    }

    if (c->p >= c->end || !isDigit(*c->p)) return PROTO_BAD_NUMBER;

    int64_t magnitude = 0;
    bool    overflow  = false;

    if (*c->p == '0') {
        c->p++;
        if (c->p < c->end && isDigit(*c->p)) return PROTO_BAD_NUMBER;  // 01
    } else {
        while (c->p < c->end && isDigit(*c->p)) {
            if (!overflow) {
                magnitude = magnitude * 10 + (*c->p - '0');
                if (magnitude > 2147483648LL) overflow = true;
            }
            c->p++;
        }
    }

    // Catches 150abc, 1.5, 1e3 -- anything glued to the digits.
    if (!atValueEnd(c)) return PROTO_BAD_NUMBER;

    if (!negative && magnitude > 2147483647LL) overflow = true;

    f->type        = PV_INT;
    f->intOverflow = overflow;
    f->intValue    = overflow ? 0 : (int32_t)(negative ? -magnitude : magnitude);
    return PROTO_OK;
}

static bool matchLiteral(Cursor *c, const char *lit)
{
    size_t n = strlen(lit);
    if ((size_t)(c->end - c->p) < n || strncmp(c->p, lit, n) != 0) {
        return false;
    }
    c->p += n;
    return atValueEnd(c);
}

static ProtoStatus parseValue(Cursor *c, ProtoField *f)
{
    if (c->p >= c->end) return PROTO_BAD_SYNTAX;

    char ch = *c->p;

    if (ch == '"') {
        f->type = PV_STR;
        return parseString(c, f->str);
    }
    if (ch == '-' || isDigit(ch)) {
        return parseInteger(c, f);
    }
    if (ch == 't' && matchLiteral(c, "true")) {
        f->type = PV_BOOL;
        f->boolValue = true;
        return PROTO_OK;
    }
    if (ch == 'f' && matchLiteral(c, "false")) {
        f->type = PV_BOOL;
        f->boolValue = false;
        return PROTO_OK;
    }
    if (ch == 'n' && matchLiteral(c, "null")) {
        f->type = PV_NULL;
        return PROTO_OK;
    }

    // '{', '[', a bare word, '+5' ... none of them exist in this protocol.
    return PROTO_BAD_SYNTAX;
}

static ProtoStatus parseObject(const char *payload, size_t len, ProtoMessage *out)
{
    Cursor c = { payload + 1, payload + len };     // just past '{'
    out->count = 0;

    skipWs(&c);
    if (c.p < c.end && *c.p == '}') {
        c.p++;
        return (c.p == c.end) ? PROTO_OK : PROTO_BAD_SYNTAX;
    }

    for (;;) {
        skipWs(&c);

        ProtoField tmp;
        memset(&tmp, 0, sizeof(tmp));

        ProtoStatus st = parseKey(&c, tmp.key);
        if (st != PROTO_OK) return st;

        for (uint8_t i = 0; i < out->count; i++) {
            if (strcmp(out->field[i].key, tmp.key) == 0) {
                return PROTO_DUPLICATE_FIELD;
            }
        }
        if (out->count >= PROTO_MAX_FIELDS) return PROTO_TOO_MANY_FIELDS;

        skipWs(&c);
        if (c.p >= c.end || *c.p != ':') return PROTO_BAD_SYNTAX;
        c.p++;
        skipWs(&c);

        st = parseValue(&c, &tmp);
        if (st != PROTO_OK) return st;

        out->field[out->count++] = tmp;

        skipWs(&c);
        if (c.p >= c.end) return PROTO_BAD_SYNTAX;

        if (*c.p == ',') {
            c.p++;
            continue;
        }
        if (*c.p == '}') {
            c.p++;
            // The closing brace must be the last payload byte.
            return (c.p == c.end) ? PROTO_OK : PROTO_BAD_SYNTAX;
        }
        return PROTO_BAD_SYNTAX;
    }
}


// ============================================================================
// FRAME
// ============================================================================

ProtoStatus protoParseFrame(const char *line, size_t len, ProtoMessage *out)
{
    if (len > 0 && line[len - 1] == '\r') {
        len--;
    }
    if (len == 0) {
        return PROTO_EMPTY;
    }

    // Printable ASCII only. Rejects NUL, stray CR, and line noise outright.
    for (size_t i = 0; i < len; i++) {
        uint8_t b = (uint8_t)line[i];
        if (b < 0x20 || b > 0x7E) {
            return PROTO_BAD_FRAME;
        }
    }

    // Shortest possible frame is "{}*XXXX".
    if (len < 2 + PROTO_TRAILER_LEN) {
        return PROTO_BAD_FRAME;
    }

    const char *trailer = line + len - PROTO_TRAILER_LEN;
    if (trailer[0] != '*') {
        return PROTO_BAD_FRAME;
    }

    uint16_t rxCrc = 0;
    for (uint8_t i = 1; i < PROTO_TRAILER_LEN; i++) {
        int v = hexValue(trailer[i]);
        if (v < 0) {
            return PROTO_BAD_FRAME;
        }
        rxCrc = (uint16_t)((rxCrc << 4) | (uint16_t)v);
    }

    // Anything before '{' -- boot noise, a half-received earlier frame -- makes
    // the whole line invalid. It is never skipped over to find a command.
    if (line[0] != '{') {
        return PROTO_BAD_FRAME;
    }

    size_t payloadLen = len - PROTO_TRAILER_LEN;
    if (protoCrc16(line, payloadLen) != rxCrc) {
        return PROTO_BAD_CRC;
    }

    return parseObject(line, payloadLen, out);
}

const ProtoField *protoFindField(const ProtoMessage *msg, const char *key)
{
    for (uint8_t i = 0; i < msg->count; i++) {
        if (strcmp(msg->field[i].key, key) == 0) {
            return &msg->field[i];
        }
    }
    return NULL;
}


uint16_t protoSeqDistance(uint16_t from, uint16_t to)
{
    // 65535 values (0 is excluded), so the circle has 65535 positions.
    return (uint16_t)(((uint32_t)to + PROTO_SEQ_MAX - from) % PROTO_SEQ_MAX);
}


// ============================================================================
// WRITER
// ============================================================================
//
// Reserve at the end of the buffer: '}' + "*XXXX" + '\n' + NUL = 8 bytes.
// ============================================================================

#define WRITER_RESERVE 8

void protoWriterBegin(ProtoWriter *w, char *buf, size_t cap, const char *type)
{
    w->buf      = buf;
    w->cap      = cap;
    w->len      = 0;
    w->overflow = (cap <= WRITER_RESERVE);
    protoWriterAppend(w, "{\"type\":\"%s\"", type);
}

void protoWriterAppendV(ProtoWriter *w, const char *fmt, va_list ap)
{
    if (w->overflow) {
        return;
    }

    size_t room = w->cap - w->len - (WRITER_RESERVE - 1);  // incl. vsnprintf NUL
    int n = vsnprintf(w->buf + w->len, room, fmt, ap);

    if (n < 0 || (size_t)n >= room) {
        w->overflow = true;
        return;
    }
    w->len += (size_t)n;
}

void protoWriterAppend(ProtoWriter *w, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    protoWriterAppendV(w, fmt, ap);
    va_end(ap);
}

size_t protoWriterFinish(ProtoWriter *w)
{
    if (w->overflow) {
        return 0;
    }

    w->buf[w->len++] = '}';

    uint16_t crc = protoCrc16(w->buf, w->len);
    snprintf(w->buf + w->len, 7, "*%04X\n", (unsigned)crc);
    w->len += 6;

    return w->len;
}

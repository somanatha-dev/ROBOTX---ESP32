// ============================================================================
//  Unit tests for protocol.cpp -- CRC, framing, strict parser, writer.
//  Compiled on the host against the unmodified protocol.cpp.
// ============================================================================

#include <stdio.h>
#include <string.h>

#include "protocol.h"

static int gFail = 0;
static int gPass = 0;

#define CHECK(cond, ...) do {                                   \
        if (cond) { gPass++; }                                  \
        else { gFail++; printf("FAIL %s:%d  ", __FILE__, __LINE__); \
               printf(__VA_ARGS__); printf("\n"); }             \
    } while (0)

// Build "<payload>*XXXX" with a correct CRC (no newline).
static const char *framed(const char *payload)
{
    static char buf[512];
    snprintf(buf, sizeof(buf), "%s*%04X", payload,
             (unsigned)protoCrc16(payload, strlen(payload)));
    return buf;
}

static ProtoStatus parse(const char *line, ProtoMessage *m)
{
    return protoParseFrame(line, strlen(line), m);
}

static ProtoStatus parsePayload(const char *payload, ProtoMessage *m)
{
    return parse(framed(payload), m);
}

static void testCrc(void)
{
    CHECK(protoCrc16("123456789", 9) == 0x29B1,
          "CRC-16/CCITT-FALSE check value, got %04X",
          (unsigned)protoCrc16("123456789", 9));
    CHECK(protoCrc16("", 0) == 0xFFFF, "empty input = init value");

    // Incremental and one-shot agree.
    const char *s = "{\"type\":\"COMMAND\",\"seq\":1,\"cmd\":\"PING\"}";
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < strlen(s); i++) c = protoCrc16Update(c, (uint8_t)s[i]);
    CHECK(c == protoCrc16(s, strlen(s)), "incremental CRC");
}

static void testFraming(void)
{
    ProtoMessage m;
    const char *ok = "{\"type\":\"COMMAND\",\"seq\":1,\"cmd\":\"PING\"}";

    CHECK(parsePayload(ok, &m) == PROTO_OK, "valid frame");
    CHECK(m.count == 3, "three fields");

    // Trailing CR tolerated, lowercase hex accepted.
    char line[256];
    snprintf(line, sizeof(line), "%s\r", framed(ok));
    CHECK(parse(line, &m) == PROTO_OK, "trailing CR");
    snprintf(line, sizeof(line), "%s*%04x", ok, (unsigned)protoCrc16(ok, strlen(ok)));
    CHECK(parse(line, &m) == PROTO_OK, "lowercase CRC hex");

    CHECK(parse("", &m)   == PROTO_EMPTY, "empty");
    CHECK(parse("\r", &m) == PROTO_EMPTY, "CR only");

    CHECK(parse(ok, &m) == PROTO_BAD_FRAME, "no trailer");
    snprintf(line, sizeof(line), "%s*12", ok);
    CHECK(parse(line, &m) == PROTO_BAD_FRAME, "short trailer");
    snprintf(line, sizeof(line), "%s*12G4", ok);
    CHECK(parse(line, &m) == PROTO_BAD_FRAME, "non-hex trailer");
    snprintf(line, sizeof(line), "%s*0000", ok);
    CHECK(parse(line, &m) == PROTO_BAD_CRC, "wrong CRC");

    // Garbage before or after a valid frame invalidates the whole line.
    snprintf(line, sizeof(line), "xx%s", framed(ok));
    CHECK(parse(line, &m) == PROTO_BAD_FRAME, "prefix garbage");
    snprintf(line, sizeof(line), "%szz", framed(ok));
    CHECK(parse(line, &m) == PROTO_BAD_FRAME, "suffix garbage");
    snprintf(line, sizeof(line), " %s", framed(ok));
    CHECK(parse(line, &m) == PROTO_BAD_FRAME, "leading space");

    // Non-printable bytes.
    char bin[64];
    memcpy(bin, "{\x01}*0000", 8);
    CHECK(protoParseFrame(bin, 8, &m) == PROTO_BAD_FRAME, "control byte");
    memcpy(bin, "{}\0*0000", 8);
    CHECK(protoParseFrame(bin, 8, &m) == PROTO_BAD_FRAME, "NUL byte");
    memcpy(bin, "{}\xff*0000", 8);
    CHECK(protoParseFrame(bin, 8, &m) == PROTO_BAD_FRAME, "high byte");
    memcpy(bin, "{\r}*0000", 8);
    CHECK(protoParseFrame(bin, 8, &m) == PROTO_BAD_FRAME, "embedded CR");

    CHECK(parse(framed("{}"), &m) == PROTO_OK && m.count == 0, "empty object");
}

static void testNumbers(void)
{
    ProtoMessage m;
    const ProtoField *f;

    CHECK(parsePayload("{\"a\":150}", &m) == PROTO_OK, "int");
    f = protoFindField(&m, "a");
    CHECK(f && f->type == PV_INT && f->intValue == 150 && !f->intOverflow, "150");

    CHECK(parsePayload("{\"a\":-255}", &m) == PROTO_OK &&
          m.field[0].intValue == -255, "-255");
    CHECK(parsePayload("{\"a\":0}", &m) == PROTO_OK && m.field[0].intValue == 0, "0");
    CHECK(parsePayload("{\"a\":-0}", &m) == PROTO_OK && m.field[0].intValue == 0, "-0");
    CHECK(parsePayload("{\"a\":2147483647}", &m) == PROTO_OK &&
          !m.field[0].intOverflow && m.field[0].intValue == 2147483647, "INT32_MAX");
    CHECK(parsePayload("{\"a\":-2147483648}", &m) == PROTO_OK &&
          !m.field[0].intOverflow, "INT32_MIN");
    CHECK(parsePayload("{\"a\":2147483648}", &m) == PROTO_OK &&
          m.field[0].intOverflow, "INT32_MAX+1 flagged");
    CHECK(parsePayload("{\"a\":99999999999999999999}", &m) == PROTO_OK &&
          m.field[0].intOverflow, "huge flagged, not wrapped");

    CHECK(parsePayload("{\"a\":150abc}", &m) == PROTO_BAD_NUMBER, "150abc");
    CHECK(parsePayload("{\"a\":1.5}", &m)    == PROTO_BAD_NUMBER, "1.5");
    CHECK(parsePayload("{\"a\":1e3}", &m)    == PROTO_BAD_NUMBER, "1e3");
    CHECK(parsePayload("{\"a\":01}", &m)     == PROTO_BAD_NUMBER, "01");
    CHECK(parsePayload("{\"a\":-}", &m)      == PROTO_BAD_NUMBER, "-");
    CHECK(parsePayload("{\"a\":--1}", &m)    == PROTO_BAD_NUMBER, "--1");
    CHECK(parsePayload("{\"a\":0x10}", &m)   == PROTO_BAD_NUMBER, "0x10");
    CHECK(parsePayload("{\"a\":+5}", &m)     == PROTO_BAD_SYNTAX, "+5");
}

static void testStructure(void)
{
    ProtoMessage m;

    CHECK(parsePayload("{\"a\":\"x\",\"b\":true,\"c\":false,\"d\":null}", &m) == PROTO_OK
          && m.count == 4, "all value kinds");
    CHECK(m.field[0].type == PV_STR && strcmp(m.field[0].str, "x") == 0, "string");
    CHECK(m.field[1].type == PV_BOOL && m.field[1].boolValue, "true");
    CHECK(m.field[2].type == PV_BOOL && !m.field[2].boolValue, "false");
    CHECK(m.field[3].type == PV_NULL, "null");

    CHECK(parsePayload("{ \"a\" : 1 , \"b\" : 2 }", &m) == PROTO_OK, "whitespace");
    CHECK(parsePayload("{\"a\":1,\"a\":2}", &m) == PROTO_DUPLICATE_FIELD, "duplicate key");
    CHECK(parsePayload("{\"a\":{\"b\":1}}", &m) == PROTO_BAD_SYNTAX, "nested object");
    CHECK(parsePayload("{\"a\":[1]}", &m)       == PROTO_BAD_SYNTAX, "array");
    CHECK(parsePayload("{\"a\":1,}", &m)        == PROTO_BAD_SYNTAX, "trailing comma");
    CHECK(parsePayload("{\"a\" 1}", &m)         == PROTO_BAD_SYNTAX, "missing colon");
    CHECK(parsePayload("{\"a\":1 \"b\":2}", &m) == PROTO_BAD_SYNTAX, "missing comma");
    CHECK(parsePayload("{\"a\":1} ", &m)        == PROTO_BAD_SYNTAX, "junk after brace");
    CHECK(parsePayload("{\"a\":1}}", &m)        == PROTO_BAD_SYNTAX, "double brace");
    CHECK(parsePayload("{\"a\":1", &m)          == PROTO_BAD_SYNTAX, "no closing brace");
    CHECK(parsePayload("{a:1}", &m)             == PROTO_BAD_SYNTAX, "unquoted key");
    CHECK(parsePayload("{\"a-b\":1}", &m)       == PROTO_BAD_SYNTAX, "bad key char");
    CHECK(parsePayload("{\"\":1}", &m)          == PROTO_BAD_SYNTAX, "empty key");
    CHECK(parsePayload("{\"a\":\"x\\\"y\"}", &m) == PROTO_BAD_SYNTAX, "escape rejected");
    CHECK(parsePayload("{\"a\":tru}", &m)       == PROTO_BAD_SYNTAX, "bad literal");
    CHECK(parsePayload("{\"a\":truex}", &m)     == PROTO_BAD_SYNTAX, "literal + junk");
    CHECK(parsePayload("{\"a\":\"unterminated}", &m) == PROTO_BAD_SYNTAX, "open string");

    CHECK(parsePayload("{\"abcdefghijklmnop\":1}", &m) == PROTO_OK, "16-char key");
    CHECK(parsePayload("{\"abcdefghijklmnopq\":1}", &m) == PROTO_FIELD_TOO_LONG, "17-char key");
    CHECK(parsePayload("{\"a\":\"0123456789012345678901234567890X\"}", &m) == PROTO_OK,
          "32-char string");
    CHECK(parsePayload("{\"a\":\"0123456789012345678901234567890XY\"}", &m) ==
          PROTO_FIELD_TOO_LONG, "33-char string");

    CHECK(parsePayload("{\"a\":1,\"b\":1,\"c\":1,\"d\":1,\"e\":1,\"f\":1,\"g\":1,"
                       "\"h\":1,\"i\":1,\"j\":1}", &m) == PROTO_OK, "10 fields");
    CHECK(parsePayload("{\"a\":1,\"b\":1,\"c\":1,\"d\":1,\"e\":1,\"f\":1,\"g\":1,"
                       "\"h\":1,\"i\":1,\"j\":1,\"k\":1}", &m) == PROTO_TOO_MANY_FIELDS,
          "11 fields");
}

static void testSeqDistance(void)
{
    CHECK(protoSeqDistance(10, 10) == 0, "same seq");
    CHECK(protoSeqDistance(10, 11) == 1, "next");
    CHECK(protoSeqDistance(65535, 1) == 1, "wrap 65535 -> 1");
    CHECK(protoSeqDistance(65534, 2) == 3, "wrap across");
    CHECK(protoSeqDistance(1, 65535) == 65534, "1 -> 65535 is behind");
    CHECK(protoSeqDistance(11, 10) > PROTO_SEQ_WINDOW, "one behind is stale");
    CHECK(protoSeqDistance(1, 1 + PROTO_SEQ_WINDOW) == PROTO_SEQ_WINDOW, "max jump is new");
    CHECK(protoSeqDistance(1, 2 + PROTO_SEQ_WINDOW) > PROTO_SEQ_WINDOW, "max jump + 1 is stale");
}

static void testWriter(void)
{
    char buf[64];
    ProtoWriter w;

    protoWriterBegin(&w, buf, sizeof(buf), "ACK");
    protoWriterAppend(&w, ",\"seq\":%u", 7u);
    size_t n = protoWriterFinish(&w);
    CHECK(n > 0 && buf[n - 1] == '\n', "writer produces a line");

    // The writer's output must parse back with a valid CRC.
    ProtoMessage m;
    CHECK(protoParseFrame(buf, n - 1, &m) == PROTO_OK, "writer output round-trips");
    const ProtoField *t = protoFindField(&m, "type");
    CHECK(t && strcmp(t->str, "ACK") == 0, "type first");

    // Overflow is all-or-nothing.
    protoWriterBegin(&w, buf, sizeof(buf), "ACK");
    protoWriterAppend(&w, ",\"x\":\"%s\"", "0123456789012345678901234567890123456789012345");
    CHECK(protoWriterFinish(&w) == 0, "overflow returns 0");

    // Exactly-full boundary: fill to the last usable byte.
    char big[32];
    for (size_t k = 0; k < 40; k++) {
        protoWriterBegin(&w, big, sizeof(big), "E");
        char pad[40];
        memset(pad, 'a', k); pad[k] = '\0';
        protoWriterAppend(&w, "%s", pad);
        size_t len = protoWriterFinish(&w);
        if (len > 0) {
            CHECK(len < sizeof(big) && protoParseFrame(big, len - 1, &m) != PROTO_BAD_CRC,
                  "boundary k=%u", (unsigned)k);
        }
    }
}

int main(void)
{
    testCrc();
    testFraming();
    testNumbers();
    testStructure();
    testSeqDistance();
    testWriter();

    printf("protocol unit tests: %d passed, %d failed\n", gPass, gFail);
    return gFail ? 1 : 0;
}

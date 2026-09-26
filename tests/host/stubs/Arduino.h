#ifndef SIM_ARDUINO_H
#define SIM_ARDUINO_H

// ============================================================================
//  HOST SIMULATION STUB -- stands in for the Arduino-ESP32 core.
//
//  Only what the firmware actually calls. Timing is real wall-clock time, so
//  the watchdog and telemetry cadence behave as they do on the ESP32. The
//  UART is the process's stdin/stdout. See tests/host/sim_hw.cpp.
// ============================================================================

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define HIGH   1
#define LOW    0
#define INPUT  0
#define OUTPUT 1

uint32_t      millis(void);
uint32_t      micros(void);
void          delay(uint32_t ms);
void          delayMicroseconds(uint32_t us);
void          pinMode(uint8_t pin, uint8_t mode);
void          digitalWrite(uint8_t pin, uint8_t value);
int           digitalRead(uint8_t pin);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeoutUs);

class SimSerial {
public:
    void   begin(unsigned long baud)      { (void)baud; }
    size_t setTxBufferSize(size_t n)      { return n; }
    size_t setRxBufferSize(size_t n)      { return n; }
    int    available(void);
    int    read(void);
    size_t write(const uint8_t *buf, size_t len);
    size_t write(uint8_t b)               { return write(&b, 1); }
};

extern SimSerial Serial;

#endif

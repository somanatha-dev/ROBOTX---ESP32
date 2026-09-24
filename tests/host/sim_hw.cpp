// ============================================================================
//  HOST SIMULATION -- hardware below the firmware.
//
//  The firmware above this file is REAL: robot_controller.ino, comm.cpp,
//  protocol.cpp, motor.cpp, safety.cpp, rear_tof.cpp, tca9548a.cpp and
//  rover_i2c.cpp are compiled unchanged. Only these are simulated:
//
//    * Arduino core  -- millis()/delay() on the wall clock, GPIO as no-ops
//    * UART0         -- stdin (RX) and stdout (TX), raw bytes
//    * HC-SR04 echo  -- pulseIn() returns the echo for SIM_FRONT_CM
//    * I2C bus       -- ACKs at 0x40, 0x42 and 0x70; 0x70 echoes its last
//                       write the way a TCA9548A does; everything else NACKs
//    * PCA9685       -- replaced entirely (pca9685.cpp is NOT compiled), so
//                       "drive available" can be simulated with SIM_DRIVE_AVAILABLE=1
//                       without changing config.h. No motor exists here.
//
//  Environment variables:
//    SIM_DRIVE_AVAILABLE  0 (default, matches the rover today) or 1
//    SIM_FRONT_CM         simulated front distance in cm (default 150)
//    SIM_ROM_NOISE        1 = print ESP32-boot-ROM-style text before setup()
// ============================================================================

#include <windows.h>
#include <mmsystem.h>
#include <io.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Arduino.h"
#include "Wire.h"
#include "pca9685.h"

void setup(void);
void loop(void);

static DWORD  gStartMs       = 0;
static bool   gDriveAvailable = false;
static double gFrontCm       = 150.0;

// ---------------------------------------------------------------------------
// Arduino core
// ---------------------------------------------------------------------------

uint32_t millis(void)                         { return (uint32_t)(timeGetTime() - gStartMs); }
void     delay(uint32_t ms)                   { Sleep(ms); }
void     delayMicroseconds(uint32_t us)       { (void)us; }
void     pinMode(uint8_t pin, uint8_t mode)   { (void)pin; (void)mode; }
void     digitalWrite(uint8_t pin, uint8_t v) { (void)pin; (void)v; }
int      digitalRead(uint8_t pin)             { (void)pin; return HIGH; }

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeoutUs)
{
    (void)pin; (void)state;
    if (gFrontCm <= 0.0) {
        return 0;                                   // no echo
    }
    unsigned long us = (unsigned long)(gFrontCm / 0.01715);
    return (us > timeoutUs) ? 0 : us;
}

// ---------------------------------------------------------------------------
// UART0 = stdin / stdout
// ---------------------------------------------------------------------------

SimSerial Serial;

static uint8_t gRx[4096];
static size_t  gRxHead = 0;
static size_t  gRxTail = 0;

static void pumpStdin(void)
{
    if (gRxHead != gRxTail) {
        return;
    }
    gRxHead = gRxTail = 0;

    HANDLE h = GetStdHandle(STD_INPUT_HANDLE);
    DWORD avail = 0;
    if (!PeekNamedPipe(h, NULL, 0, NULL, &avail, NULL)) {
        exit(0);                                    // test harness went away
    }
    if (avail == 0) {
        return;
    }
    if (avail > sizeof(gRx)) {
        avail = sizeof(gRx);
    }
    DWORD got = 0;
    if (!ReadFile(h, gRx, avail, &got, NULL) || got == 0) {
        exit(0);
    }
    gRxTail = got;
}

int SimSerial::available(void)
{
    pumpStdin();
    return (int)(gRxTail - gRxHead);
}

int SimSerial::read(void)
{
    pumpStdin();
    if (gRxHead == gRxTail) {
        return -1;
    }
    return gRx[gRxHead++];
}

size_t SimSerial::write(const uint8_t *buf, size_t len)
{
    if (fwrite(buf, 1, len, stdout) != len || fflush(stdout) != 0) {
        exit(0);
    }
    return len;
}

// ---------------------------------------------------------------------------
// I2C bus
// ---------------------------------------------------------------------------

TwoWire Wire;

static int     gTxAddr     = -1;
static uint8_t gTcaMask    = 0;
static int     gRxAvail    = 0;
static uint8_t gRxByte     = 0;
static uint8_t gLastWrite  = 0;
static bool    gWroteByte  = false;

static bool devicePresent(int addr)
{
    return addr == 0x40 || addr == 0x42 || addr == 0x70;
}

bool TwoWire::begin(int sda, int scl, uint32_t freq)
{
    (void)sda; (void)scl; (void)freq;
    return true;
}

void TwoWire::beginTransmission(int address)
{
    gTxAddr    = address;
    gWroteByte = false;
}

size_t TwoWire::write(uint8_t b)
{
    gLastWrite = b;
    gWroteByte = true;
    return 1;
}

uint8_t TwoWire::endTransmission(bool sendStop)
{
    (void)sendStop;
    if (!devicePresent(gTxAddr)) {
        return 2;                                   // address NACK
    }
    if (gTxAddr == 0x70 && gWroteByte) {
        gTcaMask = gLastWrite;                      // TCA9548A control register
    }
    return 0;
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
    if (!devicePresent(address) || quantity < 1) {
        gRxAvail = 0;
        return 0;
    }
    gRxByte  = (address == 0x70) ? gTcaMask : 0x00;
    gRxAvail = 1;
    return 1;
}

int TwoWire::available(void) { return gRxAvail; }

int TwoWire::read(void)
{
    if (gRxAvail <= 0) {
        return -1;
    }
    gRxAvail--;
    return gRxByte;
}

// ---------------------------------------------------------------------------
// PCA9685 (replaces pca9685.cpp)
// ---------------------------------------------------------------------------

bool        pca9685Init(void)             { return gDriveAvailable; }
bool        pca9685AddressConfirmed(void) { return gDriveAvailable; }
uint8_t     pca9685Address(void)          { return 0x40; }
bool        pca9685Ready(void)            { return gDriveAvailable; }
const char *pca9685StatusName(void)       { return gDriveAvailable ? "OK" : "ADDRESS_UNCONFIRMED"; }
bool        pca9685SetDuty(uint8_t ch, uint16_t duty) { (void)ch; (void)duty; return gDriveAvailable; }
bool        pca9685AllOff(void)           { return gDriveAvailable; }

bool pca9685ProbeCandidate(uint8_t address, uint8_t *mode1Out, uint8_t *prescaleOut)
{
    if (mode1Out)    *mode1Out    = 0;
    if (prescaleOut) *prescaleOut = 0;
    if (address != 0x40) {
        return false;
    }
    if (mode1Out)    *mode1Out    = 0x11;           // power-on MODE1
    if (prescaleOut) *prescaleOut = 0x1E;           // power-on PRESCALE
    return true;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(void)
{
    _setmode(_fileno(stdin),  _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);

    timeBeginPeriod(1);
    gStartMs = timeGetTime();

    const char *e;
    if ((e = getenv("SIM_DRIVE_AVAILABLE")) != NULL) gDriveAvailable = (atoi(e) != 0);
    if ((e = getenv("SIM_FRONT_CM"))        != NULL) gFrontCm        = atof(e);

    if ((e = getenv("SIM_ROM_NOISE")) != NULL && atoi(e) != 0) {
        static const char kRom[] =
            "ets Jun  8 2016 00:22:57\r\n\r\n"
            "rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\r\n"
            "configsip: 0, SPIWP:0xee\r\n"
            "mode:DIO, clock div:1\r\n"
            "load:0x3fff0030,len:1344\r\n"
            "entry 0x400805f0\r\n";
        Serial.write((const uint8_t *)kRom, sizeof(kRom) - 1);
    }

    setup();
    for (;;) {
        loop();
    }
}

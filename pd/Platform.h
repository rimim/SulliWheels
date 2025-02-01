// Copyright 2025 Mimir Reynisson
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the “Software”),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>
#include <assert.h>
#include <algorithm>
#include <math.h>
#endif
#include <stdarg.h>

#ifndef ARDUINO
#include <string>
namespace pd {
    typedef std::string String;
}

inline pd::String toLower(const pd::String& str) {
    pd::String lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    return lowerStr;
}
#else
namespace pd {
    typedef String String;
    using PrintStream=::Print;
}

inline pd::String toLower(pd::String& str) {
    return str.toLowerCase();
}
#endif

namespace pd {
    template<class T, class L> 
    auto min(const T& a, const L& b) -> decltype((b < a) ? b : a) {
        return (b < a) ? b : a;
    }

    template<class T, class L> 
    auto max(const T& a, const L& b) -> decltype((b < a) ? b : a) {
        return (a < b) ? b : a;
    }
}

#ifndef SizeOfArray
#define SizeOfArray(arr) (sizeof(arr)/sizeof(arr[0]))
#endif

namespace pd {
    enum BusType {
        UnknownBus,
        TTL,
        RS485,
        CAN,
        I2C
    };
#ifdef ARDUINO
    typedef struct StreamProxy* Stream;
    static constexpr Stream STREAM_INVALID = nullptr;
#else
    typedef int Stream;
    static constexpr Stream STREAM_INVALID = -1;
#endif

    enum MotorType {
        UnknownMotor,
        Go1,
        Go2,
        UkiA1,
        UkiB1,
        CyberGear,
        Dynamixel,
        Servo
    };

    uint16_t crc(const void* bytes, size_t len, uint8_t lockMask = 0, uint8_t high = 0, uint8_t low = 0);
    uint32_t crc32(const void* bytes, size_t len, uint32_t crc = uint32_t(~0));

    template<class T>
    uint16_t hash(const T& a) {
        return crc(&a, sizeof(a));
    }

    inline uint16_t hash(const String& str) {
        const char* cstr = str.c_str();
        return crc(cstr, strlen(cstr));
    }

    enum PinMode {
        PIN_MODE_INPUT = 1,
        PIN_MODE_OUTPUT = 2,
        PIN_MODE_INPUT_PULLUP = 3,
        PIN_MODE_INPUT_DISABLE = 4
    };
}

namespace pd::platform {

using Stream = pd::Stream;
using BusType = pd::BusType;
using PinMode = pd::PinMode;

// Platform initialization
void init();

uint64_t currentTimeMillis();
uint64_t currentTimeMicros();

void usleep(uint32_t micros);
void reboot();

void pinModeOutput(uint8_t pin);
void pinMode(uint8_t pin, pd::PinMode mode);
void digitalWrite(uint8_t pin, uint8_t val);
uint8_t digitalRead(uint8_t pin);

long random(long max, long min = 0); //range : [min, max)

Stream openStream(const char* adapter, BusType type, unsigned baud, int readtimeout = -1);
pd::String getAdapterName(Stream stream);
bool setStreamReadTimeout(Stream stream, uint32_t centiseconds, int minAvailable = 0);
void vprintStream(Stream stream, const char* str, va_list args);
void printStream(Stream stream, const char* str, ...);
void closeStream(Stream& stream);

void setLEDState(unsigned mask);
bool available(Stream stream);
bool setSpeed(Stream stream, uint32_t baud);
ssize_t write(Stream stream, const void* buffer, size_t size, uint32_t id = 0, bool ext = false);
ssize_t read(Stream stream, void* buffer, size_t size, uint32_t* id = nullptr);
inline int read(Stream stream) {
    uint8_t buf;
    if (read(stream, &buf, 1) == 1) {
        return buf;
    }
    return -1;
}

}

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

#include "pin-map.h"
#include <Wire.h>
#include "pd/StreamProxy.h"
#ifdef GPIO_EXPANDER_ADDRESS
#include "PCF8574.h"
#endif

using namespace pd;

void pd::platform::init() {
}

#ifdef GPIO_EXPANDER_ADDRESS
static PCF8574 GPIOExpander(GPIO_EXPANDER_ADDRESS);
#endif

void pd::platform::digitalWrite(uint8_t pin, uint8_t val) {
#ifdef GPIO_EXPANDER_ADDRESS
    if (pin >= GPIO_PIN_BASE) {
        if (!GPIOExpander.digitalWrite(pin-GPIO_PIN_BASE, val)) {
            PDLOG_ERROR("digitalWrite failed: pin%d val=%d\n", pin, val);
        }
        return;
    }
#endif
    ::digitalWrite(pin, val);
}

uint8_t pd::platform::digitalRead(uint8_t pin) {
#ifdef GPIO_EXPANDER_ADDRESS
    if (pin >= GPIO_PIN_BASE) {
        return 0;
    }
#endif
    return ::digitalRead(pin);
}

void pd::platform::pinMode(uint8_t pin, pd::PinMode pdmode) {
    uint8_t mode;
    switch (pdmode) {
        case pd::PIN_MODE_INPUT:
            mode = INPUT;
            break;
        case pd::PIN_MODE_OUTPUT:
            mode = OUTPUT;
            break;
        case pd::PIN_MODE_INPUT_PULLUP:
            mode = INPUT_PULLUP;
            break;
        case pd::PIN_MODE_INPUT_DISABLE:
        #ifndef INPUT_DISABLE
            return;
        #endif
            mode = INPUT_DISABLE;
            break;
        default:
            // UNSUPPORTED
            return;
    }
#ifdef GPIO_EXPANDER_ADDRESS
    if (pin >= GPIO_PIN_BASE) {
        GPIOExpander.pinMode(pin-GPIO_PIN_BASE, mode);
    }
#endif
    ::pinMode(pin, mode);
}

void pd::platform::pinModeOutput(uint8_t pin) {
    pd::platform::pinMode(pin, pd::PIN_MODE_OUTPUT);
}

ssize_t
pd::platform::write(pd::platform::Stream stream, const void* buffer, size_t size, uint32_t id, bool ext) {
    size_t result = 0;
    if (stream != STREAM_INVALID) {
        result = stream->write((const char*)buffer, size, id, ext);
    }
    return result;
}

ssize_t
pd::platform::read(pd::platform::Stream stream, void* buffer, size_t size, uint32_t* id) {
    ssize_t result = -1;
    if (stream != STREAM_INVALID) {
        result = stream->read((char*)buffer, size, id);
    }
    return result;
}

bool
pd::platform::available(pd::platform::Stream stream) {
    if (stream != STREAM_INVALID) {
        return stream->available();
    }
    return false;
}

String
pd::platform::getAdapterName(pd::platform::Stream stream) {
    if (stream != STREAM_INVALID) {
        return stream->getName().c_str();
    }
    return "<invalid>";
}

void
pd::platform::vprintStream(Stream stream, const char* str, va_list args) {
    char buf[1024];
    vsnprintf(buf, sizeof(buf), str, args);
    write(stream, buf, strlen(buf));
}

void
pd::platform::printStream(Stream stream, const char* str, ...) {
    va_list args;
    va_start(args, str);
    vprintStream(stream, str, args);
    va_end(args);
}

void
pd::platform::closeStream(Stream &stream) {
    if (stream != STREAM_INVALID) {
        stream->close();
    }
}

pd::platform::Stream
pd::platform::openStream(const char* adapter, BusType type, unsigned baud, int readtimeout) {
    pd::platform::Stream stream = StreamProxy::getByName(adapter);
    if (stream == nullptr) {
        return STREAM_INVALID;
    }
    stream->begin(baud);
    if (readtimeout != -1) {
        stream->setTimeout(readtimeout);
    }
    return stream;
}

bool setStreamReadTimeout(pd::platform::Stream stream, uint32_t centiseconds, int minAvailable = 0) {
    if (stream != STREAM_INVALID) {
        stream->setTimeout(centiseconds);
        return true;
    }
    return false;
}

uint64_t
pd::platform::currentTimeMicros() {
    return micros();
}

uint64_t
pd::platform::currentTimeMillis() {
    return millis();
}

long
pd::platform::random(long min, long max) {
    return ::random(min, max);
}

void
pd::platform::usleep(uint32_t micros) {
    delayMicroseconds(micros);
}

void
pd::platform::reboot() {
    #define RESTART_ADDR 0xE000ED0C
    #define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
    #define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

    WRITE_RESTART(0x5FA0004);

    #undef RESTART_ADDR
    #undef READ_RESTART
    #undef WRITE_RESTART
}

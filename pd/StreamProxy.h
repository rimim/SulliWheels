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

#include <Arduino.h>
#include <Wire.h>
#include "Platform.h"
#include "Log.h"
#include <FlexCAN_T4.h>

namespace pd {
class StreamProxy {
public:
    StreamProxy(String name) :
        fName(name),
        fNext(nullptr)
    {
        if (*head() == NULL)
            *head() = this;
        if (*tail() != NULL)
            (*tail())->fNext = this;
        *tail() = this;
    }

    String getName() const {
        return fName;
    }

    static pd::platform::Stream getByName(const char* name) {
        for (StreamProxy* stream = *head(); stream != NULL; stream = stream->fNext) {
            if (stream->getName() == name) {
                return stream;
            }
        }
        return nullptr;
    }

    virtual void begin(unsigned baud) = 0;
    virtual void close() {}
    virtual void setTimeout(int centiseconds) = 0;
    virtual size_t write(const void* buffer, size_t size, uint32_t id, bool ext) = 0;
    virtual size_t read(char* buffer, size_t size, uint32_t* id) = 0;
    virtual bool available() = 0;

protected:
    String fName;
    StreamProxy* fNext;

private:
    static StreamProxy** head()
    {
        static StreamProxy* sHead;
        return &sHead;
    }

    static StreamProxy** tail()
    {
        static StreamProxy* sTail;
        return &sTail;
    }
};

FCTP_CLASS class FlexCANStreamProxy: public StreamProxy {
public:
    FlexCANStreamProxy(String name, int rtsPin = -1, int rtsLEDPin = -1) :
        StreamProxy(name),
        fRTS(rtsPin),
        fRTSLED(rtsLEDPin)
    {
    }

    virtual void begin(unsigned baud) override {
        fCAN.begin();
        fCAN.setBaudRate(baud);
    }

    virtual void setTimeout(int centiseconds) override {
        // TODO
    }

    virtual size_t write(const void* buffer, size_t size, uint32_t id, bool ext) override {
        CAN_message_t msg;
        if (size != sizeof(msg.buf)) {
            PDLOG_ERROR("invalid CAN message sizen\n");
            return 0;
        }
        msg.id = id;
        msg.flags.extended = ext;
        memcpy(msg.buf, buffer, sizeof(msg.buf));
        if (pd::Log::isVerboseCAN()) {
            PDLOG_INFO("[CAN:W id=0x%08X ext=%d] ", id, ext);
            for (unsigned i = 0; i < sizeof(msg.buf); i++) {
                PDLOG_INFO("%02X ", ((uint8_t*)msg.buf)[i]);
            }
            PDLOG_INFO("\n");
        }
        if (fRTS != -1)
            pd::platform::digitalWrite(fRTS, true);
        if (fRTSLED != -1)
            pd::platform::digitalWrite(fRTSLED, true);
        size_t result = (fCAN.write(msg) == 1) ? size : 0;
        if (fRTS != -1)
            pd::platform::digitalWrite(fRTS, false);
        if (fRTSLED != -1)
            pd::platform::digitalWrite(fRTSLED, false);
        delayMicroseconds(20);
        return result;
    }

    virtual size_t read(char* buffer, size_t size, uint32_t* id) override {
        CAN_message_t msg;
        if (fCAN.read(msg)) {
            if (pd::Log::isVerboseCAN()) {
                PDLOG_INFO("[CAN:R id=0x%08X] ", msg.id);
                for (unsigned i = 0; i < sizeof(msg.buf); i++) {
                    PDLOG_INFO("%02X ", ((uint8_t*)msg.buf)[i]);
                }
                PDLOG_INFO("\n");
            }
            memcpy(buffer, msg.buf, size);
            if (id != nullptr)
                *id = msg.id;
            return msg.len;
        }
        if (id != nullptr)
            *id = 0;
        return 0;
    }

    virtual bool available() override {
        return false;
    }

private:
    FCTP_OPT fCAN;
    int fRTS;
    int fRTSLED;
};
}

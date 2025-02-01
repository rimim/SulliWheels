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

#include "Platform.h"
#include "Log.h"
#include "Utils.h"

namespace pd {

using namespace pd::platform;

struct BusExtension {
    pd::String type;
};

class Bus {
public:
    Bus(pd::String name, BusType type, pd::String adapter, unsigned baud, int id = 0, int readtimeout = -1) :
        fName(name),
        fAdapter(adapter),
        fType(type),
        fID(id),
        fBaud(baud)
    {
        fStream = openStream(fAdapter.c_str(), type, baud, readtimeout);
        if (fStream == pd::STREAM_INVALID) {
            PDLOG_ERROR("Invalid adapter\n");
            return;
        }
    }

    ~Bus() {
        if (isOpen()) {
            closeStream(fStream);
        }
    }

    int ID() const {
        return fID;
    }

    inline bool isOpen() const {
        return (fStream != pd::STREAM_INVALID);
    }

    inline uint32_t getSpeed() const {
        return fBaud;
    }

    bool setSpeed(uint32_t baudrate) {
        if (isOpen()) {
            if (pd::platform::setSpeed(fStream, baudrate)) {
                fBaud = baudrate;
                return true;
            }
        }
        return false;
    }

    bool setReadTimeout(uint32_t centiseconds, int minAvailable = 0) {
        if (isOpen()) {
            return setStreamReadTimeout(fStream, centiseconds, minAvailable);
        }
        return false;
    }

    void setDebug(bool debug) {
        fDebug = debug;
    }

    bool available() {
        if (isOpen()) {
            return pd::platform::available(fStream);
        }
        return false;
    }

    ssize_t read(void* buffer, size_t bufferSize, uint32_t* id = nullptr) {
        ssize_t bytesRead = -1;
        if (isOpen()) {
            bytesRead = pd::platform::read(fStream, buffer, bufferSize, id);
            if (bytesRead > 0 && (pd::Log::isVerboseMotor() || fDebug)) {
                PDLOG_INFO("[%s:R] ", fName.c_str());
                for (ssize_t i = 0; i < bytesRead; i++) {
                    PDLOG_INFO("%02X ", ((uint8_t*)buffer)[i]);
                }
                PDLOG_INFO("\n");
            }
        }
        return bytesRead;
    }

    ssize_t write(const void* buffer, size_t bufferSize, uint32_t id = 0, bool ext = false) {
        if (isOpen()) {
            if (bufferSize > 0 && (pd::Log::isVerboseMotor() || fDebug)) {
                PDLOG_INFO("[%s:W] ", fName.c_str());
                for (unsigned i = 0; i < bufferSize; i++) {
                    PDLOG_INFO("%02X ", ((uint8_t*)buffer)[i]);
                }
                PDLOG_INFO("\n");
            }
            return pd::platform::write(fStream, buffer, bufferSize, id, ext);
        }
        return -1;
    }

    pd::String getName() {
        return fName;
    }

    pd::platform::Stream getStream() const {
        return fStream;
    }

    pd::BusExtension* getExtension(const char* ext) const {
        // TODO: Support multiple extensions
        if (fBusExtension != nullptr) {
            if (fBusExtension->type == ext) {
                return fBusExtension;
            }
            PDLOG_WARNING("BUS %s ALREADY HAS EXTENSION=%s WANTED %s\n", fName.c_str(), fBusExtension->type.c_str(), ext);
        }
        return nullptr;
    }

    void setExtension(pd::BusExtension* ext) {
        fBusExtension = ext;
    }

    void setError(bool error) {
        fError = error;
    }

    void clearError() {
        fError = false;
    }

    bool hasError() const {
        return fError;
    }

private:
    pd::String fName;
    pd::String fAdapter;
    pd::BusType fType;
    int fID;
    uint32_t fBaud = 0;
    bool fDebug = false;
    bool fError = false;
    pd::BusExtension* fBusExtension = nullptr;
    pd::platform::Stream fStream = pd::STREAM_INVALID;
};

}
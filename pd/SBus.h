/*
* Brian R Taylor
* brian.taylor@bolderflight.com
*
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "Event.h"

namespace pd {

struct SbusData {
    bool lost_frame;
    bool failsafe;
    bool ch17, ch18;
    static constexpr int8_t NUM_CH = 16;
    int16_t ch[NUM_CH];
};

class SbusRx {
public:
#if defined(ESP32)
    SbusRx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin,
         const bool inv) : uart_(bus), inv_(inv), rxpin_(rxpin), txpin_(txpin)
         {}
    SbusRx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin,
         const bool inv, const bool fast) : uart_(bus), inv_(inv), fast_(fast),
                                            rxpin_(rxpin), txpin_(txpin) {}
#else
    explicit SbusRx(HardwareSerial *bus) : uart_(bus) {}
    SbusRx(HardwareSerial *bus, const bool inv) : uart_(bus), inv_(inv) {}
    SbusRx(HardwareSerial *bus, const bool inv, const bool fast) : uart_(bus),
                                                                 inv_(inv),
                                                                 fast_(fast) {}
#endif
    void Begin();
    bool Read();
    inline SbusData data() const {return data_;}

    int16_t operator[](int channel) {
        return (!data_.failsafe && channel >= 0 && channel < data_.NUM_CH) ? data_.ch[channel] : 0;
    }

    bool isNeutral(int channel, int neutral, int tol) {
        if (!data_.failsafe && channel >= 0 && channel < data_.NUM_CH) {
            auto value = data_.ch[channel];
            return (value >= neutral - tol) && (value <= neutral + tol);
        }
        return false;
    }

 private:
    /* Communication */
    HardwareSerial *uart_;
    bool inv_ = true;
    bool fast_ = false;
#if defined(ESP32)
    int8_t rxpin_, txpin_;
#endif
    int32_t baud_ = 100000;
    /* Message len */
    static constexpr int8_t PAYLOAD_LEN_ = 23;
    static constexpr int8_t HEADER_LEN_ = 1;
    static constexpr int8_t FOOTER_LEN_ = 1;
    /* SBUS message defs */
    static constexpr int8_t NUM_SBUS_CH_ = 16;
    static constexpr uint8_t HEADER_ = 0x0F;
    static constexpr uint8_t FOOTER_ = 0x00;
    static constexpr uint8_t FOOTER2_ = 0x04;
    static constexpr uint8_t CH17_MASK_ = 0x01;
    static constexpr uint8_t CH18_MASK_ = 0x02;
    static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
    static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
    /* Parsing state tracking */
    int8_t state_ = 0;
    uint8_t prev_byte_ = FOOTER_;
    uint8_t cur_byte_;
    /* Buffer for storing messages */
    uint8_t buf_[25];
    /* Data */
    bool new_data_;
    SbusData data_;
    bool Parse();
};

class SbusTx {
 public:
#if defined(ESP32)
    SbusTx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin,
         const bool inv) : uart_(bus), inv_(inv), rxpin_(rxpin), txpin_(txpin)
         {}
    SbusTx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin,
         const bool inv, const bool fast) : uart_(bus), inv_(inv), fast_(fast),
                                            rxpin_(rxpin), txpin_(txpin) {}
#else
    explicit SbusTx(HardwareSerial *bus) : uart_(bus) {}
    SbusTx(HardwareSerial *bus, const bool inv) : uart_(bus), inv_(inv) {}
    SbusTx(HardwareSerial *bus, const bool inv, const bool fast) : uart_(bus),
                                                                 inv_(inv),
                                                                 fast_(fast) {}
#endif
    void Begin();
    void Write();
    inline void data(const SbusData &data) {data_ = data;}
    inline SbusData data() const {return data_;}

 private:
    /* Communication */
    HardwareSerial *uart_;
    bool inv_ = true;
    bool fast_ = false;
    #if defined(ESP32)
    int8_t rxpin_, txpin_;
    #endif
    int32_t baud_ = 100000;
    /* Message len */
    static constexpr int8_t BUF_LEN_ = 25;
    /* SBUS message defs */
    static constexpr int8_t NUM_SBUS_CH_ = 16;
    static constexpr uint8_t HEADER_ = 0x0F;
    static constexpr uint8_t FOOTER_ = 0x00;
    static constexpr uint8_t FOOTER2_ = 0x04;
    static constexpr uint8_t CH17_MASK_ = 0x01;
    static constexpr uint8_t CH18_MASK_ = 0x02;
    static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
    static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
    /* Data */
    uint8_t buf_[BUF_LEN_];
    SbusData data_;
};

struct SBusControllerEvent: public pd::ControllerEvent {
public:
    float sbus_to_float(int sbus_value) {
        // 1) Define input limits and midpoint
        const int   min_input    = 170;
        const int   max_input    = 1800;
        const int   midpoint     = (min_input + max_input) / 2;  // ~985

        // 2) Define deadband as a percentage of the total range
        const float deadband_percent = 0.05f;  // 5%
        const float total_range      = (float)(max_input - min_input);      // e.g. 1630
        const float deadband_range   = total_range * deadband_percent / 2;  // +/- around midpoint

        // 3) Compute offset from midpoint (this can be negative or positive)
        float offset = (float)sbus_value - (float)midpoint;

        // 4) If within deadband, return 0.0
        if (fabsf(offset) <= deadband_range) {
            return 0.0f;
        }

        // 5) "Effective" half-range is total_range/2 minus deadband_range.
        //    i.e. the portion from midpoint to min/max not included in deadband.
        float half_range = (total_range / 2.0f) - deadband_range;

        // 6) Figure out sign and how far we are outside deadband
        float sign         = (offset > 0.0f) ? +1.0f : -1.0f;
        float magnitude    = fabsf(offset) - deadband_range;
        float normalized   = magnitude / half_range;    // un-clamped
        float final_value  = sign * normalized;         // negative if below midpoint, positive if above

        // 7) Clamp to [-1, +1] just to be safe
        if (final_value >  1.0f) final_value =  1.0f;
        if (final_value < -1.0f) final_value = -1.0f;

        return final_value;
    }

    bool read(pd::SbusRx& sbus) {
        fValid = false;
        if (sbus.Read()) {
            static const int16_t sSequence[] = {
                176, 244, 311, 379, 448, 516, 584, 651, 720, 788, 856, 924, 1059, 1127, 1195
            };
            auto rssi = sbus[15];
            fButtonPressed = 0;
            for (unsigned i = 0; i < sizeof(sSequence)/sizeof(sSequence[0]); i++) {
                if (sSequence[i] == sbus[14]) {
                    if (!fButtonState[i]) {
                        fButtonState[i] = true;
                        fButtonPressed = i+1;
                    }
                } else if (fButtonState[i]) {
                    fButtonState[i] = false;
                }
            }
            for (unsigned i = 0; i < sizeof(fValues)/sizeof(fValues[0]); i++) {
                fValues[i] = sbus_to_float(sbus[i]);
            }
            auto data = sbus.data();
            fFailSafe = data.failsafe;
            fLostFrame = data.lost_frame;
            fRSSI = rssi;
            fValid = true;
        }
        return fValid;
    }
};

}  // pd

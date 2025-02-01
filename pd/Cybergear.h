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

#include "Actuator.h"

namespace pd::motor::xiaomi {

class Cybergear: public pd::Actuator {
public:
    Cybergear() {
    }

    Cybergear(uint8_t id, pd::String name) :
        pd::Actuator(id, pd::CyberGear, name.c_str())
    {
    }

    Cybergear(uint8_t id, const char* name = nullptr) :
        pd::Actuator(id, pd::CyberGear, name)
    {
    }

    virtual bool update() override {
        if (shouldIgnore()) {
            return true;
        }
        if (fBus == nullptr) {
            PDLOG_ERROR("UNRESOLVED ACTUATOR BUS\n");
            return false;
        }
        switch (fRunMode) {
            case MODE_UNKNOWN:
                if (initialize() && requestStatus()) {
                    printf("requestStatus\r\n");
                    return process();
                }
                return false;
            case MODE_MOTION:
                if (fActive) {
                    if (pd::Log::isVerboseMove()) {
                        PDLOG_INFO("[%s] %f\n", getName().c_str(), fPosNow);
                    }
                    if (!control(degreesToRadians(fPosNow), 0, fKP, fKD, fTau))
                        return false;
                } else if (!control(0, 0, 0, 0, 0)) {
                    return false;
                }
                return process();
            case MODE_POSITION:
                PDLOG_ERROR("NYI MODE_POSITION\n");
                return false;
            case MODE_SPEED:
                PDLOG_ERROR("NYI MODE_SPEED\n");
                return false;
            case MODE_CURRENT:
                PDLOG_ERROR("NYI MODE_CURRENT\n");
                return false;
        }
        return false;
    }

    enum RunMode {
        MODE_UNKNOWN  = -1,
        MODE_MOTION   = 0,
        MODE_POSITION = 1,
        MODE_SPEED    = 2,
        MODE_CURRENT  = 3
    };

    struct MotorStatus
    {
        uint8_t motor_id;             //!< motor id
        float position;               //!< encoder position (-4pi to 4pi)
        float velocity;               //!< motor velocity (-30rad/s to 30rad/s)
        float torque;                 //!< motor torque (-12Nm - 12Nm)
        float temperature;            //!< temperature
        uint16_t raw_position;        //!< raw position (for sync data)
        uint16_t raw_velocity;        //!< raw velocity (for sync data)
        uint16_t raw_torque;          //!< raw torque (for sync data)
        uint16_t raw_temperature;     //!< raw temperature (for sync data)

        bool hasCalibrationError;
        bool hasHallEncoderError;
        bool hasMagneticEncodingError;
        bool hasOverTemperature;
        bool hasOverCurrent;
        bool hasUnderVoltage;
        RunMode mode;
    };

    int getBusID() {
        return (fBus != nullptr) ? fBus->ID() : 0;
    }

    bool initialize() {
        if (initMotor(MODE_POSITION)) {
            if (!setLimitSpeedPercent(0.1f)) {
                PDLOG_ERROR("Failed setLimitSpeed\n");
            }
            return enableMotor();
        }
        return false;
    }

    bool initMotor(RunMode mode) {
        if (resetMotor()) {
            return setMode(mode);
        }
        return false;
    }

    bool enableMotor()
    {
        uint8_t data[8] = {0};
        return send(CMD_ENABLE, getBusID(), 8, data);
    }

    bool resetMotor() {
        uint8_t data[8] = {0};
        return send(CMD_RESET, getBusID(), 8, data);
    }

    RunMode getMode() const {
        return fRunMode;
    }

    bool setMode(RunMode mode) {
        uint8_t data[8] = {0};
        data[0] = ADDR_RUN_MODE & 0x00FF;
        data[1] = ADDR_RUN_MODE >> 8;
        data[4] = mode;
        if (send(CMD_RAM_WRITE, getBusID(), 8, data)) {
            fRunMode = mode;
            return true;
        }
        return false;
    }

    bool requestStatus() {
        uint8_t data[8] = {0};
        return send(CMD_GET_STATUS, getBusID(), 8, data);
    }

    bool setLimitSpeed(float speed) {
        return write_float_data(ADDR_LIMIT_SPEED, speed, 0.0f, V_MAX);
    }

    // Input range 0.0-1.0
    bool setLimitSpeedPercent(float speed) {
        speed = (speed < 0.0f) ? 0.0f : (speed > 1.0f) ? 1.0f : speed;
        return write_float_data(ADDR_LIMIT_SPEED, V_MAX * speed, 0.0f, V_MAX);
    }

    bool setLimitCurrent(float current) {
        return write_float_data(ADDR_LIMIT_CURRENT, current, 0.0f, IQ_MAX);
    }

    bool setCurrentKp(float kp) {
        return write_float_data(ADDR_CURRENT_KP, kp, 0.0f, KP_MAX);
    }

    void setCurrentKi(float ki) {
        write_float_data(ADDR_CURRENT_KI, ki, 0.0f, KI_MAX);
    }

    bool setCurrentFilterGain(float gain) {
        return write_float_data(ADDR_CURRENT_FILTER_GAIN, gain, CURRENT_FILTER_GAIN_MIN, CURRENT_FILTER_GAIN_MAX);
    }

    bool setLimitTorque(float torque) {
        return write_float_data(ADDR_LIMIT_TORQUE, torque, 0.0f, T_MAX);
    }

    bool setPositionKp(float kp) {
        return write_float_data(ADDR_LOC_KP, kp, 0.0f, 200.0f);
    }

    bool setVelocityKp(float kp) {
        return write_float_data(ADDR_SPD_KP, kp, 0.0f, 200.0f);
    }

    bool setVelocityKi(float ki) {
        return write_float_data(ADDR_SPD_KI, ki, 0.0f, 200.0f);
    }

    bool setPositionRef(float position) {
        return write_float_data(ADDR_LOC_REF, position, P_MIN, P_MAX);
    }

    bool setPositionDegrees(float degrees) {
        return write_float_data(ADDR_LOC_REF, degreesToRadians(degrees), P_MIN, P_MAX);
    }

    bool setSpeedRef(float speed) {
        return write_float_data(ADDR_SPEED_REF, speed, V_MIN, V_MAX);
    }

    bool setSpeedRefPercentage(float speed) {
        speed = (speed < -1.0f) ? -1.0f : (speed > 1.0f) ? 1.0f : speed;
        return write_float_data(ADDR_SPEED_REF, V_MAX * speed, V_MIN, V_MAX);
    }

    bool setCurrentRef(float current) {
        return write_float_data(ADDR_IQ_REF, current, IQ_MIN, IQ_MAX);
    }

    bool setZeroPosition() {
        uint8_t data[8] = {0};
        data[0] = 0x01;
        return send(CMD_SET_MECH_POSITION_TO_ZERO, getBusID(), 8, data);
    }

    bool changeMotorID(uint8_t can_id) {
        uint8_t data[8] = {0};
        uint16_t option = can_id << 8 | getBusID();
        return send(CMD_CHANGE_CAN_ID, option, 8, data);
    }

    bool process() {
        while (true) {
            auto ret = receive_motor_data(fStatus);
            if (ret == 1)
                return true;
            else if (ret == 0)
                break;
        }
        return false;
    }

    MotorStatus getStatus() const {
        return fStatus;
    }

    bool responded() const {
        return fResponded;
    }

    bool control(float pos, float speed, float kp, float kd, float tau) {
        uint16_t p = float_to_uint(pos, P_MIN, P_MAX, 16);
        uint16_t v = float_to_uint(speed, V_MIN, V_MAX, 16);
        uint16_t kpi = float_to_uint(kp, KP_MIN, KP_MAX, 16);
        uint16_t kdi = float_to_uint(kd, KD_MIN, KD_MAX, 16);

        uint8_t data[] = {
            uint8_t(p >> 8),
            uint8_t(p & 0xFF),

            uint8_t(v >> 8),
            uint8_t(v & 0xFF),

            uint8_t(kpi >> 8),
            uint8_t(kpi & 0xFF),

            uint8_t(kdi >> 8),
            uint8_t(kdi & 0xFF)
        };
        uint16_t torque = float_to_uint(tau, T_MIN, T_MAX, 16);
        return send(CMD_POSITION, torque, 8, data);
    }

    static unsigned process(Cybergear* motors[], unsigned count) {
        Bus* bus = nullptr;
        unsigned responseCount = 0;
        for (unsigned i = 0; i < count; i++) {
            bus = motors[i]->getBus();
            motors[i]->fResponded = false;
        }

        for (unsigned i = 0; i < count; i++) {
            uint32_t id;
            uint8_t data[8] = {0};
            if (bus->read(data, sizeof(data), &id) != sizeof(data)) {
                // No more data
                break;
            }
            for (unsigned i = 0; i < count; i++) {
                Cybergear* motor = motors[i];
                if (motor->parse_motor_data(id, data, motor->fStatus) == 1) {
                    motor->fResponded = true;
                    // PDLOG_INFO("MOTOR id=%d mode=%d\n", motor->fMotorID, motor->fStatus.mode);
                    responseCount++;
                }
            }
        }
        return responseCount;
    }

protected:
    RunMode fRunMode = MODE_UNKNOWN;
    MotorStatus fStatus;
    bool fResponded = false;

    enum Address {
        ADDR_SPD_KP                = 0x2014,
        ADDR_SPD_KI                = 0x2015,
        ADDR_LOC_KP                = 0x2016,
        ADDR_RUN_MODE              = 0x7005,
        ADDR_IQ_REF                = 0x7006,
        ADDR_SPEED_REF             = 0x700A,
        ADDR_LIMIT_TORQUE          = 0x700B,
        ADDR_CURRENT_KP            = 0x7010,
        ADDR_CURRENT_KI            = 0x7011,
        ADDR_CURRENT_FILTER_GAIN   = 0x7014,
        ADDR_LOC_REF               = 0x7016,
        ADDR_LIMIT_SPEED           = 0x7017,
        ADDR_LIMIT_CURRENT         = 0x7018
    };

    enum Cmd {
        CMD_POSITION                   = 1,
        CMD_RESPONSE                   = 2,
        CMD_ENABLE                     = 3,
        CMD_RESET                      = 4,
        CMD_SET_MECH_POSITION_TO_ZERO  = 6,
        CMD_CHANGE_CAN_ID              = 7,
        CMD_RAM_READ                   = 17,
        CMD_RAM_WRITE                  = 18,
        CMD_GET_STATUS                 = 21
    };

    enum {
        OK             = 0,
        MSG_NOT_AVAIL  = 1,
        INVALID_CAN_ID = 2,
        INVALID_PACKET = 3
    };

    static constexpr float GEAR_RATIO = 6.33;
    static constexpr float P_MIN= -12.5f;
    static constexpr float P_MAX=  12.5f;
    static constexpr float V_MIN  = -30.0f;
    static constexpr float V_MAX  =  30.0f;
    static constexpr float KP_MIN =   0.0f;
    static constexpr float KP_MAX = 500.0f;
    static constexpr float KI_MIN =   0.0f;
    static constexpr float KI_MAX =  10.0f;
    static constexpr float KD_MIN =   0.0f;
    static constexpr float KD_MAX =   5.0f;
    static constexpr float T_MIN  = -12.0f;
    static constexpr float T_MAX  =  12.0f;
    static constexpr float IQ_MIN = -27.0f;
    static constexpr float IQ_MAX =  27.0f;
    static constexpr float CURRENT_FILTER_GAIN_MIN = 0.0f;
    static constexpr float CURRENT_FILTER_GAIN_MAX = 1.0f;

    bool send(uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data) {
        uint32_t id = (0x00000000 | cmd_id << 24 | option << 8 | fMotorID);
        // printf("ID=%08uX fMotorID=%d\r\n", id, fMotorID);
        if (fBus->write(data, len, id, true) == len) {
            return true;
        }
        // dbg("failed to send");
        return false;
    }

    bool read_ram_data(uint16_t index) {
        uint8_t data[8] = {0};
        memcpy(&data[0], &index, 2);
        return send(CMD_RAM_READ, getBusID(), 8, data);
    }

    bool write_float_data(Address addr, float value, float min, float max) {
        uint8_t data[8] = {0};
        data[0] = addr & 0x00FF;
        data[1] = addr >> 8;

        float val = (max < value) ? max : value;
        val = (min > value) ? min : value;
        memcpy(&data[4], &val, 4);
        return send(CMD_RAM_WRITE, getBusID(), 8, data);
    }

    int float_to_uint(float x, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        if (x > x_max)
            x = x_max;
        else if (x < x_min)
            x = x_min;
        return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    float uint_to_float(uint16_t x, float x_min, float x_max) {
        uint16_t type_max = 0xFFFF;
        float span = x_max - x_min;
        return (float) x / type_max * span + x_min;
    }

    int receive_motor_data(MotorStatus &mot)
    {
        uint32_t id;
        uint8_t data[8] = {0};
        if (fBus->read(data, sizeof(data), &id) != sizeof(data)) {
            return 0;
        }
        return parse_motor_data(id, data, mot);
    }

    int parse_motor_data(uint32_t id, uint8_t* data, MotorStatus &mot)
    {
        // if id is not mine
        uint8_t receive_can_id = (id & 0xFF);
        uint8_t motor_can_id = ((id >> 8) & 0xFF);
        if (receive_can_id != getBusID() ||
            motor_can_id != fMotorID)
        {
            return -1;
        }

        // check packet type
        uint8_t packet_type = ((id >> 24) & 0x3F);
        if (packet_type != CMD_RESPONSE) {
            return -1;
        }

        mot.raw_position = (data[1] | data[0] << 8);
        mot.raw_velocity = (data[3] | data[2] << 8);
        mot.raw_torque = (data[5] | data[4] << 8);
        mot.raw_temperature = (data[7] | data[6] << 8);

        // convert motor data
        mot.motor_id = motor_can_id;
        mot.position = uint_to_float(mot.raw_position, P_MIN, P_MAX);
        mot.velocity = uint_to_float(mot.raw_velocity, V_MIN, V_MAX);
        mot.torque = uint_to_float(mot.raw_torque, T_MIN, T_MAX);
        mot.temperature = mot.raw_temperature / 10.0;

        mot.hasUnderVoltage = (id >> 16) & 1;
        mot.hasOverCurrent = (id >> 17) & 1;
        mot.hasOverTemperature = (id >> 18) & 1;
        mot.hasMagneticEncodingError = (id >> 19) & 1;
        mot.hasHallEncoderError = (id >> 20) & 1;
        mot.hasCalibrationError = (id >> 21) & 1;
        mot.mode = RunMode((id >> 22) & 3);

        setFeedback(
            radiansToDegrees(mot.position / GEAR_RATIO),
            mot.velocity,
            mot.torque,
            mot.temperature);
        if (mot.hasUnderVoltage)
            setError(Error::VoltageError);
        else if (mot.hasOverCurrent)
            setError(Error::CurrentError);
        else if (mot.hasOverTemperature)
            setError(Error::TemperatureError);
        else if (mot.hasMagneticEncodingError)
            setError(Error::EncoderError);
        else if (mot.hasHallEncoderError)
            setError(Error::EncoderError);
        else if (mot.hasCalibrationError)
            setError(Error::EncoderError);
        else
            clearError();
        return 1;
    }
};

}

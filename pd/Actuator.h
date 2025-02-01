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

#include "Bus.h"
#include "Log.h"
#include "Easing.h"
#include "Utils.h"

namespace pd {

using MotorType = pd::MotorType;

class Actuator {
public:
    enum Error {
        Normal = 0,
        TemperatureError = 1,
        CurrentError = 2,
        VoltageError = 3,
        EncoderError = 4
    };

    Actuator() {
        fName[0] = '\0';
    }

    Actuator(uint8_t id, MotorType type, const char* name = nullptr) {
        setMotorID(id, type, name);
    }

    MotorType getType() {
        return fType;
    }

    Bus* getBus() const {
        return fBus;
    }

    void setBus(Bus* bus) {
        fBus = bus;
    }

    // void setFakeRad(double rad) {
    //     fDegrees = degreesToRadians(radiansToDegrees(rad) + mapPositionToDegrees(0));
    // }

    inline void setMotorID(uint8_t id, MotorType type, const char* name = nullptr) {
        fMotorID = id;
        fType = type;
        fName[0] = '\0';
        fChanged = true;
        if (name != nullptr) {
            snprintf(fName, sizeof(fName), "%s", name);
        }
    }

    void setZeroOnStart(bool zeroOnStart) {
        fZeroOnStart = zeroOnStart;
    }

    double getMinimum() const
    {
        return std::min(fRange[0], fRange[1]);
    }

    double getMaximum() const
    {
        return std::max(fRange[0], fRange[1]);
    }

    const double* getRange() const {
        return fRange;
    }

    void setRange(double pos1, double pos2) {
        printf("%s.setRange(%f, %f)\r\n", fName, pos1, pos2);
        fRange[0] = pos1;
        fRange[1] = pos2;
        fChanged = true;
    }

    /*
     * Proportional Gain (kp):
     * - Definition: The proportional gain kp determines the response of the controller to the current error.
     *   It is a scaling factor applied to the error value.
     * - Effect:
     *   - Increasing kp: This results in a stronger response to the error, making the system react more quickly
     *     to discrepancies between the desired and actual positions (or other controlled variables). However, if kp
     *     is too high, it can cause the system to become overly aggressive, leading to oscillations or instability.
     *   - Decreasing kp: This results in a weaker response to the error, making the system slower to react. While this
     *     can reduce oscillations and improve stability, it may also result in sluggish performance and a slower return
     *     to the desired position.
     */
    void setKP(double kp) {
        fKP = kp;
        fChanged = true;
    }

    /*
     * Derivative Gain (kd):
     * - Definition: The derivative gain kd determines the response of the controller to the rate of change of the error.
     *   It is a scaling factor applied to the derivative (rate of change) of the error.
     * - Effect:
     *   - Increasing kd: This results in a stronger damping effect, helping to smooth out the response and reduce overshoot
     *     and oscillations. Essentially, it predicts future error based on the current rate of change and adjusts the control
     *     effort accordingly.
     *   - Decreasing kd: This results in a weaker damping effect, which can lead to increased oscillations and overshoot if
     *     the proportional gain kp is high. However, if kd is too low, the system might not sufficiently counteract rapid changes
     *     in the error, leading to a less smooth response.
     */
    void setKD(double kd) {
        fKD = kd;
        fChanged = true;
    }

    /*
     * setTau(float tau):
     * - Description: Sets the torque (tau) for the motor. The torque represents the rotational force being applied.
     * - Parameters:
     *   - tau: The desired torque value in Newton-meters (Nm).
     * - Usage: Use this function to control the amount of rotational force the motor applies.
     */
    void setTau(double tau) {
        fTau = tau;
        fChanged = true;
    }

    void setInvert(bool invert) {
        fInvert = invert;
        fChanged = true;
    }

    inline bool isInverted() const {
        return fInvert;
    }

    inline bool isRangeValid() const {
        return (fRange[0] == fRange[0] && fRange[1] == fRange[1] && fRange[0] != fRange[1]);
    }

    inline bool isPositionValid() const {
        return (fDegrees == fRange[0] && fRange[1] == fRange[1] && fRange[0] != fRange[1]);
    }

    /*
     * mapDegreesToPosition(double degrees) const:
     * - Description: Maps motor degrees to a normalized position value in the range [-1.0, 1.0].
     * - Parameters:
     *   - degrees: The motor's angular position in degrees.
     * - Returns: A double representing the normalized position in the range [-1.0, 1.0].
     * - Usage: Use this function to convert an angular position in degrees to a normalized
     *    position suitable for control algorithms or user interfaces.
     */
    double mapDegreesToPosition(double degrees) const {
        double tolerance = 0.1;
        double min = getMinimum();
        double max = getMaximum();
        double mid = (min + max) / 2.0;
        double position = (2.0 * (degrees - mid)) / (max - min);
        if (position < -1.0) {
            if (position < -1.0 - tolerance) {
                // Out of bounds
                return NAN;
            }
            // Acceptable
            position = -1.0;
        } else if (position > 1.0) {
            if (position > 1.0 + tolerance) {
                // Out of bounds
                return NAN;
            }
            // Acceptable
            position = 1.0;
        }
        return isInverted() ? -position : position; // Within the valid range
    }

    /*
     * mapPositionToDegrees(double pos) const:
     * - Description: Maps a normalized position value in the range [-1.0, 1.0] to motor degrees.
     * - Parameters:
     *   - pos: The normalized position value in the range [-1.0, 1.0].
     * - Returns: A double representing the motor's angular position in degrees.
     * - Usage: Use this function to convert a normalized position value to an angular position in
     *    degrees for motor control or visualization.
     */
    double mapPositionToDegrees(double pos) const {
        if (isInverted())
            pos = -pos;
        double min = getMinimum();
        double max = getMaximum();
        double mid = (min + max) / 2.0;
        double degrees = pos * (max - min) / 2.0 + mid;

        if (degrees < min) {
            return min;
        } else if (degrees > max) {
            return max;
        }
        return degrees;
    }

    /*
     * moveToPosition(uint32_t startDelay, uint32_t moveTime, double pos):
     * - Description: Initiates a movement to a specified normalized position after a delay.
     * - Parameters:
     *   - startDelay: The delay in milliseconds before starting the movement.
     *   - moveTime: The duration in milliseconds for the movement to complete.
     *   - pos: The target position as a normalized value in the range [-1.0, 1.0].
     * - Usage: Use this function to move the motor to a specified normalized position with a specified delay and movement duration.
     * - Note: This function internally maps the normalized position to motor degrees and calls moveToDegrees.
     */
    void moveToPosition(uint32_t startDelay, uint32_t moveTime, double pos) {
        moveToDegrees(startDelay, moveTime, mapPositionToDegrees(pos));
    }

    /*
     * moveToDegrees(uint32_t startDelay, uint32_t moveTime, double degrees):
     * - Description: Initiates a movement to a specified angular position in degrees after a delay.
     * - Parameters:
     *   - startDelay: The delay in milliseconds before starting the movement.
     *   - moveTime: The duration in milliseconds for the movement to complete.
     *   - degrees: The target angular position in degrees.
     * - Usage: Use this function to move the motor to a specified angular position with a specified delay and movement duration.
     */
    void moveToDegrees(uint32_t startDelay, uint32_t moveTime, double degrees) {
        if (std::isnan(getPosition()))
            return;
        fActive = true;
        double finalPos = std::min(getMaximum(), std::max(getMinimum(), degrees));
        PDLOG_INFO("moveToDegrees: finalPos=%f invert=%d\n", finalPos, isInverted());
        if (moveTime != 0) {
            uint64_t timeNow = pd::platform::currentTimeMillis();
            fStartTime = startDelay + timeNow;
            fFinishTime = moveTime + fStartTime;
            fOffTime = fFinishTime;
            if (moveTime == 0)
                fOffTime += 200;
            fFinishPos = finalPos;
            fPosNow = fDegrees;
            fStartPosition = fPosNow;
            fDeltaPos = fFinishPos - fPosNow;
            fLastMoveTime = pd::platform::currentTimeMillis();
            PDLOG_INFO("moveToDegrees: %f\n", fPosNow);
        } else {
            fPosNow = finalPos;
        }
    }

    void moveToRelativeRadians(double radians, uint32_t startTime = 0, uint32_t moveTime = 0) {
        moveToDegrees(startTime, moveTime, radiansToDegrees(radians) + mapPositionToDegrees(0));
        // printf("%s.moveToRelativeRadians[%f] %f -> %f\r\n", fName, radians, fDegrees, radiansToDegrees(radians) + mapPositionToDegrees(0));
    }

    void move(uint64_t timeNow) {
        Easing::Method easing = Easing::get(fEasingMethod);
        if (fFinishTime != 0)
        {
            if (timeNow < fStartTime)
            {
                /* wait */
            }
            else if (timeNow >= fFinishTime)
            {
                fPosNow = fFinishPos;
                reset();
            }
            else if (fLastMoveTime != timeNow)
            {
                uint32_t timeSinceLastMove = timeNow - fStartTime;
                uint32_t denominator = fFinishTime - fStartTime;
                double fractionChange = easing(double(timeSinceLastMove) / double(denominator));
                double distanceToMove = fDeltaPos * fractionChange;
                double newPos = fStartPosition + distanceToMove;
                if (newPos != fPosNow)
                {
                    fPosNow = fStartPosition + distanceToMove;
                    fLastMoveTime = timeNow;
                }
            }
        }
        else if (fOffTime != 0 && timeNow >= fOffTime)
        {
            fOffTime = 0;
        }
    }

    void clearError() {
        fError = Normal;
        fErrorCount = 0;
    }

    void setError(Error error) {
        fError = error;
        fErrorCount += 1;
    }

    inline double getFeedbackTorque() const {
        return fFeedbackTorque;
    }

    inline double getFeedbackVelocity() const {
        return fFeedbackVelocity;
    }

    inline double getFeedbackTemp() const {
        return fFeedbackTemp;
    }

    void setFeedback(double degrees, double velocity = NAN, double torque = NAN, float temperature = NAN) {
        fDegrees = degrees;
        fFeedbackTorque = torque;
        fFeedbackVelocity = velocity;
        fFeedbackTemp = temperature;
        if (!fActive) {
            fPosNow = fStartPosition = fDegrees;
        }
        if (std::isnan(fMinDegrees) || fMinDegrees < fDegrees) {
            fMinDegrees = fDegrees;
        }
        if (std::isnan(fMaxDegrees) || fMaxDegrees > fDegrees) {
            fMaxDegrees = fDegrees;
        }
        fLastResponse = pd::platform::currentTimeMillis();
    }

    void reset() {
        fFinishTime = 0;
        fLastMoveTime = 0;
        fFinishPos = 0;
        fOffTime = 0;
    }

    bool hasVoltageError() const {
        return fError == VoltageError;
    }

    bool hasCurrentError() const {
        return fError == CurrentError;
    }

    bool hasTemperatureError() const {
        return fError == TemperatureError;
    }

    bool hasEncoderError() const {
        return fError == EncoderError;
    }

    bool checkRange() const {
        if (std::isnan(fMinDegrees) || std::isnan(fMaxDegrees)) {
            return false;
        }
        return true;
    }

    virtual bool update() = 0;

    unsigned getMissCount() const {
        return fMissCount;
    }

    unsigned getErrorCount() const {
        return fErrorCount;
    }

    bool hasError() const {
        return (fMissCount != 0 || fErrorCount != 0);
    }

    bool stop() {
        reset();
        fActive = false;
        return update();
    }

    void relax() {
        fActive = false;
    }

    void stiff() {
        reset();
        fActive = true;
    }

    bool isMoving() const {
        if (fActive)
            return (fFinishTime != 0 || fLastMoveTime != 0 || fFinishPos != 0);
        return false;
    }

    bool isStiff() const {
        return (fActive && !isMoving());
    }

    /*
     * getDegrees() const:
     * - Description: Retrieves the current angular position of the motor in degrees.
     * - Returns: A double representing the motor's current angular position in degrees.
     * - Usage: Use this function to get the current angular position of the motor.
     */
    inline double getDegrees() const {
        return fDegrees;
    }

    inline double getRelativeRadians() const {
        return degreesToRadians(fDegrees - mapPositionToDegrees(0));
    }

    /*
     * getPosition() const:
     * - Description: Retrieves the current normalized position of the motor in the range [-1.0, 1.0].
     * - Returns: A double representing the motor's current normalized position.
     * - Usage: Use this function to get the current normalized position of the motor.
     */
    inline double getPosition() {
        if (!isRangeValid()) {
            pd::String busName;
            if (fBus != nullptr)
                busName = fBus->getName();
            if (!fRangeError) {
                PDLOG_ERROR("ERROR ACUTATOR: \"%s.%s\" RANGE NOT SET\n", busName.c_str(), fName);
                fRangeError = true;
            }
        } else {
            fRangeError = false;
            if (fZeroOnStart) {
                pd::String busName;
                if (fBus != nullptr)
                    busName = fBus->getName();
                PDLOG_ERROR("%s.ZEROPOS %s fDegrees = %f [%f,%f]\n",
                    busName.c_str(), fName, fDegrees,
                    fDegrees + getMinimum(),
                    fDegrees + getMaximum());
                setRange(fDegrees + getMinimum(), fDegrees + getMaximum());
                PDLOG_ERROR("             new POS = %f\n", mapDegreesToPosition(fDegrees));
                fZeroOnStart = false;
            }
            double pos = mapDegreesToPosition(fDegrees);
            if (std::isnan(pos)) {
                pd::String busName;
                if (fBus != nullptr)
                    busName = fBus->getName();
                if (!fAlignmentError) {
                    PDLOG_ERROR("ERROR ACUTATOR: \"%s.%s\" OUT OF ALIGNMENT\n", busName.c_str(), fName);
                    PDLOG_ERROR("[%s] pos=%f degrees=%f [%f:%f]\n", fName, pos, fDegrees, fRange[0], fRange[1]);
                    fAlignmentError = true;
                }
                return NAN;
            }
            fAlignmentError = false;
            return pos;
        }
        return NAN;
    }

    void setEasing(Easing::Method method) {
        fEasingMethod = method;
    }

    double getMinDegrees() const {
        return fMinDegrees;
    }

    double getMaxDegrees() const {
        return fMaxDegrees;
    }

    uint8_t getID() const {
        return fMotorID;
    }

    void setID(uint8_t id) {
        fMotorID = id;
    }

    pd::String getName() const {
        return fName;
    }

    void setIgnore() {
        fIgnore = true;
    }

    bool shouldIgnore() const {
        return fIgnore;
    }

    inline uint32_t timeSinceLastResponse() const {
        if (fLastResponse) {
            return pd::platform::currentTimeMillis() - fLastResponse;
        }
        return ~0;
    }

    inline bool isResponding() const {
        return (timeSinceLastResponse() < 100);
    }

protected:
    char        fName[16];
    Bus*        fBus = nullptr;
    MotorType   fType = pd::UnknownMotor;
    Error       fError = Normal;
    bool        fZeroOnStart = false;
    unsigned    fErrorCount = 0;
    unsigned    fMissCount = 0;
    bool        fChanged = true;
    bool        fIgnore = false;
    uint8_t     fMotorID = 0;
    bool        fActive = false;
    bool        fInvert = false;
    double      fRange[2] = { 0, 0 };
    double      fMinDegrees = NAN;
    double      fMaxDegrees = NAN;
    uint64_t    fStartTime = 0;
    uint64_t    fFinishTime = 0;
    uint64_t    fLastMoveTime = 0;
    uint64_t    fOffTime = 0;
    double      fFeedbackVelocity = NAN;
    double      fFeedbackTorque = NAN;
    float       fFeedbackTemp = NAN;
    double      fKP = 1.0;
    double      fKD = 0.01;
    double      fTau = 0;
    double      fFinishPos = 0;
    double      fStartPosition = 0;
    double      fPosNow = 0;
    double      fDeltaPos = 0;
    double      fDegrees = 0;
    uint64_t    fLastResponse = 0;
    bool        fAlignmentError = false;
    bool        fRangeError = false;
    double      (*fEasingMethod)(double completion) = nullptr;
};

};

// --------------------------------------------------
// Sulli CyberGear motors
// --------------------------------------------------
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

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "pin-map.h"

#include "pd/Bus.h"
#include "pd/SBus.h"
#include "pd/Log.h"
#include "pd/StreamProxy.h"
#include "pd/Cybergear.h"

// ---------------------------------------------------------------------------
using Bus = pd::Bus;
using NeoPixel = Adafruit_NeoPixel;
using Cybergear = pd::motor::xiaomi::Cybergear;
// ---------------------------------------------------------------------------

#define LEFT_MOTOR_ID           101
#define RIGHT_MOTOR_ID          102

#define SBUS_THROTTLE_CHANNEL   1
#define SBUS_TURN_CHANNEL       5

#define TOP_SPEED_LIMIT         0.1f /* Range 0.0 - 1.0 increase for faster response */ 

#define LED1_FAILSAFE           0
#define LED2_FAILSAFE           1
#define LED1_LEFT_MOTOR         2
#define LED2_LEFT_MOTOR         3
#define LED3_LEFT_MOTOR         4
#define LED1_RIGHT_MOTOR        5
#define LED2_RIGHT_MOTOR        6
#define LED3_RIGHT_MOTOR        7

#define LED_BLINK_DELAY         1000 /* ms */

#define FAILSAFE_ERROR_COLOR    NeoPixel::Color(127, 0, 0, 0)
#define LEFT_ERROR_COLOR        NeoPixel::Color(127, 0, 0, 0)
#define RIGHT_ERROR_COLOR       NeoPixel::Color(127, 0, 0, 0)
#define ALL_IS_WELL_COLOR       NeoPixel::Color(0, 127, 0, 0)

#define MAX_ALLOWED_INITIAL_THROTTLE 0.1

// ---------------------------------------------------------------------------
// Error states

enum {
  STATUS_FAILSAFE         = 1<<0,
  STATUS_LEFT_RESPONDING  = 1<<1,
  STATUS_RIGHT_RESPONDING = 1<<2,

  STATUS_INITIAL = (
    STATUS_FAILSAFE         |
    STATUS_LEFT_RESPONDING  |
    STATUS_RIGHT_RESPONDING),
};
unsigned STATUS = STATUS_INITIAL;

// ---------------------------------------------------------------------------
pd::SbusRx sbus_rx(&Serial3);
//pd::SbusTx sbus_tx(&Serial3);
pd::SbusData data;
// ---------------------------------------------------------------------------

// Setup CAN1 stream
pd::FlexCANStreamProxy<CAN2> sCAN1("Can1");
// ---------------------------------------------------------------------------
NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);

// ---------------------------------------------------------------------------

void stripShowColor(uint32_t color) {
  strip.fill(color);
  strip.show();
}

void stripShowColor(unsigned start, unsigned count, uint32_t color) {
  for (unsigned i = start; i < start + count; i++) {
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
  strip.show();
}

float mapMinusOneToOne(float x) {
    // x is expected to be in range [-1.0, 1.0]
    // return mapped value in [0.0, 1.0]
    return (x + 1.0f) / 2.0f;  
}

// ---------------------------------------------------------------------------

pd::SBusControllerEvent event;
Cybergear leftMotor(LEFT_MOTOR_ID, "Left");
Cybergear rightMotor(RIGHT_MOTOR_ID, "Right");

bool throttleHasBeenNeutral = false;

bool initLeftMotor() {
  if (!leftMotor.initMotor(Cybergear::MODE_SPEED)) {
    return false;
  }
  delay(1);
  if (!leftMotor.enableMotor()) {
    return false;
  }
  delay(1);
  if (!leftMotor.setSpeedRefPercentage(0)) {
    return false;
  }
  delay(10);
  return true;
}

bool initRightMotor() {
  if (!rightMotor.initMotor(Cybergear::MODE_SPEED)) {
    return false;
  }
  delay(1);
  if (!rightMotor.enableMotor()) {
    return false;
  }
  delay(1);
  if (!rightMotor.setSpeedRefPercentage(0)) {
    return false;
  }
  delay(10);
  return true;
}

void setup(void) {

  Serial.begin(4E6); //4Mbit/s
  // Enable to wait for PC to be ready
  // while (!Serial.dtr()) {
  // }
  pd::platform::init();

  strip.begin();
  strip.setBrightness(NEOPIXEL_BRIGHTNESS);
  strip.show();

  sbus_rx.Begin();
//  sbus_tx.Begin();

  auto bus = new pd::Bus("CAN", pd::CAN, "Can1", 1000000, 0, 2);

  // Force verbose logging for can commands
  // pd::Log::log().parse("-v:can");

  leftMotor.setBus(bus);
  rightMotor.setBus(bus);
  if (!initLeftMotor()) {
      PDLOG_ERROR("Failed left initMotor\n");
  }
  if (!initRightMotor()) {
      PDLOG_ERROR("Failed right initMotor\n");
  }
}

void printMotorStatus(const char* motorName, const Cybergear::MotorStatus &motorStatus) {
  if (motorStatus.hasCalibrationError)
    PDLOG_ERROR("ERROR: %s hasCalibrationError\n", motorName);
  if (motorStatus.hasHallEncoderError)
    PDLOG_ERROR("ERROR: %s hasHallEncoderError\n", motorName);
  if (motorStatus.hasMagneticEncodingError)
    PDLOG_ERROR("ERROR: %s hasMagneticEncodingError\n", motorName);
  if (motorStatus.hasOverTemperature)
    PDLOG_ERROR("ERROR: %s hasOverTemperature\n", motorName);
  if (motorStatus.hasOverCurrent)
    PDLOG_ERROR("ERROR: %s hasOverCurrent\n", motorName);
  if (motorStatus.hasUnderVoltage)
    PDLOG_ERROR("ERROR: %s hasUnderVoltage\n", motorName);
}

void loop() {
  if (event.read(sbus_rx) && event.fValid) {
    if (event.fFailSafe) {
      if ((STATUS & STATUS_FAILSAFE) == 0) {
        PDLOG_ERROR("RC FAILSAFE\n");
        STATUS |= STATUS_FAILSAFE;
      }
      // Set everything to neutral in case of failsafe
      event.fValues[SBUS_THROTTLE_CHANNEL] = 0;
      event.fValues[SBUS_TURN_CHANNEL] = 0;
      event.fButtonPressed = 0;
    } else {
      STATUS &= ~STATUS_FAILSAFE;
      if (!throttleHasBeenNeutral) {
        // Safety check. Ignore the throttle if sketch is just starting
        // the current value is greater than MAX_ALLOWED_INITIAL_THROTTLE
        if (fabs(event.fValues[SBUS_THROTTLE_CHANNEL]) < MAX_ALLOWED_INITIAL_THROTTLE) {
          throttleHasBeenNeutral = true;
        }
      }
      if (!throttleHasBeenNeutral) {
        event.fValues[SBUS_THROTTLE_CHANNEL] = 0;
      }
    }
    if (event.fButtonPressed != 0) {
      PDLOG_INFO("PRESSED BUTTON: %d\n", event.fButtonPressed);
      switch (event.fButtonPressed) {
        case 1:
          /* Do something for button#1 */
          break;
        case 2:
          /* Do something for button#2 */
          break;
        case 3:
          /* Do something for button#3 */
          break;
        case 4:
          /* Do something for button#4 */
          break;
        case 5:
          /* Do something for button#5 */
          break;
        case 6:
          /* Do something for button#6 */
          break;
        case 7:
          /* Do something for button#7 */
          break;
        case 8:
          /* Do something for button#8 */
          break;
        case 9:
          /* Do something for button#9 */
          break;
        case 10:
          /* Do something for button#10 */
          break;
        case 11:
          /* Do something for button#11 */
          break;
        case 12:
          /* Do something for button#12 */
          break;
        case 13:
          /* Do something for button#13 */
          break;
        case 14:
          /* Do something for button#14 */
          break;
        case 15:
          /* Do something for button#15 */
          break;
      }
    }
  }

  // -------------------------------------------------------------------------
  // 1) Motor Mixing
  // -------------------------------------------------------------------------
  // Combine user throttle, turn
  // Each of these is in [-1.0, 1.0], so sum can exceed that.
  double throttle = 0.0 - event.fValues[SBUS_THROTTLE_CHANNEL]; // WOODY EDIT - added the "0.0 -" to INVERT throttle and turn inputs
  double turn = 0.0 - event.fValues[SBUS_TURN_CHANNEL];
  double motorLeft  = throttle + turn;
  double motorRight = throttle - turn;

  // Scale by top speed limit
  motorLeft *= TOP_SPEED_LIMIT;
  motorRight *= TOP_SPEED_LIMIT;

  // Clamp final motor commands to [-1.0, 1.0]
  if (motorLeft >  1.0) motorLeft =  1.0;
  if (motorLeft < -1.0) motorLeft = -1.0;
  if (motorRight >  1.0) motorRight =  1.0;
  if (motorRight < -1.0) motorRight = -1.0;

  // -------------------------------------------------------------------------
  // 5) Send to Motors
  // -------------------------------------------------------------------------
  // Now we directly call .setSpeedRefPercentage() with values in [-1.0, 1.0].
  if (motorLeft != 0 || motorRight != 0)
    PDLOG_INFO("motorLeft: %0.2f motorRight: %0.2f\n", motorLeft, motorRight);

  {
    leftMotor.setSpeedRefPercentage(motorLeft);
    delay(1);

    // Process motor responses
    Cybergear* motors[] = { &leftMotor };
    Cybergear::process(motors, sizeof(motors)/sizeof(motors[0]));

    // If left motor responded check if there was an error
    if (leftMotor.responded()) {
      auto motorStatus = leftMotor.getStatus();
      if (leftMotor.hasError()) {
        printMotorStatus("leftMotor", motorStatus);
        STATUS &= ~STATUS_LEFT_RESPONDING;
      } else if ((STATUS & STATUS_LEFT_RESPONDING) == 0) {
        if (motorStatus.mode != Cybergear::MODE_SPEED) {
          initLeftMotor();
        } else {
          STATUS |= STATUS_LEFT_RESPONDING;
          PDLOG_ERROR("LEFT MOTOR RESPONDING\n");
        }
      }
    } else if ((STATUS & STATUS_LEFT_RESPONDING) != 0) {
      PDLOG_ERROR("LEFT MOTOR NOT RESPONDING\n");
      STATUS &= ~STATUS_LEFT_RESPONDING;
    }
  }

  {
    rightMotor.setSpeedRefPercentage(motorRight);
    delay(1);

    // Process motor responses
    Cybergear* motors[] = { &rightMotor };
    Cybergear::process(motors, sizeof(motors)/sizeof(motors[0]));

    // If right motor responded check if there was an error
    if (rightMotor.responded()) {
      auto motorStatus = rightMotor.getStatus();
      if (rightMotor.hasError()) {
        printMotorStatus("rightMotor", motorStatus);
        STATUS &= ~STATUS_RIGHT_RESPONDING;
      } else if ((STATUS & STATUS_RIGHT_RESPONDING) == 0) {
        if (motorStatus.mode != Cybergear::MODE_SPEED) {
          initRightMotor();
        } else {
          STATUS |= STATUS_RIGHT_RESPONDING;
          PDLOG_ERROR("RIGHT MOTOR RESPONDING\n");
        }
      }
    } else if ((STATUS & STATUS_RIGHT_RESPONDING) != 0) {
      PDLOG_ERROR("RIGHT MOTOR NOT RESPONDING\n");
      STATUS &= ~STATUS_RIGHT_RESPONDING;
    }
  }

  static unsigned lastStatus;
  static unsigned lastBlinkTime;
  static bool flipFlop;
  unsigned now = millis();
  if (STATUS != lastStatus || lastBlinkTime + LED_BLINK_DELAY < now) {
    bool failSafeTriggered = ((STATUS & STATUS_FAILSAFE) != 0);
    bool leftMotorResponding = ((STATUS & STATUS_LEFT_RESPONDING) != 0);
    bool rightMotorResponding = ((STATUS & STATUS_RIGHT_RESPONDING) != 0);

    strip.clear();

    if (failSafeTriggered) {
      if (!flipFlop) {
        strip.setPixelColor(LED1_FAILSAFE, FAILSAFE_ERROR_COLOR);
        strip.setPixelColor(LED2_FAILSAFE, FAILSAFE_ERROR_COLOR);
      }
    } else {
      strip.setPixelColor(LED1_FAILSAFE, ALL_IS_WELL_COLOR);      
      strip.setPixelColor(LED2_FAILSAFE, ALL_IS_WELL_COLOR);      
    }

    if (!leftMotorResponding) {
      if (!flipFlop) {
        // Try and reconnect to the left motor
        initLeftMotor();
        strip.setPixelColor(LED1_LEFT_MOTOR, LEFT_ERROR_COLOR);
        strip.setPixelColor(LED2_LEFT_MOTOR, LEFT_ERROR_COLOR);
        strip.setPixelColor(LED3_LEFT_MOTOR, LEFT_ERROR_COLOR);
      }
    } else {
      strip.setPixelColor(LED1_LEFT_MOTOR, ALL_IS_WELL_COLOR);      
      strip.setPixelColor(LED2_LEFT_MOTOR, ALL_IS_WELL_COLOR);      
      strip.setPixelColor(LED3_LEFT_MOTOR, ALL_IS_WELL_COLOR);      
    }

    if (!rightMotorResponding) {
      if (!flipFlop) {
        // Try and reconnect to the right motor
        initRightMotor();
        strip.setPixelColor(LED1_RIGHT_MOTOR, RIGHT_ERROR_COLOR);
        strip.setPixelColor(LED2_RIGHT_MOTOR, RIGHT_ERROR_COLOR);
        strip.setPixelColor(LED3_RIGHT_MOTOR, RIGHT_ERROR_COLOR);
      }
    } else {
      strip.setPixelColor(LED1_RIGHT_MOTOR, ALL_IS_WELL_COLOR);      
      strip.setPixelColor(LED2_RIGHT_MOTOR, ALL_IS_WELL_COLOR);      
      strip.setPixelColor(LED3_RIGHT_MOTOR, ALL_IS_WELL_COLOR);      
    }
    strip.show();
    lastStatus = STATUS;
    lastBlinkTime = now;
    flipFlop = !flipFlop;
  }
}

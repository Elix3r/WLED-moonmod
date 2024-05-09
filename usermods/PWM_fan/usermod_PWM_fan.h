#pragma once

#include "wled.h"

#if !defined(USERMOD_DALLASTEMPERATURE) && !defined(USERMOD_SHT)
#error "The 'PWM fan' usermod requires 'Dallas Temperature' or 'SHT' usermod to function properly."
#endif

#ifndef PWMFAN_MQTT_TOPIC
#define PWMFAN_MQTT_TOPIC "wled/PWMFan"
#endif

// Tacho counter
static volatile unsigned long counter_rpm = 0;

// Interrupt for counting fan rotations
static void IRAM_ATTR rpm_fan() {
  counter_rpm++;
}

class PWMFanUsermod : public Usermod {
  private:
    bool initDone = false;
    bool enabled = true;
    unsigned long msLastTachoMeasurement = 0;
    uint16_t last_rpm = 0;
    #ifdef ARDUINO_ARCH_ESP32
    uint8_t pwmChannel = 255;
    #endif
    bool lockFan = false;
    char pwmFanMqttTopic[64];

    #ifdef USERMOD_DALLASTEMPERATURE
    UsermodTemperature* tempUM;
    #elif defined(USERMOD_SHT)
    ShtUsermod* tempUM;
    #endif

    int8_t tachoPin = TACHO_PIN;
    int8_t pwmPin = PWM_PIN;
    uint8_t tachoUpdateSec = 30;
    float targetTemperature = 35.0;
    uint8_t minPWMValuePct = 0;
    uint8_t maxPWMValuePct = 100;
    uint8_t numberOfInterrupsInOneSingleRotation = 2;

    static const uint8_t _pwmMaxValue = 255;
    static const uint8_t _pwmMaxStepCount = 7;
    float _pwmTempStepSize = 0.5f;

    static const char _name[];
    static const char _enabled[];
    static const char _tachoPin[];
    static const char _pwmPin[];
    static const char _temperature[];
    static const char _tachoUpdateSec[];
    static const char _minPWMValuePct[];
    static const char _maxPWMValuePct[];
    static const char _IRQperRotation[];
    static const char _speed[];
    static const char _lock[];

    void initTacho() {
      if (tachoPin < 0 || !pinManager.allocatePin(tachoPin, false, PinOwner::UM_Unspecified)) {
        tachoPin = -1;
        return;
      }
      pinMode(tachoPin, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);
      DEBUG_PRINTLN(F("Tacho successfully initialized."));
    }

    void deinitTacho() {
      if (tachoPin >= 0) {
        detachInterrupt(digitalPinToInterrupt(tachoPin));
        pinManager.deallocatePin(tachoPin, PinOwner::UM_Unspecified);
      }
      tachoPin = -1;
    }

    void initPWMfan() {
      if (pwmPin < 0 || !pinManager.allocatePin(pwmPin, true, PinOwner::UM_Unspecified)) {
        enabled = false;
        return;
      }
      #ifdef ARDUINO_ARCH_ESP32
      pwmChannel = pinManager.allocateLedc(1);
      if (pwmChannel == 255) return;
      ledcSetup(pwmChannel, 25000, 8);
      ledcAttachPin(pwmPin, pwmChannel);
      #else
      analogWriteRange(_pwmMaxValue);
      analogWriteFreq(WLED_PWM_FREQ);
      analogWrite(pwmPin, 0); // Set initial speed to 0
      #endif
      DEBUG_PRINTLN(F("Fan PWM successfully initialized."));
    }

    void deinitPWMfan() {
      if (pwmPin >= 0) {
        #ifdef ARDUINO_ARCH_ESP32
        pinManager.deallocateLedc(pwmChannel, 1);
        #else
        analogWrite(pwmPin, 0);  // Turn off PWM on pin
        #endif
        pinManager.deallocatePin(pwmPin, PinOwner::UM_Unspecified);
      }
      pwmPin = -1;
    }

    void updateFanSpeed(uint8_t pwmValue) {
      if (!enabled || pwmPin < 0) return;
      #ifdef ARDUINO_ARCH_ESP32
      ledcWrite(pwmChannel, pwmValue);
      #else
      analogWrite(pwmPin, pwmValue);
      #endif
      if (WLED_MQTT_CONNECTED) {
        char buff[16];
        sprintf(buff, "%d%%", pwmValue); // Send PWM value as percentage
        mqtt->publish(pwmFanMqttTopic, 0, false, buff);
      }
    }

    float getActualTemperature() {
      #if defined(USERMOD_DALLASTEMPERATURE) || defined(USERMOD_SHT)
      if (tempUM != nullptr)
        return tempUM->getTemperatureC();
      #endif
      return -127.0f; // Default invalid temperature
    }

    void setFanPWMbasedOnTemperature() {
      float temp = getActualTemperature();
      int pwmStepSize = ((maxPWMValuePct - minPWMValuePct) * _pwmMaxValue) / (_pwmMaxStepCount * 100);
      int pwmStep = calculatePwmStep(temp - targetTemperature);
      int pwmMinimumValue = (minPWMValuePct * _pwmMaxValue) / 100;
      updateFanSpeed(pwmMinimumValue + pwmStep * pwmStepSize);
    }

    uint8_t calculatePwmStep(float diffTemp) {
      if ((diffTemp == NAN) || (diffTemp <= -100.0)) {
        DEBUG_PRINTLN(F("WARNING: No temperature value available. Cannot do temperature control. Will set PWM fan to 255."));
        return _pwmMaxStepCount;
      }
      if (diffTemp <= 0) {
        return 0;
      }
      return (uint8_t) min((int) _pwmMaxStepCount, (int) (diffTemp / _pwmTempStepSize) + 1);
    }

  public:
    void setup() override {
      #ifdef USERMOD_DALLASTEMPERATURE
      tempUM = (UsermodTemperature*) usermods.lookup(USERMOD_ID_TEMPERATURE);
      #elif defined(USERMOD_SHT)
      tempUM = (ShtUsermod*) usermods.lookup(USERMOD_ID_SHT);
      #endif
      sprintf(pwmFanMqttTopic, "%s/status", PWMFAN_MQTT_TOPIC);
      initTacho();
      initPWMfan();
      updateFanSpeed((minPWMValuePct * 255) / 100); // Initial fan speed
      initDone = true;
    }

    void loop() {
      if (!enabled || strip.isUpdating()) return;

      unsigned long now = millis();
      if ((now - msLastTachoMeasurement) < (tachoUpdateSec * 1000)) return;

      updateTacho();
      if (!lockFan) setFanPWMbasedOnTemperature();
    }

    uint16_t getId() {
        return USERMOD_ID_PWM_FAN;
    }
};

const char PWMFanUsermod::_name[] PROGMEM = "PWM-fan";
const char PWMFanUsermod::_enabled[] PROGMEM = "enabled";
const char PWMFanUsermod::_tachoPin[] PROGMEM = "tacho-pin";
const char PWMFanUsermod::_pwmPin[] PROGMEM = "PWM-pin";
const char PWMFanUsermod::_temperature[] PROGMEM = "target-temp-C";
const char PWMFanUsermod::_tachoUpdateSec[] PROGMEM = "tacho-update-s";
const char PWMFanUsermod::_minPWMValuePct[] PROGMEM = "min-PWM-percent";
const char PWMFanUsermod::_maxPWMValuePct[] PROGMEM = "max-PWM-percent";
const char PWMFanUsermod::_IRQperRotation[] PROGMEM = "IRQs-per-rotation";
const char PWMFanUsermod::_speed[] PROGMEM = "speed";
const char PWMFanUsermod::_lock[] PROGMEM = "lock";

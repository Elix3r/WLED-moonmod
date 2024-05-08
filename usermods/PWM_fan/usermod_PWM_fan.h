#pragma once

#include "wled.h"

#if !defined(USERMOD_DALLASTEMPERATURE) && !defined(USERMOD_SHT)
#error "The 'PWM fan' usermod requires 'Dallas Temperature' or 'SHT' usermod to function properly."
#endif

// Define MQTT topics
#ifndef PWMFAN_MQTT_TOPIC
#define PWMFAN_MQTT_TOPIC "wled/PWMFan"
#endif

// PWM & tacho code courtesy of @KlausMu
// https://github.com/KlausMu/esp32-fan-controller/tree/main/src
// Adapted for WLED usermod by @blazoncek

#ifndef TACHO_PIN
  #define TACHO_PIN -1
#endif

#ifndef PWM_PIN
  #define PWM_PIN -1
#endif

// Tacho counter
static volatile unsigned long counter_rpm = 0;

// Interrupt counting every rotation of the fan
// https://desire.giesecke.tk/index.php/2018/01/30/change-global-variables-from-isr/
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

    // Configurable parameters
    int8_t tachoPin = TACHO_PIN;
    int8_t pwmPin = PWM_PIN;
    uint8_t tachoUpdateSec = 30;
    float targetTemperature = 35.0;
    uint8_t minPWMValuePct = 0;
    uint8_t maxPWMValuePct = 100;
    uint8_t numberOfInterrupsInOneSingleRotation = 2;

    // Constant values
    static const uint8_t _pwmMaxValue = 255;
    static const uint8_t _pwmMaxStepCount = 7;
    float _pwmTempStepSize = 0.5f;

    void initTacho(void) {
      if (tachoPin < 0 || !pinManager.allocatePin(tachoPin, false, PinOwner::UM_Unspecified)) {
        tachoPin = -1;
        return;
      }
      pinMode(tachoPin, INPUT);
      digitalWrite(tachoPin, HIGH);
      attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);
      DEBUG_PRINTLN(F("Tacho successfully initialized."));
    }

    void deinitTacho(void) {
      if (tachoPin < 0) return;
      detachInterrupt(digitalPinToInterrupt(tachoPin));
      pinManager.deallocatePin(tachoPin, PinOwner::UM_Unspecified);
      tachoPin = -1;
    }

    void updateTacho(void) {
      msLastTachoMeasurement = millis();
      if (tachoPin < 0) return;

      // Start of tacho measurement
      // Detach interrupt while calculating rpm
      detachInterrupt(digitalPinToInterrupt(tachoPin));
      // Calculate rpm
      last_rpm = (counter_rpm * 60) / numberOfInterrupsInOneSingleRotation;
      last_rpm /= tachoUpdateSec;
      // Reset counter
      counter_rpm = 0;
      // Attach interrupt again
      attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);

      // Publish RPM to MQTT
      if (WLED_MQTT_CONNECTED) {
        char buff[16];
        sprintf(buff, "%u RPM", last_rpm);
        mqtt->publish(pwmFanMqttTopic, 0, false, buff);
      }
    }

    // PWM fan initialization
    void initPWMfan(void) {
      if (pwmPin < 0 || !pinManager.allocatePin(pwmPin, true, PinOwner::UM_Unspecified)) {
        enabled = false;
        pwmPin = -1;
        return;
      }

      #ifdef ESP8266
      analogWriteRange(255);
      analogWriteFreq(WLED_PWM_FREQ);
      #else
      pwmChannel = pinManager.allocateLedc(1);
      if (pwmChannel == 255) { // No more free LEDC channels
        deinitPWMfan();
        return;
      }
      // Configure LED PWM functionalities
      ledcSetup(pwmChannel, 25000, 8);
      // Attach the channel to the GPIO to be controlled
      ledcAttachPin(pwmPin, pwmChannel);
      #endif
      DEBUG_PRINTLN(F("Fan PWM successfully initialized."));
    }

    void deinitPWMfan(void) {
      if (pwmPin < 0) return;

      pinManager.deallocatePin(pwmPin, PinOwner::UM_Unspecified);
      #ifdef ARDUINO_ARCH_ESP32
      pinManager.deallocateLedc(pwmChannel, 1);
      #endif
      pwmPin = -1;
    }

    void updateFanSpeed(uint8_t pwmValue) {
      if (!enabled || pwmPin < 0) return;

      #ifdef ESP8266
      analogWrite(pwmPin, pwmValue);
      #else
      ledcWrite(pwmChannel, pwmValue);
      #endif

      // Publish fan speed to MQTT
      if (WLED_MQTT_CONNECTED) {
        char buff[16];
        sprintf(buff, "%d%%", pwmValue); // Send PWM value as percentage
        mqtt->publish(pwmFanMqttTopic, 0, false, buff);
      }
    }

    float getActualTemperature(void) {
      #if defined(USERMOD_DALLASTEMPERATURE) || defined(USERMOD_SHT)
      if (tempUM != nullptr)
        return tempUM->getTemperatureC();
      #endif
      return -127.0f;
    }

    void setFanPWMbasedOnTemperature(void) {
      float temp = getActualTemperature();
      // Dividing minPercent and maxPercent into equal PWM value sizes
      int pwmStepSize = ((maxPWMValuePct - minPWMValuePct) * _pwmMaxValue) / (_pwmMaxStepCount * 100);
      int pwmStep = calculatePwmStep(temp - targetTemperature);
      // Minimum based on full speed - not entered MaxPercent
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
      int calculatedStep = (diffTemp / _pwmTempStepSize) + 1;
      // Anything greater than max step count gets max
      return (uint8_t) min((int) _pwmMaxStepCount, calculatedStep);
    }

  public:
    void setup() override {
      #ifdef USERMOD_DALLASTEMPERATURE
      // This Usermod requires Temperature usermod
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

    // Additional methods (e.g., addToJsonInfo, readFromJsonState) as before
    ...
};


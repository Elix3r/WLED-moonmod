#pragma once

#include "wled.h"

#if !defined(USERMOD_DALLASTEMPERATURE) && !defined(USERMOD_SHT)
#error "The 'PWM fan' usermod requires 'Dallas Temperature' or 'SHT' usermod to function properly."
#endif

#ifndef PWMFAN_MQTT_TOPIC
#define PWMFAN_MQTT_TOPIC "wled/PWMFan"
#endif

// PWM & tacho code courtesy of @KlausMu
// https://github.com/KlausMu/esp32-fan-controller/tree/main/src
// adapted for WLED usermod by @blazoncek

#ifndef TACHO_PIN
#define TACHO_PIN -1
#endif

#ifndef PWM_PIN
#define PWM_PIN -1
#endif

// tacho counter
static volatile unsigned long counter_rpm = 0;

// Interrupt counting every rotation of the fan
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

    // configurable parameters
    int8_t tachoPin = TACHO_PIN;
    int8_t pwmPin = PWM_PIN;
    uint8_t tachoUpdateSec = 30;
    float targetTemperature = 35.0;
    uint8_t minPWMValuePct = 0;
    uint8_t maxPWMValuePct = 100;
    uint8_t numberOfInterrupsInOneSingleRotation = 2;

    // Constant strings to reduce flash memory usage (used more than twice)
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
      pinMode(tachoPin, INPUT);
      digitalWrite(tachoPin, HIGH);
      attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);
      DEBUG_PRINTLN(F("Tacho successfully initialized."));
    }

    void deinitTacho() {
      if (tachoPin < 0) return;
      detachInterrupt(digitalPinToInterrupt(tachoPin));
      pinManager.deallocatePin(tachoPin, PinOwner::UM_Unspecified);
      tachoPin = -1;
    }

    void updateTacho() {
      msLastTachoMeasurement = millis();
      if (tachoPin < 0) return;

      detachInterrupt(digitalPinToInterrupt(tachoPin));
      last_rpm = (counter_rpm * 60) / numberOfInterrupsInOneSingleRotation;
      last_rpm /= tachoUpdateSec;
      counter_rpm = 0;
      attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);

      if (WLED_MQTT_CONNECTED) {
        char buff[16];
        sprintf(buff, "%u RPM", last_rpm);
        mqtt->publish(pwmFanMqttTopic, 0, false, buff);
      }
    }

    void initPWMfan() {
      if (pwmPin < 0 || !pinManager.allocatePin(pwmPin, true, PinOwner::UM_Unspecified)) {
        enabled = false;
        return;
      }

      #ifdef ESP8266
      analogWriteRange(255);
      analogWriteFreq(WLED_PWM_FREQ);
      #else
      pwmChannel = pinManager.allocateLedc(1);
      if (pwmChannel == 255) { //no more free LEDC channels
        deinitPWMfan(); return;
      }
      // configure LED PWM functionalities
      ledcSetup(pwmChannel, 25000, 8);
      // attach the channel to the GPIO to be controlled
      ledcAttachPin(pwmPin, pwmChannel);
      #endif
      DEBUG_PRINTLN(F("Fan PWM successfully initialized."));
    }

    void updateFanSpeed(uint8_t pwmValue) {
      if (!enabled || pwmPin < 0) return;

      #ifdef ESP8266
      analogWrite(pwmPin, pwmValue);
      #else
      ledcWrite(pwmChannel, pwmValue);
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
      return -127.0f;
    }

    void setFanPWMbasedOnTemperature() {
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
      return (uint8_t) min((int) _pwmMaxStepCount, (int) (diffTemp / _pwmTempStepSize) + 1);
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

    void addToJsonInfo(JsonObject& root) {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray infoArr = user.createNestedArray(FPSTR(_name));
      String uiDomString = F("<button class=\"btn btn-xs\" onclick=\"requestJson({'");
      uiDomString += FPSTR(_name);
      uiDomString += F("':{'");
      uiDomString += FPSTR(_enabled);
      uiDomString += F("':");
      uiDomString += enabled ? "false" : "true";
      uiDomString += F("}});\"><i class=\"icons ");
      uiDomString += enabled ? "on" : "off";
      uiDomString += F("\">&#xe08f;</i></button>");
      infoArr.add(uiDomString);

      if (enabled) {
        JsonArray infoArr = user.createNestedArray(F("Manual"));
        String uiDomString = F("<div class=\"slider\"><div class=\"sliderwrap il\"><input class=\"noslide\" onchange=\"requestJson({'");
        uiDomString += FPSTR(_name);
        uiDomString += F("':{'");
        uiDomString += FPSTR(_speed);
        uiDomString += F("':parseInt(this.value)}});\" oninput=\"updateTrail(this);\" max=100 min=0 type=\"range\" value=");
        uiDomString += pwmValuePct;
        uiDomString += F(" /><div class=\"sliderdisplay\"></div></div></div>");
        infoArr.add(uiDomString);

        JsonArray data = user.createNestedArray(F("Speed"));
        if (tachoPin >= 0) {
          data.add(last_rpm);
          data add(F("rpm"));
        } else {
          if (lockFan) data.add(F("locked"));
          else         data.add(F("auto"));
        }
      }
    }

    void readFromJsonState(JsonObject& root) {
      if (!initDone) return;
      JsonObject usermod = root[FPSTR(_name)];
      if (!usermod.isNull()) {
        if (usermod[FPSTR(_enabled)].is<bool>()) {
          enabled = usermod[FPSTR(_enabled)].as<bool>();
          if (!enabled) updateFanSpeed(0);
        }
        if (enabled && !usermod[FPSTR(_speed)].isNull() && usermod[FPSTR(_speed)].is<int>()) {
          pwmValuePct = usermod[FPSTR(_speed)].as<int>();
          updateFanSpeed((constrain(pwmValuePct,0,100) * 255) / 100);
          if (pwmValuePct) lockFan = true;
        }
        if (enabled && !usermod[FPSTR(_lock)].isNull() && usermod[FPSTR(_lock)].is<bool>()) {
          lockFan = usermod[FPSTR(_lock)].as<bool();
        }
      }
    }
};

// strings to reduce flash memory usage (used more than twice)
const char PWMFanUsermod::_name[]           PROGMEM = "PWM-fan";
const char PWMFanUsermod::_enabled[]        PROGMEM = "enabled";
const char PWMFanUsermod::_tachoPin[]       PROGMEM = "tacho-pin";
const char PWMFanUsermod::_pwmPin[]         PROGMEM = "PWM-pin";
const char PWMFanUsermod::_temperature[]    PROGMEM = "target-temp-C";
const char PWMFanUsermod::_tachoUpdateSec[] PROGMEM = "tacho-update-s";
const char PWMFanUsermod::_minPWMValuePct[] PROGMEM = "min-PWM-percent";
const char PWMFanUsermod::_maxPWMValuePct[] PROGMEM = "max-PWM-percent";
const char PWMFanUsermod::_IRQperRotation[] PROGMEM = "IRQs-per-rotation";
const char PWMFanUsermod::_speed[]          PROGMEM = "speed";
const char PWMFanUsermod::_lock[]           PROGMEM = "lock";

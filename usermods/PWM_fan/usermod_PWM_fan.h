#pragma once

#if !defined(USERMOD_DALLASTEMPERATURE) && !defined(USERMOD_SHT)
#error The "PWM fan" usermod requires "Dallas Temperature" or "SHT" usermod to function properly.
#endif

#include "wled.h"

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
    uint8_t numberOfInterrupsInOneSingleRotation = 2;
    uint8_t pwmValuePct = 0;

    static const char _name[];
    static const char _enabled[];
    static const char _tachoPin[];
    static const char _pwmPin[];
    static const char _temperature[];
    static const char _tachoUpdateSec[];
    static const char _minPWMValuePct[];
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
    }

    void initPWMfan() {
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
      if (pwmChannel == 255) return; // No more free LEDC channels
      ledcSetup(pwmChannel, 25000, 8);
      ledcAttachPin(pwmPin, pwmChannel);
      #endif
      DEBUG_PRINTLN(F("Fan PWM successfully initialized."));
    }

    void deinitPWMfan() {
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
      float difftemp = temp - targetTemperature;
      int newPWMvalue = 255;
      int pwmStep = ((100 - minPWMValuePct) * newPWMvalue) / (7*100);
      int pwmMinimumValue = (minPWMValuePct * newPWMvalue) / 100;

      if ((temp == NAN) || (temp <= -100.0)) {
        DEBUG_PRINTLN(F("WARNING: no temperature value available. Cannot do temperature control. Will set PWM fan to 255."));
      } else if (difftemp <= 0.0) {
        newPWMvalue = pwmMinimumValue;
      } else if (difftemp <= 0.5) {
        newPWMvalue = pwmMinimumValue + pwmStep;
      } else if (difftemp <= 1.0) {
        newPWMvalue = pwmMinimumValue + 2*pwmStep;
      } else if (difftemp <= 1.5) {
        newPWMvalue = pwmMinimumValue + 3*pwmStep;
      } else if (difftemp <= 2.0) {
        newPWMvalue = pwmMinimumValue + 4*pwmStep;
      } else if (difftemp <= 2.5) {
        newPWMvalue = pwmMinimumValue + 5*pwmStep;
      } else if (difftemp <= 3.0) {
        newPWMvalue = pwmMinimumValue + 6*pwmStep;
      }
      updateFanSpeed(newPWMvalue);
    }

  public:

    void setup() {
      #ifdef USERMOD_DALLASTEMPERATURE
      tempUM = (UsermodTemperature*) usermods.lookup(USERMOD_ID_TEMPERATURE);
      #elif defined(USERMOD_SHT)
      tempUM = (ShtUsermod*) usermods.lookup(USERMOD_ID_SHT);
      #endif
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
      // Additional MQTT publish logic here
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
          data.add(F("rpm"));
        } else {
          if (lockFan) data.add(F("locked"));
          else data.add(F("auto"));
        }
      }
    }

    void addToConfig(JsonObject& root) {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;
      top[FPSTR(_pwmPin)] = pwmPin;
      top[FPSTR(_tachoPin)] = tachoPin;
      top[FPSTR(_tachoUpdateSec)] = tachoUpdateSec;
      top[FPSTR(_temperature)] = targetTemperature;
      top[FPSTR(_minPWMValuePct)] = minPWMValuePct;
      top[FPSTR(_IRQperRotation)] = numberOfInterrupsInOneSingleRotation;
      DEBUG_PRINTLN(F("Autosave config saved."));
    }

    bool readFromConfig(JsonObject& root) {
      JsonObject top = root[FPSTR(_name)];
      if (top.isNull()) {
        DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
        return false;
      }
      enabled = top[FPSTR(_enabled)] | enabled;
      int8_t newTachoPin = top[FPSTR(_tachoPin)] | tachoPin;
      int8_t newPwmPin = top[FPSTR(_pwmPin)] | pwmPin;
      tachoUpdateSec = top[FPSTR(_tachoUpdateSec)] | tachoUpdateSec;
      tachoUpdateSec = (uint8_t) max(1, (int)tachoUpdateSec);
      targetTemperature = top[FPSTR(_temperature)] | targetTemperature;
      minPWMValuePct = top[FPSTR(_minPWMValuePct)] | minPWMValuePct;
      minPWMValuePct = (uint8_t) min(100, max(0, (int)minPWMValuePct));
      numberOfInterrupsInOneSingleRotation = top[FPSTR(_IRQperRotation)] | numberOfInterrupsInOneSingleRotation;
      numberOfInterrupsInOneSingleRotation = (uint8_t) max(1, (int)numberOfInterrupsInOneSingleRotation);
      if (!initDone) {
        tachoPin = newTachoPin;
        pwmPin = newPwmPin;
        DEBUG_PRINTLN(F(" config loaded."));
      } else {
        DEBUG_PRINTLN(F(" config (re)loaded."));
        if (tachoPin != newTachoPin || pwmPin != newPwmPin) {
          DEBUG_PRINTLN(F("Re-init pins."));
          deinitTacho();
          deinitPWMfan();
          tachoPin = newTachoPin;
          pwmPin = newPwmPin;
          setup();
        }
      }
      return !top[FPSTR(_IRQperRotation)].isNull();
    }

    uint16_t getId() {
      return USERMOD_ID_PWM_FAN;
    }
};

// Static variable definitions
const char PWMFanUsermod::_name[] = "PWM-fan";
const char PWMFanUsermod::_enabled[] = "enabled";
const char PWMFanUsermod::_tachoPin[] = "tacho-pin";
const char PWMFanUsermod::_pwmPin[] = "PWM-pin";
const char PWMFanUsermod::_temperature[] = "target-temp-C";
const char PWMFanUsermod::_tachoUpdateSec[] = "tacho-update-s";
const char PWMFanUsermod::_minPWMValuePct[] = "min-PWM-percent";
const char PWMFanUsermod::_IRQperRotation[] = "IRQs-per-rotation";
const char PWMFanUsermod::_speed[] = "speed";
const char PWMFanUsermod::_lock[] = "lock";

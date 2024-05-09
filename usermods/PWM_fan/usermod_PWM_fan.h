#pragma once

#include "wled.h"

#if !defined(USERMOD_DALLASTEMPERATURE) && !defined(USERMOD_SHT)
#error "The 'PWM fan' usermod requires 'Dallas Temperature' or 'SHT' usermod to function properly."
#endif

#ifndef PWMFAN_MQTT_TOPIC
#define PWMFAN_MQTT_TOPIC "wled/PWMFan"
#endif

#ifndef TACHO_PIN
#define TACHO_PIN -1
#endif

#ifndef PWM_PIN
#define PWM_PIN -1
#endif

static volatile unsigned long counter_rpm = 0;

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
    uint8_t pwmValuePct = 0; // Ensure this is declared at class level

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

    void initTacho(void) {
      if (tachoPin < 0 || !pinManager.allocatePin(tachoPin, false, PinOwner::UM_Unspecified)){
        tachoPin = -1;
        return;
      }
      pinMode(tachoPin, INPUT);
      digitalWrite(tachoPin, HIGH);
      attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);
    }

    void deinitTacho(void) {
      if (tachoPin < 0) return;
      detachInterrupt(digitalPinToInterrupt(tachoPin));
      pinManager.deallocatePin(tachoPin, PinOwner::UM_Unspecified);
      tachoPin = -1;
    }

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
      if (pwmChannel == 255) {
        deinitPWMfan();
        return;
      }
      ledcSetup(pwmChannel, 25000, 8);
      ledcAttachPin(pwmPin, pwmChannel);
      #endif
    }

    void deinitPWMfan(void) {
      if (pwmPin < 0) return;
      #ifdef ARDUINO_ARCH_ESP32
      ledcDetachPin(pwmPin);
      ledcStop(pwmChannel, 0); // Stop with a duty cycle of 0
      #endif
      pinManager.deallocatePin(pwmPin, PinOwner::UM_Unspecified);
      pwmPin = -1;
    }

    void updateTacho(void) {
      msLastTachoMeasurement = millis();
      if (tachoPin < 0) return;
      detachInterrupt(digitalPinToInterrupt(tachoPin));
      last_rpm = (counter_rpm * 60) / numberOfInterrupsInOneSingleRotation;
      last_rpm /= tachoUpdateSec;
      counter_rpm = 0;
      attachInterrupt(digitalPinToInterrupt(tachoPin), rpm_fan, FALLING);
    }

    void updateFanSpeed(uint8_t pwmValue){
      if (!enabled || pwmPin < 0) return;
      #ifdef ESP8266
      analogWrite(pwmPin, pwmValue);
      #else
      ledcWrite(pwmChannel, pwmValue);
      #endif
    }

    float getActualTemperature(void) {
      if (tempUM != nullptr)
        return tempUM->getTemperatureC();
      return -127.0f;
    }

    void setFanPWMbasedOnTemperature(void) {
      float temp = getActualTemperature();
      float difftemp = temp - targetTemperature;
      int newPWMvalue = 255;
      if (difftemp <= 0.0) {
        newPWMvalue = minPWMValuePct * newPWMvalue / 100;
      } else {
        int steps = (int)(difftemp / 0.5f) + 1; // Assuming a step of 0.5 degrees
        newPWMvalue = minPWMValuePct * newPWMvalue / 100 + steps * ((100 - minPWMValuePct) * newPWMvalue / 700); // Assuming 7 steps max
      }
      updateFanSpeed(newPWMvalue);
    }

  public:
    void setup() {
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
      JsonArray infoArr = user.createNestedArray("PWM-fan");
      String uiDomString = "<button class=\"btn btn-xs\" onclick=\"requestJson({'PWM-fan':{'enabled':";
      uiDomString += enabled ? "false" : "true";
      uiDomString += "}});\"><i class=\"icons ";
      uiDomString += enabled ? "on" : "off";
      uiDomString += "\">&#xe08f;</i></button>";
      infoArr.add(uiDomString);

      if (enabled) {
        JsonArray infoArr = user.createNestedArray("Manual");
        String uiDomString = "<div class=\"slider\"><div class=\"sliderwrap il\"><input class=\"noslide\" onchange=\"requestJson({'PWM-fan':{'speed':parseInt(this.value)}});\" oninput=\"updateTrail(this);\" max=100 min=0 type=\"range\" value=";
        uiDomString += String(pwmValuePct);
        uiDomString += " /><div class=\"sliderdisplay\"></div></div></div>";
        infoArr.add(uiDomString);

        JsonArray data = user.createNestedArray("Speed");
        if (tachoPin >= 0) {
          data.add(last_rpm);
          data.add("rpm");
        } else {
          if (lockFan) data.add("locked");
          else         data.add("auto");
        }
      }
    }

    void readFromJsonState(JsonObject& root) {
      if (!initDone) return; // Prevent crash on boot applyPreset()
      JsonObject usermod = root["PWM-fan"];
      if (!usermod.isNull()) {
        if (usermod["enabled"].is<bool>()) {
          enabled = usermod["enabled"].as<bool>();
          if (!enabled) updateFanSpeed(0);
        }
        if (enabled && !usermod["speed"].isNull() && usermod["speed"].is<int>()) {
          pwmValuePct = usermod["speed"].as<int>();
          updateFanSpeed((constrain(pwmValuePct, 0, 100) * 255) / 100);
          if (pwmValuePct) lockFan = true;
        }
        if (enabled && !usermod["lock"].isNull() && usermod["lock"].is<bool>()) {
          lockFan = usermod["lock"].as<bool>();
        }
      }
    }

    uint16_t getId() {
      return USERMOD_ID_PWM_FAN;
    }
};

// Static member initialization
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

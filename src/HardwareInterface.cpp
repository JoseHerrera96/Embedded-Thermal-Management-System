/**
 * @file HardwareInterface.cpp
 * @brief Implementation of Hardware Abstraction Layer
 */

#include "HardwareInterface.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

// ============================================================================
// SimulatedHardware Implementation (for PC testing)
// ============================================================================

SimulatedHardware::SimulatedHardware() 
    : currentFanDuty(0)
    , currentPowerState(PowerState::NORMAL)
    , simulatedTemp(AMBIENT_TEMP)
    , startTime(0)
{
}

void SimulatedHardware::init() {
    startTime = static_cast<uint32_t>(time(nullptr) * 1000);
    simulatedTemp = AMBIENT_TEMP;
    printf("[SIM] Hardware initialized - Ambient: %.1f°C\n", AMBIENT_TEMP);
}

uint16_t SimulatedHardware::readADC() {
    // Simulate thermal dynamics
    uint32_t now = getMillis();
    float dt = (now - startTime) * 0.001f; // seconds
    
    // Heat generation based on power state
    float heatGen = 0.0f;
    switch (currentPowerState) {
        case PowerState::NORMAL:
            heatGen = HEAT_RATE;
            break;
        case PowerState::THROTTLED:
            heatGen = HEAT_RATE * 0.6f;
            break;
        default:
            heatGen = 0.0f;
            break;
    }
    
    // Cooling based on fan duty
    float coolingFactor = (currentFanDuty / 255.0f) * COOL_RATE;
    
    // Temperature physics simulation
    float targetTemp = AMBIENT_TEMP + (heatGen / COOL_RATE) * 50.0f;
    simulatedTemp += (targetTemp - simulatedTemp) * 0.01f;
    simulatedTemp -= coolingFactor;
    
    // Add sensor noise (Gaussian approximation)
    float noise = ((rand() % 100) / 50.0f - 1.0f) * NOISE_AMPLITUDE;
    float noisyTemp = simulatedTemp + noise;
    
    // Convert temperature to ADC value (NTC thermistor voltage divider)
    // R_thermistor = R_nominal * exp(B * (1/T - 1/T_nominal))
    float kelvin = noisyTemp + 273.15f;
    float resistance = 10000.0f * expf(3950.0f * (1.0f/kelvin - 1.0f/298.15f));
    
    // Voltage divider: V_out = V_ref * (R_thermistor / (R_thermistor + R_series))
    float voltage = 3.3f * (resistance / (resistance + 10000.0f));
    
    // ADC conversion (12-bit)
    uint16_t adcValue = static_cast<uint16_t>((voltage / 3.3f) * 4095.0f);
    
    return adcValue;
}

void SimulatedHardware::setFanPWM(uint8_t dutyCycle) {
    currentFanDuty = dutyCycle;
    float percent = (dutyCycle / 255.0f) * 100.0f;
    printf("[SIM] Fan PWM set to %d/255 (%.1f%%)\n", dutyCycle, percent);
}

void SimulatedHardware::setCPUPowerState(PowerState state) {
    if (currentPowerState != state) {
        currentPowerState = state;
        const char* stateStr = "";
        switch (state) {
            case PowerState::NORMAL: stateStr = "NORMAL"; break;
            case PowerState::THROTTLED: stateStr = "THROTTLED"; break;
            case PowerState::CRITICAL: stateStr = "CRITICAL"; break;
            case PowerState::SHUTDOWN: stateStr = "SHUTDOWN"; break;
        }
        printf("[SIM] CPU Power State: %s\n", stateStr);
    }
}

void SimulatedHardware::sendTelemetry(const char* data) {
    printf("[TELEMETRY] %s\n", data);
}

uint32_t SimulatedHardware::getMillis() {
    return static_cast<uint32_t>(time(nullptr) * 1000) - startTime;
}

#ifdef ARDUINO
// ============================================================================
// ArduinoHardware Implementation (for real hardware)
// ============================================================================

ArduinoHardware::ArduinoHardware(uint8_t adc, uint8_t pwm, uint8_t throttle)
    : adcPin(adc)
    , pwmPin(pwm)
    , throttlePin(throttle)
{
}

void ArduinoHardware::init() {
    pinMode(pwmPin, OUTPUT);
    pinMode(throttlePin, OUTPUT);
    analogWrite(pwmPin, 0);
    digitalWrite(throttlePin, LOW);
    
    Serial.begin(115200);
    Serial.println("[HW] Arduino Hardware initialized");
}

uint16_t ArduinoHardware::readADC() {
    // Arduino Uno/Nano: 10-bit ADC (0-1023)
    // Scale to 12-bit for compatibility (0-4095)
    uint16_t raw10bit = analogRead(adcPin);
    return raw10bit << 2; // Scale 10-bit to 12-bit
}

void ArduinoHardware::setFanPWM(uint8_t dutyCycle) {
    analogWrite(pwmPin, dutyCycle);
}

void ArduinoHardware::setCPUPowerState(PowerState state) {
    // Simulate CPU throttle interrupt with GPIO pin
    if (state == PowerState::THROTTLED || state == PowerState::CRITICAL) {
        digitalWrite(throttlePin, HIGH);
    } else {
        digitalWrite(throttlePin, LOW);
    }
}

void ArduinoHardware::sendTelemetry(const char* data) {
    Serial.println(data);
}

uint32_t ArduinoHardware::getMillis() {
    return millis();
}
#endif

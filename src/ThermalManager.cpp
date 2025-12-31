/**
 * @file ThermalManager.cpp
 * @brief Implementation of Thermal Management System
 */

#include "ThermalManager.hpp"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// Constructor & Initialization
// ============================================================================

ThermalManager::ThermalManager(IHardwareInterface* hw)
    : hardware(hw)
    , currentState(ThermalState::IDLE)
    , previousState(ThermalState::IDLE)
    , currentPowerState(PowerState::NORMAL)
    , filteredTemp(25.0f)
    , bufferIndex(0)
    , bufferFilled(false)
    , currentFanDuty(0)
    , lastSampleTime(0)
{
    // Initialize filter buffer
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        tempBuffer[i] = 25.0f; // Assume ambient start
    }
}

void ThermalManager::init() {
    hardware->init();
    
    // Initial sensor reading
    filteredTemp = readTemperature();
    
    // Pre-fill buffer for stable startup
    for (uint8_t i = 0; i < FILTER_SIZE; i++) {
        tempBuffer[i] = filteredTemp;
    }
    bufferFilled = true;
    
    // Set initial state
    updateState();
    executeStateActions();
    
    lastSampleTime = hardware->getMillis();
}

// ============================================================================
// Main Update Loop (Non-Blocking)
// ============================================================================

bool ThermalManager::update() {
    uint32_t now = hardware->getMillis();
    
    // Check if sample interval elapsed (non-blocking design)
    if ((now - lastSampleTime) < SAMPLE_INTERVAL_MS) {
        return false; // Not time to sample yet
    }
    
    lastSampleTime = now;
    
    // Read and filter temperature
    float rawTemp = readTemperature();
    filteredTemp = applyFilter(rawTemp);
    
    // Update FSM
    previousState = currentState;
    updateState();
    
    // Execute state-specific actions
    executeStateActions();
    
    // Send telemetry
    sendTelemetry();
    
    return true; // Data processed
}

// ============================================================================
// Temperature Sensing (Steinhart-Hart Equation)
// ============================================================================

float ThermalManager::readTemperature() {
    uint16_t adcValue = hardware->readADC();
    
    // Convert ADC to voltage
    float voltage = (adcValue / static_cast<float>(ADC_MAX)) * VREF;
    
    // Calculate thermistor resistance from voltage divider
    // V_out = V_ref * (R_thermistor / (R_thermistor + R_series))
    // Solving for R_thermistor:
    // R_thermistor = R_series * (V_out / (V_ref - V_out))
    
    if (voltage >= VREF) {
        voltage = VREF - 0.001f; // Prevent division by zero
    }
    
    float resistance = SERIES_RESISTOR * (voltage / (VREF - voltage));
    
    // Steinhart-Hart simplified (Beta parameter equation)
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    // Where T and T0 are in Kelvin
    
    float steinhart;
    steinhart = resistance / THERMISTOR_NOMINAL;     // (R/R0)
    steinhart = logf(steinhart);                      // ln(R/R0)
    steinhart /= B_COEFFICIENT;                       // 1/B * ln(R/R0)
    steinhart += 1.0f / (TEMP_NOMINAL + 273.15f);    // + (1/T0)
    steinhart = 1.0f / steinhart;                     // Invert
    steinhart -= 273.15f;                             // Convert Kelvin to Celsius
    
    return steinhart;
}

// ============================================================================
// Moving Average Filter (Noise Reduction)
// ============================================================================

float ThermalManager::applyFilter(float newSample) {
    // Add new sample to circular buffer
    tempBuffer[bufferIndex] = newSample;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    
    if (!bufferFilled && bufferIndex == 0) {
        bufferFilled = true;
    }
    
    // Calculate average
    float sum = 0.0f;
    uint8_t count = bufferFilled ? FILTER_SIZE : bufferIndex;
    
    for (uint8_t i = 0; i < count; i++) {
        sum += tempBuffer[i];
    }
    
    return sum / count;
}

// ============================================================================
// Finite State Machine (FSM) with Hysteresis
// ============================================================================

void ThermalManager::updateState() {
    ThermalState newState = currentState;
    
    // State transitions with hysteresis to prevent oscillation
    switch (currentState) {
        case ThermalState::IDLE:
            if (filteredTemp >= TEMP_IDLE_MAX) {
                newState = ThermalState::ACTIVE;
            }
            break;
            
        case ThermalState::ACTIVE:
            if (filteredTemp < (TEMP_IDLE_MAX - HYSTERESIS)) {
                newState = ThermalState::IDLE;
            } else if (filteredTemp >= TEMP_ACTIVE_MAX) {
                newState = ThermalState::THROTTLING;
            }
            break;
            
        case ThermalState::THROTTLING:
            if (filteredTemp < (TEMP_ACTIVE_MAX - HYSTERESIS)) {
                newState = ThermalState::ACTIVE;
            } else if (filteredTemp >= TEMP_THROTTLE_MAX) {
                newState = ThermalState::CRITICAL;
            }
            break;
            
        case ThermalState::CRITICAL:
            // Once critical, require temperature to drop significantly
            // before returning to throttling
            if (filteredTemp < (TEMP_THROTTLE_MAX - HYSTERESIS * 2)) {
                newState = ThermalState::THROTTLING;
            }
            break;
    }
    
    currentState = newState;
}

// ============================================================================
// State Action Execution
// ============================================================================

void ThermalManager::executeStateActions() {
    switch (currentState) {
        case ThermalState::IDLE:
            // Low power mode - fan off
            currentFanDuty = 0;
            currentPowerState = PowerState::NORMAL;
            hardware->setFanPWM(0);
            hardware->setCPUPowerState(PowerState::NORMAL);
            break;
            
        case ThermalState::ACTIVE:
            // Proportional fan control
            currentFanDuty = calculateFanDuty(filteredTemp);
            currentPowerState = PowerState::NORMAL;
            hardware->setFanPWM(currentFanDuty);
            hardware->setCPUPowerState(PowerState::NORMAL);
            break;
            
        case ThermalState::THROTTLING:
            // Maximum cooling + CPU throttle
            currentFanDuty = 255; // 100% fan
            currentPowerState = PowerState::THROTTLED;
            hardware->setFanPWM(255);
            hardware->setCPUPowerState(PowerState::THROTTLED);
            break;
            
        case ThermalState::CRITICAL:
            // Emergency shutdown
            currentFanDuty = 255; // Keep fan running
            currentPowerState = PowerState::CRITICAL;
            hardware->setFanPWM(255);
            hardware->setCPUPowerState(PowerState::CRITICAL);
            break;
    }
}

// ============================================================================
// Fan Control Algorithm
// ============================================================================

uint8_t ThermalManager::calculateFanDuty(float temp) {
    // Linear mapping: 40°C -> 30% duty, 70°C -> 100% duty
    const float MIN_TEMP = TEMP_IDLE_MAX;
    const float MAX_TEMP = TEMP_ACTIVE_MAX;
    const uint8_t MIN_DUTY = 77;  // 30% of 255
    const uint8_t MAX_DUTY = 255; // 100%
    
    if (temp <= MIN_TEMP) {
        return MIN_DUTY;
    }
    if (temp >= MAX_TEMP) {
        return MAX_DUTY;
    }
    
    // Linear interpolation
    float ratio = (temp - MIN_TEMP) / (MAX_TEMP - MIN_TEMP);
    uint8_t duty = static_cast<uint8_t>(MIN_DUTY + ratio * (MAX_DUTY - MIN_DUTY));
    
    return duty;
}

// ============================================================================
// Telemetry (JSON Output)
// ============================================================================

void ThermalManager::sendTelemetry() {
    char buffer[256];
    
    // Format JSON telemetry
    snprintf(buffer, sizeof(buffer),
        "{\"temp_c\":%.2f,\"fan_duty_cycle\":%d,\"fan_percent\":%.1f,\"pwr_state\":\"%s\",\"thermal_state\":\"%s\"}",
        filteredTemp,
        currentFanDuty,
        (currentFanDuty / 255.0f) * 100.0f,
        powerStateToString(currentPowerState),
        thermalStateToString(currentState)
    );
    
    hardware->sendTelemetry(buffer);
}

// ============================================================================
// Helper Functions
// ============================================================================

const char* ThermalManager::powerStateToString(PowerState state) {
    switch (state) {
        case PowerState::NORMAL:    return "NORMAL";
        case PowerState::THROTTLED: return "THROTTLED";
        case PowerState::CRITICAL:  return "CRITICAL";
        case PowerState::SHUTDOWN:  return "SHUTDOWN";
        default:                    return "UNKNOWN";
    }
}

const char* ThermalManager::thermalStateToString(ThermalState state) {
    switch (state) {
        case ThermalState::IDLE:       return "IDLE";
        case ThermalState::ACTIVE:     return "ACTIVE";
        case ThermalState::THROTTLING: return "THROTTLING";
        case ThermalState::CRITICAL:   return "CRITICAL";
        default:                       return "UNKNOWN";
    }
}

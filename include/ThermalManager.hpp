/**
 * @file ThermalManager.hpp
 * @brief High-Performance Thermal Management System for Embedded Processors
 * 
 * Features:
 * - Steinhart-Hart NTC thermistor conversion
 * - Moving average filter (noise reduction)
 * - FSM-based thermal control with hysteresis
 * - Non-blocking, interrupt-driven architecture
 * - JSON telemetry output
 * 
 * @author Jose H
 * @date December 2025
 */

#ifndef THERMAL_MANAGER_HPP
#define THERMAL_MANAGER_HPP

#include "HardwareInterface.hpp"
#include <stdint.h>

/**
 * @enum ThermalState
 * @brief Finite State Machine states for thermal control
 */
enum class ThermalState : uint8_t {
    IDLE,        // T < 40°C - Passive cooling, low power
    ACTIVE,      // 40°C ≤ T < 70°C - Proportional fan control
    THROTTLING,  // 70°C ≤ T < 90°C - CPU frequency reduction
    CRITICAL     // T ≥ 90°C - Emergency shutdown
};

/**
 * @class ThermalManager
 * @brief Core thermal management controller
 * 
 * Design Notes:
 * - Uses Look-Up Table (LUT) for Steinhart-Hart to reduce computation
 * - Non-blocking design suitable for RTOS integration
 * - Configurable thermal thresholds via constants
 */
class ThermalManager {
public:
    // === Thermal Thresholds (Configurable) ===
    static constexpr float TEMP_IDLE_MAX       = 40.0f;  // °C
    static constexpr float TEMP_ACTIVE_MAX     = 70.0f;  // °C - Tj throttle start
    static constexpr float TEMP_THROTTLE_MAX   = 90.0f;  // °C - Tj critical
    static constexpr float HYSTERESIS          = 2.0f;   // °C - prevent oscillation
    
    // === NTC Thermistor Parameters (10k @ 25°C) ===
    static constexpr float SERIES_RESISTOR     = 10000.0f; // Ohms
    static constexpr float THERMISTOR_NOMINAL  = 10000.0f; // Ohms @ 25°C
    static constexpr float TEMP_NOMINAL        = 25.0f;    // °C
    static constexpr float B_COEFFICIENT       = 3950.0f;  // Beta value
    
    // === ADC Configuration ===
    static constexpr uint16_t ADC_MAX          = 4095;     // 12-bit ADC
    static constexpr float VREF                = 3.3f;     // Volts
    
    // === Filter Configuration ===
    static constexpr uint8_t FILTER_SIZE       = 10;       // Moving average window
    
    // === Timing ===
    static constexpr uint32_t SAMPLE_INTERVAL_MS = 100;    // 10 Hz sample rate
    
    /**
     * @brief Constructor
     * @param hw Pointer to hardware abstraction layer
     */
    explicit ThermalManager(IHardwareInterface* hw);
    
    /**
     * @brief Initialize thermal management system
     * Must be called once before update() loop
     */
    void init();
    
    /**
     * @brief Main update function - call in loop()
     * Non-blocking: only processes if sample interval elapsed
     * @return true if new data was processed
     */
    bool update();
    
    /**
     * @brief Get current temperature reading
     * @return Temperature in Celsius
     */
    float getCurrentTemp() const { return filteredTemp; }
    
    /**
     * @brief Get current thermal state
     * @return Current FSM state
     */
    ThermalState getState() const { return currentState; }
    
    /**
     * @brief Get current fan duty cycle
     * @return PWM duty (0-255)
     */
    uint8_t getFanDuty() const { return currentFanDuty; }
    
    /**
     * @brief Get current power state
     * @return CPU power state
     */
    PowerState getPowerState() const { return currentPowerState; }

private:
    // === Hardware Interface ===
    IHardwareInterface* hardware;
    
    // === State Variables ===
    ThermalState currentState;
    ThermalState previousState;
    PowerState currentPowerState;
    
    // === Sensor Data ===
    float filteredTemp;
    float tempBuffer[FILTER_SIZE];
    uint8_t bufferIndex;
    bool bufferFilled;
    
    // === Control Output ===
    uint8_t currentFanDuty;
    
    // === Timing ===
    uint32_t lastSampleTime;
    
    // === Private Methods ===
    
    /**
     * @brief Read ADC and convert to temperature
     * Uses optimized Steinhart-Hart equation
     * @return Temperature in Celsius
     */
    float readTemperature();
    
    /**
     * @brief Apply moving average filter
     * @param newSample Latest temperature reading
     * @return Filtered temperature
     */
    float applyFilter(float newSample);
    
    /**
     * @brief Update FSM state based on temperature
     * Implements hysteresis to prevent rapid state changes
     */
    void updateState();
    
    /**
     * @brief Execute actions for current state
     * Controls fan PWM and CPU power state
     */
    void executeStateActions();
    
    /**
     * @brief Calculate fan duty cycle for ACTIVE state
     * Linear mapping: 40°C = 30% duty, 70°C = 100% duty
     * @param temp Current temperature
     * @return PWM duty cycle (0-255)
     */
    uint8_t calculateFanDuty(float temp);
    
    /**
     * @brief Send telemetry data via Serial
     * Format: {"temp_c": 65.2, "fan_duty_cycle": 75, "pwr_state": "THROTTLED"}
     */
    void sendTelemetry();
    
    /**
     * @brief Convert power state to string
     * @param state Power state enum
     * @return String representation
     */
    const char* powerStateToString(PowerState state);
    
    /**
     * @brief Convert thermal state to string
     * @param state Thermal state enum
     * @return String representation
     */
    const char* thermalStateToString(ThermalState state);
};

#endif // THERMAL_MANAGER_HPP

/**
 * @file HardwareInterface.hpp
 * @brief Hardware Abstraction Layer for Thermal Management System
 * 
 * This HAL provides platform-independent access to:
 * - ADC peripherals (temperature sensor)
 * - PWM outputs (fan control)
 * - GPIO (power management signals)
 * - Timer interrupts (non-blocking sampling)
 * 
 * Supports both real hardware (Arduino/STM32) and PC simulation
 */

#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <stdint.h>

/**
 * @enum PowerState
 * @brief CPU power states for thermal throttling
 */
enum class PowerState : uint8_t {
    NORMAL,      // Full performance
    THROTTLED,   // Reduced frequency/voltage
    CRITICAL,    // Emergency shutdown initiated
    SHUTDOWN     // System halted
};

/**
 * @class IHardwareInterface
 * @brief Abstract interface for hardware operations
 * 
 * Design Pattern: Strategy Pattern - allows swapping real hardware
 * with simulation without changing ThermalManager logic
 */
class IHardwareInterface {
public:
    virtual ~IHardwareInterface() = default;
    
    /**
     * @brief Read ADC value from temperature sensor
     * @return Raw ADC value (0-4095 for 12-bit ADC, 0-1023 for 10-bit)
     */
    virtual uint16_t readADC() = 0;
    
    /**
     * @brief Set fan PWM duty cycle
     * @param dutyCycle PWM duty (0-255, where 255 = 100%)
     */
    virtual void setFanPWM(uint8_t dutyCycle) = 0;
    
    /**
     * @brief Trigger CPU throttling interrupt
     * @param state Target power state
     */
    virtual void setCPUPowerState(PowerState state) = 0;
    
    /**
     * @brief Send telemetry data via Serial/UART
     * @param data JSON string to transmit
     */
    virtual void sendTelemetry(const char* data) = 0;
    
    /**
     * @brief Get current timestamp (milliseconds)
     * @return Monotonic timestamp for timing operations
     */
    virtual uint32_t getMillis() = 0;
    
    /**
     * @brief Initialize hardware peripherals
     */
    virtual void init() = 0;
};

/**
 * @class SimulatedHardware
 * @brief Software simulation for testing on PC without hardware
 * 
 * Generates realistic sensor data with noise and drift
 * Perfect for developing/debugging the thermal algorithm
 */
class SimulatedHardware : public IHardwareInterface {
private:
    uint8_t currentFanDuty;
    PowerState currentPowerState;
    float simulatedTemp;  // Simulated die temperature
    uint32_t startTime;
    
    // Simulation parameters
    static constexpr float AMBIENT_TEMP = 25.0f;
    static constexpr float HEAT_RATE = 0.02f;     // °C/ms when CPU active
    static constexpr float COOL_RATE = 0.015f;    // °C/ms with fan
    static constexpr float NOISE_AMPLITUDE = 0.5f; // Sensor noise
    
public:
    SimulatedHardware();
    
    uint16_t readADC() override;
    void setFanPWM(uint8_t dutyCycle) override;
    void setCPUPowerState(PowerState state) override;
    void sendTelemetry(const char* data) override;
    uint32_t getMillis() override;
    void init() override;
    
    // Simulation helpers
    void setSimulatedTemp(float temp) { simulatedTemp = temp; }
    float getSimulatedTemp() const { return simulatedTemp; }
};

#ifdef ARDUINO
/**
 * @class ArduinoHardware
 * @brief Real hardware implementation for Arduino platform
 */
class ArduinoHardware : public IHardwareInterface {
private:
    const uint8_t adcPin;
    const uint8_t pwmPin;
    const uint8_t throttlePin;
    
public:
    ArduinoHardware(uint8_t adc, uint8_t pwm, uint8_t throttle);
    
    uint16_t readADC() override;
    void setFanPWM(uint8_t dutyCycle) override;
    void setCPUPowerState(PowerState state) override;
    void sendTelemetry(const char* data) override;
    uint32_t getMillis() override;
    void init() override;
};
#endif

#endif // HARDWARE_INTERFACE_HPP

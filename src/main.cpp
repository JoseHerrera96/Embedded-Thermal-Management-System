/**
 * @file main.cpp
 * @brief Embedded Thermal Management System - Main Application
 * 
 * This is the entry point for the thermal management system.
 * Demonstrates non-blocking, interrupt-driven architecture suitable
 * for real-time embedded systems.
 * 
 * ============================================================================
 * HARDWARE SCHEMATIC & ELECTRICAL SPECIFICATIONS
 * ============================================================================
 * 
 * COMPONENT LIST:
 * ---------------
 * [1] Arduino Uno/Nano (ATmega328P) or equivalent
 *     - Operating Voltage: 5V
 *     - ADC Resolution: 10-bit (0-1023), internally scaled to 12-bit
 *     - PWM Frequency: ~490 Hz (Timer 1), configurable to 25 kHz
 *     - Logic Level: 5V TTL
 * 
 * [2] NTC Thermistor (Negative Temperature Coefficient)
 *     - Resistance @ 25°C (R25): 10kΩ ±1%
 *     - Beta Coefficient (B25/85): 3950K ±1%
 *     - Temperature Range: -40°C to +125°C
 *     - Max Power Dissipation: 100mW
 *     - Recommended: EPCOS B57891M0103J000 or equivalent
 * 
 * [3] Series Resistor (Voltage Divider)
 *     - Resistance: 10kΩ ±1% (Metal Film)
 *     - Power Rating: 1/4W (0.25W)
 *     - Temperature Coefficient: ±100ppm/°C
 * 
 * [4] Cooling Fan (12V DC Brushless)
 *     - Operating Voltage: 12V DC (nominal)
 *     - Voltage Range: 7-13.8V
 *     - Current Draw: 80-200mA (typical)
 *     - PWM Control: 25 kHz (recommended for silent operation)
 *     - Connector: 3-pin (PWM) or 2-pin (simple DC)
 * 
 * [5] N-Channel MOSFET (Fan Driver) - Optional but recommended
 *     - Part: IRLZ44N or 2N7000 (logic-level)
 *     - Vgs(th): 1-2V (turns on with 5V logic)
 *     - Rds(on): <0.1Ω @ Vgs=5V
 *     - Id(max): 500mA+ (adequate for fan load)
 *     - Power Dissipation: Calculate P = I²×Rds = (0.2A)²×0.1Ω = 4mW (no heatsink needed)
 * 
 * [6] Flyback Diode (Inductive Load Protection)
 *     - Part: 1N4007 or 1N5819 (Schottky - faster recovery)
 *     - Vr(max): 50V minimum
 *     - If(avg): 200mA minimum
 *     - Purpose: Suppresses voltage spikes when fan is switched off
 * 
 * [7] Pull-down Resistor (MOSFET Gate)
 *     - Resistance: 10kΩ
 *     - Purpose: Ensures fan stays OFF when Arduino boots/resets
 * 
 * ============================================================================
 * CIRCUIT DIAGRAM (ASCII Schematic)
 * ============================================================================
 * 
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                    TEMPERATURE SENSOR CIRCUIT                           │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 *                    Arduino Uno/Nano
 *                    ┌─────────────┐
 *                    │             │
 *     VCC (5V) ──────┤ 5V          │
 *        │           │             │
 *       [R1]         │             │
 *      10kΩ          │             │
 *     ±1% MF         │             │
 *        │           │             │
 *        ├───────────┤ A0 (ADC0)   │  ← Temperature Sensing
 *        │           │             │
 *      [NTC]         │             │
 *     10kΩ @25°C     │             │
 *     β=3950K        │             │
 *        │           │             │
 *       GND ─────────┤ GND         │
 *                    │             │
 *                    └─────────────┘
 * 
 * VOLTAGE DIVIDER EQUATION:
 * -------------------------
 * V(A0) = VCC × (R_NTC / (R1 + R_NTC))
 * 
 * At 25°C: V(A0) = 5V × (10kΩ / 20kΩ) = 2.5V → ADC ≈ 512 (10-bit)
 * At 50°C: V(A0) ≈ 1.9V (R_NTC ≈ 3.6kΩ)
 * At  0°C: V(A0) ≈ 3.5V (R_NTC ≈ 27kΩ)
 * 
 * 
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        FAN CONTROL CIRCUIT                              │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 *                    Arduino Uno/Nano               12V DC Supply
 *                    ┌─────────────┐                    +12V
 *                    │             │                      │
 *                    │         D9  ├────┬─[R_gate]─┬     │
 *                    │        (PWM)│    │  1kΩ     │     │
 *                    │             │  [R_pd]       │     │
 *                    │             │  10kΩ      ┌──┴──┐  │
 *                    │             │    │       │ G   │  │
 *                    │             │   GND      │     │  │       ┌─────────┐
 *                    │             │        Q1  │  D  ├──┴───┬───┤ FAN +   │
 *                    │             │      MOSFET│  S  │      │   │ (12V DC)│
 *                    │             │   IRLZ44N  └──┬──┘      │   │         │
 *                    │             │               │      [D_fly] │         │
 *                    │         GND ├───────────────┴─────────┤───┤ FAN -   │
 *                    │             │                      │  ▼   └─────────┘
 *                    └─────────────┘                     GND 1N4007
 * 
 * PWM CONTROL THEORY:
 * -------------------
 * - 0% Duty Cycle:   Gate = 0V → MOSFET OFF → Fan OFF (0V)
 * - 50% Duty Cycle:  Gate toggles → Average voltage ≈ 6V → Fan half speed
 * - 100% Duty Cycle: Gate = 5V → MOSFET ON → Fan full speed (12V)
 * 
 * MOSFET Power Dissipation (Worst Case @ 200mA):
 * P_loss = I² × Rds(on) = (0.2A)² × 0.1Ω = 4mW (no heatsink required)
 * 
 * 
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                    CPU THROTTLE SIGNAL (Optional)                       │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 *                    Arduino
 *                    ┌─────────────┐
 *                    │             │
 *                    │         D8  ├──────┬─── To external CPU/MCU IRQ pin
 *                    │    (GPIO)   │      │
 *                    │             │     LED (Debug Indicator)
 *                    │             │      ├─[R]─ 220Ω ─ [LED] ─ GND
 *                    │             │      │
 *                    └─────────────┘     GND
 * 
 * SIGNAL BEHAVIOR:
 * ----------------
 * - LOW (0V):  Normal CPU operation
 * - HIGH (5V): Thermal throttling active → Reduce CPU frequency/voltage
 * 
 * ============================================================================
 * PIN ASSIGNMENT TABLE
 * ============================================================================
 * 
 * │ Arduino Pin │ Function        │ Direction │ Electrical Specs            │
 * ├─────────────┼─────────────────┼───────────┼─────────────────────────────┤
 * │ A0          │ Temp Sensor     │ Analog IN │ 0-5V, 10-bit ADC (scaled)   │
 * │ D9 (OC1A)   │ Fan PWM         │ PWM OUT   │ 0-5V, 490Hz (Timer1)        │
 * │ D8          │ Throttle Signal │ Digital   │ 0V=Normal, 5V=Throttle      │
 * │ 5V          │ Power Supply    │ Power OUT │ Max 200mA from USB          │
 * │ GND         │ Ground          │ Ground    │ Common reference            │
 * 
 * ============================================================================
 * CALIBRATION & TESTING PROCEDURE
 * ============================================================================
 * 
 * 1. THERMISTOR CALIBRATION:
 *    - Measure R_NTC at known temperatures (0°C ice bath, 25°C room, 100°C boiling)
 *    - Verify Beta coefficient: β = (T1×T2)/(T2-T1) × ln(R1/R2)
 *    - Adjust B_COEFFICIENT in ThermalManager.hpp if needed
 * 
 * 2. ADC VERIFICATION:
 *    - At 25°C, ADC should read ≈ 512 (10-bit) or ≈ 2048 (12-bit scaled)
 *    - If off by >5%, check series resistor tolerance or supply voltage
 * 
 * 3. PWM FREQUENCY TUNING (for silent fan operation):
 *    - Add to setup(): TCCR1B = (TCCR1B & 0xF8) | 0x01; // Set Timer1 to 31.25kHz
 *    - Audible noise occurs at <20 kHz
 * 
 * 4. FAN LOAD TEST:
 *    - Measure current with multimeter: I_fan = V_supply / R_load
 *    - Verify MOSFET doesn't overheat (touch test - should be cool)
 * 
 * ============================================================================
 * SAFETY CONSIDERATIONS
 * ============================================================================
 * 
 * ⚠️  CAUTION - READ BEFORE POWERING:
 * 
 * 1. Reverse Polarity Protection: Add 1N4007 diode in series with 12V input
 * 2. Overcurrent Protection: Use 500mA fuse on 12V rail
 * 3. Thermal Runaway: If temp exceeds 100°C, system enters CRITICAL shutdown
 * 4. ESD Protection: Use anti-static wrist strap when handling thermistor
 * 5. Power Supply Isolation: Do NOT connect 12V fan supply to Arduino Vin
 *    (can cause ground loops - use separate regulated supply)
 * 
 * ============================================================================
 * BUILD CONFIGURATIONS
 * ============================================================================
 * 
 * For simulation (no hardware): Build with -DSIMULATION flag
 * For Arduino hardware: Build normally with Arduino IDE or PlatformIO
 * 
 * @author Jose H
 */

#include "ThermalManager.hpp"
#include "HardwareInterface.hpp"

#ifndef ARDUINO
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#endif

// ============================================================================
// Platform Selection
// ============================================================================

#ifdef ARDUINO
// Real Arduino Hardware
ArduinoHardware hardware(A0, 9, 8); // ADC=A0, PWM=D9, Throttle=D8
#else
// PC Simulation
SimulatedHardware hardware;
#endif

// Thermal Manager instance
ThermalManager thermalManager(&hardware);

// ============================================================================
// Arduino Setup (or main initialization)
// ============================================================================

void setup() {
    // Initialize thermal management system
    thermalManager.init();
    
    #ifndef ARDUINO
    printf("\n");
    printf("========================================\n");
    printf(" Thermal Management System - Simulation\n");
    printf("========================================\n");
    printf("Thermal Thresholds:\n");
    printf("  IDLE:       < %.0f°C\n", ThermalManager::TEMP_IDLE_MAX);
    printf("  ACTIVE:     %.0f-%.0f°C\n", ThermalManager::TEMP_IDLE_MAX, ThermalManager::TEMP_ACTIVE_MAX);
    printf("  THROTTLING: %.0f-%.0f°C\n", ThermalManager::TEMP_ACTIVE_MAX, ThermalManager::TEMP_THROTTLE_MAX);
    printf("  CRITICAL:   > %.0f°C\n", ThermalManager::TEMP_THROTTLE_MAX);
    printf("========================================\n\n");
    #endif
}

// ============================================================================
// Arduino Loop (or main loop)
// ============================================================================

void loop() {
    // Non-blocking update - only processes when sample interval elapses
    thermalManager.update();
    
    // In a real system, other tasks could run here
    // Example: Process UART commands, update display, etc.
    
    #ifndef ARDUINO
    // Simulate some delay for PC simulation
    // (In real hardware, this would be other work or sleep mode)
    #ifdef _WIN32
    // Windows
    Sleep(10); // Sleep for 10ms
    #else
    // POSIX
    usleep(10000); // Sleep for 10ms
    #endif
    #endif
}

// ============================================================================
// Main Entry Point (for PC simulation)
// ============================================================================

#ifndef ARDUINO
volatile bool running = true;

void signalHandler(int signum) {
    printf("\n\n[SHUTDOWN] Thermal management system halted\n");
    running = false;
}

int main() {
    // Setup signal handler for clean exit (Ctrl+C)
    signal(SIGINT, signalHandler);
    
    // Initialize system
    setup();
    
    // Simulate different thermal scenarios
    printf("[DEMO] Starting thermal scenario simulation...\n\n");
    
    uint32_t startTime = hardware.getMillis();
    uint32_t lastScenarioChange = startTime;
    int scenario = 0;
    
    // Main loop
    while (running) {
        loop();
        
        // Change temperature scenario every 10 seconds for demonstration
        uint32_t now = hardware.getMillis();
        if ((now - lastScenarioChange) > 10000) {
            lastScenarioChange = now;
            scenario++;
            
            switch (scenario % 5) {
                case 0:
                    printf("\n[SCENARIO] Setting temperature to 35°C (IDLE)\n");
                    hardware.setSimulatedTemp(35.0f);
                    break;
                case 1:
                    printf("\n[SCENARIO] Setting temperature to 55°C (ACTIVE)\n");
                    hardware.setSimulatedTemp(55.0f);
                    break;
                case 2:
                    printf("\n[SCENARIO] Setting temperature to 75°C (THROTTLING)\n");
                    hardware.setSimulatedTemp(75.0f);
                    break;
                case 3:
                    printf("\n[SCENARIO] Setting temperature to 92°C (CRITICAL)\n");
                    hardware.setSimulatedTemp(92.0f);
                    break;
                case 4:
                    printf("\n[SCENARIO] Cooling down to 30°C\n");
                    hardware.setSimulatedTemp(30.0f);
                    break;
            }
        }
        
        // Run for maximum 60 seconds in demo mode
        if ((now - startTime) > 60000) {
            printf("\n[DEMO] Simulation complete (60s elapsed)\n");
            break;
        }
    }
    
    printf("\nProgram terminated.\n");
    return 0;
}
#endif

# Embedded Thermal Management System

A professional-grade thermal management controller for high-performance embedded processors. Implements real-world thermal control techniques used in CPU/GPU power management systems.

## 🎯 Project Overview

This project focus on **embedded systems thermal management** by implementing a complete software solution for monitoring junction temperature (Tj) and controlling cooling systems. The architecture mirrors techniques used in modern processors like Intel's Dynamic Voltage and Frequency Scaling (DVFS) and AMD's Precision Boost.

### Key Features

- **Steinhart-Hart NTC Thermistor Conversion** - Accurate temperature sensing with industry-standard equation
- **Moving Average Filter** - 10-sample window for sensor noise reduction
- **Finite State Machine (FSM)** - State-based thermal control with hysteresis
- **Non-blocking Architecture** - Interrupt-driven design suitable for RTOS integration
- **Hardware Abstraction Layer (HAL)** - Platform-independent code (Arduino, STM32, PC simulation)
- **JSON Telemetry Output** - Real-time monitoring via UART/Serial

---

## 🔥 Thermal Control States

| State | Temperature Range | Action | CPU Power State |
|-------|------------------|--------|----------------|
| **IDLE** | < 40°C | Fan OFF, passive cooling | Normal |
| **ACTIVE** | 40°C - 70°C | Proportional fan control (30-100% PWM) | Normal |
| **THROTTLING** | 70°C - 90°C | Fan at 100%, reduce CPU frequency | Throttled |
| **CRITICAL** | > 90°C | Emergency shutdown initiated | Critical |

### Hysteresis Implementation

A **2°C hysteresis** prevents rapid state oscillation at threshold boundaries:

```
IDLE ──(T ≥ 40°C)──> ACTIVE ──(T < 38°C)──> IDLE
ACTIVE ──(T ≥ 70°C)──> THROTTLING ──(T < 68°C)──> ACTIVE
```

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   ThermalManager                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │   ADC Read   │→ │    Filter    │→ │     FSM      │ │
│  │ (Steinhart-  │  │   (Moving    │  │  (State      │ │
│  │   Hart)      │  │   Average)   │  │  Machine)    │ │
│  └──────────────┘  └──────────────┘  └──────────────┘ │
│                           ↓                             │
│                  ┌──────────────────┐                   │
│                  │   PWM Control    │                   │
│                  │ (Fan Duty Cycle) │                   │
│                  └──────────────────┘                   │
└─────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────┐
│              HardwareInterface (HAL)                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ ADC Peripheral│ │  PWM Timer   │ │  GPIO/IRQ    │ │
│  └──────────────┘  └──────────────┘  └──────────────┘ │
└─────────────────────────────────────────────────────────┘
```

---

## 📐 Technical Implementation Details

### 1. Steinhart-Hart Temperature Conversion

Uses the simplified Beta parameter equation for NTC thermistors:

```
1/T = 1/T₀ + (1/B) × ln(R/R₀)
```

Where:
- `T₀ = 25°C (298.15K)` - Nominal temperature
- `R₀ = 10kΩ` - Nominal resistance
- `B = 3950K` - Beta coefficient

**Optimization Note:** In production systems, this can be replaced with a **Look-Up Table (LUT)** to reduce computational overhead from `log()` operations (critical for microcontrollers without FPU).

### 2. Moving Average Filter

Implements a **circular buffer** for efficient memory usage:

```cpp
float sum = 0.0f;
for (uint8_t i = 0; i < FILTER_SIZE; i++) {
    sum += tempBuffer[i];
}
return sum / FILTER_SIZE;
```

- **Window Size:** 10 samples
- **Effective Bandwidth:** ~1 Hz at 10 Hz sample rate
- **Latency:** 1 second (acceptable for thermal systems with slow dynamics)

### 3. Proportional Fan Control

Linear mapping between temperature and fan duty cycle:

```
Fan Duty = 30% + (T - 40°C) / (70°C - 40°C) × 70%
```

This ensures:
- Minimum audible fan speed at 40°C
- Maximum cooling at thermal throttle point

### 4. Non-Blocking Design

The system uses **time-based polling** instead of `delay()`:

```cpp
bool ThermalManager::update() {
    uint32_t now = hardware->getMillis();
    if ((now - lastSampleTime) < SAMPLE_INTERVAL_MS) {
        return false; // Not time to sample
    }
    // Process thermal control...
}
```

This allows the CPU to handle other tasks (e.g., communications, display updates) between thermal samples.

---

## 🛠️ Hardware Configuration

### Pin Assignment (Arduino)

| Component | Pin | Type | Description |
|-----------|-----|------|-------------|
| NTC Thermistor | A0 | Analog Input | 10kΩ @ 25°C with 10kΩ series resistor |
| Cooling Fan | D9 | PWM Output | 25 kHz PWM frequency |
| Throttle Signal | D8 | Digital Output | CPU frequency control interrupt |

### Schematic (Voltage Divider)

```
    VCC (3.3V)
        │
       [R_series = 10kΩ]
        │
        ├────→ A0 (ADC Input)
        │
       [R_thermistor (NTC)]
        │
       GND
```

**Note:** For STM32 or ESP32, use 12-bit ADC for better resolution.

---

## 💻 Building & Running

### Option 1: Simulation (No Hardware Required)

Compile for PC to test the thermal algorithm:

```bash
# Linux/macOS
g++ -std=c++11 -DSIMULATION \
    -I include \
    src/main.cpp \
    src/ThermalManager.cpp \
    src/HardwareInterface.cpp \
    -o thermal_sim -lm

./thermal_sim
```

```powershell
# Windows (MinGW)
g++ -std=c++11 -DSIMULATION -I include src/main.cpp src/ThermalManager.cpp src/HardwareInterface.cpp -o thermal_sim.exe -lm

.\thermal_sim.exe
```

### Option 2: Arduino

1. Open `src/main.cpp` in Arduino IDE
2. Copy `include/*.hpp` to the sketch folder
3. Copy `src/*.cpp` to the sketch folder
4. Select **Arduino Uno/Nano**
5. Upload

### Option 3: PlatformIO (Recommended for Professional Development)

```bash
pio run --target upload
pio device monitor -b 115200
```

---

## 📊 Telemetry Output

The system outputs JSON telemetry every 100ms via Serial (115200 baud):

```json
{
  "temp_c": 65.24,
  "fan_duty_cycle": 180,
  "fan_percent": 70.6,
  "pwr_state": "NORMAL",
  "thermal_state": "ACTIVE"
}
```

### Integration with Monitoring Tools

- **Python:** Use `pyserial` to log data
- **Node-RED:** Create real-time dashboards
- **Grafana:** Store in InfluxDB/Prometheus for visualization

---

## 🎓 Professional Talking Points (For Interviews)

When presenting this project, highlight these industry-relevant concepts:

### 1. Thermal Design Power (TDP) Management
*"The system implements dynamic thermal throttling similar to Intel's Tjunction monitoring. When the die temperature exceeds 70°C, we reduce CPU frequency to lower power dissipation, preventing thermal runaway."*

### 2. Interrupt-Driven I/O
*"The architecture is non-blocking—we use timer-based sampling instead of `delay()` calls. This allows the CPU to service other critical tasks like communications or display updates without missing thermal events."*

### 3. Computational Optimization
*"In production, I would replace the Steinhart-Hart logarithmic calculation with a pre-computed Look-Up Table (LUT). This reduces execution time from ~200 CPU cycles to ~10, critical for systems without an FPU."*

### 4. Hysteresis for Stability
*"The 2°C hysteresis prevents fan 'hunting'—rapid oscillation between states when temperature hovers near a threshold. This is standard practice in HVAC and thermal control systems."*

### 5. Hardware Abstraction Layer (HAL)
*"The HAL pattern allows the same thermal logic to run on Arduino, STM32, or even PC simulation. This is essential for CI/CD pipelines where you want to unit test thermal algorithms without hardware."*

---

## 📁 Project Structure

```
ADC/
├── include/
│   ├── HardwareInterface.hpp    # HAL abstraction
│   └── ThermalManager.hpp       # Thermal control logic
├── src/
│   ├── HardwareInterface.cpp    # Platform implementations
│   ├── ThermalManager.cpp       # FSM & filtering
│   └── main.cpp                 # Entry point
├── platformio.ini               # PlatformIO config
└── README.md                    # This file
```

---

## 🚀 Future Enhancements

- [ ] **PID Controller:** Replace linear fan control with PID for faster settling time
- [ ] **Kalman Filter:** Better noise rejection than moving average
- [ ] **Multi-Zone Thermal:** Monitor multiple sensors (CPU, GPU, VRM)
- [ ] **EEPROM Calibration:** Store thermistor calibration coefficients
- [ ] **Over-Temperature Alerts:** Send notifications via MQTT/LoRa
- [ ] **Power Estimation:** Estimate CPU power draw from temperature rise rate

---


## 👤 Author
**Jose Herrera**  
---

## 📚 References

1. **Steinhart-Hart Equation:** [Wikipedia](https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation)
2. **Intel Thermal Management:** [Intel® 64 and IA-32 Architectures Software Developer's Manual](https://www.intel.com/content/www/us/en/architecture-and-technology/64-ia-32-architectures-software-developer-vol-3a-part-1-manual.html)
3. **Moving Average Filters:** Smith, S.W. (1997). "The Scientist and Engineer's Guide to Digital Signal Processing"
4. **Finite State Machines in Embedded Systems:** Samek, M. (2008). "Practical UML Statecharts in C/C++"

---

**⭐ If you found this project useful, please star it on GitHub!**

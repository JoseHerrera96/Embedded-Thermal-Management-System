# Thermal Management System - Quick Start Guide

## 🚀 Getting Started in 3 Minutes

### Option 1: Run Simulation (Easiest - No Hardware)

**Windows:**
```powershell
# Compile
g++ -std=c++11 -DSIMULATION -I include src/main.cpp src/ThermalManager.cpp src/HardwareInterface.cpp -o thermal_sim.exe -lm

# Run
.\thermal_sim.exe
```

**Linux/macOS:**
```bash
# Compile
g++ -std=c++11 -DSIMULATION -I include src/main.cpp src/ThermalManager.cpp src/HardwareInterface.cpp -o thermal_sim -lm

# Run
./thermal_sim
```

**Expected Output:**
```
========================================
 Thermal Management System - Simulation
========================================
Thermal Thresholds:
  IDLE:       < 40°C
  ACTIVE:     40-70°C
  THROTTLING: 70-90°C
  CRITICAL:   > 90°C
========================================

[SCENARIO] Setting temperature to 35°C (IDLE)
[TELEMETRY] {"temp_c":35.12,"fan_duty_cycle":0,"fan_percent":0.0,"pwr_state":"NORMAL","thermal_state":"IDLE"}
...
```

---

### Option 2: PlatformIO (Professional Setup)

```bash
# Install PlatformIO
pip install platformio

# Build for simulation
pio run -e simulation

# Or build for Arduino Uno
pio run -e uno --target upload
pio device monitor -b 115200
```

---

### Option 3: Arduino IDE

1. Copy all files from `include/` and `src/` to a new sketch folder
2. Rename `main.cpp` to `ThermalSystem.ino`
3. Select **Tools → Board → Arduino Uno**
4. Upload and open Serial Monitor (115200 baud)

---

## 🔌 Hardware Setup (If using real Arduino)

### Components Needed
- Arduino Uno/Nano
- 10kΩ NTC Thermistor (Beta 3950)
- 10kΩ Resistor
- 5V PC Fan (or LED for testing)
- Breadboard + Jumper wires

### Wiring Diagram
```
       VCC (5V)
          │
         [10kΩ]
          │
          ├──→ A0 (Temperature)
          │
       [NTC 10kΩ]
          │
         GND

       D9 (PWM) ──→ Fan (+) or LED Anode
       GND ──────→ Fan (-) or LED Cathode
```

---

## 📊 Monitor Telemetry

### Python Script (Recommended)
```bash
pip install pyserial

# Monitor serial data
python tools/monitor_telemetry.py COM3 115200
# Or on Linux/Mac:
python tools/monitor_telemetry.py /dev/ttyUSB0 115200
```

### Manual Monitoring
- **Arduino IDE:** Tools → Serial Monitor (115200 baud)
- **PlatformIO:** `pio device monitor -b 115200`

---

## 🧪 Test Scenarios

Once running, you can manually test thermal states:

1. **IDLE State:** Keep thermistor at room temperature (~25°C)
2. **ACTIVE State:** Warm the thermistor with your fingers (40-70°C)
3. **THROTTLING:** Use a heat gun or hair dryer (70-90°C)
4. **CRITICAL:** Briefly touch with soldering iron tip (>90°C) ⚠️

---

## 🐛 Troubleshooting

### Compilation Errors

**Error:** `logf not defined`
**Solution:** Add `-lm` flag for math library

**Error:** `Serial not found`
**Solution:** Make sure `ARDUINO` is defined (not `SIMULATION`)

### No Serial Output

1. Check baud rate is **115200**
2. Verify correct COM port selected
3. On Arduino, add delay in setup: `delay(2000);`

### Temperature Always 0°C

1. Verify ADC wiring (A0 pin)
2. Check thermistor polarity (should be non-polarized)
3. Measure voltage at A0 with multimeter (should be ~1.65V at 25°C)

---

## 📖 Next Steps

1. ✅ Review the [README.md](README.md) for technical details
2. ✅ Check thermal thresholds in [ThermalManager.hpp](include/ThermalManager.hpp)
3. ✅ Try modifying FSM states for custom behavior
4. ✅ Integrate with IoT platform (MQTT, Blynk, etc.)

---

## 💡 For Your Interview

Key points to mention:
- **Non-blocking design** - uses timer-based sampling
- **Hysteresis** - prevents fan oscillation
- **HAL abstraction** - portable across platforms
- **Steinhart-Hart** - industry-standard thermistor equation
- **FSM control** - similar to real CPU thermal management

Good luck! 🚀

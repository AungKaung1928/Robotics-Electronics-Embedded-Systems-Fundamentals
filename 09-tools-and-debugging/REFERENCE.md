# Tools and Debugging

Hardware tools, debugging techniques, and troubleshooting methods for embedded systems and robotics.

---

## 🔧 Essential Hardware Tools

### 1. Multimeter
**The most important tool** for electronics debugging.

#### Basic Functions:
- **Voltage (V)**: Measure voltage across components
- **Current (A)**: Measure current through circuit (break circuit, insert meter in series)
- **Resistance (Ω)**: Measure resistance, check continuity
- **Continuity Test**: Beep when connection exists (0Ω)
- **Diode Test**: Check diode forward voltage drop (~0.3-0.7V)

#### Common Measurements:

**Check Power Supply**:
```
1. Set to DC voltage mode (V⎓)
2. Black probe to GND
3. Red probe to 5V rail
4. Should read ~5.0V
```

**Check Continuity**:
```
1. Set to continuity mode (♪)
2. Touch probes to both ends of wire
3. Should beep if connected
```

**Measure Current Draw**:
```
1. Set to current mode (A)
2. BREAK circuit
3. Insert meter in series (current flows through meter)
4. Read current (e.g., motor draws 500mA)
```

**Test Resistor**:
```
1. Set to resistance mode (Ω)
2. Touch probes to resistor leads
3. Should read nominal value ±tolerance (e.g., 220Ω ±5%)
```

#### Safety:
- **Never measure resistance with power on** (will damage meter)
- **Don't measure current in voltage mode** (meter acts as short circuit → fuse blows)
- **Check probe connections** before measuring high voltage
- **Understand AC vs DC modes**

---

### 2. Oscilloscope (Optional but Valuable)

**What it does**: Displays voltage over time (waveform)

#### Use Cases:
- **PWM signals**: Verify duty cycle, frequency
- **Sensor signals**: See noise, spikes, waveform shape
- **Communication protocols**: Decode I2C, SPI, UART timing
- **Debugging intermittent issues**: Capture glitches
- **Motor driver output**: Verify H-bridge switching

#### Basic Measurements:
- **Voltage amplitude**: Peak-to-peak, RMS
- **Frequency**: Hz, period
- **Duty cycle**: % HIGH time
- **Rise/fall time**: Signal edge speed

**Budget Option**: USB oscilloscope (~$50) or logic analyzer with analog input

---

### 3. Logic Analyzer

**What it does**: Captures and decodes digital signals (I2C, SPI, UART, etc.)

#### Why Use It?
- **See protocol communication**: Visualize I2C address, data bytes
- **Timing issues**: Check if clock/data lines have correct timing
- **Multiple signals simultaneously**: Monitor 8+ channels at once
- **Decoding**: Automatically decode I2C, SPI, UART into human-readable format

#### Example Use:
```
Problem: I2C sensor not responding
Logic Analyzer:
1. Connect to SDA, SCL, GND
2. Capture I2C transaction
3. See: Master sends 0x68, but no ACK from slave
4. Conclusion: Wrong I2C address or sensor not powered
```

**Budget Option**: Saleae clones (~$10-20 on Aliexpress), PulseView software (free)

---

### 4. Breadboard and Jumper Wires

**Breadboard**: Solderless prototyping

**Structure**:
- **Power rails**: Vertical columns (+ and -) connected top-to-bottom
- **Signal rows**: Horizontal rows (a-e, f-j) connected left-right
- **Center gap**: Separates IC legs

**Best Practices**:
- Use **solid-core wire** (22-24 AWG) for breadboard connections
- Keep wires **short and organized**
- Use **color coding**: Red = power, Black = GND, other colors = signals
- Add **decoupling capacitors** near ICs (0.1μF between V_CC and GND)
- Avoid **loose connections** (main source of intermittent bugs)

**Breadboard Limitations**:
- Not suitable for high-frequency signals (>1MHz)
- Not suitable for high currents (>1A per rail)
- Connections can become unreliable over time

---

### 5. Soldering Iron

**For permanent connections, PCB assembly**

#### Soldering Basics:
1. **Heat the joint** (wire + pad), not the solder
2. **Apply solder** to joint (not iron tip)
3. **Remove solder**, then remove iron
4. **Shiny joint** = good, **dull/grainy** = cold joint (bad)

#### Temperature:
- **Electronics soldering**: 320-360°C (600-680°F)
- **Lead-free solder**: Slightly higher temperature needed

#### Tips:
- Use **flux** for better wetting, easier soldering
- Clean tip frequently with wet sponge or brass wool
- Don't keep iron on pad too long (>3 seconds → damage PCB)
- Use proper size tip (small for 0805 SMD, larger for through-hole)

**Safety**:
- Soldering fumes are toxic → ventilation required
- Hot iron can cause serious burns
- Lead-based solder → wash hands after

---

### 6. Basic Hand Tools

- **Wire strippers**: Strip insulation from wires (22-18 AWG typical)
- **Diagonal cutters**: Cut wires, component leads
- **Needle-nose pliers**: Bend wires, hold small components
- **Screwdrivers**: Phillips and flathead (for terminal blocks)
- **Tweezers**: Place/hold small components (SMD resistors, ICs)
- **Helping hands / PCB holder**: Hold boards during soldering

---

## 🐛 Debugging Techniques

### 1. Serial.print() Debugging

**The universal embedded debugging method**

```cpp
void loop() {
  int sensorValue = analogRead(A0);
  
  Serial.print("Sensor: ");
  Serial.print(sensorValue);
  Serial.print(" | Voltage: ");
  Serial.println(sensorValue * 5.0 / 1023.0);
  
  delay(100);
}
```

**Advanced: Structured Debug Output**:
```cpp
#define DEBUG 1  // Set to 0 to disable debug prints

#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

void loop() {
  DEBUG_PRINT("Sensor value: ");
  DEBUG_PRINTLN(analogRead(A0));
}
```

**Serial Plotter** (Arduino IDE):
```cpp
void loop() {
  int sensor1 = analogRead(A0);
  int sensor2 = analogRead(A1);
  
  // Serial Plotter format: comma-separated values
  Serial.print(sensor1);
  Serial.print(",");
  Serial.println(sensor2);
  
  delay(50);
}
```

**Limitations**:
- Serial.print() is **slow** (~1ms per line at 9600 baud)
- Can disrupt timing-sensitive code
- Doesn't work inside ISRs (interrupt service routines)

---

### 2. LED Debugging

**When Serial doesn't work or isn't available**

```cpp
void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  // Blink pattern indicates state
  if (sensorValue > threshold) {
    // Fast blink = error
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
  } else {
    // Slow blink = OK
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
  }
}
```

**Morse Code for Debugging**:
```cpp
void blinkCode(int code) {
  // Blink LED 'code' times to indicate state/error
  for (int i = 0; i < code; i++) {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
  delay(1000);
}
```

---

### 3. Binary Search Debugging

**For complex programs: Isolate the problem**

**Strategy**:
1. Comment out half of the code
2. Does problem still occur?
   - Yes → Problem is in remaining half
   - No → Problem is in commented half
3. Repeat until problem found

**Example**:
```cpp
void loop() {
  readSensors();   // ← Problem somewhere here?
  // calculatePID();
  // updateMotors();
  // sendTelemetry();
}
```

If problem disappears, issue is in commented-out functions.

---

### 4. Watchdog Timer

**Detect when program hangs**

```cpp
#include <avr/wdt.h>

void setup() {
  Serial.begin(9600);
  wdt_enable(WDTO_2S);  // Reset if no wdt_reset() for 2 seconds
  Serial.println("Watchdog enabled");
}

void loop() {
  // Normal operation
  doWork();
  
  wdt_reset();  // Feed the watchdog
}

void doWork() {
  // If this function hangs, watchdog will reset MCU after 2 seconds
  while(1);  // Simulated hang
}
```

**After reset, check why**:
```cpp
void setup() {
  if (MCUSR & (1 << WDRF)) {
    Serial.println("Watchdog reset occurred!");
    MCUSR = 0;  // Clear reset flags
  }
}
```

---

### 5. Assertions

**Catch logic errors early**

```cpp
#define ASSERT(condition) \
  if (!(condition)) { \
    Serial.print("ASSERT FAILED: "); \
    Serial.print(__FILE__); \
    Serial.print(":"); \
    Serial.println(__LINE__); \
    while(1);  /* Halt program */ \
  }

void loop() {
  int encoderCount = readEncoder();
  
  ASSERT(encoderCount >= 0);  // Encoder should never be negative
  
  // Continue execution
}
```

---

## 🔍 Common Issues and Solutions

### Issue: "Code uploads but doesn't run"

**Possible Causes**:
1. **Serial Monitor open during upload** → Close Serial Monitor
2. **Wrong board selected** → Check Tools > Board
3. **Bootloader corrupted** → Re-burn bootloader (Tools > Burn Bootloader)
4. **Pins 0/1 connected** → Disconnect TX/RX during upload

---

### Issue: "I2C device not found"

**Debug Steps**:
1. **Run I2C scanner code** (see Communication Protocols section)
2. **Check pull-up resistors** (4.7kΩ on SDA and SCL)
3. **Check voltage** (sensor 3.3V vs Arduino 5V?)
4. **Check wiring** (SDA ↔ SDA, SCL ↔ SCL, not swapped)
5. **Check address** (some sensors have configurable address)
6. **Check device power** (use multimeter to verify V_CC)

---

### Issue: "Encoder missing counts"

**Solution**:
1. **Use interrupts**, not polling
2. **Use quadrature decoding** (4x resolution)
3. **Check encoder power/GND**
4. **Add pull-up resistors** if needed (typically 10kΩ)
5. **Check wire length** (long wires → add shielding)

---

### Issue: "Motor runs erratically"

**Debug Steps**:
1. **Check power supply** (voltage sag under load?)
2. **Add decoupling capacitors** (100μF-1000μF on motor driver input)
3. **Add flyback diode** across motor terminals
4. **Separate motor and logic grounds** (connect at one point only)
5. **Check PWM frequency** (too low → stuttering)
6. **Check duty cycle limits** (0-255 on Arduino)

---

### Issue: "Random crashes/resets"

**Possible Causes**:
1. **Power supply insufficient** (brownout reset)
2. **Stack overflow** (too many nested function calls)
3. **SRAM overflow** (too many/large variables)
4. **Watchdog timeout** (operation takes too long)
5. **EMI from motors** (add capacitors across motor terminals)

**Debug**:
```cpp
// Check for brownout reset
if (MCUSR & (1 << BORF)) {
  Serial.println("Brownout reset!");
}

// Check free RAM
Serial.println(freeRam());  // Should be >100 bytes
```

---

### Issue: "Serial output is garbled"

**Solutions**:
1. **Check baud rate** (must match on both ends)
2. **Check voltage levels** (3.3V ↔ 5V?)
3. **Check GND connection** (TX/RX without common GND won't work)
4. **Check TX/RX not swapped**
5. **Avoid Serial.print() inside ISR**

---

## 🛡️ Best Practices

### 1. Test Hardware First
Before blaming software, verify hardware with multimeter:
- Power voltages correct?
- Continuity between pins?
- Sensor outputs changing when expected?

### 2. Incremental Development
**Don't write 500 lines and upload once.** Instead:
1. Write 10-20 lines
2. Upload and test
3. Verify it works
4. Add next feature
5. Repeat

### 3. Use Version Control (Git)
```bash
git init
git add *.ino
git commit -m "Working encoder reading"
```

When something breaks, you can revert to working version.

### 4. Add Test Points
When designing PCBs or breadboards:
- Expose critical signals for scope/multimeter probing
- Add test pads for V_CC, GND
- Label pins clearly

### 5. Document Known Issues
```cpp
// BUG: IMU drifts after 1 hour of operation
// TODO: Implement temperature compensation
// FIXME: Encoder occasionally misses counts at high speeds
```

---

## 📚 Resources

- "Debugging Embedded Systems" by Jon Titus
- "Practical Electronics for Inventors" (Chapter on Test Equipment)
- EEVblog (YouTube) - Electronics debugging tutorials
- SparkFun "How to Use a Multimeter"
- Adafruit "Collin's Lab: Soldering"

---

**Next**: [ROS 2 Hardware Integration](../10-ros2-hardware-integration/) - Interfacing microcontrollers with ROS 2

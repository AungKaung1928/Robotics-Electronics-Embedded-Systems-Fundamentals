# Electronic Components

Understanding passive and active components commonly used in robotics circuits.

---

## 🔷 Resistors

### Function:
- Limit current flow
- Divide voltage
- Pull-up/pull-down for digital signals
- Current sensing

### Key Parameters:
- **Resistance value** (Ω, kΩ, MΩ)
- **Power rating** (1/4W, 1/2W, 1W typical)
- **Tolerance** (±1%, ±5%, ±10%)

### Color Code:
- Brown-Black-Red-Gold = 1kΩ ±5%
- Red-Red-Brown-Gold = 220Ω ±5%

### Robotics Applications:
- **LED current limiting**: 220Ω - 1kΩ resistor in series with LED
- **I2C pull-ups**: 4.7kΩ for most applications
- **Voltage dividers**: 10kΩ + 10kΩ for 50% division
- **Current sensing**: Low-value shunt resistor (0.1Ω) for motor current measurement

---

## 🔶 Capacitors

### Function:
- Store electrical energy
- Filter noise and smooth voltage
- Couple AC signals, block DC
- Timing circuits

### Types:

#### Ceramic Capacitors
- **Values**: pF to μF range
- **Application**: Decoupling, high-frequency filtering
- **Non-polarized** (can be installed either direction)

#### Electrolytic Capacitors
- **Values**: μF to mF range
- **Application**: Power supply filtering, bulk energy storage
- **Polarized** (+ and - must be correct or it explodes)

### Key Parameters:
- **Capacitance** (pF, nF, μF, mF)
- **Voltage rating** (must exceed circuit voltage)
- **ESR** (Equivalent Series Resistance) - lower is better

### Robotics Applications:
- **Decoupling capacitors**: 0.1μF ceramic near every IC power pin
- **Power supply filtering**: 1000μF electrolytic on motor driver power input
- **Motor noise suppression**: 0.1μF across motor terminals
- **Smoothing PWM**: Low-pass filter for analog-like voltage from PWM

---

## 🔸 Diodes

### Function:
- Allow current flow in one direction only
- Protect circuits from reverse voltage
- Voltage regulation

### Types:

#### Standard Diode (1N4001, 1N4007)
- Rectification, reverse polarity protection
- Forward voltage drop: ~0.7V

#### Schottky Diode
- Lower forward voltage drop (~0.3V)
- Faster switching
- Used in high-frequency circuits, battery protection

#### Zener Diode
- Voltage regulation at specific voltage
- Overvoltage protection

#### LED (Light Emitting Diode)
- Emits light when current flows
- Forward voltage: Red ~1.8V, Blue/White ~3.0V
- Requires current-limiting resistor

### Key Parameters:
- **Forward voltage drop** (V_f)
- **Maximum forward current** (I_f)
- **Reverse voltage rating** (PIV)

### Robotics Applications:
- **Flyback diode**: Across motor/solenoid coil to protect driver from voltage spikes
- **Reverse polarity protection**: Series diode on battery input
- **Voltage clamping**: Zener diode to protect 3.3V input from 5V signals
- **Status indicators**: LEDs for power, error, communication status

---

## 🔺 Transistors (MOSFETs)

### MOSFET (Metal-Oxide-Semiconductor Field-Effect Transistor)

**Why MOSFETs over BJTs for robotics?**
- Voltage-controlled (not current-controlled)
- Lower power dissipation
- Faster switching
- Higher current handling

### Types:

#### N-Channel MOSFET
- **Common**: IRLZ44N, 2N7000, IRF540
- **Use**: Low-side switching (switch ground connection)
- Turns ON when gate voltage > source voltage

#### P-Channel MOSFET
- **Common**: IRF9540
- **Use**: High-side switching (switch power connection)
- Turns ON when gate voltage < source voltage

### Three Pins:
1. **Gate (G)**: Control input (connect to microcontroller PWM)
2. **Drain (D)**: Load connection
3. **Source (S)**: Ground (N-channel) or power (P-channel)

### Key Parameters:
- **V_DS**: Drain-source voltage rating (e.g., 60V)
- **I_D**: Continuous drain current (e.g., 30A)
- **R_DS(on)**: On-resistance (lower is better, e.g., 0.03Ω)
- **V_GS(th)**: Gate threshold voltage (e.g., 2V - 4V)

### Robotics Applications:
- **Motor control**: PWM switching for DC motor speed control
- **LED dimming**: PWM control of high-power LED strips
- **Solenoid/relay driver**: Switch high-current loads from 5V logic
- **Power management**: Switch power to sensors/modules on/off

**Circuit Example**: N-channel MOSFET as low-side switch
```
Arduino Pin → 220Ω → MOSFET Gate
                     MOSFET Drain → Motor (-) 
                     MOSFET Source → GND
Motor (+) → Battery (+)
Flyback diode across motor terminals
```

---

## 💡 LEDs and Current Limiting

### LED Basics:
- Forward voltage (V_f): Red ~1.8V, Green ~2.0V, Blue/White ~3.0V
- Typical current: 10mA - 20mA
- **MUST have current-limiting resistor** or LED will burn out

### Current-Limiting Resistor Calculation:

**Formula**: `R = (V_supply - V_f) / I_LED`

**Example**: Red LED on 5V supply
- V_supply = 5V
- V_f (red LED) = 1.8V
- I_LED = 20mA = 0.02A
- R = (5 - 1.8) / 0.02 = 160Ω → Use 220Ω (standard value)

### Robotics Applications:
- Status indicators (power, communication, error states)
- Debugging: Visual feedback for code execution
- User interface indicators
- Optical sensors (IR LED + photodiode)

---

## 🔘 Switches and Buttons (Debouncing)

### Mechanical Switch Problem: Bouncing
When pressed, mechanical contacts "bounce" multiple times (1-10ms), creating multiple pulses instead of one clean signal.

### Hardware Debouncing:
**RC Filter**: Resistor + Capacitor smooth the signal
- R = 10kΩ, C = 0.1μF → ~1ms debounce time

### Software Debouncing:
```cpp
// Simple debounce in code
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // 50ms

void loop() {
  int reading = digitalRead(buttonPin);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      // Button state has changed, do something
    }
  }
  
  lastButtonState = reading;
}
```

### Types:
- **Momentary**: Spring-loaded, returns to default state (pushbuttons)
- **Toggle**: Stays in position when switched (rocker switches)
- **Limit switches**: Mechanical sensor for robot position detection
- **Reed switches**: Magnetic proximity sensor

### Robotics Applications:
- Emergency stop button
- User input (mode selection, start/stop)
- Limit switches for robot arm homing
- Magnetic door/cover sensors

---

## ⚙️ Voltage Regulators

### Function:
Convert variable/higher voltage to stable lower voltage

### Types:

#### Linear Regulator (LM7805, AMS1117)
- **Pros**: Simple, low noise, no switching noise
- **Cons**: Inefficient (wastes energy as heat), limited current
- **Example**: LM7805 converts 7V-35V → 5V at up to 1A
- **Application**: Power Arduino from 12V battery

#### Switching Regulator (Buck/Boost)
- **Buck converter**: Steps down voltage (e.g., 12V → 5V)
- **Boost converter**: Steps up voltage (e.g., 3.7V LiPo → 5V)
- **Buck-boost**: Can step up or down
- **Pros**: High efficiency (85-95%), handles higher currents
- **Cons**: Switching noise, more complex

### Key Parameters:
- **Input voltage range**
- **Output voltage** (fixed or adjustable)
- **Maximum output current**
- **Dropout voltage** (minimum V_in - V_out difference)
- **Efficiency**

### Robotics Applications:
- Convert battery voltage to logic levels (12V battery → 5V/3.3V)
- Power distribution: One battery, multiple voltage rails
- USB power (5V) → 3.3V for ESP32
- Efficiency matters for battery-powered robots

**Example**: LM2596 buck converter
- Input: 4.5V - 40V
- Output: 1.25V - 37V adjustable
- Current: 3A max
- Efficiency: ~92%

---

## 🔀 Optocouplers (Electrical Isolation)

### Function:
Transfer electrical signal between circuits while maintaining electrical isolation using light (LED + photodetector)

### Why Isolation?
- Protect low-voltage circuits (3.3V/5V) from high-voltage circuits (24V/48V)
- Eliminate ground loops
- Safety: Separate user interface from motor power circuits

### Common Optocouplers:
- **4N35**: Basic optocoupler (5kV isolation)
- **PC817**: Common, inexpensive
- **TLP281**: High-speed optocoupler

### Key Parameters:
- **Isolation voltage** (e.g., 5000V)
- **Current transfer ratio (CTR)**: Output current / input current (typically 50% - 200%)
- **Response time** (for high-speed signals)

### Robotics Applications:
- Isolate microcontroller from high-voltage motor driver circuits
- Encoder signal isolation from noisy motor environment
- Industrial robot communication (isolate CAN bus, RS-485)
- Safety interlocks: Separate control logic from power circuits

**Circuit Example**: Isolated signal from Arduino to 24V system
```
Arduino GPIO → 220Ω → Optocoupler LED (pins 1,2)
Optocoupler Photodiode (pins 3,4) → Pull-up to 24V → 24V Input
```

---

## 🔍 Component Selection Guidelines

### For Resistors:
- Choose standard E12/E24 values (easier to source)
- Power rating: At least 2x expected power dissipation
- Use 1% tolerance for precision circuits (voltage dividers)

### For Capacitors:
- Voltage rating: At least 2x operating voltage for safety margin
- Decoupling: 0.1μF ceramic for every IC
- Bulk filtering: 100μF - 1000μF electrolytic for motor drivers

### For MOSFETs:
- V_DS rating > maximum voltage in circuit
- I_D rating > maximum current (add safety margin)
- Logic-level gate (V_GS(th) < 2.5V) for 3.3V/5V microcontrollers
- Low R_DS(on) for high-current applications (reduces heat)

### For Diodes:
- Flyback diodes: Current rating > motor current
- Schottky for low voltage drop (battery circuits)
- Fast-recovery for high-frequency switching

---

## 🛠️ Practical Tips

### Common Mistakes:
1. **Forgetting flyback diodes on inductive loads** (motors, solenoids, relays)
2. **No decoupling capacitors near IC power pins** (causes erratic behavior)
3. **Wrong MOSFET type** (P-channel when N-channel needed)
4. **LED without current-limiting resistor** (instant death)
5. **Electrolytic capacitor reverse polarity** (explosion risk)

### Testing Components:
- **Resistor**: Multimeter resistance mode
- **Capacitor**: Capacitance meter, or check for short/open
- **Diode**: Multimeter diode mode (0.3V - 0.7V forward, open reverse)
- **LED**: 3V coin cell + 100Ω resistor
- **MOSFET**: Check continuity drain-source when gate pulled high/low

---

## 📚 Resources

- SparkFun Component Tutorials
- Adafruit Learn System
- Electronics Tutorials (electronics-tutorials.ws)
- MOSFET datasheets (Infineon, International Rectifier)
- "Make: Electronics" by Charles Platt

---

**Next**: [Sensors](../03-sensors/) - Interfacing sensors with microcontrollers and robots

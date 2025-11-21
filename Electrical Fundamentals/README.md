# 01-electrical-fundamentals

Basic electrical concepts essential for understanding robot circuits, power systems, and signal flow.

---

## 📘 Ohm's Law

**Formula**: `V = I × R`

- **V** (Voltage): Electrical potential difference [Volts]
- **I** (Current): Flow of electrical charge [Amperes]
- **R** (Resistance): Opposition to current flow [Ohms]

### Robotics Applications:
- Calculate current draw for motors and sensors
- Select appropriate resistors for LED circuits
- Size power supplies for robot systems
- Determine wire gauge for current capacity

---

## 🔋 Voltage, Current, Resistance, Power

### Voltage (V)
- Electrical "pressure" that pushes current through a circuit
- Common robotics voltages: 3.3V, 5V, 12V, 24V, 48V
- Example: Battery voltage determines motor speed capability

### Current (I)
- Rate of electron flow through a conductor
- Measured in Amperes (A) or milliamperes (mA)
- Example: Servo motor draws 500mA at stall current

### Resistance (R)
- Opposition to current flow
- Measured in Ohms (Ω), kilohms (kΩ), megohms (MΩ)
- Example: Pull-up resistor limits current to microcontroller pin

### Power (P)
- **Formula**: `P = V × I` or `P = I² × R` or `P = V² / R`
- Measured in Watts (W)
- Example: 12V motor drawing 2A consumes 24W

---

## 🔌 Series and Parallel Circuits

### Series Circuit
- Components connected end-to-end
- **Same current** flows through all components
- **Voltage divides** across components
- Total resistance: `R_total = R1 + R2 + R3 + ...`

**Robotics Example**: Voltage divider for sensor level shifting

### Parallel Circuit
- Components connected across same voltage points
- **Same voltage** across all components
- **Current divides** between branches
- Total resistance: `1/R_total = 1/R1 + 1/R2 + 1/R3 + ...`

**Robotics Example**: Multiple sensors powered from same 5V rail

---

## 📊 Digital vs Analog Signals

### Digital Signals
- Two discrete states: HIGH (1) or LOW (0)
- Example: 5V = HIGH, 0V = LOW
- Used for: GPIO control, communication protocols, encoders

### Analog Signals
- Continuous range of values
- Example: 0V to 3.3V representing sensor output
- Used for: Sensor readings, PWM motor control, audio signals

**Robotics Application**: Ultrasonic sensor outputs analog voltage, microcontroller ADC converts to digital value

---

## 🔧 Pull-up and Pull-down Resistors

### Pull-up Resistor
- Connects signal line to HIGH voltage (e.g., 5V)
- Keeps input HIGH when switch/button is open
- Typical values: 1kΩ to 10kΩ

### Pull-down Resistor
- Connects signal line to ground (0V)
- Keeps input LOW when switch/button is open

**Robotics Example**: 
- I2C communication requires pull-up resistors on SDA and SCL lines
- Encoder channels often use pull-up resistors

---

## ⚡ Voltage Dividers

**Formula**: `V_out = V_in × (R2 / (R1 + R2))`

### Applications:
- Level shifting: Convert 5V sensor output to 3.3V microcontroller input
- Battery voltage monitoring
- Analog sensor interfacing

**Example**: Reading 12V battery voltage with 3.3V ADC
- R1 = 10kΩ, R2 = 3.3kΩ
- 12V input → 2.98V output (safe for ADC)

---

## 📐 Reading Electrical Schematics

### Common Symbols:
- **Resistor**: Zigzag line or rectangle
- **Capacitor**: Two parallel lines
- **Diode**: Triangle with line
- **Ground**: Multiple horizontal lines
- **Power**: Circle with + or voltage label
- **Switch**: Break in line with diagonal
- **Motor**: Circle with M
- **Microcontroller**: Rectangle with pin labels

**Practice**: Read Arduino schematics, motor driver diagrams, sensor interface circuits

---

## 📄 Reading Component Datasheets

### Key Information to Extract:
1. **Operating voltage range** (e.g., 3.0V - 5.5V)
2. **Current consumption** (e.g., 10mA typical, 50mA max)
3. **Pin configuration and pinout diagram**
4. **Communication interface** (I2C, SPI, UART, analog)
5. **Timing requirements** (setup time, hold time, clock speed)
6. **Absolute maximum ratings** (what will destroy the component)
7. **Typical application circuit**

**Example**: Reading IMU sensor datasheet to understand I2C address, voltage requirements, and register mapping

---

## 🔌 Power Supply Basics

### Common Robotics Voltages:

| Voltage | Application |
|---------|-------------|
| 3.3V | Modern microcontrollers, low-power sensors |
| 5V | Arduino Uno, servos, older sensors, USB power |
| 12V | DC motors, larger servos, automotive systems |
| 24V | Industrial robots, stepper motors, higher power actuators |
| 48V | High-power mobile robots, BLDC motors |

### Key Concepts:
- **Logic levels**: 3.3V and 5V systems are NOT directly compatible
- **Current capacity**: Power supply must handle total system current draw
- **Voltage regulation**: Buck/boost converters maintain stable voltage
- **Decoupling capacitors**: Smooth out voltage fluctuations

**Robotics Example**: Mobile robot with 24V battery, 12V for motors, 5V for Arduino, 3.3V for ESP32

---

## 🌍 Grounding Concepts

### Ground (GND)
- Reference point (0V) for all voltages in circuit
- Completes current return path
- **All grounds must be connected** in a system

### Common Grounding Issues:
- **Ground loops**: Multiple ground paths creating noise
- **Floating ground**: Device not connected to system ground
- **Ground bounce**: Rapid current changes causing voltage spikes

**Robotics Application**:
- Connect Arduino GND to motor driver GND to sensor GND
- Use common ground for I2C, SPI, UART communication
- Star grounding for noise-sensitive sensors (IMU, force sensors)

---

## 🔍 Practical Exercises

1. Calculate current draw for a 12V motor with 2Ω resistance
2. Design voltage divider to read 24V battery with 3.3V ADC
3. Select pull-up resistor value for I2C running at 400kHz
4. Calculate power dissipation in motor driver H-bridge
5. Read a servo motor datasheet and identify voltage range, current draw, and control signal type

---

## 📚 Resources

- "Practical Electronics for Inventors" - Paul Scherz
- "The Art of Electronics" - Horowitz & Hill
- SparkFun Electronics Tutorials
- All About Circuits (allaboutcircuits.com)
- EEVblog YouTube channel

---

**Next**: [Electronic Components](../02-electronic-components/) - Understanding resistors, capacitors, diodes, and transistors used in robot circuits

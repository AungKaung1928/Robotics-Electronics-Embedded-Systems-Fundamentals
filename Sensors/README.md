# Sensors

Understanding sensor types, interfacing methods, and integration with robotic systems.

---

## 📡 Digital vs Analog Sensors

### Analog Sensors
- Output continuous voltage/current proportional to measured quantity
- Require ADC (Analog-to-Digital Converter) to read with microcontroller
- Example outputs: 0-5V, 0-3.3V, 4-20mA

**Examples**: Potentiometer, analog temperature sensor (LM35), force-sensitive resistor (FSR)

### Digital Sensors
- Output discrete HIGH/LOW signals or digital data via communication protocol
- Can be read directly by microcontroller GPIO or bus (I2C, SPI, UART)

**Examples**: 
- **Simple digital**: IR obstacle sensor (HIGH/LOW), limit switch
- **Protocol-based**: IMU (I2C), GPS (UART), camera (USB/Ethernet)

### Choosing Analog vs Digital:
- **Analog**: Simple, cheap, fast sampling, but susceptible to noise
- **Digital**: More complex, often more accurate, built-in signal processing, noise-resistant

---

## 🔄 ADC (Analog-to-Digital Conversion)

### What is ADC?
Converts continuous analog voltage into discrete digital value that microcontroller can process.

### Key Parameters:

#### Resolution
Number of discrete values ADC can represent
- **10-bit ADC**: 2^10 = 1024 values (0-1023) — Arduino Uno
- **12-bit ADC**: 2^12 = 4096 values (0-4095) — ESP32, STM32
- **16-bit ADC**: 2^16 = 65536 values — High-precision systems

#### Reference Voltage (V_ref)
Maximum voltage ADC can measure
- Arduino Uno: 5V reference → 10-bit → 5V/1024 = 4.88mV per step
- ESP32: 3.3V reference → 12-bit → 3.3V/4096 = 0.81mV per step

#### Sampling Rate
How many readings per second
- Typical microcontroller: 1 kHz - 1 MHz
- Important for fast-changing signals (encoders, audio, vibration)

### ADC Calculation:

**Voltage to ADC value**:
```
ADC_value = (V_sensor / V_ref) × (2^resolution - 1)
```

**ADC value to Voltage**:
```
V_sensor = (ADC_value / (2^resolution - 1)) × V_ref
```

**Example**: Arduino reads analog sensor
- Sensor outputs 2.5V
- V_ref = 5V, 10-bit ADC
- ADC_value = (2.5 / 5) × 1023 = 511

### Improving ADC Accuracy:
1. **Averaging**: Take multiple readings and average them
2. **External reference**: Use precise voltage reference IC instead of noisy V_ref
3. **Filtering**: Software low-pass filter to remove noise
4. **Proper grounding**: Minimize noise in ADC reference and signal paths

---

## 🤖 Common Sensor Types for Robotics

### 1️⃣ Ultrasonic Sensors (HC-SR04)

**Function**: Measure distance using sound waves

**How it works**:
1. Transmit ultrasonic pulse (40kHz)
2. Wait for echo reflection
3. Measure time-of-flight
4. Distance = (time × speed_of_sound) / 2

**Specifications**:
- Range: 2cm - 400cm
- Accuracy: ±3mm
- Interface: Trigger (digital output), Echo (digital input with pulse width)

**Code Example**:
```cpp
long duration, distance;
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

duration = pulseIn(echoPin, HIGH);
distance = duration * 0.034 / 2;  // cm
```

**Applications**: Obstacle avoidance, distance measurement, object detection

**Limitations**: Soft surfaces absorb sound, narrow beam angle (~15°), affected by temperature

---

### 2️⃣ IMU (Inertial Measurement Unit)

**Components**:
- **Accelerometer**: Measures linear acceleration (m/s²) in X, Y, Z axes
- **Gyroscope**: Measures angular velocity (°/s or rad/s) around X, Y, Z axes
- **Magnetometer** (optional): Measures magnetic field (compass)

**Common IMUs**:
- **MPU6050**: 6-axis (accel + gyro), I2C, ±2g/±250°/s to ±16g/±2000°/s
- **MPU9250**: 9-axis (accel + gyro + mag)
- **BNO055**: 9-axis with built-in sensor fusion processor

**Interface**: I2C (most common) or SPI

**Applications**:
- Robot orientation and tilt detection
- Motion detection and gesture recognition
- Balancing robots (inverted pendulum)
- Drone stabilization
- Dead reckoning for navigation

**Key Concepts**:
- **Sensor fusion**: Combine accel + gyro data to get accurate orientation (Kalman filter, complementary filter)
- **Calibration**: IMUs need calibration to remove bias and drift
- **Coordinate frames**: Body frame vs world frame transformations

**ROS 2 Integration**: Publish `sensor_msgs/Imu` message

---

### 3️⃣ Encoders

**Function**: Measure rotational position or speed

**Types**:

#### Incremental Encoder
- Outputs pulses as shaft rotates
- Two channels (A and B) for direction detection (quadrature encoding)
- Cannot determine absolute position on startup
- Common in motor control

**Example**: 600 PPR (pulses per revolution) encoder
- 1 revolution = 600 pulses
- Quadrature (4x counting) → 2400 counts per revolution

#### Absolute Encoder
- Outputs unique code for each position
- Knows position even after power-off
- More expensive, used in robot joints

**Interface**: 
- Incremental: 2-3 digital pins (A, B, optional Index)
- Absolute: SPI, SSI, or parallel

**Reading Encoders**:
```cpp
// Interrupt-based quadrature encoder
volatile long encoderCount = 0;

void encoderISR_A() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderCount++;  // Forward
  } else {
    encoderCount--;  // Reverse
  }
}

attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR_A, CHANGE);
```

**Applications**:
- Motor speed/position feedback (closed-loop control)
- Wheel odometry for mobile robots
- Robot joint angle measurement

---

### 4️⃣ Temperature Sensors

#### Analog (LM35, TMP36)
- Output voltage proportional to temperature
- LM35: 10mV/°C (0°C = 0V, 100°C = 1V)
- Simple but less accurate

#### Digital (DHT22, DS18B20, BME280)
- Output digital data via 1-Wire or I2C
- Higher accuracy, built-in calibration
- DS18B20: ±0.5°C accuracy, 1-Wire interface

**Applications**:
- Motor temperature monitoring (prevent overheating)
- Environmental sensing
- Battery temperature monitoring (safety)

---

### 5️⃣ Proximity Sensors

#### IR Proximity Sensor (Sharp GP2Y0A21YK)
- Emits infrared light, measures reflection
- Analog output voltage vs distance
- Range: 10cm - 80cm
- Non-linear output (requires calibration curve)

#### Infrared Obstacle Sensor (TCRT5000)
- Digital output: HIGH = no obstacle, LOW = obstacle detected
- Very short range (1-3cm)
- Used for line following robots

**Applications**:
- Collision avoidance
- Line following
- Edge detection
- Object detection

---

### 6️⃣ Force Sensors

#### FSR (Force-Sensitive Resistor)
- Resistance decreases with applied force
- Analog output (used with voltage divider)
- Not very accurate, but good for threshold detection

#### Load Cell (HX711)
- Strain gauge-based, very accurate
- Digital interface (HX711 amplifier module)
- Typical range: 1kg - 200kg

**Applications**:
- Gripper force control
- Weight measurement
- Collision detection
- Touch sensing

---

## 🔌 Sensor Interfacing and Wiring

### Power Supply Considerations:
1. **Voltage compatibility**: 3.3V sensor with 5V Arduino requires level shifter
2. **Current capacity**: Ensure power supply can handle all sensors
3. **Decoupling capacitors**: 0.1μF ceramic near sensor V_CC pin
4. **Separate analog/digital grounds** when possible (reduce noise)

### Signal Integrity:
- **Twisted pair cables** for differential signals (encoder A/B channels)
- **Shielded cables** for analog sensors in noisy environments
- **Pull-up resistors** for I2C (typically 4.7kΩ)
- **Low-pass filters** for noisy analog signals (RC filter)

### Common Wiring:
- **3-wire sensors**: V+, GND, Signal
- **I2C**: V+, GND, SDA, SCL (+ pull-ups)
- **SPI**: V+, GND, MISO, MOSI, SCK, CS
- **Encoders**: V+, GND, Channel A, Channel B, (Index)

---

## 🎯 Sensor Calibration Basics

### Why Calibration?
- Manufacturing variations
- Environmental effects (temperature, humidity)
- Non-linear sensor response
- Zero-offset and scale errors

### Calibration Methods:

#### 1. **Zero-offset Calibration**
Remove constant bias from sensor reading
```cpp
// Take readings at known zero condition
float offset = 0;
for (int i = 0; i < 100; i++) {
  offset += readSensor();
}
offset /= 100;

// Subtract offset from all readings
float calibratedValue = readSensor() - offset;
```

#### 2. **Two-point Calibration**
Linear mapping between sensor output and real-world value

Given two known points:
- (ADC_low, RealValue_low)
- (ADC_high, RealValue_high)

```cpp
float slope = (RealValue_high - RealValue_low) / (ADC_high - ADC_low);
float intercept = RealValue_low - (slope * ADC_low);
float calibratedValue = (slope * ADC_reading) + intercept;
```

**Example**: Calibrate IR distance sensor
- Place object at 10cm → ADC reads 512
- Place object at 80cm → ADC reads 150
- Calculate slope and intercept
- Convert future ADC readings to distance (cm)

#### 3. **Multi-point Calibration (Lookup Table)**
For non-linear sensors

Store calibration data:
```cpp
float calData[5][2] = {
  {100, 10.0},   // ADC value, Real distance (cm)
  {200, 20.0},
  {300, 40.0},
  {400, 60.0},
  {500, 80.0}
};

// Use linear interpolation between points
```

#### 4. **IMU Calibration**
- **Accelerometer**: Place on flat surface, calibrate each axis to ±1g
- **Gyroscope**: Keep stationary, record bias, subtract from readings
- **Magnetometer**: Rotate in figure-8 pattern, fit ellipsoid, correct for hard/soft iron distortion

---

## 🔍 Sensor Fusion

Combine multiple sensors to get better estimate than any single sensor.

**Example**: Orientation estimation (IMU)
- **Accelerometer**: Good for static orientation, but noisy during motion
- **Gyroscope**: Good for dynamic changes, but drifts over time
- **Sensor fusion**: Combine both using complementary filter or Kalman filter

**Complementary Filter** (simple):
```cpp
float angle = 0.98 * (angle + gyro * dt) + 0.02 * accel_angle;
```

**Applications**:
- IMU + GPS for robot localization
- Ultrasonic + IR + camera for obstacle detection
- Force sensor + encoder for compliant control

---

## 🛠️ Practical Tips

### Noise Reduction:
1. **Hardware filtering**: RC low-pass filter on analog signals
2. **Software filtering**: Moving average, median filter, exponential smoothing
3. **Shielding**: Use shielded cables for long sensor wires
4. **Grounding**: Star grounding topology
5. **Decoupling capacitors**: 0.1μF on every sensor

### Debugging Sensors:
1. **Check voltage**: Use multimeter to verify sensor power
2. **Check signal**: Oscilloscope or logic analyzer to see waveform
3. **Test in isolation**: Connect sensor alone before integrating with robot
4. **Serial debug**: Print raw ADC values to Serial Monitor
5. **Datasheet**: Always refer to timing diagrams and specs

### Common Issues:
- **I2C not working**: Check pull-up resistors (4.7kΩ), verify address
- **Encoder missing counts**: Use interrupts, not polling
- **IMU drift**: Implement sensor fusion, recalibrate
- **Ultrasonic timeout**: Increase timeout, check wiring
- **Analog noise**: Add capacitor, use averaging

---

## 📚 Resources

- "Sensors and Actuators in Mechatronics" by Jiri Novak
- SparkFun Sensor Tutorials
- Adafruit Learning System
- ROS 2 Sensor Integration Tutorials
- "Make: Sensors" by Tero Karvinen

---

**Next**: [Actuators and Motors](../04-actuators-and-motors/) - Understanding motors, drivers, and motion control

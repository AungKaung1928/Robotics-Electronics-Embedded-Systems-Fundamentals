# Actuators and Motors

Understanding motor types, motor drivers, encoders, and actuator control for robotic systems.

---

## ⚙️ DC Motors (Brushed)

### How it works:
- Electric current through coils creates magnetic field
- Magnetic field interacts with permanent magnets → rotation
- Brushes and commutator switch current direction

### Characteristics:
- **Simple to control**: Just apply voltage, it spins
- **Speed proportional to voltage**
- **Direction controlled by polarity**
- **No inherent position feedback** (requires encoder)

### Key Specifications:
- **Rated voltage** (e.g., 12V nominal)
- **No-load speed** (RPM at rated voltage, no load)
- **Stall torque** (maximum torque when motor can't rotate)
- **Stall current** (current at maximum torque)
- **No-load current** (current when spinning freely)

### Torque-Speed Curve:
- Maximum speed at no load
- Maximum torque at zero speed (stall)
- Linear relationship between torque and speed

### Applications:
- Mobile robot drive wheels
- Simple actuators
- Low-cost robotics projects
- Continuous rotation applications

### Advantages:
- Cheap and widely available
- Simple control (just voltage)
- Good torque at low speeds

### Disadvantages:
- Brushes wear out over time
- Electrical noise from arcing brushes
- Lower efficiency than brushless motors
- Requires external encoder for position control

---

## 🔄 Servo Motors

### What is a Servo?
A DC motor + gearbox + position sensor + control circuit in one package

### Types:

#### Standard Servo (Positional)
- **Range**: Typically 0° - 180° (some up to 270°)
- **Control**: PWM signal (50Hz, pulse width 1ms - 2ms)
- **Feedback**: Internal potentiometer measures position
- **Use**: Robot arms, camera gimbals, steering

**PWM Control**:
- 1.0ms pulse → 0°
- 1.5ms pulse → 90° (center)
- 2.0ms pulse → 180°

#### Continuous Rotation Servo
- Modified to rotate continuously
- PWM controls speed and direction, not position
- 1.5ms = stop, <1.5ms = reverse, >1.5ms = forward

### Key Specifications:
- **Torque** (kg-cm or oz-in): How much force at given distance from shaft
- **Speed** (sec/60°): How fast it can move (e.g., 0.12 sec/60° means 500ms for 180°)
- **Voltage range** (typically 4.8V - 6V, some 7.4V)
- **Current draw** (idle vs stall)

### Arduino Control:
```cpp
#include <Servo.h>
Servo myservo;

void setup() {
  myservo.attach(9);  // Pin 9
}

void loop() {
  myservo.write(90);  // Move to 90 degrees
  delay(1000);
  myservo.write(0);   // Move to 0 degrees
  delay(1000);
}
```

### Applications:
- Robot arm joints
- Gripper mechanisms
- Camera pan/tilt
- Steering in small robots

### Advantages:
- Built-in position control (no external controller needed)
- Easy to use (just PWM signal)
- Holds position under load
- Accurate positioning

### Disadvantages:
- Limited rotation range (except continuous type)
- Jitter when holding position
- Not suitable for high-speed continuous rotation
- Moderate torque (not as high as stepper/DC geared motors)

---

## 🎯 Stepper Motors

### How it works:
- Electromagnets arranged around rotor
- Energize coils in sequence → rotor moves in discrete steps
- Typical: 200 steps per revolution (1.8° per step)

### Types:

#### Unipolar Stepper
- 5 or 6 wires
- Simpler drive circuit
- Lower torque
- Common: 28BYJ-48 (5V stepper with ULN2003 driver)

#### Bipolar Stepper
- 4 wires
- Requires H-bridge driver (A4988, DRV8825, TMC2208)
- Higher torque than unipolar
- More complex control

### Key Specifications:
- **Steps per revolution** (e.g., 200 steps = 1.8°/step)
- **Holding torque** (torque when energized and stationary)
- **Rated current per coil** (e.g., 1.5A)
- **Voltage rating** (e.g., 12V nominal, but can use higher with current limiting)

### Control Modes:

#### Full Step
- One or two coils energized at a time
- Standard resolution (e.g., 200 steps/rev)

#### Half Step
- Alternates between 1-coil and 2-coil energization
- Double resolution (e.g., 400 steps/rev)

#### Microstepping
- Vary current in coils to create intermediate positions
- 1/4, 1/8, 1/16, 1/32 microstepping common
- Example: 200 steps/rev × 16 = 3200 microsteps/rev
- Smoother motion, quieter, but lower torque at microsteps

### Arduino Control (with A4988 driver):
```cpp
#define stepPin 3
#define dirPin 4

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  digitalWrite(dirPin, HIGH);  // Set direction
  
  for(int i = 0; i < 200; i++) {  // 200 steps = 1 revolution
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}
```

### Applications:
- 3D printers (X, Y, Z axes)
- CNC machines
- Precise positioning systems
- Robot arm joints (where position matters)

### Advantages:
- **Open-loop position control** (no encoder needed)
- Precise, repeatable positioning
- Holds position when energized (high holding torque)
- Can be controlled at very low speeds

### Disadvantages:
- Can lose steps if overloaded (no feedback)
- Resonance at certain speeds (vibration)
- Lower top speed than DC/BLDC motors
- Constant current draw even when stationary
- More complex driver circuitry

---

## 🚀 Brushless DC Motors (BLDC)

### How it works:
- No brushes (electronically commutated)
- Permanent magnet rotor, electromagnet stator
- Electronic Speed Controller (ESC) switches coil current
- Requires Hall sensors or back-EMF sensing for commutation

### Key Specifications:
- **KV rating**: RPM per volt (e.g., 1000KV motor at 12V → 12,000 RPM no-load)
- **Voltage range** (e.g., 2S-4S LiPo = 7.4V - 14.8V)
- **Maximum current** (e.g., 30A)
- **Number of poles** (affects torque and smoothness)

### Control:
Requires ESC (Electronic Speed Controller)
- Converts PWM/PPM signal or digital protocol to 3-phase motor drive
- Protocols: PWM (1-2ms pulses), OneShot, DShot, CAN

### Arduino Control (with standard ESC):
```cpp
#include <Servo.h>  // ESCs often use servo library
Servo esc;

void setup() {
  esc.attach(9);
  esc.write(0);  // Initialize ESC (send minimum throttle)
  delay(2000);   // ESC calibration time
}

void loop() {
  esc.write(50);   // 50% throttle
  delay(2000);
  esc.write(100);  // 100% throttle
  delay(2000);
}
```

### Applications:
- Drones and quadcopters (high power-to-weight ratio)
- High-speed mobile robots
- Electric vehicles
- High-efficiency applications

### Advantages:
- Very high efficiency (>90%)
- High power-to-weight ratio
- Long lifespan (no brush wear)
- High speeds possible
- Less electrical noise

### Disadvantages:
- Requires complex ESC
- More expensive than brushed motors
- Position control requires encoder or resolver
- ESC can be bulky

---

## 🛠️ Motor Drivers and H-bridges

### Why Motor Drivers?
Microcontroller GPIO pins:
- Low current output (20-40mA typical)
- Cannot directly drive motors (motors need amps)
- Cannot reverse polarity for direction control

**Solution**: Motor driver or H-bridge circuit

### H-Bridge Circuit:
4 switches (MOSFETs or transistors) arranged to control motor direction:

```
         +V
          |
     [S1] | [S2]
          |
    ------+------  Motor
          |
     [S3] | [S4]
          |
         GND
```

- **Forward**: S1 + S4 ON, S2 + S3 OFF
- **Reverse**: S2 + S3 ON, S1 + S4 OFF
- **Brake**: S3 + S4 ON (short motor terminals)
- **Coast**: All OFF

### Common Motor Driver ICs:

#### L298N (Dual H-bridge)
- **Current**: 2A per channel (4A peak)
- **Voltage**: 5V - 35V
- **Control**: Direction pins + PWM enable
- **Pros**: Cheap, built-in voltage regulator
- **Cons**: High voltage drop (~2V), inefficient, gets hot

#### L293D (Quad Half-H-bridge)
- **Current**: 600mA per channel (1.2A peak)
- **Voltage**: 4.5V - 36V
- **Use**: Small DC motors, servos
- **Cons**: Low current capacity

#### TB6612FNG
- **Current**: 1.2A per channel (3.2A peak)
- **Voltage**: 4.5V - 13.5V
- **Pros**: Low voltage drop, efficient, dual motor control
- **Better than L298N** for most applications

#### DRV8833 (Dual H-bridge)
- **Current**: 1.5A per channel (2A peak)
- **Voltage**: 2.7V - 10.8V
- **Pros**: Very efficient, low-voltage capable
- **Use**: Battery-powered robots, small motors

#### BTS7960 (High-current)
- **Current**: 43A continuous
- **Voltage**: 5.5V - 27V
- **Pros**: Very high current, good for large robots
- **Use**: Heavy-duty mobile robots, large DC motors

### Control Signals:
- **IN1, IN2**: Direction control (HIGH/LOW combinations)
- **ENA**: PWM for speed control (0-255 → 0-100% duty cycle)

**Example** (L298N):
```cpp
#define IN1 8
#define IN2 9
#define ENA 10

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
}

void loop() {
  // Forward at 75% speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 191);  // 75% of 255
  delay(2000);
  
  // Reverse at 50% speed
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 127);  // 50% of 255
  delay(2000);
  
  // Stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(1000);
}
```

### Heat Management:
- Motor drivers generate heat (especially L298N)
- Add heatsink if continuous high current
- Consider airflow in enclosed robot chassis
- Monitor driver temperature in critical applications

---

## 🔢 Encoders (Detailed)

### Why Encoders?
- Measure motor shaft position
- Calculate motor speed
- Enable closed-loop control (PID)
- Odometry for mobile robots

### Incremental Encoder (Quadrature)

**Two channels (A and B)**, 90° phase shift:
- Channel A pulses as motor rotates
- Channel B leads or lags A depending on direction

**Example**: 600 PPR (Pulses Per Revolution) encoder
- 600 pulses per full rotation on channel A
- Quadrature decoding (rising + falling edges on both channels) → 2400 counts/rev

**Reading Direction**:
```
Forward:  A: ↑ → B already HIGH → forward
Reverse:  A: ↑ → B already LOW  → reverse
```

**Arduino Code** (Interrupt-based):
```cpp
volatile long encoderCount = 0;
int lastEncoded = 0;

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
}

void updateEncoder() {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;
  
  lastEncoded = encoded;
}

void loop() {
  Serial.println(encoderCount);
}
```

### Calculating Speed:
```cpp
long prevCount = 0;
unsigned long prevTime = 0;

void loop() {
  unsigned long currentTime = millis();
  long currentCount = encoderCount;
  
  if (currentTime - prevTime >= 100) {  // Every 100ms
    long deltaCounts = currentCount - prevCount;
    float deltaTime = (currentTime - prevTime) / 1000.0;  // seconds
    
    float RPM = (deltaCounts / 2400.0) * (60.0 / deltaTime);  // 2400 counts/rev
    
    prevCount = currentCount;
    prevTime = currentTime;
  }
}
```

### Absolute Encoder

**Output**: Unique code for each position (Gray code, binary)
- No need to track counts (knows position after power-up)
- More expensive, used in critical applications (robot joints)
- Interface: Parallel, SSI, BiSS, or magnetic (AS5600, AS5048)

---

## 📚 Resources

- "Electric Motors and Drives" by Austin Hughes
- Pololu Motor Selection Guide
- SparkFun Motor Driver Tutorials
- STM32 Motor Control Workbench
- "Practical Electronics for Inventors" - Motor Control Chapter

---

**Next**: [Motor Control](../05-motor-control/) - PWM, PID control, and motion control algorithms

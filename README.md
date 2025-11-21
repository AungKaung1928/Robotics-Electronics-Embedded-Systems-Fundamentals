# Robotics Electronics & Embedded Systems Fundamentals

A comprehensive guide to electrical, electronics, and embedded systems concepts essential for robotics software engineers, focusing on practical knowledge needed for robot control, sensor integration, and hardware interfacing.

> **🎓 New to Arduino?** Start with the [Complete Beginner's Guide](./00-getting-started/) for step-by-step tutorials!

## 📚 Repository Structure

This repository covers fundamental concepts required for robotics engineers working with hardware integration, embedded systems, and control systems.

### 0. [🎓 Getting Started (Beginners)](./00-getting-started/)
Complete beginner's guide with step-by-step Arduino tutorials, breadboard basics, and building your first robot car.

### 1. [Electrical Fundamentals](./01-electrical-fundamentals/)
Basic electrical concepts, circuit analysis, and power systems for robotics applications.

### 2. [Electronic Components](./02-electronic-components/)
Understanding passive and active components used in robotics circuits.

### 3. [Sensors](./03-sensors/)
Sensor types, interfacing, calibration, and integration with robotic systems.

### 4. [Actuators and Motors](./04-actuators-and-motors/)
Motor types, drivers, encoders, and actuator control for robotic systems.

### 5. [Motor Control](./05-motor-control/)
PWM, PID control, and motion control algorithms for robotics.

### 6. [Microcontrollers](./06-microcontrollers/)
Arduino, ESP32, and embedded programming basics for robot control.

### 7. [Communication Protocols](./07-communication-protocols/)
Serial communication, I2C, SPI, CAN bus for robotics applications.

### 8. [Embedded Programming](./08-embedded-programming/)
C/C++ programming for microcontrollers and real-time systems.

### 9. [Tools and Debugging](./09-tools-and-debugging/)
Hardware tools, debugging techniques, and troubleshooting methods.

### 10. [ROS 2 Hardware Integration](./10-ros2-hardware-integration/)
Interfacing microcontrollers with ROS 2, hardware abstraction layers, and sensor publishing.

---
### [📖 Arduino Troubleshooting Guide](./Arduino%20Troubleshooting%20Guide/)
Common Arduino problems and solutions: upload issues, hardware debugging, serial communication errors, and sensor troubleshooting.

### [🚀 Getting Started with Arduino and Robotics](./Getting%20Started%20with%20Arduino%20and%20Robotics/)
Beginner-friendly tutorials with shopping list, Arduino IDE setup, breadboard basics, and 5 complete projects from LED to robot car.

## 🎯 Target Audience

This repository is designed for:
- Mechanical engineers transitioning to robotics software
- Robotics software engineers needing hardware knowledge
- Students learning embedded systems for robotics
- Engineers preparing for robotics job interviews in Japan

## 🔧 Focus Areas

- **Robotics-specific electronics** (not general electrical engineering)
- **Low-voltage circuits** (3.3V, 5V, 12V, 24V systems)
- **Sensor and actuator interfacing**
- **Embedded control systems**
- **ROS 2 hardware integration**

### Arduino & Microcontroller Fundamentals

This repository emphasizes **practical Arduino skills** for robotics:

**Hardware Basics:**
- Arduino Uno, Mega, Nano for robot control
- ESP32 for WiFi/Bluetooth robotics projects
- Breadboard prototyping and circuit building
- Component selection (resistors, LEDs, buttons, sensors)

**Programming Skills:**
- Arduino C/C++ programming fundamentals
- Digital I/O (digitalWrite, digitalRead)
- Analog I/O (analogRead, analogWrite/PWM)
- Serial communication for debugging and ROS 2 integration
- Interrupt-based programming for encoders
- Non-blocking code patterns (millis() instead of delay())

**Sensor Integration:**
- Ultrasonic sensors (HC-SR04) for distance measurement
- IMU sensors (MPU6050) for orientation
- Encoders for motor feedback
- Temperature, pressure, and proximity sensors
- ADC reading and signal processing

**Motor Control:**
- DC motor control with H-bridge drivers (L298N, TB6612)
- Servo motor control (SG90, MG995)
- Stepper motor control (28BYJ-48, NEMA17)
- PWM speed control and direction control
- PID control implementation for precise motion

**Communication:**
- UART/Serial for Arduino ↔ ROS 2 communication
- I2C for multiple sensors (IMU, displays, I/O expanders)
- SPI for high-speed peripherals (SD cards, displays)

**Robot Projects:**
- 2WD/4WD mobile robots with obstacle avoidance
- Line-following robots
- Bluetooth/WiFi controlled robots
- Arduino as hardware abstraction layer for ROS 2

## 📖 How to Use This Repository

Each folder contains a `README.md` with:
1. Fundamental concepts and theory
2. Practical examples for robotics applications
3. Common issues and troubleshooting
4. References and resources for deeper learning

## 🚀 Getting Started

### For Complete Beginners:

**Start here if you're new to Arduino and electronics:**

1. **Basic Arduino Setup** (See examples below)
   - Blink LED
   - Read button input
   - Control LED brightness with potentiometer

2. **Simple Circuits** 
   - Breadboard basics
   - LED + resistor circuits
   - Button with pull-up resistor

3. **First Robot Project**
   - 2WD robot car chassis kit
   - Motor control with L298N
   - Obstacle avoidance with ultrasonic sensor

4. **ROS 2 Integration**
   - Arduino ↔ ROS 2 serial communication
   - Publish sensor data to ROS 2
   - Control motors from ROS 2

### Arduino Learning Path (Zero to Robot):

**Week 1-2: Arduino Basics**
- [ ] Install Arduino IDE and upload first program (Blink)
- [ ] Understand digital I/O (LED on/off, button reading)
- [ ] Learn analog I/O (potentiometer, analogRead/Write)
- [ ] Master Serial communication (Serial.print for debugging)
- [ ] Build breadboard circuits (resistors, LEDs, buttons)

**Week 3-4: Sensors & Actuators**
- [ ] Ultrasonic sensor (HC-SR04) distance measurement
- [ ] Servo motor control (SG90)
- [ ] DC motor control with L298N motor driver
- [ ] Temperature/humidity sensor (DHT22)
- [ ] LCD display for sensor readings

**Week 5-6: Robot Car Project**
- [ ] Assemble 2WD robot chassis with motors
- [ ] Wire L298N motor driver to Arduino
- [ ] Program forward, backward, turn functions
- [ ] Add ultrasonic sensor for obstacle detection
- [ ] Implement obstacle avoidance algorithm

**Week 7-8: Advanced Concepts**
- [ ] IMU sensor (MPU6050) for orientation
- [ ] Rotary encoders for odometry
- [ ] PID control for straight-line driving
- [ ] Bluetooth control (HC-05 module)
- [ ] Non-blocking code patterns

**Week 9-10: ROS 2 Integration**
- [ ] Arduino publishes sensor data via serial
- [ ] ROS 2 Python node reads serial data
- [ ] Publish ultrasonic data to `/scan` topic
- [ ] Subscribe to `/cmd_vel` for motor control
- [ ] Build complete Arduino ↔ ROS 2 robot

### Essential Arduino Components (~$100 Budget):

**Starter Kit (~$50):**
- Arduino Uno R3 + USB cable
- Breadboard + jumper wires
- LED assortment (red, green, blue, yellow)
- Resistor kit (220Ω, 1kΩ, 10kΩ)
- Push buttons, potentiometer
- HC-SR04 ultrasonic sensor

**Robot Kit (~$30):**
- 2WD robot car chassis with motors
- L298N motor driver module
- Battery pack (4× AA or 18650)
- Servo motor (SG90)

**Expansion (~$20):**
- MPU6050 IMU sensor
- ESP32 DevKit (WiFi/Bluetooth)
- Rotary encoders (2×)
- HC-05 Bluetooth module

### Learning Path:

Start with:
1. [Electrical Fundamentals](./01-electrical-fundamentals/) - Learn basic circuit concepts
2. [Electronic Components](./02-electronic-components/) - Understand components in robot circuits
3. [Sensors](./03-sensors/) - Learn sensor types and interfacing
4. [Microcontrollers](./06-microcontrollers/) - Get hands-on with Arduino/ESP32

Then move to:
5. [Motor Control](./05-motor-control/) - Control algorithms
6. [Communication Protocols](./07-communication-protocols/) - Connect devices
7. [ROS 2 Hardware Integration](./10-ros2-hardware-integration/) - Integrate with ROS 2

---

## ⚡ Arduino Quick Reference

### Most Common Arduino Commands:

```cpp
// Digital I/O
pinMode(pin, OUTPUT);        // Set pin as output
pinMode(pin, INPUT);         // Set pin as input
pinMode(pin, INPUT_PULLUP);  // Input with pull-up resistor
digitalWrite(pin, HIGH);     // Set pin HIGH (5V)
digitalWrite(pin, LOW);      // Set pin LOW (0V)
int value = digitalRead(pin);  // Read digital pin (HIGH or LOW)

// Analog I/O
int value = analogRead(A0);   // Read analog pin (0-1023)
analogWrite(pin, 128);        // PWM output (0-255)

// Timing
delay(1000);                  // Wait 1 second (blocking)
unsigned long t = millis();   // Time since power on (ms)
delayMicroseconds(100);       // Wait 100 microseconds

// Serial Communication
Serial.begin(9600);           // Initialize serial at 9600 baud
Serial.println("Hello");      // Print line to Serial Monitor
Serial.print(value);          // Print value without newline
int available = Serial.available();  // Bytes waiting to be read
char c = Serial.read();       // Read one byte
```

### Arduino Pin Types:

**Arduino Uno:**
- **Digital pins**: 0-13 (0, 1 used for Serial)
- **PWM pins**: 3, 5, 6, 9, 10, 11 (marked with ~)
- **Analog inputs**: A0-A5 (can also be used as digital)
- **Power**: 5V, 3.3V, GND

**ESP32:**
- **GPIO pins**: 0-39 (some input-only)
- **ADC pins**: 0, 2, 4, 12-15, 25-27, 32-39 (12-bit)
- **PWM**: Any GPIO (16 channels available)
- **Power**: 3.3V, GND (5V-tolerant inputs)

### Common Circuit Patterns:

**LED Circuit:**
```
Arduino Pin → 220Ω Resistor → LED (+) → LED (-) → GND
```

**Button with Pull-up:**
```
Button: One side → 5V
        Other side → Arduino Pin + 10kΩ Resistor → GND
```

**Button with INPUT_PULLUP:**
```
Button: One side → Arduino Pin
        Other side → GND
```

**Potentiometer:**
```
Left pin   → 5V
Middle pin → Arduino A0
Right pin  → GND
```

**HC-SR04 Ultrasonic:**
```
VCC  → 5V
Trig → Digital Pin
Echo → Digital Pin  
GND  → GND
```

**Servo Motor:**
```
Red (V+)    → 5V (small servo) or external 5-6V
Brown (GND) → GND
Orange (Signal) → PWM Pin
```

**DC Motor with L298N:**
```
L298N IN1, IN2 → Arduino Digital Pins (direction)
L298N ENA      → Arduino PWM Pin (speed)
L298N OUT1, OUT2 → Motor terminals
L298N 12V, GND → Battery
```

---

## 🎓 Beginner-Friendly Examples

### Example 1: Blink LED (Hello World of Arduino)

**What you need:**
- Arduino Uno
- LED (any color)
- 220Ω resistor
- Breadboard
- Jumper wires

**Circuit:**
```
Arduino Pin 13 → 220Ω Resistor → LED (+) → LED (-) → GND
```

**Code:**
```cpp
void setup() {
  pinMode(13, OUTPUT);  // Set pin 13 as output
}

void loop() {
  digitalWrite(13, HIGH);  // Turn LED ON
  delay(1000);             // Wait 1 second
  digitalWrite(13, LOW);   // Turn LED OFF
  delay(1000);             // Wait 1 second
}
```

---

### Example 2: Button Control LED

**What you need:**
- Arduino Uno
- LED + 220Ω resistor
- Push button
- 10kΩ resistor (pull-down)
- Breadboard
- Jumper wires

**Circuit:**
```
Button: One side → 5V
        Other side → Arduino Pin 2 AND 10kΩ resistor → GND
LED: Arduino Pin 13 → 220Ω Resistor → LED (+) → LED (-) → GND
```

**Code:**
```cpp
const int buttonPin = 2;
const int ledPin = 13;

void setup() {
  pinMode(buttonPin, INPUT);   // Button as input
  pinMode(ledPin, OUTPUT);     // LED as output
}

void loop() {
  int buttonState = digitalRead(buttonPin);  // Read button
  
  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);  // Button pressed → LED ON
  } else {
    digitalWrite(ledPin, LOW);   // Button released → LED OFF
  }
}
```

---

### Example 3: Potentiometer Control LED Brightness

**What you need:**
- Arduino Uno
- LED + 220Ω resistor
- Potentiometer (10kΩ)
- Breadboard
- Jumper wires

**Circuit:**
```
Potentiometer: 
  - Left pin → 5V
  - Middle pin → Arduino A0
  - Right pin → GND

LED: Arduino Pin 9 → 220Ω Resistor → LED (+) → LED (-) → GND
```

**Code:**
```cpp
const int potPin = A0;    // Potentiometer on analog pin A0
const int ledPin = 9;     // LED on PWM pin 9

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(potPin);        // Read pot (0-1023)
  int brightness = map(potValue, 0, 1023, 0, 255);  // Convert to PWM (0-255)
  
  analogWrite(ledPin, brightness);  // Set LED brightness
  
  Serial.print("Potentiometer: ");
  Serial.print(potValue);
  Serial.print(" | Brightness: ");
  Serial.println(brightness);
  
  delay(100);
}
```

---

### Example 4: Ultrasonic Sensor Distance Measurement

**What you need:**
- Arduino Uno
- HC-SR04 Ultrasonic sensor
- Breadboard
- Jumper wires

**Circuit:**
```
HC-SR04:
  VCC → Arduino 5V
  Trig → Arduino Pin 9
  Echo → Arduino Pin 10
  GND → Arduino GND
```

**Code:**
```cpp
const int trigPin = 9;
const int echoPin = 10;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Send ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure echo time
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance in cm
  float distance = duration * 0.034 / 2;
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  delay(500);
}
```

---

### Example 5: Simple 2WD Robot Car

**What you need:**
- Arduino Uno
- 2WD robot car chassis kit (motors, wheels, chassis)
- L298N motor driver
- HC-SR04 ultrasonic sensor
- Battery pack (6-12V for motors)
- Jumper wires

**Circuit:**
```
L298N Motor Driver:
  IN1 → Arduino Pin 8
  IN2 → Arduino Pin 9
  IN3 → Arduino Pin 10
  IN4 → Arduino Pin 11
  ENA → Arduino Pin 5 (PWM)
  ENB → Arduino Pin 6 (PWM)
  Motor A → Left motor
  Motor B → Right motor
  12V, GND → Battery pack
  5V → Arduino 5V (power Arduino from L298N)

HC-SR04:
  VCC → Arduino 5V
  Trig → Arduino Pin 3
  Echo → Arduino Pin 4
  GND → Arduino GND
```

**Code:**
```cpp
// Motor pins
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;
const int ENA = 5;
const int ENB = 6;

// Ultrasonic sensor pins
const int trigPin = 3;
const int echoPin = 4;

void setup() {
  // Motor pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  float distance = readUltrasonic();
  
  Serial.print("Distance: ");
  Serial.println(distance);
  
  if (distance > 20) {
    // No obstacle → Move forward
    moveForward(150);  // Speed 0-255
  } else {
    // Obstacle detected → Stop, turn right, continue
    stopMotors();
    delay(500);
    turnRight(150);
    delay(800);
  }
  
  delay(100);
}

void moveForward(int speed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnRight(int speed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  
  return distance;
}
```

---

### Example 6: Arduino → ROS 2 Simple Integration

**Arduino publishes sensor data to ROS 2**

**Arduino Code:**
```cpp
// Arduino sends ultrasonic sensor data via serial
const int trigPin = 9;
const int echoPin = 10;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(115200);  // High baud rate for ROS 2
}

void loop() {
  float distance = readUltrasonic();
  
  // Send data in simple format: "DIST:25.3"
  Serial.print("DIST:");
  Serial.println(distance);
  
  delay(100);  // 10Hz update rate
}

float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  
  return distance;
}
```

**ROS 2 Python Node:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

class ArduinoUltrasonicNode(Node):
    def __init__(self):
        super().__init__('arduino_ultrasonic_node')
        
        # Create publisher for ultrasonic data
        self.publisher = self.create_publisher(Range, '/ultrasonic', 10)
        
        # Open serial connection to Arduino
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Timer to read serial data
        self.timer = self.create_timer(0.1, self.read_serial)
        
        self.get_logger().info('Arduino Ultrasonic Node Started')
    
    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            try:
                # Read line from Arduino
                line = self.serial_port.readline().decode('utf-8').strip()
                
                # Parse "DIST:25.3"
                if line.startswith('DIST:'):
                    distance_cm = float(line.split(':')[1])
                    distance_m = distance_cm / 100.0  # Convert to meters
                    
                    # Create Range message
                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'ultrasonic_link'
                    msg.radiation_type = Range.ULTRASOUND
                    msg.field_of_view = 0.26  # ~15 degrees in radians
                    msg.min_range = 0.02
                    msg.max_range = 4.0
                    msg.range = distance_m
                    
                    # Publish to ROS 2
                    self.publisher.publish(msg)
                    
                    self.get_logger().info(f'Published distance: {distance_m:.2f}m')
                    
            except Exception as e:
                self.get_logger().warn(f'Error reading serial: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoUltrasonicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to run:**
```bash
# Terminal 1: Upload Arduino code

# Terminal 2: Run ROS 2 node
python3 arduino_ultrasonic_node.py

# Terminal 3: View data
ros2 topic echo /ultrasonic
```

---

### Example 7: ROS 2 → Arduino Motor Control

**ROS 2 sends motor commands to Arduino**

**Arduino Code:**
```cpp
// Arduino receives motor commands from ROS 2 and controls motors
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 5;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("Motor controller ready");
}

void loop() {
  if (Serial.available() > 0) {
    // Read command: "MOTOR:150" (speed -255 to 255)
    String command = Serial.readStringUntil('\n');
    
    if (command.startsWith("MOTOR:")) {
      int speed = command.substring(6).toInt();
      setMotorSpeed(speed);
      
      // Send acknowledgment
      Serial.print("ACK:");
      Serial.println(speed);
    }
  }
}

void setMotorSpeed(int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}
```

**ROS 2 Python Node:**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Open serial connection to Arduino
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        self.get_logger().info('Motor Control Node Started')
    
    def cmd_vel_callback(self, msg):
        # Get linear velocity from Twist message
        linear_velocity = msg.linear.x  # m/s
        
        # Convert to motor PWM (-255 to 255)
        # Assume max velocity = 0.5 m/s corresponds to PWM 255
        max_velocity = 0.5
        motor_speed = int((linear_velocity / max_velocity) * 255)
        
        # Clamp to valid range
        motor_speed = max(-255, min(255, motor_speed))
        
        # Send command to Arduino
        command = f"MOTOR:{motor_speed}\n"
        self.serial_port.write(command.encode('utf-8'))
        
        self.get_logger().info(f'Sent motor command: {motor_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to test:**
```bash
# Terminal 1: Upload Arduino code

# Terminal 2: Run ROS 2 node
python3 motor_control_node.py

# Terminal 3: Send test command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"
```

---

## 📚 Essential Arduino Libraries for Robotics

### Built-in Libraries (No installation needed):

```cpp
#include <Servo.h>          // Control servo motors
#include <Wire.h>           // I2C communication
#include <SPI.h>            // SPI communication
#include <EEPROM.h>         // Save data to EEPROM
```

### Popular Third-Party Libraries:

**Motor Control:**
```cpp
// L298N, TB6612 motor drivers - no library needed, use digitalWrite/analogWrite
// Or use: AFMotor library for Adafruit Motor Shield
```

**Sensors:**
```cpp
#include <NewPing.h>        // Ultrasonic sensors (HC-SR04)
#include <MPU6050.h>        // IMU sensor
#include <DHT.h>            // Temperature/humidity (DHT11, DHT22)
```

**Displays:**
```cpp
#include <LiquidCrystal_I2C.h>  // I2C LCD display
#include <Adafruit_SSD1306.h>   // OLED display
```

**Communication:**
```cpp
#include <SoftwareSerial.h>  // Software UART on any pins
#include <WiFi.h>            // ESP32 WiFi (built-in)
#include <BluetoothSerial.h> // ESP32 Bluetooth (built-in)
```

**Encoders:**
```cpp
#include <Encoder.h>         // Paul Stoffregen's Encoder library
```

### Installing Libraries:

**Method 1: Library Manager (Easy)**
1. Arduino IDE → Tools → Manage Libraries
2. Search for library name (e.g., "NewPing")
3. Click Install

**Method 2: Manual Installation**
1. Download .zip from GitHub
2. Arduino IDE → Sketch → Include Library → Add .ZIP Library
3. Select downloaded file

### Example: Using Servo Library

```cpp
#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);  // Servo on pin 9
}

void loop() {
  myServo.write(0);    // 0 degrees
  delay(1000);
  myServo.write(90);   // 90 degrees
  delay(1000);
  myServo.write(180);  // 180 degrees
  delay(1000);
}
```

### Example: Using NewPing Library

```cpp
#include <NewPing.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 200  // cm

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(50);  // Wait between pings
  unsigned int distance = sonar.ping_cm();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
```

### Example: Using MPU6050 Library

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected!");
  }
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);
  
  Serial.print("Accel: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.println(az);
  
  delay(100);
}
```

---

## 🔧 Arduino Best Practices for Robotics

### 1. Use Non-Blocking Code
```cpp
// ❌ BAD: Blocks entire program
void loop() {
  digitalWrite(LED, HIGH);
  delay(1000);  // Robot can't respond during this time!
  digitalWrite(LED, LOW);
  delay(1000);
}

// ✅ GOOD: Non-blocking
unsigned long previousMillis = 0;
bool ledState = LOW;

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(LED, ledState);
  }
  
  // Other code runs continuously!
  readSensors();
  updateMotors();
}
```

### 2. Use Interrupts for Encoders
```cpp
const int encoderPinA = 2;  // Must be interrupt-capable pin
volatile long encoderCount = 0;

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
}

void encoderISR() {
  encoderCount++;  // Keep ISR short!
}

void loop() {
  Serial.println(encoderCount);
  delay(100);
}
```

### 3. Organize Code into Functions
```cpp
void setup() {
  Serial.begin(9600);
  setupMotors();
  setupSensors();
}

void loop() {
  readSensors();
  makeDecisions();
  updateMotors();
  sendTelemetry();
  delay(50);
}

void setupMotors() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // ...
}

void readSensors() {
  distance = readUltrasonic();
  encoderValue = readEncoder();
}
```

### 4. Use #define for Constants
```cpp
#define LED_PIN 13
#define MOTOR_SPEED 150
#define DISTANCE_THRESHOLD 20

void loop() {
  if (distance < DISTANCE_THRESHOLD) {
    setMotorSpeed(MOTOR_SPEED);
  }
}
```

### 5. Add Serial Debugging
```cpp
#define DEBUG 1

void loop() {
  float distance = readUltrasonic();
  
  #if DEBUG
    Serial.print("Distance: ");
    Serial.println(distance);
  #endif
  
  // Set DEBUG to 0 to disable all debug prints
}
```

### 6. Handle Serial Timeouts
```cpp
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if (command.length() > 0) {
      processCommand(command);
    }
  }
}
```

### 7. Validate Sensor Readings
```cpp
float readUltrasonic() {
  long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
  
  if (duration == 0) {
    return 400.0;  // Out of range
  }
  
  float distance = duration * 0.034 / 2;
  
  if (distance < 2 || distance > 400) {
    return -1;  // Invalid reading
  }
  
  return distance;
}
```

---

## 📝 Contributing

This is a personal learning repository. Concepts are explained from a robotics software engineering perspective.

## 📚 References

- ROS 2 Humble Documentation
- Arduino Reference
- ESP32 Documentation
- Robotics textbooks and industry practices

---

**Author**: Robotics Software Engineer transitioning from Mechanical Engineering  
**Focus**: Practical electronics and embedded systems for real-world robotics applications

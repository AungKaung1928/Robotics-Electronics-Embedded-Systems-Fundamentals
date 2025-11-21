# Getting Started with Arduino and Robotics

Complete beginner's guide to Arduino, electronics basics, and building your first robot.

---

## 🛒 Shopping List for Beginners

### Essential Kit (Start Here):
- **Arduino Uno R3** + USB cable (~$25)
- **Breadboard** (830 tie-points) + jumper wires (~$10)
- **LED Starter Kit**: LEDs (red, green, blue, yellow), 220Ω resistors (~$5)
- **Resistor kit**: 220Ω, 1kΩ, 10kΩ assortment (~$5)
- **Push buttons** (4-pin tactile switches) (~$3)
- **Potentiometer** 10kΩ (~$2)
- **9V battery** + battery clip OR USB power bank

**Total: ~$50**

### Robot Car Kit (Next Step):
- **2WD Robot Car Chassis Kit** including:
  - Chassis (acrylic or metal)
  - 2× DC motors with wheels
  - Battery holder (4× AA or 18650)
  - (~$15-25)
- **L298N Motor Driver Module** (~$3)
- **HC-SR04 Ultrasonic Sensor** (~$2)
- **Batteries**: 4× AA or 18650 rechargeable
- **More jumper wires** (male-male, male-female)

**Total: ~$25**

### Optional but Useful:
- **Multimeter** (basic digital multimeter) (~$15)
- **Soldering iron kit** for permanent connections (~$20)
- **ESP32 DevKit** for WiFi/Bluetooth projects (~$8)
- **Servo motors** SG90 (for robot arms, camera gimbal) (~$3 each)

---

## 📚 Setup Arduino IDE

### 1. Download and Install:
- Go to https://www.arduino.cc/en/software
- Download Arduino IDE 2.x for your OS (Windows/Mac/Linux)
- Install following default settings

### 2. Connect Arduino Uno:
- Plug USB cable from Arduino to computer
- Windows: Driver should install automatically
- Mac/Linux: Usually works immediately

### 3. Select Board and Port:
- **Tools > Board > Arduino AVR Boards > Arduino Uno**
- **Tools > Port > COM3** (Windows) or **/dev/ttyUSB0** (Linux) or **/dev/cu.usbserial-xxxxx** (Mac)

### 4. Test Installation:
```cpp
void setup() {
  pinMode(13, OUTPUT);  // Built-in LED on pin 13
}

void loop() {
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
}
```

- Click **Upload** button (→)
- Built-in LED on Arduino should blink every 1 second
- Success! You're ready to go.

---

## 🔌 Breadboard Basics

### What is a Breadboard?

**Breadboard = Solderless prototyping board**

```
     1  2  3  4  5  (columns)
    ━━━━━━━━━━━━━━━━
  + ████████████████  ← Power rail (+) - all connected vertically
  - ████████████████  ← Ground rail (-) - all connected vertically
    ━━━━━━━━━━━━━━━━
a   ● ● ● ● ●         ← Rows a-e connected horizontally
b   ● ● ● ● ●
c   ● ● ● ● ●
d   ● ● ● ● ●
e   ● ● ● ● ●
    ━━━━━━━━━━━━━━━━  ← Center gap (no connection)
f   ● ● ● ● ●         ← Rows f-j connected horizontally
g   ● ● ● ● ●
h   ● ● ● ● ●
i   ● ● ● ● ●
j   ● ● ● ● ●
    ━━━━━━━━━━━━━━━━
  + ████████████████  ← Power rail (+)
  - ████████████████  ← Ground rail (-)
    ━━━━━━━━━━━━━━━━
```

### Key Concepts:
1. **Vertical power rails**: + and - columns run full length
2. **Horizontal rows**: a-e connected (1-2-3-4-5 all connected in row a)
3. **Center gap**: Separates top and bottom (for IC chips)
4. **No connection** between different rows (a and b NOT connected)

### Setting Up Power:
```
Arduino 5V → Breadboard + rail (red wire)
Arduino GND → Breadboard - rail (black wire)
```

Now all components can share 5V and GND easily!

---

## 💡 Project 1: Blink External LED

### Components:
- Arduino Uno
- Breadboard
- LED (any color)
- 220Ω resistor (Red-Red-Brown-Gold)
- 2× jumper wires

### Why 220Ω Resistor?
LEDs need **current limiting** or they burn out!

**Calculation**:
- Arduino pin: 5V
- LED forward voltage: ~2V (red/green/yellow), ~3V (blue/white)
- LED current: 20mA = 0.02A
- Resistor needed: R = (5V - 2V) / 0.02A = 150Ω → Use 220Ω (standard value, safer)

### Circuit Diagram:
```
Arduino Pin 8 → 220Ω Resistor → LED Long Leg (+)
                                 LED Short Leg (-) → Arduino GND
```

### Breadboard Layout:
```
Arduino Pin 8 ──────┐
                    │
Breadboard:         │
  Row 5: ●──●──●──●─┘
         │  
         └─ 220Ω resistor
         │
  Row 7: ●──●──●──●
         │
         └─ LED long leg
         │
  Row 9: ●──●──●──●
         │
         └─ LED short leg
         │
       - rail (GND) ──── Arduino GND
```

### Code:
```cpp
const int ledPin = 8;  // LED connected to pin 8

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);  // Turn ON
  delay(1000);                 // Wait 1 second
  digitalWrite(ledPin, LOW);   // Turn OFF
  delay(1000);                 // Wait 1 second
}
```

### Challenge:
- Make LED blink faster (100ms delay)
- Make LED blink in pattern: fast-fast-slow
- Connect 3 LEDs (red, green, blue) and blink in sequence

---

## 🔘 Project 2: Button Control LED

### Components:
- Arduino Uno
- Breadboard
- LED + 220Ω resistor
- Push button (4-pin tactile switch)
- 10kΩ resistor (Brown-Black-Orange-Gold)
- Jumper wires

### Circuit Diagram:
```
Button circuit (with pull-down resistor):
  Arduino 5V ─── Button ─┬─── Arduino Pin 2
                         │
                         └─── 10kΩ Resistor ─── GND

LED circuit:
  Arduino Pin 8 ─── 220Ω Resistor ─── LED (+) ─── LED (-) ─── GND
```

### Why Pull-down Resistor?
Without resistor, pin 2 is "floating" (undefined state) when button not pressed.

- **Button pressed**: Pin 2 = HIGH (5V)
- **Button released**: Pin 2 = LOW (0V, through 10kΩ to GND)

### Code:
```cpp
const int buttonPin = 2;
const int ledPin = 8;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  
  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);
    Serial.println("Button pressed - LED ON");
  } else {
    digitalWrite(ledPin, LOW);
    Serial.println("Button released - LED OFF");
  }
  
  delay(100);
}
```

### Alternative: INPUT_PULLUP
Arduino has **internal pull-up resistor** (no external resistor needed!)

**Circuit:**
```
Button: One side → Arduino Pin 2
        Other side → GND
LED: Same as before
```

**Code:**
```cpp
const int buttonPin = 2;
const int ledPin = 8;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  // Enable internal pull-up
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  
  if (buttonState == LOW) {  // Note: LOW when pressed (opposite!)
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}
```

**Important**: With `INPUT_PULLUP`, button reads `LOW` when pressed!

### Challenge:
- Toggle LED on/off with each button press (not just hold)
- Connect 3 buttons and 3 LEDs - each button controls one LED
- Debounce button (avoid multiple triggers from one press)

---

## 🎛️ Project 3: Potentiometer Control LED Brightness

### Components:
- Arduino Uno
- Breadboard
- LED + 220Ω resistor
- Potentiometer (10kΩ)
- Jumper wires

### What is a Potentiometer?
**Variable resistor** with 3 pins:
- **Pin 1**: One end (connect to 5V)
- **Pin 2**: Wiper/middle (output, connect to Arduino analog pin)
- **Pin 3**: Other end (connect to GND)

Turn knob → resistance changes → voltage changes (0V to 5V)

### Circuit Diagram:
```
Potentiometer:
  Pin 1 (left)   → Arduino 5V
  Pin 2 (middle) → Arduino A0
  Pin 3 (right)  → Arduino GND

LED:
  Arduino Pin 9 → 220Ω Resistor → LED (+) → LED (-) → GND
```

### Code:
```cpp
const int potPin = A0;    // Potentiometer on analog pin A0
const int ledPin = 9;     // LED on PWM pin 9 (must be PWM-capable!)

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Read potentiometer (0-1023)
  int potValue = analogRead(potPin);
  
  // Convert to LED brightness (0-255)
  int brightness = map(potValue, 0, 1023, 0, 255);
  
  // Set LED brightness using PWM
  analogWrite(ledPin, brightness);
  
  // Print values
  Serial.print("Pot: ");
  Serial.print(potValue);
  Serial.print(" | Brightness: ");
  Serial.println(brightness);
  
  delay(100);
}
```

### Key Concepts:
- **analogRead(A0)**: Returns 0-1023 (10-bit ADC)
- **analogWrite(pin, value)**: PWM output, 0-255 (0% to 100% duty cycle)
- **map()**: Converts one range to another
- **PWM pins**: Only certain pins support analogWrite (3, 5, 6, 9, 10, 11 on Uno)

### Challenge:
- Control 3 LEDs brightness with 3 potentiometers
- Use pot to control servo motor angle
- Display pot value on Serial Plotter (Tools > Serial Plotter)

---

## 📏 Project 4: Ultrasonic Distance Sensor

### Components:
- Arduino Uno
- HC-SR04 Ultrasonic Sensor
- Breadboard
- Jumper wires

### HC-SR04 Pinout:
```
┌─────────────┐
│   HC-SR04   │
│  [  ]  [  ] │  ← Ultrasonic transmitter/receiver
└──┬──┬──┬──┬─┘
   │  │  │  │
  VCC Trig Echo GND
```

### How It Works:
1. Arduino sends 10μs pulse to **Trig** pin
2. HC-SR04 emits 8 ultrasonic pulses (40kHz)
3. Ultrasonic waves bounce off object
4. **Echo** pin goes HIGH for duration = time-of-flight
5. Calculate distance: `distance = (time × speed_of_sound) / 2`

### Circuit:
```
HC-SR04:
  VCC  → Arduino 5V
  Trig → Arduino Pin 9
  Echo → Arduino Pin 10
  GND  → Arduino GND
```

### Code:
```cpp
const int trigPin = 9;
const int echoPin = 10;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Send 10μs pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure echo pulse duration (μs)
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance
  // Speed of sound = 343 m/s = 0.0343 cm/μs
  // distance = (duration × 0.0343) / 2
  float distance = duration * 0.034 / 2;  // cm
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  delay(500);
}
```

### Understanding the Math:
```
Sound travels 343 m/s = 34300 cm/s = 0.0343 cm/μs

Time measured includes:
  Arduino → Object → Arduino (round trip)

So actual distance = (total time × speed) / 2

Example:
  duration = 1000 μs
  distance = (1000 × 0.034) / 2 = 17 cm
```

### Challenge:
- Add LED that lights up when object is closer than 10cm
- Add buzzer that beeps faster as object gets closer (parking sensor)
- Display distance on LCD display

---

## 🚗 Project 5: Building a Simple Robot Car

### Components:
- Arduino Uno
- 2WD Robot Car Chassis Kit (motors, wheels, chassis)
- L298N Motor Driver Module
- HC-SR04 Ultrasonic Sensor
- Battery pack (6-12V, 4× AA or 18650)
- Jumper wires

### Understanding L298N Motor Driver

**Why do we need it?**
- Arduino pins: Max 40mA
- DC motors: Need 100mA - 2A
- **Solution**: L298N drives high-current motors, Arduino controls L298N with low-current signals

**L298N Pinout:**
```
┌──────────────────────────────┐
│         L298N Module          │
├───────────────────────────────┤
│ 12V  GND  5V  (Power input)   │
│                               │
│ IN1 IN2 ENA  (Motor A control)│
│ IN3 IN4 ENB  (Motor B control)│
│                               │
│ OUT1 OUT2    (Motor A output) │
│ OUT3 OUT4    (Motor B output) │
└───────────────────────────────┘
```

**Control Logic:**

| IN1 | IN2 | Motor A Direction |
|-----|-----|-------------------|
| LOW | LOW | Stop |
| HIGH | LOW | Forward |
| LOW | HIGH | Backward |
| HIGH | HIGH | Brake |

**ENA (Enable A)**: 
- HIGH = Motor ON
- PWM = Variable speed (0-255)

### Circuit Connections:

**L298N to Arduino:**
```
IN1 → Arduino Pin 8
IN2 → Arduino Pin 9
IN3 → Arduino Pin 10
IN4 → Arduino Pin 11
ENA → Arduino Pin 5 (PWM-capable)
ENB → Arduino Pin 6 (PWM-capable)
GND → Arduino GND
```

**L298N to Motors:**
```
OUT1, OUT2 → Left Motor
OUT3, OUT4 → Right Motor
```

**L298N to Battery:**
```
12V → Battery + (6-12V)
GND → Battery -
```

**Power Arduino from L298N:**
```
L298N 5V → Arduino 5V pin
L298N GND → Arduino GND
```

**Ultrasonic Sensor:**
```
VCC  → Arduino 5V
Trig → Arduino Pin 3
Echo → Arduino Pin 4
GND  → Arduino GND
```

### Complete Robot Code:

```cpp
// Motor driver pins
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
  
  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600);
  Serial.println("Robot Car Started!");
}

void loop() {
  // Measure distance
  float distance = readUltrasonic();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Decision making
  if (distance > 30) {
    // Clear path → Move forward
    moveForward(150);  // Speed 0-255
  } else if (distance > 15) {
    // Getting close → Slow down
    moveForward(100);
  } else {
    // Obstacle! → Stop, back up, turn
    stopMotors();
    delay(500);
    
    moveBackward(150);
    delay(800);
    
    turnRight(150);
    delay(600);
  }
  
  delay(100);
}

// Motor control functions

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

void moveBackward(int speed) {
  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
  
  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  // Right motor backward (or stop for gentler turn)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void turnLeft(int speed) {
  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
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

float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout
  
  if (duration == 0) {
    return 400;  // No echo = far away
  }
  
  float distance = duration * 0.034 / 2;
  return distance;
}
```

### Testing Procedure:

1. **Test motors individually first:**
```cpp
void loop() {
  moveForward(100);
  delay(2000);
  stopMotors();
  delay(1000);
}
```

2. **Test ultrasonic sensor:**
```cpp
void loop() {
  float distance = readUltrasonic();
  Serial.println(distance);
  delay(500);
}
```

3. **Test motor directions:**
- If robot moves backward when it should go forward → Swap motor wires
- If robot turns wrong direction → Swap left and right motor connections
- If one wheel is faster → Adjust speed: `analogWrite(ENA, speed * 0.9);`

### Common Issues:

**Motors don't spin:**
- Check battery voltage (must be 6V+)
- Check motor wire connections
- Check ENA/ENB pins are HIGH or PWM
- Check jumper caps on L298N (should be ON for basic mode)

**Robot moves erratically:**
- Check battery level (weak battery = erratic movement)
- Add decoupling capacitor (100μF) across battery terminals
- Separate motor and logic grounds (single connection point)

**Ultrasonic reads 0 or wrong values:**
- Check wiring (Trig, Echo not swapped)
- Check 5V power to sensor
- Add timeout to pulseIn: `pulseIn(echoPin, HIGH, 30000);`

---

## 📚 Next Steps

### Intermediate Projects:
1. **Line Following Robot** (add IR sensors)
2. **Bluetooth Controlled Car** (add HC-05 Bluetooth module)
3. **WiFi Robot with Camera** (ESP32-CAM)
4. **Robot Arm** (servo motors + joystick control)
5. **Balancing Robot** (MPU6050 IMU + PID control)

### Learn More:
- [Sensors](../03-sensors/) - Read different sensor types
- [Motor Control](../05-motor-control/) - Learn PID control
- [Communication Protocols](../07-communication-protocols/) - Connect multiple devices
- [ROS 2 Hardware Integration](../10-ros2-hardware-integration/) - Connect Arduino to ROS 2

### Resources:
- Arduino Official Tutorials: https://www.arduino.cc/en/Tutorial/HomePage
- Arduino Forum: https://forum.arduino.cc/
- YouTube: "Paul McWhorter Arduino Tutorial"
- YouTube: "How To Mechatronics"

---

**Now you're ready to build robots! 🤖**

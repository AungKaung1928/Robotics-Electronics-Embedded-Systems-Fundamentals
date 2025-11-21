# Arduino Troubleshooting Guide

Common problems and solutions when working with Arduino for robotics.

---

## 🔴 Upload Issues

### Problem: "Port not found" or "Access denied"

**Linux:**
```bash
# Check if Arduino is detected
lsusb | grep Arduino

# Check port name
ls /dev/ttyUSB* /dev/ttyACM*

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in, or run:
newgrp dialout

# Give temporary permissions (until reboot)
sudo chmod 666 /dev/ttyUSB0
```

**Windows:**
- Install Arduino drivers from arduino.cc
- Check Device Manager for COM port
- Close Serial Monitor before uploading

**Mac:**
- Port usually `/dev/cu.usbserial-*` or `/dev/cu.usbmodem*`
- Install CH340 drivers if using cheap clones

---

### Problem: "Sketch too big"

**Error:** `Sketch uses 33000 bytes (101%) of program storage space`

**Solutions:**
1. Remove unused libraries:
```cpp
// Remove:
#include <LiquidCrystal.h>  // If not using LCD
```

2. Use F() macro for strings:
```cpp
// ❌ Uses RAM
Serial.println("This uses RAM");

// ✅ Stores in Flash
Serial.println(F("This saves RAM"));
```

3. Reduce array sizes:
```cpp
// ❌ Large array
int dataBuffer[1000];

// ✅ Smaller array
int dataBuffer[100];
```

4. Use smaller board (if possible):
- Arduino Nano → Arduino Mega (more space)

---

### Problem: Upload works but nothing happens

**Check:**
1. **Power**: Is Arduino getting power? (Built-in LED should light up)
2. **Correct board selected**: Tools → Board → Arduino Uno (or your board)
3. **Code in loop()**: Ensure code is in `loop()`, not just `setup()`
4. **Serial pins**: Don't use pins 0, 1 if Serial needed

**Test:**
```cpp
void setup() {
  pinMode(13, OUTPUT);  // Built-in LED
}

void loop() {
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
}
```

If built-in LED blinks → Arduino works, problem is in your circuit

---

## ⚡ Hardware Issues

### Problem: LED doesn't light up

**Check:**
1. **Polarity**: Long leg (+) to resistor, short leg (-) to GND
2. **Resistor**: 220Ω - 1kΩ required (LED will burn out without it!)
3. **Pin mode**: `pinMode(pin, OUTPUT);` in setup()
4. **Voltage**: Pin should output 5V (measure with multimeter)
5. **Breadboard connection**: Ensure good contact

**Test LED separately:**
```
Arduino 5V → 220Ω Resistor → LED (+) → LED (-) → GND
```

If LED lights → Problem is in code  
If LED doesn't light → Check LED polarity or replace LED

---

### Problem: Button doesn't work / reads random values

**Issue:** Floating pin (undefined state when button not pressed)

**Solution 1: External pull-down resistor**
```
Button: One side → 5V
        Other side → Arduino Pin 2 + 10kΩ → GND
```

**Solution 2: INPUT_PULLUP (easier)**
```cpp
pinMode(buttonPin, INPUT_PULLUP);  // Use internal pull-up

// Note: Button reads LOW when pressed!
if (digitalRead(buttonPin) == LOW) {
  // Button is pressed
}
```

---

### Problem: Motor doesn't spin

**Check:**
1. **Power supply**: Motors need separate power (battery, not USB)
2. **Voltage**: Battery voltage sufficient? (6-12V for most motors)
3. **Connections**: Verify motor driver wiring
4. **Enable pin**: ENA/ENB must be HIGH or PWM
5. **Direction pins**: IN1/IN2 must be opposite (one HIGH, one LOW)

**Test motor directly:**
- Connect motor directly to battery
- If spins → Problem is motor driver or code
- If doesn't spin → Motor is broken or voltage too low

**Test motor driver:**
```cpp
// L298N test code
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
analogWrite(ENA, 255);  // Full speed

// Motor should spin at full speed
```

---

### Problem: Ultrasonic sensor reads 0 or wrong values

**Check:**
1. **Wiring**: Trig and Echo not swapped
2. **Power**: VCC to 5V, GND to GND
3. **Timeout**: Add timeout to `pulseIn()`:
```cpp
long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms timeout

if (duration == 0) {
  // No echo received
  return 400;  // Out of range
}
```

4. **Angle**: HC-SR04 has ~15° beam angle
5. **Distance**: Too close (<2cm) or too far (>400cm)

**Test code:**
```cpp
void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  
  Serial.print("Duration: ");
  Serial.print(duration);
  Serial.print(" | Distance: ");
  Serial.println(duration * 0.034 / 2);
  
  delay(500);
}
```

---

### Problem: Servo jitters or doesn't move

**Check:**
1. **Power**: Servo needs separate 5-6V power (not Arduino 5V pin)
```
Servo Red (V+) → External 5V (battery with voltage regulator)
Servo Brown (GND) → Common GND with Arduino
Servo Orange (Signal) → Arduino PWM pin
```

2. **PWM pin**: Must use PWM-capable pin (3, 5, 6, 9, 10, 11 on Uno)

3. **Angle range**: Most servos: 0-180°
```cpp
myServo.write(0);    // Minimum
myServo.write(90);   // Center
myServo.write(180);  // Maximum
```

4. **Code:**
```cpp
#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);
}

void loop() {
  myServo.write(90);  // Center position
  delay(2000);
}
```

---

## 💻 Software Issues

### Problem: Serial Monitor shows garbage characters

**Solutions:**
1. **Baud rate mismatch**:
```cpp
Serial.begin(9600);  // Code

// Serial Monitor must also be set to 9600 (bottom-right dropdown)
```

2. **Board still uploading**: Wait for upload to complete before opening Serial Monitor

3. **Wrong board selected**: Tools → Board → Correct board

---

### Problem: Program crashes or resets randomly

**Possible causes:**

1. **Power supply insufficient**
```cpp
// Check voltage with multimeter
// Arduino needs 7-12V on VIN, or 5V on 5V pin
// Motors need separate battery
```

2. **SRAM overflow** (Arduino Uno has only 2KB!)
```cpp
// ❌ BAD: Large array
int bigArray[1000];  // 2000 bytes!

// ✅ GOOD: Smaller array
int smallArray[50];  // 100 bytes

// Check free RAM:
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

Serial.println(freeRam());  // Should be >100 bytes
```

3. **Stack overflow** (too many nested function calls)

4. **Brown-out reset** (voltage drops below threshold)
- Add decoupling capacitor (100μF) near Arduino power pins
- Use separate power for motors

---

### Problem: Encoder counts are wrong

**Issue:** Polling encoder pins (too slow, misses pulses)

**Solution:** Use interrupts
```cpp
const int encoderPinA = 2;  // Must be interrupt-capable
const int encoderPinB = 3;
volatile long encoderCount = 0;

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);
}

void encoderISR() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
```

**Interrupt-capable pins:**
- **Arduino Uno**: Pins 2, 3
- **Arduino Mega**: Pins 2, 3, 18, 19, 20, 21
- **ESP32**: Any GPIO pin

---

### Problem: I2C device not found

**Check:**
1. **Pull-up resistors**: I2C requires 4.7kΩ pull-ups on SDA and SCL
```
SDA line → 4.7kΩ → 5V
SCL line → 4.7kΩ → 5V
```

2. **Wiring**:
```
Arduino Uno: SDA = A4, SCL = A5
Arduino Mega: SDA = 20, SCL = 21
ESP32: SDA = GPIO 21, SCL = GPIO 22
```

3. **I2C address**: Use scanner to find device address
```cpp
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Scanning I2C bus...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }
  }
}

void loop() {}
```

4. **Voltage level**: 3.3V device with 5V Arduino needs level shifter

---

## 🔍 Debugging Tips

### 1. Serial Print Everything
```cpp
void loop() {
  int sensorValue = analogRead(A0);
  
  Serial.print("Sensor: ");
  Serial.print(sensorValue);
  Serial.print(" | Calculated: ");
  Serial.println(sensorValue * 5.0 / 1023.0);
}
```

### 2. LED Debugging (when Serial doesn't work)
```cpp
void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  if (errorCondition) {
    // Fast blink = error
    for (int i = 0; i < 5; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
  } else {
    // Slow blink = OK
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  }
}
```

### 3. Binary Search Debugging
```cpp
void loop() {
  readSensors();
  Serial.println("Checkpoint 1");  // Add checkpoints
  
  calculatePID();
  Serial.println("Checkpoint 2");
  
  updateMotors();
  Serial.println("Checkpoint 3");
  
  // If code stops printing after "Checkpoint 1",
  // problem is in calculatePID()
}
```

### 4. Use Multimeter
- Verify voltage: 5V rail should be 5.0V ±0.2V
- Check continuity: Test wire connections
- Measure current: Insert meter in series (check if motor draws expected current)

---

## 🛠️ Common Fixes

### Robot moves erratically
- **Weak battery**: Charge or replace batteries
- **Poor connections**: Re-seat all jumper wires
- **Noise from motors**: Add 0.1μF capacitor across motor terminals

### Code uploads slowly
- Lower upload speed: File → Preferences → Upload speed → 57600

### Arduino doesn't respond
- Press reset button
- Re-upload bootloader: Tools → Burn Bootloader (requires programmer)

### Can't find port on Linux
```bash
# Permanently add to dialout group
sudo usermod -a -G dialout $USER
sudo reboot
```

---

## 📚 Resources

- Arduino Forum: https://forum.arduino.cc/
- Arduino Troubleshooting Guide: https://www.arduino.cc/en/Guide/Troubleshooting
- Multimeter Tutorial: SparkFun "How to Use a Multimeter"
- Oscilloscope basics: EEVblog #112

---

**When stuck: Simplify, test components individually, use Serial debugging!**

# Arduino Troubleshooting Guide

Common problems and quick solutions.

---

## Upload Issues

### "Port not found" or "Access denied"

**Linux:**
```bash
sudo usermod -a -G dialout $USER
sudo reboot
# Or temporary:
sudo chmod 666 /dev/ttyUSB0
```

**Windows:**
- Install drivers from arduino.cc
- Close Serial Monitor before upload

**Mac:**
- Install CH340 drivers for cheap clones

---

### "Sketch too big"

```cpp
// Use F() macro to save RAM
Serial.println(F("Text in Flash"));

// Remove unused libraries
// #include <Library.h>  // Comment out
```

---

### Upload works but nothing happens

1. Check correct board selected (Tools → Board)
2. Check code is in `loop()` not just `setup()`
3. Test with blink LED:
```cpp
void setup() { pinMode(13, OUTPUT); }
void loop() { 
  digitalWrite(13, HIGH); delay(1000);
  digitalWrite(13, LOW); delay(1000);
}
```

---

## Hardware Issues

### LED doesn't work

- Check polarity: Long leg (+) to resistor, short leg (-) to GND
- Need 220Ω resistor or LED burns out
- Check `pinMode(pin, OUTPUT)` in setup

---

### Button doesn't work

Use `INPUT_PULLUP`:
```cpp
pinMode(buttonPin, INPUT_PULLUP);
if (digitalRead(buttonPin) == LOW) {  // Pressed
  // Do something
}
```

---

### Motor doesn't spin

1. Separate power for motors (not USB)
2. Check battery voltage (6-12V)
3. Enable pin must be HIGH or PWM
4. Test motor directly with battery

---

### Ultrasonic reads 0 or wrong values

```cpp
long duration = pulseIn(echoPin, HIGH, 30000);  // Add timeout
if (duration == 0) return 400;  // Out of range
```

---

### Servo jitters

- Use separate 5V power (not Arduino 5V pin)
- Common GND between Arduino and servo power
```cpp
Servo Red → External 5V
Servo Brown → Common GND
Servo Orange → Arduino PWM pin
```

---

## Software Issues

### Serial Monitor shows garbage

- Match baud rate: `Serial.begin(9600)` and Serial Monitor = 9600
- Wait for upload to finish before opening

---

### Random crashes/resets

```cpp
// Check free RAM (should be >100 bytes)
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
```

- Use smaller arrays
- Add 100μF capacitor on Arduino power pins

---

### Encoder counts wrong

Use interrupts:
```cpp
volatile long count = 0;

void setup() {
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, CHANGE);
}

void encoderISR() {
  count++;
}
```

---

### I2C device not found

1. Add 4.7kΩ pull-up resistors on SDA and SCL
2. Check wiring: Arduino Uno (SDA=A4, SCL=A5)
3. Scan for address:
```cpp
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin();
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found: 0x");
      Serial.println(addr, HEX);
    }
  }
}
```

---

## Debugging Tips

### 1. Serial Debug Everything
```cpp
Serial.print("Sensor: ");
Serial.println(sensorValue);
```

### 2. LED Debugging
```cpp
// Fast blink = error, slow = OK
if (error) {
  for (int i=0; i<5; i++) {
    digitalWrite(13, HIGH); delay(100);
    digitalWrite(13, LOW); delay(100);
  }
}
```

### 3. Test Components Individually
- Test sensor alone first
- Test motor alone first
- Combine when both work

### 4. Use Multimeter
- Check 5V rail = 5.0V
- Check continuity of wires
- Measure motor current

---

## Common Fixes

| Problem | Fix |
|---------|-----|
| Port permission | `sudo usermod -a -G dialout $USER` |
| Weak battery | Replace/charge batteries |
| Loose wires | Re-seat all connections |
| Motor noise | Add 0.1μF capacitor across motor terminals |
| Can't find port (Linux) | `ls /dev/ttyUSB*` or `ls /dev/ttyACM*` |

# Microcontrollers

Arduino, ESP32, and embedded programming basics for robot control.

---

## 🔷 What is a Microcontroller?

A small computer on a single integrated circuit containing:
- **CPU**: Executes program instructions
- **Memory**: RAM (temporary data), Flash (program storage), EEPROM (persistent data)
- **GPIO**: General Purpose Input/Output pins
- **Peripherals**: ADC, PWM, timers, UART, I2C, SPI, etc.

### Microcontroller vs Microprocessor:
- **Microcontroller**: All-in-one (CPU + memory + I/O) → Arduino, ESP32
- **Microprocessor**: Just CPU, needs external components → Raspberry Pi CPU

---

## 🤖 Arduino Basics

### Popular Arduino Boards:

#### Arduino Uno (ATmega328P)
- **MCU**: 8-bit AVR @ 16MHz
- **Flash**: 32KB (program storage)
- **SRAM**: 2KB (runtime variables)
- **EEPROM**: 1KB (persistent data)
- **Digital I/O**: 14 pins (6 PWM-capable)
- **Analog Input**: 6 pins (10-bit ADC, 0-5V)
- **Voltage**: 5V logic
- **Price**: ~$25

**Use Cases**: Learning, simple robots, sensor interfacing

#### Arduino Mega (ATmega2560)
- **MCU**: 8-bit AVR @ 16MHz
- **Flash**: 256KB
- **SRAM**: 8KB
- **Digital I/O**: 54 pins (15 PWM)
- **Analog Input**: 16 pins
- **Voltage**: 5V logic

**Use Cases**: Complex robots (many sensors/actuators), robot arms

#### Arduino Nano
- **Same as Uno** but smaller form factor
- Breadboard-friendly
- USB Mini/Micro instead of USB-B

**Use Cases**: Compact robots, embedded projects

### Arduino Pinout:

**Digital Pins (0-13)**:
- Can be INPUT or OUTPUT
- Pins with ~ symbol: PWM-capable (3, 5, 6, 9, 10, 11)
- Pins 0, 1: UART (TX, RX) — avoid using if Serial communication needed

**Analog Pins (A0-A5)**:
- 10-bit ADC: 0-1023 range for 0-5V
- Can also be used as digital I/O (digitalWrite, digitalRead)

**Power Pins**:
- **5V**: Regulated 5V output (max 500mA)
- **3.3V**: Regulated 3.3V output (max 50mA)
- **GND**: Ground (multiple pins, all connected)
- **Vin**: Power input (7-12V recommended)

**Special Pins**:
- **RESET**: Pull LOW to reset microcontroller
- **AREF**: External analog reference voltage
- **ICSP**: In-Circuit Serial Programming header

---

## 📡 ESP32 Basics

### Why ESP32 over Arduino?
- **Faster**: 240MHz dual-core vs 16MHz single-core
- **More memory**: 520KB SRAM vs 2KB
- **Built-in WiFi and Bluetooth**
- **More ADC channels**: 18× 12-bit ADC
- **More PWM channels**: 16 independent
- **3.3V logic**: Modern sensor compatible

### ESP32 Specifications:
- **MCU**: Xtensa 32-bit LX6 dual-core @ 240MHz
- **Flash**: 4MB (typical)
- **SRAM**: 520KB
- **GPIO**: 34 pins (some input-only)
- **ADC**: 18 channels, 12-bit (0-4095)
- **DAC**: 2 channels, 8-bit
- **PWM**: 16 channels
- **Interfaces**: I2C (2), SPI (4), UART (3), I2S, CAN
- **Wireless**: WiFi 802.11 b/g/n, Bluetooth 4.2/BLE
- **Voltage**: 3.3V logic (5V-tolerant inputs)
- **Price**: ~$5-10

### ESP32 Important Notes:

**Input-Only Pins**: GPIOs 34-39 cannot be used as outputs

**Strapping Pins** (affect boot mode): GPIOs 0, 2, 5, 12, 15
- Avoid using during boot unless you know what you're doing
- GPIO 0: Must be HIGH at boot (pulled HIGH internally)

**ADC2 pins** (GPIOs 0, 2, 4, 12-15, 25-27): Cannot use ADC when WiFi is active

**Touch-sensitive pins**: GPIOs 0, 2, 4, 12-15, 27, 32, 33 (capacitive touch)

### Programming ESP32:
```cpp
// Same Arduino IDE, slightly different syntax
void setup() {
  Serial.begin(115200);  // ESP32 typically uses 115200 baud
  pinMode(2, OUTPUT);    // Built-in LED on most boards
}

void loop() {
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  delay(1000);
}
```

---

## 🧩 GPIO (General Purpose Input/Output)

### Digital Output:
```cpp
pinMode(13, OUTPUT);       // Set pin 13 as output
digitalWrite(13, HIGH);    // Set pin HIGH (5V on Arduino, 3.3V on ESP32)
digitalWrite(13, LOW);     // Set pin LOW (0V)
```

**Applications**: Control LEDs, relays, motor drivers, servos

### Digital Input:
```cpp
pinMode(7, INPUT);         // Set pin 7 as input (floating)
pinMode(7, INPUT_PULLUP);  // Set pin 7 as input with internal pull-up resistor

int buttonState = digitalRead(7);  // Read pin state (HIGH or LOW)
```

**INPUT vs INPUT_PULLUP**:
- **INPUT**: Pin floats (undefined state when not connected)
- **INPUT_PULLUP**: Pin pulled to HIGH internally, goes LOW when button connects to GND

---

## 📊 Analog Input (ADC Pins)

### Reading Analog Voltage:
```cpp
int sensorValue = analogRead(A0);  // Read ADC (0-1023 on Arduino, 0-4095 on ESP32)

// Convert to voltage
float voltage = sensorValue * (5.0 / 1023.0);  // Arduino Uno
// float voltage = sensorValue * (3.3 / 4095.0);  // ESP32
```

**Applications**: Read sensors (temperature, distance, light, potentiometers)

### Analog Reference (Arduino):
```cpp
analogReference(DEFAULT);   // 5V reference (Uno default)
analogReference(INTERNAL);  // 1.1V reference (more precision for low voltages)
analogReference(EXTERNAL);  // Use voltage on AREF pin
```

---

## ⚡ PWM Output

### Generate PWM:
```cpp
analogWrite(9, 128);  // 50% duty cycle (0-255 scale)
analogWrite(9, 255);  // 100% duty cycle (full on)
analogWrite(9, 0);    // 0% duty cycle (off)
```

**PWM-capable pins**: 
- **Arduino Uno**: 3, 5, 6, 9, 10, 11
- **ESP32**: Any GPIO pin (up to 16 channels)

**Applications**: Motor speed control, LED dimming, servo control

### ESP32 Advanced PWM (LEDC):
```cpp
const int freq = 5000;      // 5kHz
const int ledChannel = 0;
const int resolution = 8;   // 8-bit (0-255)

void setup() {
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(25, ledChannel);  // Attach channel to GPIO 25
  
  ledcWrite(ledChannel, 128);  // 50% duty cycle
}
```

---

## ⏱️ Timers and Delays

### Blocking Delays:
```cpp
delay(1000);       // Delay 1 second (1000 milliseconds)
delayMicroseconds(100);  // Delay 100 microseconds
```

**Problem**: Blocks entire program (can't read sensors or respond during delay)

### Non-Blocking Timing:
```cpp
unsigned long previousMillis = 0;
const long interval = 1000;  // 1 second

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Do something every 1 second
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Toggle LED
  }
  
  // Other code runs continuously
}
```

**Advantage**: Program can multitask (read sensors, update display, control motors simultaneously)

---

## 🔔 Interrupts

### What are Interrupts?
Function called immediately when external event occurs (pin change, timer overflow)

### Hardware Interrupts (Pin Change):
```cpp
const int buttonPin = 2;
volatile int buttonPresses = 0;  // volatile = can change in ISR

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
}

void loop() {
  // Main code runs normally
  Serial.println(buttonPresses);
}

void buttonISR() {
  // Interrupt Service Routine (ISR)
  buttonPresses++;
}
```

**Trigger Modes**:
- **LOW**: Interrupt when pin is LOW
- **CHANGE**: Interrupt on any pin state change
- **RISING**: Interrupt on LOW → HIGH transition
- **FALLING**: Interrupt on HIGH → LOW transition

**ISR Best Practices**:
- Keep ISR short and fast
- Don't use `delay()` inside ISR
- Don't use Serial inside ISR (unreliable)
- Use `volatile` keyword for variables modified in ISR

**Use Cases**: Encoder reading, button presses, emergency stop, time-critical events

---

## 💾 Memory Types

### Flash Memory
- **Stores program code**
- Non-volatile (persists after power-off)
- Arduino Uno: 32KB, ESP32: 4MB
- Read-only during program execution

### SRAM (Static RAM)
- **Stores runtime variables**
- Volatile (lost when power off)
- Arduino Uno: 2KB, ESP32: 520KB
- Fast, but limited

**Optimization**:
```cpp
// Bad (uses SRAM for string)
char message[] = "Hello, World!";

// Good (stores string in Flash, not SRAM)
const char message[] PROGMEM = "Hello, World!";
```

### EEPROM (Electrically Erasable Programmable ROM)
- **Stores persistent data**
- Non-volatile (survives power-off)
- Arduino Uno: 1KB, ESP32: Emulated in Flash
- Slow write, limited write cycles (~100,000)

**Use Cases**: Save calibration data, robot settings, WiFi credentials

```cpp
#include <EEPROM.h>

void setup() {
  EEPROM.write(0, 42);     // Write byte 42 to address 0
  int value = EEPROM.read(0);  // Read byte from address 0
}
```

### Avoiding SRAM Overflow:
- Use `F()` macro for strings: `Serial.println(F("Text"));`
- Store large data in Flash: `const PROGMEM`
- Use smaller data types: `byte` instead of `int` when possible
- Avoid large arrays in SRAM

---

## 🔧 Common Microcontroller Peripherals

### UART (Serial Communication):
```cpp
Serial.begin(9600);         // Initialize at 9600 baud
Serial.println("Hello!");   // Send text
if (Serial.available()) {
  char c = Serial.read();   // Read one byte
}
```

### I2C (Inter-Integrated Circuit):
```cpp
#include <Wire.h>

Wire.begin();               // Master mode
Wire.beginTransmission(0x68);  // Address 0x68 (IMU)
Wire.write(0x3B);           // Register address
Wire.endTransmission();
Wire.requestFrom(0x68, 6);  // Request 6 bytes
```

### SPI (Serial Peripheral Interface):
```cpp
#include <SPI.h>

SPI.begin();
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV16);
digitalWrite(CS_PIN, LOW);   // Select device
SPI.transfer(0x42);          // Send/receive byte
digitalWrite(CS_PIN, HIGH);  // Deselect device
```

---

## 🛠️ Practical Tips

### Power Considerations:
- **Arduino 5V pin**: Max 500mA (from USB) or depends on external power
- **ESP32 3.3V pin**: Max 600mA total across all pins
- **Individual GPIO**: Max 20-40mA per pin
- Use external power supply for motors and high-current devices

### Voltage Level Compatibility:
- **5V Arduino ↔ 3.3V sensor**: Use voltage divider or level shifter
- **3.3V ESP32 ↔ 5V sensor**: 
  - Input pins: Most are 5V-tolerant (check datasheet)
  - Output pins: NOT 5V tolerant (use level shifter)

### Reset Issues:
- ESP32 auto-resets when uploading (DTR/RTS signals)
- Add 10μF capacitor between EN and GND if auto-reset fails

### Common Mistakes:
1. **Forgetting pinMode()**: Pin won't work as expected
2. **Using wrong pins**: Not all pins support PWM/ADC
3. **Not using pull-up/pull-down**: Floating input causes random readings
4. **Blocking code**: Using delay() instead of millis()
5. **SRAM overflow**: Large arrays cause crashes

---

## 📚 Resources

- Arduino Official Documentation (arduino.cc)
- ESP32 Datasheet (Espressif)
- Random Nerd Tutorials (ESP32 projects)
- Nick Gammon's Arduino Forum Posts
- "Programming Arduino" by Simon Monk

---

**Next**: [Communication Protocols](../07-communication-protocols/) - UART, I2C, SPI, and CAN bus

# Embedded Programming

C/C++ programming for microcontrollers, real-time constraints, and embedded systems best practices.

---

## 💻 C/C++ for Microcontrollers

### Why C/C++ for Embedded?
- **Low-level hardware access**: Direct register manipulation
- **Memory efficient**: Minimal overhead, runs on 2KB RAM (Arduino Uno)
- **Deterministic**: Predictable execution time (important for real-time)
- **Portability**: Works across different microcontrollers
- **Performance**: Compiled to native machine code

### Arduino Language = C++ Subset
Arduino uses simplified C++ with built-in libraries:
- `digitalWrite()`, `analogRead()`, `Serial.println()` are C++ functions
- Arduino IDE handles `main()` function and initialization
- Limited C++ features (no exceptions, RTTI, STL on small MCUs)

---

## 🏗️ Program Structure

### Standard C/C++ Program:
```cpp
#include <stdio.h>

int main() {
    printf("Hello, World!\n");
    return 0;
}
```

### Arduino Program Structure:
```cpp
// Arduino IDE automatically generates main() which calls:
void setup() {
  // Runs once when microcontroller powers on or resets
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Runs repeatedly forever
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
}
```

**Behind the scenes**, Arduino IDE generates:
```cpp
int main() {
  init();       // Initialize hardware
  setup();      // Your setup code
  while(1) {    // Infinite loop
    loop();     // Your loop code
  }
  return 0;     // Never reached
}
```

---

## 🔢 Data Types

### Integer Types:

| Type | Size | Range | Use Case |
|------|------|-------|----------|
| `byte` / `uint8_t` | 1 byte | 0 to 255 | Small counters, flags, sensor readings |
| `int` | 2 bytes | -32,768 to 32,767 | General purpose (Arduino) |
| `unsigned int` | 2 bytes | 0 to 65,535 | Always positive values |
| `long` | 4 bytes | -2,147,483,648 to 2,147,483,647 | Large counters, timestamps |
| `unsigned long` | 4 bytes | 0 to 4,294,967,295 | `millis()`, encoder counts |

**Use smallest type possible** to save RAM!

### Floating-Point:
| Type | Size | Precision | Use Case |
|------|------|-----------|----------|
| `float` | 4 bytes | ~7 decimal digits | Calculations, PID |
| `double` | 4 bytes (Arduino) | Same as float | Avoid on Arduino (no benefit) |

**Floating-point math is slow** on most microcontrollers (no FPU)!

### Boolean:
```cpp
bool motorRunning = true;
bool sensorDetected = false;

if (motorRunning) {
  // Do something
}
```

---

## 🔧 Variables and Scope

### Global Variables:
```cpp
int sensorValue = 0;  // Available everywhere, persists between loop() calls

void setup() {
  // Can access sensorValue
}

void loop() {
  sensorValue = analogRead(A0);  // Modify global
}
```

**Use sparingly**: Global variables consume SRAM continuously

### Local Variables:
```cpp
void loop() {
  int localVar = 42;  // Created each time loop() runs, destroyed when exiting
}
```

### Static Variables:
```cpp
void loop() {
  static int counter = 0;  // Initialized once, persists between calls
  counter++;
  Serial.println(counter);  // Prints 1, 2, 3, ...
}
```

**Use for**: Counters that persist, state machines

### Const Variables:
```cpp
const int LED_PIN = 13;  // Cannot be changed, compiler optimizes
const float PI = 3.14159;
```

**Advantage**: Compiler can optimize (no RAM needed if not stored)

---

## 🎯 Control Flow

### If-Else:
```cpp
if (temperature > 30) {
  Serial.println("Hot");
} else if (temperature > 20) {
  Serial.println("Warm");
} else {
  Serial.println("Cold");
}
```

### Switch-Case:
```cpp
switch (robotState) {
  case 0:  // IDLE
    stopMotors();
    break;
  case 1:  // MOVING_FORWARD
    moveForward();
    break;
  case 2:  // TURNING
    turnLeft();
    break;
  default:
    stopMotors();
    break;
}
```

**Better than multiple if-else** for checking one variable against many values.

### Loops:

#### For Loop:
```cpp
for (int i = 0; i < 10; i++) {
  Serial.println(i);
}
```

#### While Loop:
```cpp
while (buttonPressed()) {
  // Do something while button is held
}
```

#### Do-While Loop:
```cpp
do {
  readSensor();
} while (sensorValue < 100);  // Runs at least once
```

---

## 🧩 Functions

### Basic Function:
```cpp
void blinkLED(int pin, int delayTime) {
  digitalWrite(pin, HIGH);
  delay(delayTime);
  digitalWrite(pin, LOW);
  delay(delayTime);
}

void loop() {
  blinkLED(13, 500);  // Call function
}
```

### Return Values:
```cpp
float calculateDistance(int echoTime) {
  float distance = echoTime * 0.034 / 2;  // cm
  return distance;
}

void loop() {
  float dist = calculateDistance(1000);
  Serial.println(dist);
}
```

### Pass by Reference (pointers):
```cpp
void readIMU(float *accelX, float *accelY, float *accelZ) {
  *accelX = readAccelX();  // Modify value at pointer
  *accelY = readAccelY();
  *accelZ = readAccelZ();
}

void loop() {
  float ax, ay, az;
  readIMU(&ax, &ay, &az);  // Pass addresses
  Serial.println(ax);
}
```

**Advantage**: Return multiple values without copying large data

---

## 🕒 Non-Blocking Code

### Problem with delay():
```cpp
void loop() {
  blinkLED();    // Uses delay() internally
  readSensor();  // Blocked by delay()!
}
```

### Solution: millis() State Machine:
```cpp
unsigned long previousMillis = 0;
const long interval = 1000;
bool ledState = LOW;

void loop() {
  unsigned long currentMillis = millis();
  
  // Blink LED without blocking
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(13, ledState);
  }
  
  // Other code runs continuously
  readSensor();
  updateMotors();
  checkButtons();
}
```

**Key Concept**: Check time difference instead of waiting.

---

## 🤖 State Machines

### Finite State Machine (FSM):
Structured way to manage robot behavior

```cpp
enum RobotState {
  IDLE,
  MOVING_FORWARD,
  AVOIDING_OBSTACLE,
  TURNING,
  STOPPED
};

RobotState currentState = IDLE;

void loop() {
  switch (currentState) {
    case IDLE:
      stopMotors();
      if (startButtonPressed()) {
        currentState = MOVING_FORWARD;
      }
      break;
      
    case MOVING_FORWARD:
      moveForward();
      if (obstacleDetected()) {
        currentState = AVOIDING_OBSTACLE;
      }
      break;
      
    case AVOIDING_OBSTACLE:
      stopMotors();
      delay(500);
      currentState = TURNING;
      break;
      
    case TURNING:
      turnRight();
      delay(1000);
      currentState = MOVING_FORWARD;
      break;
      
    case STOPPED:
      stopMotors();
      break;
  }
}
```

**Advantages**:
- Clear logic flow
- Easy to debug
- Easy to add new states
- Scalable for complex robots

---

## 📦 Arrays

### Declaring Arrays:
```cpp
int sensorReadings[10];  // Array of 10 integers

void setup() {
  // Initialize array
  for (int i = 0; i < 10; i++) {
    sensorReadings[i] = 0;
  }
}

void loop() {
  // Store sensor reading
  sensorReadings[0] = analogRead(A0);
  sensorReadings[1] = analogRead(A1);
}
```

### Multi-dimensional Arrays:
```cpp
int lookupTable[3][3] = {
  {1, 2, 3},
  {4, 5, 6},
  {7, 8, 9}
};

int value = lookupTable[1][2];  // Access row 1, column 2 (value = 6)
```

### Strings (Character Arrays):
```cpp
char message[] = "Hello";  // Automatically null-terminated
Serial.println(message);

// Manual character array
char buffer[20];
buffer[0] = 'H';
buffer[1] = 'i';
buffer[2] = '\0';  // Null terminator required!
```

---

## 🔍 Debouncing in Code

### Problem: Button Bounce
Mechanical switches bounce, creating multiple false triggers

### Software Debounce:
```cpp
const int buttonPin = 2;
int buttonState = LOW;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // 50ms

void loop() {
  int reading = digitalRead(buttonPin);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // Reset timer
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Reading has been stable for debounceDelay
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == HIGH) {
        // Button was pressed (rising edge)
        Serial.println("Button pressed!");
      }
    }
  }
  
  lastButtonState = reading;
}
```

---

## ⚡ Interrupts in Detail

### Hardware Interrupt:
```cpp
const int encoderPinA = 2;  // Interrupt-capable pin
volatile long encoderCount = 0;

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
}

void encoderISR() {
  // Interrupt Service Routine
  encoderCount++;  // Very short, no delays!
}

void loop() {
  Serial.println(encoderCount);  // Use interrupt data
  delay(100);
}
```

### ISR Rules:
1. **Keep it SHORT**: Exit quickly (microseconds, not milliseconds)
2. **No delay()**: Blocks interrupt system
3. **No Serial.print()**: Can cause crashes
4. **Use volatile**: Variables modified in ISR must be `volatile`
5. **Disable interrupts if needed**: `noInterrupts()` / `interrupts()`

### Critical Section (Atomic Operation):
```cpp
long getEncoderCount() {
  long count;
  noInterrupts();  // Disable interrupts temporarily
  count = encoderCount;  // Read volatile variable safely
  interrupts();    // Re-enable interrupts
  return count;
}
```

---

## 💾 Memory Optimization

### Flash vs SRAM:

**Store constants in Flash, not SRAM**:
```cpp
// BAD: Stores string in SRAM (wastes limited RAM)
char message[] = "This is a long error message that wastes RAM";

// GOOD: Stores string in Flash
const char message[] PROGMEM = "This is a long error message in Flash";
Serial.println(F("Direct string in Flash"));
```

### F() Macro:
```cpp
Serial.println(F("This string stored in Flash, not RAM"));
```

**Use F() for all Serial.print strings** to save RAM!

### Checking Free RAM:
```cpp
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void loop() {
  Serial.print(F("Free RAM: "));
  Serial.println(freeRam());
}
```

---

## 🛠️ Best Practices

### 1. Use #define for Constants:
```cpp
#define LED_PIN 13
#define MOTOR_SPEED 150
```

Better than variables (no RAM used, compiler replaces before compile).

### 2. Avoid String Class on Arduino:
```cpp
// BAD: Dynamic memory allocation, can fragment SRAM
String message = "Hello " + "World";

// GOOD: Use char arrays
char message[20];
strcpy(message, "Hello ");
strcat(message, "World");
```

### 3. Use PROGMEM for Large Data:
```cpp
const int sineTable[256] PROGMEM = { /* 256 values */ };

int value = pgm_read_word(&sineTable[index]);  // Read from Flash
```

### 4. Avoid Floating-Point When Possible:
```cpp
// BAD: Slow floating-point
float speed = 12.5;  
float distance = speed * 2.0;

// GOOD: Use integer math (fixed-point)
int speed = 125;  // Represents 12.5 (scaled by 10)
int distance = speed * 2;  // Result = 250 (represents 25.0)
```

### 5. Watchdog Timer:
Reset microcontroller if program hangs

```cpp
#include <avr/wdt.h>

void setup() {
  wdt_enable(WDTO_2S);  // 2-second watchdog timeout
}

void loop() {
  // Do work
  wdt_reset();  // Reset watchdog (must be called every 2 seconds)
}
```

---

## 📚 Resources

- "Programming Arduino" by Simon Monk
- AVR Libc Reference Manual
- "Embedded C Programming" by Michael Barr
- Nick Gammon's Arduino Forum Posts
- "The C Programming Language" by K&R

---

**Next**: [Tools and Debugging](../09-tools-and-debugging/) - Hardware tools and debugging techniques

# Motor Control

PWM, PID control, and motion control algorithms for robotics.

---

## ⚡ PWM (Pulse Width Modulation)

### What is PWM?
A technique to control average power delivered to a device by rapidly switching it ON and OFF.

### Key Parameters:

#### Duty Cycle
Percentage of time signal is HIGH during one period
- **0% duty cycle**: Always OFF
- **50% duty cycle**: ON half the time
- **100% duty cycle**: Always ON

**Formula**: `Duty Cycle (%) = (T_on / T_period) × 100`

#### Frequency
How many ON/OFF cycles per second (Hz)
- **Motor control**: Typically 500Hz - 20kHz
- **Servo control**: 50Hz (20ms period)
- **LED dimming**: 100Hz - 10kHz

**Formula**: `Frequency (Hz) = 1 / T_period`

### Why PWM for Motor Control?

**Problem**: Reducing voltage to motor using resistor wastes power as heat  
**Solution**: PWM delivers full voltage in pulses → motor averages it out

**Example**: 12V motor at 50% duty cycle  
- Motor receives 12V for 50% of time, 0V for 50%
- Average voltage: ~6V
- Motor runs at ~half speed
- Minimal power wasted as heat

### Arduino PWM:
```cpp
int motorPin = 9;  // PWM-capable pin (3, 5, 6, 9, 10, 11 on Uno)

void setup() {
  pinMode(motorPin, OUTPUT);
}

void loop() {
  analogWrite(motorPin, 0);     // 0% duty cycle (OFF)
  delay(1000);
  
  analogWrite(motorPin, 64);    // 25% duty cycle (0-255 scale)
  delay(1000);
  
  analogWrite(motorPin, 128);   // 50% duty cycle
  delay(1000);
  
  analogWrite(motorPin, 191);   // 75% duty cycle
  delay(1000);
  
  analogWrite(motorPin, 255);   // 100% duty cycle (full speed)
  delay(1000);
}
```

### PWM Frequency Considerations:

**Too Low (<100Hz)**:
- Motor "stutters" (audible pulsing)
- Inefficient, rough motion

**Optimal (500Hz - 20kHz)**:
- Smooth motor operation
- Inaudible to humans (>20kHz)
- Good balance between switching losses and smoothness

**Too High (>50kHz)**:
- Increased switching losses in MOSFETs
- EMI (electromagnetic interference)
- Less efficient

### Changing PWM Frequency on Arduino:
```cpp
// Timer 0 (pins 5, 6): Default 976Hz
// TCCR0B = (TCCR0B & 0xF8) | 0x03;  // 976Hz
// TCCR0B = (TCCR0B & 0xF8) | 0x02;  // 3906Hz

// Timer 1 (pins 9, 10): Default 490Hz
TCCR1B = (TCCR1B & 0xF8) | 0x01;  // 31250Hz
// TCCR1B = (TCCR1B & 0xF8) | 0x02;  // 3906Hz

// Timer 2 (pins 3, 11): Default 490Hz
// TCCR2B = (TCCR2B & 0xF8) | 0x01;  // 31250Hz
```

**Warning**: Changing Timer 0 affects `millis()` and `delay()`

---

## 🎯 PID Control Fundamentals

### What is PID?
Closed-loop control algorithm that continuously calculates error and applies correction to reach target.

**Goal**: Keep robot/motor at desired setpoint (position, speed, temperature, etc.)

### The Three Terms:

#### **P - Proportional**
Response proportional to current error

`P_term = Kp × error`

- **Kp (Proportional Gain)**: How aggressively to respond to error
- **Effect**: 
  - High Kp → Fast response, but overshoot and oscillation
  - Low Kp → Slow response, may never reach setpoint

#### **I - Integral**
Accumulates past errors over time

`I_term = Ki × ∫error dt`

- **Ki (Integral Gain)**: How much to consider accumulated error
- **Purpose**: Eliminate steady-state error (offset from setpoint)
- **Effect**:
  - Removes persistent offset
  - Can cause windup (accumulates too much → large overshoot)

#### **D - Derivative**
Predicts future error based on rate of change

`D_term = Kd × (d(error)/dt)`

- **Kd (Derivative Gain)**: How much to dampen based on error rate
- **Purpose**: Reduce overshoot and oscillation
- **Effect**:
  - Dampens system response
  - Reduces oscillation
  - Sensitive to noise (can amplify sensor noise)

### PID Formula:

`output = Kp × error + Ki × ∫error dt + Kd × (d(error)/dt)`

---

## 🛠️ Implementing PID in Arduino

### Basic PID Code:
```cpp
// PID Variables
float setpoint = 100.0;       // Desired value (e.g., RPM, position)
float input = 0.0;            // Current measured value
float output = 0.0;           // Control output (PWM value)

float Kp = 2.0;               // Proportional gain
float Ki = 0.5;               // Integral gain
float Kd = 0.1;               // Derivative gain

float lastError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // seconds
  
  if (deltaTime >= 0.01) {  // Run PID every 10ms
    // Read sensor (e.g., encoder for motor speed)
    input = readSensor();
    
    // Calculate error
    float error = setpoint - input;
    
    // Proportional term
    float P = Kp * error;
    
    // Integral term (accumulate error over time)
    integral += error * deltaTime;
    float I = Ki * integral;
    
    // Derivative term (rate of change of error)
    float derivative = (error - lastError) / deltaTime;
    float D = Kd * derivative;
    
    // PID output
    output = P + I + D;
    
    // Constrain output to valid range (0-255 for PWM)
    output = constrain(output, 0, 255);
    
    // Apply output to motor
    analogWrite(motorPin, output);
    
    // Save values for next iteration
    lastError = error;
    lastTime = currentTime;
  }
}
```

### Integral Windup Prevention:
Integral term can accumulate excessively during sustained error (e.g., motor stalled)

**Solution**: Clamp integral term
```cpp
integral += error * deltaTime;

// Limit integral term
float integralMax = 50.0;
integral = constrain(integral, -integralMax, integralMax);

float I = Ki * integral;
```

### Derivative Kick Prevention:
When setpoint changes suddenly, derivative term spikes (based on setpoint change, not measurement change)

**Solution**: Calculate derivative on measurement, not error
```cpp
// Instead of: derivative = (error - lastError) / deltaTime;
float derivative = -(input - lastInput) / deltaTime;  // Note the negative sign
float D = Kd * derivative;

lastInput = input;
```

---

## ⚙️ PID Tuning

### Tuning Methods:

#### **Method 1: Manual Tuning**
Start with all gains at 0, increase one at a time.

**Step 1: Tune Kp**
- Set Ki = 0, Kd = 0
- Increase Kp until system responds to setpoint change
- Continue increasing until oscillation starts
- Reduce Kp to 50-70% of oscillation point

**Step 2: Tune Ki**
- Increase Ki to eliminate steady-state error
- Too much Ki → slow oscillations and overshoot
- Reduce if system becomes unstable

**Step 3: Tune Kd**
- Increase Kd to reduce overshoot
- Kd should be ~1/10 to 1/5 of Kp (as starting point)
- Too much Kd → system becomes sluggish or noisy

#### **Method 2: Ziegler-Nichols Method**
1. Set Ki = 0, Kd = 0
2. Increase Kp until sustained oscillation occurs (Ku = ultimate gain)
3. Measure oscillation period (Pu)
4. Calculate:
   - Kp = 0.6 × Ku
   - Ki = 2 × Kp / Pu
   - Kd = Kp × Pu / 8

### Observing PID Behavior:

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Slow to reach setpoint | Kp too low | Increase Kp |
| Overshoots and oscillates | Kp too high | Decrease Kp |
| Settles below/above setpoint | Ki too low | Increase Ki |
| Oscillates slowly around setpoint | Ki too high | Decrease Ki |
| Overshoots significantly | Kd too low | Increase Kd |
| Noisy, jittery motion | Kd too high | Decrease Kd |

### Plotting PID Response:
Use Arduino Serial Plotter to visualize:
```cpp
Serial.print("Setpoint:");
Serial.print(setpoint);
Serial.print(",");
Serial.print("Input:");
Serial.print(input);
Serial.print(",");
Serial.print("Output:");
Serial.println(output);
```

---

## 🎮 Open-Loop vs Closed-Loop Control

### Open-Loop Control
**No feedback** - Apply control signal without measuring result

**Example**: Send 50% PWM to motor, assume it runs at 50% speed
- Simple, no sensor needed
- Inaccurate (load changes, battery voltage drops → speed changes)
- Cannot compensate for disturbances

**Use Cases**: 
- Simple robots where precision doesn't matter
- RC cars with joystick control
- Low-cost applications

### Closed-Loop Control
**Feedback sensor** - Measure output, adjust control to reach target

**Example**: Use encoder to measure speed, PID adjusts PWM to maintain exact RPM
- Accurate and robust
- Compensates for disturbances (load changes, voltage drops)
- Requires sensor and control algorithm

**Use Cases**:
- Precise positioning (robot arms)
- Constant speed (mobile robots)
- Balancing robots
- Any application requiring accuracy

---

## 🚗 Velocity vs Position Control

### Velocity Control
Maintain constant speed regardless of load

**Sensor**: Encoder (measure RPM)  
**Control**: PID output → PWM to motor driver

```cpp
float targetRPM = 100.0;
float currentRPM = readEncoder();  // Calculate from encoder counts
float error = targetRPM - currentRPM;
// ... PID calculation
analogWrite(motorPin, output);
```

**Applications**:
- Mobile robots (constant wheel speed)
- Conveyor belts
- Fans, pumps

### Position Control
Move to specific position and hold

**Sensor**: Encoder (measure angle or distance)  
**Control**: PID output → PWM to motor driver

```cpp
float targetPosition = 1000;  // Encoder counts
float currentPosition = encoderCount;
float error = targetPosition - currentPosition;
// ... PID calculation
analogWrite(motorPin, output);
```

**Applications**:
- Robot arm joints
- Gantry systems (3D printer, CNC)
- Antenna pointing

---

## 🔄 Motion Profiles

### Problem:
Instant acceleration/deceleration causes:
- Mechanical stress
- Wheel slip
- Jerky motion

### Solution: Trapezoidal Motion Profile
1. **Acceleration phase**: Ramp up speed
2. **Constant velocity phase**: Maintain max speed
3. **Deceleration phase**: Ramp down to stop

```cpp
float maxSpeed = 200.0;    // RPM
float acceleration = 50.0; // RPM/s
float currentSpeed = 0.0;

void loop() {
  float targetSpeed = calculateTargetSpeed();  // From motion planner
  
  if (currentSpeed < targetSpeed) {
    // Accelerate
    currentSpeed += acceleration * deltaTime;
    currentSpeed = min(currentSpeed, targetSpeed);
  } else if (currentSpeed > targetSpeed) {
    // Decelerate
    currentSpeed -= acceleration * deltaTime;
    currentSpeed = max(currentSpeed, targetSpeed);
  }
  
  // Use currentSpeed as setpoint for velocity PID
  velocityPID.setSetpoint(currentSpeed);
}
```

**Applications**: Smooth robot motion, 3D printer movement, CNC machining

---

## 🎛️ Cascaded Control

### Position Control with Velocity Inner Loop

**Why?** Position PID output → velocity setpoint → velocity PID → motor PWM

**Advantages**:
- Smoother motion
- Better disturbance rejection
- Easier tuning (tune velocity loop first, then position loop)

```cpp
// Outer loop: Position PID
float positionError = targetPosition - currentPosition;
float velocitySetpoint = positionPID.compute(positionError);

// Inner loop: Velocity PID
float velocityError = velocitySetpoint - currentVelocity;
float motorPWM = velocityPID.compute(velocityError);

analogWrite(motorPin, motorPWM);
```

**Applications**: 
- High-performance motor control
- Robot manipulators (joint position control)
- Mobile robots (position and heading control)

---

## 🛠️ Practical Tips

### Motor Control Best Practices:
1. **Measure first**: Plot setpoint vs actual response before tuning
2. **Start conservative**: Low gains, increase gradually
3. **Tune one parameter at a time**
4. **Use appropriate sample rate**: 10-100Hz typical for mechanical systems
5. **Filter noisy sensors**: Moving average or low-pass filter before PID
6. **Anti-windup**: Limit integral term accumulation
7. **Dead zone**: Ignore very small errors (< 1-2% of range) to prevent jitter

### Common Mistakes:
- **No derivative filtering** → Amplifies sensor noise
- **Tuning with different loads** → Gains optimal for one load may not work for another
- **Ignoring saturation** → Motor can't go faster than 100% PWM
- **Too fast sample rate** → Doesn't allow motor to respond between samples
- **Too slow sample rate** → Control loop can't react fast enough

### Debugging PID:
1. **Log data**: Setpoint, input, output, error, P/I/D terms
2. **Plot response**: Use Serial Plotter or logging software
3. **Check sensor**: Is sensor working correctly? Is noise present?
4. **Test open-loop first**: Does motor respond correctly to PWM?
5. **Start simple**: Test P-only controller before adding I and D

---

## 📚 Resources

- "PID Control Fundamentals" by NI (National Instruments)
- Brett Beauregard's Arduino PID Library
- "Feedback Control of Dynamic Systems" by Franklin, Powell, Emami-Naeini
- Brian Douglas' Control Systems Lectures (YouTube)
- MATLAB/Simulink PID Tuner

---

**Next**: [Microcontrollers](../06-microcontrollers/) - Arduino, ESP32, and embedded programming basics

# ROS 2 Hardware Integration

Interfacing microcontrollers with ROS 2, hardware abstraction layers, and sensor data publishing.

---

## 🤖 Why Integrate Microcontrollers with ROS 2?

**ROS 2 runs on Linux (Ubuntu)** → Cannot directly access hardware like Arduino can

**Solution**: Microcontroller acts as **hardware abstraction layer**
- **Arduino/ESP32**: Low-level hardware control (sensors, motors, encoders)
- **ROS 2 (Linux PC)**: High-level logic (navigation, planning, vision)
- **Communication**: Serial (UART), Ethernet, or WiFi

---

## 🔗 Communication Architectures

### 1. Serial Communication (Most Common)

**Arduino ↔ ROS 2 via USB Serial**

```
┌─────────────┐       USB Cable        ┌──────────────┐
│  Arduino    │◄─────────────────────►│  ROS 2 PC    │
│  (Sensors,  │    /dev/ttyUSB0        │  (Planning,  │
│   Motors)   │                        │   Vision)    │
└─────────────┘                        └──────────────┘
```

**Advantages**:
- Simple hardware (just USB cable)
- Reliable
- Built into most microcontrollers

**Disadvantages**:
- Limited speed (~115200 baud typical)
- Wired connection only
- One microcontroller per USB port

---

### 2. WiFi Communication (ESP32)

**ESP32 ↔ ROS 2 via WiFi (micro-ROS or custom protocol)**

```
┌─────────────┐                       ┌──────────────┐
│   ESP32     │◄─────────WiFi────────►│  ROS 2 PC    │
│  (Sensors,  │   (MQTT, UDP, TCP)    │  (Planning)  │
│   Motors)   │                        │              │
└─────────────┘                        └──────────────┘
```

**Advantages**:
- Wireless (mobile robots)
- ESP32 has built-in WiFi
- Can use multiple microcontrollers

**Disadvantages**:
- More complex setup
- WiFi latency/dropouts
- Power consumption

---

### 3. Ethernet (Industrial Robots)

**Microcontroller with Ethernet ↔ ROS 2 via LAN**

**Advantages**:
- Fast, reliable
- Low latency
- Suitable for high-bandwidth sensors (cameras, LIDAR)

**Disadvantages**:
- Requires Ethernet hardware (W5500 module, etc.)
- Wired connection

---

## 📡 Serial Protocol Design

### Simple Custom Protocol

**Arduino sends sensor data**:
```
<SENSOR_TYPE>:<VALUE>\n
```

**Example**:
```
ULTRASONIC:25.3\n
IMU_ACCEL_X:0.15\n
ENCODER_LEFT:1524\n
```

**Arduino Code**:
```cpp
void loop() {
  float distance = readUltrasonic();
  Serial.print("ULTRASONIC:");
  Serial.println(distance);
  
  long encoderCount = readEncoder();
  Serial.print("ENCODER_LEFT:");
  Serial.println(encoderCount);
  
  delay(50);  // 20Hz update rate
}
```

**ROS 2 Python Node**:
```python
import rclpy
from rclpy.node import Node
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.05, self.read_serial)
    
    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            self.parse_data(line)
    
    def parse_data(self, line):
        try:
            sensor_type, value = line.split(':')
            if sensor_type == 'ULTRASONIC':
                distance = float(value)
                self.get_logger().info(f'Ultrasonic: {distance} cm')
            elif sensor_type == 'ENCODER_LEFT':
                count = int(value)
                self.get_logger().info(f'Encoder: {count}')
        except ValueError:
            self.get_logger().warn(f'Invalid data: {line}')

def main():
    rclpy.init()
    node = ArduinoSerialNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## 🔄 Bidirectional Communication

### Arduino receives commands from ROS 2

**Protocol**:
- **ROS 2 → Arduino**: `CMD:MOTOR_LEFT:150\n` (set left motor speed to 150)
- **Arduino → ROS 2**: `ACK:MOTOR_LEFT:150\n` (acknowledge command)

**Arduino Code**:
```cpp
void loop() {
  // Send sensor data to ROS 2
  sendSensorData();
  
  // Receive commands from ROS 2
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
  }
}

void parseCommand(String command) {
  // Parse: "CMD:MOTOR_LEFT:150"
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);
  
  String cmdType = command.substring(0, firstColon);
  String device = command.substring(firstColon + 1, secondColon);
  String value = command.substring(secondColon + 1);
  
  if (cmdType == "CMD") {
    if (device == "MOTOR_LEFT") {
      int speed = value.toInt();
      setMotorSpeed(LEFT_MOTOR, speed);
      
      // Send acknowledgment
      Serial.print("ACK:MOTOR_LEFT:");
      Serial.println(speed);
    }
  }
}
```

**ROS 2 Node**:
```python
def send_motor_command(self, motor, speed):
    command = f"CMD:{motor}:{speed}\n"
    self.serial_port.write(command.encode('utf-8'))
    self.get_logger().info(f'Sent command: {command.strip()}')

# Usage
self.send_motor_command('MOTOR_LEFT', 150)
```

---

## 📦 Message Framing

### Problem: Incomplete Messages

Serial data can arrive in chunks:
```
"ULTRASON"  ← Partial message
"IC:25.3\nIMU_" ← Incomplete
```

### Solution: Delimiters and Buffering

**Use newline `\n` as delimiter**:
```cpp
// Arduino
Serial.println("ULTRASONIC:25.3");  // Adds \n automatically
```

```python
# ROS 2
line = self.serial_port.readline().decode('utf-8').strip()
```

**Alternative: Start/Stop Markers**:
```cpp
// Arduino
Serial.print("<ULTRASONIC:25.3>");
```

```python
# ROS 2
if '<' in buffer and '>' in buffer:
    start = buffer.index('<')
    end = buffer.index('>')
    message = buffer[start+1:end]
    # Process message
```

---

## 🎯 Publishing ROS 2 Topics from Serial Data

### Publishing Sensor Data

**Arduino sends**: `ULTRASONIC:25.3\n`

**ROS 2 publishes**: `sensor_msgs/Range` on `/ultrasonic` topic

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(Range, '/ultrasonic', 10)
        
        # Open serial port
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Timer to read serial data
        self.timer = self.create_timer(0.05, self.read_and_publish)
    
    def read_and_publish(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            
            if line.startswith('ULTRASONIC:'):
                distance = float(line.split(':')[1]) / 100.0  # cm to meters
                
                # Create Range message
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'ultrasonic_link'
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 0.26  # 15 degrees
                msg.min_range = 0.02
                msg.max_range = 4.0
                msg.range = distance
                
                # Publish
                self.publisher.publish(msg)
                self.get_logger().info(f'Published range: {distance:.2f}m')
```

---

### Publishing IMU Data

**Arduino sends**: `IMU:ax,ay,az,gx,gy,gz\n`

**ROS 2 publishes**: `sensor_msgs/Imu` on `/imu/data` topic

```python
from sensor_msgs.msg import Imu

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.01, self.read_and_publish)  # 100Hz
    
    def read_and_publish(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            
            if line.startswith('IMU:'):
                data = line.split(':')[1].split(',')
                ax, ay, az = map(float, data[0:3])  # Acceleration (m/s²)
                gx, gy, gz = map(float, data[3:6])  # Angular velocity (rad/s)
                
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = az
                
                msg.angular_velocity.x = gx
                msg.angular_velocity.y = gy
                msg.angular_velocity.z = gz
                
                self.publisher.publish(msg)
```

---

### Publishing Encoder Data (Odometry)

**Arduino sends**: `ENCODER:left,right\n`

**ROS 2 publishes**: `nav_msgs/Odometry` on `/odom` topic

```python
from nav_msgs.msg import Odometry
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Robot parameters
        self.wheel_radius = 0.033  # meters
        self.wheel_base = 0.16     # meters
        self.counts_per_rev = 2400  # Encoder counts per wheel revolution
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_count = 0
        self.last_right_count = 0
        self.last_time = self.get_clock().now()
    
    def read_and_publish(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            
            if line.startswith('ENCODER:'):
                data = line.split(':')[1].split(',')
                left_count = int(data[0])
                right_count = int(data[1])
                
                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds / 1e9
                
                # Calculate wheel distances
                d_left = (left_count - self.last_left_count) * (2 * math.pi * self.wheel_radius) / self.counts_per_rev
                d_right = (right_count - self.last_right_count) * (2 * math.pi * self.wheel_radius) / self.counts_per_rev
                
                # Update odometry
                d_center = (d_left + d_right) / 2.0
                d_theta = (d_right - d_left) / self.wheel_base
                
                self.x += d_center * math.cos(self.theta)
                self.y += d_center * math.sin(self.theta)
                self.theta += d_theta
                
                # Create and publish Odometry message
                msg = Odometry()
                msg.header.stamp = current_time.to_msg()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_link'
                
                msg.pose.pose.position.x = self.x
                msg.pose.pose.position.y = self.y
                msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
                msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
                
                self.publisher.publish(msg)
                
                # Update state
                self.last_left_count = left_count
                self.last_right_count = right_count
                self.last_time = current_time
```

---

## 🎮 Subscribing to ROS 2 Topics (Motor Control)

**ROS 2 sends motor commands → Arduino**

**ROS 2 Node subscribes to `/cmd_vel`**:
```python
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    
    def cmd_vel_callback(self, msg):
        # Convert Twist to differential drive motor speeds
        linear = msg.linear.x   # m/s
        angular = msg.angular.z # rad/s
        
        wheel_base = 0.16  # meters
        wheel_radius = 0.033  # meters
        
        # Calculate wheel velocities
        v_left = linear - (angular * wheel_base / 2.0)
        v_right = linear + (angular * wheel_base / 2.0)
        
        # Convert to motor PWM (-255 to 255)
        max_speed = 0.5  # m/s
        pwm_left = int((v_left / max_speed) * 255)
        pwm_right = int((v_right / max_speed) * 255)
        
        # Clamp to valid range
        pwm_left = max(-255, min(255, pwm_left))
        pwm_right = max(-255, min(255, pwm_right))
        
        # Send to Arduino
        command = f"MOTOR:{pwm_left},{pwm_right}\n"
        self.serial_port.write(command.encode('utf-8'))
        
        self.get_logger().info(f'Motor command: L={pwm_left}, R={pwm_right}')
```

**Arduino receives and applies**:
```cpp
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if (command.startsWith("MOTOR:")) {
      int commaIndex = command.indexOf(',');
      int pwmLeft = command.substring(6, commaIndex).toInt();
      int pwmRight = command.substring(commaIndex + 1).toInt();
      
      setMotorSpeed(LEFT_MOTOR, pwmLeft);
      setMotorSpeed(RIGHT_MOTOR, pwmRight);
    }
  }
}

void setMotorSpeed(int motor, int pwm) {
  // Handle direction and PWM
  if (pwm >= 0) {
    digitalWrite(motor_dir_pin, HIGH);
    analogWrite(motor_pwm_pin, pwm);
  } else {
    digitalWrite(motor_dir_pin, LOW);
    analogWrite(motor_pwm_pin, -pwm);
  }
}
```

---

## 🔧 Hardware Abstraction Layer Pattern

**Separate low-level hardware from high-level logic**

### Arduino (Hardware Layer):
```cpp
// Handles:
// - Reading sensors (encoders, IMU, ultrasonic)
// - Controlling motors (PWM, direction)
// - Emergency stop (button pressed → stop motors immediately)
// - Communicates via serial
```

### ROS 2 (Logic Layer):
```python
# Handles:
# - Path planning (Nav2)
# - Mapping (SLAM)
# - Decision making
# - User interface
# - Sends high-level commands to Arduino
```

**Benefits**:
- Real-time safety (Arduino handles emergency stop instantly)
- Modularity (swap Arduino for different hardware)
- Separation of concerns (easier to debug)

---

## 📚 Resources

- ROS 2 Serial Communication Tutorial
- rosserial (ROS 1) - reference for protocol design
- micro-ROS (ROS 2 on microcontrollers)
- "Programming Robots with ROS" by Quigley et al.

---

**Repository Complete!** All fundamental topics for robotics electronics and embedded systems covered.

# Arduino + ROS 2 Quick Reference

Quick copy-paste code templates for Arduino-ROS 2 integration.

---

## Arduino → ROS 2 (Sensor Data)

**Arduino:**
```cpp
void setup() {
  Serial.begin(115200);
}

void loop() {
  float value = readSensor();
  Serial.print("DATA:");
  Serial.println(value);
  delay(100);
}
```

**ROS 2 Python:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.pub = self.create_publisher(Float32, '/sensor', 10)
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)
    
    def read_serial(self):
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode('utf-8').strip()
            if line.startswith('DATA:'):
                value = float(line.split(':')[1])
                msg = Float32()
                msg.data = value
                self.pub.publish(msg)

def main():
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## ROS 2 → Arduino (Motor Control)

**Arduino:**
```cpp
void setup() {
  Serial.begin(115200);
  pinMode(motorPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("MOTOR:")) {
      int speed = cmd.substring(6).toInt();
      analogWrite(motorPin, speed);
    }
  }
}
```

**ROS 2 Python:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.sub = self.create_subscription(Int32, '/motor_speed', self.callback, 10)
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    
    def callback(self, msg):
        cmd = f"MOTOR:{msg.data}\n"
        self.serial.write(cmd.encode('utf-8'))

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## Bidirectional (Robot Car)

**Arduino:**
```cpp
const int trigPin = 9, echoPin = 10;
const int motorL1 = 8, motorL2 = 7, motorR1 = 6, motorR2 = 5;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Send sensor data
  float dist = readUltrasonic();
  Serial.print("DIST:");
  Serial.println(dist);
  
  // Receive motor commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("VEL:")) {
      int comma = cmd.indexOf(',');
      int left = cmd.substring(4, comma).toInt();
      int right = cmd.substring(comma + 1).toInt();
      setMotors(left, right);
    }
  }
  delay(50);
}

float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void setMotors(int left, int right) {
  digitalWrite(motorL1, left > 0 ? HIGH : LOW);
  digitalWrite(motorL2, left > 0 ? LOW : HIGH);
  digitalWrite(motorR1, right > 0 ? HIGH : LOW);
  digitalWrite(motorR2, right > 0 ? LOW : HIGH);
}
```

**ROS 2 Python:**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import serial

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.pub = self.create_publisher(Range, '/ultrasonic', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.05, self.read_serial)
    
    def read_serial(self):
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode('utf-8').strip()
            if line.startswith('DIST:'):
                dist = float(line.split(':')[1]) / 100.0
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.range = dist
                self.pub.publish(msg)
    
    def cmd_callback(self, msg):
        left = int((msg.linear.x - msg.angular.z) * 255)
        right = int((msg.linear.x + msg.angular.z) * 255)
        cmd = f"VEL:{left},{right}\n"
        self.serial.write(cmd.encode('utf-8'))

def main():
    rclpy.init()
    rclpy.spin(RobotNode())

if __name__ == '__main__':
    main()
```

---

## Quick Commands

```bash
# Find Arduino port
ls /dev/ttyUSB* /dev/ttyACM*

# Give permission
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER  # Permanent

# Run ROS 2 node
python3 robot_node.py

# Test topics
ros2 topic list
ros2 topic echo /ultrasonic
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"
```

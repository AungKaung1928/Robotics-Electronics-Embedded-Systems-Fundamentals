# Communication Protocols

Serial communication (UART, I2C, SPI, CAN bus) for connecting devices in robotics systems.

---

## 📡 Why Communication Protocols?

Robots need multiple devices to communicate:
- Microcontroller ↔ Sensors (IMU, GPS, LIDAR)
- Microcontroller ↔ Motor drivers
- Microcontroller ↔ Computer (ROS 2 node)
- Multiple microcontrollers in one system

**Key Questions**:
- How many wires?
- How fast can data transfer?
- How many devices can connect?
- Master-slave or peer-to-peer?

---

## 🔌 UART (Universal Asynchronous Receiver-Transmitter)

### Overview:
- **Wires**: 2 (TX, RX) + GND
- **Speed**: 9600 - 921600 baud typical (bits per second)
- **Devices**: Point-to-point (one sender, one receiver)
- **Synchronization**: Asynchronous (no clock line)

### How UART Works:

**Connections**:
```
Device A              Device B
   TX  ------------->  RX
   RX  <-------------  TX
  GND  ------------- GND
```

**Data Format**:
```
[START] [DATA BITS: 8] [PARITY: optional] [STOP: 1-2 bits]
```

- **Start bit**: Signals beginning of transmission (LOW)
- **Data bits**: Actual data (usually 8 bits)
- **Parity bit**: Error checking (optional)
- **Stop bit**: Signals end of transmission (HIGH)

### Arduino UART:
```cpp
void setup() {
  Serial.begin(9600);  // 9600 baud rate
}

void loop() {
  // Send data
  Serial.println("Hello");
  Serial.write(0x42);  // Send single byte
  
  // Receive data
  if (Serial.available() > 0) {
    char c = Serial.read();  // Read one byte
    Serial.print("Received: ");
    Serial.println(c);
  }
}
```

### Multiple UART Ports:
- **Arduino Uno**: 1 hardware UART (pins 0, 1)
- **Arduino Mega**: 4 hardware UARTs (Serial, Serial1, Serial2, Serial3)
- **ESP32**: 3 hardware UARTs (Serial, Serial1, Serial2)

```cpp
// Arduino Mega example
Serial.begin(9600);    // USB Serial Monitor
Serial1.begin(9600);   // TX1 (pin 18), RX1 (pin 19) - GPS
Serial2.begin(9600);   // TX2 (pin 16), RX2 (pin 17) - XBee radio
```

### Software Serial (Arduino):
Emulate UART on any GPIO pins (slower, less reliable)
```cpp
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11);  // RX, TX

void setup() {
  mySerial.begin(9600);
}

void loop() {
  mySerial.println("Hello from software serial");
}
```

### Common Baud Rates:
- **9600**: Default, reliable, slow
- **19200, 38400, 57600**: Medium speed
- **115200**: Fast, common for ESP32
- **230400, 460800, 921600**: Very fast, less reliable over long wires

### Applications:
- **GPS modules**: NMEA data over UART
- **Bluetooth/WiFi modules**: AT commands via UART
- **ROS 2 serial communication**: Arduino ↔ PC via rosserial
- **Debugging**: Serial.print() to Serial Monitor
- **XBee wireless**: Radio modules use UART

### Troubleshooting:
- **No data received**: Check baud rate match, TX/RX swap, GND connection
- **Garbage characters**: Wrong baud rate, electrical noise
- **Data loss**: Buffer overflow (read faster or increase buffer size)

---

## 🔗 I2C (Inter-Integrated Circuit)

### Overview:
- **Wires**: 2 (SDA, SCL) + GND + VCC
- **Speed**: 100kHz (standard), 400kHz (fast), 1MHz (fast-mode plus)
- **Devices**: Multiple (up to 127 devices on one bus)
- **Topology**: Multi-master, multi-slave (typically single master)
- **Synchronization**: Synchronous (clock line SCL)

### I2C Bus Structure:
```
        +5V or +3.3V
         |    |
       [4.7kΩ][4.7kΩ]  <- Pull-up resistors (required!)
         |    |
    SDA  o----o----o----o
    SCL  o----o----o----o
       Master Slave1 Slave2 Slave3
```

**Pull-up resistors** (typically 4.7kΩ) required on SDA and SCL lines!

### How I2C Works:

1. **Addressing**: Each device has unique 7-bit or 10-bit address
2. **Master initiates**: Sends START condition + device address
3. **Slave responds**: Acknowledges (ACK) if address matches
4. **Data transfer**: Master sends/receives data bytes
5. **STOP**: Master sends STOP condition to end communication

### Arduino I2C:
```cpp
#include <Wire.h>

void setup() {
  Wire.begin();  // Join I2C bus as master
  Serial.begin(9600);
}

void loop() {
  // Write to I2C device (e.g., address 0x68)
  Wire.beginTransmission(0x68);  // Start communication
  Wire.write(0x3B);              // Send register address
  Wire.endTransmission();        // End transmission
  
  // Read from I2C device
  Wire.requestFrom(0x68, 6);  // Request 6 bytes from address 0x68
  while (Wire.available()) {
    byte data = Wire.read();
    Serial.println(data);
  }
  
  delay(100);
}
```

### I2C Pins:
- **Arduino Uno**: SDA = A4, SCL = A5
- **Arduino Mega**: SDA = 20, SCL = 21
- **ESP32**: Configurable, default SDA = GPIO 21, SCL = GPIO 22

### I2C Addressing:
- **7-bit addresses**: 0x00 - 0x7F (0-127 in decimal)
- Common addresses:
  - MPU6050 (IMU): 0x68 or 0x69
  - BMP280 (pressure): 0x76 or 0x77
  - OLED display: 0x3C or 0x3D
  - PCA9685 (servo driver): 0x40 - 0x7F (configurable)

### Scanning I2C Bus:
```cpp
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) Serial.println("No I2C devices found");
  delay(5000);
}
```

### Applications:
- **IMU**: MPU6050, MPU9250, BNO055
- **Displays**: OLED (SSD1306), LCD with I2C backpack
- **Sensors**: BMP280 (pressure), BME280 (temp/humidity/pressure)
- **PWM drivers**: PCA9685 (16-channel servo/LED driver)
- **EEPROMs**: AT24C256 (external memory)
- **Real-time clocks**: DS3231, DS1307

### Troubleshooting:
- **No ACK / device not found**: 
  - Check pull-up resistors (4.7kΩ on SDA and SCL)
  - Verify device address (use I2C scanner)
  - Check wiring (SDA, SCL, VCC, GND)
  - Check voltage (3.3V vs 5V compatibility)
- **Intermittent communication**: 
  - Add decoupling capacitors (0.1μF near device VCC)
  - Shorten wires (I2C works best under 1 meter)
  - Lower speed (change from 400kHz to 100kHz)
- **Bus lockup**: Reset microcontroller, check for noise on lines

---

## ⚡ SPI (Serial Peripheral Interface)

### Overview:
- **Wires**: 4 (MISO, MOSI, SCK, CS/SS) + GND
- **Speed**: 1-50MHz (much faster than I2C)
- **Devices**: Multiple (one CS pin per slave)
- **Topology**: Master-slave (full-duplex simultaneous read/write)
- **Synchronization**: Synchronous (clock line SCK)

### SPI Bus Structure:
```
Master                    Slave 1              Slave 2
  MISO  <------------------  MISO  <-----------  MISO
  MOSI  ------------------>  MOSI  ----------->  MOSI
  SCK   ------------------>  SCK   ----------->  SCK
  CS1   ------------------>  CS
  CS2   ------------------------------------->  CS
  GND   ------------------>  GND   ----------->  GND
```

**Pins**:
- **MISO**: Master In, Slave Out (data from slave to master)
- **MOSI**: Master Out, Slave In (data from master to slave)
- **SCK**: Serial Clock (master controls timing)
- **CS/SS**: Chip Select / Slave Select (master selects which slave)

### How SPI Works:
1. Master pulls CS LOW to select slave
2. Master generates clock pulses on SCK
3. Data shifts out on MOSI (master → slave)
4. Data shifts in on MISO (slave → master) **simultaneously**
5. Master pulls CS HIGH to deselect slave

**Full-duplex**: Send and receive at same time (unlike I2C/UART)

### Arduino SPI:
```cpp
#include <SPI.h>

const int CS_PIN = 10;

void setup() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);  // Deselect initially
  
  SPI.begin();  // Initialize SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // Slow down clock (1MHz on 16MHz Arduino)
}

void loop() {
  digitalWrite(CS_PIN, LOW);   // Select device
  
  byte response = SPI.transfer(0x42);  // Send 0x42, receive byte simultaneously
  
  digitalWrite(CS_PIN, HIGH);  // Deselect device
  
  delay(100);
}
```

### SPI Pins:
- **Arduino Uno**: MISO = 12, MOSI = 11, SCK = 13, CS = 10 (or any GPIO)
- **Arduino Mega**: MISO = 50, MOSI = 51, SCK = 52, CS = 53
- **ESP32**: VSPI (default): MISO = GPIO 19, MOSI = GPIO 23, SCK = GPIO 18, CS = GPIO 5

### SPI Modes:
SPI has 4 modes based on clock polarity (CPOL) and phase (CPHA):

| Mode | CPOL | CPHA | Description |
|------|------|------|-------------|
| 0    | 0    | 0    | Clock idle LOW, sample on rising edge |
| 1    | 0    | 1    | Clock idle LOW, sample on falling edge |
| 2    | 1    | 0    | Clock idle HIGH, sample on falling edge |
| 3    | 1    | 1    | Clock idle HIGH, sample on rising edge |

```cpp
SPI.setDataMode(SPI_MODE0);  // Most common
```

### Applications:
- **SD cards**: File storage (SPI mode)
- **Displays**: TFT LCD, E-paper displays
- **Sensors**: High-speed accelerometers, magnetometers
- **RF modules**: NRF24L01 (2.4GHz wireless)
- **Ethernet**: W5500, ENC28J60
- **ADC/DAC**: MCP3008 (8-channel ADC), MCP4921 (DAC)

### Advantages:
- **Fast**: 10-50MHz typical (vs 400kHz for I2C)
- **Full-duplex**: Simultaneous read/write
- **Simple protocol**: No addressing, direct master control

### Disadvantages:
- **More wires**: 4 + 1 per slave (vs 2 for I2C)
- **No multi-master**: Only one master allowed
- **No ACK**: Master doesn't know if slave received data

### Troubleshooting:
- **No response**: Check CS is going LOW, verify SPI mode matches device
- **Garbled data**: Wrong SPI mode, clock too fast for long wires
- **Works sometimes**: Check electrical noise, add decoupling caps

---

## 🚗 CAN Bus (Controller Area Network)

### Overview:
- **Wires**: 2 (CAN_H, CAN_L) + GND
- **Speed**: 125kbps - 1Mbps
- **Devices**: Multi-master (up to 127 nodes)
- **Topology**: Multi-drop bus (all devices on same two wires)
- **Robustness**: Differential signaling, very noise-resistant
- **Use**: Automotive, industrial robots, distributed control systems

### CAN Bus Structure:
```
                   120Ω
      Node1   Node2   Node3   Node4   120Ω
        |       |       |       |       |
   -----o-------o-------o-------o-------o----- CAN_H
   -----o-------o-------o-------o-------o----- CAN_L
        |       |       |       |       |
       GND     GND     GND     GND     GND
```

**120Ω termination resistors** at both ends of bus!

### Why CAN Bus?
- **Automotive standard**: Used in cars for ECU communication
- **Robust**: Works in electrically noisy environments (motors, high voltage)
- **Multi-master**: Any node can transmit
- **Message priority**: Higher priority messages sent first
- **Error detection**: Built-in CRC, automatic retransmission

### CAN Message Format:
- **Identifier**: 11-bit (standard) or 29-bit (extended)
- **Data**: 0-8 bytes per message
- **Priority**: Lower identifier = higher priority

### Arduino CAN (MCP2515 module):
```cpp
#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  
  // Initialize CAN at 500kbps
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN Initialized");
  }
  CAN.setMode(MCP_NORMAL);
}

void loop() {
  // Send CAN message
  byte data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  CAN.sendMsgBuf(0x100, 0, 8, data);  // ID=0x100, standard frame, 8 bytes
  
  // Receive CAN message
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    
    CAN.readMsgBuf(&rxId, &len, rxBuf);
    
    Serial.print("ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" Data: ");
    for(int i = 0; i < len; i++) {
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  delay(100);
}
```

### CAN Baud Rates:
- 125kbps: Long cables (up to 500m)
- 250kbps: Medium cables
- 500kbps: Common in industrial robots
- 1Mbps: Short cables (up to 40m)

### Applications:
- **Automotive**: ECU communication, OBD-II diagnostics
- **Industrial robots**: Distributed motor controllers
- **Heavy machinery**: Construction equipment communication
- **Agricultural robots**: Multi-node sensor systems
- **Mobile robots**: High-reliability command/control

### Advantages:
- **Extremely robust**: Differential signaling, immune to noise
- **Multi-master**: Any node can send
- **Built-in priority**: Critical messages always sent first
- **Error handling**: Automatic error detection and retransmission
- **Long cables**: Works up to 500m (at lower speeds)

### Disadvantages:
- **Complex**: Requires CAN controller IC (MCP2515, etc.)
- **Limited data**: Only 8 bytes per message
- **Termination required**: 120Ω resistors at both ends

---

## 📊 Protocol Comparison

| Feature | UART | I2C | SPI | CAN |
|---------|------|-----|-----|-----|
| **Wires** | 2 | 2 | 4+ | 2 |
| **Speed** | Up to 1Mbps | Up to 400kHz | Up to 50MHz | Up to 1Mbps |
| **Devices** | 2 (point-to-point) | Up to 127 | Multiple (1 CS/slave) | Up to 127 |
| **Complexity** | Simple | Medium | Medium | Complex |
| **Robustness** | Low | Medium | Low | Very High |
| **Use Case** | GPS, Bluetooth, debug | Sensors, displays | SD card, high-speed sensors | Automotive, industrial |

---

## 🛠️ Practical Tips

### Choosing a Protocol:
- **UART**: Simple sensor (GPS), wireless module, debugging
- **I2C**: Multiple low-speed sensors (IMU, temp, pressure)
- **SPI**: High-speed data (SD card, display, ADC)
- **CAN**: Noisy environments, automotive, distributed systems

### Common Issues:
1. **No pull-up resistors (I2C)**: Most common I2C problem
2. **Wrong baud rate (UART)**: Check datasheet for correct rate
3. **Voltage mismatch**: 5V ↔ 3.3V requires level shifting
4. **Long wires**: I2C/SPI struggle beyond 1m (use UART or CAN for longer distances)
5. **Multiple I2C devices with same address**: Use I2C multiplexer (TCA9548A)

---

## 📚 Resources

- I2C Specification (NXP)
- SPI Tutorial (SparkFun)
- CAN Bus Explained (CSS Electronics)
- "Serial Communications" by Arduino
- Logic Analyzer for protocol debugging (Saleae, cheap clones)

---

**Next**: [Embedded Programming](../08-embedded-programming/) - C/C++ for microcontrollers and best practices

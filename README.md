# RemoteXY Drone Controller with MPU6050, ESCs, and Li-Po Voltage Monitoring

This project controls a drone using an Arduino with an MPU6050 sensor, servos (ESCs), and a RemoteXY mobile interface for wireless control. The setup allows you to control the drone’s pitch, roll, and throttle through a graphical user interface on your smartphone. It also includes a voltage monitoring feature for a 6-cell Li-Po battery, ensuring the battery voltage stays within a safe range.

## Requirements

### Hardware:
- Arduino (compatible with ESP8266 for Wi-Fi)
- MPU6050 (Gyro/Accelerometer)
- 4 ESCs (Electronic Speed Controllers)
- 4 Motors
- Wi-Fi Module (ESP8266)
- Power Supply
- Android/iOS device with the RemoteXY app
- Resistors for Voltage Divider (100kΩ and 20kΩ)
- 6S Li-Po Battery (18.2V - 25.2V)

### Software:
- Arduino IDE
- RemoteXY library (v3.1.13 or later)
- Servo library
- Adafruit MPU6050 and Sensor libraries

## RemoteXY Setup

Download the RemoteXY app and configure the connection:
- **Wi-Fi SSID:** `shiro`
- **Wi-Fi Password:** `12345678`
- **Server Port:** `6377`

### Features:
- **Joysticks**: Two joysticks for controlling pitch, roll, and yaw.
- **Switch**: A toggle switch to control an LED (or another component).
- **Voltage Monitoring**: Monitors a 6-cell Li-Po battery voltage to ensure it doesn't drop below 19.2V (3.2V per cell), with warnings via the Serial Monitor if the voltage is too low.

## Pin Layout

| **Component**           | **Arduino Pin**   |
|-------------------------|-------------------|
| **ESC 1 (Motor 1)**      | Pin 9             |
| **ESC 2 (Motor 2)**      | Pin 10            |
| **ESC 3 (Motor 3)**      | Pin 8             |
| **ESC 4 (Motor 4)**      | Pin 7             |
| **MPU6050 SDA**          | A4 (I2C)          |
| **MPU6050 SCL**          | A5 (I2C)          |
| **Wi-Fi Module (ESP8266)**| TX/RX (Serial)    |
| **LED (Optional)**       | Pin 5             |
| **Voltage Divider Output**| A0                |

## Setup Instructions

1. **Install the required libraries**:
   - RemoteXY
   - Adafruit MPU6050
   - Servo

2. **Build the voltage divider circuit**:
   - Use a 100kΩ resistor and a 20kΩ resistor in a voltage divider configuration to scale down the Li-Po battery voltage for safe measurement by the Arduino.
   - Connect the voltage divider output to Arduino analog pin A0.

3. **Upload the provided code** to your Arduino.

4. **Open the RemoteXY app** and connect to the drone's Wi-Fi network.

5. **Use the joysticks** to control the drone's movement and adjust the throttle.

6. **Monitor battery voltage** via the Serial Monitor to ensure safe operation.

## Notes
- Ensure the ESCs are properly calibrated for your motors.
- Adjust the MPU6050 sensor orientation in the code if necessary.
- You may need to fine-tune the pitch/roll throttle adjustment for stable flight.
- The voltage monitoring feature helps prevent over-discharging of the Li-Po battery, protecting it from damage.

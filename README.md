# RemoteXY Drone Controller with MPU6050 and ESCs

This project controls a drone using an Arduino with an MPU6050 sensor, servos (ESCs), and a RemoteXY mobile interface for wireless control. The setup allows you to control the drone’s pitch, roll, and throttle through a graphical user interface on your smartphone.

## Requirements

### Hardware:
- Arduino (compatible with ESP8266 for Wi-Fi)
- MPU6050 (Gyro/Accelerometer)
- 4 ESCs (Electronic Speed Controllers)
- 4 Motors
- Wi-Fi Module (ESP8266)
- Power Supply
- Android/iOS device with the RemoteXY app

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

## Pin Layout

| **Component**          | **Arduino Pin**   |
|------------------------|-------------------|
| **ESC 1 (Motor 1)**     | Pin 9             |
| **ESC 2 (Motor 2)**     | Pin 10            |
| **ESC 3 (Motor 3)**     | Pin 8             |
| **ESC 4 (Motor 4)**     | Pin 7             |
| **MPU6050 SDA**         | A4 (I2C)          |
| **MPU6050 SCL**         | A5 (I2C)          |
| **Wi-Fi Module (ESP8266)** | TX/RX (Serial)   |
| **LED (Optional)**      | Pin 5             |

## Setup Instructions

1. Install the required libraries:
   - RemoteXY
   - Adafruit MPU6050
   - Servo

2. Upload the provided code to your Arduino.

3. Open the RemoteXY app and connect to the drone's Wi-Fi network.

4. Use the joysticks to control the drone's movement and adjust the throttle.

## Notes
- Ensure the ESCs are properly calibrated for your motors.
- Adjust the MPU6050 sensor orientation in the code if necessary.
- You may need to fine-tune the pitch/roll throttle adjustment for stable flight.
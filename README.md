# Droony - ESP32 Drone Flight Controller

Welcome to **Droony**, an open-source drone flight controller project based on ESP32. This project integrates sensor fusion, PID control loops, and wireless control via RemoteXY Wi-Fi interface to enable stable quadcopter flight.

---

## Features

- **ESP32 based flight controller** with multitasking support
- Sensor integration:
  - WT901 IMU (Inertial Measurement Unit) via I2C
  - Barometer for altitude measurement
  - ADC voltage monitoring for battery status
- PID Controllers for pitch and roll stabilization
- PWM motor control with ESC output (4 motors)
- Wireless remote control using **RemoteXY** via Wi-Fi Access Point mode
- Real-time telemetry data displayed on RemoteXY GUI:
  - Voltage
  - Altitude
  - Pitch & Roll angles
  - Flight status
  - Remaining flight time estimation

---

## Hardware Setup

| Component           | Connection           | Notes                           |
|---------------------|---------------------|--------------------------------|
| ESP32               | Main controller     | GPIO pins for PWM and I2C       |
| WT901 IMU           | I2C (SDA/SCL)       | For orientation sensing         |
| Barometer           | I2C or SPI          | For altitude measurement        |
| Battery Voltage ADC | GPIO36 (ADC1_CH0)   | Voltage divider needed           |
| ESC Motors          | GPIO32, 33, 25, 26  | PWM output channels              |
| LED Indicator       | GPIO2               | Status LED                      |

---

## Software Details

### Libraries Used

- [RemoteXY](https://remotexy.com/) — for remote GUI control over Wi-Fi
- [WT901_I2C](https://github.com/...) — IMU sensor interface
- PIDController — Custom PID control class
- ESP-IDF `ledc` PWM driver for motor control
- ESP ADC Calibration library for battery voltage measurement

---

### RemoteXY GUI Overview

The interface provides:

- Two joysticks controlling throttle and direction
- PID tuning sliders (P, I, D gains)
- Arm/Disarm toggle switch
- Calibration button
- Live telemetry values:
  - Voltage (V)
  - Altitude (m)
  - Pitch and Roll (°)
  - Flight time left (minutes)
  - Status message

---

### PID Tuning Parameters

| Parameter | Default Value | Description                           |
|-----------|---------------|-----------------------------------|
| Kp        | 0.15          | Proportional gain                  |
| Ki        | 0.10          | Integral gain                     |
| Kd        | 0.02          | Derivative gain                   |
| Filter T  | 0.0053        | Sampling time / cutoff frequency  |

These parameters can be tuned dynamically from the RemoteXY GUI to optimize flight stability.

---

## Build & Upload Instructions

1. Install ESP-IDF or Arduino IDE with ESP32 support.
2. Clone this repository.
3. Install required libraries for RemoteXY and sensors.
4. Configure Wi-Fi SSID and password in the source code if needed.
5. Connect the hardware according to the pin mappings.
6. Compile and upload the firmware to the ESP32.
7. Connect to the ESP32 Wi-Fi network named `Droony` with password `12345678`.
8. Open the RemoteXY app or web client on your phone/computer.
9. Control and monitor your drone in real time.

---

## Notes & Tips

- Ensure your ESCs and motors are compatible with the PWM frequency and signal range.
- Calibrate the IMU before first flight using the calibration button.
- Monitor battery voltage carefully to avoid crashes due to low power.
- Start PID tuning with low gains and gradually increase to avoid oscillations.
- Disarm the drone when not flying to avoid motor spinning.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Contact

For questions or contributions, please open an issue or pull request on GitHub.

---

Thank you for checking out **Droony** — happy flying! 🚁✨

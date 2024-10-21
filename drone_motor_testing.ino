/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.13 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.14.08 or later version;
     - for iOS 1.11.2 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <RemoteXY.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

// RemoteXY configuration
#define REMOTEXY_MODE__ESP8266_HARDSERIAL_POINT
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "shiro"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY configurator 
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,5,0,0,0,47,0,18,0,0,0,31,1,106,200,1,1,3,0,5,
    12,18,49,49,32,2,26,31,5,15,121,49,49,32,2,26,31,2,56,79,
    44,22,1,2,26,31,31,79,78,0,79,70,70,0 };
  
struct {
    int8_t joystick_01_x; // from -100 to 100
    int8_t joystick_01_y; // from -100 to 100
    int8_t joystick_02_x; // from -100 to 100
    int8_t joystick_02_y; // from -100 to 100
    uint8_t switch_01; // =1 if switch ON and =0 if OFF
    uint8_t connect_flag;  // =1 if wire connected, else =0
} RemoteXY;
#pragma pack(pop)

Adafruit_MPU6050 mpu;
Servo esc1, esc2, esc3, esc4;

// Kalman filter variables
Matrix<2, 2> F; // State transition matrix
Matrix<2, 2> H; // Measurement matrix
Matrix<2, 2> Q; // Process noise covariance
Matrix<2, 2> R; // Measurement noise covariance
Matrix<2, 2> P; // Estimate error covariance
Matrix<2, 1> x; // State vector
Matrix<2, 1> z; // Measurement vector

const float dt = 0.01; // Time step in seconds (100Hz update rate)

void setup() 
{
    RemoteXY_Init(); 
    
    if (!mpu.begin()) {
        while (1) {
            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    pinMode(5, OUTPUT);

    esc1.attach(9);
    esc2.attach(10);
    esc3.attach(8);
    esc4.attach(7);

    // Initialize ESCs
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);

    // Initialize Kalman filter matrices
    F = {1, -dt, 0, 1};
    H = {1, 0, 0, 1};
    Q = {0.001, 0, 0, 0.003};
    R = {0.03, 0, 0, 0.03};
    P = {1, 0, 0, 1};
    x = {0, 0}; // Initial state (pitch and pitch rate)

    delay(2000);
}

void kalmanUpdate(float angle, float rate) {
    // Prediction
    x = F * x;
    P = F * P * ~F + Q;

    // Measurement
    z = {angle, rate};

    // Update
    Matrix<2, 2> S = H * P * ~H + R;
    Matrix<2, 2> K = P * ~H * S.Inverse();
    x = x + K * (z - H * x);
    P = (Matrix<2,2>::Identity() - K * H) * P;
}

void loop() 
{
    RemoteXY_Handler();
    
    digitalWrite(5, RemoteXY.switch_01);

    static unsigned long previousTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= 10) { // Ensure 100Hz update rate
        previousTime = currentTime;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Calculate angles from accelerometer data
        float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
        float accAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

        // Apply Kalman filter
        kalmanUpdate(accAngleX, g.gyro.x);
        float pitch = x(0);
        kalmanUpdate(accAngleY, g.gyro.y);
        float roll = x(0);

        // Yaw calculation (simple integration, as before)
        static float yaw = 0;
        yaw += g.gyro.z * dt;

        // Calculate base throttle from joystick input
        int baseThrottle = map(max(0, RemoteXY.joystick_01_x), 0, 100, 1000, 2000);

        // Adjust throttle based on pitch and roll
        int throttle1 = baseThrottle + pitch + roll;
        int throttle2 = baseThrottle - pitch + roll;
        int throttle3 = baseThrottle + pitch - roll;
        int throttle4 = baseThrottle - pitch - roll;

        // Constrain throttle values
        throttle1 = constrain(throttle1, 1000, 2000);
        throttle2 = constrain(throttle2, 1000, 2000);
        throttle3 = constrain(throttle3, 1000, 2000);
        throttle4 = constrain(throttle4, 1000, 2000);

        // Send throttle signals to ESCs
        esc1.writeMicroseconds(throttle1);
        esc2.writeMicroseconds(throttle2);
        esc3.writeMicroseconds(throttle3);
        esc4.writeMicroseconds(throttle4);

        // Optional: Print debug information
        // Serial.print("Pitch: "); Serial.print(pitch);
        // Serial.print(", Roll: "); Serial.print(roll);
        // Serial.print(", Yaw: "); Serial.println(yaw);
    }
}

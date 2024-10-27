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

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Kalman.h> // 

// RemoteXY configuration
#define REMOTEXY_MODE__ESP8266_HARDSERIAL_POINT
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "shiro"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377
#include <RemoteXY.h>  // RemoteXY include


float RatePitch, RateRoll, RateYaw;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 54 bytes
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

// Voltage measurement variables
const int analogPin = A0;  // Pin connected to the voltage divider
const float R1 = 100000.0; // Resistor 1 value in ohms (100k ohms)
const float R2 = 20000.0;  // Resistor 2 value in ohms (20k ohms)
const float referenceVoltage = 5.0; // Assuming Arduino operates at 5V
const int adcMax = 1023;  // Max ADC value for 10-bit ADC
const float lowVoltageThreshold = 19.2; // 6 cells at 3.2V per cell





void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void batteryVoltageDetection(){
  // Read the battery voltage
  int sensorValue = analogRead(analogPin);
  float voltageOut = (sensorValue * referenceVoltage) / adcMax;
  float batteryVoltage = voltageOut * (R1 + R2) / R2;
}

void setup() 
{
  RemoteXY_Init(); 
  Serial.begin(115200);  // Initialize serial for debugging
    
  if (!mpu.begin()) {
      while (1) {
          delay(10);
      }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  pinMode(5, OUTPUT); // Set pin 5 as an output

  // Initialize ESCs
  esc1.attach(9);  // Attach the first ESC to pin 9
  esc2.attach(6); // Attach the second ESC to pin 6 bottom right mottor
  esc3.attach(8);  // Attach the third ESC to pin 8
  esc4.attach(7);  // Attach the fourth ESC to pin 7
  
  // Wait 2 seconds to allow the ESC to recognize the startup condition
  delay(2000); 
   
  //esc1.writeMicroseconds(1000);  // Send the minimum throttle signal to the ESC
  //esc2.writeMicroseconds(1000);  // Uncomment if needed for calibration
  //testing
  
    // Initialize Kalman filter matrices
    F = {1, -dt, 0, 1};
    H = {1, 0, 0, 1};
    Q = {0.001, 0, 0, 0.003};
    R = {0.03, 0, 0, 0.03};
    P = {1, 0, 0, 1};
    x = {0, 0}; // Initial state (pitch and pitch rate)
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
      
    // Read the battery voltage
    int sensorValue = analogRead(analogPin);
    float voltageOut = (sensorValue * referenceVoltage) / adcMax;
    float batteryVoltage = voltageOut * (R1 + R2) / R2;

    // Check if the battery voltage is below the safe threshold
    if (batteryVoltage < lowVoltageThreshold) {
      Serial.println("Warning: Battery voltage too low! Please charge the battery.");
    } else {
      Serial.print("Battery Voltage: ");
      Serial.println(batteryVoltage);
    }
  
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
              RateRoll-=RateCalibrationRoll;
      RatePitch-=RateCalibrationPitch;
      RateYaw-=RateCalibrationYaw;

      InputThrottle = map(max(0, RemoteXY.joystick_01_x), 0, 100, 1000, 1700);
      
      DesiredRateYaw=0.15*(RemoteXY.RemoteXYjoystick_01_y-1500);   // change rate is 15 degrees per sec
      DesiredRateRoll= 0.15 * (RemoteXY.RemoteXYjoystick_02_y-1500);
      DesiredRatePitch=0.15*(RemoteXY.RemoteXYjoystick_02_x - 1500);

      // Error rates
      ErrorRateRoll=DesiredRateRoll-RateRoll;
      ErrorRatePitch=DesiredRatePitch-RatePitch;
      ErrorRateYaw=DesiredRateYaw-RateYaw;

      pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw,
       IRateYaw, DRateYaw, PrevErrorRateYaw,
       PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];
  if (InputThrottle > 1800) InputThrottle = 1800;
  MotorInput1= 1.024*(InputThrottle-InputRoll-InputPitch-InputYaw);
  MotorInput2= 1.024*(InputThrottle-InputRoll+InputPitch+InputYaw);
  MotorInput3= 1.024*(InputThrottle+InputRoll+InputPitch-InputYaw);
  MotorInput4= 1.024*(InputThrottle+InputRoll-InputPitch+InputYaw);
}

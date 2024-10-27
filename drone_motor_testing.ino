#define REMOTEXY_MODE__ESP8266_HARDSERIAL_POINT
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 115200
#define REMOTEXY_WIFI_SSID "shiro"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>
#include <RemoteXY.h>

using namespace BLA;

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

// Calibration variables
const int CALIBRATION_READINGS = 20;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;

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
  int sensorValue = analogRead(analogPin);
  float voltageOut = (sensorValue * referenceVoltage) / adcMax;
  float batteryVoltage = voltageOut * (R1 + R2) / R2;
  if (batteryVoltage < lowVoltageThreshold) {
    Serial.println("Warning: Battery voltage too low! Please charge the battery.");
  } else {
    Serial.print("Battery Voltage: ");
    Serial.println(batteryVoltage);
  }
}

void calibrateSensors() {
    Serial.println("Calibrating sensors. Keep the drone still...");
    float sumRoll = 0, sumPitch = 0, sumYaw = 0;

    for (int i = 0; i < CALIBRATION_READINGS; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        sumRoll += g.gyro.x;
        sumPitch += g.gyro.y;
        sumYaw += g.gyro.z;

        delay(50); // Short delay between readings
    }

    RateCalibrationRoll = sumRoll / CALIBRATION_READINGS;
    RateCalibrationPitch = sumPitch / CALIBRATION_READINGS;
    RateCalibrationYaw = sumYaw / CALIBRATION_READINGS;

    Serial.println("Calibration complete.");
    Serial.print("Roll Offset: "); Serial.println(RateCalibrationRoll);
    Serial.print("Pitch Offset: "); Serial.println(RateCalibrationPitch);
    Serial.print("Yaw Offset: "); Serial.println(RateCalibrationYaw);
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

  // Perform automatic calibration
  calibrateSensors();
  
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
    Matrix<2, 2> K;
    Matrix<2, 2> I = {1, 0, 0, 1}; // Identity matrix
    
    // Check if S is invertible
    float det = S(0,0) * S(1,1) - S(0,1) * S(1,0);
    if(abs(det) > 1e-6) {
        Invert(S, K);
        K = P * ~H * K;
        x = x + K * (z - H * x);
        P = (I - K * H) * P;
    } else {
        // S is not invertible, skip the update step
        Serial.println("Warning: Kalman filter update skipped due to singular matrix");
    }
}

void loop() 
{
    RemoteXY_Handler();
    
    digitalWrite(5, RemoteXY.switch_01);
      
    batteryVoltageDetection();
  
    static unsigned long previousTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - previousTime >= 10) { // Ensure 100Hz update rate
        previousTime = currentTime;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Apply calibration offsets
        RateRoll = g.gyro.x - RateCalibrationRoll;
        RatePitch = g.gyro.y - RateCalibrationPitch;
        RateYaw = g.gyro.z - RateCalibrationYaw;

        // Calculate angles from accelerometer data
        float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
        float accAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

        // Apply Kalman filter
        kalmanUpdate(accAngleX, RateRoll);
        float pitch = x(0);
        kalmanUpdate(accAngleY, RatePitch);
        float roll = x(0);

        // Yaw calculation (simple integration)
        static float yaw = 0;
        yaw += RateYaw * dt;

        // Calculate base throttle from joystick input
        InputThrottle = map(max(0, RemoteXY.joystick_01_x), 0, 100, 1000, 1700);
      
        DesiredRateYaw = 0.15 * (RemoteXY.joystick_01_y - 50);   // change rate is 15 degrees per sec
        DesiredRateRoll = 0.15 * (RemoteXY.joystick_02_y - 50);
        DesiredRatePitch = 0.15 * (RemoteXY.joystick_02_x - 50);

        // Error rates
        ErrorRateRoll = DesiredRateRoll - RateRoll;
        ErrorRatePitch = DesiredRatePitch - RatePitch;
        ErrorRateYaw = DesiredRateYaw - RateYaw;

        pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
        InputRoll = PIDReturn[0];
        PrevErrorRateRoll = PIDReturn[1]; 
        PrevItermRateRoll = PIDReturn[2];

        pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch = PIDReturn[0]; 
        PrevErrorRatePitch = PIDReturn[1]; 
        PrevItermRatePitch = PIDReturn[2];

        pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
        InputYaw = PIDReturn[0]; 
        PrevErrorRateYaw = PIDReturn[1]; 
        PrevItermRateYaw = PIDReturn[2];

        if (InputThrottle > 1800) InputThrottle = 1800;
        MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
        MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
        MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
        MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

        // Apply motor inputs
        esc1.writeMicroseconds(MotorInput1);
        esc2.writeMicroseconds(MotorInput2);
        esc3.writeMicroseconds(MotorInput3);
        esc4.writeMicroseconds(MotorInput4);
    }
}
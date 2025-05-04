#define LED_PIN 2  // Change this if needed (e.g., 4 or 5)
#include "WT901_I2C.h"
//#include "driver/ledc.h"
#include <driver/ledc.h>
#include "Barometer.h"
#include "PIDController.h" 
#include "esp_adc_cal.h"
 
//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "Droony"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 525 bytes
  { 255,11,0,65,0,6,2,19,0,0,0,68,114,111,111,110,121,0,25,1,
  200,84,1,1,44,0,5,20,27,42,42,32,2,26,31,5,131,27,43,43,
  32,2,26,31,67,21,1,21,4,77,17,26,2,129,75,3,54,10,64,17,
  68,114,111,111,110,121,32,240,159,155,184,0,65,193,1,6,6,112,67,51,
  75,98,6,5,135,26,16,129,5,1,23,4,64,2,86,111,108,116,97,103,
  101,58,0,129,146,2,33,4,64,2,69,46,32,116,105,109,101,32,108,101,
  102,116,58,0,67,167,2,21,4,77,17,26,2,67,60,17,12,4,77,2,
  26,2,1,6,69,9,9,0,95,31,0,129,44,17,15,4,64,17,65,108,
  116,105,116,117,100,101,58,0,129,84,17,17,4,64,17,80,105,116,99,104,
  58,0,129,122,17,13,4,64,17,82,111,108,108,58,0,67,96,17,11,4,
  77,2,26,2,67,131,17,11,4,77,2,26,2,4,107,27,7,33,0,2,
  26,4,78,27,7,33,0,2,26,4,93,27,7,33,0,2,26,129,109,61,
  4,3,64,2,75,100,0,129,95,61,3,3,64,2,75,105,0,129,80,61,
  4,3,64,2,75,112,0,2,86,66,22,5,1,2,27,31,31,80,73,68,
  32,85,78,76,79,67,75,32,0,80,73,68,32,76,79,67,75,69,68,0,
  129,38,76,20,4,64,2,83,116,97,116,117,115,58,0,10,180,64,18,18,
  48,1,135,31,83,84,79,80,32,0,31,83,116,97,114,116,0,129,4,79,
  23,3,64,95,67,97,108,105,98,114,97,116,101,32,0,67,77,25,9,4,
  77,2,26,4,67,91,25,10,4,77,2,26,4,67,106,25,9,4,77,2,
  26,4,4,186,23,6,31,0,2,26,129,181,55,16,3,64,2,77,97,120,
  32,84,104,114,111,116,108,101,0,67,185,21,9,4,85,2,26,67,3,12,
  12,4,85,2,26,67,18,12,12,4,85,2,26,67,3,21,12,4,85,2,
  26,67,18,21,12,4,85,2,26,129,7,8,5,4,64,17,70,76,0,129,
  22,8,5,4,64,17,70,82,0,129,7,17,5,4,64,17,66,76,0,129,
  22,17,5,4,64,17,66,82,0,67,3,30,12,4,85,2,26,67,3,39,
  12,4,85,2,26,129,7,26,6,4,64,17,67,80,0,129,7,35,6,4,
  64,17,67,82,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100
  int8_t joystick_02_x; // from -100 to 100
  int8_t joystick_02_y; // from -100 to 100
  uint8_t Calibirate; // =1 if button pressed, else =0
  int8_t PID_D; // from 0 to 100
  int8_t PID_P; // from 0 to 100
  int8_t PID_I; // from 0 to 100
  uint8_t PIDLock; // =1 if switch ON and =0 if OFF
  uint8_t isArmed; // =1 if state is ON, else =0
  int8_t MaxThrottleSlider; // from 0 to 100

    // output variables
  float voltageGui;
  uint8_t led_01_r; // =0..255 LED Red brightness
  uint8_t led_01_g; // =0..255 LED Green brightness
  uint8_t led_01_b; // =0..255 LED Green brightness
  char statusGui[16]; // string UTF8 end zero
  float timeLeftGui;
  float altitudeGui;
  float pitchGui;
  float rollGui;
  float Kpv;
  float Kiv;
  float Kdv;
  int16_t MaxThrottle; // -32768 .. +32767
  int16_t FLM; // -32768 .. +32767
  int16_t FRM; // -32768 .. +32767
  int16_t BLM; // -32768 .. +32767
  int16_t BRM; // -32768 .. +32767
  int16_t CP; // -32768 .. +32767
  int16_t CR; // -32768 .. +32767

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

//------ semaphores and loops creattion / for Cpu assinmegts
TaskHandle_t TaskControlLoop;
TaskHandle_t TaskWifiLoop;
SemaphoreHandle_t i2cMutex;

// ----- IMU global data -----
float g_pitch = 0;
float g_roll = 0;
float g_pitch_offset = 0;
float g_roll_offset = 0;

// PWM Configuration
// Constants
const int PWM_FREQ = 500;              // 400 Hz
const int PWM_RESOLUTION = 12;         // 12-bit resolution (0–4095)
const int PWM_MIN = 1000;              // Min pulse in microseconds
const int PWM_MAX = 2000;              // Max pulse in microseconds
const int DUTY_MAX = (1 << PWM_RESOLUTION) - 1;  // 4095 for 12-bit

// GPIO pin assignments for motors
const int MOTOR_FL_PIN = 32;  // Front left Motor --  yellow
const int MOTOR_FR_PIN = 33;  // Front right motor --  white
const int MOTOR_BL_PIN = 25;  // back left motor --  gray
const int MOTOR_BR_PIN = 26;  // back right motor --  orange

// LEDC Channels (can be 0–15)
const int CH_FL = 0;
const int CH_FR = 1;
const int CH_BL = 2;
const int CH_BR = 3;


// ----- Create PID instances -----
float initial_Kp = 0.15;  // Proportional gain
float initial_Ki = 0.10;  // Integral gain
float initial_Kd = 0.02;  // Derivative gain
float initial_T = 0.0053;  // a starting point of 30Hz cutoff.
int maxThrottle = 1200;

PIDController pid_pitch(initial_Kp, initial_Ki, initial_Kd, initial_T);
PIDController pid_roll(initial_Kp, initial_Ki, initial_Kd, initial_T);

// | Parameter | Current Value | Comment |
// |----------|----------------|---------|
// | **Kp** | 0.15 | Good low-mid starting point. Should give some response without oscillation. |
// | **Ki** | 0.10 | Conservative, won't overshoot quickly. If drift/horizon tilt stays, increase. |
// | **Kd** | 0.02 | Modest; derivative might need to go higher if you see overshoot/oscillation. |
// | **T**  | 0.0053 | 🟢 Excellent for a ~30Hz filter cutoff on this kind of quad. |

// barometer data
float relativeAltitude = 0;

// voltage detection parameters
// Configuration
const int analogPin = 36; // GPIO36 (ADC1_CH0)
const float dividerRatio = 8.5596707;
const float batteryCapacity = 1800.0; // mAh
const float averageCurrent = 15.0;    // Amperes
const float minVoltage = 19.8;        // V
const float maxVoltage = 25.2;        // v

// Variables
float batteryVoltage = 0.0;
float remainingCapacityPercent = 0.0;
float remainingCapacity = 0.0;
float flightTimeRemainingMinutes = 0.0;

// ADC Calibration
esp_adc_cal_characteristics_t adc_chars;


int motorFL = 1000;
int motorFR = 1000;
int motorBL = 1000;
int motorBR = 1000;


int pitch_correction = 0;
int roll_correction  =0;

void controlLoop(void *parameter) {
  static unsigned long lastUpdateTime = 0;
  float dt = 0;

  // // monitoring paramters / comment once not needed
  // static float accumulated_dt = 0;
  // static int sample_count = 0;
  // const int averaging_window = 100; // Average over 50 iterations - adjust as needed

  while (true) {
    unsigned long now = micros();
    dt = (now - lastUpdateTime) / 1e6;  // Convert to seconds
    lastUpdateTime = now; 

    // running time cal logic/ comment once done with debuging
    // accumulated_dt += dt;
    // sample_count++;

    // if (sample_count >= averaging_window) {
    //   float average_dt = accumulated_dt / averaging_window;
    //   Serial.print("Average Control dt> ");
    //   Serial.println(average_dt * 1000, 6); // Print in milliseconds

    //   accumulated_dt = 0; // Reset for the next average
    //   sample_count = 0;
    // }


    if (xSemaphoreTake(i2cMutex, 0) == pdTRUE) {
      // ✅ Access I2C or shared resource here
      // read MPU6050
      updateGlobalAngles();

      //  run main control
      if(RemoteXY.isArmed == 1 and RemoteXY_isConnected() == 1) {
        // Motors can run
        // Read joysticks -> calculate PID -> mix motors -> output to ESCs
        strcpy(RemoteXY.statusGui, "ARMED!");
        writeMotors(dt);
        
      }
      else {
        // Motors disarmed - set all to minimum
        strcpy(RemoteXY.statusGui, "DISARMED."); 
        stopMotors();
        RemoteXY.isArmed = 0;
      }

      xSemaphoreGive(i2cMutex);  // Always release after use
    }

    vTaskDelay(3);  // Yield time to other tasks (1 tick = ~1ms)
  }
}

void wifiLoop(void *parameter) {
  static unsigned long lastUpdateTime = 0;
  float dt = 0;
  while (true) {
    unsigned long now = micros();
    dt = (now - lastUpdateTime) / 1e6;  // Convert to seconds
    lastUpdateTime = now; 
    // Serial.print("wifi dt> ");
    // Serial.println(dt * 1000, 6);  // prints dt in milliseconds with 6 decimal places

    //paramters that are not common used here
    updateBatteryVoltage();
    estimateFlightTime();
  
    RemoteXY_Handler();  // ✅ Non-blocking communication handling

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      // ✅ Low-priority access to shared I2C
      // e.g., read values for display, telemetry

      //updating Altitude
      relativeAltitude = Barometer::getAltitude();
          
      RemoteXYGuiHandlar();

      // calibiration button
      if (RemoteXY.Calibirate == 1 and RemoteXY.isArmed == 0)
      {
        calibrateAngles();
        RemoteXY.Calibirate = 0;
      }

      xSemaphoreGive(i2cMutex);
    }

    //vTaskDelay(10); // delaz 10 ms affect the control loop time, make it unexpectedlz inccrease!
  }
}

void setup() {
  Serial.begin(115200);
  RemoteXY_Init();
  calibrateAngles();
  setupMotors();

  // Configure ADC
  analogReadResolution(12); // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db); // Measure up to ~3.3V

  // Characterize ADC
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  if (!Barometer::begin()) {
    Serial.println("Barometer failed to initialize!");
    while (1);
  }
  Serial.println("Barometer initialized and calibrated.");

  // ✅ Make sure the mutex was created successfully
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println("Failed to create mutex!");
    while (true); // Halt execution
  }


  Serial.println("Setup complete. Starting tasks...");

  xTaskCreatePinnedToCore(
    controlLoop,
    "Control Loop",
    10000,
    NULL,
    2,                // High priority
    &TaskControlLoop,
    0                 // Core 0
  );

  xTaskCreatePinnedToCore(
    wifiLoop,
    "WiFi and Aux Loop",
    10000,
    NULL,
    1,                // Lower priority
    &TaskWifiLoop,
    1                 // Core 1
  );
}

void loop() {
  // 🔇 Empty — all logic runs in FreeRTOS tasks
}


//------ Helper Methods ---------------

// ----- Calibrate IMU / non urgent task happen while on groud/ wifiLoop -----
void calibrateAngles() {
  JY901.calibrate();

  float pitch_sum = 0;
  float roll_sum = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    JY901.updateAll();

    pitch_sum += JY901.stcAngle.Anglef[0];  // pitch
    roll_sum  += JY901.stcAngle.Anglef[1];  // roll

    vTaskDelay(pdMS_TO_TICKS(10));  // Allow time for stable sampling
  }

  g_pitch_offset = pitch_sum / samples;
  g_roll_offset  = roll_sum / samples;

  updateGlobalAngles();  // Set initial values
  //  reset pid values
  pid_pitch.reset();
  pid_roll.reset();
  //calibrate barometer
  Barometer::calibrate();

  strcpy(RemoteXY.statusGui, "Calibration complete ✅");
  vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second, allow time for user to read the message!
}


// ----- Update pitch/roll from IMU/ Urgent task/ Control loop -----
void updateGlobalAngles() {
  JY901.updateAll();

  g_pitch = JY901.stcAngle.Anglef[0] - g_pitch_offset;
  g_roll  = JY901.stcAngle.Anglef[1] - g_roll_offset;


  // // Debug output
  // Serial.print("Pitch: ");
  // Serial.println(g_pitch);
  // Serial.print("°, Roll: ");
  // Serial.println(g_roll);

}

// ----- Update RemoteXY GUI values /un urgent task/ (call from WiFi loop) -----
void RemoteXYGuiHandlar() {
  RemoteXY.pitchGui = g_pitch;
  RemoteXY.rollGui  = g_roll;
  RemoteXY.altitudeGui = relativeAltitude;
  RemoteXY.voltageGui = batteryVoltage; // Update GUI voltage
  RemoteXY.timeLeftGui = flightTimeRemainingMinutes; // Update GUI

  RemoteXY.FLM = motorFL; // Update GUI
  RemoteXY.FRM = motorFR; // Update GUI
  RemoteXY.BLM = motorBL; // Update GUI
  RemoteXY.BRM = motorBR; // Update GUI
  RemoteXY.CP = pitch_correction; // Update GUI
  RemoteXY.CR = roll_correction; // Update GUI


  if (RemoteXY.PIDLock == 1 and RemoteXY.isArmed == 0) { // If  unlocked, allow updating and un armed update the pids fromt the slider

    // Map the sliders (-100..100) into their ranges
    initial_Kp = map(RemoteXY.PID_P, 0, 100, 0, 500) / 100.0; // 0.0 to 3.0
    initial_Ki = map(RemoteXY.PID_I, 0, 100, 0, 1000) / 1000.0; //0.0 to 0.5
    initial_Kd = map(RemoteXY.PID_D, 0, 100, 0, 100) / 1000.0; //0.0 to 0.1

    RemoteXY.Kpv= initial_Kp;
    RemoteXY.Kiv = initial_Ki;
    RemoteXY.Kdv = initial_Kd; 

    // updating the PID  instances parameters
    pid_pitch.setTunings(initial_Kp, initial_Ki , initial_Kd, initial_T);
    pid_roll.setTunings(initial_Kp, initial_Ki , initial_Kd, initial_T);

    // update max throtle value 
    maxThrottle = map(RemoteXY.MaxThrottleSlider, 0, 100, 1000, 1500);
    RemoteXY.MaxThrottle = maxThrottle;
  }
  else { // show initiliyation value
    RemoteXY.Kpv= initial_Kp;
    RemoteXY.Kiv = initial_Ki;
    RemoteXY.Kdv = initial_Kd; 
    RemoteXY.MaxThrottle = maxThrottle;
  }


}

//--- Motors functions
void setupMotors() {
  ledcSetup(CH_FL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_FR, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_BL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_BR, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_FL_PIN, CH_FL);
  ledcAttachPin(MOTOR_FR_PIN, CH_FR);
  ledcAttachPin(MOTOR_BL_PIN, CH_BL);
  ledcAttachPin(MOTOR_BR_PIN, CH_BR);
}


void writeMotors(float dt) {

  Serial.print("Control dt> ");  // prints dt in milliseconds with 6 decimal places
  Serial.println(dt * 1000, 6);  // prints dt in milliseconds with 6 decimal places

  float throttleSignal = map(max(0, (int)RemoteXY.joystick_01_y), 0, 100, PWM_MIN, maxThrottle);
  float target_pitch = map(RemoteXY.joystick_02_y, -100, 100, 30, -30);
  float target_roll  = map(RemoteXY.joystick_02_x, -100, 100, -30, 30);

  pitch_correction = pid_pitch.compute(target_pitch, g_pitch, dt, 35, -35);
  roll_correction  = pid_roll.compute(target_roll, g_roll, dt, 35, -35);

  // UN NESSARZ AS THE PID IS FINE!
  // if (abs(pitch_correction) < 1.0) pitch_correction = 0.0;
  // if (abs(roll_correction) < 1.0)  roll_correction = 0.0;

  motorFL = throttleSignal + pitch_correction + roll_correction;
  motorFR = throttleSignal + pitch_correction - roll_correction;
  motorBL = throttleSignal - pitch_correction + roll_correction;
  motorBR = throttleSignal - pitch_correction - roll_correction;

  ledcWrite(CH_FL,  usToDuty(motorFL));
  ledcWrite(CH_FR,  usToDuty(motorFR));
  ledcWrite(CH_BL,  usToDuty(motorBL));
  ledcWrite(CH_BR,  usToDuty(motorBR));
   
  // // debugung orintln. pls commrnt once real flight testing
  // Serial.print("g_pitch: ");
  //Serial.println(g_pitch);
  //Serial.print(" g_roll: ");  
  //Serial.println(g_roll);
  // Serial.print("Pitch Correction: "); Serial.println(pitch_correction);
  // Serial.print("Roll Correction: ");  Serial.println(roll_correction);
  // Serial.print("motorFL: "); Serial.println(motorFL);
  // Serial.print("motorFR: "); Serial.println(motorFR);
  // Serial.print("motorBL: "); Serial.println(motorBL);
  // Serial.print("motorBR: "); Serial.println(motorBR);

  // Serial.print("Kp: "); Serial.print(Kp);
  // Serial.print(" | Ki: "); Serial.print(Ki);
  // Serial.print(" | Kd: "); Serial.println(Kd);
}

void stopMotors() {
  int dutyMin = usToDuty(1000); // Maps 1000 µs to correct duty
  ledcWrite(CH_FL, dutyMin);
  ledcWrite(CH_FR, dutyMin);
  ledcWrite(CH_BL, dutyMin);
  ledcWrite(CH_BR, dutyMin);
  //for debuging!!!
 // Serial.println("motor stop ");
}

// Helper: Convert microsecond signal to duty cycle value
int usToDuty(int us) {
  // Clamp to avoid overshooting due to bad inputs
  us = constrain(us, PWM_MIN,  maxThrottle + 50);

  // For 12-bit resolution: max duty = 4095L
  // PWM period = 1 / freq = 1 / 500 = 2ms = 2000us
  // duty = (us  * 4095L) / 2000us
  return (us * DUTY_MAX) / PWM_MAX;
}

void updateBatteryVoltage() {
  uint32_t adc_reading = analogRead(analogPin);
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
  batteryVoltage = (voltage / 1000.0) * dividerRatio; // Convert mV to V and apply divider ratio
}

void estimateFlightTime() {
  remainingCapacityPercent = (batteryVoltage - minVoltage) / (maxVoltage - minVoltage);
  remainingCapacityPercent = constrain(remainingCapacityPercent, 0.0, 1.0);
  remainingCapacity = batteryCapacity * remainingCapacityPercent;
  flightTimeRemainingMinutes = (remainingCapacity / (averageCurrent * 1000.0)) * 60.0;
}

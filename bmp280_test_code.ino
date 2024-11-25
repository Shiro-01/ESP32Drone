#include <Wire.h>
#include <Adafruit_BMP280.h>

// Global variables
Adafruit_BMP280 bmp; // Using I2C

// Altitude and velocity tracking variables
float prevAlt = 0.0;
float currAlt = 0.0;
float vertVel = 0.0;

// Time tracking for velocity calculation
unsigned long prevAltTime = 0;
unsigned long currAltTime = 0;

void setup() {

  Serial.begin(115200);  
  
  // Initialize BMP280 sensor
  if (!bmp.begin()) {
    Serial.println("Failed to find BMP280 chip");
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temperature Oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure Oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time */
}

void loop() {
  
  // Read altitude from the barometer
  currAlt = bmp.readAltitude(1006.64); // Sea level pressure
  currAltTime = millis(); 
  
  // Calculate vertical velocity (rate of altitude change) if there's a previous reading
  if (prevAltTime != 0) {
    float dt = (currAltTime - prevAltTime) / 1000.0; 
    vertVel = (currAlt - prevAlt) / dt; 
  }

  // Update previous altitude and time for the next calculation
  prevAlt = currAlt;
  prevAltTime = currAltTime;

  // Serial.println("Pressure = " + String(bmp.readPressure() / 100) +  " hPa \n");
  Serial.println("Altitude: " + String(currAlt) + " m ");
  Serial.println();
  Serial.println("Vertical Velocity: " + String(vertVel) + " m/s \n");
  delay(750); 
}

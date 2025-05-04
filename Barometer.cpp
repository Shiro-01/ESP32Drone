#include "Barometer.h"

Adafruit_BMP280 Barometer::bmp;
float Barometer::baselineAltitude = 0;

bool Barometer::begin() {
  if (!bmp.begin(0x76)) {  // Default I2C address is 0x76 or 0x77
    return false;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  calibrate();
  return true;
}

void Barometer::calibrate() {
  baselineAltitude = bmp.readAltitude(1013.25); // Replace 1013.25 with actual sea-level pressure if known
}

float Barometer::getAltitude() {
  float current = bmp.readAltitude(1013.25);
  return current - baselineAltitude;
}

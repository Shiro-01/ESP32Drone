#ifndef BAROMETER_H
#define BAROMETER_H

#include <Adafruit_BMP280.h>
#include <Wire.h>

class Barometer {
public:
  static bool begin();
  static float getAltitude();
  static void calibrate();

private:
  static Adafruit_BMP280 bmp;
  static float baselineAltitude;
};

#endif

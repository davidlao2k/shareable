#include <stdint.h>
#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include "AHT_Sensor.h"

// Specify the constants for water vapor and barometric pressure.
#define WATER_VAPOR 17.62f
#define BAROMETRIC_PRESSURE 243.5f

CMD CalibrateCmd[3] = {0xE1, 0x08, 0x00};
CMD MeasureCmd[3]   = {0xAC, 0x33, 0x00};
CMD ResetCmd        = 0xBA;

AHTSensor::AHTSensor() {
}

boolean AHTSensor::begin(unsigned char _addr)
{
  addr = _addr;
  Wire.begin(_addr);
  Wire.beginTransmission(_addr);
  Wire.write(CalibrateCmd, 3);
  Wire.endTransmission();

  delay(500);
  return (readStatus() & 0x68) == 0x08;
}

float AHTSensor::getHumidity(bool _readSensor)
{
  if (_readSensor)
    readSensor();

  unsigned long r  = ((values[1] << 16) | (values[2] << 8) | values[3]) >> 4;
  if (r == 0)
    return 0;

  return r * 100 / 1048576.0;
}

float AHTSensor::getTemperature(bool _readSensor)
{
  if (_readSensor)
    readSensor();

  unsigned long r  = ((values[3] & 0x0F) << 16) | (values[4] << 8) | values[5];
  return ((200 * r) / 1048576.0) - 50;
}

float AHTSensor::getDewPoint()
{
  readSensor();
  float humidity = getHumidity(false);
  float temperature = getTemperature(false);

  float gamma = log(humidity / 100) + WATER_VAPOR * temperature / (BAROMETRIC_PRESSURE + temperature);
  float dewPoint = BAROMETRIC_PRESSURE * gamma / (WATER_VAPOR - gamma);

  return dewPoint;
}

void AHTSensor::readSensor()
{
  for (int i = 0; i < 6; i++)
    values[i] = 0;

  Wire.beginTransmission(addr);
  Wire.write(MeasureCmd, 3);
  Wire.endTransmission();
  delay(100);

  reqSize = 6;
  Wire.requestFrom(addr, reqSize);

  for (int i = 0; Wire.available() > 0; i++)
    values[i] = Wire.read();
}

unsigned char AHTSensor::readStatus()
{
  unsigned char result = 0;

  reqSize = 1;
  Wire.requestFrom(addr, reqSize);
  result = Wire.read();
  return result;
}

void AHTSensor::reset()
{
  Wire.beginTransmission(addr);
  Wire.write(ResetCmd);
  Wire.endTransmission();
  delay(20);
}

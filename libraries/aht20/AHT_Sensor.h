#ifndef AHT_Sensor_H
#define AHT_Sensor_H

#include <stdint.h>

typedef enum {
  Default = 0x38,
  Low     = 0x38,
  High    = 0x39,
} I2C_ADDRESS;

typedef unsigned char CMD;

class AHTSensor
{
  private:
    unsigned long values[6] = {0, 0, 0, 0, 0, 0};
    unsigned char addr;
    uint8_t reqSize;
    void readSensor();

  public:
    AHTSensor();
    boolean begin(unsigned char _addr = Default);
    float getHumidity(bool _readSensor = true);
    float getTemperature(bool _readSensor = true);
    float getDewPoint();
    unsigned char readStatus();
    void reset();
};

#endif

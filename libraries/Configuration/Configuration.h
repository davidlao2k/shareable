#ifndef configuration_h
#define configuration_h
#ifndef ARDUINO_ESP8266_RELEASE
#include "SPIFFS.h"
#endif

struct WifiConfig {
  char ssid[32];
  char passphase[32];
  char hostName[64];  
};

struct ThingSpeakConfig {
  long channel;
  char writeKey[17];
  char readKey[17];
  bool enabled;
  char fingerPrint[64];
};

struct MqttConfig {
  char host[64];
  int port;
  char fingerPrint[64];
  char id[32];
  char user[32];
  char pass[32];
  bool enabled;
};

struct I2cConfig {
  int scl;
  int sda;
  int powerPin;
};

struct SpiConfig {
  int mosi;
  int miso;
  int sclk;
  int cs;
};

struct McuConfig {
  I2cConfig i2c;
  SpiConfig spi;
  int led;
  int deepSleepTime;
  float voltageCC;
};

struct Config {
  McuConfig mcu;
  WifiConfig wfc;
  ThingSpeakConfig tsc;
  MqttConfig mqtt;
};

void config_load(Config *conf, FS fs = SPIFFS);
void config_diag(Config *conf);

void wifi_diag(Config *conf);
void thingspeak_diag(Config *conf);
void mqtt_diag(Config *conf);
void mcu_diag(Config *conf);

#endif

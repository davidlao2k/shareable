#include <FS.h>
#include <ArduinoJson.h>
#include "Configuration.h"

void safe_copy(char *dst, const char *src, size_t size) {
  if (src != NULL) {
    strlcpy(dst, src, size);
  }
}

void config_load(Config *conf, FS fs) {
  if (fs.begin()) {
    if (File file = fs.open("/config.json", "r")) {
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, file);
      if (error.code() == OK) {
        // Wifi
        strlcpy(conf->wfc.ssid, doc["wifi"]["ssid"] | "", sizeof(conf->wfc.ssid));
        strlcpy(conf->wfc.passphase, doc["wifi"]["passphase"] | "", sizeof(conf->wfc.passphase));
        strlcpy(conf->wfc.hostName, doc["wifi"]["hostName"] | "", sizeof(conf->wfc.hostName));
        
        // ThingSpeak
        conf->tsc.channel = doc["thingspeak"]["channel"] | 0;
        strlcpy(conf->tsc.writeKey, doc["thingspeak"]["writeKey"] | "", sizeof(conf->tsc.writeKey));
        strlcpy(conf->tsc.readKey, doc["thingspeak"]["readKey"] | "", sizeof(conf->tsc.readKey));
        //strlcpy(conf->tsc.writeKey, doc["thingspeak"]["writeKey"] | "", 1+sizeof(conf->tsc.writeKey));
        strlcpy(conf->mqtt.host, doc["mqtt"]["host"] | "", sizeof(conf->mqtt.host));
        conf->tsc.enabled = doc["thingspeak"]["enabled"] | false;
        safe_copy(conf->tsc.fingerPrint, doc["thingspeak"]["fingerPrint"], sizeof(conf->tsc.fingerPrint));
        
        // MQTT
        conf->mqtt.port = doc["mqtt"]["port"] | 1883;
        safe_copy(conf->mqtt.fingerPrint, doc["mqtt"]["fingerPrint"], sizeof(conf->mqtt.fingerPrint));
        strlcpy(conf->mqtt.id, doc["mqtt"]["id"] | "", sizeof(conf->mqtt.id));
        strlcpy(conf->mqtt.user, doc["mqtt"]["user"] | "", sizeof(conf->mqtt.user));
        strlcpy(conf->mqtt.pass, doc["mqtt"]["pass"] | "", sizeof(conf->mqtt.pass));
        conf->mqtt.enabled = doc["mqtt"]["enabled"] | false;
        
        // MCU
        conf->mcu.i2c.scl = doc["mcu"]["i2c"]["scl"] | 5;
        conf->mcu.i2c.sda = doc["mcu"]["i2c"]["sda"] | 4;
        conf->mcu.i2c.powerPin = doc["mcu"]["i2c"]["powerPin"] | 14;
        conf->mcu.spi.mosi = doc["mcu"]["spi"]["mosi"] | 13;
        conf->mcu.spi.miso = doc["mcu"]["spi"]["miso"] | 12;
        conf->mcu.spi.sclk = doc["mcu"]["spi"]["sclk"] | 14;
        conf->mcu.spi.cs = doc["mcu"]["spi"]["cs"] | 15;
        conf->mcu.led = doc["mcu"]["led"] | 2;
        conf->mcu.deepSleepTime = doc["mcu"]["deepSleepTime"] | 60;
        conf->mcu.voltageCC = doc["mcu"]["voltageCC"] | 1.0;
      }
      file.close();
    }
    fs.end();
  }
}

void config_diag(Config *conf) {
  wifi_diag(conf);
  thingspeak_diag(conf);
  mqtt_diag(conf);
  mcu_diag(conf);
}

void wifi_diag(Config *conf) {
  Serial.printf("Wifi: {ssid: %s, passphase: %s, hostName: %s}\n", conf->wfc.ssid, conf->wfc.passphase, conf->wfc.hostName);  
}

void thingspeak_diag(Config *conf) {
  Serial.printf("ThingSpeak: {channel: %d, writeKey: %s, readKey: %s}\n", conf->tsc.channel, conf->tsc.writeKey, conf->tsc.readKey);
}

void mqtt_diag(Config *conf) {
  Serial.printf("Mqtt: {host: %s, port: %d, id: %s, user: %s, pass: %s}\n", conf->mqtt.host, conf->mqtt.port, conf->mqtt.id, conf->mqtt.user, conf->mqtt.pass);
}

void mcu_diag(Config *conf) {
  Serial.printf("I2C: {scl: %d, sda: %d, powerPin: %d}\n", conf->mcu.i2c.scl, conf->mcu.i2c.sda, conf->mcu.i2c.powerPin);
  Serial.printf("SPI: {mosi: %d, miso: %d, sclk: %d, cs: %d}\n", conf->mcu.spi.mosi, conf->mcu.spi.miso, conf->mcu.spi.sclk, conf->mcu.spi.cs);
  Serial.printf("LED: %d\n", conf->mcu.led);
  Serial.printf("Vcc voltage calibration constant: %f\n", conf->mcu.voltageCC);
}

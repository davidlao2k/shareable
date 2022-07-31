#include <Wire.h>
#include <AHT_Sensor.h>
#include <Adafruit_BMP280.h>

#include <ThingSpeak.h>
#include <ESP8266WiFi.h>
#include <include/WiFiState.h>
#include <Configuration.h>

#define RTC_USER_DATA_SLOT_BOOT_STATE 126u
#define RTC_USER_DATA_SLOT_WIFI_STATE 33u

Config conf;
WiFiState wifiState;
WiFiClient client;

Adafruit_BMP280 bmp;
AHTSensor hum;

unsigned int bootCounter;
unsigned long startWatch;
unsigned long sleepTimeMiro;

bool hasTempSensor;
bool hasHumiditySensor;

void fetch_rtc_states() {
  delay(1);
  ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&wifiState), sizeof(wifiState));
  ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_BOOT_STATE, reinterpret_cast<uint32_t *>(&bootCounter), sizeof(bootCounter));
}

void connect_wifi() {
  unsigned long startTime = millis();

  if (!WiFi.resumeFromShutdown(wifiState) || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {
    Serial.println("Cannot resume WiFi connection from RTC state");
    WiFi.persistent(false);

    WiFi.mode(WIFI_STA);
    if (!WiFi.begin(conf.wfc.ssid, conf.wfc.passphase) || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {
      WiFi.mode(WIFI_OFF);
      Serial.printf("Failed to connect to %s with passphase %s, try again in %d seconds\n",
                    conf.wfc.ssid, conf.wfc.passphase, conf.mcu.deepSleepTime);
      deepsleep();
    }
  }

  unsigned long elapsed = millis() - startTime;
  Serial.printf("%s WiFi connected\nConnect took %.2f seconds\nIP address = ", conf.wfc.ssid, elapsed * 0.001);
  Serial.println(WiFi.localIP());
}

void deepsleep() {
  unsigned long elapsed = micros() - startWatch;
  unsigned long adjusted = elapsed > sleepTimeMiro ? elapsed : sleepTimeMiro - elapsed;

  ESP.deepSleep(adjusted, RF_DISABLED);
}

void hibernate() {
  WiFi.shutdown(wifiState);
  ESP.rtcUserMemoryWrite(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&wifiState), sizeof(wifiState));

  bootCounter++;
  ESP.rtcUserMemoryWrite(RTC_USER_DATA_SLOT_BOOT_STATE, reinterpret_cast<uint32_t *>(&bootCounter), sizeof(bootCounter));

  Serial.printf("Hibernate for %d seconds\n", conf.mcu.deepSleepTime);
  Serial.flush();

  deepsleep();
}

void setup() {
  startWatch = micros();
  config_load(&conf);
  sleepTimeMiro = conf.mcu.deepSleepTime * 1000 * 1000;

  // initialized GPIO
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  fetch_rtc_states();
  
  // connect Wifi
  connect_wifi();
  ThingSpeak.begin(client);
  
  // initialize I2C
  Wire.pins(conf.mcu.i2c.sda, conf.mcu.i2c.scl);
  Wire.begin(conf.mcu.i2c.sda, conf.mcu.i2c.scl);
  
  // initialize sensors
  if (hasTempSensor = bmp.begin()) {
    // Default settings from datasheet
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   // Operating Mode
                    Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                    Adafruit_BMP280::FILTER_X16,      // Filtering
                    Adafruit_BMP280::STANDBY_MS_500); // Standby time
  }
  hasHumiditySensor = hum.begin();

  if (hasTempSensor && hasHumiditySensor) {
    // read barometric sensor
    float temperature1 = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = bmp.readAltitude();

    // read humidity sensor
    float temperature2 = hum.getTemperature();
    float humidity = hum.getHumidity();
    float dewPoint = hum.getDewPoint();
    
    Serial.printf("Sensor 1: Temperature = %.2f°C, Pressure = %.2fPa, Altitude = %.2fm\n", temperature1, pressure, altitude);
    Serial.printf("Sensor 2: Temperature = %.2f°C, Humidity = %.0f%%, Dew Point = %.2f°C\n", temperature2, humidity, dewPoint);

    // write to ThingSpeak
    if (conf.tsc.enabled) {
      ThingSpeak.setField(1, temperature1);
      ThingSpeak.setField(2, pressure);
      ThingSpeak.setField(3, altitude);
      ThingSpeak.setField(4, temperature2);
      ThingSpeak.setField(5, humidity);
      ThingSpeak.setField(6, dewPoint);

      int ret = ThingSpeak.writeFields(conf.tsc.channel, conf.tsc.writeKey);
      if (ret == 200) {
        Serial.printf("Wrote to thinkspeed channel: %d\n", conf.tsc.channel);
      } else {
        Serial.printf("Problem writting to channel %d with key %s. Error code %d\n", conf.tsc.channel, conf.tsc.writeKey, ret);
      }
    }
  }

  hibernate();
}

void loop() {
  
}

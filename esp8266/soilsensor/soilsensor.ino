#include <Wire.h>
#include <Adafruit_BMP280.h>

#include <ThingSpeak.h>
#include <ESP8266WiFi.h>
#include <include/WiFiState.h>
#include <Configuration.h>

#define RTC_USER_DATA_SLOT_BOOT_STATE 126u
#define RTC_USER_DATA_SLOT_WIFI_STATE 33u

#define CONFIG_DEEPSLEEP_INTERVAL 15 * 60 * 1000 * 1000
#define CONFIG_POWER_PIN 14
#define CONFIG_SENSOR_PIN 0

#define CONFIG_AIR_VALUE 820
#define CONFIG_WATER_VALUE 480

Config conf;
WiFiState state;
WiFiClient client;
Adafruit_BMP280 bmp;

unsigned int bootState;
unsigned long startWatch;

bool hasTempSensor;

void increamentBootState() {
  // increament boot counter
  ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_BOOT_STATE, reinterpret_cast<uint32_t *>(&bootState), sizeof(bootState));
  Serial.println("\nBoot count = " + String(bootState));
  
  bootState++;
  ESP.rtcUserMemoryWrite(RTC_USER_DATA_SLOT_BOOT_STATE, reinterpret_cast<uint32_t *>(&bootState), sizeof(bootState));
}

void wifiConnect() {
  // May be necessary after deepSleep. Otherwise you may get "error: pll_cal exceeds 2ms!!!" when trying to connect
  delay(1);

  ESP.rtcUserMemoryRead(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));
  unsigned long start = millis();

  if (!WiFi.resumeFromShutdown(state)
      || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {
    //Serial.println("Cannot resume WiFi connection, connecting via begin...");
    WiFi.persistent(false);

    if (!WiFi.mode(WIFI_STA)
        || !WiFi.begin(conf.wfc.ssid, conf.wfc.passphase)
        || (WiFi.waitForConnectResult(10000) != WL_CONNECTED)) {
      WiFi.mode(WIFI_OFF);
      Serial.println("Cannot connect!");
      deepsleep();
    }
  }

  unsigned long duration = millis() - start;
  Serial.printf("Duration: %f", duration * 0.001);
  Serial.println();

  Serial.println("WiFi connected");
  Serial.print("IP address = ");
  Serial.println(WiFi.localIP());
}

void deepsleep() {
  unsigned long elapsed = micros() - startWatch;
  unsigned long adjusted = elapsed > CONFIG_DEEPSLEEP_INTERVAL ? elapsed : CONFIG_DEEPSLEEP_INTERVAL - elapsed;

  Serial.flush();
  ESP.deepSleep(adjusted, RF_DISABLED);
}

void setup() {
  startWatch = micros();
  config_load(&conf);
  
  // initialized GPIO
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CONFIG_POWER_PIN, OUTPUT);
  pinMode(CONFIG_SENSOR_PIN, INPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(CONFIG_POWER_PIN, HIGH);

  Serial.begin(115200);
  increamentBootState();

  wifiConnect();
  ThingSpeak.begin( client );

  // initialized I2C
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

  // read moisture sensor
  int sensorValue = analogRead(CONFIG_SENSOR_PIN);
  int adjusted = sensorValue > CONFIG_AIR_VALUE? CONFIG_AIR_VALUE : sensorValue < CONFIG_WATER_VALUE? CONFIG_WATER_VALUE : sensorValue;
  // compute moisture percentage
  float moisturePc = map(adjusted, CONFIG_AIR_VALUE, CONFIG_WATER_VALUE, 0, 100);
  Serial.println("Moisture reading = " + String(sensorValue) + " moisturePc = " + String(moisturePc));

  ThingSpeak.setField(1, sensorValue);
  ThingSpeak.setField(2, moisturePc);

  // read barometric sensor
  if (hasTempSensor) {
    float temperatureC = bmp.readTemperature();
    long pressure = bmp.readPressure();
    float altitude = bmp.readAltitude();
    Serial.println("Temperature = " + String(temperatureC) + " Pressure = " + String(pressure) + " Altitude = " + String(altitude));

    ThingSpeak.setField(3, temperatureC);
    ThingSpeak.setField(4, pressure);
    ThingSpeak.setField(5, altitude);
  }

  // write to ThingSpeak
  ThingSpeak.writeFields(conf.tsc.channel, conf.tsc.writeKey);

  WiFi.shutdown(state);
  ESP.rtcUserMemoryWrite(RTC_USER_DATA_SLOT_WIFI_STATE, reinterpret_cast<uint32_t *>(&state), sizeof(state));
  Serial.println("Done.");
  
  deepsleep();
}

void loop() {

}

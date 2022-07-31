#include <Wire.h>
#include <AHT_Sensor.h>

AHTSensor sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin(6, 4);
  if (sensor.begin())
    Serial.println("Init sensor Sucess.");
  else
    Serial.println("Init sensor Failure.");
}

void loop() {
  Serial.printf("Humidity: %.f %%\n", sensor.getHumidity());
  Serial.printf("Temperature: %.2f °C\n", sensor.getTemperature());
  Serial.printf("Dewpoint: %.2f °C\n", sensor.getDewPoint());
  Serial.printf("Moisture: %d\n", analogRead(0));
  delay(1000);
}

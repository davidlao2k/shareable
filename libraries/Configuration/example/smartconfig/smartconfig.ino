#include <ESP8266WiFi.h>
#include <Configuration.h>

Config conf;

void setup() {
  int cnt = 0;
  Serial.begin(74880);
  delay(10);

  config_load(&conf);
  config_diag(&conf);
  
  WiFi.mode(WIFI_STA);
  delay(500);
  WiFi.beginSmartConfig();
  while (1) {
    delay(1000);
    if (WiFi.smartConfigDone()) {
      Serial.println("SmartConfig Success");
      break;
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

int value = 0;

void loop() {

}

#include <SPI.h>
#include <WiFiEsp.h>

void setup(void){
  Serial.begin(115200);
  String fw = WiFi.firmwareVersion();
  Serial.print("WiFi firmware version: ");
  Serial.println(fw);
}

void loop(void){
}

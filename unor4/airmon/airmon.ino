#define DEVNAME "ARDUINOR4"
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include "ArduinoSecrets.h"
#include "Arduino_LED_Matrix.h"

//MqttClient client(wifi);
ArduinoLEDMatrix matrix;

static const int MQ4_PIN = A0;
static const int MQ9_PIN = A1;
static const int MQ6_PIN = A2;
static const int MQ135_PIN = A3;

struct sensor {
  int pin;
  const char* hw;
} sensors[] = {
  { MQ4_PIN, "MQ-4", },
  { MQ9_PIN, "MQ-9", },
  { MQ6_PIN, "MQ-6", },
  { MQ135_PIN, "MQ-135", }
};

void setup(){
  Serial.begin(115200);
  matrix.begin();
  matrix.loadFrame(LEDMATRIX_BOOTLOADER_ON);
  while(!Serial){
    ;
  }
  if(WiFi.status() == WL_NO_MODULE){
    while(true){
      Serial.println("couldn't find wifi module");
    }
  }
  Serial.print("WiFi firmware: ");
  Serial.println(WiFi.firmwareVersion());
  int status;
  do{
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(SSID);
    status = WiFi.begin(SSID, WPAPASS);
    if(status != WL_CONNECTED){
      Serial.println("waiting 10s to retry");
      delay(10000);
    }
  }while(status != WL_CONNECTED);
  analogReadResolution(14);
  for(unsigned i = 0 ; i < sizeof(sensors) / sizeof(*sensors) ; ++i){
    pinMode(sensors[i].pin, INPUT);
  }
}

void asample(const struct sensor* s){
  int a = analogRead(s->pin);
  printf("%s: %d\n", s->hw, a);
}

void loop(){
  matrix.loadFrame(LEDMATRIX_CLOUD_WIFI);
  for(unsigned i = 0 ; i < sizeof(sensors) / sizeof(*sensors) ; ++i){
    asample(&sensors[i]);
  }
  delay(1000);
}

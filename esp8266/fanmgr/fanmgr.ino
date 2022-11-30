// intended for use on a Hi-LetGo ESP8266, this manages a PWM fan and two
// PWM pumps, and two thermistors. it receives PWM control messages, and sends
// RPM and temperature reports, over MQTT. unlike the ESP32 version, this
// doesn't run LEDs, nor does it support a pressure sensor.
#define DEVNAME "esp8266"
#include "ESP8266WiFi.h"
#include <float.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoOTA.h>
#include "common.h"

// only one ADC on the ESP8266
const int TEMPPIN = A0; // coolant thermistor (2-wire)

// we keep the LED on when we're not connected. when we are connected, we blink it
// each time we transmit. LEDs are both inverted (there's another at D4).
const int LEDPIN = D0; // fixed for nodemcu
static volatile bool mqttconnected;

const int AMBIENTPIN = D2; // ambient temperature (digital thermometer, Dallas 1-wire)
const int PUMPPWMPIN = D5;
const int FANPWMPIN = D6;

// RPMs as reported to us over UART
static unsigned RPM;
static unsigned XTopRPMA;
static unsigned XTopRPMB;

static unsigned Pwm;
static unsigned PumpPwm;
static unsigned XTopPWMA;
static unsigned XTopPWMB;

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

void setup(){
  Serial.begin(115200);
  Serial.print("booting ");
  Serial.print(DEVNAME);
  Serial.print(" v");
  Serial.println(VERSION);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  pinMode(PUMPPWMPIN, OUTPUT);
  pinMode(FANPWMPIN, OUTPUT);
  analogWriteFreq(25000);
  set_pwm(INITIAL_FAN_PWM);
  set_pump_pwm(INITIAL_PUMP_PWM);
  pinMode(TEMPPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  ArduinoOTA.setHostname(DEVNAME);
  ArduinoOTA.setPassword(DEVNAME); // FIXME
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

// set up the desired PWM value
static int set_pwm(unsigned p){
  analogWrite(FANPWMPIN, p);
  Serial.print("configured fan PWM: ");
  Serial.println(p);
  Pwm = p;
  return 0;
}

// set up the desired pump PWM value
static int set_pump_pwm(unsigned p){
  analogWrite(PUMPPWMPIN, p);
  Serial.print("configured pump PWM: ");
  Serial.println(p);
  PumpPwm = p;
  return 0;
}

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
  digitalWrite(LEDPIN, HIGH);
  mqttconnected = true;
  client.subscribe("control/" DEVNAME "/pwm", [](const String &payload){
      Serial.print("received PWM via mqtt: ");
      Serial.println(payload);
      unsigned long p = 0;
      if(payload.length() == 0){
        Serial.println("empty PWM input");
        return;
      }
      for(int i = 0 ; i < payload.length() ; ++i){
        char h = payload.charAt(i);
        if(!isdigit(h)){
          Serial.println("invalid PWM character");
          return;
        }
        p *= 10; // FIXME check for overflow
        p += h - '0';
      }
      if(valid_pwm_p(p)){
        set_pwm(p);
      }
    }
  );
  client.subscribe("control/" DEVNAME "/pumppwm", [](const String &payload){
      Serial.print("received pump PWM via mqtt: ");
      Serial.println(payload);
      unsigned long p = 0;
      if(payload.length() == 0){
        Serial.println("empty PWM input");
        return;
      }
      for(int i = 0 ; i < payload.length() ; ++i){
        char h = payload.charAt(i);
        if(!isdigit(h)){
          Serial.println("invalid PWM character");
          return;
        }
        p *= 10; // FIXME check for overflow
        p += h - '0';
      }
      if(valid_pwm_p(p)){
        set_pump_pwm(p);
      }
    }
  );
}

// we transmit approximately every 15 seconds, sampling RPMs at this time.
// we continuously sample the temperature, and use the most recent valid read
// for transmit/display. there are several blocking calls (1-wire and MQTT)
// that can lengthen a given cycle.
void loop(){
  static bool ledstatus = true;
  static bool onewire_connected;
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  // these are the most recent valid reads (i.e. we don't reset to FLT_MAX
  // on error, but instead only on transmission).
  static float coolant_temp = FLT_MAX;
  static float ambient_temp = FLT_MAX;
  ArduinoOTA.handle();
  client.loop(); // handle any necessary wifi/mqtt
  readThermistor(&coolant_temp, TEMPPIN, 1024);
  if(!onewire_connected){
    if(connect_onewire(&digtemp) == 0){
      onewire_connected = true;
    }
  }
  if(onewire_connected){
    if(readAmbient(&ambient_temp, &digtemp)){
      onewire_connected = false;
    }
  }
  unsigned long m = micros();
  unsigned long diff = m - last_tx;
  if(diff < 15000000){
    return;
  }
  // blink for duration of transmit. if we're not connected, we're already
  // on, and this will have no effect.
  digitalWrite(LEDPIN, LOW);
  last_tx = m;
  Serial.print(diff);
  Serial.println(" µsec expired for cycle");
  bool success = true;
  success &= rpmPublish(client, "moraxtop0rpm", XTopRPMA);
  success &= rpmPublish(client, "moraxtop1rpm", XTopRPMB);
  success &= rpmPublish(client, "morarpm", RPM);
  success &= publish_temps(client, ambient_temp, coolant_temp);
  success &= publish_pwm(client, Pwm, PumpPwm);
  success &= publish_uptime(client, millis() / 1000); // FIXME handle overflow
  coolant_temp = FLT_MAX;
  ambient_temp = FLT_MAX;
  if(mqttconnected){
    if(!success){
      mqttconnected = false; // leave the LED on
    }else{
      digitalWrite(LEDPIN, HIGH);
    }
  }
}

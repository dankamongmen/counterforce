// intended for use on a Hi-LetGo ESP8266, this manages a PWM fan and two
// PWM pumps, and two thermistors. it receives PWM control messages, and sends
// temperature reports, over MQTT. unlike the ESP32 version, this
// doesn't run LEDs, nor does it support a pressure sensor.
#define DEVNAME "esp8266"
#include "ESP8266WiFi.h"
#include <float.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <ESP8266_PWM.h>
#include <DallasTemperature.h>
#include <user_interface.h>
#include <ESP8266mDNS.h>
#include "common.h"

const int TEMPPIN = A0; // coolant thermistor (2-wire), only one ADC on the ESP8266
const int AMBIENTPIN = D2; // ambient temperature (digital thermometer, Dallas 1-wire)
const int PWMPIN = D4;

// we keep the LED on when we're not connected. when we are connected, we blink it
// each time we transmit. LEDs are both inverted (there's another at D4).
const int LEDPIN = D0; // fixed for nodemcu
static volatile bool mqttconnected;

// PWM as ordered from us over MQTT (we only have one)
static unsigned Pwm = INITIAL_FAN_PWM;

EspMQTTClient client(
  #include "EspMQTTConfig.h",
  DEVNAME
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

void setup(){
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.print("booting ");
  Serial.print(DEVNAME);
  Serial.print(" v");
  Serial.println(VERSION);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  client.enableHTTPWebUpdater();
  pinMode(TEMPPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(PWMPIN, OUTPUT);
  set_pwm(INITIAL_FAN_PWM);
  if(!MDNS.begin("Replace with unique_device_name")){
    Serial.println("Error setting up MDNS responder!");
    while(1){
      delay(1000);
    }
  }
  digitalWrite(LEDPIN, LOW);
}

// set up the desired PWM value
static int set_pwm(unsigned p){
  Serial.print("configured fan PWM: ");
  Serial.println(p);
  analogWrite(PWMPIN, p);
  Pwm = p;
  return 0;
}

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
  digitalWrite(LEDPIN, HIGH);
  mqttconnected = true;
}

static void publish_unified_pwm(mqttmsg& mmsg, int pwm){
  if(valid_pwm_p(pwm)){
    mmsg.add("pwm", pwm);
  }
}

// we transmit approximately every 15 seconds.
// we continuously sample the temperature, and use the most recent valid read
// for transmit/display. there are several blocking calls (1-wire and MQTT)
// that can lengthen a given cycle.
void loop(){
  static bool ledstatus = true;
  static bool onewire_connected;
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  // these are the most recent valid reads (i.e. we don't reset to FLT_MAX
  // on error, but instead only on transmission).
  float ambient_temp = NAN;
  client.loop(); // handle any necessary wifi/mqtt
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
  if(last_tx){
    unsigned long diff = m - last_tx;
    if(diff < 15000000){
      return;
    }
  }
  // blink for duration of transmit. if we're not connected, we're already
  // on, and this will have no effect.
  digitalWrite(LEDPIN, LOW);
  bool success = true;
  
  mqttmsg mmsg(client);
  float coolant_temp = NAN;
  readThermistor(&coolant_temp, TEMPPIN, 1024);
  publish_temps(mmsg, ambient_temp, coolant_temp);
  publish_unified_pwm(mmsg, Pwm);
  if(mmsg.publish()){
    Serial.print("Successful xmit at ");
    Serial.println(m);
    last_tx = m;
  }
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

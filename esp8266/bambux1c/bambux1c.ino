// for use with a HiLetgo ESP8266 NodeMCUv2
// http://www.hiletgo.com/ProductDetail/1906570.html
//
// sits in a bambux1c and controls:
//  a ceramic heating element using a 3V relay
//  a 12V fan for the heating element (tach + PWM)
//  a CCS811 VOC sensor
//  a 12V fan for the BentoBox VOC fans (tach + PWM)
//
// and reports stats via wireless MQTT
#define DEVNAME "bambumanager"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = D0;

// the fan(s) for the heater
static const int HEATFANPWMPIN = 37;
static const int HEATFANTACHPIN = 24;

// the fan(s) in the bento box
static const int VOCFANPWMPIN = 32;
static const int VOCFANTACHPIN = 33;

// SCL is pin 22, SDA is pin 21
static const int I2C_SCL = SCL;
static const int I2C_SDA = SDA;

#include <Wire.h>
#include <OneWire.h>
#include <ESP8266WiFi.h>
#include <SparkFunCCS811.h>
#include <DallasTemperature.h>
#include <ESP8266MQTTClient.h>
#include "EspMQTTConfig.h"
#include "espcommon.h"

MQTTClient client;

#define CCS811_ADDR 0x5B // i2c address
CCS811 ccs811(CCS811_ADDR);

int mqtt_setup(MQTTClient& mqtt){
  mqtt.begin("mqtt://" MQTTUSER ":" MQTTPASS "@mqtt.orthanc");
  WiFi.begin(WIFIESSID, WIFIPASS);
  return 0;
}

void setup(void){
  Serial.begin(115200);
  Serial.println("initializing!");
  //setCpuFrequencyMhz(80);
  /*
  initialize_25k_pwm(HEATFANCHAN, HEATFANPWMPIN, LEDC_TIMER_1);
  initialize_25k_pwm(VOCFANCHAN, VOCFANPWMPIN, LEDC_TIMER_2);
  FanPwm = 0;
  set_pwm(HEATFANCHAN, FanPwm);
  set_pwm(VOCFANCHAN, FanPwm);
  init_tach(HEATFANTACHPIN, rpm_fan);
  init_tach(VOCFANTACHPIN, rpm_pumpa); // FIXME no pumps
  nvs_setup(&Nvs);
  */
  mqtt_setup(client);
  /*
  printf("Fan PWM initialized to %u\n", FanPwm);
  while(!ccs811.begin()){
    Serial.println("failure initializing CCS811");
  }
  */
  Serial.println("initialized!");
}

void loop(void){
  unsigned long m = micros();
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  unsigned long diff = m - last_tx;
  if(last_tx){
    if(diff < 15000000){
      return;
    }
  }
  float ambient = getAmbient();
  // FIXME publish
}

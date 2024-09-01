// for use with an ESP32-WROOM
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
static const int AMBIENTPIN = 34;

// the fan(s) for the heater
static const int HEATFANPWMPIN = 4;
static const int HEATFANTACHPIN = 35;

// the fan(s) in the bento box
static const int VOCFANPWMPIN = 2;
static const int VOCFANTACHPIN = 32;

// SCL is pin 22, SDA is pin 21
static const int I2C_SCL = SCL;
static const int I2C_SDA = SDA;

static const int LEDPIN = 2;

#include <Wire.h>
#include <OneWire.h>
#include <SparkFunCCS811.h>
#include <DallasTemperature.h>
#include "EspMQTTConfig.h"
#include "common.h"

CCS811 ccs811(0x5A);

void setup(void){
  Serial.begin(115200);
  Serial.println("initializing!");
  //setCpuFrequencyMhz(80);
  initialize_25k_pwm(FANCHAN, HEATFANPWMPIN, LEDC_TIMER_1);
  initialize_25k_pwm(PUMPACHAN, VOCFANPWMPIN, LEDC_TIMER_2);
  set_pwm(FANCHAN, FanPwm);
  set_pwm(PUMPACHAN, PumpPwm);
  init_tach(HEATFANTACHPIN, rpm_fan);
  init_tach(VOCFANTACHPIN, rpm_pumpa);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  nvs_setup(&Nvs);
  mqtt_setup(client);
  printf("Fan PWM initialized to %u\n", FanPwm);
  printf("Pump PWM initialized to %u\n", PumpPwm);
  Serial.println("initialized!");
}

void loop(void){
  static bool gotccs = false;
  
  if(!gotccs){
    if(ccs811.begin()){
      Serial.println("initialized CCS811");
      gotccs = true;
    }else{
      Serial.println("failure initializing CCS811");
    }
  }
  //client.handle();
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

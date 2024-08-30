// for use with a LilyGo TTGO T-Display ESP32
// https://www.lilygo.cc/products/lilygo%C2%AE-ttgo-t-display-1-14-inch-lcd-esp32-control-board
//
// sits in a bambux1c and controls:
//  a ceramic heating element using a 3V relay
//  a 12V fan for the heating element (tach + PWM)
//  a CCS811 VOC sensor
//  a 12V fan for the BentoBox VOC fans (tach + PWM)
//
// and reports stats via wireless MQTT
#define DEVNAME "bambumanager"

#include "driver/ledc.h"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 25;

// the fan(s) for the heater
static const int HEATFANPWMPIN = 37;
static const int HEATFANTACHPIN = 38;
static const ledc_channel_t HEATFANCHAN = LEDC_CHANNEL_0;

// the fan(s) in the bento box
static const int VOCFANPWMPIN = 32;
static const int VOCFANTACHPIN = 33;
static const ledc_channel_t VOCFANCHAN = LEDC_CHANNEL_1;

// SCL is pin 22, SDA is pin 21
static const int I2C_SCL = SCL;
static const int I2C_SDA = SDA;

#include "common.h"
#include <SparkFunCCS811.h>
#include <driver/ledc.h>

void setup(void){
  Serial.begin(115200);
  Serial.println("initializing!");
  //setCpuFrequencyMhz(80);
  initialize_25k_pwm(HEATFANCHAN, HEATFANPWMPIN, LEDC_TIMER_1);
  initialize_25k_pwm(VOCFANCHAN, VOCFANPWMPIN, LEDC_TIMER_2);
  set_pwm(HEATFANCHAN, FanPwm);
  set_pwm(VOCFANCHAN, PumpPwm);
  init_tach(HEATFANTACHPIN, rpm_fan);
  init_tach(VOCFANTACHPIN, rpm_pumpa);
  nvs_setup(&Nvs);
  mqtt_setup(client);
  printf("Fan PWM initialized to %u\n", FanPwm);
  printf("Pump PWM initialized to %u\n", PumpPwm);
  Serial.println("initialized!");
}

void loop(void){
}

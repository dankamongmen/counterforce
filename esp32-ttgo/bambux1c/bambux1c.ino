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

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 25;

// the fan(s) for the heater
static const int HEATFANPWMPIN = 37;
static const int HEATFANTACHPIN = 38;

// the fan(s) in the bento box
static const int VOCFANPWMPIN = 32;
static const int VOCFANTACHPIN = 33;

// SCL is pin 22, SDA is pin 21
static const int I2C_SCL = SCL;
static const int I2C_SDA = SDA;

#include "common.h"

void setup(void){
}

void loop(void){
}

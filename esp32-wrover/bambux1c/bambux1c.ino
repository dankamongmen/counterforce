// for use with an ESP32-WROOM
//
// sits in a bambux1c and controls:
//  a ceramic heating element using a 3V relay
//  a 12V fan for the heating element (tach + PWM)
//  a CCS811 VOC sensor
//  a 12V fan for the BentoBox VOC fans (tach + PWM)
//
// and reports stats via wireless MQTT
#define VERSION "0.0.1"
#define DEVNAME "bambumanager"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 16;

// the fan(s) for the heater
static const int HEATFANPWMPIN = 15;
static const int HEATFANTACHPIN = 35;

// the fan(s) in the bento box
static const int VOCFANPWMPIN = 2;
static const int VOCFANTACHPIN = 32;

// SCL is pin 22, SDA is pin 21
static const int I2C_SCL = SCL;
static const int I2C_SDA = SDA;

static const int LEDPIN = 2;

#include <SparkFunCCS811.h>
#include "EspMQTTConfig.h"
#include "espcommon.h"

CCS811 ccs811(-1);

void onMqttConnect(esp_mqtt_client_handle_t cli){
  Serial.println("got an MQTT connection");
  // FIXME subscribe to control channels
}

void setup(void){
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  printf("initializing\n");
  mqtt_setup(client);
  printf("initialized!\n");
  digitalWrite(LEDPIN, HIGH);
}

void bambumanager_loop(int ledpin){
  static bool gotccs = false;
  
  if(client.isConnected()){
    digitalWrite(ledpin, LOW);
  }else{
    digitalWrite(ledpin, HIGH);
  }
  float ambient = getAmbient();
  if(!gotccs){
    if(ccs811.begin()){
      printf("initialized CCS811\n");
      gotccs = true;
    }else{
      printf("failure initializing ccs811\n");
    }
  }
  if(ccs811.dataAvailable()){
    printf("got ccs data!\n");
  }
  unsigned long m = micros();
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  unsigned long diff = m - last_tx;
  if(last_tx){
    if(diff < 15000000){
      return;
    }
  }
  last_tx = micros();
  mqttmsg mmsg(client);
  publish_version(mmsg);
  publish_temps(mmsg, ambient);
  // go high for the duration of the transmit. we'll go low again when we
  // reenter fanmgrLoop() at the top, assuming we're connected.
  digitalWrite(ledpin, HIGH);
  if(mmsg.publish()){
    printf("successful xmit at %lu\n", m);
  }
}

void loop(void){
  bambumanager_loop(LEDPIN);
}

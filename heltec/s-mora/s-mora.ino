#define DEVNAME "demo"
//#define DEVNAME "s-mora"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 13;

static const int FANPWMPIN = 2;
static const int PUMPAPWMPIN = 17;
static const int PUMPBPWMPIN = 0;

static const int FANTACHPIN = 36;
static const int PUMPATACHPIN = 37;
static const int PUMPBTACHPIN = 38;

#include "helteccommon.h"

static const int LEDPIN = 25;

void setup(){
  heltecSetup(LEDPIN);
}

void loop(void){
  float ambient = sampleSensors(LEDPIN);
  heltecLoop(ambient);
}

#define DEVNAME "demo"
//#define DEVNAME "s-mora"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 12;

static const int FANPWMPIN = 14;
static const int FANTACHPIN = 27;

static const int PUMPAPWMPIN = 33;
static const int PUMPATACHPIN = 32;

static const int PUMPBPWMPIN = 35;
static const int PUMPBTACHPIN = 34;

#include "helteccommon.h"

static const int LEDPIN = 25;

void setup(){
  heltecSetup(LEDPIN);
}

void loop(void){
  float ambient = sampleSensors(LEDPIN);
  heltecLoop(ambient);
}

#define DEVNAME "s-mora"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 27;

static const int FANPWMPIN = 26;
static const int FANTACHPIN = TX2;

static const int PUMPAPWMPIN = 12;
static const int PUMPATACHPIN = 18;

static const int PUMPBPWMPIN = 32;
static const int PUMPBTACHPIN = RX2;

#include "common.h"

static const int LEDPIN = 2;

void setup(void){
  fanmgrSetup(LEDPIN);
}

void loop(void){
  float ambient = sampleSensors(LEDPIN);
  fanmgrLoop(ambient);
}

#define DEVNAME "r-mora"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 27;

static const int FANPWMPIN = 32;
static const int FANTACHPIN = 19;

static const int PUMPAPWMPIN = 2;
static const int PUMPATACHPIN = 4;

static const int PUMPBPWMPIN = 12;
static const int PUMPBTACHPIN = 23;

#include "common.h"

static const int LEDPIN = 2;

void setup(void){
  fanmgrSetup(LEDPIN);
}

void loop(void){
  fanmgrLoop(LEDPIN);
}

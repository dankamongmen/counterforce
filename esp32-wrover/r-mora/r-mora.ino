#define DEVNAME "r-mora"

#include <float.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <DallasTemperature.h>

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 27;

static const int FANPWMPIN = 32;
static const int FANTACHPIN = 19;

static const int PUMPAPWMPIN = 2;
static const int PUMPATACHPIN = 4;

static const int PUMPBPWMPIN = 12;
static const int PUMPBTACHPIN = 23;

#include "common.h"

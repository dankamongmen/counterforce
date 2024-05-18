#define DEVNAME "s-mora"

#include <float.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <DallasTemperature.h>

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 27;

static const int FANPWMPIN = 26;
static const int FANTACHPIN = TX2;

static const int PUMPAPWMPIN = 12;
static const int PUMPATACHPIN = 18;

static const int PUMPBPWMPIN = 32;
static const int PUMPBTACHPIN = RX2;

#include "common.h"

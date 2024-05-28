#define DEVNAME "r-mora"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// constraints for BOOT side
//  D15 (must be HIGH on boot)
//  D2 (must be LOW on boot, controls LED)
//  D16 (RX2)
//  D17 (TX2)
//  D5 (must be HIGH on boot)
//  D21 (SDA)
//  D3 (RX0)
//  D1 (TX0)
//  D22 (SCL)

// constraints for EN side
//  D12 (must be LOW on boot)
//  D35 (input only)
//  D34 (input only)
//  D39 (input only)
//  D36 (input only)

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 15; // also fucking up D2

static const int FANPWMPIN = 23;
static const int FANTACHPIN = 22;

static const int PUMPAPWMPIN = 19;
static const int PUMPATACHPIN = 18;

static const int PUMPBPWMPIN = 16; // also RX2
static const int PUMPBTACHPIN = 4;

static const int I2C_SCL = 32;
static const int I2C_SDA = 33;

#include "common.h"

static const int LEDPIN = 2;

void setup(void){
  displaySetup(I2C_SCL, I2C_SDA);
  fanmgrSetup(LEDPIN);
}

void loop(void){
  float ambient = sampleSensors();
  fanmgrLoop(LEDPIN, ambient);
}

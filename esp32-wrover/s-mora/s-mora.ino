//#define DEVNAME "s-mora"
#define DEVNAME "demo"
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
static const int AMBIENTPIN = 4;

static const int FANPWMPIN = 23;
static const int FANTACHPIN = 34;

static const int PUMPAPWMPIN = 13;
static const int PUMPATACHPIN = 35;

static const int PUMPBPWMPIN = 27;
static const int PUMPBTACHPIN = 32;

#include "common.h"

static const int LEDPIN = 2;

void setup(void){
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  displaySetup();
  fanmgrSetup(LEDPIN);
}

void loop(void){
  float ambient = sampleSensors();
  fanmgrLoop(LEDPIN, ambient);
}

// common routines for fanmgr on the heltec esp32 w/ integreated 128x64 display
#include <Wire.h>
#include <HT_SSD1306Wire.h>
#include "common.h"

// addr, freq, i2c group, resolution, rst
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

#define FANMGRSTR DEVNAME " " VERSION

// height of text, taken from font
#define THEIGHT 10

static void
maketempstr(char* buf, float ambient){
  if(isnan(ambient)){
    strcpy(buf, "no thermal");
  }else{
    sprintf(buf, "%0.2fC", ambient);
  }
}

static void
drawNum(SSD1306Wire* d, int x, int y, int num){
  char tstr[10];
  if(num >= 0){
    snprintf(tstr, sizeof(tstr), "%d", num);
  }else{
    snprintf(tstr, sizeof(tstr), "n/a");
  }
  d->drawString(x, y, tstr);
}

static void
drawPwm(SSD1306Wire* d, int x, int y, int pwm){
  float f = pwm * 100.0 / 255.0;
  char tstr[10];
  snprintf(tstr, sizeof(tstr), "%0.2f", f);
  d->drawString(x, y, tstr);
}

// if pwm is positive, but rpm is 0, that's a no-signal; set rpm to -1
static inline void
normalizeRPM(int* rpm, int pwm){
  if(!*rpm){
    if(pwm){
      *rpm = -1;
    }
  }
}

// update the display with current samples
static void
updateDisplay(float ambient, int fanrpm, int pumparpm, int pumpbrpm,
              int fanpwm, int pumppwm){
  display.clear();
  char tempstr[16];
  maketimestr(tempstr);
  display.drawString(0, 0, tempstr);
  maketempstr(tempstr, ambient);
  display.drawString(2, display.getHeight() - THEIGHT, tempstr);
  display.drawRect(2, 11, display.getWidth() - 6, 45);
  display.drawString(60, 10, "pwm(%)");
  display.drawString(102, 10, "rpm");
  display.drawString(4, 20, "fans");
  display.drawString(4, 30, "pump a");
  display.drawString(4, 40, "pump b");
  drawPwm(&display, 67, 20, fanpwm);
  drawPwm(&display, 67, 34, pumppwm);
  normalizeRPM(&fanrpm, fanpwm);
  drawNum(&display, 100, 20, fanrpm);
  normalizeRPM(&pumparpm, pumppwm);
  drawNum(&display, 100, 30, pumparpm);
  normalizeRPM(&pumpbrpm, pumppwm);
  drawNum(&display, 100, 40, pumpbrpm);
  display.drawString(display.getWidth() - 60, display.getHeight() - THEIGHT, FANMGRSTR);
  display.display();
}

static int
heltecSetup(int ledpin){
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  fanmgrSetup(ledpin);
  display.init();
  display.setContrast(255);
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.display();
  return 0;
}

static int
heltecLoop(int ledpin, float ambient){
  fanmgrLoop(ledpin, ambient);
  updateDisplay(ambient, FanRpm, PumpARpm, PumpBRpm, FanPwm, PumpPwm);
  return 0;
}

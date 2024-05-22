// common routines for fanmgr on the heltec esp32 w/ integreated 128x64 display
#include <Wire.h>
#include <HT_SSD1306Wire.h>
#include "common.h"

// addr, freq, i2c group, resolution, rst
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

#define FANMGRSTR "moradank "VERSION
#define FANMGRSTRWIDTH 70 // determined experimentally

// height of text, taken from font
#define THEIGHT 10

static void
maketempstr(char* buf, float ambient){
  sprintf(buf, "%0.2fC", ambient);
}

static void
drawNum(SSD1306Wire* d, int x, int y, int num){
  char tstr[10];
  snprintf(tstr, sizeof(tstr), "%d", num);
  d->drawString(x, y, tstr);
}

// update the display with current samples
static void
updateDisplay(float ambient, int fanrpm, int pumparpm, int pumpbrpm,
              int fanpwm, int pumppwm){
  display.clear();
  char tempstr[16];
  maketempstr(tempstr, ambient);
  display.drawString(0, 0, tempstr);
  display.drawString(40, 11, "pwm");
  display.drawString(80, 11, "rpm");
  display.drawString(0, 22, "fans");
  display.drawString(0, 33, "pump a");
  display.drawString(0, 44, "pump b");
  drawNum(&display, 40, 22, fanpwm);
  drawNum(&display, 40, 38, pumppwm);
  drawNum(&display, 80, 22, fanrpm);
  drawNum(&display, 80, 33, pumparpm);
  drawNum(&display, 80, 44, pumpbrpm);
  display.drawString(display.getWidth() - FANMGRSTRWIDTH, display.getHeight() - 10, FANMGRSTR);
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
heltecLoop(int ledpin){
  fanmgrLoop(ledpin);
  updateDisplay(20.0, FanRpm, PumpARpm, PumpBRpm, FanPwm, PumpPwm);
  return 0;
}

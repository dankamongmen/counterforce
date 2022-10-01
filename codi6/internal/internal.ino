#include <FastLED.h>
 
CRGB flt[15];      // Quantum FLT
CRGB lianli[37];   // Lian Li LAN2-2X strips
CRGB phanteks[38]; // Phanteks NEON strips
 
void setup() {
  FastLED.addLeds<NEOPIXEL, 3>(flt, 0, sizeof(flt) / sizeof(*flt));
  FastLED.addLeds<NEOPIXEL, 6>(lianli, 0, sizeof(lianli) / sizeof(*lianli));
  FastLED.addLeds<NEOPIXEL, 9>(phanteks, 0, sizeof(phanteks) / sizeof(*phanteks));
}

// largely ripped from Effects/Hypnotoad/Hypnotoad.cpp in OpenRGBEffectsPlugin (GPL2)
#define animation_speed 5.0
#define color_rotation_speed 10.0
#define spacing 2.0
#define thickness 2.0
#define Speed 60
#define FPS 60

double progress = 1000;
unsigned cx_shift = 50;
unsigned cy_shift = 50;

static void GetColor(unsigned x, unsigned y, float cx, float cy, CRGB* leds){
  float animation_mult = 0.01 * animation_speed;
  float color_mult = 0.01 * color_rotation_speed;

  double angle    = atan2(y - cy, x - cx) * 180 / 3.14159265359;
  double distance = sqrt(pow(cx - x, 2) + pow(cy - y, 2));
  float  value    = cos(animation_mult * distance / (0.1 * (float) spacing)  + progress);

  int hue = abs((int)(angle + distance + progress * color_mult * color_rotation_speed) % 360);
  //hue = (hue % 60) + 90; // 90--150; we only want greens
  CHSV hsv =
    CHSV(hue, 255, pow((value + 1) * 0.5, (11 - thickness)) * 255);

  hsv2rgb_rainbow(hsv, leds[x]);
}

static void StepEffect(CRGB* leds, unsigned ledcount){
  float cx_shift_mult = cx_shift / 100.f;
  float cy_shift_mult = cy_shift / 100.f;
  unsigned width = ledcount;
  unsigned int height = 1;

  float cx = (width-1) * cx_shift_mult;
  float cy = height * cy_shift_mult;

  for(unsigned int i = 0; i < width; i++){
    GetColor(i, 0, cx, cy, leds);
  }
  progress +=  0.1 * (float) Speed / (float) FPS;
}

void loop() {
  for(int i = 0 ; i < sizeof(flt) / sizeof(*flt) ; ++i){
    flt[i] = CRGB::Cyan;
  }
  StepEffect(lianli, sizeof(lianli) / sizeof(*lianli));
  StepEffect(phanteks, sizeof(phanteks) / sizeof(*phanteks));
  FastLED.show();
}

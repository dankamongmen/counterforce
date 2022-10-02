#include <FastLED.h>
 
CRGB lianliA[40];   // Lian Li LAN2-2X strips 
CRGB lianliB[40];   // Lian Li Infinity fan
CRGB lianliC[40];   // Lian Li LAN2-2X strips
CRGB phanteksA[38]; // Phanteks NEON strips
CRGB phanteksB[38]; // Phanteks NEON strips
CRGB phanteksC[38]; // Phanteks NEON strips
 
void setup() {
  FastLED.addLeds<NEOPIXEL, 3>(lianliA, 0, sizeof(lianliA) / sizeof(*lianliA));
  FastLED.addLeds<NEOPIXEL, 5>(lianliB, 0, sizeof(lianliB) / sizeof(*lianliB));
  FastLED.addLeds<NEOPIXEL, 6>(lianliC, 0, sizeof(lianliC) / sizeof(*lianliC));
  FastLED.addLeds<NEOPIXEL, 9>(phanteksA, 0, sizeof(phanteksA) / sizeof(*phanteksA));
  FastLED.addLeds<NEOPIXEL, 10>(phanteksB, 0, sizeof(phanteksB) / sizeof(*phanteksB));
  FastLED.addLeds<NEOPIXEL, 11>(phanteksC, 0, sizeof(phanteksC) / sizeof(*phanteksC));
}

// ripped from Effects/Hypnotoad/Hypnotoad.cpp in OpenRGBEffectsPlugin (GPL2)
#define animation_speed 10.0
#define color_rotation_speed 10.0
#define spacing 1.0
#define thickness 2.0
#define Speed 50
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
  //hue = (hue % 60) + 90;
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
  StepEffect(lianliA, sizeof(lianliA) / sizeof(*lianliA));
  StepEffect(phanteksA, sizeof(phanteksA) / sizeof(*phanteksA));
  StepEffect(lianliB, sizeof(lianliB) / sizeof(*lianliB));
  StepEffect(phanteksB, sizeof(phanteksB) / sizeof(*phanteksB));
  StepEffect(lianliC, sizeof(lianliC) / sizeof(*lianliC));
  StepEffect(phanteksC, sizeof(phanteksC) / sizeof(*phanteksC));
  FastLED.show();
}

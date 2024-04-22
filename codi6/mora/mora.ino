#include <FastLED.h>
 
#define FANS_PER_CHAN 3
#define LEDS_PER_FAN 12 // Arctic P14 ARGB
#define LEDS_PER_PWM (FANS_PER_CHAN * LEDS_PER_FAN)
#define PWM_CHANNELS 6
CRGB p14[PWM_CHANNELS * LEDS_PER_PWM];
 
void setup() {
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, 3>(p14 + LEDS_PER_PWM * 0, 0, sizeof(CRGB) * LEDS_PER_PWM);
  FastLED.addLeds<NEOPIXEL, 5>(p14 + LEDS_PER_PWM * 1, 0, sizeof(CRGB) * LEDS_PER_PWM);
  FastLED.addLeds<NEOPIXEL, 6>(p14 + LEDS_PER_PWM * 2, 0, sizeof(CRGB) * LEDS_PER_PWM);
  FastLED.addLeds<NEOPIXEL, 9>(p14 + LEDS_PER_PWM * 3, 0, sizeof(CRGB) * LEDS_PER_PWM);
  FastLED.addLeds<NEOPIXEL, 10>(p14 + LEDS_PER_PWM * 4, 0, sizeof(CRGB) * LEDS_PER_PWM);
  FastLED.addLeds<NEOPIXEL, 11>(p14 + LEDS_PER_PWM * 5, 0, sizeof(CRGB) * LEDS_PER_PWM);
  Serial.println("MoRa3 CODI6 online");
  for(unsigned i = 0 ; i < sizeof(p14) / sizeof(*p14) ; ++i){
    p14[i] = (i % 2) ? CRGB::Cyan : CRGB::Green;
  }
  FastLED.show();
  Serial.println("MoRa3 CODI6 visible");
}

// ripped from Effects/Hypnotoad/Hypnotoad.cpp in OpenRGBEffectsPlugin (GPL2)
#define animation_speed 4.0
#define color_rotation_speed 3.0
#define spacing 1.0
#define thickness 3.0

double progress = 10;
unsigned cx_shift = 50;
unsigned cy_shift = 50;

static void GetColor(unsigned x, unsigned y, float cx, float cy, CRGB* leds){
  float animation_mult = 0.01 * animation_speed;
  float color_mult = 0.01 * color_rotation_speed;

  /*
  Serial.print("y: ");
  Serial.print(y);
  Serial.print(" x: ");
  Serial.print(x);
  Serial.print(" cx: ");
  Serial.print(cx);
  Serial.print(" cy: ");
  Serial.println(cy);
  */
  double angle    = atan2(y - cy, x - cx) * 180 / 3.14159265359;
  double distance = sqrt(pow(cx - x, 2) + pow(cy - y, 2));
  float  value    = cos(animation_mult * distance / (0.1 * (float) spacing) + progress);

  int hue = abs((int)(angle + distance + progress * color_mult * color_rotation_speed) % 360);
  // 90--150; we only want greens
  hue = (hue % 60) + 90;
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
  progress += 0.05;
}

void loop() {
  for(int p = 0 ; p < PWM_CHANNELS / 2 ; ++p){
    //for(int i = 0 ; i < FANS_PER_CHAN ; ++i){
      StepEffect(p14 + LEDS_PER_PWM * p /*+ LEDS_PER_FAN * i*/, LEDS_PER_PWM);
      StepEffect(p14 + LEDS_PER_PWM * (p + PWM_CHANNELS / 2) /*+ LEDS_PER_FAN * i*/, LEDS_PER_PWM);
    //}
  }
  FastLED.show();
}

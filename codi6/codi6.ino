#include <FastLED.h>
 
CRGB flt[15];    // Quantum FLT
CRGB strips[60]; // 2 EZDIY strips of 30 LEDs each, mobo side
 
void setup() {
  FastLED.addLeds<NEOPIXEL, 3>(flt, 0, sizeof(flt) / sizeof(*flt));
  FastLED.addLeds<NEOPIXEL, 6>(strips, 0, sizeof(strips) / sizeof(*strips));
}

void loop() {
  for(int i = 0 ; i < sizeof(flt) / sizeof(*flt) ; ++i){
    flt[i] = CRGB::Cyan;
  }
  for(int i = 0 ; i < sizeof(strips) / sizeof(*strips) ; ++i){
    strips[i] = CRGB::DarkGreen;
  }
  FastLED.show();
}

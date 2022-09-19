#include <FastLED.h>
#define NUM_STRIPS 3
#define NUM_LEDS_PER_STRIP 20
#define NUM_LEDS NUM_LEDS_PER_STRIP * NUM_STRIPS
 
CRGB leds[NUM_LEDS];
 
// For mirroring strips, all the "special" stuff happens just in setup.  We
// just addLeds multiple times, once for each strip
void setup() {
  // tell FastLED there's 15  NEOPIXEL leds on pin 3, starting at index 0 in the led array
  FastLED.addLeds<NEOPIXEL, 3>(leds, 0, NUM_LEDS_PER_STRIP);
  FastLED.addLeds<NEOPIXEL, 6>(leds, 0, NUM_LEDS_PER_STRIP);
  FastLED.addLeds<NEOPIXEL, 9>(leds, 0, NUM_LEDS_PER_STRIP);
}

void loop() {
  for(int i = 0 ; i < NUM_LEDS ; ++i){
    leds[i] = CRGB::Cyan;
  }
  FastLED.show();
}

#include <FastLED.h>

#define DATA_PIN 8

DEFINE_GRADIENT_PALETTE(hypno) {
0,		128,		0,		128,		// light green
85,		255,		0,		255,		// medium teal
171,	0,		255,		255,		// medium teal
255,	0,		128,		128 };	// full green

// on 12V 720 LEDs/m, each WS2811 controls 36 LEDs, and thus there are
// 20 WS2811s per meter. Ours is a four meter strip.
#define NUM_LEDS 4 * 20

static CRGB leds[NUM_LEDS];
static CRGBPalette16 p = hypno;

void setup() { FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS); }

void loop() {
	for(unsigned heat = 0 ; heat < 256 ; ++heat){
		for(int i = 0 ; i < NUM_LEDS ; ++i){
			leds[i] = ColorFromPalette(p, heat);
		}
		delay(10 * heat);
		FastLED.show();
	}
}

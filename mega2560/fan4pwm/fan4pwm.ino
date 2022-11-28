#include <FastLED.h>
#include <SoftwareSerial.h>
// requires an Arduino Mega2560, possibly rev3, possibly only the authentic one.

const unsigned long SERIALSPEED = 115200;
const unsigned long UARTSPEED = 9600;
// we'll use two pins for UART communication with the ESP32, based
// atop a serial pair. Serial2 is RX on 17, TX on 16.
#define UART Serial2

#define INITIAL_PWM 128
unsigned Pwm; // initialized through setPwm

// we average tach signals over this many quanta, if we have them. we track
// the samples in a ringbuffer.
#define TACHQUANTA 5

static unsigned tachsamples[TACHQUANTA];
static unsigned long tachtotal; // sum of elements in tachsamples
static unsigned tachidx; // next position to write in tachsamples
static unsigned long lasttachtick; // last micros() tick for which we saved samples
static unsigned tachsamplecount; // [0..5], saturates at 5

// currently tracked tach values for this quantum, reset at each tick
volatile unsigned Pulses; // counter for input events, reset each second
volatile unsigned XTOPPulsesA; // counter for Dual XTOP Pump 1
volatile unsigned XTOPPulsesB; // counter for Dual XTOP Pump 2

const unsigned long QUANTUMUS = 1000000;

// artificial ceiling enforced for fans; if we count above this, we
// don't send it, as it's likely an error.
const unsigned long FANMAXRPM = 2000;
// artificial ceiling enforced for pumps, see above.
const unsigned long PUMPMAXRPM = 5000;

// fan tachometer needs an interrupt-capable digital pin. on Mega,
// this is 2, 3, 18, 19, 20, 21 (last two conflict with i2c).
const int RPMPIN = 19; // pin connected to tachometer

// tachometers for the two D5 pumps of the Dual XTOP
const int XTOPPINA = 20;
const int XTOPPINB = 21;

// for control of 12V RGB LEDs. we go through 10Kohm resistors on our way
// to 3 N-MOSFETs (IRLB8721s), and from there emerge 3x 12V signals.
const int RRGBPIN = 11;
const int GRGBPIN = 10;
const int BRGBPIN = 9;

// on mega:
//  pin 13, 4 == timer 0 (used for micros())
//  pin 12, 11 == timer 1
//  pin 10, 9 == timer 2
//  pin 5, 3, 2 == timer 3
//  pin 8, 7, 6 == timer 4
//  pin 44, 45, 46 == timer 5

// we need one digital output pin for fan PWM
const int PWMPIN = 44;
// and one for pump PWM, which must be on a different timer
// (we send the same PWM to both pumps)
const int XTOPPWMPIN = 8;

// Intel spec for PWM fans demands a 25K frequency.
const word PWM_FREQ_HZ = 25000;

// initialize to white so we can immediately diagnose any missing colors
byte Red = 255;
byte Green = 255;
byte Blue = 255;

#define FANS_PER_CHAN 3
#define LEDS_PER_FAN 12 // Arctic P14 ARGB
#define LEDS_PER_PWM (FANS_PER_CHAN * LEDS_PER_FAN)
#define PWM_CHANNELS 3
CRGB p14[PWM_CHANNELS * LEDS_PER_PWM];
 
// pulses is the tack count sampled in this last quantum. tick is the
// micros() value that ended the quantum. returns the average over
// valid tracked samples (i.e. up to TACHQUANTA seconds' worth).
static unsigned long update_tach_samples(unsigned pulses, unsigned long tick){
  if(tachidx == TACHQUANTA){
    tachidx = 0;
  }
  unsigned idx = tachidx++; // where we're writing to
  // first, we subtract any samples which are no longer valid from tachtotal
  // (aside from any we're about to overwrite). they're all zero at the
  // beginning and thus not a problem. any non-zero values must have been
  // added to tachtotal at some point, so they can be safely subtracted away
  // once expired without going negative.
  unsigned expired = (tick - lasttachtick) / QUANTUMUS;
  if(expired && --expired){
    Serial.print("expired: ");
    Serial.println(expired);
    if(expired > TACHQUANTA - 1){
      expired = TACHQUANTA - 1;
    }
  }
  unsigned expidx = idx; // where we're writing must be the oldest
  while(expired-- && tachsamplecount){
    Serial.print("subtracting at ");
    Serial.print(expidx);
    Serial.print(": ");
    Serial.println(tachsamples[expidx]);
    if(++expidx == TACHQUANTA){
      expidx = 0;
    }
    tachtotal -= tachsamples[expidx];
    tachsamples[expidx] = 0;
    --tachsamplecount;
  }
  Serial.print("adding at ");
  Serial.print(idx);
  Serial.print(": ");
  Serial.println(pulses);
  tachtotal -= tachsamples[idx];
  if(tachsamplecount == TACHQUANTA){
    --tachsamplecount; // for the one we just knocked out
  }
  tachtotal += pulses;
  tachsamples[idx] = pulses;
  ++tachsamplecount;
  lasttachtick = tick;
  Serial.print("total: ");
  Serial.print(tachtotal);
  Serial.print(" samples: ");
  Serial.println(tachsamplecount);
  return tachtotal / tachsamplecount;
}

static void rpm(void){
  if(Pulses < 65535){
    ++Pulses;
  }
}

static void xtopa(void){
  if(XTOPPulsesA < 65535){
    ++XTOPPulsesA;
  }
}

static void xtopb(void){
  if(XTOPPulsesB < 65535){
    ++XTOPPulsesB;
  }
}

static void setup_timers(void){
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  // Mode 10: phase correct PWM with ICR4 as Top (phase-correct needs a
  // factor of 2 in division below). OCR4C as Non-Inverted PWM output.
  // 16MHz / (25KHz * 2) == 320 cycles per interrupt.
  ICR4 = (F_CPU / PWM_FREQ_HZ) / 2;
  TCCR4A = _BV(COM4C1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(CS40);
  ICR5 = (F_CPU / PWM_FREQ_HZ) / 2;
  TCCR5A = _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(CS50);
}

static void apply_rgb(void){
  Serial.println("committing RGB values");
  analogWrite(RRGBPIN, Red);
  analogWrite(GRGBPIN, Green);
  analogWrite(BRGBPIN, Blue);
}

static void setup_rgb(int rpin, int gpin, int bpin){
  pinMode(rpin, OUTPUT);
  pinMode(gpin, OUTPUT);
  pinMode(bpin, OUTPUT);
}

// for the MEGA+CODI6, we're running 5V ARGB rather than 12V RGB LEDs.
// set them up on PWM-mapped pins 3, 5, and 6.
static void setup_argb(void){
  FastLED.addLeds<NEOPIXEL, 3>(p14 + LEDS_PER_PWM * 0, 0, sizeof(CRGB) * LEDS_PER_PWM);
  FastLED.addLeds<NEOPIXEL, 5>(p14 + LEDS_PER_PWM * 1, 0, sizeof(CRGB) * LEDS_PER_PWM);
  FastLED.addLeds<NEOPIXEL, 6>(p14 + LEDS_PER_PWM * 2, 0, sizeof(CRGB) * LEDS_PER_PWM);
  for(unsigned i = 0 ; i < sizeof(p14) / sizeof(*p14) ; ++i){
    p14[i] = (i % 2) ? CRGB::Cyan : CRGB::Green;
  }
  FastLED.show();
}

void setup(){
  Serial.begin(SERIALSPEED);
  while(!Serial); // only necessary/meaningful for boards with native USB
  UART.begin(UARTSPEED);

  pinMode(PWMPIN, OUTPUT);
  pinMode(XTOPPWMPIN, OUTPUT);
  setup_timers();
  Serial.print("pwm write on ");
  Serial.print(PWMPIN);
  Serial.print(", ");
  Serial.println(XTOPPWMPIN);
  setFanPWM(INITIAL_PWM);
  setPumpPWM(INITIAL_PWM);
  Pulses = 0;
  pinMode(RPMPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPMPIN), rpm, RISING);
  Serial.print("fan tachometer read on ");
  Serial.println(RPMPIN);

  XTOPPulsesA = 0;
  XTOPPulsesB = 0;
  pinMode(XTOPPINA, INPUT_PULLUP);
  pinMode(XTOPPINB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(XTOPPINA), xtopa, RISING);
  attachInterrupt(digitalPinToInterrupt(XTOPPINB), xtopb, RISING);

  setup_rgb(RRGBPIN, GRGBPIN, BRGBPIN);
  // FIXME only want this for schwarzgerat mora
  setup_argb();
}

// ripped from Effects/Hypnotoad/Hypnotoad.cpp in OpenRGBEffectsPlugin (GPL2)
#define animation_speed 4.0
#define color_rotation_speed 3.0
#define spacing 1.0
#define thickness 3.0
#define Speed 30
#define FPS 60

double progress = 10;
unsigned cx_shift = 50;
unsigned cy_shift = 50;

static void GetColor(unsigned x, unsigned y, float cx, float cy, CRGB* leds){
  float animation_mult = 0.01 * animation_speed;
  float color_mult = 0.01 * color_rotation_speed;

  double angle    = atan2(y - cy, x - cx) * 180 / 3.14159265359;
  double distance = sqrt(pow(cx - x, 2) + pow(cy - y, 2));
  float  value    = cos(animation_mult * distance / (0.1 * (float) spacing)  + progress);

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
  progress +=  0.1 * (float) Speed / (float) FPS;
}

static void setFanPWM(unsigned pwm){
  Pwm = pwm;
  OCR4C = ICR4 * pwm / 256;
  Serial.print("PWM to ");
  Serial.println(pwm);
}

static void setPumpPWM(unsigned pwm){
  Pwm = pwm;
  OCR4C = ICR4 * pwm / 256;
  Serial.print("PWM to ");
  Serial.println(pwm);
}

// apply a PWM value between 0 and 255, inclusive.
static int apply_pwm(int in){
  if(in >= 0){
    if(in <= 255){
      setFanPWM(in);
      return 0;
    }
  }
  Serial.print("invalid PWM level: ");
  Serial.println(in);
  return -1;
}

// handle a byte read from the UART
static void handle_uart(int in){
  static enum {
    STATE_BEGIN,
    STATE_PWM,  // got a 'P'
    STATE_RRGB, // got a 'C'
    STATE_GRGB, // got a Red
    STATE_BRGB, // got a Green
  } state = STATE_BEGIN;
  static byte uart_red;
  static byte uart_green;
  switch(state){
    case STATE_BEGIN:
      if(in == 'P'){
        state = STATE_PWM;
      }else if(in == 'C'){
        state = STATE_RRGB;
      }else{
        Serial.println("invalid uart input");
      }
      break;
    case STATE_PWM:
      apply_pwm(in);
      state = STATE_BEGIN;
      break;
    case STATE_RRGB:
      uart_red = in;
      state = STATE_GRGB;
      break;
    case STATE_GRGB:
      uart_green = in;
      state = STATE_BRGB;
      break;
    case STATE_BRGB:
      state = STATE_BEGIN;
      Red = uart_red;
      Green = uart_green;
      Blue = in;
      apply_rgb();
      break;
    default:
      Serial.println("invalid uart state");
      break;
  }
}

// read bytes from UART/USB, using the global references.
// each USB byte is interpreted as a PWM level, and ought be between [0..255].
// UART bytes move through a state machine, with state preserved across calls.
static void check_pwm_update(void){
  int in;
  while((in = UART.read()) != -1){
    Serial.print("read byte from uart: ");
    Serial.println(in);
    handle_uart(in);
  }
  while((in = Serial.read()) != -1){
    Serial.print("read byte from usb: ");
    Serial.println(in);
    apply_pwm(in);
  }
}

static void printRPMSample(unsigned long val, char proto,
                           const char* desc, int pin,
                           unsigned long typemax){
  Serial.print(pin, DEC);
  Serial.print(" ");
  // each rotation is two tach signals, and RPM is reported per
  // minutes: 60 / 2 == 30.
  unsigned c = val * 30;
  if(c > typemax){
    Serial.print("invalid read ");
  }else{
    UART.print(proto);
    UART.print(c);
  }
  Serial.print(val, DEC);
  Serial.print(" ");
  Serial.print(c, DEC);
  Serial.print(" ");
  Serial.println(desc);
}

void loop(){
  static unsigned long m;
  unsigned long cur;

  apply_rgb();
  for(int p = 0 ; p < PWM_CHANNELS ; ++p){
    StepEffect(p14 + LEDS_PER_PWM * p, LEDS_PER_PWM);
  }
  FastLED.show();
  apply_pwm(Pwm);
  do{
    cur = micros();
    check_pwm_update();
  }while(cur - m < QUANTUMUS); // handles overflow implicitly
  noInterrupts();
  unsigned p = Pulses;
  Pulses = 0;
  unsigned xa = XTOPPulsesA;
  XTOPPulsesA = 0;
  unsigned xb = XTOPPulsesB;
  XTOPPulsesB = 0;
  interrupts();
  unsigned long rpmavg = update_tach_samples(p, cur);
  printRPMSample(rpmavg, 'R', "fans", RPMPIN, FANMAXRPM);
  printRPMSample(xa, 'A', "XtopA", XTOPPINA, PUMPMAXRPM);
  printRPMSample(xb, 'B', "XtopB", XTOPPINB, PUMPMAXRPM);
  Serial.print(cur - m, DEC);
  Serial.println("µs");
  UART.print("P");
  UART.print(Pwm);
  Serial.print("XTOPA: ");
  Serial.print(xa);
  Serial.print(" XTOPB: ");
  Serial.print(xb);
  Serial.print(" PWM output: ");
  Serial.print(Pwm);
  Serial.println();
  m = cur;
}

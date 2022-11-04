#include <SoftwareSerial.h>
// requires an Arduino Mega2560, possibly rev3, possibly only the authentic one.

#define INITIAL_PWM 0

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

// pulses is the tack count sampled in this last quantum. tick is the
// micros() value that ended the quantum. returns the average over
// valid tracked samples (i.e. up to TACHQUANTA seconds' worth).
static unsigned long
update_tach_samples(unsigned pulses, unsigned long tick){
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

const unsigned long SERIALSPEED = 115200;
const unsigned long UARTSPEED = 9600;

// artificial ceiling enforced for fans and pumps
const unsigned long MAXRPM = 4000;

// tachometer needs an interrupt-capable digital pin. on Mega,
// this is 2, 3, 18, 19, 20, 21 (last two conflict with i2c).
const int RPMPIN = 2; // pin connected to tachometer

// tachometers for the two D5 pumps of the Dual XTOP
const int XTOPPINA = 20;
const int XTOPPINB = 21;

// we'll use two pins for UART communication with the ESP32, based
// atop a serial pair. Serial2 is RX on 17, TX on 16.
#define UART Serial2

// for control of 12V RGB LEDs. we go through 10Kohm resistors on our way
// to 3 N-MOSFETs (IRLB8721s), and from there emerge 3x 12V signals.
const int RRGBPIN = 11;
const int GRGBPIN = 10;
const int BRGBPIN = 9;

// we need three digital output pins for PWM (one for fans, two for pumps).
const int PWMPIN = 8;
const int XTOPPWMPINA = 44;
const int XTOPPWMPINB = 45;

// Intel spec for PWM fans demands a 25K frequency.
const word PWM_FREQ_HZ = 25000;

// initialize to white so we can immediately diagnose any missing colors
byte Red = 255;
byte Green = 255;
byte Blue = 255;

unsigned Pwm;
// on mega:
//  pin 13, 4 == timer 0 (used for micros())
//  pin 12, 11 == timer 1
//  pin 10, 9 == timer 2
//  pin 5, 3, 2 == timer 3
//  pin 8, 7, 6 == timer 4
//  pin 44, 45, 46 == timer 5

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
  // factor of 2 in division below). OC4C as Non-Inverted PWM output.
  // 16MHz / (25KHz * 2) == 320 cycles per interrupt.
  ICR4 = (F_CPU / PWM_FREQ_HZ) / 2;
  TCCR4A = _BV(COM4C1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(CS40);
  ICR5 = (F_CPU / PWM_FREQ_HZ) / 2;
  TCCR5A = _BV(COM4C1) | _BV(WGM41);
  TCCR5B = _BV(WGM43) | _BV(CS40);
}

static void apply_rgb(void){
  Serial.println("committing RGB values");
  analogWrite(RRGBPIN, Red);
  analogWrite(GRGBPIN, Green);
  analogWrite(BRGBPIN, Blue);
}

void setup(){
  Serial.begin(SERIALSPEED);
  while(!Serial); // only necessary/meaningful for boards with native USB
  UART.begin(UARTSPEED);

  pinMode(PWMPIN, OUTPUT);
  pinMode(XTOPPWMPINA, OUTPUT);
  pinMode(XTOPPWMPINB, OUTPUT);
  setup_timers();
  Serial.print("pwm write on ");
  Serial.print(PWMPIN);
  Serial.print(", ");
  Serial.print(XTOPPWMPINA);
  Serial.print(", ");
  Serial.println(XTOPPWMPINB);
  setPWM(INITIAL_PWM);

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

  pinMode(RRGBPIN, OUTPUT);
  pinMode(GRGBPIN, OUTPUT);
  pinMode(BRGBPIN, OUTPUT);
  apply_rgb();
}

void setPWM(unsigned pwm){
  Pwm = pwm;
  OCR4C = ICR4 * (unsigned long)(pwm + 1) / 256;
  OCR5C = ICR5 * (unsigned long)(pwm + 1) / 256;
  Serial.print("PWM to ");
  Serial.print(pwm);
  Serial.print(", OCR4C to ");
  Serial.print(OCR4C);
  Serial.print(", OCR5C to ");
  Serial.println(OCR5C);
}

// apply a PWM value between 0 and 255, inclusive.
static int apply_pwm(int in){
  if(in >= 0){
    if(in <= 255){
      setPWM(in);
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

static void printRPMSample(unsigned long val, const char* proto,
                           const char* desc, int pin){
  // each rotation is two tach signals, and RPM is reported per
  // minutes: 60 / 2 == 30.
  unsigned c = val * 30;
  if(c > MAXRPM){
    Serial.print("invalid read ");
    Serial.print(val);
  }else{
    UART.print(proto);
    UART.print(c);
    Serial.print(pin, DEC);
    Serial.print(" ");
    Serial.print(val, DEC);
    Serial.print(" ");
    Serial.print(c, DEC);
  }
  Serial.print(" ");
  Serial.print(desc);
  Serial.print(' ');
}

void loop (){
  static unsigned long m = 0;
  unsigned long cur;

  apply_rgb();
  apply_pwm(Pwm);
  if(m == 0){
    m = micros();
  }
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
  printRPMSample(rpmavg, "R", "fans", RPMPIN);
  printRPMSample(xa, "A", "XtopA", XTOPPINA);
  printRPMSample(xb, "B", "XtopB", XTOPPINB);
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

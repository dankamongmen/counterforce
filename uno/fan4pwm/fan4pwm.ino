#include <FastLED.h>
#include <SoftwareSerial.h>
// requires an Arduino Mega2560, possibly rev3, possibly only the authentic one.

const unsigned long SERIALSPEED = 115200;
//const unsigned long UARTSPEED = 9600;
// we'll use two pins for UART communication with the ESP32, based
// atop a serial pair. Serial2 is RX on 17, TX on 16.
//#define UART Serial2

#define INITIAL_PWM 130 // silent with Arctic P-14s
unsigned Pwm; // initialized through setFanPWM
unsigned PumpPwm; // initialized through setPumpPWM

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
const unsigned long FANMAXRPM = 3000; // FIXME ought be 2000
// artificial ceiling enforced for pumps, see above.
const unsigned long PUMPMAXRPM = 5000;

// fan tachometer needs an interrupt-capable digital pin. on Mega,
// this is 2, 3, 18, 19, 20, 21 (last two conflict with i2c).
const int RPMPIN = 2; // pin connected to tachometer

// tachometers for the two D5 pumps of the Dual XTOP
const int XTOPPINA = 20;
const int XTOPPINB = 21;

// 3-wire transducer outputs 0.5--4.5V
const int PRESSUREPIN = A0;

// on mega:
//  pin 13, 4 == timer 0 (used for micros())
//  pin 12, 11 == timer 1
//  pin 10, 9 == timer 2
//  pin 5, 3, 2 == timer 3
//  pin 8, 7, 6 == timer 4
//  pin 44, 45, 46 == timer 5

// we need one digital output pin for fan PWM
const int PWMPIN = 11;

// and one for pump PWM, which must be on a different timer
// (we send the same PWM to both pumps)
const int XTOPPWMPIN = 8;

// Intel spec for PWM fans demands a 25K frequency.
const word PWM_FREQ_HZ = 25000;

// initialize to white so we can immediately diagnose any missing colors
byte Red = 255;
byte Green = 255;
byte Blue = 255;

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

static void setup_timers(unsigned initpwm){
  TCNT1 = 0;
  TCCR1A = _BV(1 << COM1A1) | _BV(1 << WGM11);
  TCCR1B = _BV(1 << WGM13) | _BV(1 << CS10);
  ICR1 = (F_CPU / PWM_FREQ_HZ) / 2; // 320
  setFanPWM(initpwm);
  setPumpPWM(initpwm);
}

void setup(){
  Serial.begin(SERIALSPEED);
  while(!Serial); // only necessary/meaningful for boards with native USB
  //UART.begin(UARTSPEED);

  pinMode(PWMPIN, OUTPUT);
  pinMode(XTOPPWMPIN, OUTPUT);
  setup_timers(INITIAL_PWM);
  Serial.print("pwm write on ");
  Serial.print(PWMPIN);
  Serial.print(", ");
  Serial.println(XTOPPWMPIN);
  apply_pwm(INITIAL_PWM);
  apply_pump_pwm(INITIAL_PWM);
  Pulses = 0;
  XTOPPulsesA = 0;
  XTOPPulsesB = 0;
  pinMode(RPMPIN, INPUT_PULLUP);
  pinMode(XTOPPINA, INPUT_PULLUP);
  pinMode(XTOPPINB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPMPIN), rpm, FALLING);
  Serial.print("fan tachometer read on ");
  Serial.println(RPMPIN);
  attachInterrupt(digitalPinToInterrupt(XTOPPINA), xtopa, RISING);
  Serial.print("pump A tachometer read on ");
  Serial.println(XTOPPINA);
  attachInterrupt(digitalPinToInterrupt(XTOPPINB), xtopb, RISING);
  Serial.print("pump B tachometer read on ");
  Serial.println(XTOPPINB);

  pinMode(PRESSUREPIN, INPUT_PULLUP);
}

static void setFanPWM(unsigned pwm){
  Pwm = pwm;
  OCR2A = pwm;
  Serial.print("PWM to ");
  Serial.print(pwm);
}

static void setPumpPWM(unsigned pwm){
  PumpPwm = pwm;
  OCR2B = pwm / 256;
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

// apply a PWM value between 0 and 255, inclusive.
static int apply_pump_pwm(int in){
  if(in >= 0){
    if(in <= 255){
      setPumpPWM(in);
      return 0;
    }
  }
  Serial.print("invalid PWM level: ");
  Serial.println(in);
  return -1;
}

// check whether csum byte (0..255) is equal to 255 - (sum % 256)
static bool valid_csum(unsigned csum, unsigned sum){
  return csum == 255 - (sum % 256);
}

// handle a byte read from the UART. all UART commands are terminated by a
// checksum byte which ought be 255 less the 8-bit sum of all bytes.
static void handle_uart(int in){
  static enum {
    STATE_BEGIN,
    STATE_PWM,  // got a 'P'
    STATE_PWM_FINAL, // got a PWM, want checksum
    STATE_RRGB, // got a 'C'
    STATE_GRGB, // got a Red
    STATE_BRGB, // got a Green
    STATE_RGB_FINAL, // got an RGB, want checksum
    STATE_PUMPPWM, // got a 'U'
    STATE_PUMPPWM_FINAL, // got a PWM, want checksum
  } state = STATE_BEGIN;
  static byte uart_red;
  static byte uart_green;
  static byte uart_blue;
  static byte pwm_pending;
  static byte pumppwm_pending;
  switch(state){
    case STATE_BEGIN:
      if(in == 'P'){
        state = STATE_PWM;
      }else if(in == 'C'){
        state = STATE_RRGB;
      }else if(in == 'U'){
        state = STATE_PUMPPWM;
      }else{
        Serial.print("invalid uart input: ");
        Serial.println(in, HEX);
      }
      break;
    case STATE_PWM:
      pwm_pending = in;
      state = STATE_PWM_FINAL;
      break;
    case STATE_PWM_FINAL:
      if(valid_csum(in, 'P' + pwm_pending)){
        apply_pwm(pwm_pending);
      }else{
        Serial.println("invalid PWM checksum");
      }
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
      uart_blue = in;
      state = STATE_RGB_FINAL;
      break;
    case STATE_RGB_FINAL:
      if(valid_csum(in, 'C' + uart_red + uart_green + uart_blue)){
        Red = uart_red;
        Green = uart_green;
        Blue = uart_blue;
        //apply_rgb();
      }else{
        Serial.println("invalid RGB checksum");
      }
      state = STATE_BEGIN;
      break;
    case STATE_PUMPPWM:
      pumppwm_pending = in;
      state = STATE_PUMPPWM_FINAL;
      break;
    case STATE_PUMPPWM_FINAL:
      if(valid_csum(in, 'U' + pumppwm_pending)){
        apply_pump_pwm(pumppwm_pending);
      }else{
        Serial.println("invalid pump checksum");
      }
      state = STATE_BEGIN;
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
    /*
  while((in = UART.read()) != -1){
    Serial.print("read byte from uart: 0x");
    Serial.print(in, HEX);
    Serial.print(", ");
    Serial.println((char)in);
    handle_uart(in);
  }
    */
  while((in = Serial.read()) != -1){
    Serial.print("read byte from usb: ");
    Serial.println(in);
    apply_pwm(in);
  }
}

static inline float rpm(unsigned long pulses, unsigned long usec){
  Serial.print(pulses);
  Serial.println(" pulses measured");
  // each rotation is two tach signals, and RPM is reported per minutes
  return pulses * 60 * 1000000.0 / usec / 2;
}

static void printRPMSample(float rpm, char proto,
                           const char* desc, int pin,
                           unsigned long typemax){
  Serial.print(pin, DEC);
  Serial.print(" ");
  Serial.print(desc);
  Serial.print(" rpm: ");
  /*
  if(rpm < typemax){
    UART.print(proto);
    UART.print(rpm);
  }
  */
  Serial.print(rpm, DEC);
  Serial.print(" ");
}

static void printPWM(unsigned long val, char proto, const char* desc){
  /*
  UART.print(proto);
  UART.print(val, DEC);
  */
  Serial.print(desc);
  Serial.print(": ");
  Serial.print(val, DEC);
  Serial.print(" ");
}

static void read_pressure(int pin, float* p){
  *p = analogRead(pin);
  Serial.print("read pressure: ");
  Serial.println(*p);
}

void loop(){
  static unsigned long m;
  unsigned long cur;

  do{
    /*for(int p = 0 ; p < PWM_CHANNELS ; ++p){
      StepEffect(p14 + LEDS_PER_PWM * p, LEDS_PER_PWM);
    }
    FastLED.show();*/
    cur = micros();
    check_pwm_update();
    float p;
    read_pressure(PRESSUREPIN, &p);
  }while(cur - m < QUANTUMUS); // handles overflow implicitly
  noInterrupts();
  unsigned p = Pulses;
  Pulses = 0;
  unsigned xa = XTOPPulsesA;
  XTOPPulsesA = 0;
  unsigned xb = XTOPPulsesB;
  XTOPPulsesB = 0;
  interrupts();
  //unsigned long rpmavg = update_tach_samples(p, cur);
  // FIXME need update_tach_samples() for pumps also, methinks?
  /*
  UART.print("P");
  UART.print(Pwm);
  UART.print("U");
  UART.print(PumpPwm);
  */
  float fp = rpm(p, cur - m);
  float fxa = rpm(xa, cur - m);
  float fxb = rpm(xb, cur - m);
  printRPMSample(fp, 'R', "fans", RPMPIN, FANMAXRPM);
  printRPMSample(fxa, 'A', "XtopA", XTOPPINA, PUMPMAXRPM);
  printRPMSample(fxb, 'B', "XtopB", XTOPPINB, PUMPMAXRPM);
  printPWM(Pwm, 'P', "fans");
  printPWM(PumpPwm, 'U', "pumps");
  Serial.print(cur - m, DEC);
  Serial.println("µs");
  m = cur;
}

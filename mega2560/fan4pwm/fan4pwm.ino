#include <SoftwareSerial.h>
// requires an Arduino Mega2560, possibly rev3, possibly only the authentic one.

const unsigned long SERIALSPEED = 115200;
const unsigned long UARTSPEED = 9600;

// artificial ceiling enforced for fans and pumps
const unsigned long MAXRPM = 6000;

volatile unsigned Pulses; // counter for input events, reset each second
volatile unsigned XTOPPulsesA; // counter for Dual XTOP Pump 1
volatile unsigned XTOPPulsesB; // counter for Dual XTOP Pump 2

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

// fixed 12V LEDs in the reservoir atop the MO-RA3
const int RRESPIN = 7;
const int GRESPIN = 6;
const int BRESPIN = 5;

// we need a digital output pin for PWM.
const int PWMPIN = 8;

// Intel spec for PWM fans demands a 25K frequency.
const word PWM_FREQ_HZ = 25000;

byte Red = 32;
byte Green = 64;
byte Blue = 128;

unsigned Pwm;
// on mega:
//  pin 13, 4 == timer 0 (used for micros())
//  pin 12, 11 == timer 1
//  pin 10, 9 == timer 2
//  pin 5, 3, 2 == timer 3
//  pin 8, 7, 6 == timer 4

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
  // Mode 10: phase correct PWM with ICR4 as Top
  // OC4C as Non-Inverted PWM output
  ICR4 = (F_CPU / PWM_FREQ_HZ) / 2;
  TCCR4A = _BV(COM4C1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(CS40);
}

static void apply_rgb(void){
  Serial.println("committing RGB values");
  analogWrite(RRGBPIN, Red);
  analogWrite(GRGBPIN, Green);
  analogWrite(BRGBPIN, Blue);
}

void setup(){
  const byte INITIAL_PWM = 0;
  Serial.begin(SERIALSPEED);
  while(!Serial); // only necessary/meaningful for boards with native USB
  UART.begin(UARTSPEED);

  setup_timers();
  pinMode(PWMPIN, OUTPUT);
  Serial.print("pwm write on ");
  Serial.println(PWMPIN);
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

  // the reservoir color is fixed; just set it once here
  pinMode(RRESPIN, OUTPUT);
  pinMode(GRESPIN, OUTPUT);
  pinMode(BRESPIN, OUTPUT);
  analogWrite(RRESPIN, 0);
  analogWrite(GRESPIN, 0);
  analogWrite(BRESPIN, 255);
}

void setPWM(byte pwm){
  Pwm = pwm;
  OCR4C = pwm;
  Serial.print("PWM to ");
  Serial.print(pwm);
  Serial.print(" OCR4C to ");
  Serial.println(OCR4C);
}

// apply a PWM value between 0 and 255, inclusive.
static int apply_pwm(int in){
  if(in >= 0){
    if(in > 255){
      Serial.print("invalid PWM level: ");
      Serial.println(in);
    }else{
      setPWM(in);
    }
  }
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
// each byte is interpreted as a PWM level, and ought be between [0..255].
// we act on the last byte available. USB has priority over UART.
static void check_pwm_update(void){
  int last = -1;
  int in;
  // only apply the last in a sequence
  while((in = UART.read()) != -1){
    Serial.print("read byte from uart: ");
    Serial.println(in);
    handle_uart(in);
  }
  while((in = Serial.read()) != -1){
    Serial.print("read byte from usb: ");
    Serial.println(in);
    last = in;
  }
  apply_pwm(last);
}

static void printRPMSample(unsigned long val, const char* proto,
                           const char* desc, int pin){
  if(val * 30 > MAXRPM){
    Serial.print("invalid read ");
    Serial.print(val);
  }else{
    unsigned c = val * 30;
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

const unsigned long LOOPUS = 1000000;

void loop (){
  static unsigned long m = 0;
  unsigned long cur;

  if(m == 0){
    m = micros();
  }
  do{
    cur = micros();
    // handle micros() overflow...
    if(cur < m){
      if(m + LOOPUS > m){
        break;
      }else if(cur > m + LOOPUS){
        break;
      }
    }
  }while(cur - m < LOOPUS);
  noInterrupts();
  unsigned p = Pulses;
  Pulses = 0;
  unsigned xa = XTOPPulsesA;
  XTOPPulsesA = 0;
  unsigned xb = XTOPPulsesB;
  XTOPPulsesB = 0;
  interrupts();
  printRPMSample(p, "R", "fans", RPMPIN);
  printRPMSample(xa, "A", "XtopA", XTOPPINA);
  printRPMSample(xb, "B", "XtopB", XTOPPINB);
  Serial.print(cur - m, DEC);
  Serial.println("µs");
  UART.print("P");
  UART.print(Pwm);
  Serial.print("XTOPA: ");
  Serial.print(xa);
  Serial.print(" XTOPB: ");
  Serial.println(xb);
  Serial.print(" PWM output: ");
  Serial.print(Pwm);
  Serial.println();
  Serial.print("R");
  Serial.print(p * 30);
  Serial.print("P");
  Serial.println(Pwm);
  check_pwm_update();
  m = cur;
}

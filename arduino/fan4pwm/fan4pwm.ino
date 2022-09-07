#include <SoftwareSerial.h>
// requires an Arduino Uno, possibly rev3, possibly only the authentic one.

const unsigned long SERIALSPEED = 115200;

volatile unsigned Pulses; // counter for input events, reset each second

// tachometer needs an interrupt-capable digital pin. on Mega,
// this is 2, 3, 18, 19, 20, 21 (last two conflict with i2c).
// on Uno, only 2 and 3 are available!
const int RPMPIN = 3; // pin connected to tachometer

// we'll use two pins for UART communication with the ESP32
const int RXPIN = 6;
const int TXPIN = 7;
SoftwareSerial uart(RXPIN, TXPIN);

// we need a digital output pin for PWM.
const int PWMPIN = 9;

const int TEMPPIN = A0;

// Intel spec for PWM fans demands a 25K frequency.
const word PWM_FREQ_HZ = 25000;

// Arduino Uno has a 16MHz processor.
const unsigned long TCNT1_TOP = 16000000ul / (2 * PWM_FREQ_HZ);

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

static void setup_timers(void){
  // configure timer0
  //  COM1A(1:0) = 0b10   (output A clear rising/set falling)
  //  COM1B(1:0) = 0b00   (output B normal operation)
  //  WGM(13:10) = 0b1010 (phase-correct pwm)
  //  ICNC1      = 0b0    (input noise canceler disabled)
  //  ICES1      = 0b0    (input edge select disabled)
  //  CS(12:10)  = 0b001  (input clock select = clock / 1)
  TCNT1  = 0;
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
}

void setup(){
  const byte INITIAL_PWM = 40;
  Serial.begin(SERIALSPEED);
  while(!Serial); // only necessary/meaningful for boards with native USB
  uart.begin(SERIALSPEED);

  pinMode(PWMPIN, OUTPUT);
  setup_timers();
  Serial.print("pwm write on ");
  Serial.println(PWMPIN);
  setPWM(INITIAL_PWM);

  Pulses = 0;
  pinMode(RPMPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RPMPIN), rpm, RISING);
  Serial.print("tachometer read on ");
  Serial.println(RPMPIN);

  pinMode(TEMPPIN, INPUT);
  // we'll get better thermistor readings if we use the cleaner
  // 3.3V line. connect 3.3V to AREF.
  //analogReference(EXTERNAL);

  //ADMUX = 0xc8; // enable internal temperature sensor via ADC
}

void setPWM(byte pwm){
  Pwm = pwm;
  OCR1A = (unsigned long)(((float)pwm  / 100) * TCNT1_TOP);
  Serial.print("PWM to ");
  Serial.print(pwm);
  Serial.print(" OCR1A to ");
  Serial.println(OCR1A);
}

// apply a PWM value between 0 and 100, inclusive.
static int apply_pwm(int in){
  if(in >= 0){
    if(in > 100){
      Serial.print("invalid PWM level: ");
      Serial.println(in);
    }else{
      setPWM(in);
    }
  }
}

// read bytes from UART/USB, using the global references.
// each byte is interpreted as a PWM level, and ought be between [0..100].
// we act on the last byte available. USB has priority over UART.
static void check_pwm_update(void){
  int last = -1;
  int in;
  // only apply the last in a sequence
  while((in = uart.read()) != -1){
    Serial.print("read byte from uart: ");
    Serial.println(in);
    last = in;
  }
  while((in = Serial.read()) != -1){
    Serial.print("read byte from usb: ");
    Serial.println(in);
    last = in;
  }
  apply_pwm(last);
}

float readThermistor(void){
  const int BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 25;
  const float R1 = 10;
  const float VREF = 3.3;
  float v0 = analogRead(TEMPPIN);
  //Serial.print("read raw voltage: ");
  //Serial.print(v0);
  float scaled = v0 * (VREF / 1023.0);
  //Serial.print(" scaled: ");
  //Serial.println(scaled);
  float R = (scaled * R1) / (VREF - scaled);
  float t = 1.0 / ((1.0 / NOMINAL) + ((log(R / 10)) / BETA));
  //Serial.print("read raw temp: ");
  //Serial.println(t);
  return t;
}

const unsigned long LOOPUS = 1000000;

void loop (){
  unsigned long m = micros();
  unsigned long cur;

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
  interrupts();
  if(p * 30 > 65535){
    Serial.print("invalid RPM read: ");
    Serial.print(p);
  }else{
    unsigned c = p * 30;
    Serial.print(RPMPIN, DEC);
    Serial.print(" ");
    Serial.print(p, DEC);
    Serial.print(" ");
    Serial.print(c, DEC);
    Serial.print(" rpm");
  }
  Serial.print(' ');
  Serial.print(cur - m, DEC);
  Serial.println("µs");
  Serial.println("");
  float therm = readThermistor();
  Serial.print("Thermistor: ");
  Serial.print(therm);
  Serial.print(" PWM output: ");
  Serial.print(Pwm);
  //Serial.print(" Internal temp: ");
  //Serial.print(temp);
  Serial.println();
  check_pwm_update();
}

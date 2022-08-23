// requires an Arduino Uno, possibly rev3, possibly only the authentic one.
// a Mega2560 does not seem to work (though it was a clone...)
#include "heltec.h"
volatile unsigned Pulses; // counter for input events, reset each second

// tachometer needs an interrupt-capable digital pin. on Mega,
// this is 2, 3, 18, 19, 20, 21 (last two conflict with i2c).
// on Uno, only 2 and 3 are available!
const int RPMPIN = 2; // pin connected to tachometer

// we need a digital output pin for PWM.
const int PWMPIN = 9;

#define TEMPPIN A0

// Intel spec for PWM fans demands a 25K frequency.
const word PWM_FREQ_HZ = 25000;

unsigned Pwm;

// on mega:
//  pin 13, 4 == timer 0 (used for micros())
//  pin 12, 11 == timer 1
//  pin 10, 9 == timer 2
//  pin 5, 3, 2 == timer 3
//  pin 8, 7, 6 == timer 4

static void rpm(){
  if(Pulses < 65535){
    ++Pulses;
  }
}

void setup(){
  const byte INITIAL_PWM = 40;
  Heltec.begin(true /*DisplayEnable Enable*/,
               false /*LoRa Disable*/,
               true /*Serial Enable*/);
  Heltec.display->setFont(ArialMT_Plain_10);
  
  pinMode(PWMPIN, OUTPUT);
  Serial.print("pwm write on ");
  Serial.println(PWMPIN);
  setPWM(INITIAL_PWM);

  Pulses = 0;
  pinMode(RPMPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RPMPIN), rpm, RISING);
  Serial.print("tachometer read on ");
  Serial.println(RPMPIN);

  // we'll get better thermistor readings if we use the cleaner
  // 3.3V line. connect 3.3V to AREF.
  pinMode(TEMPPIN, INPUT);
  //analogReference(EXTERNAL);

}

void setPWM(byte pwm){
  Serial.print("PWM to ");
  Serial.println(pwm);
  Pwm = pwm;
  // FIXME
}

const unsigned long LOOPUS = 1000000;

// read bytes from Serial, using the global state. each byte is interpreted as a PWM
// level, and ought be between [0..100]. we act on the last byte available.
void check_pwm_update(){
  int last = -1;
  int in;
  while((in = Serial.read()) != -1){
    Serial.print("read byte from input: ");
    Serial.println(in);
    last = in;
  }
  if(last >= 0){
    if(last > 100){
      Serial.print("invalid PWM level: ");
      Serial.println(in);
    }else{
      setPWM(last);
    }
  }
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
  unsigned p = Pulses;
  Pulses = 0;
  unsigned c;
  if(p * 30 > 65535){
    Serial.print("invalid RPM read: ");
    Serial.print(p);
    c = 65535;
  }else{
    c = p * 30;
    Serial.print(RPMPIN, DEC);
    Serial.print(" ");
    Serial.print(p, DEC);
    Serial.print(" ");
    Serial.print(c, DEC);
    Serial.print(" rpm ");
  }
  Serial.print(cur - m, DEC);
  Serial.println("µs");
  float therm = readThermistor();
  Serial.print("Thermistor: ");
  Serial.print(therm);
  Serial.print(" PWM output: ");
  Serial.print(Pwm);
  Serial.println();

    Heltec.display->clear();
    Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
    Heltec.display->drawString(0, 0, "RPM: ");
    Heltec.display->drawString(30, 0, String(c));
    Heltec.display->drawString(0, 11, "PWM: ");
    Heltec.display->drawString(35, 11, String(Pwm));
    Heltec.display->drawString(0, 21, "Temp: ");
    Heltec.display->drawString(40, 21, String(therm));
    Heltec.display->drawString(0, 31, "Uptime: ");
    Heltec.display->drawString(40, 31, String(millis() / 1000));
    Heltec.display->display();

  check_pwm_update();
}

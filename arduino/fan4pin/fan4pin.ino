// intended to run on a Mega2560
#include <PWM.h>

volatile unsigned Pulses; // counter for input events, reset each second

// we need an interrupt-capable pin. on MEGA, this is
// 2, 3, 18, 19, 20, 21 (last two conflict with i2c)
const int RPMPIN = 21; // pin connected to tachometer
// on mega:
//  pin 13, 4 == timer 0 (used for micros())
//  pin 12, 11 == timer 1
//  pin 10, 9 == timer 2
//  pin 5, 3, 2 == timer 3
//  pin 8, 7, 6 == timer 4
const int PWMPIN = 9; // pin connected to PWM

static void rpm(){
  if(Pulses < 65535){
    ++Pulses;
  }
}

void setup(){
  Serial.begin(115200);
  InitTimersSafe(); // don't blow away timer 0
  if(!SetPinFrequency(PWMPIN, 25000)){
    Serial.println("Failure initializing PWMPIN");
  }else{
    Serial.print("Successfully initialized PWMPIN at resolution ");
    pinMode(PWMPIN, OUTPUT);
    float res = GetPinResolution(PWMPIN);
    Serial.println(res);
    pwmWrite(PWMPIN, 128);
  }
  pinMode(RPMPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPMPIN), rpm, RISING);
}
 
void loop (){
  Pulses = 0;
  unsigned long m = micros();
  unsigned long cur;

  sei();       // enable interrupts
  do{
    cur = micros();
    if(cur <= 0){
      Serial.print("NEGATIVE MICROS!\n"); // did you fuck timer 0?
    }
  }while(cur - m < 1000000); // FIXME handle overflow
  cli();       // disable interrupts
  unsigned c = Pulses * 30;
  Serial.print(RPMPIN, DEC);
  Serial.print(" ");
  Serial.print(Pulses, DEC);
  Serial.print(" ");
  Serial.print(c, DEC);
  Serial.print(" rpm ");
  Serial.print(cur - m, DEC);
  Serial.println("µs");
  pwmWrite(PWMPIN, 128);
}

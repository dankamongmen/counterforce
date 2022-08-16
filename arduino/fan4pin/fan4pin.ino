volatile unsigned Pulses; // counter for input events, reset each second

const int RPMPIN = 2; // pin connected to tachometer
//const int PWMPIN = 9; // pin connected to PWM

static void rpm(){
  Pulses++;
}

/*
//configure Timer 1 (pins 9,10) to output 25kHz PWM
static void setupTimer1(){
    //Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << CS10) | (1 << WGM13);
    ICR1 = 320;
    OCR1A = 0;
    OCR1B = 0;
}

//configure Timer 2 (pin 3) to output 25kHz PWM. Pin 11 will be unavailable for output in this mode
static void setupTimer2(){
    //Set PWM frequency to about 25khz on pin 3 (timer 2 mode 5, prescale 8, count to 79)
    TIMSK2 = 0;
    TIFR2 = 0;
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << WGM22) | (1 << CS21);
    OCR2A = 79;
    OCR2B = 0;
}

//equivalent of analogWrite on pin 9
static void setPWM1A(float f){
    f=f<0?0:f>1?1:f;
    OCR1A = (uint16_t)(320*f);
}

//equivalent of analogWrite on pin 10
static void setPWM1B(float f){
    f=f<0?0:f>1?1:f;
    OCR1B = (uint16_t)(320*f);
}

//equivalent of analogWrite on pin 3
static void setPWM2(float f){
    f=f<0?0:f>1?1:f;
    OCR2B = (uint8_t)(79*f);
}
*/

void setup(){
  Serial.begin(115200);
  pinMode(RPMPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RPMPIN), rpm, RISING);
  /*
  //enable outputs for Timer 1
  pinMode(PWMPIN, OUTPUT); //1A
  //pinMode(10,OUTPUT); //1B
  //setupTimer1();
  //enable outputs for Timer 2
  //pinMode(3,OUTPUT); //2
  //setupTimer2();
  //note that pin 11 will be unavailable for output in this mode!
  */
}
 
void loop (){
  Pulses = 0;
  unsigned long m = micros();
  unsigned long cur;

/*
  for(int i=0; i<255; i++){
    analogWrite(PWMPIN, i);
    delay(500);
  }
 
  //setPWM1A(0.4f); //set duty to 50% on pin 9
  //setPWM1B(0.4f); //set duty to 50% on pin 10
  //setPWM2(0.4f); //set duty to 80% on pin 3
  */

  sei();       // enable interrupts
  do{
    cur = micros();
  }while(cur - m < 1000000); // FIXME handle overflow
  cli();       // disable interrupts
  int c = Pulses * 30;
  Serial.print(c, DEC);
  Serial.print(" rpm gunk\r\n");
}

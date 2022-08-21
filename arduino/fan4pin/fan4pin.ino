// requires an Arduino Uno, possibly rev3, possibly only the authentic one.
// a Mega2560 does not seem to work (though it was a clone...)

volatile unsigned Pulses; // counter for input events, reset each second

// tachometer needs an interrupt-capable digital pin. on Mega,
// this is 2, 3, 18, 19, 20, 21 (last two conflict with i2c).
// on Uno, only 2 and 3 are available!
const int RPMPIN = 2; // pin connected to tachometer

// we need a digital output pin for PWM.
const int PWMPIN = 9;

const word PWM_FREQ_HZ = 25000;
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);
const byte FIXED_PWM = 80; // FIXME yuck

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
  Serial.begin(115200);

  pinMode(RPMPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPMPIN), rpm, RISING);
  Serial.print("tachometer read on ");
  Serial.println(RPMPIN);

  pinMode(PWMPIN, OUTPUT);
  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Set Timer1 configuration
  // COM1A(1:0) = 0b10   (Output A clear rising/set falling)
  // COM1B(1:0) = 0b00   (Output B normal operation)
  // WGM(13:10) = 0b1010 (Phase correct PWM)
  // ICNC1      = 0b0    (Input capture noise canceler disabled)
  // ICES1      = 0b0    (Input capture edge select disabled)
  // CS(12:10)  = 0b001  (Input clock select = clock/1)

  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;
  Serial.print("pwm write on");
  Serial.println(PWMPIN);
  Serial.print("PWM to ");
  Serial.println(FIXED_PWM);

  ADMUX = 0xc8; // enable internal temperature sensor via ADC
}

void setPWM(byte pwm){
  OCR1A = (word)(pwm * TCNT1_TOP) / 100;
}

int readTemp(void){
  ADCSRA |= _BV(ADSC);
  while(bit_is_set(ADCSRA, ADSC)){
    ;
  }
  return (ADCL | (ADCH << 8)) - 342;
}

void loop (){
  setPWM(FIXED_PWM);
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

  int temp = readTemp();
  Serial.print("Internal temp: ");
  Serial.println(temp);
}

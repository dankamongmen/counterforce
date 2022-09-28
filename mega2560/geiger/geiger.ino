// use an Arduino Mega to read from RHElectronics geiger counter and KY-038
// sound level sensor. the former needs 5V; the latter can get by with 3.3V.
#include <float.h>

#define RADPIN 19  // digital pin 18 on INT4
#define MICPIN A0 // analog input 0

// we want a sample per second
#define SAMPLEMS 1000

volatile unsigned long count; // disintegration events this quantum
unsigned long previousMillis; // variable for time measurement
unsigned long totalCount;

void tube_impulse(){
  ++count;
}

void setup(){
  count = 0;
  Serial.begin(115200);
  pinMode(RADPIN, INPUT_PULLUP); // set pin INT4 input for capturing GM Tube events
  //digitalWrite(RADPIN, HIGH); // turn on internal pullup resistors, solder C-INT on the PCB
  attachInterrupt(digitalPinToInterrupt(RADPIN), tube_impulse, FALLING); // define external interrupts
  pinMode(MICPIN, INPUT);
  previousMillis = millis();
  totalCount = 0;
}

void loop(){
  static float maxvolquant = FLT_MIN;
  static float minvolquant = FLT_MAX;
  unsigned long now = millis();
  // need to keep everything unsigned long so that we properly
  // handle overflow of millis() at ~50 days
  float candvol = analogRead(MICPIN);
  if(candvol > maxvolquant){
    maxvolquant = candvol;
  }
  if(candvol < minvolquant){
    minvolquant = candvol;
  }
  if(now - previousMillis > SAMPLEMS){
    noInterrupts();
    previousMillis = now;
    unsigned long thiscount = count;
    count = 0;
    interrupts();

    totalCount += thiscount;
    Serial.print("Count: ");
    Serial.println(thiscount);

    Serial.print("Mic: ");
    float peakpeak = maxvolquant - minvolquant;
    Serial.println(peakpeak);
    maxvolquant = FLT_MIN;
    minvolquant = FLT_MAX;
  }
}

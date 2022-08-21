// read from RHElectronics geiger counter and
// KY-038 sound level sensor

#include <SPI.h>

#define RADPIN 2
#define MICPIN A0

// we want a count per minute
#define SAMPLEMS 60000

volatile unsigned long count; // GM Tube events
unsigned long previousMillis; // variable for time measurement

void tube_impulse(){
  ++count;
}

void setup(){
  count = 0;
  Serial.begin(115200);
  while(!Serial);
  pinMode(RADPIN, INPUT);     // set pin INT0 input for capturing GM Tube events
  digitalWrite(RADPIN, HIGH); // turn on internal pullup resistors, solder C-INT on the PCB
  attachInterrupt(digitalPinToInterrupt(RADPIN), tube_impulse, FALLING); // define external interrupts
  pinMode(MICPIN, INPUT);
}

void loop(){
  unsigned long now = millis();
  // need to keep everything unsigned long so that we properly
  // handle overflow of millis() at ~50 days
  if(now - previousMillis > SAMPLEMS){
    previousMillis = now;
    Serial.print(count);
    Serial.println(" counts");
    count = 0;
  }
}

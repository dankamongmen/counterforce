// originally from https://www.rhelectronics.store/radiation-detector-geiger-counter-diy-kit-second-edition

#include <SPI.h>

#define RADPIN 2

#define LOG_PERIOD 15000  // logging period in milliseconds, recommended value 15000-60000.
#define MAX_PERIOD 60000  // maximum logging period

unsigned long counts;         // variable for GM Tube events
unsigned long cpm;            // variable for CPM
unsigned int multiplier;      // variable for calculation CPM in this sketch
unsigned long previousMillis; // variable for time measurement

void tube_impulse(){
  counts++;
}

void setup(){
  counts = 0;
  cpm = 0;
  multiplier = MAX_PERIOD / LOG_PERIOD;      // calculating multiplier, depend on your log period
  Serial.begin(115200);                      // start serial monitor
  while(!Serial);
  pinMode(RADPIN, INPUT);                    // set pin INT0 input for capturing GM Tube events
  digitalWrite(RADPIN, HIGH);                // turn on internal pullup resistors, solder C-INT on the PCB
  attachInterrupt(digitalPinToInterrupt(RADPIN), tube_impulse, FALLING); // define external interrupts
}

void loop(){
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > LOG_PERIOD){
    previousMillis = currentMillis;
    cpm = counts * multiplier;
    Serial.print(cpm);
    Serial.println(" cpm");
    counts = 0;
  }
}

#include <SPI.h>

#define RADPIN 2

#define LOG_PERIOD 15000  // logging period in milliseconds, recommended value 15000-60000.

volatile unsigned long count; // GM Tube events
unsigned int multiplier;      // variable for calculation CPM in this sketch
unsigned long previousMillis; // variable for time measurement

void tube_impulse(){
  count++;
}

void setup(){
  count = 0;
  Serial.begin(115200);
  while(!Serial);
  pinMode(RADPIN, INPUT);     // set pin INT0 input for capturing GM Tube events
  digitalWrite(RADPIN, HIGH); // turn on internal pullup resistors, solder C-INT on the PCB
  attachInterrupt(digitalPinToInterrupt(RADPIN), tube_impulse, FALLING); // define external interrupts
}

void loop(){
  // FIXME need to handle overflow every 50 days
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > LOG_PERIOD){
    previousMillis = currentMillis;
    Serial.print(count);
    Serial.println(" counts");
    count = 0;
  }
}

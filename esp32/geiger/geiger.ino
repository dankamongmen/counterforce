// use a Heltect ESP32-LoRAv2 to read from RHElectronics geiger counter and
// KY-038 sound level sensor. the former needs 5V; the latter can get by with
// our native 3.3V.

#define RADPIN 36
#define MICPIN 39 // analog input

// we want a sample per second
#define SAMPLEMS 1000

#include "heltec.h"

volatile unsigned long count; // disintegration events this quantum
unsigned long previousMillis; // variable for time measurement
unsigned long totalCount;

void tube_impulse(){
  ++count;
}

void setup(){
  count = 0;
  Heltec.begin(true  /*DisplayEnable Enable*/,
               false /*LoRa Disable*/,
               true  /*Serial Enable*/);
  Heltec.display->setFont(ArialMT_Plain_10);
  pinMode(RADPIN, INPUT);     // set pin INT0 input for capturing GM Tube events
  digitalWrite(RADPIN, HIGH); // turn on internal pullup resistors, solder C-INT on the PCB
  attachInterrupt(digitalPinToInterrupt(RADPIN), tube_impulse, FALLING); // define external interrupts
  pinMode(MICPIN, INPUT);
  previousMillis = 0;
  totalCount = 0;
}

void loop(){
  unsigned long now = millis();
  // need to keep everything unsigned long so that we properly
  // handle overflow of millis() at ~50 days
  if(now - previousMillis > SAMPLEMS){
    previousMillis = now;
    unsigned long thiscount = count;
    count = 0;
    totalCount += thiscount;
    Serial.print("Count: ");
    Serial.println(thiscount);

    float a = analogRead(MICPIN);
    Serial.print("Mic: ");
    Serial.println(a);
    Heltec.display->clear();
    Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
    Heltec.display->drawString(0, 0, "Disintegrations: ");
    Heltec.display->drawString(77, 0, String(thiscount));
    Heltec.display->drawString(0, 11, "Total: ");
    Heltec.display->drawString(35, 11, String(totalCount));
    Heltec.display->drawString(0, 21, "Noise: ");
    Heltec.display->drawString(35, 21, String(a));
    Heltec.display->drawString(0, 31, "Uptime: ");
    Heltec.display->drawString(40, 31, String(millis() / 1000));
    Heltec.display->display();
  }
}

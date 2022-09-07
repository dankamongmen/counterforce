// use an Arduino Mega to read from RHElectronics geiger counter and KY-038
// sound level sensor. the former needs 5V; the latter can get by with 3.3V.

#define RADPIN 2  // digital input 2 on INT4
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
  previousMillis = 0;
  totalCount = 0;
}

void loop(){
  unsigned long now = millis();
  // need to keep everything unsigned long so that we properly
  // handle overflow of millis() at ~50 days
  if(now - previousMillis > SAMPLEMS){
    noInterrupts();
    previousMillis = now;
    unsigned long thiscount = count;
    count = 0;
    interrupts();

    totalCount += thiscount;
    Serial.print("Count: ");
    Serial.println(thiscount);

    float a = analogRead(MICPIN);
    Serial.print("Mic: ");
    Serial.println(a);
  }
}

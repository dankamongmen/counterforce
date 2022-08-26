// intended for use on a Heltec ESP32LoRav2
#include "heltec.h"
#include <float.h>
#include "EspMQTTClient.h"

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

void onConnectionEstablished() {
  Serial.println("Got an MQTT connection");
}

volatile unsigned Pulses; // counter for input events, reset each second

// tachometer needs an interrupt-capable digital pin. on Mega,
// this is 2, 3, 18, 19, 20, 21 (last two conflict with i2c).
// on Uno, only 2 and 3 are available! all ESP32 pins can
// drive interrupts.
const int RPMPIN = 36; // pin connected to tachometer

// we'll use PWM channel 14, through digital pin 36
const int PWMPIN = 39;
const int PWMCHANNEL = 14;

// Intel spec for PWM fans demands a 25K frequency.
const word PWM_FREQ_HZ = 25000;

int Pwm;

// thermistor analog input pin
const int TEMPPIN = 37;

// on mega:
//  pin 13, 4 == timer 0 (used for micros())
//  pin 12, 11 == timer 1
//  pin 10, 9 == timer 2
//  pin 5, 3, 2 == timer 3
//  pin 8, 7, 6 == timer 4

static void rpm(){
  if(Pulses + 1 > Pulses){
    ++Pulses;
  }
}

void setup(){
  const byte INITIAL_PWM = 40;
  Heltec.begin(true  /*DisplayEnable Enable*/,
               false /*LoRa Disable*/,
               true  /*Serial Enable*/);
  Heltec.display->setFont(ArialMT_Plain_10);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  
  pinMode(PWMPIN, OUTPUT);
  ledcSetup(PWMCHANNEL, PWM_FREQ_HZ, 8);
  ledcAttachPin(PWMPIN, PWMCHANNEL);
  Serial.print("pwm write on ");
  Serial.println(PWMPIN);
  setPWM(INITIAL_PWM);

  Pulses = 0;
  pinMode(RPMPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RPMPIN), rpm, RISING);
  Serial.print("tachometer read on ");
  Serial.println(RPMPIN);

  // on arduino, use the 3.3V level for reading the thermistor,
  // as it ought be cleaner than 5V. connect 3.3 to AREF, and
  // use analogReference(EXTERNAL).
  pinMode(TEMPPIN, INPUT);
}

void setPWM(byte pwm){
  Serial.print("PWM to ");
  Serial.println(pwm);
  Pwm = pwm;
  ledcWrite(PWMCHANNEL, (Pwm / 100) * 255); // from specified 8-bit resolution
}

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

// returns FLT_MAX if we couldn't get a valid read. otherwise,
// returns the approximate temperature in celsius.
float readThermistor(void){
  const int BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 25;
  const float R1 = 10;
  const float VREF = 3.3;
  const float ADCRES = 4095.0; // 12-bit ADC
  float v0 = analogRead(TEMPPIN);
  Serial.print("read raw voltage: ");
  if(v0 == ADCRES || v0 == 0){
    Serial.println("n/a");
    return FLT_MAX;
  }
  Serial.println(v0);
  float scaled = v0 * (VREF / ADCRES);
  //Serial.print(" scaled: ");
  //Serial.println(scaled);
  float R = (scaled * R1) / (VREF - scaled);
  float t = 1.0 / ((1.0 / NOMINAL) + ((log(R / 10)) / BETA));
  Serial.print("read raw temp: ");
  Serial.println(t);
  return t;
}

const unsigned long LOOPUS = 1000000;

void displayConnectionStatus(int y){
  const char* connstr;
  if(!client.isWifiConnected()){
    connstr = "No WiFi";
  }else if(!client.isMqttConnected()){
    connstr = "WiFi, no MQTT";
  }else{
    connstr = "Connected";
  }
  Heltec.display->drawString(120, y, connstr);
}

void loop(){
  // FIXME use the enqueue work version of this, as client.loop() can
  // block for arbitrary amounts of time
  client.loop(); // handle any necessary wifi/mqtt
  unsigned long m = micros();
  unsigned long cur;

  do{
    cur = micros();
    if(cur < m){     // handle micros() overflow...
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
    c = 65535;
  }else{
    c = p * 30;
  }

  // dump information to serial
  if(c >= 8192){
    Serial.print("invalid RPM read: ");
    Serial.print(p);
  }else{
    Serial.print(RPMPIN, DEC);
    Serial.print(" ");
    Serial.print(p, DEC);
    Serial.print(" ");
    Serial.print(c, DEC);
    Serial.print(" rpm");
  }
  Serial.print(' ');
  Serial.print(cur - m, DEC);
  Serial.println("µs");
  float therm = readThermistor();
  Serial.print("Thermistor: ");
  if(therm == FLT_MAX){
    Serial.print("n/a");
  }else{
    Serial.print(therm);
  }
  Serial.print(" PWM output: ");
  Serial.print(Pwm);
  Serial.println();

  // dump information to OLED
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->drawString(0, 0, "RPM: ");
  Heltec.display->drawString(30, 0, String(c));
  Heltec.display->drawString(0, 11, "PWM: ");
  Heltec.display->drawString(34, 11, String(Pwm));
  Heltec.display->drawString(0, 21, "Temp: ");
  if(therm == FLT_MAX){
    Heltec.display->drawString(36, 21, "n/a");
  }else{
    Heltec.display->drawString(36, 21, String(therm));
  }
  Heltec.display->drawString(0, 31, "Uptime: ");
  Heltec.display->drawString(40, 31, String(millis() / 1000));
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  displayConnectionStatus(51);
  Heltec.display->display();

  if(c != 65535){
    client.publish("mora3/rpms", String(c));
  }
  if(therm != FLT_MAX){
    client.publish("mora3/therm", String(therm));
  }

  check_pwm_update();
}

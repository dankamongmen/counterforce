// intended for use on a Heltec ESP32LoRav2, this manages (via UART) a device
// controlling a PWM fan. it receives PWM control messages, and sends RPM and
// temperature reports, over MQTT.
#include "heltec.h"
#include <float.h>
#include "EspMQTTClient.h"

const unsigned long UARTSPEED = 9600;
const int INITIAL_PWM = 50;
const int UartTX = 37;
const int UartRX = 38;
int Pwm = -1;
int RPM = INT_MAX;
float Therm = FLT_MAX;

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
}

void setup(){
  Heltec.begin(true  /*DisplayEnable Enable*/,
               false /*LoRa Disable*/,
               true  /*Serial Enable*/);
  Heltec.display->setFont(ArialMT_Plain_10);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  Serial2.begin(UARTSPEED, SERIAL_8N1, UartRX, UartTX);
  setPWM(INITIAL_PWM);
}

// write a PWM value over UART
static void send_pwm(int pwm){
}

static int setPWM(int pwm){
  if(pwm < 0 || pwm > 100){
    Serial.print("invalid pwm: ");
    Serial.println(pwm);
    return -1;
  }
  Serial.print("PWM to ");
  Serial.println(pwm);
  Pwm = pwm;
  send_pwm(pwm);
  return 0;
}

// read bytes from Serial. each byte is interpreted as a PWM level, and
// ought be between [0..100]. we act on the last byte available.
static void check_pwm_update(void){
  int last = -1;
  int in;
  while((in = Serial.read()) != -1){
    Serial.print("read byte from input: ");
    Serial.println(in);
    last = in;
  }
  if(last >= 0){
    setPWM(last);
  }
}

static void displayConnectionStatus(int y){
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

// up to 2 digits of years, up to 3 digits of days, up to 2 digits of
// hours, up to 2 digits of minutes, up to 2 digits of seconds, and up
// to 5 units, plus null == 17 bytes
#define MAXTIMELEN 17

// timestr needs be MAXTIMELEN bytes or more
static int maketimestr(char *timestr){
  unsigned long t = millis() / 1000; // FIXME deal with rollover
  unsigned long epoch = 365ul * 24 * 60 * 60;
  int off = 0; // write offset
  if(t > epoch){
    word years = t / epoch;
    if(years > 99){
      return -1; // whatever bro
    }
    off = sprintf(timestr, "%uy", years);
  }
  t %= epoch;
  epoch /= 365;
  if(t > epoch){
    word d = t / epoch;
    off += sprintf(timestr + off, "%ud", d);
  }
  t %= epoch;
  epoch /= 24;
  if(t > epoch){
    word h = t / epoch;
    off += sprintf(timestr + off, "%ud", h);
  }
  t %= epoch;
  epoch /= 60;
  if(t > epoch){
    word m = t / epoch;
    off += sprintf(timestr + off, "%um", m);
  }
  t %= epoch;
  off += sprintf(timestr + off, "%us", t);
  return 0;
}

static void updateDisplay(void){
  char timestr[MAXTIMELEN];
  // dump information to OLED
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->drawString(0, 0, "RPM: ");
  if(RPM == INT_MAX){
    Heltec.display->drawString(31, 0, "n/a");
  }else{
    Heltec.display->drawString(31, 0, String(RPM));
  }
  Heltec.display->drawString(0, 11, "PWM: ");
  Heltec.display->drawString(33, 11, String(Pwm));
  Heltec.display->drawString(0, 21, "Temp: ");
  if(Therm == FLT_MAX){
    Heltec.display->drawString(35, 21, "n/a");
  }else{
    Heltec.display->drawString(35, 21, String(Therm));
  }
  Heltec.display->drawString(0, 31, "Uptime: ");
  if(maketimestr(timestr) == 0){
    Heltec.display->drawString(41, 31, maketimestr(timestr) ? "a long time" : timestr);
  }
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  displayConnectionStatus(51);
  Heltec.display->display();
}

void loop(){
  // FIXME use the enqueue work version of this, as client.loop() can
  // block for arbitrary amounts of time
  client.loop(); // handle any necessary wifi/mqtt
  // FIXME ideally only run if things have changed
  updateDisplay();
  // FIXME if not connected, retain a buffer of values
  if(RPM != INT_MAX){
    client.publish("mora3/rpms", String(RPM));
  }
  if(Therm != FLT_MAX){
    client.publish("mora3/therm", String(Therm));
  }
  check_pwm_update();
  int w = Serial2.write(Pwm);
  Serial.print("write: ");
  Serial.println(w);
}

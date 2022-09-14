// intended for use on a Heltec ESP32LoRav2, this manages (via UART) a device
// controlling a PWM fan. it receives PWM control messages, and sends RPM and
// temperature reports, over MQTT.
#include "heltec.h"
#include <float.h>
#include "EspMQTTClient.h"

const unsigned long UARTSPEED = 9600;
const int INITIAL_PWM = 50;
const int UartTX = 17;
const int UartRX = 36;
int Pwm = -1;
int RPM = INT_MAX;
float Therm = FLT_MAX;

HardwareSerial UART(1);

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
}

void setup(){
  Heltec.begin(true, false, true);
  Heltec.display->setFont(ArialMT_Plain_10);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  UART.begin(UARTSPEED, SERIAL_8N1, UartRX, UartTX);
  setPWM(INITIAL_PWM);
}

// write the desired PWM value over UART
static void send_pwm(void){
  Serial.println("sending PWM over UART");
  UART.print(Pwm);
}

static int setPWM(int pwm){
  if(pwm < 0 || pwm > 255){
    Serial.print("invalid pwm: ");
    Serial.println(pwm);
    return -1;
  }
  Serial.print("PWM to ");
  Serial.println(pwm);
  Pwm = pwm;
  return 0;
}

// read bytes from Serial. each byte is interpreted as a PWM level, and
// ought be between [0..255]. we act on the last byte available. we
static void check_pwm_update(void){
  int last = -1;
  int in;
  // FIXME also need to check MQTT
  while((in = Serial.read()) != -1){
    Serial.print("read byte from input: ");
    Serial.println(in);
    last = in;
  }
  if(last >= 0){
    setPWM(last);
  }
}

// simple state machine around protocol of:
// top: { stat }*
// state: [T] float | [R] posint | [P] ubyte
// float: digit+.digit+
// all exceptions return to top
static void check_state_update(void){
  static enum {
    STATE_BEGIN,
    STATE_PWM,
    STATE_RPM,
    STATE_TEMP,
  } state = STATE_BEGIN;
  static int int_ongoing;
  static float float_ongoing;
  int last = -1;
  int in;
  while((in = UART.read()) != -1){
    //Serial.print("read byte from UART!!! ");
    //Serial.println(in, DEC);
    switch(state){
      case STATE_BEGIN:
        if(in == 'T'){
          state = STATE_TEMP;
          float_ongoing = 0;
        }else if(in == 'R'){
          state = STATE_RPM;
          int_ongoing = 0;
        }else if(in == 'P'){
          state = STATE_PWM;
          int_ongoing = 0;
        }else{
          Serial.println("unexpected stat prefix");
        }
        break;
      case STATE_PWM:
        if(isdigit(in)){
          int_ongoing *= 10;
          int_ongoing += in - '0';
        }else if(in == 'T'){
          state = STATE_TEMP;
          Pwm = int_ongoing;
          float_ongoing = 0;
        }else if(in == 'R'){
          state = STATE_RPM;
          Pwm = int_ongoing;
          int_ongoing = 0;
        }else{
          state = STATE_BEGIN;
        }
        break;
      case STATE_RPM:
        if(isdigit(in)){
          int_ongoing *= 10;
          int_ongoing += in - '0';
        }else if(in == 'T'){
          state = STATE_TEMP;
          RPM = int_ongoing;
          float_ongoing = 0;
        }else if(in == 'P'){
          state = STATE_PWM;
          RPM = int_ongoing;
          int_ongoing = 0;
        }else{
          state = STATE_BEGIN;
        }
        break;
      case STATE_TEMP:
        if(isdigit(in)){
          float_ongoing *= 10;
          float_ongoing += in - '0';
        }else if(in == '.'){
          // FIXME
        }else if(in == 'R'){
          state = STATE_RPM;
          Therm = float_ongoing;
          int_ongoing = 0;
        }else if(in == 'P'){
          state = STATE_PWM;
          Therm = float_ongoing;
          int_ongoing = 0;
        }else{
          state = STATE_BEGIN;
        }
        break;
      default:
        Serial.println("invalid lexer state!");
        state = STATE_BEGIN;
        break;
    }
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
  Heltec.display->drawString(41, 31, maketimestr(timestr) ? "a long time" : timestr);
Serial.print("uptime: ");
Serial.println(timestr);
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  displayConnectionStatus(51);
  Heltec.display->display();
}

void loop(){
  // millis() when we last sent a PWM directive over the UART. we send it
  // frequently, in case the arduino has only just come online, but we don't
  // want to drown the partner in UART content. 
  static unsigned long last_pwm_write = 0;
  static int lastRPM = INT_MAX;
  static float lastTherm = FLT_MAX;
  static int lastPWM = -1;
  // only update the screen on a change, or in a new second (resolution
  // of the uptime string used on the display)
  static unsigned long last_screen_second = 0;
  // FIXME use the enqueue work version of this, as client.loop() can
  // block for arbitrary amounts of time
  client.loop(); // handle any necessary wifi/mqtt
  check_state_update();
  check_pwm_update();
  bool change = false;
  unsigned long m = millis();
  if(lastPWM != Pwm){
    lastPWM = Pwm;
    last_pwm_write = m;
    send_pwm();
    change = true;
  }
  if(m < last_pwm_write || m - last_pwm_write > 1000){
    last_pwm_write = m;
    send_pwm();
  }
  if(lastRPM != RPM){
    lastRPM = RPM;
    if(RPM != INT_MAX){
      client.publish("mora3/rpms", String(RPM));
    }
    change = true;
  }
  if(lastTherm != Therm){
    lastTherm = Therm;
    if(Therm != FLT_MAX){
      client.publish("mora3/therm", String(Therm));
    }
    change = true;
  }
  m /= 1000;
  if(m != last_screen_second){
    last_screen_second = m;
    change = true;
  }
  if(change){
    updateDisplay();
  }
}

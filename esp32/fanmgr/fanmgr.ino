// intended for use on a Heltec ESP32LoRav2, this manages (via UART) a device
// controlling a PWM fan. it receives PWM control messages, and sends RPM and
// temperature reports, over MQTT.
#include "heltec.h"
#include <float.h>
#include "EspMQTTClient.h"
#include <ArduinoJson.h>

const unsigned long UARTSPEED = 9600;
const int INITIAL_PWM = 50;
const int UartTX = 17;
const int UartRX = 36;
int Pwm = -1;
int Red = -1;
int Green = -1;
int Blue = -1;
int RPM = INT_MAX;
float Therm = FLT_MAX;

HardwareSerial UART(1);

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

void setup(){
  Heltec.begin(true, false, true);
  Heltec.display->setFont(ArialMT_Plain_10);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  UART.begin(UARTSPEED, SERIAL_8N1, UartRX, UartTX);
  setPWM(INITIAL_PWM);
}

// write the desired PWM value over UART (1 byte + label 'P')
static void send_pwm(void){
  Serial.println("sending PWM over UART");
  UART.write('P');
  UART.write(Pwm); // send as single byte
}

// write the desired RGB values over UART (3 bytes + label 'C');
static void send_rgb(void){
  Serial.println("sending RGB over UART");
  UART.write('C');
  UART.write(Red);
  UART.write(Green);
  UART.write(Blue);
}

// precondition: isxdigit(c) is true
static byte getHex(char c){
  if(isdigit(c)){
    return c - '0';
  }
  c = tolower(c);
  return c - 'a' + 10;
}

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
  client.subscribe("mora3/rgb", [](const String &payload){
      Serial.print("received RGB via mqtt: ");
      Serial.println(payload);
      byte colors[3]; // rgb
      if(payload.length() != 2 * sizeof(colors)){
        Serial.println("RGB wasn't 6 characters");
        return;
      }
      for(int i = 0 ; i < sizeof(colors) ; ++i){
        char h = payload.charAt(i * 2);
        char l = payload.charAt(i * 2 + 1);
        if(!isxdigit(h) || !isxdigit(l)){
          Serial.println("invalid RGB character");
          return;
        }
        byte hb = getHex(h);
        byte lb = getHex(l);
        colors[i] = hb * 16 + lb;
      }
      // everything was valid; update globals
      Red = colors[0];
      Green = colors[1];
      Blue = colors[2];
      send_rgb();
    }
  );
  client.subscribe("mora3/pwm", [](const String &payload){
      Serial.print("received PWM via mqtt: ");
      Serial.println(payload);
      unsigned long p = 0;
      if(payload.length() == 0){
        Serial.println("empty PWM input");
        return;
      }
      for(int i = 0 ; i < payload.length() ; ++i){
        char h = payload.charAt(i);
        if(!isdigit(h)){
          Serial.println("invalid PWM character");
          return;
        }
        p *= 10; // FIXME check for overflow
        p += h - '0';
      }
      setPWM(p);
    }
  );
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
    STATE_TEMP_MANTISSA,
  } state = STATE_BEGIN;
  static int int_ongoing;
  static float float_ongoing;
  static int divisor_exponent; // used for float mantissa
  int last = -1;
  int in;
  while((in = UART.read()) != -1){
    //Serial.print("read byte from UART!!! ");
    //Serial.println(in, HEX);
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
          Serial.print("unexpected stat prefix: ");
          Serial.println(in, HEX);
        }
        break;
      case STATE_PWM:
        if(isdigit(in)){
          int_ongoing *= 10;
          int_ongoing += in - '0';
        }else if(in == 'T'){
          state = STATE_TEMP;
          if(Pwm != int_ongoing){
            send_pwm();
          }
          float_ongoing = 0;
        }else if(in == 'R'){
          state = STATE_RPM;
          if(Pwm != int_ongoing){
            send_pwm();
          }
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
          int_ongoing = 0;
          divisor_exponent = 0;
          state = STATE_TEMP_MANTISSA;
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
      case STATE_TEMP_MANTISSA:
        if(isdigit(in)){
          int_ongoing *= 10;
          int_ongoing += in - '0';
          ++divisor_exponent;
        }else if(in == 'R'){
          state = STATE_RPM;
          Therm = float_ongoing + int_ongoing / pow(10, divisor_exponent);
          int_ongoing = 0;
        }else if(in == 'P'){
          state = STATE_PWM;
          Therm = float_ongoing + int_ongoing / pow(10, divisor_exponent);
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
static int maketimestr(char *timestr, unsigned long m){
  unsigned long t = m / 1000; // FIXME deal with rollover
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

int mqttPublish(EspMQTTClient& mqtt, const char* key, const String& value){
  DynamicJsonDocument doc(BUFSIZ); // FIXME
  doc[key] = value;
  // PubSubClient limits messages to 256 bytes
  char buf[257];
  size_t n = serializeJson(doc, buf);
  mqtt.publish("mora3", buf, n);
  return 0;
}

// m is millis()
static void updateDisplay(unsigned long m){
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
  Heltec.display->drawString(41, 31, maketimestr(timestr, m) ?
                             "a long time" : timestr);
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  displayConnectionStatus(51);
  Heltec.display->display();
}

void loop(){
  // millis() when we last sent a PWM directive over the UART. we send it
  // frequently, in case the arduino has only just come online, but we don't
  // want to drown the partner in UART content. 
  static unsigned long last_broadcast = 0;
  static int lastRPM = INT_MAX;
  static float lastTherm = FLT_MAX;
  static int lastPWM = -1;
  // FIXME use the enqueue work version of this, as client.loop() can
  // block for arbitrary amounts of time
  client.loop(); // handle any necessary wifi/mqtt
  check_state_update();
  check_pwm_update();
  unsigned long m = millis();
  updateDisplay(m);
  bool broadcast = false;
  if(m < last_broadcast || m - last_broadcast > 1000){
    broadcast = true;
    last_broadcast = m;
    lastPWM = Pwm;
    send_pwm();
    send_rgb();
  }else if(lastPWM != Pwm){
    lastPWM = Pwm;
    last_broadcast = m;
    send_pwm();
  }
  if(lastRPM != RPM || broadcast){
    lastRPM = RPM;
    if(RPM != INT_MAX){
      mqttPublish(client, "rpm", String(RPM));
    }
  }
  if(lastTherm != Therm || broadcast){
    lastTherm = Therm;
    if(Therm != FLT_MAX){
      mqttPublish(client, "therm", String(Therm));
    }
  }
}

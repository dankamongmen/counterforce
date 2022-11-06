// intended for use on a Heltec ESP32LoRav2, this manages (via UART) a device
// controlling a PWM fan. it receives PWM control messages, and sends RPM and
// temperature reports, over MQTT.
#include "heltec.h"
#include <float.h>
#include "EspMQTTClient.h"
#include <ArduinoJson.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <DallasTemperature.h>

const unsigned long RPM_CUTOFF = 10000;
const unsigned long UARTSPEED = 9600;
const int INITIAL_PWM = 0;
const int UartTX = 17;
const int UartRX = 36;
// coolant thermistor (2-wire)
const int TEMPPIN = 39;
// ambient temperature (digital thermometer, Dallas 1-wire)
const int AMBIENTPIN = 23;

// PWM channels for RGB fans
const ledc_channel_t FANCHANR = LEDC_CHANNEL_0;
const ledc_channel_t FANCHANG = LEDC_CHANNEL_1;
const ledc_channel_t FANCHANB = LEDC_CHANNEL_2;
const ledc_channel_t FANCHANPWM = LEDC_CHANNEL_3;
const int FANPWMPIN = 37;
const int FANTACHPIN = 38;
const int RGBPINR = 35;
const int RGBPING = 32;
const int RGBPINB = 33;

// RGB we want for the 12V fan LEDs (initialized to green, read from MQTT)
int Red = 0;
int Green = 0xff;
int Blue = 0;

// RPMs as reported to us by the client for fans and the two XTop Dual pumps.
// we only get the RPM count from one of our fans; it stands for all.
int RPM = INT_MAX;
int XTopRPMA = INT_MAX;
int XTopRPMB = INT_MAX;
// PWM we want the client to run at (initialized to 0, read from MQTT)
int Pwm = 0;
int XTopPWMA = 0;
int XTopPWMB = 0;
// PWM as reported to us by the client (ought match what we request)
int ReportedPwm = INT_MAX;

// did we hit an error in initialization? only valid following setup().
int InitError;

HardwareSerial UART(1);

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

void readThermistor(float* t){
  const int BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 25;
  const float R1 = 10000;
  const float VREF = 3.3;
  float v0 = analogRead(TEMPPIN);
  Serial.print("read raw voltage: ");
  Serial.print(v0);
  if(v0 == 0){
    Serial.println(", throwing it out");
    return;
  }
  // 12-bit ADC on the ESP32
  float scaled = v0 * (VREF / 4095.0);
  Serial.print(" scaled: ");
  Serial.print(scaled);
  float R = ((scaled * R1) / (VREF - scaled)) / R1;
  Serial.print(" R: ");
  Serial.println(R);
  float tn = 1.0 / ((1.0 / NOMINAL) - ((log(R)) / BETA));
  Serial.print("read raw temp: ");
  Serial.println(tn);
  *t = tn;
}

int initialize_pwm(ledc_channel_t channel, int pin, int freq){
  ledc_channel_config_t conf;
  memset(&conf, 0, sizeof(conf));
  conf.gpio_num = pin;
  conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  conf.intr_type = LEDC_INTR_DISABLE;
  conf.timer_sel = LEDC_TIMER_0;
  conf.duty = 8;
  conf.channel = channel;
  Serial.print("setting up pin ");
  Serial.print(pin);
  Serial.print(" for ");
  Serial.print(freq);
  Serial.print("Hz PWM...");
  if(ledc_channel_config(&conf) != ESP_OK){
    Serial.println("error!");
    return -1;
  }
  Serial.println("success!");
  return 0;
}

int initialize_rgb_pwm(ledc_channel_t channel, int pin){
  return initialize_pwm(channel, pin, 5000);
}

int initialize_fan_pwm(ledc_channel_t channel, int pin){
  return initialize_pwm(channel, pin, 25000);
}

void setup(){
  int error = 0;
  Heltec.begin(true, false, true);
  Heltec.display->setFont(ArialMT_Plain_10);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  UART.begin(UARTSPEED, SERIAL_8N1, UartRX, UartTX);
  error |= initialize_fan_pwm(FANCHANPWM, FANPWMPIN);
  error |= initialize_rgb_pwm(FANCHANR, RGBPINR);
  error |= initialize_rgb_pwm(FANCHANG, RGBPING);
  error |= initialize_rgb_pwm(FANCHANB, RGBPINB);
  setPWM(INITIAL_PWM);
  pinMode(TEMPPIN, INPUT);
  pinMode(FANTACHPIN, INPUT);
  digtemp.begin();
  Serial.print("1-Wire devices: ");
  Serial.println(digtemp.getDeviceCount());
  Serial.print("DS18xxx devices: ");
  Serial.println(digtemp.getDS18Count());
  InitError = error;
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
  ledcWrite(FANCHANR, Red);
  ledcWrite(FANCHANG, Green);
  ledcWrite(FANCHANB, Blue);
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
  client.subscribe("control/mora3/rgb", [](const String &payload){
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
  client.subscribe("control/mora3/pwm", [](const String &payload){
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
      send_pwm();
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

// simple state machine around protocol of:
// top: { stat }*
// state: [R] posint | [A] posint | [B] posint | [P] ubyte
//  float: digit+.digit+
// t: coolant temp
// r: fan rpm based off previous second
// a: xtop pump 1 rpm
// b: xtop pump 2 rpm
// p: pwm
// all exceptions return to top
static void check_state_update(void){
  static enum {
    STATE_BEGIN,
    STATE_PWM,
    STATE_RPM,
    STATE_XTOPA,
    STATE_XTOPB,
  } state = STATE_BEGIN;
  static int int_ongoing;
  int in;
  while((in = UART.read()) != -1){
    Serial.print("read byte from UART!!! ");
    Serial.println(in, HEX);
    switch(state){
      case STATE_BEGIN:
        if(in == 'R'){
          state = STATE_RPM;
          int_ongoing = 0;
        }else if(in == 'A'){
          state = STATE_XTOPA;
          int_ongoing = 0;
        }else if(in == 'B'){
          state = STATE_XTOPB;
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
        }else if(in == 'A'){
          state = STATE_XTOPA;
          ReportedPwm = int_ongoing;
          if(Pwm != ReportedPwm){
            send_pwm();
          }
          int_ongoing = 0;
        }else if(in == 'B'){
          state = STATE_XTOPB;
          ReportedPwm = int_ongoing;
          if(Pwm != ReportedPwm){
            send_pwm();
          }
          int_ongoing = 0;
        }else if(in == 'R'){
          state = STATE_RPM;
          ReportedPwm = int_ongoing;
          if(Pwm != ReportedPwm){
            send_pwm();
          }
          int_ongoing = 0;
        }else{
          state = STATE_BEGIN;
        }
        break;
      case STATE_XTOPA:
        if(isdigit(in)){
          int_ongoing *= 10;
          int_ongoing += in - '0';
        }else if(in == 'R'){
          state = STATE_RPM;
          XTopRPMA = int_ongoing;
          int_ongoing = 0;
        }else if(in == 'B'){
          state = STATE_XTOPB;
          XTopRPMA = int_ongoing;
          int_ongoing = 0;
        }else if(in == 'P'){
          state = STATE_PWM;
          XTopRPMA = int_ongoing;
          int_ongoing = 0;
        }else{
          state = STATE_BEGIN;
        }
        break;
      case STATE_XTOPB:
        if(isdigit(in)){
          int_ongoing *= 10;
          int_ongoing += in - '0';
        }else if(in == 'R'){
          state = STATE_RPM;
          XTopRPMB = int_ongoing;
          int_ongoing = 0;
        }else if(in == 'A'){
          state = STATE_XTOPA;
          XTopRPMB = int_ongoing;
          int_ongoing = 0;
        }else if(in == 'P'){
          state = STATE_PWM;
          XTopRPMB = int_ongoing;
          int_ongoing = 0;
        }else{
          state = STATE_BEGIN;
        }
        break;
      case STATE_RPM:
        if(isdigit(in)){
          int_ongoing *= 10;
          int_ongoing += in - '0';
        }else if(in == 'A'){
          state = STATE_XTOPA;
          RPM = int_ongoing;
          int_ongoing = 0;
        }else if(in == 'B'){
          state = STATE_XTOPB;
          RPM = int_ongoing;
          int_ongoing = 0;
        }else if(in == 'P'){
          state = STATE_PWM;
          RPM = int_ongoing;
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
  if(InitError){
    Heltec.display->drawString(0, y, "Init Error!");
  }
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
// hours, up to 2 digits of minutes, up to 2 digits of seconds, up
// to 5 units, and up to five spaces, plus null == 22 bytes
#define MAXTIMELEN 22

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
    off = sprintf(timestr, "%uy ", years);
  }
  t %= epoch;
  epoch /= 365;
  if(t > epoch){
    word d = t / epoch;
    off += sprintf(timestr + off, "%ud ", d);
  }
  t %= epoch;
  epoch /= 24;
  if(t > epoch){
    word h = t / epoch;
    off += sprintf(timestr + off, "%uh ", h);
  }
  t %= epoch;
  epoch /= 60;
  if(t > epoch){
    word m = t / epoch;
    off += sprintf(timestr + off, "%um ", m);
  }
  t %= epoch;
  off += sprintf(timestr + off, "%lus ", t);
  return 0;
}

template<typename T>
int mqttPublish(EspMQTTClient& mqtt, const char* key, const T& value){
  DynamicJsonDocument doc(BUFSIZ); // FIXME
  doc[key] = value;
  // PubSubClient limits messages to 256 bytes
  char buf[257];
  size_t n = serializeJson(doc, buf);
  mqtt.publish("sensors/mora3", buf, n);
  return 0;
}

int rpmPublish(EspMQTTClient& mqtt, const char* key, int val, int* lastval,
               bool broadcast){
  if(*lastval != val || broadcast){
    *lastval = val;
    if(val < RPM_CUTOFF){ // filter out obviously incorrect values
      return mqttPublish(client, key, val);
    }
    Serial.println("don't have a valid rpm sample");
  }
  return 0;
}

// at most, two three-digit numbers, two spaces, '/', and null term
#define MAXPWMSTRLEN 10

// compiler doesn't support static spec =\ pwmstr must be MAXPWMSTRLEN
static char* makepwmstr(char* pwmstr, unsigned reported, unsigned requested){
  snprintf(pwmstr, MAXPWMSTRLEN, "%u / %u", reported, requested);
  return pwmstr;
}

// m is millis()
static void updateDisplay(unsigned long m, float therm){
  char timestr[MAXTIMELEN];
  char pwmstr[MAXPWMSTRLEN];
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
  Heltec.display->drawString(33, 11, makepwmstr(pwmstr, ReportedPwm, Pwm));
  Heltec.display->drawString(0, 21, "Temp: ");
  if(therm == FLT_MAX){
    Heltec.display->drawString(35, 21, "n/a");
  }else{
    Heltec.display->drawString(35, 21, String(therm));
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
  static int lastReportedPWM = INT_MAX;
  static int lastReportedXA = INT_MAX;
  static int lastReportedXB = INT_MAX;
  static int lastPWM = -1;
  float therm = FLT_MAX;
  readThermistor(&therm);
  // FIXME we can remove this updateDisplay/millis() once the client.loop()
  // below is bounded
  unsigned long m = millis();
  updateDisplay(m, therm);
  // FIXME use the enqueue work version of this, as client.loop() can
  // block for arbitrary amounts of time
  client.loop(); // handle any necessary wifi/mqtt
  check_state_update();
  m = millis();

  float digiC = FLT_MAX;
  // FIXME need some error litmus
  digtemp.requestTemperatures();
  digiC = digtemp.getTempCByIndex(0);
  Serial.print("AMBIENT: ");
  Serial.println(digiC);

  updateDisplay(m, therm);
  bool broadcast = false;
  if(m - last_broadcast >= 3000){
    broadcast = true;
    last_broadcast = m;
    lastPWM = Pwm;
    send_pwm();
  }else if(lastPWM != Pwm){
    lastPWM = Pwm;
    last_broadcast = m;
    send_pwm();
  }
  // we send reported values to MQTT as soon as they change, and also as
  // part of any broadcast. this relies on infrequent value reports; we
  // implement no ratelimiting atop receipt.
  rpmPublish(client, "pwm", ReportedPwm, &lastReportedPWM, broadcast);
  rpmPublish(client, "moraxtop0rpm", XTopRPMA, &lastReportedXA, broadcast);
  rpmPublish(client, "moraxtop1rpm", XTopRPMB, &lastReportedXB, broadcast);
  rpmPublish(client, "rpm", RPM, &lastRPM, broadcast);
  if(broadcast){
    // send RGB regularly in case device resets (RGB isn't reported to us,
    // unlike PWM, so we can't merely send on unexpected read).
    send_rgb();
    if(therm != FLT_MAX && !isnan(therm)){
      mqttPublish(client, "therm", therm);
    }else{
      Serial.println("don't have a therm sample");
    }
    // there are several error codes returned by DallasTermperature, all of
    // them equal to or less than DEVICE_DISCONNECTED_C (there are also
    // DEVICE_FAULT_OPEN_C, DEVICE_FAULT_SHORTGND_C, and
    // DEVICE_FAULT_SHORTVDD_C).
    if(digiC != FLT_MAX && !isnan(digiC) && digiC > DEVICE_DISCONNECTED_C){
      mqttPublish(client, "moraambient", digiC);
    }else{
      Serial.println("don't have an ambient sample");
    }
  }
}

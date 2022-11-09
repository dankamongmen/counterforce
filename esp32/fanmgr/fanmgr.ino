// intended for use on a Heltec ESP32LoRav2, this manages a PWM fan and two
// PWM pumps. it receives PWM control messages, and sends RPM and temperature
// reports, over MQTT.
#include "heltec.h"
#include <float.h>
#include "EspMQTTClient.h"
#include <ArduinoJson.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <DallasTemperature.h>

#define VERSION "v2.0.0"

const unsigned long RPM_CUTOFF = 5000;
const int TEMPPIN = 39; // coolant thermistor (2-wire)
// ambient temperature (digital thermometer, Dallas 1-wire)
const int AMBIENTPIN = 17;
// pressure sensor
const int PRESSUREPIN = 13;

// PWM channels for RGB fans
const ledc_channel_t FANCHANR = LEDC_CHANNEL_0;
const ledc_channel_t FANCHANG = LEDC_CHANNEL_1;
const ledc_channel_t FANCHANB = LEDC_CHANNEL_2;
const ledc_channel_t FANCHANPWM = LEDC_CHANNEL_3;
const ledc_channel_t PUMPCHAN = LEDC_CHANNEL_4;
const int PUMPPWMPIN = 22;
const int FANPWMPIN = 25;
const int FANTACHPIN = 38;
const int XTOPATACHPIN = 36;
const int XTOPBTACHPIN = 37;
const int RGBPINR = 12;
const int RGBPING = 32;
const int RGBPINB = 33;

// RGB we want for the 12V fan LEDs (initialized to green, read from MQTT)
int Red = 0x8f;
int Green = 0x8f;
int Blue = 0x08;

// RPMs as determined by our interrupt handlers.
// we only get the RPM count from one of our fans; it stands for all.
static unsigned RPM;
static unsigned XTopRPMA;
static unsigned XTopRPMB;
static volatile unsigned long Pulses, XTAPulses, XTBPulses;

// PWMs we want to run at (initialized to INITIAL_*_PWM, read from MQTT)
#define INITIAL_FAN_PWM  192
#define INITIAL_PUMP_PWM 128
static unsigned Pwm;
static unsigned PumpPwm;
static unsigned XTopPWMA;
static unsigned XTopPWMB;

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

void readAmbient(float* t){
  digtemp.requestTemperatures();
  float tmp = digtemp.getTempCByIndex(0);
  if(*t <= DEVICE_DISCONNECTED_C){
    Serial.println("error reading 1-wire temp");
  }else{
    *t = tmp;
    Serial.print("ambientC: ");
    Serial.println(*t);
  }
}

void readPressure(float* t){
  float v0 = analogRead(PRESSUREPIN);
  Serial.print("read raw V for pressure: ");
  Serial.println(v0);
}

void readThermistor(float* t){
  const float BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 298.15;
  const float R0 = 10100;
  const float R1 = 10000;
  const float VREF = 3.3;
  float v0 = analogRead(TEMPPIN);
  Serial.print("read raw V for coolant: ");
  Serial.print(v0);
  if(v0 == 0 || v0 >= 4095){
    Serial.println(" discarding");
    return;
  }
  // 12-bit ADC on the ESP32. get voltage [0..3.3]...
  float scaled = v0 * VREF / 4095.0;
  Serial.print(" scaled: ");
  Serial.print(scaled);
  float Rt = R1 * scaled / (VREF - scaled);
  Serial.print(" Rt: ");
  Serial.println(Rt);
  float tn = 1.0 / ((1.0 / NOMINAL) + log(Rt / R0) / BETA);
  tn -= 273.15;
  Serial.print("coolantC: ");
  Serial.println(tn);
  *t = tn;
}

#define FANPWM_BIT_NUM LEDC_TIMER_8_BIT
#define FANPWM_TIMER LEDC_TIMER_1

int initialize_pwm(ledc_channel_t channel, int pin, int freq){
  ledc_channel_config_t conf;
  memset(&conf, 0, sizeof(conf));
  conf.gpio_num = pin;
  conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  conf.intr_type = LEDC_INTR_DISABLE;
  conf.timer_sel = FANPWM_TIMER;
  conf.duty = FANPWM_BIT_NUM;
  conf.channel = channel;
  Serial.print("setting up pin ");
  Serial.print(pin);
  Serial.print(" for ");
  Serial.print(freq);
  Serial.print("Hz PWM...");
  if(ledc_channel_config(&conf) != ESP_OK){
    Serial.println("error (channel config)!");
    return -1;
  }
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer.bit_num = FANPWM_BIT_NUM;
  ledc_timer.timer_num = FANPWM_TIMER;
  ledc_timer.freq_hz = 25000;
  if(ledc_timer_config(&ledc_timer) != ESP_OK){
    Serial.println("error (timer config)!");
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

void fantach(void){
  ++Pulses;
}

void xtop1tach(void){
  ++XTAPulses;
}

void xtop2tach(void){
  ++XTBPulses;
}

void setup(){
  int error = 0;
  Heltec.begin(true, false, true);
  Heltec.display->setFont(ArialMT_Plain_10);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  error |= initialize_fan_pwm(PUMPCHAN, PUMPPWMPIN);
  error |= initialize_fan_pwm(FANCHANPWM, FANPWMPIN);
  error |= initialize_rgb_pwm(FANCHANR, RGBPINR);
  error |= initialize_rgb_pwm(FANCHANG, RGBPING);
  error |= initialize_rgb_pwm(FANCHANB, RGBPINB);
  set_pwm(INITIAL_FAN_PWM);
  set_pump_pwm(INITIAL_PUMP_PWM);
  set_rgb();
  pinMode(TEMPPIN, INPUT);
  pinMode(PRESSUREPIN, INPUT);
  pinMode(FANTACHPIN, INPUT);
  pinMode(XTOPATACHPIN, INPUT);
  pinMode(XTOPBTACHPIN, INPUT);
  attachInterrupt(FANTACHPIN, fantach, RISING);
  attachInterrupt(XTOPATACHPIN, xtop1tach, RISING);
  attachInterrupt(XTOPBTACHPIN, xtop2tach, RISING);
  updateDisplay(0, FLT_MAX, FLT_MAX);
  // FIXME we ought rerun this until it works, to support hotplug
  digtemp.begin();
  Serial.print("1-Wire devices: ");
  Serial.println(digtemp.getDeviceCount());
  Serial.print("DS18xxx devices: ");
  Serial.println(digtemp.getDS18Count());
}

// set up the desired PWM value
static int set_pwm(unsigned p){
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FANCHANPWM, p) != ESP_OK){
    Serial.println("error setting PWM!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FANCHANPWM) != ESP_OK){
    Serial.println("error committing PWM!");
    return -1;
  }else{
    Serial.print("configured PWM: ");
    Serial.println(p);
    Pwm = p;
  }
  return 0;
}

// set up the desired pump PWM value
static int set_pump_pwm(unsigned p){
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, PUMPCHAN, p) != ESP_OK){
    Serial.println("error setting PWM!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, PUMPCHAN) != ESP_OK){
    Serial.println("error committing PWM!");
    return -1;
  }else{
    Serial.print("configured pump PWM: ");
    Serial.println(p);
    PumpPwm = p;
  }
  return 0;
}

// set up the desired RGB PWM values
static int set_rgb(void){
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FANCHANR, Red) != ESP_OK){
    Serial.println("error setting red!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FANCHANR) != ESP_OK){
    Serial.println("error committing red!");
    return -1;
  }
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FANCHANB, Blue) != ESP_OK){
    Serial.println("error setting blue!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FANCHANB) != ESP_OK){
    Serial.println("error committing blue!");
    return -1;
  }
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FANCHANG, Green) != ESP_OK){
    Serial.println("error setting green!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FANCHANG) != ESP_OK){
    Serial.println("error committing green!");
    return -1;
  }
  Serial.println("configured RGB");
  return 0;
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
      set_rgb();
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
      if(p < 256){
        set_pwm(p);
      }
    }
  );
  client.subscribe("control/mora3/pumppwm", [](const String &payload){
      Serial.print("received pump PWM via mqtt: ");
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
      if(p < 256){
        set_pump_pwm(p);
      }
    }
  );
}

static void displayConnectionStatus(int y){
  const char* connstr;
  Heltec.display->drawString(0, y, VERSION);
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
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
  static unsigned rollovercount;
  static unsigned long last_m;
  if(m < last_m){ // we rolled over
    ++rollovercount;
  }
  unsigned long t = m / 1000000; // FIXME factor in rollover (~1.1h)
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

template<typename T> int mqttPublish(EspMQTTClient& mqtt, const char* key, const T value){
  DynamicJsonDocument doc(BUFSIZ); // FIXME
  doc[key] = value;
  // PubSubClient limits messages to 256 bytes
  char buf[257];
  size_t n = serializeJson(doc, buf);
  mqtt.publish("sensors/mora3", buf, n);
  return 0;
}

int rpmPublish(EspMQTTClient& mqtt, const char* key, unsigned val){
  if(val < RPM_CUTOFF){ // filter out obviously incorrect values
    return mqttPublish(client, key, val);
  }
  return 0;
}

// at most, three five-digit numbers, four spaces, two '/'s, and null term
#define MAXRPMSTRLEN 22

// compiler doesn't support static spec =\ pwmstr must be MAXRPMSTRLEN
static char* makerpmstr(char* rpmstr, unsigned fan, unsigned pump1, unsigned pump2){
  if(fan > 65535){
    fan = 0;
  }
  if(pump1 > 65535){
    pump1 = 0;
  }
  if(pump2 > 65535){
    pump2 = 0;
  }
  snprintf(rpmstr, MAXRPMSTRLEN, "%u / %u / %u", fan, pump1, pump2);
  return rpmstr;
}

// at most, two three-digit numbers, two spaces, '/', and null term
#define MAXPWMSTRLEN 10

// compiler doesn't support static spec =\ pwmstr must be MAXPWMSTRLEN
static char* makepwmstr(char* pwmstr, unsigned fan, unsigned pump){
  if(fan > 65535){
    fan = 0;
  }
  if(pump > 65535){
    pump = 0;
  }
  snprintf(pwmstr, MAXPWMSTRLEN, "%u / %u", fan, pump);
  return pwmstr;
}

static inline bool valid_temp(float t){
  return !(t == FLT_MAX || isnan(t) || t <= DEVICE_DISCONNECTED_C);
}

static char* maketempstr(char* tempstr, size_t n, float coolant, float ambient){
  if(!valid_temp(coolant)){
    if(!valid_temp(ambient)){
      snprintf(tempstr, n, "n/a / n/a");
    }else{
      snprintf(tempstr, n, "%.2f / n/a", ambient);
    }
  }else if(!valid_temp(ambient)){
    snprintf(tempstr, n, "n/a / %.2f", coolant);
  }else{
    snprintf(tempstr, n, "%.2f / %.2f", ambient, coolant);
  }
  return tempstr;
}

// m is millis()
static void updateDisplay(unsigned long m, float therm, float ambient){
  char timestr[MAXTIMELEN];
  char pwmstr[MAXPWMSTRLEN];
  char rpmstr[MAXRPMSTRLEN];
  // dump information to OLED
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->drawString(0, 0, "RPM: ");
  Heltec.display->drawString(31, 0, makerpmstr(rpmstr, RPM, XTopRPMA, XTopRPMB));
  Heltec.display->drawString(0, 11, "PWM: ");
  Heltec.display->drawString(33, 11, makepwmstr(pwmstr, Pwm, PumpPwm));
  Heltec.display->drawString(0, 21, "Temp: ");
  Heltec.display->drawString(35, 21, maketempstr(rpmstr, sizeof(rpmstr), therm, ambient));
  Heltec.display->drawString(0, 31, "Uptime: ");
  Heltec.display->drawString(41, 31, maketimestr(timestr, m) ?
                             "a long time" : timestr);
  displayConnectionStatus(51);
  Heltec.display->display();
}

static inline float rpm(unsigned long pulses, unsigned long usec){
  return pulses * 60 / 2 * (1000000.0 / usec);
}

// we transmit and update the display approximately every 3s, sampling
// RPM at this time. we continuously sample the temperature, and use the
// most recent valid read for transmit/display. there are several blocking
// calls (1-wire and MQTT) that can lengthen a given cycle.
void loop(){
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  static float pressure = FLT_MAX;
  static float coolant_temp = FLT_MAX;
  static float ambient_temp = FLT_MAX;
  readThermistor(&coolant_temp);
  readPressure(&pressure);
  readAmbient(&ambient_temp);
  unsigned long m = micros();
  client.loop(); // handle any necessary wifi/mqtt

  unsigned long diff = m - last_tx;
  if(diff < 3000000){
    return;
  }
  // sample RPM, transmit, and update the display
  // FIXME replace with MEGA's rolling 5s logic
  last_tx = m;
  noInterrupts();
  unsigned long p = Pulses;
  Pulses = 0;
  unsigned long x1p = XTAPulses;
  XTAPulses = 0;
  unsigned long x2p = XTBPulses;
  XTBPulses = 0;
  interrupts();
  Serial.print(diff);
  Serial.println(" µsec expired for cycle");
  Serial.print(p);
  Serial.println(" pulses measured at fan");
  RPM = rpm(p, diff);
  XTopRPMA = rpm(x1p, diff);
  XTopRPMB = rpm(x2p, diff);
  Serial.print(RPM);
  Serial.println(" RPM measured at fan");
  updateDisplay(m, coolant_temp, ambient_temp);
  rpmPublish(client, "moraxtop0rpm", XTopRPMA);
  rpmPublish(client, "moraxtop1rpm", XTopRPMB);
  rpmPublish(client, "rpm", RPM);
  if(valid_temp(coolant_temp)){
    mqttPublish(client, "therm", coolant_temp);
  }else{
    Serial.println("don't have a coolant sample");
  }
  // there are several error codes returned by DallasTermperature, all of
  // them equal to or less than DEVICE_DISCONNECTED_C (there are also
  // DEVICE_FAULT_OPEN_C, DEVICE_FAULT_SHORTGND_C, and
  // DEVICE_FAULT_SHORTVDD_C).
  if(valid_temp(ambient_temp)){
    mqttPublish(client, "moraambient", ambient_temp);
  }else{
    Serial.println("don't have an ambient sample");
  }
}

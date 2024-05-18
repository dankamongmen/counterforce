// common routines for the ESP32 and ESP8266 implementations of fanmgr
#include "EspMQTTClient.h"

#include <ArduinoJson.h>

#define VERSION "v2.4.0"

#ifdef ESP32
#define ISR IRAM_ATTR
#else
#define ISR IRAM_ATTR //ICACHE_RAM_ATRR
#endif

static const ledc_channel_t FANCHAN = LEDC_CHANNEL_0;
static const ledc_channel_t PUMPACHAN = LEDC_CHANNEL_1;
static const ledc_channel_t PUMPBCHAN = LEDC_CHANNEL_2;

#define FANPWM_BIT_NUM LEDC_TIMER_8_BIT

#define RPMMAX (1u << 13u)

static volatile unsigned FanRpm;
static volatile unsigned PumpARpm;
static volatile unsigned PumpBRpm;

void ISR rpm_fan(void){
  if(FanRpm < RPMMAX){
    ++FanRpm;
  }
}

void ISR rpm_pumpa(void){
  if(PumpARpm < RPMMAX){
    ++PumpARpm;
  }
}

void ISR rpm_pumpb(void){
  if(PumpBRpm < RPMMAX){
    ++PumpBRpm;
  }
}

// PWMs we want to run at (initialized here, read from MQTT)
static unsigned FanPwm = 128;
static unsigned PumpPwm = 128;

EspMQTTClient client(
  #include "EspMQTTConfig.h",
  DEVNAME
);

// precondition: isxdigit(c) is true
static byte getHex(char c){
  if(isdigit(c)){
    return c - '0';
  }
  c = tolower(c);
  return c - 'a' + 10;
}

static int extract_pwm(const String& payload){
  if(payload.length() != 2){
    Serial.println("pwm wasn't 2 characters");
    return -1;
  }
  char h = payload.charAt(0);
  char l = payload.charAt(1);
  if(!isxdigit(h) || !isxdigit(l)){
    Serial.println("invalid hex character");
    return -1;
  }
  byte hb = getHex(h);
  byte lb = getHex(l);
  // everything was valid
  return hb * 16 + lb;
}

static int readAmbient(float* t, DallasTemperature *dt){
  dt->requestTemperatures();
  float tmp = dt->getTempCByIndex(0);
  if(tmp <= DEVICE_DISCONNECTED_C){
    Serial.println("error reading 1-wire temp");
    return -1;
  }
  *t = tmp;
  Serial.print("ambientC: ");
  Serial.println(*t);
  return 0;
}

static void readThermistor(float* t, int pin, int levels){
  *t = NAN;
  const float BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 298.15;
  const float R0 = 10100;
  const float R1 = 10000;
  const float VREF = 3.3;
  float v0 = analogRead(pin);
  if(v0 <= 1 || v0 >= levels - 1){
    return;
  }
  // 10-bit ADC on the ESP8266. get voltage [0..3.3]...
  float scaled = v0 * VREF / (levels - 1);
  float Rt = R1 * scaled / (VREF - scaled);
  float tn = 1.0 / ((1.0 / NOMINAL) + log(R0 / Rt) / BETA);
  tn -= 273.15;
  *t = tn;
}

// attempt to establish a connection to the DS18B20
static int connect_onewire(DallasTemperature* dt){
  static unsigned long last_error_diag;
  dt->begin();
  int devcount = dt->getDeviceCount();
  if(devcount){
    Serial.print("1-Wire devices: ");
    Serial.println(devcount);
    return 0;
  }
  unsigned long m = millis();
  if(last_error_diag + 1000 <= m){
    Serial.println("1Wire connerr");
    last_error_diag = m;
  }
  return -1;
}

typedef struct mqttmsg {
 private: 
  EspMQTTClient& mqtt;
  DynamicJsonDocument doc{BUFSIZ};
 public:
  mqttmsg(EspMQTTClient& esp) :
    mqtt(esp)
    {}
  template<typename T> void add(const char* key, const T value){
    doc[key] = value;
  }
  bool publish(){
    add("uptimesec", millis() / 1000); // FIXME handle overflow
    char buf[257]; // PubSubClient limits messages to 256 bytes
    size_t n = serializeJson(doc, buf);
    return mqtt.publish("sensors/" DEVNAME, buf);
  }
} mqttmsg;

void rpmPublish(mqttmsg& mmsg, const char* key, unsigned val){
  if(val < RPMMAX){ // filter out obviously incorrect values
    mmsg.add(key, val);
  }else{
    Serial.print("not publishing ");
    Serial.print(val);
    Serial.print(" for ");
    Serial.println(key);
  }
}

// we shouldn't ever see 60C (140F) at the MO-RA3; filter them.
static inline bool valid_temp(float t){
  return !(isnan(t) || t > 60 || t <= DEVICE_DISCONNECTED_C);
}

static inline float rpm(unsigned long pulses, unsigned long usec){
  Serial.print(pulses);
  Serial.println(" pulses measured");
  return pulses * 30000000.0 / usec;
}

static void publish_rgb(mqttmsg& mmsg, unsigned r, unsigned g, unsigned b){
  mmsg.add("rgbr", r);
  mmsg.add("rgbg", g);
  mmsg.add("rgbb", b);
}

static void publish_temps(mqttmsg& mmsg, float amb, float cool){
  if(valid_temp(cool) && cool > 0){
    mmsg.add("atemp", cool);
  }else{
    Serial.println("don't have an analog temp sample");
  }
  // there are several error codes returned by DallasTemperature, all of
  // them equal to or less than DEVICE_DISCONNECTED_C (there are also
  // DEVICE_FAULT_OPEN_C, DEVICE_FAULT_SHORTGND_C, and
  // DEVICE_FAULT_SHORTVDD_C).
  if(valid_temp(amb)){
    mmsg.add("dtemp", amb);
  }else{
    Serial.println("don't have a digital temp sample");
  }
}

static bool valid_pwm_p(int pwm){
  return pwm >= 0 && pwm <= 255;
}

static void publish_pwm(mqttmsg& mmsg, int fanpwm, int pumppwm){
  if(valid_pwm_p(fanpwm)){
    mmsg.add("fanpwm", fanpwm);
  }
  if(valid_pwm_p(pumppwm)){
    mmsg.add("pumppwm", pumppwm);
  }
}

void publish_pair(mqttmsg& mmsg, const char* key, int val){
  mmsg.add(key, val);
}

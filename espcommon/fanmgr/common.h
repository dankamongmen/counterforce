// common routines for the ESP32 and ESP8266 implementations of fanmgr
#include "EspMQTTClient.h"

#include <ArduinoJson.h>

#define VERSION "v2.3.0"

#ifdef ESP32
#define ISR IRAM_ATTR
#else
#define ISR IRAM_ATTR //ICACHE_RAM_ATRR
#endif

const unsigned long RPM_CUTOFF = 5000;

// PWMs we want to run at (initialized to INITIAL_*_PWM, read from MQTT)
#define INITIAL_FAN_PWM  64
#define INITIAL_PUMP_PWM 64

static volatile unsigned long Pulses;
static volatile unsigned long XTAPulses;
static volatile unsigned long XTBPulses;

static void ISR fantach(void){
  ++Pulses;
}

static void ISR xtop1tach(void){
  ++XTAPulses;
}

static void ISR xtop2tach(void){
  ++XTBPulses;
}

static void print_int_pin(int pin){
  Serial.print("setting up interrupt on ");
  Serial.println(pin);
}

static void debug_interrupt(int pin){
  Serial.print("interrupt ");
  Serial.print(digitalPinToInterrupt(pin));
  Serial.print(" on pin ");
  Serial.println(pin);
}

static void setup_interrupts(int fanpin, int pumppina, int pumppinb){
  print_int_pin(fanpin);
  pinMode(fanpin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(fanpin), fantach, FALLING);
  debug_interrupt(fanpin);
  print_int_pin(pumppina);
  pinMode(pumppina, INPUT_PULLUP);
  debug_interrupt(pumppina);
  attachInterrupt(digitalPinToInterrupt(pumppina), xtop1tach, FALLING);
  print_int_pin(pumppinb);
  pinMode(pumppinb, INPUT_PULLUP);
  debug_interrupt(pumppinb);
  attachInterrupt(digitalPinToInterrupt(pumppinb), xtop2tach, FALLING);
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
    char buf[257]; // PubSubClient limits messages to 256 bytes
    size_t n = serializeJson(doc, buf);
    return mqtt.publish("sensors/" DEVNAME, buf, n);
  }
} mqttmsg;

void rpmPublish(mqttmsg& mmsg, const char* key, unsigned val){
  if(val < RPM_CUTOFF){ // filter out obviously incorrect values
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
  return pulses * 60 * 1000000.0 / usec / 2;
}

// precondition: isxdigit(c) is true
static byte getHex(char c){
  if(isdigit(c)){
    return c - '0';
  }
  c = tolower(c);
  return c - 'a' + 10;
}

static void publish_uptime(mqttmsg& mmsg, unsigned long s){
  mmsg.add("uptimesec", s);
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
    mmsg.add("morapwm", fanpwm);
  }
  if(valid_pwm_p(pumppwm)){
    mmsg.add("morapumppwm", pumppwm);
  }
}

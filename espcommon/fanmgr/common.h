// common routines for the ESP32 and ESP8266 implementations of fanmgr
#include "EspMQTTClient.h"

#define VERSION "v2.1.0"

#ifdef ESP32
#define ISR IRAM_ATTR
#else
#define ISR IRAM_ATTR //ICACHE_RAM_ATRR
#endif

const unsigned long RPM_CUTOFF = 5000;

// PWMs we want to run at (initialized to INITIAL_*_PWM, read from MQTT)
#define INITIAL_FAN_PWM  192
#define INITIAL_PUMP_PWM 128

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

static void setup_interrupts(int fanpin, int pumppina, int pumppinb){
  print_int_pin(fanpin);
  pinMode(fanpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(fanpin), fantach, RISING);
  print_int_pin(pumppina);
  pinMode(pumppina, INPUT);
  attachInterrupt(digitalPinToInterrupt(pumppina), xtop1tach, RISING);
  print_int_pin(pumppinb);
  pinMode(pumppinb, INPUT);
  attachInterrupt(digitalPinToInterrupt(pumppinb), xtop2tach, RISING);
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
  const float BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 298.15;
  const float R0 = 10100;
  const float R1 = 10000;
  const float VREF = 3.3;
  float v0 = analogRead(pin);
  Serial.print("coolantV: ");
  Serial.print(v0);
  if(v0 <= 1 || v0 >= levels - 1){
    Serial.println(", discarding");
    return;
  }
  // 10-bit ADC on the ESP8266. get voltage [0..3.3]...
  float scaled = v0 * VREF / (levels - 1);
  Serial.print(" scaled: ");
  Serial.print(scaled);
  float Rt = R1 * scaled / (VREF - scaled);
  Serial.print(" Rt: ");
  Serial.println(Rt);
  float tn = 1.0 / ((1.0 / NOMINAL) + log(R0 / Rt) / BETA);
  tn -= 273.15;
  Serial.print("coolantC: ");
  Serial.println(tn);
  *t = tn;
}

// attempt to establish a connection to the DS18B20
static int connect_onewire(DallasTemperature* dt){
  dt->begin();
  int devcount = dt->getDeviceCount();
  if(devcount){
    Serial.print("1-Wire devices: ");
    Serial.println(devcount);
    return 0;
  }
  Serial.println("1Wire connerr");
  return -1;
}

template<typename T> int mqttPublish(EspMQTTClient& mqtt, const char* key, const T value){
  DynamicJsonDocument doc(BUFSIZ); // FIXME
  doc[key] = value;
  // PubSubClient limits messages to 256 bytes
  char buf[257];
  size_t n = serializeJson(doc, buf);
  mqtt.publish("sensors/" DEVNAME, buf, n);
  return 0;
}

int rpmPublish(EspMQTTClient& mqtt, const char* key, unsigned val){
  if(val < RPM_CUTOFF){ // filter out obviously incorrect values
    return mqttPublish(mqtt, key, val);
  }else{
    Serial.print("not publishing ");
    Serial.print(val);
    Serial.print(" for ");
    Serial.println(key);
  }
  return 0;
}

// we shouldn't ever see 60C (140F) at the MO-RA3; filter them. if this
// is someday eliminated, ensure we filter explicit FLT_MAX!
static inline bool valid_temp(float t){
  return !(t > 60 || isnan(t) || t <= DEVICE_DISCONNECTED_C);
}

static inline float rpm(unsigned long pulses, unsigned long usec){
  return pulses * 60 * 1000000.0 / usec / 2;
}

static void readPressure(float* t, int pin){
  float v0 = analogRead(pin);
  Serial.print("read raw V for pressure: ");
  Serial.println(v0);
}

// precondition: isxdigit(c) is true
static byte getHex(char c){
  if(isdigit(c)){
    return c - '0';
  }
  c = tolower(c);
  return c - 'a' + 10;
}

static void publish_temps(EspMQTTClient& client, float amb, float cool){
  if(valid_temp(cool)){
    mqttPublish(client, "moracoolant", cool);
  }else{
    Serial.println("don't have a coolant sample");
  }
  // there are several error codes returned by DallasTemperature, all of
  // them equal to or less than DEVICE_DISCONNECTED_C (there are also
  // DEVICE_FAULT_OPEN_C, DEVICE_FAULT_SHORTGND_C, and
  // DEVICE_FAULT_SHORTVDD_C).
  if(valid_temp(amb)){
    mqttPublish(client, "moraambient", amb);
  }else{
    Serial.println("don't have an ambient sample");
  }
}

static bool valid_pwm_p(int pwm){
  return pwm >= 0 && pwm <= 255;
}

static void publish_pwm(EspMQTTClient& client, int fanpwm, int pumppwm){
  if(valid_pwm_p(fanpwm)){
    mqttPublish(client, "morapwm", fanpwm);
  }
  if(valid_pwm_p(pumppwm)){
    mqttPublish(client, "morapumppwm", pumppwm);
  }
}

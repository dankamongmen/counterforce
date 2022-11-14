// common routines for the ESP32 and ESP8266 implementations of fanmgr
#include "EspMQTTClient.h"

#define VERSION "v2.1.0"

const unsigned long RPM_CUTOFF = 5000;

// PWMs we want to run at (initialized to INITIAL_*_PWM, read from MQTT)
#define INITIAL_FAN_PWM  192
#define INITIAL_PUMP_PWM 128

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

static void readThermistor(float* t, int pin){
  const float BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 298.15;
  const float R0 = 10100;
  const float R1 = 10000;
  const float VREF = 3.3;
  float v0 = analogRead(pin);
  Serial.print("read raw V for coolant: ");
  Serial.print(v0);
  if(v0 <= 1 || v0 >= 1023){
    Serial.println(" discarding");
    return;
  }
  // 10-bit ADC on the ESP8266. get voltage [0..3.3]...
  float scaled = v0 * VREF / 1023.0;
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

// attempt to establish a connection to the DS18B20
static int connect_onewire(DallasTemperature* dt){
  dt->begin();
  int devcount = dt->getDeviceCount();
  if(devcount){
    Serial.print("1-Wire devices: ");
    Serial.println(devcount);
    return 0;
  }
  Serial.println("error connecting to 1Wire");
  return -1;
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

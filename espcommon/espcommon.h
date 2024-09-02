#include <WiFi.h>
#include <OneWire.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <DallasTemperature.h>
#include <ESP32MQTTClient.h>
#include "EspMQTTConfig.h" // local secrets

typedef struct mqttmsg {
 private: 
  ESP32MQTTClient& mqtt;
  DynamicJsonDocument doc{BUFSIZ};
 public:
  mqttmsg(ESP32MQTTClient& esp) :
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

static void
publish_version(mqttmsg& mmsg){
  mmsg.add("ver", VERSION);
}

// we shouldn't ever see 60C (140F) at the MO-RA3; filter them.
static inline bool
valid_temp(float t){
  return !(isnan(t) || t > 60 || t <= DEVICE_DISCONNECTED_C);
}

static void publish_temps(mqttmsg& mmsg, float amb){
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

static void publish_pair(mqttmsg& mmsg, const char* key, int val){
  mmsg.add(key, val);
}

ESP32MQTTClient client;

static OneWire twire(AMBIENTPIN);
static DallasTemperature digtemp(&twire);

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

static float
getAmbient(void){
  static bool onewire_connected;
  float ambient_temp = NAN;
  if(!onewire_connected){
    printf("connecting on 1wire\n");
    if(connect_onewire(&digtemp) == 0){
      onewire_connected = true;
      uint8_t addr;
      if(digtemp.getAddress(&addr, 0)){
        printf("digtemp 0 address: %hu\n", addr);
      }
    }
  }
  if(onewire_connected){
    if(readAmbient(&ambient_temp, &digtemp)){
      onewire_connected = false;
      ambient_temp = NAN;
    }
  }
  return ambient_temp;
}

void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  client.onEventCallback(event);
}

static int
mqtt_setup(ESP32MQTTClient& mqtt){
  mqtt.enableDebuggingMessages();
  mqtt.setURI(MQTTHOST, MQTTUSER, MQTTPASS);
  mqtt.enableLastWillMessage("", "");
  //mqtt.setKeepAlive(30);
  printf("set uri: " MQTTHOST "\n");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFIESSID, WIFIPASS);
  printf("WiFi status: %d\n", WiFi.status());
  mqtt.loopStart();
  return 0;
}

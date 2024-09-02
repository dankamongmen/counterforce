#include <WiFi.h>
#include <OneWire.h>
#include <esp_wifi.h>
#include <DallasTemperature.h>
#include <ESP32MQTTClient.h>
#include "EspMQTTConfig.h" // local secrets

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
      /*
      uint8_t addr;
      if(digtemp.getAddress(&addr, 0)){
        Serial.print("digtemp 0 address: ");
        Serial.println(addr);
      }
      */
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
  mqtt.setKeepAlive(30);
  printf("set uri: " MQTTHOST "\n");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFIESSID, WIFIPASS);
  mqtt.loopStart();
  printf("WiFi status: %d\n", WiFi.status());
}

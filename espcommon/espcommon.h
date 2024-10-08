#include <WiFi.h>
#include <OneWire.h>
#include <esp_wifi.h>
#include <driver/ledc.h>
#include <ArduinoJson.h>
#include <DallasTemperature.h>
#include <ESP32MQTTClient.h>
#include "EspMQTTConfig.h" // local secrets
#include "nvs_flash.h"
#include "nvs.h"

#ifdef ESP32
#define ISR IRAM_ATTR
#else
#error "only designed for use with esp32"
#endif

#define FANPWM_BIT_NUM LEDC_TIMER_8_BIT

#define RPMMAX (1u << 14u)

static nvs_handle_t Nvs;
static ESP32MQTTClient client;
static OneWire twire(AMBIENTPIN);
static DallasTemperature digtemp(&twire);

typedef struct mqttmsg {
 private: 
  ESP32MQTTClient& mqtt;
  JsonDocument doc;
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
    size_t r = serializeJson(doc, buf, sizeof(buf));
    if(r >= sizeof(buf)){
      printf("serialization was too large (%ub)\n", r);
      return false;
    }
    printf("xmit [%s]\n", buf);
    return mqtt.publish("sensors/" DEVNAME, buf);
  }
} mqttmsg;

static inline void
publish_version(mqttmsg& mmsg){
  mmsg.add("ver", VERSION);
}

// we shouldn't ever see 100C (212F) at the MO-RA3; filter them.
static inline bool
valid_temp(float t){
  return !(isnan(t) || t > 85 || t <= DEVICE_DISCONNECTED_C);
}

static inline void
publish_airqual(mqttmsg& mmsg, unsigned voc, unsigned co2){
  if(voc){
    mmsg.add("voc", voc);
  }
  if(co2){
    mmsg.add("co2", co2);
  }
}

static inline void
publish_temps(mqttmsg& mmsg, float amb){
  // there are several error codes returned by DallasTemperature, all of
  // them equal to or less than DEVICE_DISCONNECTED_C (there are also
  // DEVICE_FAULT_OPEN_C, DEVICE_FAULT_SHORTGND_C, and
  // DEVICE_FAULT_SHORTVDD_C).
  if(valid_temp(amb)){
    mmsg.add("dtemp", amb);
  }else{
    printf("no digital temp sample available\n");
  }
}

static inline void
publish_pair(mqttmsg& mmsg, const char* key, int val){
  mmsg.add(key, val);
}

static int
readAmbient(float* t, DallasTemperature *dt){
  dt->requestTemperatures();
  float tmp = dt->getTempCByIndex(0);
  if(tmp <= DEVICE_DISCONNECTED_C){
    printf("error reading 1-wire temp\n");
    return -1;
  }
  *t = tmp;
  printf("ambientC: %f\n", *t);
  return 0;
}

// attempt to establish a connection to the DS18B20
static int
connect_onewire(DallasTemperature* dt){
  static unsigned long last_error_diag;
  dt->begin();
  int devcount = dt->getDeviceCount();
  if(devcount){
    printf("1-Wire devices: %d", devcount);
    return 0;
  }
  unsigned long m = millis();
  if(last_error_diag + 1000 <= m){
    printf("1Wire connerr\n");
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
    }else{
      printf("connect failed\n");
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

static int extract_number(const String& payload){
  unsigned temp = 0;
  for(int pos = 0 ; pos < payload.length() ; ++pos){
    char h = payload.charAt(pos);
    if(!isdigit(h)){
      printf("unexpected non-digit character\n");
      return -1;
    }
    temp *= 10;
    temp += h - '0';
  }
  // everything was valid
  return temp;
}

static bool valid_pwm_p(int pwm){
  return pwm >= 0 && pwm <= 255;
}

// precondition: isxdigit(c) is true
static byte getHex(char c){
  if(isdigit(c)){
    return c - '0';
  }
  c = tolower(c);
  return c - 'a' + 10;
}

// FIXME handle base 10 numbers as well (can we use strtoul?)
static int extract_pwm(const String& payload){
  if(payload.length() != 2){
    printf("pwm wasn't 2 characters\n");
    return -1;
  }
  char h = payload.charAt(0);
  char l = payload.charAt(1);
  if(!isxdigit(h) || !isxdigit(l)){
    printf("invalid hex character\n");
    return -1;
  }
  byte hb = getHex(h);
  byte lb = getHex(l);
  // everything was valid
  return hb * 16 + lb;
}

// set the desired PWM value
static int
set_pwm(const ledc_channel_t channel, unsigned pwm){
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, pwm) != ESP_OK){
    printf("error setting pwm!\n");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel) != ESP_OK){
    printf("error committing pwm!\n");
    return -1;
  }
  printf("set pwm to %u on channel %d\n", pwm, channel);
  return 0;
}

static void
init_tach(int pin, void(*fxn)(void)){
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), fxn, FALLING);
}

static int
initialize_pwm(ledc_channel_t channel, int pin, int freq, ledc_timer_t timer){
  pinMode(pin, OUTPUT);
  ledc_channel_config_t conf;
  memset(&conf, 0, sizeof(conf));
  conf.gpio_num = pin;
  conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  conf.intr_type = LEDC_INTR_DISABLE;
  conf.timer_sel = timer;
  conf.duty = FANPWM_BIT_NUM;
  conf.channel = channel;
  printf("setting up pin %d for %dHz PWM\n", pin, freq);
  if(ledc_channel_config(&conf) != ESP_OK){
    printf("error (channel config)!\n");
    return -1;
  }
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer));
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer.duty_resolution = FANPWM_BIT_NUM;
  ledc_timer.timer_num = timer;
  ledc_timer.freq_hz = freq;
  if(ledc_timer_config(&ledc_timer) != ESP_OK){
    printf("error (timer config)!\n");
    return -1;
  }
  printf("success!\n");
  return 0;
}

static inline int
initialize_25k_pwm(ledc_channel_t channel, int pin, ledc_timer_t timer){
  return initialize_pwm(channel, pin, 25000, timer);
}

void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  client.onEventCallback(event);
}

static inline unsigned
rpm(unsigned long pulses, unsigned long usec){
  printf("%lu pulses measured\n", pulses);
  if(pulses >= RPMMAX){
    return RPMMAX;
  }
  return pulses * 30000000.0 / usec;
}

static void publish_pwm(mqttmsg& mmsg, const char* str, int pwm){
  if(valid_pwm_p(pwm)){
    mmsg.add(str, pwm);
  }else{
    printf("invalid pwm %d for %s\n", pwm, str);
  }
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

static int
nvs_setup(nvs_handle_t *nh){
  esp_err_t err = nvs_flash_init();
  if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND){
    // NVS partition was truncated and needs to be erased
    nvs_flash_erase();
    err = nvs_flash_init(); // Retry nvs_flash_init
  }
  if(err != ESP_OK){
    printf("error initializing flash: %s\n", esp_err_to_name(err));
    return -1;
  }
  err = nvs_open("storage", NVS_READWRITE, nh);
  if(err != ESP_OK){
    printf("error opening flash: %s\n", esp_err_to_name(err));
    return -1;
  }
  return 0;
}

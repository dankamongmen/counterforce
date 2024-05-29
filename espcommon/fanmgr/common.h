// common routines for the ESP32 and ESP8266 implementations of fanmgr
#include "EspMQTTClient.h"
#include <float.h>
#include <Wire.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DallasTemperature.h>

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"

#define VERSION "2.6.0"

#ifdef ESP32
#define ISR IRAM_ATTR
#else
#define ISR IRAM_ATTR //ICACHE_RAM_ATRR
#endif

static bool usingDisplay;
static const ledc_channel_t FANCHAN = LEDC_CHANNEL_0;
static const ledc_channel_t PUMPACHAN = LEDC_CHANNEL_1;
static const ledc_channel_t PUMPBCHAN = LEDC_CHANNEL_2;

#define FANPWM_BIT_NUM LEDC_TIMER_8_BIT

#define RPMMAX (1u << 13u)

static nvs_handle_t Nvs;

static volatile unsigned FanRpm;
static volatile unsigned PumpARpm;
static volatile unsigned PumpBRpm;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using I2C
#define OLED_RESET     -1 // Reset pin
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 disp(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

static OneWire twire(AMBIENTPIN);
static DallasTemperature digtemp(&twire);

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

static void
maketimestr(char *str){
  uint64_t ticks = esp_timer_get_time();
  ticks /= 1000000;
  unsigned s = ticks % 60;
  unsigned m = (ticks % 3600) / 60;
  unsigned h = (ticks % 86400lu) / 3600;
  unsigned d = ticks / 86400lu;
  if(d){
    sprintf(str, "%ud %uh %um %us", d, h, m, s);
  }else if(h){
    sprintf(str, "%uh %um %us", h, m, s);
  }else if(m){
    sprintf(str, "%um %us", m, s);
  }else{
    sprintf(str, "%us", s);
  }
}

#define TEXTHEIGHT 10

static void
displayDraw(float ambient){
  if(usingDisplay){
    disp.clearDisplay();
    disp.setTextSize(1);
    disp.setTextColor(WHITE);
    disp.setCursor(0, 0);
    disp.println("inaMORAta v" VERSION);
    if(isnan(ambient)){
      disp.printf("ambient: --");
    }else{
      disp.printf("ambient: %0.2f C", ambient);
    }
    disp.setCursor(0, 2 * TEXTHEIGHT + 1);
    disp.printf("fans: %u %lu\n", FanPwm, FanRpm);
    disp.setCursor(0, 3 * TEXTHEIGHT + 2);
    disp.printf("pump a: %u %lu\n", PumpPwm, PumpARpm);
    disp.printf("pump b: %u %lu\n", PumpPwm, PumpBRpm);
    char tempstr[16];
    maketimestr(tempstr);
    disp.setCursor(0, SCREEN_HEIGHT - TEXTHEIGHT);
    disp.printf("uptime: %s", tempstr);
    disp.display();
  }
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
    printf("not publishing %u for %s\n", val, key);
  }
}

// we shouldn't ever see 60C (140F) at the MO-RA3; filter them.
static inline bool
valid_temp(float t){
  return !(isnan(t) || t > 60 || t <= DEVICE_DISCONNECTED_C);
}

static inline unsigned
rpm(unsigned long pulses, unsigned long usec){
  printf("%lu pulses measured\n", pulses);
  if(pulses >= RPMMAX){
    return RPMMAX;
  }
  return pulses * 30000000.0 / usec;
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

static void
publish_version(mqttmsg& mmsg){
  mmsg.add("ver", VERSION);
}

static void publish_pair(mqttmsg& mmsg, const char* key, int val){
  mmsg.add(key, val);
}

static void init_tach(int pin, void(*fxn)(void)){
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), fxn, FALLING);
}

static int initialize_pwm(ledc_channel_t channel, int pin, int freq, ledc_timer_t timer){
  pinMode(pin, OUTPUT);
  ledc_channel_config_t conf;
  memset(&conf, 0, sizeof(conf));
  conf.gpio_num = pin;
  conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  conf.intr_type = LEDC_INTR_DISABLE;
  conf.timer_sel = timer;
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
  ledc_timer.timer_num = timer;
  ledc_timer.freq_hz = freq;
  if(ledc_timer_config(&ledc_timer) != ESP_OK){
    Serial.println("error (timer config)!");
    return -1;
  }
  Serial.println("success!");
  return 0;
}

// set up the desired PWM values
static int set_pwm(const ledc_channel_t channel, unsigned pwm){
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, pwm) != ESP_OK){
    Serial.println("error setting red!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel) != ESP_OK){
    Serial.println("error committing red!");
    return -1;
  }
  printf("set pwm to %u on channel %lu\n", pwm, channel);
  return 0;
}

static int initialize_25k_pwm(ledc_channel_t channel, int pin, ledc_timer_t timer){
  return initialize_pwm(channel, pin, 25000, timer);
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
  uint32_t pwm;
  if(nvs_get_u32(*nh, "fanpwm", &pwm) == ESP_OK && valid_pwm_p(pwm)){
    FanPwm = pwm;
    set_pwm(FANCHAN, FanPwm);
  }else{
    printf("no valid fanpwm in persistent store\n");
  }
  if(nvs_get_u32(*nh, "pumppwm", &pwm) == ESP_OK && valid_pwm_p(pwm)){
    PumpPwm = pwm;
    set_pwm(PUMPACHAN, PumpPwm);
    set_pwm(PUMPBCHAN, PumpPwm);
  }else{
    printf("no valid pumppwm in persistent store\n");
  }
  return 0;
}

static int
displaySetup(int sclpin, int sdapin){
  Wire.setPins(sclpin, sdapin);
  if(!disp.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
    Serial.println(F("SSD1306 allocation failed"));
    return -1;
  }
  usingDisplay = true;
  displayDraw(NAN);
  return 0;
}

static void
fanmgrSetup(int ledpin){
  Serial.begin(115200);
  Serial.println("initializing!");
  //setCpuFrequencyMhz(80);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  client.enableHTTPWebUpdater();
  initialize_25k_pwm(FANCHAN, FANPWMPIN, LEDC_TIMER_1);
  initialize_25k_pwm(PUMPACHAN, PUMPAPWMPIN, LEDC_TIMER_2);
  initialize_25k_pwm(PUMPBCHAN, PUMPBPWMPIN, LEDC_TIMER_3);
  set_pwm(FANCHAN, FanPwm);
  set_pwm(PUMPACHAN, PumpPwm);
  set_pwm(PUMPBCHAN, PumpPwm);
  init_tach(FANTACHPIN, rpm_fan);
  init_tach(PUMPATACHPIN, rpm_pumpa);
  init_tach(PUMPBTACHPIN, rpm_pumpb);
  pinMode(ledpin, OUTPUT);
  nvs_setup(&Nvs);
  printf("Fan PWM initialized to %u\n", FanPwm);
  printf("Pump PWM initialized to %u\n", PumpPwm);
  Serial.println("initialized!");
}

static int
commit(nvs_handle_t nh){
  esp_err_t err = nvs_commit(nh);
  if(err != ESP_OK){
    printf("error writing flash: %s\n", esp_err_to_name(err));
    return -1;
  }
  return 0;
}

void onConnectionEstablished() {
  wifi_country_t country = {
    .cc = "US",
    .schan = 1,
    .nchan = 14,
  };
  esp_err_t err = esp_wifi_set_country(&country);
  if(err == ESP_OK){
    printf("loaded US wifi regulatory policy\n");
  }else if(err == ESP_ERR_INVALID_ARG){
    printf("error setting wifi country--bad argument\n");
  }else{
    printf("error setting wifi country--not initialized\n");
  }
  Serial.println("got an MQTT connection");
  client.subscribe("control/" DEVNAME "/fanpwm", [](const String &payload){
      Serial.print("received fan pwm via mqtt: ");
      Serial.println(payload);
      int fpwm = extract_pwm(payload);
      if(valid_pwm_p(fpwm)){
        FanPwm = fpwm;
        set_pwm(FANCHAN, FanPwm);
        if(nvs_set_u32(Nvs, "fanpwm", FanPwm) == ESP_OK){
          commit(Nvs);
        }
      }
    }
  );
  client.subscribe("control/" DEVNAME "/pumppwm", [](const String &payload){
      Serial.print("received pump pwm via mqtt: ");
      Serial.println(payload);
      unsigned ppwm = extract_pwm(payload);
      if(valid_pwm_p(ppwm)){
        PumpPwm = ppwm;
        set_pwm(PUMPACHAN, PumpPwm);
        set_pwm(PUMPBCHAN, PumpPwm);
        if(nvs_set_u32(Nvs, "pumppwm", PumpPwm) == ESP_OK){
          commit(Nvs);
        }
      }
    }
  );
}

// run wifi loop, sample sensors. returns ambient temp.
static float
sampleSensors(void){
  float ambient_temp = NAN;
  static bool onewire_connected;
  if(!onewire_connected){
    if(connect_onewire(&digtemp) == 0){
      onewire_connected = true;
      uint8_t addr;
      if(digtemp.getAddress(&addr, 0)){
        Serial.print("digtemp 0 address: ");
        Serial.println(addr);
      }
      uint8_t res, dev;
      dev = 0;
      res = digtemp.getResolution(&dev);
      printf("therm resolution: %u bits\n", res);
      if(digtemp.setResolution(&dev, 9)){
        printf("set resolution to 9 bits\n");
      }
    }
  }
  if(onewire_connected){
    if(readAmbient(&ambient_temp, &digtemp)){
      onewire_connected = false;
      ambient_temp = NAN;
    }
  }
  displayDraw(ambient_temp);
  return ambient_temp;

}

// we transmit approximately every 15s, sampling at that time. there are
// several blocking calls (1-wire and MQTT) that can lengthen a given cycle.
static void
fanmgrLoop(int ledpin, float ambient){
  unsigned long m = micros();
  client.loop(); // handle any necessary wifi/mqtt
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  unsigned long diff = m - last_tx;
  if(client.isConnected()){
    digitalWrite(ledpin, HIGH);
  }else{
    digitalWrite(ledpin, LOW);
  }
  if(last_tx){
    if(diff < 15000000){
      return;
    }
  }
  unsigned frpm, parpm, pbrpm;
  detachInterrupt(digitalPinToInterrupt(FANTACHPIN));
  detachInterrupt(digitalPinToInterrupt(PUMPATACHPIN));
  detachInterrupt(digitalPinToInterrupt(PUMPBTACHPIN));
    frpm = FanRpm;
    parpm = PumpARpm;
    pbrpm = PumpBRpm;
    FanRpm = PumpARpm = PumpBRpm = 0;
  init_tach(PUMPBTACHPIN, rpm_pumpb);
  init_tach(PUMPATACHPIN, rpm_pumpa);
  init_tach(FANTACHPIN, rpm_fan);
  last_tx = micros();
  mqttmsg mmsg(client);
  frpm = rpm(frpm, diff);
  if(frpm < RPMMAX){
    printf("fanrpm: %u\n", frpm);
    publish_pair(mmsg, "rpm", frpm);
  }
  parpm = rpm(parpm, diff);
  if(parpm < RPMMAX){
    printf("pumparpm: %u\n", parpm);
    publish_pair(mmsg, "pumparpm", parpm);
  }
  pbrpm = rpm(pbrpm, diff);
  if(pbrpm < RPMMAX){
    printf("pumpbrpm: %u\n", pbrpm);
    publish_pair(mmsg, "pumpbrpm", pbrpm);
  }
  publish_temps(mmsg, ambient);
  publish_pwm(mmsg, FanPwm, PumpPwm);
  publish_version(mmsg);
  if(mmsg.publish()){
    printf("successful xmit at %lu\n", m);
  }
  /*uint32_t freq = getCpuFrequencyMhz();
  printf("cpu freq: %lu xtal: %lu\n", freq, getXtalFrequencyMhz());*/
}

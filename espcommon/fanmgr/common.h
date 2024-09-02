// common routines for the ESP32 and ESP8266 implementations of fanmgr
// much of this is also used by arduino airmon, but copied FIXME
#include <float.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "espcommon.h"
#include "nvs_flash.h"
#include "nvs.h"

#define VERSION "2.7.0"

static bool usingDisplay;
static const ledc_channel_t FANCHAN = LEDC_CHANNEL_0;
static const ledc_channel_t PUMPACHAN = LEDC_CHANNEL_1;
static const ledc_channel_t PUMPBCHAN = LEDC_CHANNEL_2;

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

void onMqttConnect(esp_mqtt_client_handle_t cli){
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
          nvs_commit(Nvs);
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
          nvs_commit(Nvs);
        }
      }
    }
  );
}

static void
get_rpms(unsigned *frpm, unsigned *parpm, unsigned *pbrpm, bool zero,
         int fanpin, int pumpapin, int pumpbpin){
  detachInterrupt(digitalPinToInterrupt(fanpin));
  detachInterrupt(digitalPinToInterrupt(pumpapin));
  detachInterrupt(digitalPinToInterrupt(pumpbpin));
    *frpm = FanRpm;
    *parpm = PumpARpm;
    *pbrpm = PumpBRpm;
    if(zero){
      FanRpm = PumpARpm = PumpBRpm = 0;
    }
  init_tach(pumpbpin, rpm_pumpb);
  init_tach(pumpapin, rpm_pumpa);
  init_tach(fanpin, rpm_fan);
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

static int
displayCoreSetup(void){
  if(!disp.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
    printf("SSD1306 allocation failed\n");
    return -1;
  }
  usingDisplay = true;
  return 0;
}

#define TEXTHEIGHT 10

static int
displayDraw(float ambient, int fanpin, int pumpapin, int pumpbpin){
  if(!usingDisplay){
    if(displayCoreSetup()){
      return -1;
    }
  }
  disp.clearDisplay();
  disp.setTextSize(1);
  disp.setTextColor(WHITE);
  disp.setCursor(0, 0);
  disp.println("inaMORAta v" VERSION);
  if(isnan(ambient)){
    disp.printf(DEVNAME " --");
  }else{
    disp.printf(DEVNAME " %0.2f C", ambient);
  }
  unsigned frpm, parpm, pbrpm;
  get_rpms(&frpm, &parpm, &pbrpm, false, fanpin, pumpapin, pumpbpin);
  disp.setCursor(0, 2 * TEXTHEIGHT + 1);
  if(frpm >= RPMMAX || (!frpm && FanPwm)){
    disp.printf("fans: %u --\n", FanPwm);
  }else{
    disp.printf("fans: %u %lu\n", FanPwm, frpm);
  }
  disp.setCursor(0, 3 * TEXTHEIGHT + 2);
  if(parpm >= RPMMAX || (!parpm && PumpPwm)){
    disp.printf("pump a: %u --\n", PumpPwm);
  }else{
    disp.printf("pump a: %u %lu\n", PumpPwm, parpm);
  }
  if(pbrpm >= RPMMAX || (!pbrpm && PumpPwm)){
    disp.printf("pump b: %u --\n", PumpPwm);
  }else{
    disp.printf("pump b: %u %lu\n", PumpPwm, pbrpm);
  }
  char tempstr[16];
  maketimestr(tempstr);
  disp.setCursor(0, SCREEN_HEIGHT - TEXTHEIGHT);
  disp.printf("uptime: %s", tempstr);
  disp.display();
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
displaySetup(int sclpin, int sdapin, int fanpin, int pumpapin, int pumpbpin){
  Wire.setPins(sdapin, sclpin);
  return displayDraw(NAN, fanpin, pumpapin, pumpbpin);
}

static void
fanmgrSetup(int ledpin, int fanpin, int pumpapin, int pumpbpin,
            int fantachpin, int pumpatachpin, int pumpbtachpin){
  Serial.begin(115200);
  Serial.println("initializing!");
  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, LOW);
  //setCpuFrequencyMhz(80);
  initialize_25k_pwm(FANCHAN, fanpin, LEDC_TIMER_1);
  initialize_25k_pwm(PUMPACHAN, pumpapin, LEDC_TIMER_2);
  if(pumpapin != pumpbpin){
    initialize_25k_pwm(PUMPBCHAN, pumpbpin, LEDC_TIMER_3);
    set_pwm(PUMPBCHAN, PumpPwm);
  }
  set_pwm(FANCHAN, FanPwm);
  set_pwm(PUMPACHAN, PumpPwm);
  init_tach(fantachpin, rpm_fan);
  init_tach(pumpatachpin, rpm_pumpa);
  init_tach(pumpbtachpin, rpm_pumpb);
  nvs_setup(&Nvs);
  mqtt_setup(client);
  printf("Fan PWM initialized to %u\n", FanPwm);
  printf("Pump PWM initialized to %u\n", PumpPwm);
  digitalWrite(ledpin, HIGH);
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

// run wifi loop, sample sensors. returns ambient temp.
static float
sampleSensors(int fanpin, int pumpapin, int pumpbpin){
  float ambient_temp = NAN;
  displayDraw(ambient_temp, fanpin, pumpapin, pumpbpin);
  return ambient_temp;
}

// we transmit approximately every 15s, sampling at that time. there are
// several blocking calls (1-wire and MQTT) that can lengthen a given cycle.
static void
fanmgrLoop(int ledpin, float ambient, int fantachpin, int pumpatachpin, int pumpbtachpin){
  unsigned long m = micros();
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  unsigned long diff = m - last_tx;
  if(client.isConnected()){
    digitalWrite(ledpin, LOW);
  }else{
    digitalWrite(ledpin, HIGH);
  }
  if(last_tx){
    if(diff < 15000000){
      return;
    }
  }
  unsigned frpm, parpm, pbrpm;
  get_rpms(&frpm, &parpm, &pbrpm, true, fantachpin, pumpatachpin, pumpbtachpin);
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
  // go high for the duration of the transmit. we'll go low again when we
  // reenter fanmgrLoop() at the top, assuming we're connected.
  digitalWrite(ledpin, HIGH);
  if(mmsg.publish()){
    printf("successful xmit at %lu\n", m);
  }
  /*uint32_t freq = getCpuFrequencyMhz();
  printf("cpu freq: %lu xtal: %lu\n", freq, getXtalFrequencyMhz());*/
}

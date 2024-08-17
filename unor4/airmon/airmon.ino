#define DEVNAME "ARDUINOR4"
#define VERSION "0.0.98"
#include <pwm.h>
#include <Wire.h>
#include <limits.h>
#include <WiFiS3.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include "ArduinoSecrets.h"
#include <Adafruit_SSD1306.h>
#include <ArduinoMqttClient.h>
#include "Arduino_LED_Matrix.h"
#include <DallasTemperature.h>

static const int MQ4_PIN = A1;
static const int MQ9_PIN = A2;
static const int MQ6_PIN = A3;
static const int MQ135_PIN = A4;
static const int TACH_PIN = D2;
static const int PWM_PIN = D3;
static const int TEMP_PIN = D4;
static const int RELAY_PIN = D5;

WiFiClient wifi;
PwmOut pwmd3(D3);
ArduinoLEDMatrix matrix;
static bool usingDisplay;
MqttClient client(&wifi);
static volatile unsigned Pulses; // fan tach

static OneWire twire(TEMP_PIN);
static DallasTemperature digtemp(&twire);

// pixel dimensions of SSD1306 OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3c // i2c

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

struct sensor {
  int pin;
  const char* hw;
} sensors[] = {
  { MQ4_PIN, "MQ-4", },
  { MQ9_PIN, "MQ-9", },
  { MQ6_PIN, "MQ-6", },
  { MQ135_PIN, "MQ-135", }
};

typedef struct mqttmsg {
 private:
  MqttClient& mqtt;
  DynamicJsonDocument doc{BUFSIZ};
 public:
  mqttmsg(MqttClient& esp) :
    mqtt(esp)
    {}
  template<typename T> void add(const char* key, const T value){
    doc[key] = value;
  }
  bool publish(){
    add("uptimesec", millis() / 1000); // FIXME handle overflow
    char buf[257]; // PubSubClient limits messages to 256 bytes
    size_t n = serializeJson(doc, buf);
    mqtt.beginMessage("sensors/" DEVNAME);
    Serial.println(buf);
    mqtt.print(buf);
    return mqtt.endMessage();
  }
} mqttmsg;

int displayCoreSetup(void){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
    printf("SSD1306 allocation failed\n");
    return -1;
  }
  usingDisplay = true;
  return 0;
}

void maketimestr(char *str){
  uint64_t ticks = millis();
  ticks /= 1000;
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

int displayDraw(float ambient, int smoke){
  if(!usingDisplay){
    if(displayCoreSetup()){
      return -1;
    }
    usingDisplay = false; // FIXME
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("bambooster v" VERSION);
  if(isnan(ambient)){
    display.println(DEVNAME " --");
  }else{
    display.print(DEVNAME " ");
    display.println((int)ambient); // FIXME float
  }
  char tempstr[16];
  maketimestr(tempstr);
  display.print("uptime: ");
  display.println(tempstr);
  display.print("smoke: ");
  display.println(smoke);
  display.display();
  return 0;
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

static void tach_pulse(void){
  if(Pulses < UINT_MAX){ // saturate
    ++Pulses;
  }
}

static void setup_interrupt(int pin){
  pinMode(pin, INPUT_PULLUP);
  digitalWrite(pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(pin), tach_pulse, FALLING);
}

// pwm takes values on [0..255]. uses PWM_PIN (9 implies P303 / GTIOC7B).
static void setup_25kpwm(int pwm){
  pwmd3.pulse_perc(pwm);
}

void setup(){
  Serial.begin(115200);
  matrix.begin();
  matrix.loadFrame(LEDMATRIX_BOOTLOADER_ON);
  while(!Serial){
    ;
  }
  Serial.println("initialized serial");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
    Serial.println("couldn't initialize ssd1306");
  }else{
    Serial.println("initialized ssd1306");
    display.display(); // adafruit splash screen
  }
  if(WiFi.status() == WL_NO_MODULE){
    while(true){
      Serial.println("couldn't find wifi module");
    }
  }
  Serial.print("WiFi firmware: ");
  Serial.println(WiFi.firmwareVersion());
  analogReadResolution(14);
  for(unsigned i = 0 ; i < sizeof(sensors) / sizeof(*sensors) ; ++i){
    pinMode(sensors[i].pin, INPUT_PULLDOWN);
  }
  pinMode(A0, INPUT_PULLDOWN);
  setup_interrupt(TACH_PIN);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pwmd3.begin(25000.0f, 0.0f);
  client.setId(DEVNAME);
  client.setUsernamePassword(MQTTUSER, MQTTPASS);
}

void asample(const struct sensor* s){
  int a = analogRead(s->pin);
  printf("%s: %d\n", s->hw, a);
}

// check for a ten second interval on milliseconds respecting wraparound
unsigned long delta(unsigned long l, unsigned long now){
  unsigned long d;
  if(now < l){
    d = now + ULONG_MAX - l;
  }else{
    d = now - l;
  }
  return d;
}

// check for a one second interval on microseconds respecting wraparound
bool deltaTenSeconds(unsigned long l, unsigned long now){
  return delta(l, now) > 10000;
}

void get_temp(float *temp){
  static bool onewire_connected;

  *temp = NAN;
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
    if(readAmbient(temp, &digtemp)){
      onewire_connected = false;
    }
  }
}

void loop(){
  // last time we attempted to connect to WiFi. we hold off for ten
  // seconds following a failed attempt, as it is a blocking call.
  static unsigned long lastWifiAttempt;
  static unsigned long lastTachSample;
  static bool wifiup;
  float ambient_temp = NAN;

  unsigned long us = micros();
  if(!lastTachSample){
    lastTachSample = us;
  }else{
    unsigned long d = delta(lastTachSample, us);
    if(d >= 1000000){
      lastTachSample = us;
      noInterrupts();
      unsigned tach = Pulses;
      Pulses = 0;
      interrupts();
      Serial.print("raw pulse count: ");
      Serial.println(tach);
      // FIXME scale by delta
      Serial.print("usec delta: ");
      Serial.println(d);
    }
  }
  int status = WiFi.status();
  if(status != WL_CONNECTED){
    Serial.print("wifi status: ");
    Serial.println(status);
    unsigned long m = millis();
    // attempt reconnection immediately if we just lost our connection,
    // or we've never attempted to connect, or if it's been at least ten
    // seconds since we last tried to connect.
    if(wifiup || !lastWifiAttempt || deltaTenSeconds(lastWifiAttempt, m)){
      if(wifiup){
        matrix.loadFrame(LEDMATRIX_EMOJI_SAD);
        wifiup = false;
      }
      lastWifiAttempt = m;
      Serial.print("connecting to ssid ");
      Serial.println(SSID);
      status = WiFi.begin(SSID, WPAPASS);
    }
  }
  if(status == WL_CONNECTED){
    wifiup = true;
    matrix.loadFrame(LEDMATRIX_CLOUD_WIFI);
    if(!client.connected()){
      Serial.println("connecting to mqtt...");
      status = client.connect(BROKER, 1883);
      Serial.print("mqttconnect result: ");
      Serial.println(status);
    }
  }
  get_temp(&ambient_temp);
  for(unsigned i = 0 ; i < sizeof(sensors) / sizeof(*sensors) ; ++i){
    asample(&sensors[i]);
  }
  int smoke = analogRead(A0);
  Serial.print("smoke: ");
  Serial.println(smoke);
  if(client.connected()){
    mqttmsg m(client);
    m.publish();
  }
  displayDraw(ambient_temp, smoke);
  delay(5000);
}

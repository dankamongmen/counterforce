#define DEVNAME "ARDUINOR4"
#include <pwm.h>
#include <Wire.h>
#include <limits.h>
#include <WiFiS3.h>
#include <ArduinoJson.h>
#include <MQTTClient.h>
#include "ArduinoSecrets.h"
#include "Arduino_LED_Matrix.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

static const int TACH_PIN = A0;
static const int MQ4_PIN = A1;
static const int MQ9_PIN = A2;
static const int MQ6_PIN = A3;
static const int MQ135_PIN = A4;
static const int PWM_PIN = D9;

WiFiClient wifi;
PwmOut pwmd3(D3);
ArduinoLEDMatrix matrix;
CooperativeMultitasking tasks;
MQTTClient client(&tasks, &wifi, BROKER, 1883, DEVNAME, MQTTUSER, MQTTPASS);
MQTTTopic topic(&client, "sensors/" DEVNAME);
static volatile unsigned Pulses; // fan tach

// pixel dimensions of SSD1306 OLED
#define SCREEN_WIDTH 64
#define SCREEN_HEIGHT 64

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
  MQTTClient& mqtt;
  DynamicJsonDocument doc{BUFSIZ};
 public:
  mqttmsg(MQTTClient& esp) :
    mqtt(esp)
    {}
  template<typename T> void add(const char* key, const T value){
    doc[key] = value;
  }
  bool publish(){
    add("uptimesec", millis() / 1000); // FIXME handle overflow
    char buf[257]; // PubSubClient limits messages to 256 bytes
    size_t n = serializeJson(doc, buf);
    Serial.println(buf);
    return topic.publish(buf);
  }
} mqttmsg;

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
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    System.println("couldn't initialize ssd1306");
  }else{
    display.display(); // adafruit splash screen
  }
  while(!Serial){
    ;
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
  setup_interrupt(TACH_PIN);
  pinMode(PWM_PIN, OUTPUT);
  pwmd3.begin(25000.0f, 0.0f);
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

void loop(){
  // last time we attempted to connect to WiFi. we hold off for ten
  // seconds following a failed attempt, as it is a blocking call.
  static unsigned long lastWifiAttempt;
  static unsigned long lastTachSample;
  static bool wifiup;

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
      if(status == WL_CONNECTED){
        Serial.println("got a connection");
        wifiup = true;
        matrix.loadFrame(LEDMATRIX_CLOUD_WIFI);
        Serial.println("connecting to mqtt...");
        status = client.connect();
        Serial.print("mqttconnect result: ");
        Serial.println(status);
      }else{
        Serial.println("couldn't get wifi");
      }
    }
  }
  mqttmsg m(client);
  m.publish();
  for(unsigned i = 0 ; i < sizeof(sensors) / sizeof(*sensors) ; ++i){
    asample(&sensors[i]);
  }
  Serial.print("wifi rssi: ");
  Serial.println(WiFi.RSSI());
  delay(1000);
}

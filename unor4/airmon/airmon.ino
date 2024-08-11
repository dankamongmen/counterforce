#define DEVNAME "ARDUINOR4"
#include <limits.h>
#include <WiFiS3.h>
//#include <PwmOut.h>
#include <ArduinoMqttClient.h>
#include "ArduinoSecrets.h"
#include "Arduino_LED_Matrix.h"

WiFiClient wifi;
MqttClient client(wifi);
ArduinoLEDMatrix matrix;
static volatile unsigned Pulses; // fan tach

static const int TACH_PIN = A0;
static const int MQ4_PIN = A1;
static const int MQ9_PIN = A2;
static const int MQ6_PIN = A3;
static const int MQ135_PIN = A4;
static const int PWM_PIN = D9;

struct sensor {
  int pin;
  const char* hw;
} sensors[] = {
  { MQ4_PIN, "MQ-4", },
  { MQ9_PIN, "MQ-9", },
  { MQ6_PIN, "MQ-6", },
  { MQ135_PIN, "MQ-135", }
};

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
  /*
  TCCR1A = 0;
  TCNT1 = 0;
  TCCR1A = _BV(COM1A1) | _BV(WGM11); // mode 10: ph. correct PWM, TOP = ICR1
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 320; // TOP = 320
  OCR1A = pwm;
  */
  pinMode(PWM_PIN, OUTPUT);
}

void setup(){
  Serial.begin(115200);
  matrix.begin();
  matrix.loadFrame(LEDMATRIX_BOOTLOADER_ON);
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
  setup_25kpwm(255);
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

void loop(){
  // last time we attempted to connect to WiFi. we hold off for ten
  // seconds following a failed attempt, as it is a blocking call.
  static unsigned long lastWifiAttempt;
  static unsigned long lastTachSample;
  static bool wifiup;
  static bool mqttup;

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
    unsigned long m = millis();
    // attempt reconnection immediately if we just lost our connection,
    // or we've never attempted to connect, or if it's been at least ten
    // seconds since we last tried to connect.
    if(wifiup || !lastWifiAttempt || deltaTenSeconds(lastWifiAttempt, m)){
      if(wifiup){
        matrix.loadFrame(LEDMATRIX_EMOJI_SAD);
        wifiup = false;
        mqttup = false;
      }
      lastWifiAttempt = m;
      Serial.print("connecting to ssid ");
      Serial.println(SSID);
      status = WiFi.begin(SSID, WPAPASS);
      if(status == WL_CONNECTED){
        Serial.println("got a connection");
        wifiup = true;
        matrix.loadFrame(LEDMATRIX_CLOUD_WIFI);
      }else{
        Serial.println("couldn't get wifi");
      }
    }
  }
  if(wifiup && !mqttup){
    status = client.connect(BROKER, 0);
    Serial.print("mqttconnect result: ");
    Serial.println(status);
    if(!status){
      mqttup = true;
    }
  }
  for(unsigned i = 0 ; i < sizeof(sensors) / sizeof(*sensors) ; ++i){
    asample(&sensors[i]);
  }
  Serial.print("wifi rssi: ");
  Serial.println(WiFi.RSSI());
  delay(1000);
}

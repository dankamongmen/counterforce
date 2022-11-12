// intended for use on a Hi-LetGo ESP8266, this manages a PWM fan and two
// PWM pumps, and two thermistors. it receives PWM control messages, and sends
// RPM and temperature reports, over MQTT. unlike the ESP32 version, this
// doesn't run LEDs, nor does it support a pressure sensor.
#include "ESP8266WiFi.h"
#include <float.h>
#include "EspMQTTClient.h"
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define VERSION "v2.0.3"

const unsigned long RPM_CUTOFF = 5000;

// only one ADC on the ESP8266
const int TEMPPIN = A0; // coolant thermistor (2-wire)
// ambient temperature (digital thermometer, Dallas 1-wire)
const int AMBIENTPIN = D2;

// PWM/tach pins for fans and pumps
const int PUMPPWMPIN = D5;
const int FANPWMPIN = D6;
const int FANTACHPIN = D1;
const int XTOPATACHPIN = D3;
const int XTOPBTACHPIN = D4;

// RPMs as determined by our interrupt handlers.
// we only get the RPM count from one of our fans; it stands for all.
static unsigned RPM;
static unsigned XTopRPMA;
static unsigned XTopRPMB;
static volatile unsigned long Pulses, XTAPulses, XTBPulses;

// PWMs we want to run at (initialized to INITIAL_*_PWM, read from MQTT)
#define INITIAL_FAN_PWM  192
#define INITIAL_PUMP_PWM 128
static unsigned Pwm;
static unsigned PumpPwm;
static unsigned XTopPWMA;
static unsigned XTopPWMB;

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

static int readAmbient(float* t){
  digtemp.requestTemperatures();
  float tmp = digtemp.getTempCByIndex(0);
  if(tmp <= DEVICE_DISCONNECTED_C){
    Serial.println("error reading 1-wire temp");
    return -1;
  }
  *t = tmp;
  Serial.print("ambientC: ");
  Serial.println(*t);
  return 0;
}

void readThermistor(float* t){
  const float BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 298.15;
  const float R0 = 10100;
  const float R1 = 10000;
  const float VREF = 3.3;
  float v0 = analogRead(TEMPPIN);
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

void IRAM_ATTR fantach(void){
  ++Pulses;
}

void IRAM_ATTR xtop1tach(void){
  ++XTAPulses;
}

void IRAM_ATTR xtop2tach(void){
  ++XTBPulses;
}

static bool ds18b20_connected_p(DallasTemperature* dt){
  int dcount = dt->getDS18Count();
  if(dcount == 0){
    return false;
  }
  Serial.print("DS18xxx devices: ");
  Serial.println(dcount);
  return true;
}

// attempt to establish a connection to the DS18B20
static int connect_onewire(void){
  digtemp.begin();
  int devcount = digtemp.getDeviceCount();
  if(devcount){
    Serial.print("1-Wire devices: ");
    Serial.println(devcount);
    return 0;
  }
  return -1;
}

void setup(){
  int error = 0;
  Serial.begin(115200);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  analogWriteFreq(25000);
  pinMode(PUMPPWMPIN, OUTPUT);
  pinMode(FANPWMPIN, OUTPUT);
  set_pwm(INITIAL_FAN_PWM);
  set_pump_pwm(INITIAL_PUMP_PWM);
  pinMode(TEMPPIN, INPUT);
  pinMode(FANTACHPIN, INPUT_PULLUP);
  pinMode(XTOPATACHPIN, INPUT_PULLUP);
  pinMode(XTOPBTACHPIN, INPUT_PULLUP);
  attachInterrupt(FANTACHPIN, fantach, FALLING);
  attachInterrupt(XTOPATACHPIN, xtop1tach, FALLING);
  attachInterrupt(XTOPBTACHPIN, xtop2tach, FALLING);
}

// set up the desired PWM value
static int set_pwm(unsigned p){
  analogWrite(FANPWMPIN, p);
  Serial.print("configured fan PWM: ");
  Serial.println(p);
  Pwm = p;
  return 0;
}

// set up the desired pump PWM value
static int set_pump_pwm(unsigned p){
  analogWrite(PUMPPWMPIN, p);
  Serial.print("configured pump PWM: ");
  Serial.println(p);
  PumpPwm = p;
  return 0;
}

// precondition: isxdigit(c) is true
static byte getHex(char c){
  if(isdigit(c)){
    return c - '0';
  }
  c = tolower(c);
  return c - 'a' + 10;
}

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
  client.subscribe("control/mora3/pwm", [](const String &payload){
      Serial.print("received PWM via mqtt: ");
      Serial.println(payload);
      unsigned long p = 0;
      if(payload.length() == 0){
        Serial.println("empty PWM input");
        return;
      }
      for(int i = 0 ; i < payload.length() ; ++i){
        char h = payload.charAt(i);
        if(!isdigit(h)){
          Serial.println("invalid PWM character");
          return;
        }
        p *= 10; // FIXME check for overflow
        p += h - '0';
      }
      if(p < 256){
        set_pwm(p);
      }
    }
  );
  client.subscribe("control/mora3/pumppwm", [](const String &payload){
      Serial.print("received pump PWM via mqtt: ");
      Serial.println(payload);
      unsigned long p = 0;
      if(payload.length() == 0){
        Serial.println("empty PWM input");
        return;
      }
      for(int i = 0 ; i < payload.length() ; ++i){
        char h = payload.charAt(i);
        if(!isdigit(h)){
          Serial.println("invalid PWM character");
          return;
        }
        p *= 10; // FIXME check for overflow
        p += h - '0';
      }
      if(p < 256){
        set_pump_pwm(p);
      }
    }
  );
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
    return mqttPublish(client, key, val);
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

// we transmit approximately every 15 seconds, sampling RPMs at this time.
// we continuously sample the temperature, and use the most recent valid read
// for transmit/display. there are several blocking calls (1-wire and MQTT)
// that can lengthen a given cycle.
void loop(){
  delay(100);
  static bool onewire_connected;
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  // these are the most recent valid reads (i.e. we don't reset to FLT_MAX
  // on error, but instead only on transmission).
  static float coolant_temp = FLT_MAX;
  static float ambient_temp = FLT_MAX;
  readThermistor(&coolant_temp);
  unsigned long m = micros();
  client.loop(); // handle any necessary wifi/mqtt

  if(!onewire_connected){
    if(connect_onewire() == 0){
      onewire_connected = true;
    }
  }
  if(onewire_connected){
    if(readAmbient(&ambient_temp)){
      onewire_connected = false;
    }
  }
  unsigned long diff = m - last_tx;
  if(diff < 15000000){
    return;
  }
  // sample RPM, transmit, and update the display
  // FIXME replace with MEGA's rolling 5s logic
  last_tx = m;
  noInterrupts();
  unsigned long p = Pulses;
  Pulses = 0;
  unsigned long x1p = XTAPulses;
  XTAPulses = 0;
  unsigned long x2p = XTBPulses;
  XTBPulses = 0;
  interrupts();
  Serial.print(diff);
  Serial.println(" µsec expired for cycle");
  Serial.print(p);
  Serial.println(" pulses measured at fan");
  RPM = rpm(p, diff);
  XTopRPMA = rpm(x1p, diff);
  XTopRPMB = rpm(x2p, diff);
  rpmPublish(client, "moraxtop0rpm", XTopRPMA);
  rpmPublish(client, "moraxtop1rpm", XTopRPMB);
  rpmPublish(client, "rpm", RPM);
  if(valid_temp(coolant_temp)){
    mqttPublish(client, "therm", coolant_temp);
  }else{
    Serial.println("don't have a coolant sample");
  }
  // there are several error codes returned by DallasTemperature, all of
  // them equal to or less than DEVICE_DISCONNECTED_C (there are also
  // DEVICE_FAULT_OPEN_C, DEVICE_FAULT_SHORTGND_C, and
  // DEVICE_FAULT_SHORTVDD_C).
  if(valid_temp(ambient_temp)){
    mqttPublish(client, "moraambient", ambient_temp);
  }else{
    Serial.println("don't have an ambient sample");
  }
  coolant_temp = FLT_MAX;
  ambient_temp = FLT_MAX;
}

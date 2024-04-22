// intended for use on a LilyGO/TTGO-Koala Type C ESP32 WROVER B
// https://opencircuit.shop/product/lilygo-ttgo-t-koala-esp32-wrover
//
// samples digital ambient 1-wire thermistor,
//         analog coolant thermistor
// controls 12V RGB LEDs via 3 MOSFETs

#define DEVNAME "demo"

#include <float.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <DallasTemperature.h>
#include "common.h"

// ambient temperature (digital thermometer, Dallas 1-wire)
const int AMBIENTPIN = 4;

const ledc_channel_t FANCHAN = LEDC_CHANNEL_0;
const int FANPWMPIN = 21;
const int FANTACHPIN = 23;
const int PUMPATACHPIN = 22;
const int PUMPBTACHPIN = 19;

static volatile unsigned FanRpm;
static volatile unsigned PumpARpm;
static volatile unsigned PumpBRpm;

void IRAM_ATTR rpm_fan(void){
  if(FanRpm < 65536){
    ++FanRpm;
  }
}

void IRAM_ATTR rpm_pumpa(void){
  if(PumpARpm < 65536){
    ++PumpARpm;
  }
}

void IRAM_ATTR rpm_pumpb(void){
  if(PumpBRpm < 65536){
    ++PumpBRpm;
  }
}

static unsigned FanPwm = 0xc0;
static unsigned PumpPwm = 0xc0;

EspMQTTClient client(
  #include "EspMQTTConfig.h",
  DEVNAME
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

#define FANPWM_BIT_NUM LEDC_TIMER_8_BIT
#define FANPWM_TIMER LEDC_TIMER_1

void init_tach(int pin, void(*fxn)(void)){
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(pin), fxn, FALLING);
}

int initialize_pwm(ledc_channel_t channel, int pin, int freq){
  ledc_channel_config_t conf;
  memset(&conf, 0, sizeof(conf));
  conf.gpio_num = pin;
  conf.speed_mode = LEDC_HIGH_SPEED_MODE;
  conf.intr_type = LEDC_INTR_DISABLE;
  conf.timer_sel = FANPWM_TIMER;
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
  ledc_timer.timer_num = FANPWM_TIMER;
  ledc_timer.freq_hz = freq;
  if(ledc_timer_config(&ledc_timer) != ESP_OK){
    Serial.println("error (timer config)!");
    return -1;
  }
  init_tach(FANTACHPIN, rpm_fan);
  init_tach(PUMPATACHPIN, rpm_pumpa);
  init_tach(PUMPBTACHPIN, rpm_pumpb);
  Serial.println("success!");
  return 0;
}

// set up the desired PWM values
static int set_pwm(const ledc_channel_t channel){
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, FanPwm) != ESP_OK){
    Serial.println("error setting red!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel) != ESP_OK){
    Serial.println("error committing red!");
    return -1;
  }
  Serial.println("configured pwm");
  return 0;
}

static int initialize_fan_pwm(ledc_channel_t channel, int pin){
  return initialize_pwm(channel, pin, 25000);
}

void setup(){
  Serial.begin(115200);
  Serial.println("initializing!");
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  client.enableHTTPWebUpdater();
  initialize_fan_pwm(FANCHAN, FANPWMPIN);
  set_pwm(FANCHAN);
  Serial.println("initialized!");
}

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
  client.subscribe("control/" DEVNAME "/fanpwm", [](const String &payload){
      Serial.print("received pwm via mqtt: ");
      Serial.println(payload);
      if(payload.length() != 2){
        Serial.println("pwm wasn't 2 characters");
        return;
      }
      char h = payload.charAt(0);
      char l = payload.charAt(1);
      if(!isxdigit(h) || !isxdigit(l)){
        Serial.println("invalid hex character");
        return;
      }
      byte hb = getHex(h);
      byte lb = getHex(l);
      // everything was valid; update globals
      FanPwm = hb * 16 + lb;
      set_pwm(FANCHAN);
    }
  );
}

// we transmit approximately every 15s, sampling at that time. there are
// several blocking calls (1-wire and MQTT) that can lengthen a given cycle.
void loop(){
  static bool onewire_connected;
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  float ambient_temp = NAN;
  client.loop(); // handle any necessary wifi/mqtt
  if(!onewire_connected){
    if(connect_onewire(&digtemp) == 0){
      onewire_connected = true;
    }
  }
  if(onewire_connected){
    if(readAmbient(&ambient_temp, &digtemp)){
      onewire_connected = false;
      ambient_temp = FLT_MAX;
    }
  }
  unsigned long m = micros();
  unsigned long diff = m - last_tx;
  if(last_tx){
    if(diff < 15000000){
      return;
    }
  }
  Serial.println("TRANSMIT");
  mqttmsg mmsg(client);
  // fan tach
    detachInterrupt(digitalPinToInterrupt(FANTACHPIN));
    unsigned zrpm = FanRpm;
    FanRpm = 0;
    attachInterrupt(digitalPinToInterrupt(FANTACHPIN), rpm_fan, FALLING);
    Serial.print("FanRpm: ");
    Serial.println(zrpm);
    unsigned rpm = zrpm / 2.0 * (60000000.0 / diff);
    publish_pair(mmsg, "rpm", rpm);
  publish_temps(mmsg, ambient_temp, NAN);
  publish_pwm(mmsg, FanPwm, PumpPwm);
  Serial.print("fan rpm: ");
  Serial.println(rpm);
  // pump a tachs
    detachInterrupt(digitalPinToInterrupt(PUMPATACHPIN));
    zrpm = PumpARpm;
    PumpARpm = 0;
    attachInterrupt(digitalPinToInterrupt(PUMPATACHPIN), rpm_pumpa, FALLING);
    Serial.print("PumpARpm: ");
    Serial.println(zrpm);
    rpm = zrpm / 2.0 * (60000000.0 / diff);
    publish_pair(mmsg, "pumparpm", rpm);
  // pump b tachs
    detachInterrupt(digitalPinToInterrupt(PUMPBTACHPIN));
    zrpm = PumpBRpm;
    PumpBRpm = 0;
    attachInterrupt(digitalPinToInterrupt(PUMPBTACHPIN), rpm_pumpb, FALLING);
    Serial.print("PumpBRpm: ");
    Serial.println(zrpm);
    rpm = zrpm / 2.0 * (60000000.0 / diff);
    publish_pair(mmsg, "pumpbrpm", rpm);
  if(mmsg.publish()){
    Serial.print("Successful xmit at ");
    Serial.println(m);
    last_tx = micros();
  }
}

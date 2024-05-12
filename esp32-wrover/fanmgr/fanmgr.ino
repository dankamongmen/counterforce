#define DEVNAME "s-mora"

#include <float.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <DallasTemperature.h>
#include "common.h"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 27;

static const ledc_channel_t FANCHAN = LEDC_CHANNEL_0;
static const ledc_channel_t PUMPACHAN = LEDC_CHANNEL_1;
static const ledc_channel_t PUMPBCHAN = LEDC_CHANNEL_2;

static const int FANPWMPIN = 26;
static const int FANTACHPIN = TX2;

static const int PUMPAPWMPIN = 12;
static const int PUMPATACHPIN = 10;

static const int PUMPBPWMPIN = 32;
static const int PUMPBTACHPIN = RX2;

static volatile unsigned FanRpm;
static volatile unsigned PumpARpm;
static volatile unsigned PumpBRpm;

#define RPMMAX (1u << 13u)

void IRAM_ATTR rpm_fan(void){
  if(FanRpm < RPMMAX){
    ++FanRpm;
  }
}

void IRAM_ATTR rpm_pumpa(void){
  if(PumpARpm < RPMMAX){
    ++PumpARpm;
  }
}

void IRAM_ATTR rpm_pumpb(void){
  if(PumpBRpm < RPMMAX){
    ++PumpBRpm;
  }
}

static unsigned FanPwm = 0xc0;
static unsigned PumpPwm = 0x80;

EspMQTTClient client(
  #include "EspMQTTConfig.h",
  DEVNAME
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

#define FANPWM_BIT_NUM LEDC_TIMER_8_BIT

void init_tach(int pin, void(*fxn)(void)){
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(pin), fxn, FALLING);
}

int initialize_pwm(ledc_channel_t channel, int pin, int freq, ledc_timer_t timer){
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
  init_tach(FANTACHPIN, rpm_fan);
  init_tach(PUMPATACHPIN, rpm_pumpa);
  init_tach(PUMPBTACHPIN, rpm_pumpb);
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
  Serial.println("configured pwm");
  return 0;
}

static int initialize_25k_pwm(ledc_channel_t channel, int pin, ledc_timer_t timer){
  return initialize_pwm(channel, pin, 25000, timer);
}

void setup(){
  Serial.begin(115200);
  Serial.println("initializing!");
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  client.enableHTTPWebUpdater();
  initialize_25k_pwm(FANCHAN, FANPWMPIN, LEDC_TIMER_1);
  initialize_25k_pwm(PUMPACHAN, PUMPAPWMPIN, LEDC_TIMER_2);
  initialize_25k_pwm(PUMPBCHAN, PUMPBPWMPIN, LEDC_TIMER_3);
  set_pwm(FANCHAN, FanPwm);
  set_pwm(PUMPACHAN, PumpPwm);
  set_pwm(PUMPBCHAN, PumpPwm);
  Serial.println("initialized!");
}

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
  client.subscribe("control/" DEVNAME "/fanpwm", [](const String &payload){
      Serial.print("received fan pwm via mqtt: ");
      Serial.println(payload);
      int fpwm = extract_pwm(payload);
      if(fpwm >= 0){
        FanPwm = fpwm;
        set_pwm(FANCHAN, FanPwm);
      }
    }
  );
  client.subscribe("control/" DEVNAME "/pumppwm", [](const String &payload){
      Serial.print("received pump pwm via mqtt: ");
      Serial.println(payload);
      unsigned ppwm = extract_pwm(payload);
      if(ppwm >= 0){
        PumpPwm = ppwm;
        set_pwm(PUMPACHAN, PumpPwm);
        set_pwm(PUMPBCHAN, PumpPwm);
      }
    }
  );
}

// we transmit approximately every 15s, sampling at that time. there are
// several blocking calls (1-wire and MQTT) that can lengthen a given cycle.
void loop(){
  static bool onewire_connected;
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  unsigned frpm, parpm, pbrpm;
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
  if(frpm < RPMMAX){
    frpm = rpm(frpm, diff);
    Serial.print("FanRpm: ");
    Serial.println(frpm);
    publish_pair(mmsg, "rpm", frpm);
  }
  if(parpm < RPMMAX){
    parpm = rpm(parpm, diff);
    Serial.print("PumpARpm: ");
    Serial.println(parpm);
    publish_pair(mmsg, "pumparpm", parpm);
  }
  if(pbrpm < RPMMAX){
    pbrpm = rpm(pbrpm, diff);
    Serial.print("PumpBRpm: ");
    Serial.println(pbrpm);
    publish_pair(mmsg, "pumpbrpm", pbrpm);
  }
  publish_temps(mmsg, ambient_temp, NAN);
  publish_pwm(mmsg, FanPwm, PumpPwm);
  if(mmsg.publish()){
    Serial.print("Successful xmit at ");
    Serial.println(m);
  }
}

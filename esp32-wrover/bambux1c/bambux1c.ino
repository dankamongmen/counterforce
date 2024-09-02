// for use with an ESP32-WROOM
//
// sits in a bambux1c and controls:
//  a ceramic heating element using a 3V relay
//  a 12V fan for the heating element (tach + PWM)
//  a CCS811 VOC sensor
//  a 12V fan for the BentoBox VOC fans (tach + PWM)
//
// and reports stats via wireless MQTT
#define VERSION "0.0.1"
#define DEVNAME "bambumanager"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 16;

// the fan(s) for the heater
static const int HEATPWMPIN = 14;
static const int HEATTACHPIN = 35;

// the fan(s) in the bento box
static const int VOCPWMPIN = 2;
static const int VOCTACHPIN = 32;

static const int RELAYPIN = 15;

// SCL is pin 22, SDA is pin 21
static const int I2C_SCL = SCL;
static const int I2C_SDA = SDA;

static const int LEDPIN = 2;

#include <SparkFunCCS811.h>
#include "EspMQTTConfig.h"
#include "espcommon.h"

static const ledc_channel_t HEATFANCHAN = LEDC_CHANNEL_0;
static const ledc_channel_t VOCFANCHAN = LEDC_CHANNEL_1;

// PWMs we want to run at (initialized here, read from NVS/MQTT)
static unsigned HeatPwm = 128;
static unsigned VOCPwm = 128;
static unsigned HeatPulses, VOCPulses;

#define MAXTEMP 60

// when HeaterTarget is 0, the heater ought always be off. otherwise, it ought
// be on iff the ambient temperature is less than the target temperature.
static unsigned HeaterTarget;

static float AmbientTemp;

CCS811 ccs811(-1);

void set_relay_state(int rpin, unsigned htarg, float ambient){
  if(htarg == 0){
    digitalWrite(RELAYPIN, LOW);
  }else if(!valid_temp(ambient)){
    digitalWrite(RELAYPIN, LOW);
    printf("ambient temp %f was invalid, forcing heater off\n", ambient);
  }else if(ambient > htarg){
    digitalWrite(RELAYPIN, LOW);
  }else{
    digitalWrite(RELAYPIN, HIGH);
  }
}

void onMqttConnect(esp_mqtt_client_handle_t cli){
  Serial.println("got an MQTT connection");
  client.subscribe("control/" DEVNAME "/heater", [](const String &payload){
      printf("received heater control via mqtt: %s\n", payload);
      int htarg = extract_number(payload);
      if(htarg < 0 || htarg > MAXTEMP){
        printf("invalid heater target: %d\n", htarg);
      }else{
        HeaterTarget = htarg;
        set_relay_state(RELAYPIN, HeaterTarget, AmbientTemp);
        // FIXME write to Nvs
      }
    }
  );
  client.subscribe("control/" DEVNAME "/heatfanpwm", [](const String &payload){
      printf("received hfan pwm via mqtt: %s\n", payload);
      int hpwm = extract_pwm(payload);
      if(valid_pwm_p(hpwm)){
        HeatPwm = hpwm;
        set_pwm(HEATFANCHAN, HeatPwm);
        if(nvs_set_u32(Nvs, "heatpwm", HeatPwm) == ESP_OK){
          nvs_commit(Nvs);
        }
      }
    }
  );
}

void ISR heatfan_isr(void){
  if(HeatPulses < RPMMAX){
    ++HeatPulses;
  }
}

void ISR vocfan_isr(void){
  if(VOCPulses < RPMMAX){
    ++VOCPulses;
  }
}

int bambumanager_nvs_setup(nvs_handle_t* nh){
  uint32_t pwm;
  if(nvs_get_u32(*nh, "hpwm", &pwm) == ESP_OK && valid_pwm_p(pwm)){
    HeatPwm = pwm;
    printf("no valid hpwm in persistent store\n");
  }
  if(nvs_get_u32(*nh, "vpwm", &pwm) == ESP_OK && valid_pwm_p(pwm)){
    VOCPwm = pwm;
  }else{
    printf("no valid vpwm in persistent store\n");
  }
  return 0;
}

void bambumanager_setup(int heatfanpin, int vocfanpin, int ledpin,
                        int relaypin, int heattachpin, int voctachpin,
                        ledc_channel_t heatchan, ledc_channel_t vocchan){
  Serial.begin(115200);
  printf("initializing\n");
  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, LOW);
  pinMode(relaypin, OUTPUT);
  digitalWrite(relaypin, LOW); // always turn off the heater by default
  AmbientTemp = getAmbient();
  set_relay_state(relaypin, HeaterTarget, AmbientTemp);
  initialize_25k_pwm(HEATFANCHAN, heatfanpin, LEDC_TIMER_1);
  initialize_25k_pwm(VOCFANCHAN, vocfanpin, LEDC_TIMER_2);
  if(!nvs_setup(&Nvs)){
    bambumanager_nvs_setup(&Nvs);
  }
  set_pwm(heatchan, HeatPwm);
  set_pwm(vocchan, VOCPwm);
  init_tach(heattachpin, heatfan_isr);
  init_tach(voctachpin, vocfan_isr);
  mqtt_setup(client);
  printf("initialized!\n");
  digitalWrite(ledpin, HIGH);
}

void setup(void){
  bambumanager_setup(HEATPWMPIN, VOCPWMPIN, LEDPIN, RELAYPIN,
                     HEATTACHPIN, VOCTACHPIN, HEATFANCHAN, VOCFANCHAN);
}

void get_rpms(unsigned *hrpm, unsigned *vrpm, bool zero, int hpin, int vpin){
  detachInterrupt(digitalPinToInterrupt(hpin));
  detachInterrupt(digitalPinToInterrupt(vpin));
  *hrpm = HeatPulses;
  *vrpm = VOCPulses;
  if(zero){
    HeatPulses = VOCPulses = 0;
  }
  init_tach(hpin, heatfan_isr);
  init_tach(vpin, vocfan_isr);
}

void publish_heattarg(mqttmsg& mmsg, unsigned htarg){
  mmsg.add("htarg", htarg);
}

void bambumanager_loop(int ledpin, int htachpin, int vtachpin, int relaypin){
  static bool gotccs = false;
  
  if(client.isConnected()){
    digitalWrite(ledpin, LOW);
  }else{
    digitalWrite(ledpin, HIGH);
  }
  AmbientTemp = getAmbient();
  set_relay_state(relaypin, HeaterTarget, AmbientTemp);
  if(!gotccs){
    if(ccs811.begin()){
      printf("initialized CCS811\n");
      gotccs = true;
    }else{
      printf("failure initializing ccs811\n");
    }
  }
  if(ccs811.dataAvailable()){
    printf("got ccs data!\n");
    // FIXME
  }
  unsigned long m = micros();
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  unsigned long diff = m - last_tx;
  if(last_tx){
    if(diff < 15000000){
      return;
    }
  }
  last_tx = micros();
  mqttmsg mmsg(client);
  publish_version(mmsg);
  unsigned hpulses, vpulses;
  get_rpms(&hpulses, &vpulses, true, htachpin, vtachpin);
  last_tx = micros();
  hpulses = rpm(hpulses, diff);
  if(hpulses < RPMMAX){
    printf("heatfanrpm: %u\n", hpulses);
    publish_pair(mmsg, "heatrpm", hpulses);
  }
  vpulses = rpm(vpulses, diff);
  if(vpulses < RPMMAX){
    printf("vocfanrpm: %u\n", vpulses);
    publish_pair(mmsg, "vocrpm", vpulses);
  }
  publish_temps(mmsg, AmbientTemp);
  publish_htarg(mmsg, HeaterTarget);
  publish_pwm(mmsg, HeatPwm, VOCPwm);
  // go high for the duration of the transmit. we'll go low again when we
  // reenter fanmgrLoop() at the top, assuming we're connected.
  digitalWrite(ledpin, HIGH);
  if(mmsg.publish()){
    printf("successful xmit at %lu\n", m);
  }
}

void loop(void){
  bambumanager_loop(LEDPIN, HEATTACHPIN, VOCTACHPIN, RELAYPIN);
}

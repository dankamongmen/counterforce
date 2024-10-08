// for use with an ESP32-WROOM
//
// sits in a bambux1c and controls:
//  a ceramic heating element using a 3V relay
//  a 12V fan for the heating element (tach + PWM)
//  a 12V fan for the BentoBox VOC fans (tach + PWM)
//
// and reports stats via wireless MQTT
#define VERSION "1.0.0"
#define DEVNAME "bambumanager"

// ambient temperature (digital thermometer, Dallas 1-wire)
static const int AMBIENTPIN = 26;

// the fan(s) for the heater
static const int HEATPWMPIN = 13;
static const int HEATTACHPIN = 35;

// the fan(s) in the bento box
static const int VOCPWMPIN = 12;
static const int VOCTACHPIN = 34;

static const int RELAYPIN = 17;

static const int LEDPIN = 2;

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
  printf("got an MQTT connection\n");
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
  client.subscribe("control/" DEVNAME "/heatpwm", [](const String &payload){
      printf("received hfan pwm via mqtt: %s\n", payload);
      int hpwm = extract_pwm(payload);
      if(valid_pwm_p(hpwm)){
        HeatPwm = hpwm;
        set_pwm(HEATFANCHAN, HeatPwm);
        if(nvs_set_u32(Nvs, "hpwm", HeatPwm) == ESP_OK){
          nvs_commit(Nvs);
        }
      }
    }
  );
  client.subscribe("control/" DEVNAME "/vocpwm", [](const String &payload){
      printf("received vfan pwm via mqtt: %s\n", payload);
      int vpwm = extract_pwm(payload);
      if(valid_pwm_p(vpwm)){
        VOCPwm = vpwm;
        set_pwm(VOCFANCHAN, VOCPwm);
        if(nvs_set_u32(Nvs, "vpwm", VOCPwm) == ESP_OK){
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
  init_tach(heattachpin, heatfan_isr);
  init_tach(voctachpin, vocfan_isr);
  set_relay_state(relaypin, HeaterTarget, AmbientTemp);
  initialize_25k_pwm(HEATFANCHAN, heatfanpin, LEDC_TIMER_1);
  initialize_25k_pwm(VOCFANCHAN, vocfanpin, LEDC_TIMER_2);
  if(!nvs_setup(&Nvs)){
    bambumanager_nvs_setup(&Nvs);
  }
  set_pwm(heatchan, HeatPwm);
  set_pwm(vocchan, VOCPwm);
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

void bambumanager_loop(int ledpin, int htachpin, int vtachpin, int relaypin,
                       ledc_channel_t vchan){
  if(client.isConnected()){
    printf("mqtt is connected, drop it low\n");
    digitalWrite(ledpin, LOW);
  }else{
    printf("mqtt is not yet connected\n");
    digitalWrite(ledpin, HIGH);
  }
  AmbientTemp = getAmbient();
  set_relay_state(relaypin, HeaterTarget, AmbientTemp);
  unsigned long m = micros();
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  unsigned long diff = m - last_tx;
  if(last_tx){
    if(diff < 15000000){
      delay(1000);
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
  publish_heattarg(mmsg, HeaterTarget);
  publish_pwm(mmsg, "heatpwm", HeatPwm);
  publish_pwm(mmsg, "vocpwm", VOCPwm);
  // go high for the duration of the transmit. we'll go low again when we
  // reenter fanmgrLoop() at the top, assuming we're connected.
  digitalWrite(ledpin, HIGH);
  if(mmsg.publish()){
    printf("successful xmit at %lu\n", m);
  }
}

void loop(void){
  bambumanager_loop(LEDPIN, HEATTACHPIN, VOCTACHPIN, RELAYPIN, VOCFANCHAN);
}

// intended for use on a LilyGO/TTGO-Koala Type C ESP32 WROVER B
// https://opencircuit.shop/product/lilygo-ttgo-t-koala-esp32-wrover
//
// samples digital ambient 1-wire thermistor,
//         analog coolant thermistor
// controls 12V RGB LEDs via 3 MOSFETs

#define DEVNAME "vroom2"

#include <float.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <DallasTemperature.h>
#include "common.h"

const int TEMPPIN = 13; // coolant thermistor (2-wire)
// ambient temperature (digital thermometer, Dallas 1-wire)
const int AMBIENTPIN = 22;

// PWM channels for RGB fans
const ledc_channel_t FANCHANR = LEDC_CHANNEL_0;
const ledc_channel_t FANCHANG = LEDC_CHANNEL_1;
const ledc_channel_t FANCHANB = LEDC_CHANNEL_2;
const int RGBPINR = 27;
const int RGBPING = 14;
const int RGBPINB = 12;
// RGB we want for the 12V fan LEDs (initialized to green, read from MQTT)
int Red = 0x0;
int Green = 0xff;
int Blue = 0x0;

// RPMs as determined by our interrupt handlers.
// we only get the RPM count from one of our fans; it stands for all.
static unsigned RPM;
static unsigned XTopRPMA;
static unsigned XTopRPMB;

static unsigned Pwm;
static unsigned PumpPwm;

EspMQTTClient client(
  #include "EspMQTTConfig.h",
  DEVNAME
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

#define FANPWM_BIT_NUM LEDC_TIMER_8_BIT
#define FANPWM_TIMER LEDC_TIMER_1

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
  Serial.println("success!");
  return 0;
}

int initialize_rgb_pwm(ledc_channel_t channel, int pin){
  return initialize_pwm(channel, pin, 5000);
}

int initialize_fan_pwm(ledc_channel_t channel, int pin){
  return initialize_pwm(channel, pin, 25000);
}

void setup(){
  Serial.begin(115200);
  Serial.println("initializing!");
  pinMode(TEMPPIN, INPUT);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  initialize_rgb_pwm(FANCHANR, RGBPINR);
  initialize_rgb_pwm(FANCHANG, RGBPING);
  initialize_rgb_pwm(FANCHANB, RGBPINB);
  set_rgb();
  Serial.println("initialized!");
}

// set up the desired RGB PWM values
static int set_rgb(void){
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FANCHANR, Red) != ESP_OK){
    Serial.println("error setting red!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FANCHANR) != ESP_OK){
    Serial.println("error committing red!");
    return -1;
  }
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FANCHANB, Blue) != ESP_OK){
    Serial.println("error setting blue!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FANCHANB) != ESP_OK){
    Serial.println("error committing blue!");
    return -1;
  }
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FANCHANG, Green) != ESP_OK){
    Serial.println("error setting green!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FANCHANG) != ESP_OK){
    Serial.println("error committing green!");
    return -1;
  }
  Serial.println("configured RGB");
  return 0;
}

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
  client.subscribe("control/mora3/rgb", [](const String &payload){
      Serial.print("received RGB via mqtt: ");
      Serial.println(payload);
      byte colors[3]; // rgb
      if(payload.length() != 2 * sizeof(colors)){
        Serial.println("RGB wasn't 6 characters");
        return;
      }
      for(int i = 0 ; i < sizeof(colors) ; ++i){
        char h = payload.charAt(i * 2);
        char l = payload.charAt(i * 2 + 1);
        if(!isxdigit(h) || !isxdigit(l)){
          Serial.println("invalid RGB character");
          return;
        }
        byte hb = getHex(h);
        byte lb = getHex(l);
        colors[i] = hb * 16 + lb;
      }
      // everything was valid; update globals
      Serial.print("RGB request: ");
      Serial.print(colors[0]);
      Serial.print(" ");
      Serial.print(colors[1]);
      Serial.print(" ");
      Serial.println(colors[2]);
      Red = colors[0];
      Green = colors[1];
      Blue = colors[2];
      set_rgb();
    }
  );
}

// we transmit approximately every 15s, sampling at that time. there are
// several blocking calls (1-wire and MQTT) that can lengthen a given cycle.
void loop(){
  static bool onewire_connected;
  static unsigned long last_tx; // micros() when we last transmitted to MQTT
  static float coolant_temp = FLT_MAX;
  static float ambient_temp = FLT_MAX;
  client.loop(); // handle any necessary wifi/mqtt
  readThermistor(&coolant_temp, TEMPPIN, 4096);
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
  if(diff < 15000000){
    return;
  }
  // sample RPM, transmit, and update the display
  last_tx = m;
  Serial.print(diff);
  Serial.println(" µsec expired for cycle");
  mqttmsg mmsg(client);
  publish_uptime(mmsg, millis() / 1000); // FIXME handle overflow
  publish_temps(mmsg, ambient_temp, coolant_temp);
  publish_rgb(mmsg, Red, Blue, Green);
  mmsg.publish();
}

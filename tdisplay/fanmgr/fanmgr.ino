#define DEVNAME "ttgo"
#include <TFT_eSPI.h>
#include <OneWire.h>
#include <driver/ledc.h>
#include <DallasTemperature.h>
#include "common.h"

const int TEMPPIN = 17; // coolant thermistor (2-wire)
// ambient temperature (digital thermometer, Dallas 1-wire)
const int AMBIENTPIN = 37;

#define BUTTON_1            35
#define BUTTON_2            0
TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

EspMQTTClient client(
  #include "EspMQTTConfig.h",
  DEVNAME
);

OneWire twire(AMBIENTPIN);
DallasTemperature digtemp(&twire);

const int FANTACHPIN = 25;
const ledc_channel_t FANCHAN = LEDC_CHANNEL_0;
const int FANPWMPIN = 38;
static unsigned FanPwm = 0xc0;
static unsigned PumpPwm = 0xc0;
#define FANPWM_BIT_NUM LEDC_TIMER_8_BIT
#define FANPWM_TIMER LEDC_TIMER_1

// set up the desired PWM values
static int set_pwm(void){
  if(ledc_set_duty(LEDC_HIGH_SPEED_MODE, FANCHAN, FanPwm) != ESP_OK){
    Serial.println("error setting red!");
    return -1;
  }else if(ledc_update_duty(LEDC_HIGH_SPEED_MODE, FANCHAN) != ESP_OK){
    Serial.println("error committing red!");
    return -1;
  }
  Serial.println("configured pwm");
  return 0;
}

static volatile unsigned FanRpm;                                                                                                    
                                                                                                                                    
static void IRAM_ATTR rpm_fan(){                                                                                                           
  if(FanRpm < UINT_MAX){                                                                                                               
    ++FanRpm;                                                                                                                       
  }                                                                                                                                 
}

static void init_tach(int pin){
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(pin), rpm_fan, FALLING);
}

static int initialize_pwm(ledc_channel_t channel, int pin, int freq){
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
  init_tach(FANTACHPIN);
  Serial.println("success!");
  return 0;
}

static int initialize_fan_pwm(ledc_channel_t channel, int pin){
  return initialize_pwm(channel, pin, 25000);
}

void setup(){
  Serial.begin(115200);
  pinMode(TEMPPIN, INPUT);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
  client.enableHTTPWebUpdater();
  initialize_fan_pwm(FANCHAN, FANPWMPIN);
  set_pwm();
  tft.init();
  tft.fillScreen(TFT_GREEN);
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
      set_pwm();
    }
  );
}

void loop(){
}

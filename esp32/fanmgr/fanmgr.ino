// intended for use on a Heltec ESP32LoRav2, this manages (via UART) a device
// controlling a PWM fan. it receives PWM control messages, and sends RPM and
// temperature reports, over MQTT.
#include "heltec.h"
#include <float.h>
#include "EspMQTTClient.h"

EspMQTTClient client(
  #include "EspMQTTConfig.h"
);

void onConnectionEstablished() {
  Serial.println("got an MQTT connection");
}

void setup(){
  const byte INITIAL_PWM = 100;
  Heltec.begin(true  /*DisplayEnable Enable*/,
               false /*LoRa Disable*/,
               true  /*Serial Enable*/);
  Heltec.display->setFont(ArialMT_Plain_10);
  client.enableDebuggingMessages();
  client.enableMQTTPersistence();
}

// write a PWM value over UART
static void send_pwm(int pwm){
}

static int setPWM(int pwm){
  if(pwm < 0 || pwm > 100){
    Serial.print("invalid pwm: ");
    Serial.println(pwm);
    return -1;
  }
  Serial.print("PWM to ");
  Serial.println(pwm);
  Pwm = pwm;
  send_pwm(pwm);
  return 0;
}

// read bytes from Serial. each byte is interpreted as a PWM level, and
// ought be between [0..100]. we act on the last byte available.
void check_pwm_update(int iterations){
  int last = -1;
  int in;
  while((in = Serial.read()) != -1){
    Serial.print("read byte from input: ");
    Serial.println(in);
    last = in;
  }
  if(last >= 0){
    setPWM(last);
  }
}

void displayConnectionStatus(int y){
  const char* connstr;
  if(!client.isWifiConnected()){
    connstr = "No WiFi";
  }else if(!client.isMqttConnected()){
    connstr = "WiFi, no MQTT";
  }else{
    connstr = "Connected";
  }
  Heltec.display->drawString(120, y, connstr);
}

void loop(){
  const unsigned long LOOPUS = 1000000;

  // FIXME use the enqueue work version of this, as client.loop() can
  // block for arbitrary amounts of time
  client.loop(); // handle any necessary wifi/mqtt
  unsigned long m = micros();
  unsigned long cur;

  do{
    cur = micros();
    if(cur < m){     // handle micros() overflow...
      if(m + LOOPUS > m){
        break;
      }else if(cur > m + LOOPUS){
        break;
      }
    }
  }while(cur - m < LOOPUS);
  Serial.print(" PWM output: ");
  Serial.print(Pwm);
  Serial.println();

  // dump information to OLED
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->drawString(0, 0, "RPM: ");
  Heltec.display->drawString(30, 0, String(c));
  Heltec.display->drawString(0, 11, "PWM: ");
  Heltec.display->drawString(34, 11, String(Pwm));
  Heltec.display->drawString(0, 21, "Temp: ");
  if(therm == FLT_MAX){
    Heltec.display->drawString(36, 21, "n/a");
  }else{
    Heltec.display->drawString(36, 21, String(therm));
  }
  Heltec.display->drawString(0, 31, "Uptime: ");
  Heltec.display->drawString(40, 31, String(millis() / 1000));
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  displayConnectionStatus(51);
  Heltec.display->display();

  // FIXME if not connected, retain a buffer of values
  if(c != 65535){
    client.publish("mora3/rpms", String(c));
  }
  if(therm != FLT_MAX){
    client.publish("mora3/therm", String(therm));
  }
  check_pwm_update();
}

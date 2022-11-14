// common routines for the ESP32 and ESP8266 implementations of fanmgr
#include "EspMQTTClient.h"

int readAmbient(float* t, DallasTemperature *dt){
  dt->requestTemperatures();
  float tmp = dt->getTempCByIndex(0);
  if(tmp <= DEVICE_DISCONNECTED_C){
    Serial.println("error reading 1-wire temp");
    return -1;
  }
  *t = tmp;
  Serial.print("ambientC: ");
  Serial.println(*t);
  return 0;
}

void readThermistor(float* t, int pin){
  const float BETA = 3435; // https://www.alphacool.com/download/kOhm_Sensor_Table_Alphacool.pdf
  const float NOMINAL = 298.15;
  const float R0 = 10100;
  const float R1 = 10000;
  const float VREF = 3.3;
  float v0 = analogRead(pin);
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

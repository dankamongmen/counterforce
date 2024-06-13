#include <driver/adc.h>

void setup(void){
  Serial.begin(115200);
  adc1_config_width(ADC_WIDTH_BIT_12);
  // 37 and 38 (channels 1 and 2) are unavailable on ESP32-WROOM.
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_0);
}

void loop(void){
  int h2s = adc1_get_raw(ADC1_CHANNEL_0);
  int ch4 = adc1_get_raw(ADC1_CHANNEL_1);
  printf("H2S: %d CH4: %d\n", h2s, ch4);
  gpio_num_t adc1chan;
  adc1_pad_get_io_num(ADC1_CHANNEL_0, &adc1chan);
  printf("unspec: pin %d\n", adc1chan);
  adc1_pad_get_io_num(ADC1_CHANNEL_3, &adc1chan);
  printf("unspec: pin %d\n", adc1chan);
  adc1_pad_get_io_num(ADC1_CHANNEL_4, &adc1chan);
  printf("H2S: pin %d\n", adc1chan);
  adc1_pad_get_io_num(ADC1_CHANNEL_5, &adc1chan);
  printf("CH4: pin %d\n", adc1chan);
  adc1_pad_get_io_num(ADC1_CHANNEL_6, &adc1chan);
  printf("VOC: pin %d\n", adc1chan);
  adc1_pad_get_io_num(ADC1_CHANNEL_7, &adc1chan);
  printf("unspec: pin %d\n", adc1chan);
  delay(500);
}

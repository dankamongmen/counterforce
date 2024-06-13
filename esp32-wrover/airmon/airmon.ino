#include <driver/adc.h>

// returns -1 if there was an error prepping the channel
// ADC_ATTEN_DB_0 only allows reading up through 1.1V
// probably want to calibrate the ADC...
int init_adc_chan(const char *sensor, adc1_channel_t channel){
  if(adc1_config_channel_atten(channel, ADC_ATTEN_DB_0) != ESP_OK){
    printf("failed configuring ADC1 channel %d for %s\n", channel, sensor);
    return -1;
  }
  gpio_num_t adc1chan;
  if(adc1_pad_get_io_num(channel, &adc1chan) != ESP_OK){
    printf("failed getting gpio for ADC1 channel %d (%s)\n", channel, sensor);
    return -1;
  }
  if(gpio_set_direction(adc1chan, GPIO_MODE_INPUT) != ESP_OK){
    printf("failed setting gpio %d to input for %s\n", adc1chan, sensor);
    return -1;
  }
  if(gpio_pulldown_en(adc1chan) != ESP_OK){
    printf("failed enabling pulldown for gpio %d (%s)\n", adc1chan, sensor);
    return -1;
  }
  printf("%s: pin %d level: %d\n", sensor, adc1chan, gpio_get_level(adc1chan));
  return 0;
}

void setup(void){
  Serial.begin(115200);
  adc1_config_width(ADC_WIDTH_BIT_12);
  // 37 and 38 (channels 1 and 2) are unavailable on ESP32-WROOM.
  init_adc_chan("H2S", ADC1_CHANNEL_0);
  init_adc_chan("CH4", ADC1_CHANNEL_3);
  init_adc_chan("VOC", ADC1_CHANNEL_4);
  init_adc_chan("unspec", ADC1_CHANNEL_5);
  init_adc_chan("unspec", ADC1_CHANNEL_6);
  init_adc_chan("unspec", ADC1_CHANNEL_7);
  delay(500);
}

void loop(void){
  int h2s = adc1_get_raw(ADC1_CHANNEL_0);
  int ch4 = adc1_get_raw(ADC1_CHANNEL_3);
  int voc = adc1_get_raw(ADC1_CHANNEL_4);
  printf("H2S: %d CH4: %d VOC: %d\n", h2s, ch4, voc);
  delay(500);
}

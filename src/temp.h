#pragma once

#include  <Arduino.h>

#define TEMPERATRUE_ADC_BIT 12  // 设置分辨率为12bits (0-4095)

#define TEM_PIN 3

void adc_init();
float get_adc_temperatrue(uint8_t ch);


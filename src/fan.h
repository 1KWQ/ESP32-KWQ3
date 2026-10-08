#ifndef FAN_H
#define FAN_H

#include <Arduino.h>

// FAN
#define FAN_PWM_PIN    42
#define FAN_PWM_FREQ    25000   // 25kHz
#define FAN_PWM_RES     8       // 8bit 足够
#define FAN_DUTY_MAX   255   // 255

void  fanInit();
void  setFan(uint8_t duty);

void setFanDependDriverTemp(float t);

#endif

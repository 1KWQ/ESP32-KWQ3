#include "fan.h"
#include "common.h"
uint8_t duty = 80; // 默认 80%

static uint8_t fanLevel = 0;
static const int FAN_PWM_CHANNEL = 0;

void  fanInit(){
    // PWM 输出
    ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RES);
    ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);

    // 启动时给高占空比，防止不起转
    ledcWrite(FAN_PWM_CHANNEL, duty);
}

/**
 * @brief Set the Fan object 占空比
 * 
 * @param duty  占空比
 */
void setFan(uint8_t duty){
    ledcWrite(FAN_PWM_CHANNEL, duty);
}

/**
 * @brief Set the Fan Depend Driver Temp object
 * 
 * @param t max temp
 */
void setFanDependDriverTemp(float t)
{
    if (t < max_temp - 55.0)
        fanLevel = 10;
    else if (t < max_temp - 45.0)
        fanLevel = 30;
    else
        fanLevel = 50;
   //  ESP_LOGI("FAN", "fanLevel:%d",fanLevel );
    setFan(FAN_DUTY_MAX * fanLevel / 100);
}



#ifndef RYG_LED_H
#define RYG_LED_H

#include <Arduino.h>


#define RGB_PWM_PIN 8

#define PWM_PIN   RGB_PWM_PIN
#define PWM_FREQ  2000
#define PWM_RES   10   // 0~1023


void setVoltage(uint8_t v);

void rygLedInit();

void setRYG(bool r, bool y, bool g);


/**
 * @brief RYG 三色灯颜色定义
 */
typedef enum {
    RYG_COLOR_OFF = 0,
    RYG_COLOR_RED,
    RYG_COLOR_YELLOW,
    RYG_COLOR_GREEN,
    RYG_COLOR_NONE
} RYG_Color_t;

/**
 * @brief RYG 三色灯工作模式
 */
typedef enum {
    RYG_MODE_OFF = 0,
    RYG_MODE_ON,
    RYG_MODE_BLINK
} RYG_Mode_t;

/**
 * @brief RYG 灯控制消息结构
 */
typedef struct {
    RYG_Mode_t  mode;           ///< 工作模式
    RYG_Color_t color;          ///< 灯颜色
    uint32_t    blinkMs;        ///< 闪烁周期

    bool        setVoltage;     ///< 是否更新电压
    uint8_t       voltage;        ///< 新电压（setVoltage=true 时有效）
} RYG_Msg_t;


/**
 * @brief @brief 初始化 RYG 灯控制任务和队列
 * 
 */
void rygLedInitTask();

/**
 * @brief 发送“常亮”控制消息
 */
void rygLedOn(RYG_Color_t color);

/**
 * @brief 发送“闪烁”控制消息
 */
void rygLedBlink(RYG_Color_t color, uint32_t blinkMs);


void rygLedSetVoltage(uint8_t voltage);

/**
 * @brief 发送“关闭”控制消息
 */
void rygLedOff();

#endif



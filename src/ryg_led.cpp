#include "ryg_led.h"

#include "pca_gpio.h"
#include "esp_log.h"
#include "exio.h"
#include "common.h"

static const char *TAG = "ryg_led";
static const int RYG_PWM_CHANNEL = 1;

/**
 * @brief 设置目标电压并调整 PWM 占空比
 *
 * @param v 目标电压值（单位：V），范围：0V 到 24V
 */
void setVoltage(uint8_t v) {
  // 如果电压小于 0，设为 0（不可能为负值）
  if (v < 0) v = 0;
  
  // 如果电压大于 24V，设为 24（不超过 24V）
  if (v > 24) v = 24;
  
  // 计算占空比。PWM 最大值为 1023，换算公式： (v / 24) * 1023
  uint32_t duty = (v / 24.0) * 1023;
  
  // 使用 ledcWrite 函数把计算出的占空比发送到指定的 PWM 通道
  ledcWrite(RYG_PWM_CHANNEL, duty);
}

/**
 * @brief 初始化 PWM 设置
 */
void rygLedInit() {
   // 配置 PWM 引脚，频率为 PWM_FREQ，分辨率为 PWM_RES
   // 这会设置 PWM 引脚，并为后续的占空比控制做准备
    ledcSetup(RYG_PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, RYG_PWM_CHANNEL);
}

/**
 * @brief 控制 RYG 三色 LED 的开关状态
 * 
 * 该函数通过 I2C 控制 PCA 驱动器上的 3 个引脚，分别控制红、黄、绿三色 LED 的开关。
 * 每个 LED 通过布尔参数 r、y、g 控制：
 * - r 为 true 时，红色 LED 点亮；为 false 时，红色 LED 熄灭。
 * - y 为 true 时，黄色 LED 点亮；为 false 时，黄色 LED 熄灭。
 * - g 为 true 时，绿色 LED 点亮；为 false 时，绿色 LED 熄灭。
 * 
 * @param r 红色 LED 状态，true 表示点亮，false 表示熄灭
 * @param y 黄色 LED 状态，true 表示点亮，false 表示熄灭
 * @param g 绿色 LED 状态，true 表示点亮，false 表示熄灭
 */
void setRYG(bool r, bool y, bool g) {
   // 控制红色 LED，如果 r 为 true，点亮；否则熄灭
   if(r) pca.digitalWrite(RYG_R_PIN, 1); else pca.digitalWrite(RYG_R_PIN, 0);

   // 控制黄色 LED，如果 y 为 true，点亮；否则熄灭
   if(y) pca.digitalWrite(RYG_Y_PIN, 1); else pca.digitalWrite(RYG_Y_PIN, 0);

   // 控制绿色 LED，如果 g 为 true，点亮；否则熄灭
   if(g) pca.digitalWrite(RYG_G_PIN, 1); else pca.digitalWrite(RYG_G_PIN, 0);
}


/* ================= 内部对象 ================= */

static QueueHandle_t rygQueue = nullptr;
static TaskHandle_t  rygTaskHandle = nullptr;

/* ================= 底层 IO ================= */



/**
 * @brief 设置三色灯状态
 */
static void setRYG(RYG_Color_t color)
{
    switch (color)
    {
        case RYG_COLOR_OFF:
            pca.digitalWrite(RYG_R_PIN, 0);
            pca.digitalWrite(RYG_Y_PIN, 0);
            pca.digitalWrite(RYG_G_PIN, 0);
            break;
            case RYG_COLOR_RED:
            pca.digitalWrite(RYG_R_PIN, 1);
            pca.digitalWrite(RYG_Y_PIN, 0);
            pca.digitalWrite(RYG_G_PIN, 0);
            break;
        case RYG_COLOR_YELLOW:
            pca.digitalWrite(RYG_R_PIN, 0);
            pca.digitalWrite(RYG_Y_PIN, 1);
            pca.digitalWrite(RYG_G_PIN, 0);
            break;
        case RYG_COLOR_GREEN:
            pca.digitalWrite(RYG_R_PIN, 0);
            pca.digitalWrite(RYG_Y_PIN, 0);
            pca.digitalWrite(RYG_G_PIN, 1);
            break;
        default:
            break;
    }

    // if(color == RYG_COLOR_OFF){
    //     pca.digitalWrite(7, 0);
    //     pca.digitalWrite(6, 0);
    //     pca.digitalWrite(5, 0);
    // }
    // else{

    // }
    // pca.digitalWrite(7, color == RYG_COLOR_RED);
    // pca.digitalWrite(6, color == RYG_COLOR_YELLOW);
    // pca.digitalWrite(5, color == RYG_COLOR_GREEN);
}

/**
 * @brief RYG 三色灯控制任务（消息队列驱动）
 */
static void rygLedTask(void *arg)
{
    RYG_Mode_t  curMode   = RYG_MODE_OFF;
    RYG_Color_t curColor  = RYG_COLOR_OFF;
    uint32_t    curBlink  = 500;
    float       curVolt   = 0.0f;

    bool ledOn = false;
    uint32_t lastToggle = 0;

    while (1) {

        RYG_Msg_t msg;
        if (xQueueReceive(rygQueue, &msg, 0) == pdTRUE) {

            curMode  = msg.mode;
            curColor = msg.color;
            curBlink = msg.blinkMs;

            if (msg.setVoltage) {
                curVolt = msg.voltage;
                setVoltage(curVolt);
            }

            lastToggle = millis();
            ledOn = false;
        }

        switch (curMode) {
            case RYG_MODE_OFF:
                setRYG(RYG_COLOR_OFF);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            case RYG_MODE_ON:
                setRYG(curColor);
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            case RYG_MODE_BLINK:
                if (millis() - lastToggle >= curBlink) {
                    lastToggle = millis();
                    ledOn = !ledOn;
                }
                setRYG(ledOn ? curColor : RYG_COLOR_OFF);
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
        }
    }
}

void rygLedOn(RYG_Color_t color)
{
    RYG_Msg_t msg = {
        .mode       = RYG_MODE_ON,
        .color      = color,
        .blinkMs    = 0,
        .setVoltage = false
    };
    xQueueSend(rygQueue, &msg, 0);
}


void rygLedBlink(RYG_Color_t color, uint32_t blinkMs)
{
    RYG_Msg_t msg = {
        .mode       = RYG_MODE_BLINK,
        .color      = color,
        .blinkMs    = blinkMs,
        .setVoltage = false
    };
    xQueueSend(rygQueue, &msg, 0);
}

/**
 * @brief 设置电压
 * 
 * @param voltage 
 */
void rygLedSetVoltage(uint8_t voltage)
{
    RYG_Msg_t msg = {
        .mode       = RYG_MODE_ON,   // 不改变当前显示逻辑
        .color      = RYG_COLOR_NONE, // 会被忽略
        .blinkMs    = 0,
        .setVoltage = true,
        .voltage    = voltage
    };
    xQueueSend(rygQueue, &msg, 0);
}


/**
 * @brief 关闭h所有灯
 * 
 */
void rygLedOff()
{
    RYG_Msg_t msg = {
        .mode    = RYG_MODE_OFF,
        .color   = RYG_COLOR_OFF,
        .blinkMs = 0,
        .setVoltage = false,
    };
    xQueueSend(rygQueue, &msg, 0);
}


/**
 * @brief rygTaskInit
 * 
 */
void rygLedInitTask()
{
    /* PWM 初始化 */
     rygLedInit();
     setVoltage(ryg_voltage);
    /* 创建消息队列（长度 5） */
    rygQueue = xQueueCreate(5, sizeof(RYG_Msg_t));

    /* 创建任务 */
    BaseType_t res =xTaskCreate(rygLedTask, "ryg_led", 2048, NULL, 1, NULL);
    if(pdTRUE != res){
        ESP_LOGE(TAG, "rygLedTask creat failed");
    }
  
}

/**
 * @file exio.cpp
 * @author your name (you@domain.com)
 * @brief 外部扩展IO相关（ESP32-KWQ3）
 * @version 0.1
 * @date 2026-01-16
 *
 * @copyright Copyright (c) 2026
 *
 * @details KWQ3 只保留三色灯、电磁阀、门/样品盘/水瓶检测、系统电源控制与
 *          5 路驱动温度轮询。电阻检测（RES CHK）相关代码已整体移除；门/样品盘/
 *          水瓶等 PCA 按键回调目前只打印状态，待后续加入任务序列时再在此处
 *          接入暂停/告警逻辑。
 */

#include "exio.h"
#include "esp_log.h"
#include "temp.h"
#include "common.h"


TaskHandle_t pollInterruptTaskHandle = NULL;

static const char *TAG = "exio";



// PCA I2C 扩展 IO 控制对象
PCAGPIO pca;

// 系统电源通断检测标志（在中断中置位，在任务中处理）
volatile bool sys_pwr_onoff_flag = false;


static bool sys_power_on = false;  // 外部电源是否已接通


// PCA 中断 ISR，仅做通知，不处理业务逻辑
void IRAM_ATTR pcaIntISR()
{
    pca.isrNotify();
}

// 系统电源状态变化中断
void IRAM_ATTR pwrISR()
{
    sys_pwr_onoff_flag = true;
}

/**
 * @brief PCA 按键回调：水瓶检测。
 * @note  KWQ3 暂无任务序列，此处只打印状态；后续接入流程时再恢复暂停逻辑。
 */
void waterBottleStateState()
{
    ESP_LOGI(TAG, "Water bottle state changed: %d", pca.digitalRead(WATER_PIN));
}

/**
 * @brief PCA 按键回调：开门检测。
 * @note  KWQ3 暂无任务序列，此处只打印状态；后续接入流程时再恢复暂停逻辑。
 */
void doorStateChange()
{
    ESP_LOGI(TAG, "Door state changed: %d", pca.digitalRead(DOOR_PIN));
}

/**
 * @brief PCA 按键回调：样品盘检测。
 * @note  KWQ3 暂无任务序列，此处只打印状态；后续接入流程时再恢复暂停逻辑。
 */
void sampleStateChange()
{
    ESP_LOGI(TAG, "Sample tray state changed: %d", pca.digitalRead(SAMPLE_PIN));
}

/**
 * @brief 外部结构上电
 * 
 * @param on true 通电
 */
void sysPoweronoFF(bool on){
  if(on) {
    pca.digitalWrite(PWR_PIN, 1); 
  } 
  else{
    pca.digitalWrite(PWR_PIN, 0); 
  }
}


/**
 * @brief 电磁阀开关
 * 
 * @param on 
 */
void setFa(bool on){
  if(on)  pca.digitalWrite(FA_PIN, 1); 
  else  pca.digitalWrite(FA_PIN, 0); 
}


/**
 * @brief 检查样品盘有没有到位
 * 
 * @return true 到位
 * @return false 
 */
bool checkSampleState(){
  uint8_t v;
 v = pca.digitalRead(SAMPLE_PIN);
  return  !v;
}

/**
 * @brief 关门状态
 * 
 * @return true 到位
 * @return false 
 */
bool checkDoorState(){
  uint8_t v;
  v = pca.digitalRead(DOOR_PIN);
  return  !v;
}

/**
 * @brief 外部电源接通状态读取
 * 
 * @return true 通电
 * @return false 未通电
 */
bool checkSysPwr(){
  sys_power_on =  !digitalRead(1);
  return sys_power_on;
}

/**
 * @brief 检测瓶子状态
 * 
 * @return true 到位
 * @return false 
 */
bool checkWaterBottleState(){
  uint8_t v;
  //safePcaRead(2, &v);
  v = pca.digitalRead(WATER_PIN);
  return  !v;
}

uint32_t last_temp_read_time  = 0; // 上一次读取时间
uint8_t temp_ch = 0;  // 驱动器温度检测通道


float getMaxDriverTemp(void)
{
    float maxT = driverTemp[0];

    for (int i = 1; i < TEMP_CH_NUM; i++)
    {
        if (driverTemp[i] > maxT)
            maxT = driverTemp[i];
    }
    return maxT;
}

/**
 * @brief 中断事件查询Task
 * @param param 
 */
void pollInterruptTask(void *param)
{
  last_temp_read_time = millis();

    for (;;)
    {
        // 处理系统电源状态变化
        if (sys_pwr_onoff_flag)
        {
            sys_pwr_onoff_flag = false;
            uint8_t v = digitalRead(1);
            if (v){  // 检测到外部结构电源断开。
              cmd_power_off = true;       // 标记软件断电，EMM 运动命令将被拒绝
              sys_power_on = false;
              /* 结构断电前广播停止所有 EMM 电机，避免电机在断电瞬间仍在运动。 */
              if (Rs485Bus_Take(500)) {
                gEmm.stopBroadcast(false);
                Rs485Bus_Give();
              }
              sysPoweronoFF(0);    // 关电。
              ESP_LOGI(TAG,"Sys pwr OFF");
            } 
            else {
             sys_power_on = true;
             ESP_LOGI(TAG,"Sys pwr ON"); 
            } 
        }

         if(millis() - last_temp_read_time>2000){   // 每2秒读取温度
           checkSysPwr();
           last_temp_read_time = millis();
            if(temp_ch>=TEMP_CH_NUM) temp_ch = 0;
            driverTemp[temp_ch] = get_adc_temperatrue(temp_ch+1);
            if( driverTemp[temp_ch] > max_temp) {  // 温度过高
              ESP_LOGI(TAG, "driver IC[%d]Temp error", temp_ch);
              rygLedOn(RYG_COLOR_RED);
            }
            float maxT = getMaxDriverTemp();
            setFanDependDriverTemp(maxT);
            temp_ch++;
         }
         
        // 轮询并分发 PCA 中断事件
        pca.pollInterrupt();
        vTaskDelay(10);
    }
}

 /**
  * @brief PCA IO 初始化
  */
void exIOInit()
{

    // 初始化 I2C（自定义 SDA / SCL）
    Wire.begin(35, 36);
    if(!pca.begin(Wire, 0x20))
    ESP_LOGE(TAG,"XXX PCA Begin Failed");


    // 电磁阀输出 IO
    pca.pinMode(FA_PIN, OUTPUT);

    // 系统电源控制 IO
    pca.pinMode(PWR_PIN, OUTPUT);

    pca.pinMode(CD4015_A, OUTPUT);
    pca.pinMode(CD4015_B, OUTPUT);
    pca.pinMode(CD4015_C, OUTPUT);

    // 限位检测输入 IO
    pca.pinMode(WATER_PIN, INPUT);
    pca.pinMode(DOOR_PIN, INPUT);
    pca.pinMode(SAMPLE_PIN, INPUT);

    // 三色灯 IO
    pca.pinMode(RYG_R_PIN, OUTPUT);  // Green
    pca.pinMode(RYG_Y_PIN, OUTPUT);  // Yellow
    pca.pinMode(RYG_G_PIN, OUTPUT);  // Red

    // 绑定按键中断回调
    pca.attachInterrupt(WATER_PIN, waterBottleStateState, CHANGE);    // 水瓶检测
    pca.attachInterrupt(DOOR_PIN, doorStateChange, CHANGE);  // 开门检测
    pca.attachInterrupt(SAMPLE_PIN, sampleStateChange, CHANGE);  // 样品检测  


    delay(100);
    attachInterrupt(digitalPinToInterrupt(46), pcaIntISR, FALLING); // 外部扩展IO的中断

    // 后级电源检测输入
    pinMode(1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(1), pwrISR, CHANGE);

    // 创建 PCA 中断处理任务
    BaseType_t res = xTaskCreate(pollInterruptTask, "pollInterruptTask", 4096, NULL, 1, &pollInterruptTaskHandle);
    if(pdTRUE != res){
        ESP_LOGE(TAG, "pollInterruptTask creat failed");
    }
}

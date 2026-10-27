#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#endif

#include "Rs485Devices.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/** RS485 收发器连接到 ESP32-S3 的 GPIO；修改硬件时只需调整这两个宏。 */
#define RS485_RX_PIN        20
#define RS485_TX_PIN        19
/** X42S/EMM 自由协议默认使用 115200-8-N-1；该串口同时服务三类设备。 */
#define RS485_BAUD          115200

/**
 * EMM 电机地址起始值。
 * GFL-Z 激光传感器已经使用 0x01，因此 EMM 默认使用 0x02，避免两个设备
 * 在同一条 RS485 总线上地址冲突。其它 EMM 电机可继续使用 0x03~0xFF。
 */
#define EMM_DEFAULT_ADDR    0x02

#define SV04M_ADDR          0x00
#define SV04M_CHANNEL_NUM   10      // 按实际阀头修改：6/8/10/12/16/24/28

static const char *TAG = "RS485_DEV";
static SemaphoreHandle_t s_rs485Mutex = nullptr;
static bool s_rs485Inited = false;

/** 接近开关监控任务句柄；GPIO 中断只负责唤醒该任务，不在中断中访问串口。 */
static TaskHandle_t s_emmEndstopMonitorTaskHandle = nullptr;

/**
 * @brief 接近开关监控使能标志（**2026/10/16 新增**，上电默认**使能**）。
 * @details 接近开关（GPIO6）自 2026/10/16 起同时承担**样品盘零点标定**的角色：
 *          整机回零的最后一步用它在速度模式下把样品盘转到零点。标定完成后必须
 *          **关闭**本监控 —— 否则托盘正常转位经过零点时会被它触发停机。
 *          重新使能的三处：① `cmdPower()` 的 `SET POWER ON` 分支（**2026/10/19 新增**，
 *          保证"上电 → 回零 → 标定完关闭"之后下一次上电仍能重新布防）；
 *          ② `sysPowerOffCleanup()`（断电收尾）；③ 样品盘零点标定步骤开始处
 *          （整机回零第 17 步与 `SET EMM TRAY HOME` 共用，双保险，见 `EmmSystemHome.cpp`）。
 * @note 用 `volatile`：它同时被监控任务、整机回零任务和命令分发上下文读写。
 */
static volatile bool s_emmEndstopMonitorEnabled = true;

/** 共享 RS485 硬件串口实例；所有设备驱动都绑定到同一个 UART。 */
HardwareSerial Rs485Uart(1);
/** SV-04M 阀头驱动实例。 */
SV04M gSv04m(Rs485Uart);
/** GFL-Z 激光测距驱动实例。 */
GFLZLaser gLaser(Rs485Uart);
/** EMM V5 闭环步进电机驱动实例；协议层不重复初始化 Rs485Uart。 */
EmmV5 gEmm(Rs485Uart);

/**
 * @brief Y_ENDSTOP2_PIN 下降沿中断处理函数。
 * @details 中断中不能执行 RS485 通信或串口打印等耗时操作，因此这里只发送
 *          FreeRTOS 任务通知，把实际的停止动作交给高优先级监控任务完成。
 */
static void IRAM_ATTR emmEndstopFallingIsr(void)
{
    if (s_emmEndstopMonitorTaskHandle == nullptr) {
        return;
    }

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_emmEndstopMonitorTaskHandle,
                           &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief 接近开关触发后的 EMM 电机停止任务。
 *
 * 工作流程：
 * 1. 平时阻塞等待 GPIO 下降沿通知，不占用 CPU；
 * 2. 确认 Y_ENDSTOP2_PIN 当前仍为低电平，过滤已经恢复的瞬时干扰；
 * 3. 获取共享 RS485 总线并向 MONITOR_EMM 地址发送立即停止命令；
 * 4. 等待引脚恢复高电平后重新布防，避免低电平持续期间重复发送停止命令。
 */
static void emmEndstopMonitorTask(void *argument)
{
    (void)argument;

    for (;;) {
        /* 阻塞等待下降沿；pdTRUE 会在取出通知时清除当前累计值。 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /*
         * 监控已被关闭（整机回零用接近开关标定完样品盘零点后会关掉它）：
         * 直接丢弃本次通知，不发停止命令。
         * 关闭期间不消费"引脚是否仍为低电平"的判断，因此托盘停在零点压住开关
         * 也不会误发停止帧。
         */
        if (!s_emmEndstopMonitorEnabled) {
            continue;
        }

        /* 若脉冲很短、任务运行时引脚已经恢复高电平，则无需停止电机。 */
        if (digitalRead(Y_ENDSTOP2_PIN) != LOW) {
            continue;
        }

        Serial.printf("[EMM监控] GPIO%d为低电平，正在停止地址%d的电机\n",
                      Y_ENDSTOP2_PIN, MONITOR_EMM);

        /*
         * RS485 还服务于阀头和激光传感器，必须先取得互斥锁。若总线正忙，
         * 只要接近开关仍保持低电平就持续重试，确保停止请求不会被漏掉。
         */
        bool stopCommandSent = false;
        while (digitalRead(Y_ENDSTOP2_PIN) == LOW && !stopCommandSent) {
            if (Rs485Bus_Take(20)) {
                /*
                 * synchronized=false 表示立即执行；100 ms 是等待驱动器 ACK 的
                 * 最长时间，停止帧在进入该函数时已立即写入 UART。
                 */
                EmmV5::Status status = gEmm.stop(MONITOR_EMM, false, 100);
                Rs485Bus_Give();
                stopCommandSent = true;

                /* 用户要求的触发提示：无论 ACK 是否返回，停止帧都已经发出。 */
                Serial.printf("接近开关触发，停止地址为%d的电机\n", MONITOR_EMM);

                if (status == EmmV5::STATUS_OK) {
                    Serial.printf("[EMM监控] 地址%d的电机已确认停止命令\n",
                                  MONITOR_EMM);
                } else {
                    Serial.printf("[EMM监控] 地址%d的停止命令未收到有效确认，状态=%s\n",
                                  MONITOR_EMM, EmmV5::statusToString(status));
                }
            } else {
                /* 总线繁忙时短暂让出 CPU，随后继续抢占总线发送安全停止命令。 */
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        /*
         * 一次低电平只触发一次停止。只有接近开关恢复高电平，任务才重新布防，
         * 从而避免触发保持期间不断发送停止帧和重复打印串口信息。
         * 条件里带上使能标志：监控被关闭时立刻退出本循环 ——
         * 否则"托盘停在零点、一直压住开关"会让任务永远卡在这里，无法响应后续的
         * 使能 / 关闭切换。
         */
        while (digitalRead(Y_ENDSTOP2_PIN) == LOW && s_emmEndstopMonitorEnabled) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (!s_emmEndstopMonitorEnabled) {
            Serial.printf("[EMM监控] 监控已关闭，GPIO%d 不再触发停止\n",
                          Y_ENDSTOP2_PIN);
        } else {
            Serial.printf("[EMM监控] GPIO%d已恢复高电平，监控重新布防\n",
                          Y_ENDSTOP2_PIN);
        }
    }
}

/**
 * @brief 初始化共享 RS485 总线及其设备驱动。
 * @details 首次调用时配置 SV-04M 所需的 UART 参数，随后为激光测距和 EMM
 *          设置默认地址。由于三类设备共用收发器，初始化完成后必须通过
 *          Rs485Bus_Take()/Rs485Bus_Give() 串行化一次完整事务。
 */
void Rs485Devices_Init(void)
{
    if (s_rs485Mutex == nullptr) {
        s_rs485Mutex = xSemaphoreCreateMutex();
    }

    if (s_rs485Inited) {
        return;
    }

    ESP_LOGI(TAG, "init RS485 bus, baud=%lu, rx=%d, tx=%d",
             (unsigned long)RS485_BAUD,
             RS485_RX_PIN,
             RS485_TX_PIN);

    /** SV-04M 的 begin() 负责实际打开共享 UART；其它驱动只绑定到该 UART。 */
    gSv04m.setDebugEnabled(false);
    gSv04m.begin(RS485_BAUD, RS485_RX_PIN, RS485_TX_PIN, SV04M_ADDR, SV04M_CHANNEL_NUM);

    /** 激光测距与 SV-04M 共用串口，这里只设置默认地址，不重复 begin UART。 */
    gLaser.setDebugEnabled(false);
    gLaser.begin(0x01);

    /** EMM 默认地址为 2；地址 1 已分配给 GFL-Z 激光传感器。 */
    /*
     * 【帧级调试】2026/10/10 为排查"任务序列 1/9 同步步：广播触发后两台电机不动、
     * 目标从未设置"临时打开，2026/10/21 按用户要求**关闭**。
     *
     * ⚠️ 别随手再打开：`logFrame()` 走 `ESP_LOGI` → `Serial`（UART0，115200），
     *    而 **CH9120 就挂在同一根 UART0 上做 TCP 透传**（见 ch9120_net.cpp
     *    `Ch9120Net_RestoreWorkMode()` 里的 `Serial.begin(CH9120_NET_UART_BAUD)`）。
     *    打开后每帧多两行（TX/RX，单行最长 192 字符）十六进制日志，会**原样进入
     *    上位机收到的 TCP 数据流**：既挤占带宽，也会打乱上位机"按行解析应答"的节奏。
     *    真需要抓帧时，只在一个短窗口内打开，抓完立刻改回 `false`。
     *    历史见《修改日志.md》§7.13。
     */
    gEmm.setDebugEnabled(false);
    gEmm.begin(EMM_DEFAULT_ADDR);

    s_rs485Inited = true;
}

/**
 * @brief 使能 / 关闭接近开关监控（**2026/10/16 新增**，上电默认**使能**）。
 * @param enabled true=使能（接近开关触发会停止 MONITOR_EMM 的电机）；
 *                false=关闭（不再发送停止命令，但 GPIO 中断仍挂着）。
 * @note 关闭后，`emmEndstopMonitorTask()` 收到下降沿通知会立即丢弃，不消费
 *       "引脚是否仍为低电平"的判断；若此刻正卡在"等待引脚恢复高电平"的循环里，
 *       本函数也会让它立刻退出（循环条件带上了该标志）。
 * @note 关闭时**不会**主动补发停止命令；重新使能时若引脚正处于低电平，
 *       也**不会**补发停止（下一次下降沿才生效）—— 这两点由调用方保证安全：
 *       重新使能发生在 `SET POWER ON`（2026/10/19 新增）、断电收尾
 *       （`sysPowerOffCleanup()`）与样品盘零点标定步骤开始处，这些时刻托盘
 *       都不处于"正在压住开关并被要求运动"的状态。
 */
void EmmEndstopMonitor_SetEnabled(bool enabled)
{
    if (s_emmEndstopMonitorEnabled == enabled) {
        return;
    }

    s_emmEndstopMonitorEnabled = enabled;

    Serial.printf("[EMM监控] 接近开关监控已%s：GPIO%d 触发%s停止地址%d的电机\n",
                  enabled ? "使能" : "关闭",
                  Y_ENDSTOP2_PIN,
                  enabled ? "会" : "不再",
                  MONITOR_EMM);
}

/**
 * @brief 查询接近开关监控当前是否使能（true=触发会停 MONITOR_EMM 的电机）。
 * @note 仅供上层判断是否需要显式恢复，不做任何副作用。
 */
bool EmmEndstopMonitor_IsEnabled(void)
{
    return s_emmEndstopMonitorEnabled;
}

/**
 * @brief 创建 Y_ENDSTOP2_PIN（GPIO6）的接近开关监控任务。
 * @details 必须在 Rs485Devices_Init() 之后调用，确保停止任务执行时 UART、
 *          EMM 驱动对象和 RS485 互斥锁均已完成初始化。
 */
bool EmmEndstopMonitor_Init(void)
{
    if (s_emmEndstopMonitorTaskHandle != nullptr) {
        Serial.println("[EMM监控] 监控任务已经启动");
        return true;
    }

    if (!s_rs485Inited) {
        Serial.println("[EMM监控] 启动失败：RS485设备尚未初始化");
        return false;
    }

    /* 配置为输入上拉，保证独立调用时也能正确读取接近开关电平。 */
    pinMode(Y_ENDSTOP2_PIN, INPUT_PULLUP);

    /*
     * 优先级 11 高于普通运动任务，确保接近开关触发后尽快执行停止逻辑；
     * 固定到 Core 1，与现有运动控制任务保持一致，减少跨核唤醒延迟。
     */
    BaseType_t result = xTaskCreatePinnedToCore(
        emmEndstopMonitorTask,
        "EmmEndstopMonitor",
        3072,
        nullptr,
        11,
        &s_emmEndstopMonitorTaskHandle,
        1);

    if (result != pdPASS) {
        s_emmEndstopMonitorTaskHandle = nullptr;
        Serial.println("[EMM监控] 启动失败：无法创建监控任务");
        return false;
    }

    attachInterrupt(digitalPinToInterrupt(Y_ENDSTOP2_PIN),
                    emmEndstopFallingIsr, FALLING);

    Serial.printf("[EMM监控] 已启动：监控GPIO%d，低电平时停止地址%d的电机\n",
                  Y_ENDSTOP2_PIN, MONITOR_EMM);

    /* 上电时引脚可能已经处于低电平，没有下降沿；主动通知一次避免漏停。 */
    if (digitalRead(Y_ENDSTOP2_PIN) == LOW) {
        xTaskNotifyGive(s_emmEndstopMonitorTaskHandle);
    }

    return true;
}

/**
 * @brief 获取 RS485 总线互斥锁。
 * @param timeoutMs 最长等待时间；超时返回 false。
 * @return 成功取得锁返回 true。
 * @note 调用方必须在事务结束后调用 Rs485Bus_Give()，否则其它设备会一直等待。
 */
bool Rs485Bus_Take(uint32_t timeoutMs)
{
    if (s_rs485Mutex == nullptr) {
        return true;
    }
    return xSemaphoreTake(s_rs485Mutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

/** @brief 释放 RS485 总线互斥锁。 */
void Rs485Bus_Give(void)
{
    if (s_rs485Mutex != nullptr) {
        xSemaphoreGive(s_rs485Mutex);
    }
}

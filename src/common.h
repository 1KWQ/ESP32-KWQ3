#ifndef COMMON_H
#define COMMON_H

/**
 * @file common.h
 * @brief 公共头文件（ESP32-KWQ3）。
 * @details KWQ3 只保留 EMM 仪器所需的外设与全局状态：三色灯、扩展 IO、
 *          温度/风扇、RS485 总线、命令分发。已移除 XYZ 步进、ZS 注射器、
 *          序列流程、坐标点位与掉电保存相关的头文件与全局量。
 */

#include <Arduino.h>
#include <string.h>
#include "exio.h"
#include "ryg_led.h"
#include "temp.h"
#include "fan.h"
#include "crc16.h"
#include "CommandDispatcher.h"
#include "EmmSystemHome.h"
#include "SysInit.h"
#include "EmmTaskQueue.h"
#include "EmmTaskSeq.h"
#include "EmmSeqPoints.h"
#include "Rs485Devices.h"
#include "esp_log.h"

#define PING_TIMEOUT 1800    // ping超时 单位秒（30 分钟；2026/10/10 由 20000 s 下调）
#define CMD_TIMEOUT  30  // 休眠指令超时 单位分钟

/** @brief 5 路驱动芯片温度（℃，由 pollInterruptTask 周期刷新）。 */
extern volatile float driverTemp[5];
/** @brief 驱动芯片高温报警阈值（℃，SET TEMP 设置）。 */
extern volatile float max_temp;
/** @brief 三色灯 PWM 电压（V，SET RYG 设置）。 */
extern volatile uint8_t ryg_voltage;

/** @brief PC 心跳是否在线（PING 置位，超时清零）。 */
extern volatile bool ping_on_line;
/** @brief 最近一次收到 PING 的时刻（ms）。 */
extern unsigned long lastPingTime;
/** @brief 最近一次收到任意指令的时刻（ms）。 */
extern unsigned long lasCmdTime;
/** @brief 是否已经建立指令连接（收到过指令）。 */
extern volatile bool cmd_on_line;
/** @brief 是否处于软件断电状态（true 时 EMM 运动命令返回 NO_STEP_PWR）。 */
extern bool cmd_power_off;

/**
 * @brief 整机是否已经完成回零。
 * @details 由整机回零（L2，EmmSystemHome）的 17 步序列**全部成功**后置位；
 *          序列失败或中止时清零。供 `GET STATUS` 的 `HOMING` 字段与后续
 *          "开始任务"的前置检查（回零检测）使用。
 * @note 样品盘回零按配置"失败忽略"，不影响本标志。
 */
extern volatile bool sys_is_homed;

/* ==========================================================================
 * 日志分级（**2026/10/21 新增**）
 * --------------------------------------------------------------------------
 * ## 为什么需要
 * 网口（CH9120）与串口（USB-UART 桥）**共用同一根 UART0**，日志天然同源。
 * 一条 `SEQ START` 跑 3 个任务，原来要打 80 多行"流水账"（每个动作两行
 * DIR/RPM/PULSE + 到位误差），既占带宽，也会在应答行之间插进大量噪声。
 * 上位机真正需要的只有：阶段开始、阶段完成、失败原因。
 *
 * ## 两档
 * | 档 | 宏 | 内容 | 默认 |
 * | --- | --- | --- | --- |
 * | 关键 | `LOGK` | 命令应答、阶段事件、失败原因 | **始终输出** |
 * | 详细 | `LOGD` | 每步参数、到位误差、驱动器标志、点位快照、总线繁忙… | **关闭** |
 *
 * ## 硬规则：失败路径永远详细
 * 门禁只作用于"成功路径的流水账"。任何失败 / 异常 / 超时分支里的诊断
 * （驱动器目标、当前位置、误差、MOTOR_FLAGS 等）必须用 `LOGK`，
 * **不受开关影响** —— 否则出问题时手里没有任何证据。
 *
 * ## 用法
 * - 平时：什么都不用发，`LOGD` 自动不输出；
 * - 现场排查：`SET LOG DETAIL=1` 打开，`SET LOG DETAIL=0` 关闭；
 * - 查询：`GET LOG`。
 *
 * ## 实现说明
 * 用宏而不是函数：`LOGD` 关掉时**连函数调用都不产生**（不占 CPU、不占栈），
 * 且格式化仍由 `Serial.printf` 自己做（它内部会按需 malloc，不会像固定
 * 缓冲那样截断长行）。
 * ========================================================================== */

/** @brief 详细日志是否打开（`SET LOG DETAIL` 控制）。 */
extern volatile bool log_detail_on;

/** @brief 关键日志：始终输出。 */
#define LOGK(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
/** @brief 详细日志：仅 `log_detail_on` 为真时输出。 */
#define LOGD(fmt, ...)                     \
    do                                     \
    {                                      \
        if (log_detail_on)                 \
        {                                  \
            Serial.printf(fmt, ##__VA_ARGS__); \
        }                                  \
    } while (0)

#endif

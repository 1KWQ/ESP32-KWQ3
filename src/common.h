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
#include "Rs485Devices.h"
#include "esp_log.h"

#define PING_TIMEOUT 2   // ping超时 单位秒
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

#endif

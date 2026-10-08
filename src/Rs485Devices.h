#ifndef RS485_DEVICES_H
#define RS485_DEVICES_H

#include <Arduino.h>
#include "SV04M.h"
#include "GFLZLaser.h"
#include "EmmV5.h"

/**
 * @brief Y 轴远限位接近开关引脚（原定义在 stepper.h，KWQ3 移除 XYZ 步进后
 *        改由本模块持有，供 EMM 接近开关急停监控使用）。
 */
#define Y_ENDSTOP2_PIN  6

/**
 * @brief 接近开关触发后需要立即停止的 EMM 电机地址。
 * @note 修改被保护的电机地址时，只需要调整此宏，不需要改动监控任务逻辑。
 */
#define MONITOR_EMM  4

/*
 * EMM 机械轴地址：回零命令不再从上位机接收地址参数，统一使用这里的宏。
 * 修改电机接线/地址时只需调整这三项，系统回零、任务流程和扫描流程会
 * 自动使用同一组地址。
 */
#define EMM_ROTARY_MOTOR       2   // 丝杆旋转电机：负责B/C水平位置
#define EMM_LIFT_MOTOR         3   // 丝杆升降电机：负责高度及旋转补偿
#define EMM_SAMPLE_TRAY_MOTOR  4   // 样品盘电机：负责样品槽位

/** 升降电机碰撞回零完成后，从上限位向下退回的脉冲数。 */
#define EMM_LIFT_HOME_BACKOFF_PULSES  1800UL

/**
 * @file Rs485Devices.h
 * @brief 共享 RS485 总线设备的统一入口。
 * @details 一条 HardwareSerial 同时挂接 SV-04M、GFL-Z 和 EMM V5；调用方应
 *          使用 Rs485Bus_Take()/Rs485Bus_Give() 保证一次只有一个设备应答。
 */
extern HardwareSerial Rs485Uart;
/** @brief 共享总线上的 SV-04M 阀头驱动。 */
extern SV04M gSv04m;
/** @brief 共享总线上的 GFL-Z 激光测距驱动。 */
extern GFLZLaser gLaser;
/** @brief 共享总线上的 EMM V5 闭环步进电机驱动。 */
extern EmmV5 gEmm;

/** @brief 初始化 UART、RS485 互斥锁及所有已挂接设备。 */
void Rs485Devices_Init(void);

/**
 * @brief 启动 Y_ENDSTOP2_PIN 接近开关的 EMM 电机安全监控。
 * @details GPIO 从高电平变为低电平时唤醒高优先级任务，任务通过 RS485 向
 *          MONITOR_EMM 地址发送立即停止命令。引脚恢复高电平后自动重新布防。
 * @return true 监控任务创建成功；false 任务创建失败。
 */
bool EmmEndstopMonitor_Init(void);

/** @brief 在指定超时时间内获取共享 RS485 总线。 */
bool Rs485Bus_Take(uint32_t timeoutMs);
/** @brief 释放共享 RS485 总线。 */
void Rs485Bus_Give(void);

#endif

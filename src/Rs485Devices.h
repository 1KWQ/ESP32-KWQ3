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

/**
 * @brief 使能 / 关闭接近开关监控（**2026/10/16 新增**，上电默认**使能**）。
 * @details 接近开关（GPIO6）自 2026/10/16 起同时承担**样品盘零点标定**的角色：
 *          整机回零的最后一步用它在速度模式下把样品盘转到零点。标定完成后必须
 *          **关闭**本监控 —— 否则托盘正常转位经过零点时会被它触发停机。
 *          重新使能的三处：① `cmdPower()` 的 `SET POWER ON` 分支（**2026/10/19 新增**，
 *          保证"上电 → 回零 → 标定完关闭"之后下一次上电仍能重新布防）；
 *          ② `sysPowerOffCleanup()`（断电收尾）；③ 样品盘零点标定步骤开始处
 *          （整机回零第 17 步与 `SET EMM TRAY HOME` 共用，双保险，见 `EmmSystemHome.cpp`）。
 * @param enabled true 使能（触发即向 MONITOR_EMM 发停止）；false 关闭（只丢弃通知）。
 * @note 关闭后 GPIO 下降沿中断仍然挂着，只是监控任务不再发停止命令；重新使能时
 *       若引脚正处于低电平，**不会补发**停止（下一次下降沿才生效），避免在托盘
 *       停在零点压住开关时反复向已断电的驱动器发帧。
 */
void EmmEndstopMonitor_SetEnabled(bool enabled);

/** @brief 查询接近开关监控当前是否使能（true=触发会停 MONITOR_EMM）。 */
bool EmmEndstopMonitor_IsEnabled(void);

/** @brief 在指定超时时间内获取共享 RS485 总线。 */
bool Rs485Bus_Take(uint32_t timeoutMs);
/** @brief 释放共享 RS485 总线。 */
void Rs485Bus_Give(void);

#endif

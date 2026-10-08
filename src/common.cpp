/**
 * @file common.cpp
 * @author your name (you@domain.com)
 * @brief 公共文件，全局变量定义（ESP32-KWQ3）
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @details KWQ3 只保留 EMM 仪器所需的全局量：驱动温度、高温阈值、三色灯电压、
 *          心跳/指令超时相关标志与断电标志。XYZ 坐标、序列参数等已移除。
 */
#include "common.h"

// 5路驱动测温
volatile float driverTemp[5] = {-1.0, -1, -1, -1, -1};

// 温度设置（驱动芯片最大温度）
volatile float max_temp = 80;

// ryg 电压
volatile uint8_t ryg_voltage = 12; // 默认12V

// 定时ping 是否在线
volatile bool ping_on_line = false;

unsigned long lastPingTime = 0;

// 定时指令, 长时间未收到指令， 休眠
unsigned long lasCmdTime = 0;

volatile bool cmd_on_line = false; // 长时间没有指令，进入休眠

bool cmd_power_off = false;  // 指令或者休眠模式下控制了断电

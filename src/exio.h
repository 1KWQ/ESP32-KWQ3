#ifndef EXIO_H
#define EXIO_H

/**
 * @file exio.h
 * @brief 外部扩展 IO（PCA9555/XL9555）接口（ESP32-KWQ3）。
 * @details KWQ3 保留三色灯、电磁阀、门/样品盘/水瓶检测、系统电源控制与
 *          CH9120 控制脚定义；电阻检测（RES CHK）相关接口已移除。
 */

#include <Arduino.h>

#include "pca_gpio.h"

// RYG LED  IO
#define RYG_R_PIN 10
#define RYG_Y_PIN 9
#define RYG_G_PIN 8

// 电磁阀
#define FA_PIN 4

// 三个限位检测
#define WATER_PIN  5 // 水瓶
#define DOOR_PIN   6 // 关门
#define SAMPLE_PIN 7 // 样品

// 电源控制
#define PWR_PIN 3

// 多路复用器控制
#define CD4015_A 2
#define CD4015_B 1
#define CD4015_C 0

// CH9120 网口透传模块控制脚（接 PCA9555/XL9555 P12~P17）。
// PCA GPIO 编号范围是 0~15，不能直接使用芯片丝印 P14~P17。
#define CH9120_RSTI_PIN  12  // 外部复位输入，低电平有效
#define CH9120_CFGO_PIN  13  // 配置模式，低电平进入配置
#define CH9120_RESET_PIN 14  // 恢复出厂设置，正常工作时保持高电平
#define CH9120_TCPCS_PIN 15  // TCP Client 状态，低电平表示已连接

// 5路温控
#define TEMP_CH_NUM  5

void sysPoweronoFF(bool on);

void setFa(bool on);

bool checkSampleState();

bool checkDoorState();

bool checkSysPwr();

bool checkWaterBottleState();

void exIOInit();

extern PCAGPIO pca;
extern volatile bool sys_pwr_onoff_flag;
extern TaskHandle_t pollInterruptTaskHandle;

#endif

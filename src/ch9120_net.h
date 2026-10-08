#ifndef CH9120_NET_H
#define CH9120_NET_H

#include <Arduino.h>

/**
 * @file ch9120_net.h
 * @brief CH9120 网络串口透传控制接口。
 *
 * @details
 * 当前硬件把 CH9120 的串口侧连接到 ESP32 默认 Serial，因此网络透传收到的
 * 数据会直接进入现有 `serialPoll()` 命令解析流程。该模块只负责 CH9120 的
 * XL9555 控制脚初始化、EEPROM 参数写入、TCP 连接状态读取以及 Web 页面触发
 * 的手动重新配置。
 */

/**
 * @brief CH9120 正常透传模式下与 ESP32 默认 Serial 保持一致的波特率。
 */
#define CH9120_NET_UART_BAUD 115200U

/**
 * @brief 初始化 CH9120 网络透传功能。
 *
 * @details
 * 函数会先初始化 XL9555 上的 CH9120 控制脚，再按配置版本决定是否重新写入
 * CH9120 EEPROM。配置完成后会恢复 `Serial.begin(115200)`，因此后续 USB 串口
 * 和局域网透传来的命令都由现有 `serialPoll()` 统一处理。
 */
void Ch9120Net_Init(void);

/**
 * @brief 请求后台任务强制重新配置 CH9120。
 *
 * @return true 请求已接受，后台任务会重新写入当前代码中的 CH9120 网络参数。
 * @return false CH9120 正在配置中，或后台任务创建失败。
 *
 * @details
 * 该函数主要供 Web 日志页面按钮调用。函数本身只创建后台任务并立即返回，
 * 具体配置过程会继续通过 ESP_LOG 写入 Web 日志页面。
 */
bool Ch9120Net_RequestReconfigure(void);

/**
 * @brief 查询 CH9120 当前是否处于串口配置流程。
 *
 * @return true 正在拉低 CFGO 并通过默认 Serial 写入 CH9120 参数。
 * @return false 当前处于正常透传或空闲状态。
 *
 * @details
 * CH9120 配置流程与业务命令共用默认 Serial。主循环在该状态为 true 时应暂停
 * `serialPoll()`，避免把 CH9120 的配置应答字节误当成业务命令读取。
 */
bool Ch9120Net_IsConfiguring(void);

/**
 * @brief 周期性记录 CH9120 TCP 客户端连接状态。
 *
 * @details
 * TCPCS 为低电平表示 TCP Client 已连接服务器，高电平表示尚未连接。主循环
 * 周期调用该函数后，Web 页面会持续看到 `TCPCS=LOW/HIGH` 日志，方便排查
 * IP、端口、网线、服务器监听和防火墙问题。
 */
void Ch9120Net_Poll(void);

/**
 * @brief 查询 TCP 客户端连接状态。
 *
 * @return true CH9120 的 TCPCS 为低电平，表示 TCP 客户端已连接。
 * @return false CH9120 的 TCPCS 为高电平，表示 TCP 客户端未连接。
 */
bool Ch9120Net_IsTcpConnected(void);

#endif

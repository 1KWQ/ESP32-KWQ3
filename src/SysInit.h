#ifndef SYS_INIT_H
#define SYS_INIT_H

#include <Arduino.h>

/**
 * @file SysInit.h
 * @brief 系统初始化流程（ESP32-KWQ3）。
 *
 * @details 本模块把"结构上电后的一次性初始化"封装成一个常驻后台任务，由上位机
 *          `SET POWER ON` 触发（见 CommandDispatcher 的 cmdPower）。之所以用
 *          独立任务而不是在命令分发里同步跑，是因为流程开头需要等待后级电源与
 *          驱动器上电稳定（数百毫秒级），同步执行会阻塞串口命令轮询。
 *
 *          执行顺序：
 *
 *            0. **上电稳定等待**：`SET POWER ON` 合上结构电源继电器后，等待
 *               `SYS_INIT_POWER_SETTLE_MS` 毫秒，让 EMM 驱动器完成上电自检。
 *
 *            1. **前置检查**（4 项，全部通过才继续）：
 *               ① PC 心跳在线      —— `ping_on_line`；
 *               ② 门已关闭          —— `checkDoorState()`；
 *               ③ 样品盘已放置      —— `checkSampleState()`；
 *               ④ 控制板温度未超限  —— `getMaxDriverTemp() <= max_temp`。
 *               任一项失败 → **中止初始化** + 三色灯**黄色闪烁(500ms)** +
 *               串口打印失败原因。注意：前置检查失败**不会**自动断开结构电源，
 *               由上位机决定后续动作。
 *
 *            2. **六通阀**：KWQ2 中无对应实现，当前**跳过并视为通过**
 *               （串口打印"跳过"提示），待后续补充实际切阀动作。
 *
 *            3. **电机回零**：调用 `EmmSystemHome_Start()` 启动已落地的
 *               17 步整机回零序列。回零是**异步**执行的，本流程不等待其结束，
 *               回零进度与结果由 `EmmSystemHome` 任务自行以 `[回零]` 前缀输出。
 *
 *          按需求，初始化流程的完成/失败**仅通过串口日志体现**，不主动向上位机
 *          发送协议帧；`SET POWER ON` 的回复始终是 `OK @<seq> POWER ON`。
 *
 * @note 本模块只做"编排"：前置检查复用 exio 的现成接口，回零复用
 *       EmmSystemHome 的 L2 序列，自身不产生任何 RS485 总线事务。
 */

/** @brief 初始化流程的结果状态。 */
enum SysInitState {
    SYS_INIT_IDLE = 0,   ///< 尚未触发过初始化
    SYS_INIT_RUNNING,    ///< 初始化流程正在执行（含上电稳定等待与前置检查）
    SYS_INIT_PASSED,     ///< 前置检查通过且整机回零已启动（或本就在运行）
    SYS_INIT_FAILED      ///< 前置检查未通过，初始化已中止
};

/**
 * @brief 创建系统初始化常驻任务。
 * @details 必须在 `Rs485Devices_Init()` 与 `EmmSystemHome_Init()` 之后调用，
 *          确保回零子流程已就绪。任务创建后立即阻塞等待启动通知，不会主动
 *          占用 CPU。重复调用安全。
 * @return true 任务已启动或早已运行；false 任务创建失败。
 */
bool SysInit_Init(void);

/**
 * @brief 请求执行一次系统初始化流程（非阻塞）。
 * @details 只做"置位标志 + 唤醒后台任务"，不等待流程结束，因此命令分发任务
 *          可以立即回复上位机。若已有初始化流程在运行，本次请求会被拒绝。
 * @return true  请求已被接受并唤醒后台任务；
 * @return false 任务不可用，或初始化流程已在运行。
 */
bool SysInit_Run(void);

/**
 * @brief 查询初始化流程是否正在执行。
 * @return true 表示流程占用中（含尚未被后台任务拾取的启动请求）。
 */
bool SysInit_IsActive(void);

/**
 * @brief 请求中止正在进行的系统初始化流程（断电 / 指令超时休眠收尾用）。
 * @details 只置位中止请求，**不阻塞**。后台任务会在三个"动作边界"检查该请求，
 *          一旦发现就立刻打印日志并退出本次流程、把状态复位为空闲：
 *            ① 刚拾取到启动通知时；
 *            ② 上电稳定等待（`SYS_INIT_POWER_SETTLE_MS`）结束时；
 *            ③ 前置检查通过、即将启动整机回零之前。
 *
 *          为什么需要它：流程开头有 1.5s 的上电稳定等待。若这段时间内结构电源
 *          被切断，没有中止检查的话，流程醒来后仍会继续做前置检查并调用
 *          `EmmSystemHome_Start()`——相当于在已断电的机器上硬启回零。
 *
 * @note 当前没有流程在跑时调用是安全的：直接把结果状态复位为空闲并打印一行
 *       日志，不做其它动作。有流程在跑时**不会**立刻清 `s_sysInitBusy`，而是
 *       交给后台任务在检查点自行复位，避免"旧的还没退干净、新的已经放行"。
 */
void SysInit_Abort(void);

/**
 * @brief 查询最近一次初始化流程的结果状态。
 * @return 见 SysInitState；默认（从未触发）为 SYS_INIT_IDLE。
 */
SysInitState SysInit_GetState(void);

#endif

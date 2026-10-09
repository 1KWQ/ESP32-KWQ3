#ifndef EMM_SYSTEM_HOME_H
#define EMM_SYSTEM_HOME_H

#include <Arduino.h>

/**
 * @file EmmSystemHome.h
 * @brief 整机回零（L2 复合动作）——ESP32-KWQ3。
 *
 * @details 本模块把"上位机一条命令触发、驱动器多步动作"的回零流程封装成一个
 *          常驻后台任务。它位于 L2（复合动作层）：向下直接调用 EmmV5 协议接口
 *          （与 L1 单一动作命令走同一套 RS485 互斥与超时策略），向上由
 *          CommandDispatcher 暴露两条文本命令：
 *
 *            SET EMM SYSTEM HOME        启动整机回零，立即回 STARTED，异步执行；
 *            SET EMM SYSTEM HOME STOP   请求中止，当前步骤结束后停止。
 *
 *          回零顺序固定为：升降电机(A=3) 6 步 → 旋转电机(A=2) 5 步 →
 *          样品盘(A=4) 1 步，共 12 步。每一步都有 `[回零]` 前缀的串口打印。
 *          除样品盘外，任一步失败都会广播停止全部 EMM 电机并中止整个流程。
 *
 * @note 三个同名 MODE 的含义必须区分清楚：
 *       - `SET EMM MOVE ... MODE=`：位置模式 0=相对上一目标 / 1=绝对零点 /
 *         2=相对实时位置。本模块的定位步统一使用 MODE=1 绝对定位。
 *       - `SET EMM HOME ... MODE=`：回零模式 0=单圈就近 / 2=无限位碰撞。
 *       - `SET EMM HOME PARAM ... MODE=`：写入驱动器的回零模式参数。
 */

/**
 * @brief 创建整机回零常驻任务。
 * @details 必须在 Rs485Devices_Init() 之后调用，确保共享 RS485 总线、EMM 驱动
 *          对象与总线互斥锁均已就绪。任务创建后立即阻塞等待启动通知，不会
 *          主动占用 CPU 或 RS485 总线。重复调用安全。
 * @return true 任务已启动或早已运行；false 任务创建失败。
 */
bool EmmSystemHome_Init(void);

/**
 * @brief 请求启动一次整机回零。
 * @details 只做"置位标志 + 唤醒任务"，不等待流程结束，因此调用方可以立即回复
 *          上位机。若已有回零流程在运行（含尚未被任务拾取的启动请求），本次
 *          请求会被拒绝。
 * @return true 请求已被接受并唤醒后台任务；false 任务不可用或流程已在运行。
 */
bool EmmSystemHome_Start(void);

/**
 * @brief 请求中止正在进行的整机回零。
 * @details 置位中止标志；后台任务在下一个轮询点或步骤边界退出，随后广播停止
 *          全部 EMM 电机。当前没有流程运行时只打印提示，不做任何动作。
 */
void EmmSystemHome_RequestStop(void);

/**
 * @brief 查询整机回零流程是否正在运行。
 * @return true 表示流程占用中（含尚未被后台任务拾取的启动请求）。
 */
bool EmmSystemHome_IsActive(void);

/**
 * @brief 判断某条 EMM 命令在回零运行期间是否应被拒绝。
 * @details 回零期间独占 2/3/4 号电机，因此拦截所有会触发动作或修改电机状态的
 *          `SET EMM *` 命令；`GET EMM *` 查询命令保持可用，便于现场诊断。
 *          回零自身的启动/中止命令，以及只切换本地布防标志、不产生总线事务的
 *          `SET EMM LIFT MONITOR`，始终放行。
 * @param commandName 已由 CommandDispatcher 匹配出的完整命令名。
 * @return true 表示应当拒绝；false 表示放行。
 */
bool EmmSystemHome_BlocksCommand(const char *commandName);

/**
 * @brief 上位机命令 `SET EMM SYSTEM HOME`。
 * @details 无参数。整机电源关闭时返回 NO_STEP_PWR；流程已在运行时返回
 *          SEQ_RUNNING；接受后立即回复 `OK @<seq> EMM SYSTEM HOME STARTED`，
 *          实际回零结果通过串口日志输出。
 */
bool cmdSetEmmSystemHome(const char *args, int seq);

/**
 * @brief 上位机命令 `SET EMM SYSTEM HOME STOP`。
 * @details 无参数。只置位中止请求，立即回复
 *          `OK @<seq> EMM SYSTEM HOME STOP ACCEPTED`；实际停止由后台任务在
 *          当前步骤结束后完成。
 */
bool cmdSetEmmSystemHomeStop(const char *args, int seq);

#endif

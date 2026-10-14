#ifndef EMM_TASK_SEQ_H
#define EMM_TASK_SEQ_H

#include <Arduino.h>

#include "EmmTaskQueue.h"

/**
 * @file  EmmTaskSeq.h
 * @brief 任务序列执行引擎（L2 复合动作）——ESP32-KWQ3，**第三步：真实机械动作**。
 *
 * @details
 *   本模块在 `EmmTaskQueue`（只建队列）之上，补一层 **序列状态机 + 4 条控制命令**，
 *   把队列真正跑起来。参考基准是 **KWQ1 的 `process.cpp` 任务序列流程**
 *   （不使用 KWQ2 的 `SET EMM TASK *` 双流程写法）。
 *
 *   从 KWQ1 学到的、并已在本模块落地的 5 个机制：
 *
 *     1. **双层状态机**：`SeqState`（停止/运行/暂停）→ `TaskPhase`（位置阶段）→
 *        子步骤索引（`s_subStep`）。对应 KWQ1 的 `taskPhase` + `move_pos_stage`。
 *     2. **运动完成判据**：KWQ1 由步进任务上报 `NOTIFY_MOTION_DONE`；KWQ3 的电机是
 *        **驱动器内部跑完定长脉冲**，因此改为"轮询 `0x3A` 系统状态判到位"
 *        （堵转即失败、起步保护、moved 前置），回零判据用 `0x3B`（bit2/bit3）。
 *        **刻意不用 `EmmSystemHome` 的"等 0x9F 主动上报"**：`requireDone=true` 期间
 *        必须一直独占总线，暂停/中止就插不进去了；本模块要能在等待中途随时停机，
 *        所以统一走轮询路径（代价是状态刷新有 100 ms 粒度，对本序列足够）。
 *     3. **暂停**：KWQ1 是 `stopAllSteppersSmooth()`；KWQ3 是 `gEmm.stop(addr)`
 *        立即停机，等停稳后停在原地。
 *     4. **恢复**：KWQ1 是 `stepperX->moveTo(targetX)` —— **重新下发原绝对目标**，
 *        由驱动器自己算剩余脉冲。KWQ3 完全同构：本模块所有真实动作（含样品盘）
 *        都统一用 **`MODE=1` 绝对定位**下发，并记住本步的**绝对目标**，恢复时
 *        重发定位命令即可（同步步会把 2 台电机一起重发）。
 *     5. **子步骤返回值**：KWQ1 的 `SM_RUNNING/SM_DONE/SM_FAILED`；KWQ3 的阶段
 *        处理器返回 `SeqStepResult`（同义三态），暂停/恢复/中止在等待循环内部处理。
 *
 *   本阶段已接入的真实动作（每一步的命令都来自用户实机验证过的报文）：
 *
 *     TASK_PREAMBLE   `SET EMM ZERO A=3` → `SET EMM ORIGIN ZERO A=2/3 SAVE=0`
 *                     （`SEQ START` 通过后**只做一次**）
 *     TASK_MOVE_POS   Ⅱ 取样位：2+3 号**同步**转到取样位 → 3 号快降到取样高度1
 *                     → 受阻检测布防（占位）→ 3 号慢降到取样高度2
 *                     → 受阻检测撤防（占位）→ 3 号抬到安全高度 → 喷头识别（占位）
 *                     （2026/10/14 起**不再停留**：下降到取样点后直接抬起）
 *     TASK_TRAY_POS   4 号样品盘用 **`MODE=1` 绝对位置**定位到**本任务**的工位：
 *                     工位 N ↔ 绝对脉冲 `(N−1)×320`（01 号 = 0、02 号 = 320、…）。
 *                     **在取样（TASK_MOVE_POS）之前执行**（2026/10/10 方案 A：把
 *                     原来的"给下一条任务预置"改为"给本任务定位"，使每条任务
 *                     自包含、支持中途 `SEQ ADD`；枚举顺序与执行顺序不同）。
 *     TASK_DETECT_POS Ⅲ 检测位：2 号 → 检测位（2850）→ 3 号降至检测高度
 *                     → **停留 T 分钟** → 3 号抬回安全高度2
 *     TASK_DISCARD_POS Ⅳ 回收位：3 号降 1000 → 2 号 280 → 喷头识别（占位）→
 *                     3 号降 35000 → 受阻检测布防（占位）→ 2 号 200 →
 *                     受阻检测撤防（占位）→ 3 号 30000 → 3 号抬到安全高度 →
 *                     喷头识别（占位）
 *     TASK_RESET_POS  **4 号样品盘就近单圈回零（`MODE=0`，先执行）** → 2+3 号
 *                     **同步**单圈回零至初始位（整条队列跑完时**只做一次**；
 *                     2026/10/10 起 4 号也回零、且排在 2/3 号之前）。
 *                     **2/3 号回零失败也不中止**：广播停机后照常收尾、**清空队列**
 *                     （用户要求"不管 2、3 号一起单圈回零是否失败，也直接清空"）
 *
 *   `T` 的含义（2026/10/14 用户确认）：**3 号下降到检测位后、抬回安全高度 2 之前
 *   的停留时长（单位：分钟）**，计时用"累计法"，暂停期间不累计，因此 PAUSE 不会
 *   把剩余停留时间吃掉。
 *
 *   仍为占位的两处（用户要求"下一阶段实现"）：
 *     - **喷头识别**：只在串口打印 `【喷头识别：占位】`；
 *     - **受阻检测**：只在串口打印 `【受阻检测：占位】`（现有 `EmmLiftCurrentMonitor`
 *       保持不变，仍可由上位机手动 `SET EMM LIFT MONITOR ARM=1` 布防）。
 *
 *   命令集（4 条，与 KWQ1 同名同义）：
 *
 *     SEQ START    前置检查全部通过 → 建立零点（TASK_PREAMBLE）→ 从第 1 条开始执行；
 *     SEQ PAUSE    仅 `RUNNING` 时接受，**立即停机**并停在原地；
 *     SEQ RESUME   仅 `PAUSED` 时接受，**重发定位命令**从断点继续；
 *     SEQ STOP     中止并清空队列，**并立即广播停止全部 EMM 电机**（"命令一到就停"，
 *                  见 §14.15；空操作时也回 OK，与 KWQ1 一致）。
 *
 * @note 队列本身（条目、长度、游标、`SeqState`）仍然由 `EmmTaskQueue` 持有，
 *       本模块只通过 `EmmTaskQueue_*` 访问器读写，不重复维护一份数据。
 * @note 失败处理：任一步失败/超时 → **广播停止全部 EMM 电机 + 序列回 `STOPPED`**
 *       （保留队列便于现场排查），与 `EmmSystemHome` 的收尾方式一致。
 *       **例外（2026/10/10 用户要求）**：队列跑完后的复位阶段里，**2/3 号同步单圈
 *       回零失败不按失败中止** —— 广播停机后照常收尾、**清空队列**（"不管最后 2、3 号
 *       一起单圈回零是否失败，也是直接清空任务序列"）；但 2/3 号**写回零参数**失败
 *       仍按失败中止（保留队列）。
 */

/* ==========================================================================
 * 数据结构
 * ========================================================================== */

/**
 * @brief 单条任务的执行阶段（前 6 个对齐 KWQ1 的 `enum TaskPhase`）。
 * @details 前 6 个取值与 KWQ1 保持一致（且已被第二步实机验证过），**不要改动**；
 *          第三步的真实机械动作按"位置"细分，新增的 5 个值**追加在末尾**，
 *          这样既不影响已有取值，又能让 `TaskPhaseToStr` 的下标一一对应。
 *
 *          第三步的位置顺序（与用户给的机械流程 Ⅰ~Ⅳ 对应）：
 *          **注意：`TASK_TRAY_POS` 的枚举值排在 `TASK_MOVE_POS` 之后，但实际执行
 *          顺序在它之前**（先转盘、再取样，2026/10/10 方案 A）。
 *
 *            TASK_PREAMBLE   序列前导：`SET EMM ZERO A=3` + 2/3 号设单圈零点
 *                            （`SEQ START` 通过后做一次，与具体任务无关）
 *            TASK_IDLE       进入本条任务（读工位号、点绿灯）
 *            TASK_TRAY_POS   样品盘（4 号）用 MODE=1 绝对位置定位到**本任务**的
 *                            工位（`(N−1)×320`）—— **在取样之前执行**
 *            TASK_MOVE_POS   Ⅱ 取样位：2+3 号同步→取样位 → 3 号降/慢降 → 抬起
 *                            （2026/10/14 起不再停留）
 *            TASK_DETECT_POS Ⅲ 检测位：2 号→检测位 → 3 号降 → 停留 T 分钟 → 3 号抬回安全高度2
 *            TASK_DISCARD_POS Ⅳ 回收位/抛弃位
 *            TASK_RESET_POS  复位：**4 号就近单圈回零（先执行）→ 2+3 号同步单圈回零**至初始位
 *                            （**整条队列跑完时只做一次**，用户确认）；**2/3 号回零失败也不
 *                            中止**，广播停机后照常收尾、清空队列（参数写入失败才按失败中止）
 */
enum TaskPhase
{
    TASK_IDLE = 0,          ///< 任务空闲（准备进入本条任务）
    TASK_MOVE_POS,          ///< Ⅱ 取样位（KWQ1 原名"移动到 Axx"，这里复用）
    TASK_WAIT_RESISTOR_ACK, ///< 预留（KWQ1 电阻检测用，KWQ3 未使用）
    TASK_AUX_RUN,           ///< 预留（KWQ1 抽/推液与回位，KWQ3 未使用）
    TASK_PAUSED,            ///< 预留（KWQ3 用序列级 PAUSED 代替，未使用）
    TASK_END,               ///< 预留
    /* ---- 第三步新增（追加在末尾，不改动上面 6 个已实机验证的取值） ---- */
    TASK_PREAMBLE,          ///< 序列前导：建立 2/3 号单圈零点（SEQ START 后一次）
    TASK_TRAY_POS,          ///< 样品盘（4 号）定位到**本任务**的工位（MODE=1 绝对，取样之前）
    TASK_DETECT_POS,        ///< Ⅲ 检测位（3 号降到检测位后停留 T 分钟）
    TASK_DISCARD_POS,       ///< Ⅳ 回收位 / 抛弃位
    TASK_RESET_POS          ///< 复位：4 号就近单圈回零（先）→ 2+3 号单圈回零（整条队列跑完时一次；2/3 号回零失败也不中止、照常清空队列）
};

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 创建任务序列执行常驻任务。
 * @details 必须在 `Rs485Devices_Init()` 之后、`EmmTaskQueue_Init()` 之后调用。
 *          任务创建后进入 10ms 周期轮询，序列未启动时每个周期立即返回，不会
 *          主动占用 RS485 总线；序列启动后每走一步都按 `Rs485Bus_Take()/
 *          Rs485Bus_Give()` 与其他设备共享总线。重复调用安全。
 * @return true 任务已启动或早已运行；false 任务创建失败。
 */
bool EmmTaskSeq_Init(void);

/**
 * @brief 序列是否正在执行（`RUNNING`）或已暂停（`PAUSED`）。
 * @details 供命令层做互锁判断，例如序列运行期间拒绝其它 EMM 动作命令。
 */
bool EmmTaskSeq_IsActive(void);

/**
 * @brief 取当前任务的执行阶段（仅用于诊断/日志）。
 */
TaskPhase EmmTaskSeq_GetPhase(void);

/**
 * @brief 执行阶段枚举转字符串（仅用于日志）。
 */
const char *EmmTaskSeq_TaskPhaseToStr(TaskPhase p);

/**
 * @brief 判断某条 EMM 命令在序列运行期间是否应被拒绝。
 * @details 与 `EmmSystemHome_BlocksCommand()` 同一套做法：序列运行期间独占
 *          机械机构，因此拦截所有会触发动作或修改电机状态的 `SET EMM *`
 *          命令（含 `SET EMM SYSTEM HOME`，避免序列中途被回零打断）；
 *          `GET EMM *` 查询、`SET EMM SYSTEM HOME STOP` 与只切换本地布防
 *          标志的 `SET EMM LIFT MONITOR` 始终放行。
 * @param commandName 已由 CommandDispatcher 匹配出的完整命令名。
 * @return true 表示应当拒绝；false 表示放行。
 */
bool EmmTaskSeq_BlocksCommand(const char *commandName);

/**
 * @brief 立即终止序列并清空队列（强制停止路径）。
 * @details 供 `SET POWER OFF`、指令超时休眠等"必须马上停下来"的路径调用，
 *          与上位机命令 `SEQ STOP` 共用同一套收尾逻辑，**顺序如下**：
 *            ① 置 `STOPPED`（先改状态，避免状态机把"Index/Count 同时归零"
 *               误判成"全部任务完成"）；
 *            ② 清空队列（`EmmTaskQueue_Clear()`）；
 *            ③ 复位执行状态机（`emmTaskSeqResetPhase()`）；
 *            ④ 黄灯待机（`RYG_COLOR_YELLOW`）；
 *            ⑤ **立即广播停止全部 EMM 电机**（`gEmm.stopBroadcast(false)`，
 *               2026/10/10 用户要求"命令一到就停，不等当前动作走完"）。
 *          幂等：序列本来就没跑时只打印日志，第 ⑤ 步照样发（重复一帧无害）。
 *
 * @note 为什么 `SET POWER OFF` 必须调它：断电时若只停电机而不管序列，
 *       `SeqState` 会一直停在 `RUNNING`，重新上电后 `SysInit` 又会去启动整机
 *       回零，于是"任务序列"和"初始化/回零"两套复合动作并行跑（实机已复现）。
 * @note 为什么第 ⑤ 步放在函数里而不是只靠序列任务轮询：序列任务可能正卡在
 *       同步步的静默间隔 / `vTaskDelay(1000)` 里不轮询状态，此时只有"命令一到
 *       就广播停机"才能立刻停住电机；等待循环里另有一道"停本步电机并等停稳"
 *       的兜底（见 `emmSeqHandlePauseResume()` 的中止分支）。
 */
void EmmTaskSeq_Stop(void);

/**
 * @brief 上位机命令 `SEQ START`。
 * @details 前置检查顺序（任一失败立即回对应错误码，并打印失败原因）：
 *            ① 结构电源已合上      → 109 `NO_STEP_PWR`
 *            ② 序列未在运行        → 110 `SEQ_RUNNING`
 *            ③ 队列非空            → 103 `TASK_NULL`
 *            ④ 上位机心跳在线      → 115 `NO_HEARTBEAT`
 *            ⑤ 舱门已关闭          → 107 `NO_DOOR`
 *            ⑥ 样品盘已放置        → 105 `NO_SAMPLE`
 *            ⑦ 电机使能            → **注释占位，默认通过**（见 .cpp 内说明）
 *            ⑧ 控制板温度未超限    → 116 `TEMP_HIGH`
 *            ⑨ 整机已回零          → 104 `NO_HOME`
 *          全部通过后先进入 `TASK_PREAMBLE`（`SET EMM ZERO A=3` + 2/3 号设单圈
 *          零点，**只做一次**），再进入第 1 条任务；立即回复 `OK @n SEQ START`，
 *          真正的动作在后台任务里跑。
 */
bool cmdSeqStart(const char *args, int seq);

/**
 * @brief 上位机命令 `SEQ PAUSE`。
 * @details 仅当序列处于 `RUNNING` 时接受，回复 `OK @n SEQ PAUSE`；
 *          其它状态回 `ERR @n 110`（`SEQ_RUNNING`，语义为"序列状态不允许该操作"）。
 *          暂停语义（用户确认）：**立即停机** —— 后台任务在等待循环里发现
 *          `PAUSED` 后对正在运动的电机发停止命令，等停稳后停在原地；`RESUME`
 *          时**重发本步的定位命令**从断点继续（对应 KWQ1 的 `moveTo(target)`）。
 */
bool cmdSeqPause(const char *args, int seq);

/**
 * @brief 上位机命令 `SEQ RESUME`。
 * @details 仅当序列处于 `PAUSED` 时接受，回复 `OK @n SEQ RESUME`；
 *          其它状态回 `ERR @n 110`。接受前会复查舱门与样品盘（对齐 KWQ1 在
 *          恢复时复查限位的行为，防止暂停期间开门/取盘后被带着继续跑）。
 */
bool cmdSeqResume(const char *args, int seq);

/**
 * @brief 上位机命令 `SEQ STOP`。
 * @details 中止序列并清空队列，始终回复 `OK @n SEQ STOP`（与 KWQ1 一致）；
 *          当前没有运行中的序列时视为空操作，只打印日志。
 *          **2026/10/10 起**：收尾时**立即广播停止全部 EMM 电机**（`EmmTaskSeq_Stop()`
 *          内 `gEmm.stopBroadcast(false)`），做到"命令一到就停、不等当前动作走完"。
 */
bool cmdSeqStop(const char *args, int seq);

#endif  /* EMM_TASK_SEQ_H */

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
 *            SET EMM SYSTEM HOME STOP   请求中止，当前步骤结束后停止；
 *            SET EMM TRAY HOME          样品盘(A=4)单独回零，**同步阻塞**，跑完才回
 *                                       （2026/10/19 新增，流程与第 17 步逐字一致）。
 *
 *          回零顺序固定为（**2026/10/23 改版：3 号与 2 号交替两轮**）：
 *            3 号回零 → 下 1200 → 2 号回零 → 3 号回零 → 下 1200
 *            → 2 号回零 → 顺 800 → 2 号回零 → 3 号回零 → 下 2500 → 样品盘，
 *          即 **3 号 3 次碰撞回零** + 2 号 3 次碰撞回零
 *          （每次 = 写参数 + 触发）+ 4 次绝对定位 + 样品盘 1 步
 *          = **17 步**。每一步都有 `[回零]` 前缀的串口打印。
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

/**
 * @brief 样品盘（A=4）单独回零 —— 与整机回零第 17 步**完全相同**的流程。
 * @details **2026/10/19 新增**。用户要求"引出样品盘电机单独回零的 API，跟现在
 *          初始化时样品盘电机回零的逻辑和流程一致"。流程：
 *            ① 使能接近开关监控（触发即停 4 号电机）；
 *            ② **防漂移**：若开关此刻已处于触发状态（托盘停在上次标定的 01 工位），
 *               先**逆时针**转出触发区直到释放 —— 否则"等触发"会立刻判定成功，
 *               零点每回零一次就顺时针漂 `N` 脉冲；
 *            ③ 速度模式（`RPM=10 ACC=10`）**顺时针**旋转，直到接近开关触发；
 *            ④ 发停止并等停稳（读 `0x43` 实时转速，连续 2 次为 0）；
 *            ⑤ **关闭接近开关监控**（此后开关被压住也不会再急停）；
 *            ⑥ `MODE=0` **相对**位置模式**顺时针**再走 `N` 脉冲（默认 50）；
 *            ⑦ `0x0A/0x6D` 位置清零（此处 = **01 号工位** = 绝对脉冲 0）；
 *            ⑧ `0x93/0x88` 设**单圈回零零点**；
 *            ⑨ 读回 `0x43` 核对"当前位置 ≈ 0"（**只告警、不判失败**）；
 *            ⑩ **无论成败都关闭**接近开关监控。
 *          与整机回零共用 `emmHomeTrayHomeCore()`，两条路径的行为**逐字一致**，
 *          仅抬头打印不同（整机回零打 `步骤 17/17`，本接口打独立抬头）。
 * @note **同步阻塞**：在调用者任务里一步步跑完才返回，耗时约 **1~10 秒**
 *       （找开关最坏接近一整圈，`RPM=10` 时一圈约 6 秒）。调用方需自行保证
 *       不与其它运动并发；本函数内部已做"整机回零 / 任务序列 / 系统初始化"互斥检查。
 * @note **不改 `sys_is_homed`**：单独回样品盘不代表 2/3 号已建立零点。
 * @return true 标定成功；false 失败，或被整机回零 / 任务序列 / 系统初始化占用而拒绝。
 */
bool EmmSystemHome_TrayHome(void);

/**
 * @brief 供**任务序列复位阶段**调用的样品盘回零入口（2026/10/19 新增）。
 * @details 与 `EmmSystemHome_TrayHome()` 的**唯一差别**：
 *            1. **不检查** `EmmTaskSeq_IsActive()` —— 调用方就是序列自己，那道检查
 *               必然为 true（否则序列无法在收尾时回零）；
 *            2. 注册外部中止钩子，让序列的 `SEQ STOP` 能**立即打断**找开关过程。
 *
 *          标定本体、监控使能/关闭、失败告警全部复用 `emmHomeTrayHomeCore()`，
 *          与 `SET EMM TRAY HOME`、整机回零第 17 步**逐字一致**。
 *
 *          **仍然保留的互锁**：整机回零运行中、系统初始化运行中（都会并行抢 4 号电机）。
 *
 *          **中止语义**：`SEQ STOP` 后在下个轮询点（100 ms）退出并停 4 号电机；
 *          流程内部在"转出触发区后 / 开关触发后 / 前进 N 脉冲后 / 位置清零后"
 *          四个关键点判中止，**不会把半路位置设成单圈零点**。`SEQ PAUSE` 不打断。
 *
 * @note 仅供 `EmmTaskSeq` 内部调用，不要暴露给 L3 命令（上位机请用 `SET EMM TRAY HOME`）。
 * @return true 标定成功；false 失败或被中止。
 */
bool EmmSystemHome_TrayHomeForSeq(void);

/**
 * @brief 修改"样品盘标定时触发后顺时针前进"的脉冲数（`SET EMM TRAY ZERO P=<n>`）。
 * @details **2026/10/26 新增**。标定流程在"接近开关触发 → 关闭监控"之后，还会
 *          **顺时针**再走 `N` 脉冲，把该点定为 01 号工位。`N` 原来写死在代码里，
 *          现场需要微调（开关安装公差、工位对齐），故引出本接口。
 *
 *          **为什么必须是相对位置**：本工程约定 4 号电机 `DIR=0`（顺时针）= 位置
 *          **递减**，从触发点（清零后的 0）继续顺时针走的绝对目标会是 **−N**，
 *          而 `positionMode()` 的目标是无符号 32 位，写不进去。走完立刻清零，
 *          所以不存在"相对运动累积漂移"的问题。
 *
 *          `N` 落 **NVS**（namespace `emmhome`，key `trayzp`）：整机回零每次上电都跑，
 *          不落盘等于改了白改。
 *
 * @param pulses 脉冲数，范围 `0 ~ 3200`（一圈 = 3200 脉冲 = 360°，故 50 ≈ 5.6°）；
 *               传 0 表示"触发点即 01 号工位"（等价于旧行为）。
 * @return true 已保存并生效；false 超范围或 NVS 写入失败（此时保持旧值不变）。
 */
bool EmmSystemHome_SetTrayZeroPulses(uint32_t pulses);

/**
 * @brief 读取当前"触发后顺时针前进"的脉冲数（`GET EMM TRAY ZERO`）。
 * @details 首次调用会懒加载 NVS；NVS 无记录或读失败时返回出厂默认值 50。
 * @return 当前生效的脉冲数。
 */
uint32_t EmmSystemHome_GetTrayZeroPulses(void);

/**
 * @brief 上位机命令 `SET EMM TRAY HOME`（样品盘单独回零）。
 * @details 无参数。**同步执行**：流程跑完才回复，因此上位机可从应答直接判断成败。
 *          前置检查与应答：
 *            - 结构电源已关闭          → `ERR @<seq> 109`（`NO_STEP_PWR`）；
 *            - 整机回零/序列/初始化运行中 → `ERR @<seq> 110`（`SEQ_RUNNING`）；
 *            - 回零执行失败            → `ERR @<seq> 117`（`EMM_HOME_FAILED`）；
 *            - 成功                    → `OK @<seq> EMM TRAY HOME DONE`。
 */
bool cmdSetEmmTrayHome(const char *args, int seq);

/**
 * @brief 上位机命令 `SET EMM TRAY ZERO P=<n>`（设置样品盘标定触发后的前进脉冲数）。
 * @details **2026/10/26 新增**。应答：
 *            - 成功                    → `OK @<seq> EMM TRAY ZERO P=<n>`；
 *            - 缺 `P=` / 非数字 / 越界 / NVS 写失败 → `ERR @<seq> 102`（`PARAM_ERR`）。
 *          **不受运行期互锁限制**：只改"下次标定用"的值，不影响正在跑的动作。
 */
bool cmdSetEmmTrayZero(const char *args, int seq);

/**
 * @brief 上位机命令 `GET EMM TRAY ZERO`（查询触发后的前进脉冲数）。
 * @details 应答：`OK @<seq> EMM TRAY ZERO P=<n>`。
 */
bool cmdGetEmmTrayZero(const char *args, int seq);

#endif

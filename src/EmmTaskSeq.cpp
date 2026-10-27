/**
 * @file  EmmTaskSeq.cpp
 * @author your name (you@domain.com)
 * @brief 任务序列执行引擎（L2 复合动作）实现——ESP32-KWQ3，**第三步：真实机械动作**。
 * @version 0.1
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 * @details 实现思路（对齐 KWQ1 的 `process.cpp` 任务序列流程）：
 *          1. 一个常驻任务（优先级 2、固定 Core 1、栈 8192）以 10ms 周期轮询
 *             `emmTaskSeqRun()`，队列状态由 `EmmTaskQueue` 持有；
 *          2. 控制命令（START/PAUSE/RESUME/STOP）只改 `SeqState` 与游标，不在
 *             命令回调里做任何耗时动作，因此上位机总能立即拿到 ACK；
 *          3. `emmTaskSeqRun()` 先按 `SeqState` 分派（STOPPED 直接返回、PAUSED 冻结
 *             计时），再按 `TaskPhase` 推进单条任务；需要跨轮询周期等待的阶段
 *             （只有"检测位停留 T 分钟"）再用 `s_subStep` 子步骤索引细化 —— 这与 KWQ1 的
 *             `taskPhase` + `move_pos_stage` / `auxStage` 是同一套双层状态机；
 *          4. 每一步都是"下发一条 EMM 命令 → 等待驱动器到位"的**阻塞式**等待，
 *             等待循环内部处理**暂停 / 恢复 / 中止**（与 `EmmSystemHome` 同构）；
 *          5. 计时用"累计法"而不是 `millis() - start`：暂停期间不累计，恢复后
 *             从断点继续，因此 PAUSE 不会把剩余时间吃掉（用于检测位 `T` 分钟停留）。
 *
 *          **暂停 / 恢复（用户确认：立即停机 + 重算剩余脉冲）**
 *            暂停：等待循环发现 `SeqState == PAUSED` → 对**本步涉及的全部电机**
 *                  `gEmm.stop()` → 等转速归零 → 停在原地空转等待；
 *            恢复：发现 `SeqState == RUNNING` → **重发本步的定位命令**。因为
 *                  `MODE=1` 是绝对定位，重发即"继续走到原目标"，剩余脉冲由驱动器
 *                  自己算 —— 这与 KWQ1 的 `stepperX->moveTo(targetX)` 完全同构。
 *                  本模块所有真实动作都统一用 `MODE=1` 绝对定位下发（含样品盘），
 *                  因此"重发"对每一步都成立。
 *
 *          **失败处理（用户确认）**：任一步失败/超时 → 广播停止全部 EMM 电机 +
 *          序列回 `STOPPED`（保留队列便于现场排查），与 `EmmSystemHome` 一致。
 *
 *          **运动完成判据**：KWQ1 由步进任务上报 `NOTIFY_MOTION_DONE`；KWQ3 的电机
 *          是"驱动器内部跑完定长脉冲"，因此改为"轮询 `0x43` 系统状态判到位"。
 *          **判据本体集中在 `EmmMotionJudge.h`（纯函数、无 IO），本文件只保留
 *          轮询循环与暂停/恢复/中止胶水** —— 2026/10/10 抽取，目的是消除
 *          "同一个判据两份实现"造成的分叉（当时 `EmmSystemHome` 用 `moved`、
 *          本文件用 `pendingSeen`，行为不一致）。
 *          定位判据随后按用户意见**简化为"已停稳 且 驱动器到位标志 `Prf_TF`"**
 *          （见 `EmmMotionJudge.h` 第四节）；回零判据走 `EmmMotionJudge.h` 的
 *          `emmHomingFeed()`（`0x3B` 的 bit2/bit3），与整机回零的 `emmHomeWaitHoming`
 *          共用同一份实现。
 */

#include "common.h"
#include "EmmTaskSeq.h"
/** 运动/回零"完成判据"的唯一实现（纯函数，无 IO）。 */
#include "EmmMotionJudge.h"
/** 序列点位（运行期可改 + NVS 保存），2026/10/19 新增。 */
#include "EmmSeqPoints.h"

#include <string.h>
/* **2026/10/27**：`SEQ RESUME MODE=` 已移除，`<strings.h>`（`strncasecmp`）与
   `<stdlib.h>`（`strtoul`）在本文件已无调用点，一并去掉。 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ==========================================================================
 * 可调参数集中定义
 * ========================================================================== */

/** 序列执行任务优先级：低于整机回零（3）与安全监控，等于系统初始化（2）。 */
#define EMM_TASK_SEQ_TASK_PRIORITY     2
/** 序列执行任务固定核心。 */
#define EMM_TASK_SEQ_TASK_CORE         1
/**
 * @brief 序列执行任务栈大小（字节）。
 * @note 第三步接入真实动作后，调用链变成
 *       `任务 → 状态机 → phase 处理 → 步骤等待 → EmmV5 组帧`，
 *       与 `EmmSystemHome` 的嵌套深度相当，因此由骨架期的 4096 提到 8192。
 */
#define EMM_TASK_SEQ_TASK_STACK        8192U

/** 状态机轮询周期，单位 ms。与 KWQ1 的 `taskProcessQueue()` 保持一致。 */
#define EMM_TASK_SEQ_POLL_MS           10U

/**
 * @brief 检测位"停留 T 分钟"期间打印进度日志的间隔，单位 ms（= 1 分钟）。
 * @note 2026/10/14 用户要求：到检测位停留的那几分钟**每隔一分钟**打印一条进度；
 *       其余日志的打印间隔不变（它们都是事件驱动、非周期打印）。
 *       原 `EMM_TASK_SEQ_PROGRESS_MS`（2000U）随取样位停留一并下线。
 */
#define EMM_TASK_SEQ_DWELL_PROGRESS_MS 60000U

/** 暂停时三色灯慢闪周期，单位 ms。 */
#define EMM_TASK_SEQ_PAUSE_BLINK_MS    500U

/*
 * **2026/10/21 下线**：原来事件帧带一个固定序号 `EVT …`（沿用 KWQ1 的
 * `EMM_TASK_SEQ_EVT_SEQ`）。用户确认 `@100` 是假序号、会与真实命令序号撞车，
 * 已从所有 `EVT` / `ERR` 帧里去掉 —— 现在只有**真实命令应答**才带 `@<seq>`
 * （`OK @600 SEQ START` / `ERR @600 102`）。此常量随之下线。
 */

/* ---------------- RS485 事务参数 ---------------- */

/** 单条 EMM 命令等待驱动器 ACK 的超时，单位 ms。 */
#define EMM_TASK_SEQ_CMD_TIMEOUT_MS    400U

/** 获取共享 RS485 总线的最长等待，单位 ms；超时则本轮跳过。 */
#define EMM_TASK_SEQ_BUS_TIMEOUT_MS    50U

/** 定位/回零过程中轮询状态的周期，单位 ms。 */
#define EMM_TASK_SEQ_STATE_POLL_MS     100U

/**
 * @brief 单步"等待到位/等待停稳/等待回零完成"的超时上限（ms）。
 * @note 步骤 5（3 号以 RPM=1 慢速走 1000 脉冲）实测约 19 s，这里给 60 s 余量。
 *       现场若觉得太长/太短，改这一个宏即可（用户确认"卡临界也没事"，
 *       这里仍留余量以避免临界抖动导致偶发误判失败）。
 */
#define EMM_TASK_SEQ_MOVE_TIMEOUT_MS   50000U

/*
 * 定位到位判据的**算法与常量**已集中到 `EmmMotionJudge.h`（2026/10/10 抽取）：
 *   - 判据函数 `emmArriveReached()`（= 已停稳 且 `Prf_TF` 置位）/
 *     `emmJudgeStopped()` / `emmJudgeStallProtect()` / `emmJudgeGraceMs()`；
 *   - 起步保护公式 `emmJudgeRampMs()` / `emmJudgeGraceMs()`；
 *   - 脉冲↔计数换算 `emmJudgePulseToCounts()`（样品盘"已在工位就不动"仍用）。
 * 本文件**不再自己定义这些数值**，避免"同一判据两份实现"再次分叉。
 */

/**
 * @brief 位置运动起步保护时间的下限（ms）。
 * @note 实际起步保护取 `max(本值, 加速时间 × 2)`，其中加速时间按手册换算
 *       `RPM × (256 - ACC) × 50 µs`。避免命令刚下发、电机还没起步就误判到位。
 */
#define EMM_TASK_SEQ_MOVE_GRACE_MS     300U

/**
 * @brief 同步步里"每条缓存命令（`SYNC=1`）之后"的静默间隔（ms）。
 * @details 实机现象（2026/10/10）：两条 `SYNC=1` 与广播 `SYNC A=ALL` 在 **4~11 ms**
 *          内连发时，驱动器**完全不执行** —— 目标从未被设置、位置纹丝不动（新判据
 *          如实报超时 + "全程未出现目标误差"）；而手动用 L1 命令、间隔约 **1 s** 时
 *          能正常同步运动。说明驱动器需要一段静默时间才能把缓存命令落盘，
 *          紧接着发触发帧会被丢掉（应答 `0x02` 是 UART 层立刻回的，不代表已落盘）。
 *
 *          因此**每条缓存命令之后**都留出本间隔：这样两台驱动器各自都拿到
 *          至少一段完整的静默时间，而不是只有最后发命令的那台才有。
 * @note 取值 500 ms 是实测失败间隔（~11 ms）的约 50 倍，足够覆盖驱动器主循环的
 *       落盘延迟；代价是每个同步步多花 2×500 ms。若实机仍不同步，先把它加到
 *       `1000`（手动验证过的量级）再试。
 */
#define EMM_TASK_SEQ_SYNC_GAP_MS       500U

/**
 * @brief 广播同步触发之后、开始轮询之前的静默时间（ms）。
 * @details 实机现象（2026/10/10）：两条 `SYNC=1` 各留 500 ms 后广播触发，两台电机
 *          仍纹丝不动、目标从未设置；而手动用 L3 命令发同样三条却能同步运动。
 *          手册 §3.5.2 只要求"每条命令之间延时**几毫秒**，防止粘包失效"，说明
 *          **间隔本身不是根因**；两条路径最终调用的又是同一个
 *          `gEmm.positionMode(...sync=true)` / `gEmm.synchronousMotionBroadcast()`，
 *          发出的字节逐字节相同。
 *
 *          唯一实质差异在**触发之后**：`emmSeqWaitArriveAll()` 一开头会立刻
 *          （约 1 ms 内）向 A=2/A=3 各发一条 `0x43` 读系统状态取基准，而手动验证
 *          时并没有这个动作。触发帧是 `writeFrameNoAck` 的**裸帧**，发完立即返回，
 *          只代表字节已上总线；驱动器还需要几毫秒才真正开始执行缓存的运动。
 *          此时紧跟一条读命令，就可能把还没起步的运动打断（症状正好是
 *          "全程未出现目标误差"）。
 *
 *          因此触发后先静默本时长，再让上层开始轮询/取基准。
 * @note 取 200 ms：远大于"驱动器起转所需"，又远小于最短一步的运动时长，
 *       不影响任何到位判据。若实机仍不动，把它加到 500 再试。
 */
#define EMM_TASK_SEQ_SYNC_SETTLE_MS    200U

/**
 * @brief 回零"必须见过 Org_SF=1"的确认截止时间，单位 ms。
 * @note 与 `EmmSystemHome` 的 `EMM_HOME_START_GRACE_MS` 同一含义：0x3B 的
 *       bit2/bit3 同时为 0 有两种含义（回零成功 / 刚上电从未回零），驱动器自己
 *       不区分，因此轮询路径必须"先见过 bit2=1"才允许判成功；超过本时间仍未见过
 *       就判"回零未执行"，**不当成功**。
 */
#define EMM_TASK_SEQ_HOME_START_GRACE_MS  600U

/** 连续通信失败次数上限；达到即判定该步失败，避免故障设备永久占用总线。 */
#define EMM_TASK_SEQ_MAX_COMM_ERRORS   3U

/**
 * @brief 单个步骤最多登记几台电机的运动描述。
 * @details 2+3 号同步补偿步会同时驱动 2 台电机，暂停恢复时要一起重发，因此
 *          运动描述用一个小数组保存（单电机步只占 1 条）。
 */
#define EMM_TASK_SEQ_MAX_SYNC_MOTIONS  2U

/* ---------------- 机械位置常量（单位：命令脉冲，3200 脉冲 = 一圈） ---------------- */

/** 旋转电机（A=2）位置：初始位 / 抛弃位 / 回收位（单圈零点处，= 0）。 */
#define EMM_SEQ_ROT_POS_HOME           0U
/*
 * ⚠️ 2 号（旋转）的 4 个可调点位已**移出本文件**（2026/10/19），改为运行期可改参数：
 *      EMM_SEQ_ROT_POS_SAMPLE / DETECT / DISCARD_1 / DISCARD_2
 *    现在定义在 `EmmSeqPoints.h`，语义是"**出厂默认值**"。
 *    运行期当前值用 `EmmSeqPoints_Get(EMM_SEQ_POINT_xxx)` 读取；上位机可用
 *    `SET EMM POINT SAMPLE=985` 修改并掉电保存（NVS）。
 *    本文件里这 4 个宏**不再直接参与运动**，改宏只影响"NVS 为空时的默认值"。
 */

/** 升降电机（A=3）位置：安全高度（= 0）。 */
#define EMM_SEQ_LIFT_POS_SAFE          5000U
/* 安全高度2（检测位临时用） */
#define EMM_SEQ_LIFT_POS_SAFE_2        5000U
/**
 * @brief 升降电机（A=3）位置：与取样位旋转**同步**的补偿高度。
 * @note **2026/10/14 新流程起本宏不再被引用**（取样位改为"3 号先降到起始位、
 *       2 号再旋转"的顺序执行，不再做同步补偿）。按用户要求**保留备用**：
 *       将来若恢复同步补偿方案，直接引用本宏即可。
 */
#define EMM_SEQ_LIFT_POS_SAMPLE_SYNC   1050U
/**
 * @brief 升降电机（A=3）位置：取样位的"起始下降位"（5000）。
 * @details **2026/10/14 新流程新增**。回零完成后 3 号零点已标定，取样动作第一条
 *          就是 `SET EMM MOVE A=3 DIR=1 RPM=180 ACC=40 PULSE=5000 MODE=1`，
 *          即绝对下降到本位置建立高度基准；随后 2 号才旋转到取样位正上方。
 * @note 数值与 `EMM_SEQ_LIFT_POS_SAFE` 相同，但**语义不同**（这里是"取样起点"，
 *       后者是"抬回安全高度"），故单独定义一个宏，避免读代码时混淆。
 */
#define EMM_SEQ_LIFT_POS_SAMPLE_START  5000U
/**
 * @brief 升降电机（A=3）位置：复位阶段"慢速上升至"的位置（**2000**）。
 * @details **2026/10/14 新流程新增**。整条队列跑完后复位的第一条：
 *          `SET EMM MOVE A=3 DIR=1 RPM=100 ACC=20 PULSE=2000 MODE=1`
 *          （2026/10/20 按代码同步：旧注释写的 `RPM=180 / PULSE=1000` 已过时）。
 * @note 数值与 `EMM_SEQ_LIFT_POS_DISCARD_1`(1000) 不同，语义也不同
 *       （回收位 vs 复位），故单独定义。
 */
#define EMM_SEQ_LIFT_POS_RESET         2000U
/*
 * ⚠️ 3 号（升降）的 5 个可调点位也已**移出本文件**（2026/10/20），改为运行期可改参数：
 *      EMM_SEQ_LIFT_POS_SAMPLE_1 / SAMPLE_2 / DETECT / DISCARD_2 / DISCARD_3
 *    现在定义在 `EmmSeqPoints.h`，语义是"**出厂默认值**"。
 *    运行期当前值用 `EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_xxx)` 读取；上位机可用
 *    `SET EMM POINT LIFT_S1=32000` 修改并掉电保存（NVS），`GET EMM POINT LIFT` 查询。
 */
/**
 * @brief 升降电机（A=3）位置：回收位第一段（先降 1000）。
 * @note 这一步对应的动作在 `emmSeqRunDiscardPos()` 里**已被注释停用**，
 *       因此**不纳入**运行期可改（2026/10/20 用户确认）。
 *       将来恢复该步时，若要让它也可调，按 `EmmSeqPoints` 的 5 张表同步加一项即可。
 */
#define EMM_SEQ_LIFT_POS_DISCARD_1     1000U

/**
 * @brief 样品盘（A=4）：每个工位对应的脉冲数（36° = 1/10 圈）。
 * @details 用户确认：样品盘改用 **绝对位置模式（MODE=1）**，
 *          工位 N ↔ 绝对脉冲 `(N − 1) × 320`：
 *            01 号 = 0、02 号 = 320、03 号 = 640、… 10 号 = 2880。
 *          这与"整机回零后样品盘在 01 号工位"完全自洽（回零零点即 01 号）。
 */
#define EMM_SEQ_TRAY_PULSE_PER_SLOT    320U
/** 样品盘（A=4）：工位总数。 */
#define EMM_SEQ_TRAY_SLOT_COUNT        10U
/**
 * @brief 样品盘定位转速与加速度。
 * @note 用户实机验证过的取值：`A=4 DIR=? RPM=5 ACC=5 PULSE=… MODE=1 SYNC=0`
 *       （更早的一次验证用的是 `ACC=0`，两者相差不到 2 ms，这里取最新值）。
 */
#define EMM_SEQ_TRAY_RPM               5U
#define EMM_SEQ_TRAY_ACC               5U

/* ---------------- 2 号（旋转）收尾回零 ---------------- */

/**
 * @brief 复位阶段 2 号（A=2）回单圈回零原点时的转速与加速度。
 * @note 2026/10/19 新增。取值与 2 号在检测位/回收位的定位动作一致（`RPM=10 ACC=10`），
 *       慢速收尾，避免收尾瞬间过冲把原点顶偏。
 */
#define EMM_SEQ_ROT_RETURN_RPM         10U
#define EMM_SEQ_ROT_RETURN_ACC         10U

/**
 * @brief 复位阶段 2 号（A=2）回原点的**旋转方向**：`1` = 逆时针。
 * @note 2026/10/19 用户明确："**回零方向始终是逆时针，= 1**"。
 *       本工程约定 `DIR=0` = 顺时针、`DIR=1` = 逆时针（见《修改日志.md》§7.17、
 *       `EmmSystemHome.cpp` 顶部宏注释）。
 *       ⚠️ 与序列里 2 号其余绝对定位不同：取样位 / 检测位 / 回收位那 4 处全部写
 *       `DIR=0`（那是"定位"，方向由驱动器按目标算）。本处按用户要求**固定用回零方向**，
 *       不再按 `delta` 符号推导。
 */
#define EMM_SEQ_ROT_RETURN_DIR         1U

/**
 * @brief 收尾回零后读回位置的允许偏差（驱动器计数，65536/圈）。
 * @note 2026/10/19 新增，取 **64** 计数（≈ 3.1 个脉冲 ≈ 0.35°）。
 *       理由：
 *         - 定位判据 `EMM_JUDGE_MOVE_TOL_COUNTS` 只有 8 计数（≈0.044°），那是给
 *           "样品盘已在工位就不动"用的，**过严**，用作回零核对会频繁误报；
 *         - 与整机回零的样品盘零点核对 `EMM_TRAY_HOME_ZERO_TOL_COUNTS`（64）取
 *           同一量级，两处"零点核对"用同一把尺子；
 *         - 64 计数远小于最小工位间隔（320 脉冲 = 6553 计数），不会漏判真正的偏移。
 */
#define EMM_SEQ_ROT_RETURN_TOL_COUNTS  64L

/* ---------------- 检测位停留时间保护 ---------------- */

/**
 * @brief `T`（分钟）停留的上限，单位：分钟。
 * @note 任务里的 `T` 在命令层无上限（用户需求），但序列里必须有个兜底，防止误填
 *       一个极大值把序列永久卡死。这里给 30 分钟；超过则按 30 分钟处理并打印告警
 *       （用户 2026/10/14 确认"保持 30 分钟"）。
 * @note 只保留"分钟"这一个上限常量：停留计算时**先按分钟钳位、再换算成毫秒**，
 *       避免 `T` 极大时 `T × 60000` 溢出 `uint32_t`。
 */
#define EMM_TASK_SEQ_DWELL_MAX_MIN     30.0f

/**
 * @brief `SEQ RESUME`（重做模式）回到取样位上方后的**稳定停留时间**（ms）。
 * @details **2026/10/20 新增**。用户要求"回到取样位的上方，停个 2s"再重新取样，
 *          给机构（取样针、丝杆、样品盘）一个稳定的时间窗，避免刚停稳就往下扎。
 */
#define EMM_TASK_SEQ_RESUME_SETTLE_MS  2000U

/* ==========================================================================
 * 模块内部状态
 * ========================================================================== */

/** 序列执行任务句柄；非空表示任务已创建。 */
static TaskHandle_t s_seqTaskHandle = nullptr;

/** 当前任务的执行阶段（`TaskPhase`）。 */
static volatile TaskPhase s_taskPhase = TASK_IDLE;

/**
 * @brief 当前 phase 内的子步骤索引（对应 KWQ1 的 `move_pos_stage` / `auxStage`）。
 * @details 每进入一个 phase 时清零，每推进一个子步骤 +1。目前 `TASK_MOVE_POS`
 *          与 `TASK_DETECT_POS` 都用它；其中只有 `TASK_DETECT_POS` 的"停留 T
 *          分钟"需要跨轮询周期，其余子步骤都是阻塞式完成。
 */
static uint8_t s_subStep = 0;

/**
 * 当前子步骤已累计用时（ms），**不含暂停时间**。
 * @details 采用"每周期累加 delta"的方式：只有在 `RUNNING` 时才把
 *          `now - s_lastTickMs` 累加进来；暂停/停止时把 `s_lastTickMs` 清零，
 *          下一周期重新取基准，于是暂停期间的时间不会被计入。
 *          目前只有 `TASK_DETECT_POS` 的"`T` 分钟停留"用得到它。
 */
static uint32_t s_stepElapsedMs = 0;

/** 上一次计时采样时刻（ms）；0 表示"计时已冻结，下一周期重新取基准"。 */
static uint32_t s_lastTickMs = 0;

/** 下一次打印停留进度的累计用时阈值（ms）。 */
static uint32_t s_nextProgressMs = 0;

/**
 * @brief `SEQ RESUME`（重做模式）待执行的"回取样位重做"请求。
 * @details **2026/10/20 新增**。由命令任务（`cmdSeqResume`）置位，由序列任务消费：
 *            - 暂停发生在**运动步**（取样/检测/抛弃里的定位）：阻塞中的
 *              `emmSeqHandlePauseResume()` 看到它就让本步作废，
 *              `emmSeqHandleStepFailure()` 再把它翻译成"换阶段"，**不按失败收尾**；
 *            - 暂停发生在**非运动步**（检测位停留计时）：`emmTaskSeqRun()` 顶层
 *              下一轮看到它直接换阶段。
 *          两条路径互斥（靠 `s_pausedMidStep` 区分），谁先消费谁清标志。
 * @note `volatile`：命令任务写、序列任务读，且要防止编译器把它缓存在寄存器里。
 */
static volatile bool s_restartSampleOnResume = false;

/* ---------------- END CUR / END ALL（2026/10/26 新增） ---------------- */

/**
 * @brief "结束当前 / 结束全部"的待执行模式。
 * @details 参考 KWQ1 的 `end_current` / `end_all` 两个标志，但 KWQ3 是四层架构、
 *          每个阶段是一个阻塞式函数，因此拆成"模式 + 一次性打断"两件东西：
 *            - `s_endMode`：**要做哪种结束**，跨阶段保留，直到回收完成才清；
 *            - `s_endInterrupt`：**现在就把当前这一步掐掉**，一次性，谁消费谁清。
 *
 *          KWQ1 是把 `seqState` 改成 `END_CUR_ALL` 再在状态机里分支；KWQ3 **不这么做**，
 *          因为 `emmTaskSeqRun()` 开头是 `if (st != RUNNING) return;`，
 *          `emmSeqHandlePauseResume()` 又把"非 RUNNING/PAUSED"一律当**中止**处理 ——
 *          改 `seqState` 会把回收/复位这两步自己也掐死。保持 `RUNNING`、
 *          用独立标志是唯一不牵动既有语义的做法。
 */
typedef enum
{
    SEQ_END_NONE = 0,   ///< 没有结束请求
    SEQ_END_CURRENT,    ///< `END CUR`：回收后进下一条；已是最后一条则进复位
    SEQ_END_ALL         ///< 结束全部：回收 → 清空队列 → 复位 → 收尾（**2026/10/27 起由 `SEQ STOP` 触发**）
} SeqEndMode;

/** 待执行的结束模式（命令任务写、序列任务读）。 */
static volatile SeqEndMode s_endMode = SEQ_END_NONE;

/**
 * @brief 一次性"立即掐断当前步骤"请求。
 * @details 由 `EmmTaskSeq_RequestEndCurrent/All()` 置位，被两处消费：
 *            - `emmSeqHandlePauseResume()`：阻塞在定位/停留里的那一步 → 停机并返回 false；
 *            - `emmSeqHandleStepFailure()`：把"被打断"翻译成"转去回收阶段"，
 *              **不按失败收尾**（否则会广播停机 + 保留队列，与"结束当前"完全相反）。
 *          非运动步（如检测位停留）则由 `emmTaskSeqRun()` 顶层直接消费。
 */
static volatile bool s_endInterrupt = false;

/**
 * @brief 当前这条任务**是否已经取到样品**。
 * @details **2026/10/26 新增**，供 `END CUR` / `END ALL` 判断要不要跑回收流程：
 *          在样品盘定位、取样途中就被结束，探头还没进样品，跑回收是空跑一趟
 *          （会把探头带到回收位走完 10 个子步）。KWQ1 用 `getSample` 做同样的事。
 *            - 取样完成（`TASK_MOVE_POS` 结束）→ 置 true；
 *            - 回收完成（`TASK_DISCARD_POS` 结束）→ 清 false；
 *            - 序列启动 / 复位 / 停止 → 清 false。
 */
static bool s_sampleTaken = false;

/**
 * @brief 下一次进入回收阶段时，是否需要**先把 3 号抬到安全高度**。
 * @details **2026/10/26 新增**，只为 `END CUR` / `END ALL` 服务。
 *
 *          正常路径（检测位 → 回收位）在进入回收前，3 号已经被检测阶段抬回
 *          安全高度（5000），所以回收的第一步"旋转至回收位"是安全的。
 *          但 `END` 是**从任意阶段中途切入**的：可能正卡在取样下降（探头在样品里）、
 *          或检测位停留（探头在检测高度 26000）。此时直接执行"旋转至回收位"
 *          会**带着探头撞样品盘**。
 *
 *          因此由 END 路径置位本标志，`emmSeqRunDiscardPos()` 开头先做一次
 *          "抬到安全高度"，做完清标志；正常路径不受影响（标志为 false，不多走一步）。
 */
static bool s_discardLiftFirst = false;

/* ---------------- 归位式暂停（2026/10/27 新增） ---------------- */

/**
 * @brief "暂停请求待执行"标志。
 * @details **2026/10/27 新增**。`cmdSeqPause()` 只置这个标志、**不直接改 `SeqState`**，
 *          原因见 `cmdSeqPause()` 的注释：暂停要先做回收 + 归位，那两步必须由状态机
 *          在 `RUNNING` 下推进；若命令回调里立刻置 `PAUSED`，`emmTaskSeqRun()` 会在
 *          开头 `if (st != RUNNING) return;` 处早退，回收与归位根本跑不起来。
 *
 *          消费点三处，与 `s_endInterrupt` 同构：
 *            - `emmSeqHandlePauseResume()`：运动步 → 停机并返回 false；
 *            - `emmSeqHandleStepFailure()`：把"被打断"翻译成"转去回收/归位"；
 *            - `emmTaskSeqRun()` 顶层：非运动步（检测位停留等）直接消费。
 */
static volatile bool s_pauseRequested = false;

/**
 * @brief "已归位、真正处于暂停待恢复状态"。
 * @details **2026/10/27 新增**。`TASK_PAUSE_PARK` 走完才置位，同时把 `SeqState`
 *          置 `PAUSED`。它与 `s_pauseRequested` 互斥：请求刚进来时只有前者，
 *          归位完成后前者清、后者置。
 *          `SEQ RESUME` 与 `SEQ STOP`/`ESTOP` 都要求先清掉它。
 */
static volatile bool s_pauseParked = false;

/* ---------------- 第三步：机械位置跟踪 ---------------- */

/**
 * @brief 当前正在执行的运动步骤描述（供暂停恢复时重发）。
 * @details 用户确认暂停语义是"立即停机 + 重算剩余脉冲"。KWQ3 的实现与 KWQ1 的
 *          `moveTo(target)` 同构：**记住本步的绝对目标，恢复时重发定位命令**，
 *          剩余脉冲交给驱动器自己算。因此每个运动步骤在下发前都会把自己的
 *          参数记到这里。
 *          同步步（2+3 号一起动）会登记 2 条，暂停时两台一起停、恢复时一起重发。
 */
struct SeqMotion
{
    uint8_t  addr;          ///< 电机地址
    uint8_t  dir;           ///< 0=CW / 1=CCW（保持用户实机验证过的取值）
    uint16_t rpm;           ///< 转速
    uint8_t  acc;           ///< 加速度字段
    uint32_t target;        ///< **绝对目标位置**（命令脉冲单位）
    bool     isHome;        ///< true 表示本步是回零（0x9A），false 是定位（0xFD）
    uint8_t  homeMode;      ///< isHome=true 时的回零模式
    const char *label;      ///< 本步中文说明（日志用，必须指向静态存储）
};

/** 本步登记的运动描述列表；`s_curMotionCount == 0` 表示当前没有运动步骤。 */
static SeqMotion s_curMotions[EMM_TASK_SEQ_MAX_SYNC_MOTIONS] = {};
static uint8_t   s_curMotionCount = 0U;

/**
 * @brief 是否处于"暂停打断在动作中间"的状态。
 * @details 暂停时置位、恢复并重发命令后清零。用于区分"暂停时电机正在跑"
 *          与"暂停时恰好停在步骤边界"两种情况。
 */
static bool s_pausedMidStep = false;

/* ---------------- 内部类型与函数前置声明 ---------------- */

/**
 * @brief 单个阶段/子步骤的推进结果（对应 KWQ1 的 `SM_RUNNING/SM_DONE/SM_FAILED`）。
 */
typedef enum
{
    SEQ_STEP_RUNNING = 0,   ///< 还没做完，下个轮询周期继续
    SEQ_STEP_DONE    = 1,   ///< 本阶段完成，可以推进到下一阶段
    SEQ_STEP_FAILED  = -1   ///< 失败或序列被中止，不要再推进
} SeqStepResult;

static void emmTaskSeqEnterPhase(TaskPhase p);
static void emmTaskSeqFinishAll(void);
static void emmTaskSeqAbort(const char *reason);
static void emmSeqNextStep(void);
static void emmSeqClearMotions(void);
static void emmSeqRecordMotion(uint8_t addr, uint8_t dir, uint16_t rpm,
                               uint8_t acc, uint32_t target, bool isHome,
                               uint8_t homeMode, const char *label);
static SeqStepResult emmSeqRunPreamble(void);
static SeqStepResult emmSeqRunMovePos(void);
static SeqStepResult emmSeqRunTrayPos(const EmmTask &t);
static SeqStepResult emmSeqRunDetectPos(const EmmTask &t);
static SeqStepResult emmSeqRunDiscardPos(void);
static SeqStepResult emmSeqRunReset(void);
static SeqStepResult emmSeqRunRestartSample(void);
static SeqStepResult emmSeqRunPausePark(void);
static bool emmSeqSyncBroadcast(const char *label);

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

const char *EmmTaskSeq_TaskPhaseToStr(TaskPhase p)
{
    /* 顺序必须与 EmmTaskSeq.h 里的 enum TaskPhase 完全一致。 */
    static const char *const kNames[] = {
        "TASK_IDLE",              /* TASK_IDLE               */
        "TASK_MOVE_POS",          /* TASK_MOVE_POS           */
        "TASK_WAIT_RESISTOR_ACK", /* TASK_WAIT_RESISTOR_ACK  */
        "TASK_AUX_RUN",           /* TASK_AUX_RUN            */
        "TASK_PAUSED",            /* TASK_PAUSED             */
        "TASK_END",               /* TASK_END                */
        "TASK_PREAMBLE",          /* TASK_PREAMBLE           */
        "TASK_TRAY_POS",          /* TASK_TRAY_POS           */
        "TASK_DETECT_POS",        /* TASK_DETECT_POS         */
        "TASK_DISCARD_POS",       /* TASK_DISCARD_POS        */
        "TASK_RESET_POS",         /* TASK_RESET_POS          */
        "TASK_RESTART_SAMPLE",    /* TASK_RESTART_SAMPLE    */
        "TASK_PAUSE_PARK"         /* TASK_PAUSE_PARK        */
    };

    if ((int)p < 0 || (size_t)p >= (sizeof(kNames) / sizeof(kNames[0]))) {
        return "UNKNOWN";
    }
    return kNames[(int)p];
}

/**
 * @brief 上报一条阶段事件帧（**2026/10/21 新增**，配合"精简日志"）。
 *
 * @param code  `crc16.h` 里的阶段事件码（`SEQ_SAMPLE_BEGIN` 等）。
 * @param stage 阶段英文名（`SAMPLE` / `DETECT` / `RECYCLE`）。
 * @param what  `BEGIN` 或 `DONE`。
 *
 * @details 帧格式：
 *          `EVT <code> <stage> <BEGIN|DONE> <任务号>/<总数>`
 *          例：`EVT 320 SAMPLE BEGIN 1/3`
 *
 *          任务号取队列游标 +1（游标是 0 基），总数取当前队列条数 ——
 *          上位机据此画进度条、区分"第几条任务"。
 *
 * @note 长度核对：最坏 `EVT 341 RECYCLE DONE 10/10` = 34 字符，
 *       远低于 `sendWithCRC()` 的 127 字符上限（那个缓冲会静默截断）。
 * @note `SAMPLE BEGIN`(320) 不用本函数 —— 它要多带 `A=`/`T=`，见 `emmSeqEventSampleBegin()`。
 */
static void emmSeqEventPhase(int code, const char *stage, const char *what)
{
    sendWithCRC("EVT %d %s %s %u/%u",
                code, stage, what,
                (unsigned)(EmmTaskQueue_Index() + 1U),
                (unsigned)EmmTaskQueue_Count());
}

/**
 * @brief 上报一条**失败**帧（**2026/10/21 改版**）。
 *
 * @param reason 中文短原因（`取样动作失败` / `检测位动作失败` / `复位失败` …）。
 *
 * @details 帧格式：`ERR <原因> <任务号>/<总数>`
 *          例：`ERR 取样动作失败 1/3`
 *
 *          **为什么从 `EVT ERR` 改成 `ERR`**（用户 2026/10/21 明确要求）：
 *          上位机现在**只把 `ERR` 当作板子报错**，其它前缀一律忽略并继续跑。
 *          原来失败要发两帧（`EVT ERR … 322 SAMPLE FAIL` + `EVT ERR … 305 STEP FAILED`），
 *          两帧都会被当未知事件丢掉 → 出错时上位机毫无反应。现在收敛成**一帧 `ERR`**。
 *
 *          **任务号钳位**：复位阶段队列已经跑完（`Index == Count`），
 *          直接 `+1` 会打出 `4/3` 这种越界值，这里钳到最后一条。
 *
 * @note `reason` 必须**短**（中文每字 3 字节）。`sendWithCRC()` 缓冲只有 127
 *       字符，超了会静默截断、连换行都发不出去 —— 只接受"XX失败"这种短句，
 *       **不要**把 `emmSeqHandleStepFailure()` 里那种带括号解释的长 `reason` 塞进来。
 */
static void emmSeqEventFail(const char *reason)
{
    const unsigned total = (unsigned)EmmTaskQueue_Count();
    unsigned cur = (unsigned)EmmTaskQueue_Index() + 1U;
    if (total > 0U && cur > total) {
        cur = total;   /* 复位阶段：队列已跑完，钳到最后一条 */
    }
    sendWithCRC("ERR %s %u/%u", (reason != nullptr) ? reason : "序列动作失败",
                cur, total);
}

/**
 * @brief 把"哪一步失败"翻译成一句中文短原因（**2026/10/21 新增**）。
 *
 * @param label `emmSeqHandleStepFailure()` 收到的步骤名。
 * @return 中文短原因；未知 label 退回 `"序列动作失败"`。
 *
 * @details 映射表：
 *          | label | 原因 |
 *          | --- | --- |
 *          | 取样位 | `取样动作失败` |
 *          | 样品盘 | `样品盘定位失败` |
 *          | 恢复重做 | `回到取样位失败` |
 *          | 检测位 | `检测位动作失败` |
 *          | 回收位 | `回收位动作失败` |
 *          | 复位 | `复位失败` |
 *          | 前导 / 其它 | `前导动作失败` |
 */
static const char *emmSeqFailReason(const char *label)
{
    if (label == nullptr) {
        return "序列动作失败";
    }
    if (strcmp(label, "取样位") == 0) {
        return "取样动作失败";
    }
    if (strcmp(label, "样品盘") == 0) {
        return "样品盘定位失败";
    }
    if (strcmp(label, "恢复重做") == 0) {
        return "回到取样位失败";
    }
    if (strcmp(label, "检测位") == 0) {
        return "检测位动作失败";
    }
    if (strcmp(label, "回收位") == 0) {
        return "回收位动作失败";
    }
    if (strcmp(label, "复位") == 0) {
        return "复位失败";
    }
    return "前导动作失败";
}

/**
 * @brief 上报一条 `SAMPLE BEGIN`（**2026/10/21 改版：带工位与停留时间**）。
 *
 * @param t 当前任务。
 *
 * @details 帧格式：`EVT 320 SAMPLE BEGIN <任务号>/<总数> A=<工位> T=<分钟>`
 *          例：`EVT 320 SAMPLE BEGIN 1/1 A=01 T=0.100`
 *
 *          - `A=` 两位数字（`%02u`），取自 `t.slot`，与 `SEQ ADD A=01` **同源**；
 *          - `T=` 三位小数（`%.3f`），取自 `t.timeSec`（单位分钟，字段名为历史命名），
 *            与 `SEQ ADD … T=` **同源**，是**配置原值**。
 *          上位机用这两个值核对"板子正在跑的就是配置里的那一行"。
 *
 * @note `T=` 在 `320` 里是**配置原值**，在 `332 DWELL BEGIN` 里是**钳位后的实际值**
 *       （超过 30 分钟上限时按 30.000 发）。两者只有在超上限时才会不同。
 */
static void emmSeqEventSampleBegin(const EmmTask &t)
{
    sendWithCRC("EVT %d SAMPLE BEGIN %u/%u A=%02u T=%.3f",
                (int)SEQ_SAMPLE_BEGIN,
                (unsigned)(EmmTaskQueue_Index() + 1U),
                (unsigned)EmmTaskQueue_Count(),
                (unsigned)t.slot, (double)t.timeSec);
}

/**
 * @brief 上报 `DWELL BEGIN`（**2026/10/21 新增**）：探头降到检测位、开始计时那一刻。
 *
 * @param minutes **实际**停留时间（已按 `EMM_TASK_SEQ_DWELL_MAX_MIN` 钳位）。
 *
 * @details 帧格式：`EVT 332 DWELL BEGIN <任务号>/<总数> T=<分钟>`
 *          例：`EVT 332 DWELL BEGIN 1/1 T=0.100`
 *
 *          上位机收到它就**开 CA、开始采集**。所以这条必须在"已经到位、还没开始计时"
 *          的瞬间发，不能提前（探头还在下降就开采集会采到空气）。
 */
static void emmSeqEventDwellBegin(float minutes)
{
    sendWithCRC("EVT %d DWELL BEGIN %u/%u T=%.3f",
                (int)SEQ_DWELL_BEGIN,
                (unsigned)(EmmTaskQueue_Index() + 1U),
                (unsigned)EmmTaskQueue_Count(),
                (double)minutes);
}

/**
 * @brief 上报 `DWELL END`（**2026/10/21 新增**）：计时到、抬起之前。
 *
 * @details 帧格式：`EVT 333 DWELL END <任务号>/<总数>`
 *          上位机收到它就**停采集、关 CA**。
 *
 *          **不带 `T=`**：上位机已经知道本轮的 T（`332` 里给过），
 *          重复一遍只是让行变长，没有新信息。
 */
static void emmSeqEventDwellEnd(void)
{
    sendWithCRC("EVT %d DWELL END %u/%u",
                (int)SEQ_DWELL_END,
                (unsigned)(EmmTaskQueue_Index() + 1U),
                (unsigned)EmmTaskQueue_Count());
}

/**
 * @brief 上报一条**复位阶段**事件帧（**2026/10/21 新增**）。
 *
 * @param code `RESET_BEGIN`(350) / `RESET_AXIS_2`(351) / `RESET_AXIS_4`(352)。
 * @param what 事件内容（`BEGIN` / `AXIS 2` / `AXIS 4`）。
 *
 * @details 帧格式：`EVT <code> RESET <what>`
 *          例：`EVT 351 RESET AXIS 2`
 *
 *          **不带任务号 / 总数**：复位是整条队列跑完后的收尾动作、只做一次，
 *          与"第几条任务"无关，`emmSeqEventPhase()` 那套 `N/M` 在这里没有意义。
 *
 *          **只报 3 帧的理由**（2026/10/21 用户选定）：3 号上升由 `RESET BEGIN`
 *          代表（通常 2 秒内完成，失败会给 `RESET FAIL`）；2 号、4 号各有一个
 *          "需要核对 / 要找开关"的节点，最坏要等 25 秒，必须报，否则现场看不出
 *          卡在哪一步。复位完成不再单发 `RESET DONE` —— 紧随其后的
 *          `EVT 300 SEQ ALL DONE` 已经表达了"整条序列结束"。
 *
 * @note 长度核对：最长 `EVT 352 RESET AXIS 4` = 25 字符，
 *       远低于 `sendWithCRC()` 的 127 字符上限（那个缓冲会静默截断）。
 */
static void emmSeqEventReset(int code, const char *what)
{
    sendWithCRC("EVT %d RESET %s", code, (what != nullptr) ? what : "");
}

/**
 * @brief 切换到指定执行阶段，并把子步骤索引、计时与运动描述全部清零。
 * @details 计时基准 `s_lastTickMs` 置 0，下一个轮询周期会重新取 `millis()`
 *          作为起点，因此阶段切换本身不会引入额外耗时。
 *
 *          **2026/10/21 起**：进入取样 / 检测 / 回收阶段时顺带上报
 *          `EVT 32x/33x/34x … BEGIN`，这是精简日志里上位机唯一能看到的
 *          "进度"信号（详细的每步参数已降为 `LOGD`）。
 *          `SEQ RESUME` 重做会重新进入 `TASK_MOVE_POS`，因此会再发一次
 *          `SAMPLE BEGIN` —— 这符合"重做取样"的语义，上位机按新的一轮处理即可。
 */
static void emmTaskSeqEnterPhase(TaskPhase p)
{
    s_taskPhase = p;
    s_subStep = 0;
    s_stepElapsedMs = 0;
    s_lastTickMs = 0;
    s_nextProgressMs = 0;
    s_curMotionCount = 0U;      /* 清掉上一步的运动描述，避免误重发 */
    s_pausedMidStep = false;

    /* 阶段事件（2026/10/21）：精简日志下，这是上位机唯一的进度信号。 */
    switch (p) {
    case TASK_MOVE_POS:
    {
        /*
         * **2026/10/21 改版**：`SAMPLE BEGIN` 要多带 `A=`（工位）与 `T=`（停留分钟），
         * 上位机用这两个值核对"板子正在跑的就是配置里的那一行"。
         * 取不到任务（理论上不会发生：Index < Count 才会进本阶段）时退回旧格式，
         * 保证绝不因为上报而把序列卡死。
         */
        EmmTask t;
        if (EmmTaskQueue_GetTask(EmmTaskQueue_Index(), &t)) {
            emmSeqEventSampleBegin(t);
        } else {
            emmSeqEventPhase(SEQ_SAMPLE_BEGIN, "SAMPLE", "BEGIN");
        }
        break;
    }
    case TASK_DETECT_POS:
        emmSeqEventPhase(SEQ_DETECT_BEGIN, "DETECT", "BEGIN");
        break;
    case TASK_DISCARD_POS:
        emmSeqEventPhase(SEQ_RECYCLE_BEGIN, "RECYCLE", "BEGIN");
        break;
    default:
        /* TASK_IDLE / TRAY / PREAMBLE / RESET / RESTART_SAMPLE 不发阶段事件 */
        break;
    }
}

/**
 * @brief 推进到本 phase 的下一个子步骤，并把子步骤计时清零。
 * @details 与 `emmTaskSeqEnterPhase` 的区别：不换 phase、不动运动描述。
 */
static void emmSeqNextStep(void)
{
    ++s_subStep;
    s_stepElapsedMs = 0;
    s_lastTickMs = 0;
    s_nextProgressMs = 0;
}

/** @brief 复位整个执行状态机到"空闲、未开始"（不动队列内容）。 */
static void emmTaskSeqResetPhase(void)
{
    /*
     * 清掉可能残留的"恢复重做"请求（2026/10/20）：例如 `SEQ RESUME` 刚置位，
     * 序列还没消费就被 `SEQ STOP` / 断电打断 —— 不清的话下次 `SEQ START` 会在
     * 第一轮就莫名其妙跳到 `TASK_RESTART_SAMPLE`。
     */
    s_restartSampleOnResume = false;
    /*
     * **2026/10/26**：`END CUR` / `END ALL` 的待执行请求也必须在这里清掉。
     * 否则"按了结束但序列被 `SEQ STOP` / 断电收尾先一步打断"时，残留的请求会让
     * 下一次 `SEQ START` 第一条任务一结束就莫名其妙跳去回收/复位。
     */
    s_endMode = SEQ_END_NONE;
    s_endInterrupt = false;
    s_sampleTaken = false;
    s_discardLiftFirst = false;
    /*
     * **2026/10/27**：`SEQ PAUSE` 的两个标志同理必须清掉。否则"按了暂停但序列
     * 被 `SEQ STOP` / 断电收尾先一步打断"时，残留的请求会让下一次 `SEQ START`
     * 第一轮就莫名其妙跳去回收 + 归位。
     */
    s_pauseRequested = false;
    s_pauseParked = false;
    emmTaskSeqEnterPhase(TASK_IDLE);
}

/**
 * @brief 整条队列执行完毕的收尾。
 * @details 与 KWQ1 一致：上报 `SEQ ALL DONE` 事件帧，复位游标与队列，回到待机
 *          黄灯。复位动作（2+3 号单圈回零）由 `TASK_RESET_POS` 在调用本函数
 *          **之前**完成（用户确认"整条队列跑完时只做一次"）。
 */
static void emmTaskSeqFinishAll(void)
{
    /* **2026/10/21**：这句降为 `LOGD` —— 紧随其后的 `EVT 300 SEQ ALL DONE`
       已经是上位机认的结束信号，串口不必再重复一遍。 */
    LOGD("[任务序列] ========== 全部任务完成 ==========\n");

    /* 事件帧：`EVT 300 SEQ ALL DONE`，供上位机识别序列结束。
       **2026/10/21 起不再带 `@100` 假序号**（会与真实命令序号撞车）。 */
    sendWithCRC("EVT %d SEQ ALL DONE", (int)TASK_DONE);

    EmmTaskQueue_SetSeqState(STOPPED);//再次添加任务不会直接执行，需要下发SEQ START命令
    EmmTaskQueue_Clear();       /* 与 KWQ1 一致：跑完即清空队列，需重新下发布任务 */
    emmTaskSeqResetPhase();
    rygLedOn(RYG_COLOR_YELLOW);
}

/**
 * @brief 内部异常导致的中止（保留队列内容，便于现场排查）。
 */
static void emmTaskSeqAbort(const char *reason)
{
    Serial.printf("[任务序列] 序列异常中止：%s\n", reason);
    EmmTaskQueue_SetSeqState(STOPPED);
    emmTaskSeqResetPhase();
    rygLedOn(RYG_COLOR_YELLOW);
}

/**
 * @brief 某一步失败时的统一收尾：广播停机 + 序列回 STOPPED（保留队列）。
 * @details 用户确认的失败处理方式，与 `EmmSystemHome` 的收尾一致：
 *          "任一步失败/超时 → 广播停止全部 EMM 电机 + 序列回 STOPPED"。
 * @param label 失败步骤的中文说明（已带 phase 前缀）。
 * @param reason 失败原因。
 */
static void emmTaskSeqStepFailed(const char *label, const char *reason)
{
    Serial.printf("[任务序列] 步骤失败：%s —— %s\n", label, reason);

    /*
     * **2026/10/21 改版：把失败上报给上位机（一帧 `ERR`）**。
     *
     * 帧格式（用户 2026/10/21 指定）：`ERR <原因> <任务号>/<总数>`
     * 例：`ERR 取样动作失败 1/3`
     *
     * 为什么不再用 `EVT ERR 305 STEP FAILED`：上位机现在**只把 `ERR`
     * 当板子报错**，`EVT ERR` 会被当未知事件忽略 —— 出错时上位机毫无反应。
     * 原因由 `emmSeqFailReason(label)` 翻译成中文短句，与串口那行
     * `[任务序列] 步骤失败：<label> —— <reason>` 一一对应。
     *
     * **先发帧、再停机**：`stopBroadcast()` 在 RS485 繁忙时最多要等
     * `EMM_TASK_SEQ_BUS_TIMEOUT_MS`，不能让上报被它拖住。
     * 代价是上位机收到帧后立刻查 `GET STATUS`，可能还在 RUNNING（窗口约 1 ms，
     * 等本函数走完就是 STOPPED）。
     */
    emmSeqEventFail(emmSeqFailReason(label));

    /* 广播停止全部 EMM 电机，避免失败后还有电机在跑。 */
    if (Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        const EmmV5::Status status = gEmm.stopBroadcast(false);
        Rs485Bus_Give();
        Serial.printf("[任务序列] 已广播停止全部 EMM 电机，状态=%s\n",
                      EmmV5::statusToString(status));
    } else {
        Serial.println("[任务序列] RS485 总线繁忙，停止全部 EMM 电机的命令未下发");
    }

    emmTaskSeqAbort(reason);
}

/**
 * @brief 广播停止全部 EMM 电机（`END CUR` / `END ALL` 的"立即停机"用）。
 * @details 与 `emmTaskSeqStepFailed()` 里那段同一套写法：拿不到总线就只打印、不阻塞。
 *          `stopBroadcast()` 失败不影响后续流程 —— 回收/复位的第一步本来就是定位，
 *          会把残留运动覆盖掉。
 */
static void emmSeqBroadcastStop(const char *tag)
{
    if (Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        const EmmV5::Status status = gEmm.stopBroadcast(false);
        Rs485Bus_Give();
        Serial.printf("[任务序列] %s：已广播停止全部 EMM 电机，状态=%s\n",
                      tag, EmmV5::statusToString(status));
    } else {
        Serial.printf("[任务序列] %s：RS485 总线繁忙，停止全部电机的命令未下发\n", tag);
    }
}

/**
 * @brief 当前这一刻收到 `END CUR` / `END ALL`，**是否必须**走回收流程。
 * @details **2026/10/26 新增**。判据不是"取到样品没有"，而是"**探头在不在安全高度**"：
 *            - `TASK_IDLE` / `TASK_PREAMBLE` / `TASK_TRAY_POS`：还没往取样位去，
 *              升降在安全高度，直接换任务/转盘不会撞 —— 跳过回收省一趟空跑；
 *            - `TASK_PAUSE_PARK`（**2026/10/27 新增**）：暂停归位本身就是"抬到安全高度
 *              + 转到取样位"，且已取样时样品在前面的回收里处理过了 —— 跳过；
 *            - 其余阶段（取样位、检测位、回收位）：探头可能已经压低，
 *              **必须**把回收流程走完、把探头抬回安全高度；否则下一条任务的
 *              `TASK_TRAY_POS` 一转盘就会撞到探头。
 *
 *          `s_sampleTaken` 只用来在日志里说明"这趟回收是不是白跑"，
 *          **不参与安全性判断** —— 取样途中被打断时它还是 false，但探头已经在下面了。
 */
static bool emmSeqEndNeedsDiscard(void)
{
    return !(s_taskPhase == TASK_IDLE ||
             s_taskPhase == TASK_PREAMBLE ||
             s_taskPhase == TASK_TRAY_POS ||
             s_taskPhase == TASK_PAUSE_PARK);
}

/**
 * @brief `END CUR` / `END ALL` 在"回收完成（或无需回收）"之后的统一分支。
 * @details **2026/10/26 新增**。两条命令在回收之后分道扬镳：
 *            - `END ALL`：清空队列 → 交给主循环"队列已跑完"分支去做复位与收尾
 *              （`Index >= Count` 自然成立，不需要额外标志）；
 *            - `END CUR`：`Index++` 推进到下一条；若已是最后一条，
 *              主循环同样会因 `Index >= Count` 转入复位阶段。
 *          两条路径最后都由 `emmTaskSeqFinishAll()` 收尾（发 `EVT 300`、回 STOPPED、
 *          清队列、黄灯待机），不新增事件码。
 */
static void emmSeqEndAfterDiscard(void)
{
    const SeqEndMode mode = s_endMode;
    s_endMode = SEQ_END_NONE;
    s_sampleTaken = false;

    if (mode == SEQ_END_NONE) {
        /* 正常跑完一条任务（非 END 触发）：保持原有行为，调用方自己推进。 */
        return;
    }

    /* 序列可能在这期间被 `SEQ STOP` / 断电收尾停掉，那就什么都不做。 */
    if (EmmTaskQueue_GetSeqState() != RUNNING) {
        Serial.println("[任务序列] END：序列已被中止，不再推进");
        return;
    }

    if (mode == SEQ_END_ALL) {
        /* **2026/10/27**：这条分支现在由 `SEQ STOP` 触发（原 `END ALL` 已删除）。 */
        Serial.println("[任务序列] SEQ STOP：回收完成，清空队列并进入复位阶段");
        EmmTaskQueue_Clear();
        emmTaskSeqEnterPhase(TASK_IDLE);
        return;
    }

    /* END CUR */
    const uint8_t idx = EmmTaskQueue_Index();
    const uint8_t total = EmmTaskQueue_Count();
    if ((uint8_t)(idx + 1U) >= total) {
        Serial.println("[任务序列] END CUR：已是最后一条任务，转入复位阶段");
    } else {
        Serial.printf("[任务序列] END CUR：第 %u 条已结束，执行下一条（%u/%u）\n",
                      (unsigned)(idx + 1U), (unsigned)(idx + 2U), (unsigned)total);
    }
    EmmTaskQueue_SetIndex((uint8_t)(idx + 1U));
    emmTaskSeqEnterPhase(TASK_IDLE);
}

/**
 * @brief 阶段处理器返回 `SEQ_STEP_FAILED` 时的统一处置。
 * @details 只有序列**仍在 RUNNING** 时才按"失败"收尾；若已经被 `SEQ STOP` /
 *          `SET POWER OFF` 置成 `STOPPED`，说明是主动中止，不再重复报失败。
 */
static void emmSeqHandleStepFailure(const char *label, const char *reason)
{
    if (EmmTaskQueue_GetSeqState() != RUNNING) {
        Serial.printf("[任务序列] %s 未完成（序列已被中止），不按失败收尾\n", label);
        return;
    }

    /*
     * **2026/10/26 新增**：`END CUR` / `END ALL` 让当前这一步"作废"，**不是失败**。
     * 被打断的阻塞步会返回 `SEQ_STEP_FAILED`，若继续走 `emmTaskSeqStepFailed()`
     * 就会"广播停机 + 发 ERR 帧 + 清队列" —— 与"结束当前后继续下一条"完全相反。
     * 这里只把阶段切到回收：状态保持 RUNNING、队列不动、任务序号不变。
     */
    if (s_endInterrupt) {
        s_endInterrupt = false;

        /* 已经在回收/复位阶段：让它继续走完，不要重入回收。 */
        if (s_taskPhase == TASK_DISCARD_POS || s_taskPhase == TASK_RESET_POS) {
            Serial.printf("[任务序列] %s 收到结束请求，但已在回收/复位阶段，继续执行\n", label);
            return;
        }

        if (!emmSeqEndNeedsDiscard()) {
            Serial.printf("[任务序列] %s 被 END 打断：探头在安全高度，跳过回收\n", label);
            emmSeqEndAfterDiscard();
            return;
        }

        Serial.printf("[任务序列] %s 被 END 打断：不按失败收尾，转入回收流程\n", label);
        s_discardLiftFirst = true;   /* 探头可能还在低位，回收前先抬到安全高度 */
        emmTaskSeqEnterPhase(TASK_DISCARD_POS);
        return;
    }

    /*
     * **2026/10/27 新增**：`SEQ PAUSE` 改成"归位式暂停"。
     * 被打断的阻塞步会返回 `SEQ_STEP_FAILED`，若继续走 `emmTaskSeqStepFailed()`
     * 就会"广播停机 + 发 ERR 帧 + 清队列" —— 与"暂停后还能恢复"完全相反。
     * 这里只把阶段切到回收（需要时）或直接切到归位：状态保持 RUNNING、队列不动、
     * 任务序号不变，所以 `SEQ RESUME` 重做的正是这条任务。
     *
     * ⚠️ `s_pauseRequested` **不在这里清**：它要一直留到归位完成
     *    （`TASK_PAUSE_PARK` 的 DONE 分支）才清 —— 因为 `TASK_DISCARD_POS` 的完成
     *    分支要靠它判断"这次回收是暂停触发的，回收完不要推进任务序号，要转去归位"。
     *
     * ⚠️ 回收 / 复位 / 归位阶段本身出错时**不进本分支**：那说明电机真的有问题，
     *    应该走下面的失败收尾把 `ERR` 帧报出来，而不是重入回收。
     */
    if (s_pauseRequested &&
        s_taskPhase != TASK_DISCARD_POS &&
        s_taskPhase != TASK_RESET_POS &&
        s_taskPhase != TASK_PAUSE_PARK) {
        /*
         * 判据（用户 2026/10/27 原话："已经取到样的情况下回收，否则直接回到取样位
         * 正上方"）：
         *   - `s_sampleTaken`：取样流程已完整走完，探头上确实有样品；
         *   - 或者正在 `TASK_MOVE_POS` 的**子步 ≥ 2**：从"3 号快降至取样高度 1"开始，
         *     探头已经压下去、很可能已经带上样品（此时 `s_sampleTaken` 还是 false，
         *     因为要等整个阶段 DONE 才置位）。这一档必须回收，否则探头带着样品回到
         *     取样位上方，`SEQ RESUME` 重做时旧样品会和新样品撞在一起。
         *   - 其余情况（子步 0/1、前导、转盘、检测位之前就失败的）探头在安全高度且
         *     没带样，直接归位即可，省一趟空回收。
         */
        const bool needRecycle = s_sampleTaken ||
                                 (s_taskPhase == TASK_MOVE_POS && s_subStep >= 2U);
        if (!needRecycle) {
            Serial.printf("[任务序列] %s 被 PAUSE 打断：还没取到样，跳过回收直接归位\n", label);
            emmTaskSeqEnterPhase(TASK_PAUSE_PARK);
            return;
        }

        Serial.printf("[任务序列] %s 被 PAUSE 打断：已取到样，先回收再归位\n", label);
        s_discardLiftFirst = true;   /* 探头可能还在低位，回收前先抬到安全高度 */
        emmTaskSeqEnterPhase(TASK_DISCARD_POS);
        return;
    }

    /*
     * **2026/10/20 新增**：`SEQ RESUME`（重做模式）让本步"作废"，**不是失败**。
     * 若走 `emmTaskSeqStepFailed()` 会广播停机 + 保留队列，与用户要的"重做"完全相反。
     * 这里只把阶段切到 `TASK_RESTART_SAMPLE`：状态保持 RUNNING、队列不动、
     * 当前任务序号不变（所以重做的正是这条任务）。
     */
    if (s_restartSampleOnResume) {
        s_restartSampleOnResume = false;
        Serial.printf("[任务序列] %s 作废（RESUME 重做模式）：回到取样位重做本任务\n", label);
        emmTaskSeqEnterPhase(TASK_RESTART_SAMPLE);
        return;
    }

    /*
     * **2026/10/21 改版**：失败帧统一由 `emmTaskSeqStepFailed()` 发一帧
     * `ERR <原因> <n>/<m>`（原来这里还要先发一条 `EVT ERR … 322 SAMPLE FAIL`，
     * 两帧说同一件事，且 `EVT ERR` 上位机不认，已合并）。
     */
    emmTaskSeqStepFailed(label, reason);
}

/**
 * @brief 阶段成功后推进到下一阶段，但**仅在序列仍是 RUNNING 时**才推进。
 * @details 阶段处理器是阻塞式的，执行期间 `SET POWER OFF` / `SEQ STOP` 可能在
 *          另一个任务的上下文里把序列置成 `STOPPED` 并复位状态机。若此时阶段
 *          处理器恰好返回 `SEQ_STEP_DONE`，不加判断就会把 `TaskPhase` 又改回去，
 *          留下一个与实际状态不符的"过期阶段"。这里统一挡一道。
 */
static void emmSeqAdvanceTo(TaskPhase next)
{
    if (EmmTaskQueue_GetSeqState() == RUNNING) {
        emmTaskSeqEnterPhase(next);
    } else {
        Serial.println("[任务序列] 序列已被中止，不再推进到下一阶段");
    }
}

/**
 * @brief 复位阶段完成后的收尾：**仅当序列仍是 RUNNING** 时才上报"全部完成"。
 * @details 防止"复位刚做完、恰好同时收到 `SEQ STOP` / `SET POWER OFF`"时误发
 *          `SEQ ALL DONE` 事件帧、把主动中止谎报成正常跑完。
 */
static void emmSeqFinishIfRunning(void)
{
    if (EmmTaskQueue_GetSeqState() == RUNNING) {
        emmTaskSeqFinishAll();
    } else {
        Serial.println("[任务序列] 序列已被中止，不发送 SEQ ALL DONE 事件帧");
    }
}

/* ==========================================================================
 * 第三步：RS485 步骤辅助函数
 * --------------------------------------------------------------------------
 * 所有 EMM 事务都遵循与 L1 / EmmSystemHome 相同的约定：
 *   先 Rs485Bus_Take()，再调用 gEmm.*，最后 Rs485Bus_Give()。
 * ========================================================================== */

/** @brief 清空本步的运动描述列表（每个运动步骤在下发前先调用）。 */
static void emmSeqClearMotions(void)
{
    s_curMotionCount = 0U;
}

/**
 * @brief 追加一条运动描述（供暂停恢复时重发）。
 * @note 列表满时从头覆盖，保证不会越界；同步步最多登记 2 条。
 */
static void emmSeqRecordMotion(uint8_t addr, uint8_t dir, uint16_t rpm,
                               uint8_t acc, uint32_t target, bool isHome,
                               uint8_t homeMode, const char *label)
{
    if (s_curMotionCount >= EMM_TASK_SEQ_MAX_SYNC_MOTIONS) {
        s_curMotionCount = 0U;
    }
    SeqMotion &m = s_curMotions[s_curMotionCount++];
    m.addr = addr;
    m.dir = dir;
    m.rpm = rpm;
    m.acc = acc;
    m.target = target;
    m.isHome = isHome;
    m.homeMode = homeMode;
    m.label = label;
}

/**
 * @brief 停止指定电机并等待其转速归零（暂停 / 中止共用）。
 * @param addr   电机地址。
 * @param reason 日志前缀里的原因标签（"暂停" / "中止"），必须指向静态存储。
 * @return true 已停稳；false 超时或通信失败。
 */
static bool emmSeqStopMotorAndWait(uint8_t addr, const char *reason)
{
    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.printf("[任务序列] %s：RS485 总线繁忙，A=%u 停止命令未下发\n",
                      reason, (unsigned)addr);
        return false;
    }
    const EmmV5::Status status = gEmm.stop(addr, false, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    LOGD("[任务序列] %s：A=%u 已下发停止命令，状态=%s\n",
                  reason, (unsigned)addr, EmmV5::statusToString(status));

    /* 轮询 0x3A 直到转速为 0（或超时）。 */
    const uint32_t startedAt = millis();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_STATE_POLL_MS));

        if (millis() - startedAt >= EMM_TASK_SEQ_MOVE_TIMEOUT_MS) {
            Serial.printf("[任务序列] %s：A=%u 等待停稳超时\n", reason, (unsigned)addr);
            return false;
        }
        if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
            continue;
        }
        EmmV5::SystemState state = {};
        const EmmV5::Status ret =
            gEmm.readSystemState(addr, state, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
        Rs485Bus_Give();
        if (ret != EmmV5::STATUS_OK) {
            continue;   /* 单次读失败不算停稳失败，继续等 */
        }
        if (emmJudgeStopped(&state)) {
            LOGD("[任务序列] %s：A=%u 已停稳（实时位置=%ld）\n",
                          reason, (unsigned)addr, (long)state.realtimePosition);
            return true;
        }
    }
}

/**
 * @brief 按 `s_curMotions` 重发本步的全部定位/回零命令（恢复用）。
 * @details 与 KWQ1 的 `stepperX->moveTo(targetX)` 同构：`MODE=1` 是绝对定位，
 *          重发即"继续走到原目标"，剩余脉冲由驱动器自己算。同步步会把登记的
 *          2 台电机都重发（不再走 SYNC 广播，各自独立走完剩余行程即可）。
 * @return true 全部命令已被驱动器接受。
 */
static bool emmSeqReissueMotion(void)
{
    if (s_curMotionCount == 0U) {
        return true;    /* 没有正在进行的运动步，无需重发 */
    }

    for (uint8_t i = 0; i < s_curMotionCount; ++i) {
        const SeqMotion &m = s_curMotions[i];
        if (m.addr == 0U) {
            continue;
        }

        LOGD("[任务序列] 恢复：重发 %s A=%u DIR=%u RPM=%u ACC=%u 目标=%lu\n",
                      m.isHome ? "回零" : "定位",
                      (unsigned)m.addr, (unsigned)m.dir, (unsigned)m.rpm,
                      (unsigned)m.acc, (unsigned long)m.target);

        if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
            Serial.println("[任务序列] 恢复：RS485 总线繁忙，重发失败");
            return false;
        }
        EmmV5::Status status;
        if (m.isHome) {
            /* 回零：重发 0x9A，模式沿用本步原值。 */
            status = gEmm.home(m.addr, m.homeMode, false, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
        } else {
            /* 定位：统一用 MODE=1 绝对定位重发，绝对目标 = 本模块记录的 target。 */
            status = gEmm.positionMode(m.addr, m.dir, m.rpm, m.acc, m.target,
                                       1U /* MODE=1 绝对零点 */, false,
                                       EMM_TASK_SEQ_CMD_TIMEOUT_MS);
        }
        Rs485Bus_Give();

        if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE &&
            status != EmmV5::STATUS_NOT_MOVED) {
            Serial.printf("[任务序列] 恢复：A=%u 重发命令被拒绝，状态=%s\n",
                          (unsigned)m.addr, EmmV5::statusToString(status));
            return false;
        }
    }
    return true;
}

/**
 * @brief 处理"等待期间收到暂停/恢复请求"。
 * @details 返回值语义：
 *          - `true`  ：调用方应继续等待（可能刚停机、也可能刚恢复重发）；
 *          - `false` ：序列已被中止，调用方应立即退出本步。
 * @param addr 本步的主电机地址（即使它不在登记列表里也会被停一次）。
 */
static bool emmSeqHandlePauseResume(uint8_t addr)
{
    const SeqState st = EmmTaskQueue_GetSeqState();

    /*
     * **2026/10/26 新增**：`END CUR` / `END ALL` 要求"电机动作立即停止"。
     * 本步是阻塞式的（`emmSeqWaitArriveAll` 每轮都调到这里），只有在这里返回
     * false 才能把它掐断；随后 `emmSeqHandleStepFailure()` 会把"被打断"翻译成
     * "转去回收阶段"，**不按失败收尾**。
     * 标志**不在这里清**：清理由消费方负责，否则同一轮里多次调用会丢状态。
     *
     * ⚠️ 已在回收/复位阶段时**不打断**：这两个阶段正是 `END` 想要的收尾动作，
     *    掐断它们只会让回收从头再来一遍。
     */
    if (s_endInterrupt &&
        s_taskPhase != TASK_DISCARD_POS &&
        s_taskPhase != TASK_RESET_POS &&
        s_taskPhase != TASK_PAUSE_PARK) {
        Serial.printf("[任务序列] 收到结束请求：立即停止本步电机（主 A=%u）并等停稳\n",
                      (unsigned)addr);
        (void)emmSeqStopMotorAndWait(addr, "结束");
        for (uint8_t i = 0; i < s_curMotionCount; ++i) {
            const uint8_t other = s_curMotions[i].addr;
            if (other != 0U && other != addr) {
                (void)emmSeqStopMotorAndWait(other, "结束");
            }
        }
        return false;
    }

    /*
     * **2026/10/27 新增**：`SEQ PAUSE` 改成"归位式暂停" —— 命令侧只置
     * `s_pauseRequested`，**不动 `SeqState`**（仍是 `RUNNING`），所以下面那些基于
     * `st == PAUSED` 的老逻辑全都不适用了。真正要做的是：把本步电机**立刻停掉**、
     * 等停稳、然后返回 false 让本步退出；`emmSeqHandleStepFailure()` 会把
     * "被打断"翻译成"转去回收 / 归位"，**不按失败收尾**（否则会误广播停机、误清队列）。
     * 标志**不在这里清**：清理由消费方（`emmTaskSeqRun()` 顶层）负责。
     *
     * ⚠️ 与 `s_endInterrupt` 一样，回收 / 复位 / 归位阶段**不打断**：这几个阶段
     *    正是暂停想要的收尾动作，掐断它们只会让回收从头再来一遍。
     */
    if (s_pauseRequested &&
        s_taskPhase != TASK_DISCARD_POS &&
        s_taskPhase != TASK_RESET_POS &&
        s_taskPhase != TASK_PAUSE_PARK) {
        Serial.printf("[任务序列] 收到暂停请求：立即停止本步电机（主 A=%u）并等停稳\n",
                      (unsigned)addr);
        (void)emmSeqStopMotorAndWait(addr, "暂停");
        for (uint8_t i = 0; i < s_curMotionCount; ++i) {
            const uint8_t other = s_curMotions[i].addr;
            if (other != 0U && other != addr) {
                (void)emmSeqStopMotorAndWait(other, "暂停");
            }
        }
        return false;
    }

    /*
     * 已被 STOP / 断电 / 失败收尾中止：**先停机等停稳，再退出**（2026/10/10 用户
     * 要求"STOP 一到就停"）。`EmmTaskSeq_Stop()` 里已经广播停过一次，这里再对本步
     * 登记的电机兜一道，并等停稳，保证收尾时不会残留运动。
     */
    if (st != RUNNING && st != PAUSED) {
        Serial.println("[任务序列] 等待期间序列被中止：立即停止本步电机并等停稳");
        (void)emmSeqStopMotorAndWait(addr, "中止");
        for (uint8_t i = 0; i < s_curMotionCount; ++i) {
            const uint8_t other = s_curMotions[i].addr;
            if (other != 0U && other != addr) {
                (void)emmSeqStopMotorAndWait(other, "中止");
            }
        }
        return false;
    }

    /* 暂停：第一次发现时把**本步涉及的全部电机**停掉并等停稳。 */
    if (st == PAUSED && !s_pausedMidStep) {
        Serial.printf("[任务序列] 收到暂停请求：立即停止本步电机（主 A=%u）并停在原地\n",
                      (unsigned)addr);
        s_pausedMidStep = true;
        (void)emmSeqStopMotorAndWait(addr, "暂停");
        for (uint8_t i = 0; i < s_curMotionCount; ++i) {
            const uint8_t other = s_curMotions[i].addr;
            if (other != 0U && other != addr) {
                (void)emmSeqStopMotorAndWait(other, "暂停");
            }
        }
        return true;
    }

    /* 暂停中：空转等待，不推进。 */
    if (st == PAUSED) {
        return true;
    }

    /* 从暂停恢复：按恢复模式决定"断点续跑"还是"作废重做"。 */
    if (st == RUNNING && (s_pausedMidStep || s_restartSampleOnResume)) {
        s_pausedMidStep = false;

        /*
         * **2026/10/20 新增**：`SEQ RESUME`（重做模式）要求丢弃断点、回取样位重来。
         * 这里**不重发**本步命令，直接返回 false 让本步退出；真正的"换阶段"由
         * `emmSeqHandleStepFailure()` 完成 —— 那里会识别 `s_restartSampleOnResume`
         * 并把本步翻译成"作废"，**不按失败收尾**（否则会误广播停机、误清队列）。
         */
        if (s_restartSampleOnResume) {
            Serial.println("[任务序列] 收到恢复请求(重做模式)：本步作废，准备回取样位重做");
            return false;
        }

        Serial.println("[任务序列] 收到恢复请求(续跑模式)：重发本步命令，从断点继续");
        if (!emmSeqReissueMotion()) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 等待本步登记的全部定位动作到位（阻塞，含暂停/恢复/中止处理）。
 * @details **判据本体在 `EmmMotionJudge.h`**（`emmArriveReached()` /
 *          `emmJudgeStopped()` / `emmJudgeStallProtect()`），本函数只负责"胶水"：
 *            - 起步保护 `emmJudgeGraceMs()`（命令刚下发时先等一段，别急着判到位）；
 *            - 暂停 / 恢复 / 中止；通信错误计数；堵转保护；超时与诊断日志。
 *
 *          判据 = "已停稳 且 驱动器报告 `Prf_TF`（目标位置已到达）"，简化理由见
 *          `EmmMotionJudge.h` 第四节（2026/10/10 用户决策：序列每条命令都等 0x02
 *          应答，命令丢帧会在发送阶段被发现并中止，无需额外的"命令已生效证据"）。
 *          同步步（2 台电机）用同一套逻辑，**全部到位**才返回 true。
 * @param rpm   本步定位命令使用的转速（用于算加速时间）。
 * @param acc   本步定位命令使用的加速度字段（0~255）。
 * @param label 本步中文说明（日志用）。
 * @return true 全部到位；false 失败或序列被中止。
 */
static bool emmSeqWaitArriveAll(uint16_t rpm, uint8_t acc, const char *label)
{
    const uint8_t count = s_curMotionCount;
    if (count == 0U) {
        Serial.printf("[任务序列] %s：内部错误，本步没有登记任何电机\n", label);
        return false;
    }

    /* 起步保护 = max(加速时间 × 2, 300 ms)，公式见 EmmMotionJudge.h 第三节。 */
    const uint32_t graceMs = emmJudgeGraceMs(rpm, acc, EMM_TASK_SEQ_MOVE_GRACE_MS);

    /* done[i]：本步这台电机已判定到位。 */
    bool done[EMM_TASK_SEQ_MAX_SYNC_MOTIONS] = {};

    /* 单步计时基准：暂停恢复后会被重置，保证暂停时长不计入本步耗时。 */
    uint32_t startedAt = millis();
    uint8_t commErrors = 0;
    bool wasPaused = false;

    for (;;) {
        /* 暂停/恢复/中止处理（内部会自己延时）。 */
        if (!emmSeqHandlePauseResume(s_curMotions[0].addr)) {
            return false;
        }

        /*
         * 暂停期间不轮询位置、不判到位，只是空转等恢复。
         * 恢复后驱动器会重发同一个绝对目标（见 `emmSeqReissueMotion`）；判据只看
         * `Prf_TF`，不保留任何"历史证据"状态，所以这里不需要维护额外变量。
         */
        if (s_pausedMidStep) {
            wasPaused = true;
            vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_POLL_MS));
            continue;
        }

        /*
         * 刚从暂停恢复：把单步超时与起步保护的计时基准都重新开始。
         * 否则暂停期间 `millis()` 仍在走，恢复后第一次检查就会把"暂停了多久"
         * 当成"本步已经跑了多久"，长时间暂停后一恢复就误判超时。
         */
        if (wasPaused) {
            wasPaused = false;
            startedAt = millis();
            LOGD("[任务序列] %s：已从暂停恢复，本步计时重新开始\n", label);
        }

        /* 起步保护：刚下发（或刚恢复重发）时先等一段，别急着判到位。 */
        if (millis() - startedAt < graceMs) {
            vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_POLL_MS));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_STATE_POLL_MS));

        if (millis() - startedAt >= EMM_TASK_SEQ_MOVE_TIMEOUT_MS) {
            Serial.printf("[任务序列] %s：等待到位超时（%lu ms）\n", label,
                          (unsigned long)EMM_TASK_SEQ_MOVE_TIMEOUT_MS);
            /*
             * 诊断：把还没到位的电机状态原样打出来（命令目标 / 驱动器目标 / 当前位置 /
             * 误差 / 电机状态字），便于现场判断"没走到"还是"走了没判到位"。
             * 判据只看 Prf_TF（见 EmmMotionJudge.h 第四节），所以这里把 MOTOR_FLAGS
             * 也打出来，看 Prf_TF(0x02) 到底有没有置位。
             */
            for (uint8_t i = 0; i < count; ++i) {
                if (done[i]) {
                    continue;
                }
                const uint8_t addr = s_curMotions[i].addr;
                if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
                    Serial.printf("[任务序列] %s：A=%u 未到位（命令目标 %lu 脉冲），RS485 繁忙\n",
                                  label, (unsigned)addr,
                                  (unsigned long)s_curMotions[i].target);
                    continue;
                }
                EmmV5::SystemState st = {};
                const EmmV5::Status status =
                    gEmm.readSystemState(addr, st, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
                Rs485Bus_Give();
                if (status != EmmV5::STATUS_OK) {
                    Serial.printf("[任务序列] %s：A=%u 未到位（命令目标 %lu 脉冲），"
                                  "读状态失败，状态=%s\n",
                                  label, (unsigned)addr,
                                  (unsigned long)s_curMotions[i].target,
                                  EmmV5::statusToString(status));
                    continue;
                }
                Serial.printf("[任务序列] %s：A=%u 未到位（命令目标 %lu 脉冲 ≈ %ld 计数）——"
                              "驱动器目标=%ld 当前位置=%ld 误差=%ld MOTOR_FLAGS=0x%02X(Prf_TF=%u)\n",
                              label, (unsigned)addr,
                              (unsigned long)s_curMotions[i].target,
                              (long)emmJudgePulseToCounts(s_curMotions[i].target),
                              (long)st.targetPosition, (long)st.realtimePosition,
                              (long)st.positionError, (unsigned)st.motorFlags,
                              (unsigned)((st.motorFlags & EmmV5::MOTOR_POSITION_REACHED) ? 1U : 0U));
            }
            return false;
        }

        /* ---- 逐个轮询本步的每台电机，全部到位才算完成 ---- */
        uint8_t remaining = 0;
        for (uint8_t i = 0; i < count; ++i) {
            if (done[i]) {
                continue;
            }
            const uint8_t addr = s_curMotions[i].addr;

            if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
                ++remaining;
                continue;
            }
            EmmV5::SystemState state = {};
            const EmmV5::Status status =
                gEmm.readSystemState(addr, state, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
            Rs485Bus_Give();

            if (status != EmmV5::STATUS_OK) {
                ++commErrors;
                if (commErrors >= EMM_TASK_SEQ_MAX_COMM_ERRORS) {
                    Serial.printf("[任务序列] %s：A=%u 连续 %u 次读状态异常，状态=%s\n",
                                  label, (unsigned)addr, (unsigned)commErrors,
                                  EmmV5::statusToString(status));
                    return false;
                }
                ++remaining;
                continue;
            }
            commErrors = 0;

            if (emmJudgeStallProtect(&state)) {
                Serial.printf("[任务序列] %s：A=%u 触发堵转保护，MOTOR_FLAGS=0x%02X\n",
                              label, (unsigned)addr, (unsigned)state.motorFlags);
                return false;
            }

            /* 判据本体见 EmmMotionJudge.h 第四节：已停稳 且 Prf_TF 置位。 */
            if (emmArriveReached(&state)) {
                LOGD("[任务序列] %s：A=%u 到位（位置=%ld 误差=%ld）\n",
                              label, (unsigned)addr,
                              (long)state.realtimePosition,
                              (long)state.positionError);
                done[i] = true;
                continue;
            }
            ++remaining;
        }

        if (remaining == 0U) {
            return true;
        }
    }
}

/**
 * @brief 等待本步登记的全部回零动作完成（阻塞，含暂停/恢复/中止处理）。
 * @details **判据本体在 `EmmMotionJudge.h` 的 `emmHomingFeed()`**（2026/10/10 抽取），
 *          与整机回零的 `EmmSystemHome::emmHomeWaitHoming` **共用同一份实现**；
 *          本函数只保留轮询、暂停/恢复/中止、通信错误计数与日志。
 *
 *          判据要点：只取 `0x3B` 的 bit2（正在回零）/ bit3（回零失败），屏蔽 bit0/bit1
 *          （编码器就绪位，默认恒为 1）。**必须先见过 bit2=1** 才允许按"bit2/bit3 都为 0"
 *          判成功，否则视为"从未回零"。
 *          同步回零（2 台）时每台各自维护一份 `homingSeen`，全部完成才返回。
 * @return true 全部回零完成；false 失败或序列被中止。
 */
static bool emmSeqWaitHomingAll(const char *label)
{
    const uint8_t count = s_curMotionCount;
    if (count == 0U) {
        Serial.printf("[任务序列] %s：内部错误，本步没有登记任何电机\n", label);
        return false;
    }

    /* 单步计时基准：暂停恢复后会被重置，保证暂停时长不计入本步耗时。 */
    uint32_t startedAt = millis();
    uint8_t commErrors = 0;
    bool wasPaused = false;
    bool homingSeen[EMM_TASK_SEQ_MAX_SYNC_MOTIONS] = {};
    bool done[EMM_TASK_SEQ_MAX_SYNC_MOTIONS] = {};

    for (;;) {
        if (!emmSeqHandlePauseResume(s_curMotions[0].addr)) {
            return false;
        }
        if (s_pausedMidStep) {
            wasPaused = true;
            vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_POLL_MS));
            continue;
        }

        /* 刚从暂停恢复：计时基准重新开始（理由同 emmSeqWaitArriveAll）。 */
        if (wasPaused) {
            wasPaused = false;
            startedAt = millis();
            LOGD("[任务序列] %s：已从暂停恢复，本步计时重新开始\n", label);
        }

        vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_STATE_POLL_MS));

        const uint32_t elapsed = millis() - startedAt;
        if (elapsed >= EMM_TASK_SEQ_MOVE_TIMEOUT_MS) {
            Serial.printf("[任务序列] %s：回零等待超时（%lu ms）\n", label,
                          (unsigned long)EMM_TASK_SEQ_MOVE_TIMEOUT_MS);
            return false;
        }

        uint8_t remaining = 0;
        for (uint8_t i = 0; i < count; ++i) {
            if (done[i]) {
                continue;
            }
            const uint8_t addr = s_curMotions[i].addr;

            if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
                ++remaining;
                continue;
            }
            uint8_t flags = 0;
            const EmmV5::Status ret =
                gEmm.readOriginFlags(addr, flags, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
            Rs485Bus_Give();

            if (ret != EmmV5::STATUS_OK) {
                ++commErrors;
                if (commErrors >= EMM_TASK_SEQ_MAX_COMM_ERRORS) {
                    Serial.printf("[任务序列] %s：A=%u 连续 %u 次读回零状态异常\n",
                                  label, (unsigned)addr, (unsigned)commErrors);
                    return false;
                }
                ++remaining;
                continue;
            }
            commErrors = 0;

            /* 判据本体见 EmmMotionJudge.h 第五节（与整机回零的 emmHomeWaitHoming 共用）。 */
            const EmmHomingPoll verdict =
                emmHomingFeed(flags, &homingSeen[i],
                              elapsed >= EMM_TASK_SEQ_HOME_START_GRACE_MS);

            if (verdict == EMM_HOMING_POLL_PROTECT) {
                Serial.printf("[任务序列] %s：A=%u 驱动器保护已触发，FLAGS=0x%02X\n",
                              label, (unsigned)addr, (unsigned)flags);
                return false;
            }
            if (verdict == EMM_HOMING_POLL_FAILED) {
                Serial.printf("[任务序列] %s：A=%u 驱动器报告回零失败，FLAGS=0x%02X\n",
                              label, (unsigned)addr, (unsigned)flags);
                return false;
            }
            if (verdict == EMM_HOMING_POLL_DONE) {
                LOGD("[任务序列] %s：A=%u 回零完成\n",
                              label, (unsigned)addr);
                done[i] = true;
                continue;
            }
            if (verdict == EMM_HOMING_POLL_NOT_STARTED) {
                Serial.printf("[任务序列] %s：A=%u 从未观察到正在回零（Org_SF），"
                              "无法确认回零已执行，FLAGS=0x%02X\n",
                              label, (unsigned)addr, (unsigned)flags);
                return false;
            }
            /* EMM_HOMING_POLL_HOMING：正在回零，继续等本台。 */
            ++remaining;
        }

        if (remaining == 0U) {
            return true;
        }
    }
}

/**
 * @brief 下发一条位置命令（不等待完成）。
 * @param sync true 表示 `SYNC=1`（先缓存，等 `SYNC A=ALL` 一起启动）。
 * @return true 命令已被驱动器接受（含"电机未动"这种合法应答）。
 */
static bool emmSeqPositionSend(uint8_t addr, uint8_t dir, uint16_t rpm,
                               uint8_t acc, uint32_t pulse, uint8_t mode,
                               bool sync, const char *label)
{
    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] RS485 总线繁忙，定位命令未下发");
        return false;
    }
    const EmmV5::Status status = gEmm.positionMode(
        addr, dir, rpm, acc, pulse, mode, sync, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();

    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE &&
        status != EmmV5::STATUS_NOT_MOVED) {
        Serial.printf("[任务序列] %s：A=%u 定位命令被拒绝，状态=%s\n",
                      label, (unsigned)addr, EmmV5::statusToString(status));
        return false;
    }
    return true;
}

/**
 * @brief 下发一条回零命令（不等待完成）。
 * @param sync true 表示 `SYNC=1`（先缓存，等 `SYNC A=ALL` 一起启动）。
 */
static bool emmSeqHomeSend(uint8_t addr, uint8_t homeMode, bool sync,
                           const char *label)
{
    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] RS485 总线繁忙，回零命令未下发");
        return false;
    }
    const EmmV5::Status status =
        gEmm.home(addr, homeMode, sync, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();

    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE &&
        status != EmmV5::STATUS_NOT_MOVED) {
        Serial.printf("[任务序列] %s：A=%u 回零命令被拒绝，状态=%s\n",
                      label, (unsigned)addr, EmmV5::statusToString(status));
        return false;
    }
    return true;
}

/**
 * @brief 执行一步定位（下发 + 等待到位），支持暂停/恢复/中止。
 * @param addr      电机地址。
 * @param dir       方向 0=CW / 1=CCW（保持用户实机验证过的取值）。
 * @param rpm       转速。
 * @param acc       加速度字段。
 * @param pulse     命令脉冲数：`MODE=1` 时就是**绝对目标**。
 * @param mode      0=相对上一目标 / 1=绝对零点 / 2=相对实时位置。
 * @param absTarget 本步的**绝对目标位置**，供暂停恢复时重发。
 * @param label     本步中文说明（日志用，必须指向静态存储）。
 * @return true 完成；false 失败或序列被中止。
 */
static bool emmSeqMoveStep(uint8_t addr, uint8_t dir, uint16_t rpm, uint8_t acc,
                           uint32_t pulse, uint8_t mode, uint32_t absTarget,
                           const char *label)
{
    LOGD("[任务序列] %s：A=%u DIR=%u RPM=%u ACC=%u PULSE=%lu MODE=%u\n",
                  label, (unsigned)addr, (unsigned)dir, (unsigned)rpm,
                  (unsigned)acc, (unsigned long)pulse, (unsigned)mode);

    emmSeqClearMotions();
    emmSeqRecordMotion(addr, dir, rpm, acc, absTarget, false, 0U, label);

    if (!emmSeqPositionSend(addr, dir, rpm, acc, pulse, mode, false, label)) {
        return false;
    }
    return emmSeqWaitArriveAll(rpm, acc, label);
}

/**
 * @brief 同步步里"缓存命令之后、触发之前"的静默间隔。
 * @details 见 `EMM_TASK_SEQ_SYNC_GAP_MS` 的说明：实机证实驱动器需要一段静默时间
 *          才能把 `SYNC=1` 缓存命令落盘，紧接着发 `SYNC A=ALL` 会被丢掉。
 *          本函数负责延时并打印一行，便于从串口时间轴确认间隔确实生效。
 * @param label 本步中文说明（日志用）。
 */
static void emmSeqSyncSettleGap(const char *label)
{
    LOGD("[任务序列] %s：同步缓存间隔 %u ms（驱动器需静默时间）\n",
                  label, (unsigned)EMM_TASK_SEQ_SYNC_GAP_MS);
    vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_SYNC_GAP_MS));
}

/**
 * @brief 同步步"广播触发之前"的已中止守卫。
 * @details 【2026/10/10 用户要求"STOP 一到就停"】同步步在缓存命令之后、广播触发
 *          之前有一段静默 / 延时（`emmSeqSyncSettleGap()` + `vTaskDelay(1000)`），
 *          这段期间序列任务**不轮询状态**。若此时收到 STOP / 中止，两条 `SYNC=1`
 *          缓存命令仍会在广播时启动 → 电机白动一下、再被 `EmmTaskSeq_Stop()` 的
 *          广播停掉。这里在广播前查一次状态，已中止就直接放弃广播。
 * @note 只挡 STOP / 中止（`!= RUNNING && != PAUSED`）；PAUSE 在该窗口内的行为
 *       **保持不变**（另议）。
 * @param label 本步中文说明（日志用）。
 * @return true 可以继续广播；false 序列已被中止，调用方应放弃本步。
 */
static bool emmSeqAbortGuard(const char *label)
{
    const SeqState st = EmmTaskQueue_GetSeqState();
    if (st != RUNNING && st != PAUSED) {
        Serial.printf("[任务序列] %s：序列已被中止，取消 SYNC 广播（缓存命令不触发）\n",
                      label);
        return false;
    }
    return true;
}

/**
 * @brief 执行一步"2 台电机同步定位"（下发 + 广播 + 等待两台都到位）。
 * @details 用于取样位的 2+3 号：用户说明"2 号旋转会带动丝杆升降"，因此 3 号必须
 *          同向转同样的脉冲做补偿，且两台必须**同时启动**，否则电离头高度会先被
 *          拉偏再被纠正。做法与实机验证过的命令一致：两台都以 `SYNC=1` 缓存，
 *          再广播 `SYNC A=ALL` 一起启动。
 * @param targetA 2 号（旋转）的绝对目标。
 * @param targetB 3 号（升降）的绝对目标 —— 与 `targetA` 取相同数值，因为 2 号转多少
 *                3 号就要反向补偿多少；两个参数分开传是为了把这段机械耦合关系写清楚。
 * @note **当前流程未使用（2026/10/14 起）**：取样位已改为"3 号先绝对下降、2 号再旋转"
 *       的两步顺序执行，不再做同步补偿（用户确认）。本函数按用户要求**保留备用**，
 *       当前无调用点，会产生 `-Wunused-function` 编译告警（不影响构建）。
 */
static bool emmSeqMoveSyncStep(uint8_t addrA, uint8_t addrB, uint8_t dir,
                               uint16_t rpm, uint8_t acc,
                               uint32_t targetA, uint32_t targetB,
                               const char *label)
{
    LOGD("[任务序列] %s：A=%u→%lu + A=%u→%lu 同步定位 DIR=%u RPM=%u ACC=%u\n",
                  label, (unsigned)addrA, (unsigned long)targetA,
                  (unsigned)addrB, (unsigned long)targetB,
                  (unsigned)dir, (unsigned)rpm, (unsigned)acc);

    emmSeqClearMotions();
    emmSeqRecordMotion(addrA, dir, rpm, acc, targetA, false, 0U, label);
    emmSeqRecordMotion(addrB, dir, rpm, acc, targetB, false, 0U, label);

    if (!emmSeqPositionSend(addrA, dir, rpm, acc, targetA, 1U, true, label)) {
        return false;
    }
    emmSeqSyncSettleGap(label);   /* 先让 A 号把 SYNC=1 缓存命令落盘 */
    if (!emmSeqPositionSend(addrB, dir, rpm, acc, targetB, 1U, true, label)) {
        return false;
    }
    emmSeqSyncSettleGap(label);   /* 再让 B 号落盘，然后才广播触发 */
    vTaskDelay(pdMS_TO_TICKS(1000));
    /* 广播前查一次状态：静默/延时期间若收到 STOP/中止，缓存命令不能再触发。 */
    if (!emmSeqAbortGuard(label)) {
        return false;
    }
    if (!emmSeqSyncBroadcast(label)) {
        return false;
    }
    return emmSeqWaitArriveAll(rpm, acc, label);
}

/**
 * @brief 执行一步"2 台电机同步单圈回零"（缓存 + 广播 + 等待两台都完成）。
 * @details 与实机验证过的复位命令一致：两台先写回零参数，再各自以 `SYNC=1`
 *          缓存 `HOME MODE=1`，最后广播 `SYNC A=ALL` 一起启动。
 * @note **当前流程未使用（2026/10/14 起）**：复位已精简为"3 号上升 →
 *       2 号回单圈回零原点"（2026/10/19 起第 2 步改为绝对定位），不再做 2+3 号
 *       同步回零（用户确认）。本函数按用户要求**保留备用**，当前无调用点，
 *       会产生 `-Wunused-function` 编译告警（不影响构建）。
 */
static bool emmSeqHomeSyncStep(uint8_t addrA, uint8_t addrB, uint8_t homeMode,
                               const char *label)
{
    LOGD("[任务序列] %s：A=%u + A=%u 同步单圈回零 MODE=%u\n",
                  label, (unsigned)addrA, (unsigned)addrB, (unsigned)homeMode);

    emmSeqClearMotions();
    emmSeqRecordMotion(addrA, 1U, 0U, 0U, EMM_SEQ_ROT_POS_HOME, true, homeMode, label);
    emmSeqRecordMotion(addrB, 1U, 0U, 0U, EMM_SEQ_ROT_POS_HOME, true, homeMode, label);

    if (!emmSeqHomeSend(addrA, homeMode, true, label)) {
        return false;
    }
    emmSeqSyncSettleGap(label);   /* 先让 A 号把 SYNC=1 缓存命令落盘 */
    if (!emmSeqHomeSend(addrB, homeMode, true, label)) {
        return false;
    }
    emmSeqSyncSettleGap(label);   /* 再让 B 号落盘，然后才广播触发 */
    /* 广播前查一次状态：静默/延时期间若收到 STOP/中止，缓存命令不能再触发。 */
    if (!emmSeqAbortGuard(label)) {
        return false;
    }
    if (!emmSeqSyncBroadcast(label)) {
        return false;
    }
    return emmSeqWaitHomingAll(label);
}

/**
 * @brief 执行一步"单台电机单圈回零"（下发 + 等待完成）。
 * @details 2026/10/10 新增，供复位阶段的 4 号样品盘使用（用户要求"4 号也要就近
 *          单圈回零，且先执行"）。与同步版 `emmSeqHomeSyncStep` 的区别：只有 1 台，
 *          不需要 `SYNC` 缓存 / 广播，直接下发 `HOME` 后轮询 `0x3B` 等完成。
 *          暂停/恢复同样支持：暂停时停本台并等停稳，恢复时按 `s_curMotions` 重发
 *          `HOME`（沿用 `m.homeMode`）。
 * @param addr     电机地址。
 * @param homeMode 回零模式（协议 0~5，见手册 5.4.2 / 附录 A.1）：
 *                   0 = 单圈**就近**回零（自动选最近方向）
 *                   1 = 单圈**方向**回零（按回零参数里的 DIR 走）
 *                   2 = 无限位碰撞回零   ← 整机回零用的就是本模式
 *                   3 = 有限位碰撞回零
 *                   4 = 回到绝对位置零点
 *                   5 = 回到掉电位置
 *                 ⚠️ 0/1 回到的都是"**单圈零点**"（由 `0x93/0x88` 设定的那个点），
 *                   与整机回零碰撞出来的"**绝对零点**"不是同一个概念 ——
 *                   这正是复位阶段改用绝对定位（`emmSeqRotaryReturnHome()`）的原因。
 * @param label    本步中文说明（日志用，必须指向静态存储）。
 * @return true 回零完成；false 失败或序列被中止。
 * @note **当前无调用点（2026/10/19 起）**：复位阶段 2 号已改为
 *       `emmSeqRotaryReturnHome()`（绝对定位到 0 + 读回核对，方案 C）。
 *       本函数按既有惯例**保留备用**，会产生 `-Wunused-function` 编译告警
 *       （与 `emmSeqWriteHomeParam` / `emmSeqZeroPosition` 情况一致，不影响构建）。
 */
static bool emmSeqHomeStep(uint8_t addr, uint8_t homeMode, const char *label)
{
    LOGD("[任务序列] %s：A=%u 单圈回零 MODE=%u\n",
                  label, (unsigned)addr, (unsigned)homeMode);

    emmSeqClearMotions();
    emmSeqRecordMotion(addr, 1U, 0U, 0U, EMM_SEQ_ROT_POS_HOME, true, homeMode, label);

    if (!emmSeqHomeSend(addr, homeMode, false /* 单台，不缓存 */, label)) {
        return false;
    }
    return emmSeqWaitHomingAll(label);
}

/**
 * @brief 复位阶段：2 号（旋转）**绝对定位**回单圈回零原点，并读回核对。
 * @details 2026/10/19 新增，取代原先的 `emmSeqHomeStep(A=2, MODE=1)`。
 *
 *          **为什么不用 `MODE=1` 单圈回零**（用户 2026/10/19 选定方案 C）：
 *            1. `MODE=1` 回到的是"**单圈零点**"（由 `0x93/0x88` 设定的那个点），
 *               而整机回零建立的是"**绝对零点**"（`MODE=2` 碰撞回零的碰撞点）。
 *               两者不是同一个概念，一旦 `0x93` 在别处被改写就会分叉；
 *            2. 绝对定位到 `EMM_SEQ_ROT_POS_HOME`（= 0）与整机回零、与序列里
 *               其余所有 2 号动作**共用同一个基准**，不会出现"两套零点"；
 *            3. 定位完成后可以**读回 `0x43` 核对**，而 `HOME` 只能听驱动器报"完成"。
 *
 *          **方向**：**固定 `DIR=1`（逆时针）** —— 用户 2026/10/19 明确
 *          "回零方向始终是逆时针，= 1"。不再按 `delta` 符号推导
 *          （原先写法是 `DIR = (delta > 0) ? 1 : 0`，与用户约定不符，已改）。
 *          本工程约定 `DIR=0` 顺时针 / `DIR=1` 逆时针（§7.17）。
 *          `delta` 仍然要用，但只用于判断"**是否已经在原点**"，不再决定方向。
 *          已在容差内则**不动**，避免空跑。
 * @return true 已回到原点（含"本来就在原点"）；
 *         false 读位置失败 / 定位失败 / 回零后核对超差。
 */
static bool emmSeqRotaryReturnHome(void)
{
    /* ① 读当前位置：用于判断"本来就在原点"。 */
    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] 【复位】2 号：RS485 总线繁忙，无法读取当前位置");
        return false;
    }
    EmmV5::SystemState state = {};
    const EmmV5::Status readStatus =
        gEmm.readSystemState(EMM_ROTARY_MOTOR, state, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (readStatus != EmmV5::STATUS_OK) {
        Serial.printf("[任务序列] 【复位】2 号：读取当前位置失败，状态=%s\n",
                      EmmV5::statusToString(readStatus));
        return false;
    }

    /* 单位统一：`EMM_SEQ_ROT_POS_HOME` 是命令脉冲，`curPos` 是驱动器计数（65536/圈），
       相差 20.48 倍，必须先换算再相减（与 `emmSeqRunTrayPos` 同一处教训）。 */
    const int32_t curPos = state.realtimePosition;
    const int32_t dstCounts = emmJudgePulseToCounts(EMM_SEQ_ROT_POS_HOME);
    const int32_t delta = dstCounts - curPos;

    if (delta <= EMM_SEQ_ROT_RETURN_TOL_COUNTS &&
        delta >= -EMM_SEQ_ROT_RETURN_TOL_COUNTS) {
        LOGD("[任务序列] 【复位】2 号已在单圈回零原点（当前位置 %ld），无需转动\n",
                      (long)curPos);
        return true;
    }

    /* ② 绝对定位到原点（MODE=1）。方向固定逆时针 `DIR=1`（用户 2026/10/19：
       "回零方向始终是逆时针，= 1"），不再由 delta 符号推导。 */
    const uint8_t dir = (uint8_t)EMM_SEQ_ROT_RETURN_DIR;
    LOGD("[任务序列] 【复位】2 号回单圈回零原点：当前位置 %ld → 绝对脉冲 %lu，DIR=%u（逆时针）\n",
                  (long)curPos, (unsigned long)EMM_SEQ_ROT_POS_HOME, (unsigned)dir);
    if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, dir, EMM_SEQ_ROT_RETURN_RPM,
                        EMM_SEQ_ROT_RETURN_ACC, EMM_SEQ_ROT_POS_HOME,
                        1U /* MODE=1 绝对 */, EMM_SEQ_ROT_POS_HOME,
                        "复位 2/3 2号回单圈回零原点")) {
        return false;
    }

    /* ③ 读回核对：`emmSeqMoveStep` 只保证"已停稳 + Prf_TF"，这里再确认真的落在 0 附近。 */
    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] 【复位】2 号：RS485 总线繁忙，回零后核对未执行");
        return false;
    }
    EmmV5::SystemState verify = {};
    const EmmV5::Status verifyStatus =
        gEmm.readSystemState(EMM_ROTARY_MOTOR, verify, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (verifyStatus != EmmV5::STATUS_OK) {
        Serial.printf("[任务序列] 【复位】2 号：回零后读取位置失败，状态=%s\n",
                      EmmV5::statusToString(verifyStatus));
        return false;
    }

    const int32_t finalPos = verify.realtimePosition;
    if (finalPos > EMM_SEQ_ROT_RETURN_TOL_COUNTS ||
        finalPos < -EMM_SEQ_ROT_RETURN_TOL_COUNTS) {
        Serial.printf("[任务序列] 【复位】2 号：回零后位置超差，当前位置 %ld（容差 ±%ld）\n",
                      (long)finalPos, (long)EMM_SEQ_ROT_RETURN_TOL_COUNTS);
        return false;
    }

    LOGD("[任务序列] 【复位】2 号：已回到单圈回零原点（核对位置 %ld）\n",
                  (long)finalPos);
    return true;
}

/**
 * @brief 写回零参数（0x4C/0xAE）。
 * @details 用户给的复位命令把 8 个字段全部写全了（`MODE/DIR/RPM/TIMEOUT/
 *          STALL_RPM/STALL_MA/STALL_MS/AUTO`），所以这里直接整组下发，
 *          不做"读-改-写"。始终 `SAVE=0`（只改运行参数，不写掉电保存区）。
 * @note **当前流程未使用（2026/10/14 起）**：新复位流程不再写回零参数（用户确认）。
 *       本函数按用户要求**保留备用**，当前无调用点，会产生 `-Wunused-function`
 *       编译告警（不影响构建）。
 * @return true 写入成功。
 */
static bool emmSeqWriteHomeParam(uint8_t addr, uint8_t mode, uint8_t dir,
                                 uint16_t rpm, uint32_t timeoutMs,
                                 uint16_t stallRpm, uint16_t stallMa,
                                 uint16_t stallMs, bool autoHome,
                                 const char *label)
{
    LOGD("[任务序列] %s：A=%u 写回零参数 MODE=%u DIR=%u RPM=%u "
                  "TIMEOUT=%lu STALL_RPM=%u STALL_MA=%u STALL_MS=%u AUTO=%u\n",
                  label, (unsigned)addr, (unsigned)mode, (unsigned)dir,
                  (unsigned)rpm, (unsigned long)timeoutMs,
                  (unsigned)stallRpm, (unsigned)stallMa, (unsigned)stallMs,
                  (unsigned)autoHome);

    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] RS485 总线繁忙，回零参数未写入");
        return false;
    }
    const EmmV5::Status status = gEmm.modifyHomeParams(
        addr, false /* SAVE=0：只改运行参数 */, mode, dir, rpm, timeoutMs,
        stallRpm, stallMa, stallMs, autoHome, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();

    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
        Serial.printf("[任务序列] %s：写入回零参数异常，状态=%s\n",
                      label, EmmV5::statusToString(status));
        return false;
    }
    return true;
}

/**
 * @brief 当前位置清零（0x0A/0x6D）。
 * @details 用户步骤 1 的第一条：`SET EMM ZERO A=3`。
 */
static bool emmSeqZeroPosition(uint8_t addr, const char *label)
{
    LOGD("[任务序列] %s：A=%u 当前位置清零\n", label, (unsigned)addr);

    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] RS485 总线繁忙，位置清零未下发");
        return false;
    }
    const EmmV5::Status status =
        gEmm.resetCurrentPosition(addr, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
        Serial.printf("[任务序列] %s：位置清零异常，状态=%s\n",
                      label, EmmV5::statusToString(status));
        return false;
    }
    return true;
}

/**
 * @brief 把当前位置设为单圈回零零点（0x93/0x88，手册 5.4.1）。
 * @details 用户步骤 1 的第 2、3 条：`SET EMM ORIGIN ZERO A=2/3 SAVE=0`。
 * @note **当前无调用点（2026/10/19 起）**：前导阶段已改为不重设零点（见
 *       `emmSeqRunPreamble()` 的说明）。本函数按既有惯例**保留备用**，会产生
 *       `-Wunused-function` 编译告警（与 `emmSeqWriteHomeParam` 情况一致，不影响构建）。
 *       ⚠️ 复用本函数前请先想清楚：它的语义是"把**当前位置**当零点"，属于自证式
 *       标定；2 号若不在原点，会把错误位置固化成新零点。
 */
static bool emmSeqSetOriginZero(uint8_t addr, bool save, const char *label)
{
    LOGD("[任务序列] %s：A=%u 设为单圈回零零点 SAVE=%u\n",
                  label, (unsigned)addr, (unsigned)save);

    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] RS485 总线繁忙，单圈零点未设置");
        return false;
    }
    const EmmV5::Status status =
        gEmm.setOriginZero(addr, save, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
        Serial.printf("[任务序列] %s：设置单圈零点异常，状态=%s\n",
                      label, EmmV5::statusToString(status));
        return false;
    }
    return true;
}

/**
 * @brief 广播同步触发（0xFF/0x66）：让此前以 `SYNC=1` 缓存的命令一起启动。
 * @details 触发后**先静默 `EMM_TASK_SEQ_SYNC_SETTLE_MS` 再返回**，理由见该宏：
 *          触发帧是裸帧，发完即返回，上层若立刻发 `0x43` 读状态取基准，可能
 *          打断还没起步的运动（实机症状：目标从未设置、电机全程不动）。
 */
static bool emmSeqSyncBroadcast(const char *label)
{
    LOGD("[任务序列] %s：广播同步触发（A=ALL）\n", label);

    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] RS485 总线繁忙，同步触发未下发");
        return false;
    }
    const EmmV5::Status status = gEmm.synchronousMotionBroadcast();
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
        Serial.printf("[任务序列] %s：同步触发异常，状态=%s\n",
                      label, EmmV5::statusToString(status));
        return false;
    }

    /* 触发后静默一段，给驱动器留出"真正开始执行缓存命令"的时间。 */
    LOGD("[任务序列] %s：触发后静默 %u ms 再轮询\n",
                  label, (unsigned)EMM_TASK_SEQ_SYNC_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_SYNC_SETTLE_MS));
    return true;
}

/**
 * @brief 占位步骤：喷头识别（用户要求"下一阶段实现"）。
 * @note 用户给的机械流程里，取样位（进入下降前、抬回安全高度后）与回收位都有
 *       "喷头识别"。本阶段全部只打印一行占位日志，不产生任何总线事务。
 */
static void emmSeqPlaceholderNozzleCheck(const char *label)
{
    LOGD("[任务序列] %s：【喷头识别：占位，下一阶段实现】\n", label);
}

/**
 * @brief 占位步骤：受阻检测布防/撤防（用户要求"暂不接"）。
 * @note 现有 `EmmLiftCurrentMonitor`（1 ms 轮询相电流 / 460 mA / 20 ms 确认）
 *       保持不动，仍可由上位机手动 `SET EMM LIFT MONITOR ARM=1` 布防；
 *       序列侧按用户要求只打印占位日志，不做任何布防动作。
 */
static void emmSeqPlaceholderLiftMonitor(const char *label, bool arm)
{
    LOGD("[任务序列] %s：【受阻检测%s：占位，暂不接】\n",
                  label, arm ? "布防" : "撤防");
}

/* ==========================================================================
 * 第三步：各阶段（TaskPhase）的真实动作
 * --------------------------------------------------------------------------
 * 每个阶段处理器都返回 `SeqStepResult`：
 *   DONE    → 调用方推进到下一阶段；
 *   RUNNING → 还没做完（只有需要跨轮询周期的子步骤才会返回它）；
 *   FAILED  → 失败或序列已被中止。
 * 阻塞式阶段（前导/转盘/检测/回收/复位）在函数内把动作全部做完再返回 DONE。
 * ========================================================================== */

/**
 * @brief 序列前导（`TASK_PREAMBLE`）：**已改为不做任何电机动作**。
 * @details 2026/10/19 修订（用户确认"现场总是先回零再启序列"）。
 *
 *          **修订前的做法**：`SEQ START` 后对 2 号发一次
 *            `SET EMM ORIGIN ZERO A=2 SAVE=0`（`0x93/0x88`，手册 5.4.1），
 *          把"**2 号此刻所在位置**"设为单圈回零零点。
 *
 *          **为什么删掉**：该命令把"当前位置"当成零点，属于**自证式**标定 ——
 *          2 号若不在原点（上一次序列被 `SEQ STOP` / 断电 / 某步失败中止，停在
 *          385 或半路；或现场手动盘动过），那个**错误位置**就会被固化成新零点，
 *          之后 1120 / 2970 / 450 / 385 全部整体偏移，收尾的"回原点"也只是回到
 *          这个错误点。零点必须由**唯一权威来源**建立，不能由序列自己重定义。
 *
 *          **现在的做法**：零点只由**整机回零**（上电初始化 → `EmmSystemHome`）
 *          建立 —— 2 号用 `MODE=2` 无限位碰撞回零，碰撞点即绝对 0。序列里 2 号
 *          的所有动作（含收尾回原点 `emmSeqRotaryReturnHome()`）都按**绝对位置**
 *          走，与整机回零共用同一基准。
 *          前提：`SEQ START` 之前整机回零已完成（用户 2026/10/19 确认现场总是如此）。
 *
 *          3 号的"位置清零 / 设单圈零点"两步在 2026/10/14 就已被注释掉，本次一并
 *          清除残留注释，保持前导阶段**无副作用**。
 * @return 恒 DONE（本阶段不再有可失败的电机动作）。
 */
static SeqStepResult emmSeqRunPreamble(void)
{
    LOGD("[任务序列] 【前导】跳过：2/3 号零点由整机回零建立，本序列不再重设（2026/10/19 起）\n");
    return SEQ_STEP_DONE;
}

/**
 * @brief Ⅱ 取样位（`TASK_MOVE_POS`）。
 * @details **2026/10/14 新流程（用户要求，取代原"2+3 号同步"方案）**：
 *          回零完成后 3 号零点已标定，取样动作改为**两步顺序执行**：
 *            1. 3 号（升降）**绝对下降**到起始下降位 5000：
 *               `SET EMM MOVE A=3 DIR=1 RPM=180 ACC=40 PULSE=5000 MODE=1 SYNC=0`
 *            2. 2 号（旋转）转到取样位正上方：
 *               `SET EMM MOVE A=2 DIR=0 RPM=15 ACC=10 PULSE=<取样位> MODE=1 SYNC=0`
 *               （取样位**运行期可改**，出厂默认 `985`；旧注释写的 `1070` 已过时）
 *          其后 3~8 步与旧流程**完全一致**：
 *            3. 3 号快速下降至取样高度 1（离针孔约 50 mm）
 *            4. 开启受阻检测（**占位**）
 *            5. 3 号慢速下降至取样高度 2（受阻检测区段）
 *            6. 撤销受阻检测（**占位**）
 *            7. 3 号抬回安全高度
 *            8. 喷头识别（**占位**）
 *
 *          **同步补偿已取消（用户 2026/10/14 确认）**：原第 1 步之所以做成"2 号
 *          旋转 + 3 号同向同量补偿"的同步步，是因为"2 号旋转会带动丝杆升降、
 *          3 号必须一起动才能把电离头保持在安全高度"。新流程改为 3 号先动、2 号
 *          后动，**不再做补偿**（用户确认按新流程顺序执行）。
 *
 *          **下降到取样点后不再等待**（2026/10/14 用户要求）：原"停留 T 秒"已删除，
 *          停留改到 `TASK_DETECT_POS`（检测位）。因此本阶段所有子步骤都是阻塞式
 *          完成，执行完立刻 `emmSeqNextStep()` 进入下一个。
 * @return RUNNING 继续；DONE 完成；FAILED 失败或序列被中止。
 */
static SeqStepResult emmSeqRunMovePos(void)
{
    switch (s_subStep) {
    /* ---- 子步骤 0：3 号绝对下降到起始下降位（5000） ---- */
    case 0:
        /*
         * 2026/10/14 新流程第 1 条：回零后 3 号零点已标定，先把它**绝对下降**到
         * 起始下降位 5000，建立高度基准；随后 2 号再转到取样位正上方。
         * 取代原先"2+3 号同步转到取样位"的同步步（用户确认不再同步补偿）。
         */
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 120U, 30U,
                            EMM_SEQ_LIFT_POS_SAMPLE_START, 1U,
                            EMM_SEQ_LIFT_POS_SAMPLE_START,
                            "取样位 1/9 3号下降至起始位(5000)")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 1：2 号旋转到取样位正上方 ---- */
    case 1:
    {
        /* 取样位是**运行期可改**点位（SET EMM POINT SAMPLE=…），不能再用宏。 */
        const uint32_t samplePos = EmmSeqPoints_Get(EMM_SEQ_POINT_SAMPLE);
        if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 15U, 10U,
                            samplePos, 1U, samplePos,
                            "取样位 2/9 2号旋转至取样位正上方")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;
    }

    /* ---- 子步骤 2：3 号快速下降至取样高度 1 ---- */
    case 2:
    {
        /* 取样高度 1 是**运行期可改**点位（`SET EMM POINT LIFT_S1=…`）。 */
        const uint32_t sample1Pos = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_SAMPLE_1);
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
                            sample1Pos, 1U,
                            sample1Pos,
                            "取样位 3/9 快降至取样高度1")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;
    }

    /* ---- 子步骤 3：开启受阻检测（占位） ---- */
    case 3:
        emmSeqPlaceholderLiftMonitor("取样位 4/9", true);
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 4：3 号慢速下降至取样高度 2 ---- */
    case 4:
    {
        /* 取样高度 2 是**运行期可改**点位（`SET EMM POINT LIFT_S2=…`）。 */
        const uint32_t sample2Pos = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_SAMPLE_2);
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 10U, 10U,
                            sample2Pos, 1U,
                            sample2Pos,
                            "取样位 5/9 慢降至取样高度2")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;
    }

    /* ---- 子步骤 5：撤销受阻检测（占位） ---- */
    case 5:
        emmSeqPlaceholderLiftMonitor("取样位 6/9", false);
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 6：3 号抬回安全高度 ---- */
    case 6:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
                            EMM_SEQ_LIFT_POS_SAFE, 1U, EMM_SEQ_LIFT_POS_SAFE,
                            "取样位 7/9 抬回安全高度")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 7：喷头识别（占位） ---- */
    case 7:
        emmSeqPlaceholderNozzleCheck("取样位 8/9");
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 8：本阶段完成 ---- */
    case 8:
    default:
        LOGD("[任务序列] 取样位 9/9 完成\n");
        return SEQ_STEP_DONE;
    }
}

/**
 * @brief 样品盘定位（`TASK_TRAY_POS`）。
 * @details **2026/10/10 语义变更（方案 A，用户批准）**：原实现是"给**下一条任务**
 *          预置工位"（`idx+1`），要求"任务 k 取样前盘子已在任务 k 工位"这条不变式
 *          由**上一条任务**保证。中途 `SEQ ADD` 新任务会掐断这条链：最后一条任务跑
 *          本步时队列里还没有新任务 → 打"已是最后一条任务，无需转盘"跳过；而新任务
 *          自己的本步又只负责"再下一条"（此时它也是最后一条，同样跳过）→ **没有任何
 *          一步把盘子转到新任务的工位** → 新任务在错误的工位上取样。
 *          现改为**给当前任务自己定位**，并挪到 `TASK_MOVE_POS`（取样）**之前**执行
 *          （用户确认：任务开头头部在安全高度，转盘安全）。这样每条任务自包含，
 *          中途加任务天然成立；顺带补上了"第 1 条任务原本也没人给它转盘"的空档。
 *
 *          样品盘用 **`MODE=1` 绝对位置模式**：工位 N ↔ 绝对脉冲 `(N−1)×320`
 *          （01 号 = 0、02 号 = 320、… 10 号 = 2880）。因此位置永远与驱动器
 *          零点同坐标系，"暂停恢复重发"也天然成立。
 *
 *          **当前工位不靠软件计数**（用户确认）：样品盘只在**整机回零**时回零一次，
 *          零点（01 号工位 = 绝对脉冲 0）建立后一直有效，因此本步直接**读驱动器
 *          的实时绝对位置**当基准，再按"目标绝对脉冲 > 当前位置 → DIR=1（逆时针），
 *          否则 DIR=0（顺时针）"选方向（用户确认反向就是顺时针）。这样跨序列、
 *          跨任意工位顺序都自洽，也不怕中途有人手动动过样品盘。
 * @param t 当前任务（用 `t.slot` 作为目标工位）。
 * @return DONE 完成（含"已在工位就不动"）；FAILED 失败或序列被中止。
 */
static SeqStepResult emmSeqRunTrayPos(const EmmTask &t)
{
    const uint8_t targetSlot = t.slot;

    /* 兜底：队列入口已经校验过 1~10，这里再挡一道，防止非法工位号算出越界脉冲。 */
    if (targetSlot < EMM_TASK_SLOT_MIN || targetSlot > EMM_SEQ_TRAY_SLOT_COUNT) {
        Serial.printf("[任务序列] 样品盘：本任务的工位号 %u 非法（应在 %u~%u）\n",
                      (unsigned)targetSlot, (unsigned)EMM_TASK_SLOT_MIN,
                      (unsigned)EMM_SEQ_TRAY_SLOT_COUNT);
        return SEQ_STEP_FAILED;
    }

    const uint32_t dstAbs = (uint32_t)(targetSlot - 1U) * EMM_SEQ_TRAY_PULSE_PER_SLOT;

    /* ---- 读驱动器实时绝对位置当基准（整机回零建立的零点坐标系） ---- */
    if (!Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        Serial.println("[任务序列] 样品盘：RS485 总线繁忙，无法读取当前位置");
        return SEQ_STEP_FAILED;
    }
    EmmV5::SystemState state = {};
    const EmmV5::Status status =
        gEmm.readSystemState(EMM_SAMPLE_TRAY_MOTOR, state, EMM_TASK_SEQ_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK) {
        Serial.printf("[任务序列] 样品盘：读取当前位置失败，状态=%s\n",
                      EmmV5::statusToString(status));
        return SEQ_STEP_FAILED;
    }
    const int32_t curPos = state.realtimePosition;

    /*
     * 已在目标工位（容差内）就不动，避免空转一圈。
     *
     * 单位必须统一：`dstAbs` 是**命令脉冲**（`(工位-1) × 320`），而 `curPos` 是 `0x43`
     * 读回的**驱动器计数**（65536/圈），两者相差 20.48 倍。因此先把目标脉冲换算成
     * 计数（`emmJudgePulseToCounts`）再比 —— 否则"已在工位"永远判不出来、每次都白跑
     * 一次 MOVE（2026/10/10 修正，原写法直接拿脉冲减计数）。
     */
    const int32_t dstCounts = emmJudgePulseToCounts(dstAbs);
    const int32_t delta = dstCounts - curPos;
    if (delta <= EMM_JUDGE_MOVE_TOL_COUNTS &&
        delta >= -EMM_JUDGE_MOVE_TOL_COUNTS) {
        LOGD("[任务序列] 样品盘：本任务已在 %02u 号工位（当前位置 %ld），无需转动\n",
                      (unsigned)targetSlot, (long)curPos);
        return SEQ_STEP_DONE;
    }

    const uint8_t dir = (delta > 0) ? 1U : 0U;
    LOGD("[任务序列] 样品盘：定位到本任务工位 %02u（当前位置 %ld，"
                  "绝对脉冲 → %lu，DIR=%u）\n",
                  (unsigned)targetSlot, (long)curPos,
                  (unsigned long)dstAbs, (unsigned)dir);

    if (!emmSeqMoveStep(EMM_SAMPLE_TRAY_MOTOR, dir, EMM_SEQ_TRAY_RPM,
                        EMM_SEQ_TRAY_ACC, dstAbs, 1U /* MODE=1 绝对 */,
                        dstAbs, "样品盘定位")) {
        return SEQ_STEP_FAILED;
    }

    LOGD("[任务序列] 样品盘：已到 %02u 号工位\n", (unsigned)targetSlot);
    return SEQ_STEP_DONE;
}

/**
 * @brief Ⅲ 检测位（`TASK_DETECT_POS`）。
 * @details 对应实机验证过的命令组 10（**2026/10/14 起加入"停留 T 分钟"**）：
 *            2 号旋转至检测位（2850）→ 3 号降至检测高度
 *            （`<检测高度>`，出厂默认 26000；旧注释写的 30000 已过时）
 *            → **停留 T 分钟** → 3 号抬回安全高度2（5000）。
 *          停留位置由用户确认：**3 号降到检测位后、抬回安全高度 2 之前**。
 *          "停留 T 分钟"要跨轮询周期，因此本阶段改用 `s_subStep` 子步骤状态机；
 *          其余子步骤都是阻塞式完成，执行完立刻 `emmSeqNextStep()` 进入下一个。
 *          计时用累计法（见 `s_stepElapsedMs`）：暂停期间不累计，PAUSE 不会把
 *          剩余停留时间吃掉；进度日志每 `EMM_TASK_SEQ_DWELL_PROGRESS_MS`（1 分钟）
 *          打印一条。
 * @param t 当前任务（用 `t.timeSec` 作为停留**分钟**数）。
 * @return RUNNING 继续；DONE 完成；FAILED 失败或序列被中止。
 */
static SeqStepResult emmSeqRunDetectPos(const EmmTask &t)
{
    switch (s_subStep) {
    /* ---- 子步骤 0：2 号旋转至检测位 ---- */
    case 0:
    {
        /* 检测位是**运行期可改**点位（SET EMM POINT DETECT=…）。 */
        const uint32_t detectPos = EmmSeqPoints_Get(EMM_SEQ_POINT_DETECT);
        if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 10U, 10U, detectPos,
                            1U, detectPos, "检测位 1/4 旋转至检测位")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;
    }

    /* ---- 子步骤 1：3 号下降至检测高度 ---- */
    case 1:
    {
        /* 检测高度是**运行期可改**点位（`SET EMM POINT LIFT_DET=…`）。 */
        const uint32_t liftDetectPos = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DETECT);
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 30U, liftDetectPos,
                            1U, liftDetectPos, "检测位 2/4 下降至检测位")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;
    }

    /* ---- 子步骤 2：停留 T 分钟（跨轮询周期，计时不含暂停时间） ---- */
    case 2:
    {
        /*
         * 先把 T（分钟）钳到上限，再换算成 ms —— 避免 T 极大时 `T × 60000`
         * 溢出 `uint32_t`（原实现是"先乘后判"，存在溢出风险，此处已修正）。
         */
        float dwellMin = t.timeSec;
        if (dwellMin > EMM_TASK_SEQ_DWELL_MAX_MIN) {
            Serial.printf("[任务序列] 检测位 3/4 停留 T=%.3f min 超过上限，按 %.1f min 处理\n",
                          (double)t.timeSec, (double)EMM_TASK_SEQ_DWELL_MAX_MIN);
            dwellMin = EMM_TASK_SEQ_DWELL_MAX_MIN;
        }
        const uint32_t dwellMs = (uint32_t)(dwellMin * 60000.0f);

        if (s_stepElapsedMs == 0U) {
            LOGD("[任务序列] 检测位 3/4 开始停留 %.3f min（原始 T=%.3f min）\n",
                          (double)dwellMin, (double)t.timeSec);
            /*
             * **2026/10/21 新增 `DWELL BEGIN`(332)**：探头已在上一子步骤降到检测位、
             * 这一刻开始计时 —— 上位机收到就开 CA、开始采集。
             * 只在 `s_stepElapsedMs == 0`（本子步骤刚进入）发一次；
             * 暂停恢复不会重发（暂停期间计时冻结，elapsed 不会回到 0）。
             * `T=` 用**钳位后**的 `dwellMin`（超过 30 min 上限时按 30.000 发）。
             */
            emmSeqEventDwellBegin(dwellMin);
        }

        if (s_stepElapsedMs < dwellMs) {
            /* 每 EMM_TASK_SEQ_DWELL_PROGRESS_MS（1 分钟）打一条进度，避免刷屏。 */
            if (s_stepElapsedMs >= s_nextProgressMs) {
                LOGD("[任务序列] 检测位 3/4 停留中：%.1f/%.1f min\n",
                              (double)(s_stepElapsedMs / 60000.0f),
                              (double)(dwellMs / 60000.0f));
                s_nextProgressMs = s_stepElapsedMs + EMM_TASK_SEQ_DWELL_PROGRESS_MS;
            }
            return SEQ_STEP_RUNNING;
        }

        LOGD("[任务序列] 检测位 3/4 停留结束（%.3f min）\n",
                      (double)(dwellMs / 60000.0f));
        /*
         * **2026/10/21 新增 `DWELL END`(333)**：计时已到、3 号还没抬起 ——
         * 上位机收到就停采集、关 CA。必须发在 `emmSeqNextStep()` **之前**，
         * 否则下一个子步骤（抬起）会先跑起来，采集窗口被切短。
         */
        emmSeqEventDwellEnd();
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;
    }

    /* ---- 子步骤 3：3 号抬回安全高度2 ---- */
    case 3:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 30U, EMM_SEQ_LIFT_POS_SAFE_2,
                            1U, EMM_SEQ_LIFT_POS_SAFE_2, "检测位 4/4 抬回安全高度2")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 4：本阶段完成 ---- */
    case 4:
    default:
        LOGD("[任务序列] 检测位 完成\n");
        return SEQ_STEP_DONE;
    }
}

/**
 * @brief Ⅳ 回收位 / 抛弃位（`TASK_DISCARD_POS`）。
 * @details 对应实机验证过的命令组 11（2026/10/10 用户改：2 号两段旋转方向由逆时针
 *          改为**顺时针** `DIR=0`）：
 *            3 号先降 1000（**该步已注释停用**）→ 2 号顺时针转到回收位1 → 喷头识别（**占位**）
 *            → 3 号降至回收位2 → 受阻检测（**占位**）→ 2 号顺时针转到回收位2
 *            → 撤销受阻检测（**占位**）→ 3 号回升至回收位3 → 3 号抬回安全高度
 *            → 喷头识别（**占位**，用户给的机械流程在此处有一次）。
 * @note **2026/10/20**：本阶段用到的 4 个点位（2 号 DISC1/DISC2、3 号 LIFT_D2/LIFT_D3）
 *       全部改为运行期可改，代码里**不再出现具体脉冲数字**；上面的流程描述也不再
 *       写死数值（旧注释写的 35000/30000 与当时的宏 37000/31000 本就不一致）。
 * @return DONE 完成；FAILED 失败或序列被中止。
 */
static SeqStepResult emmSeqRunDiscardPos(void)
{
    /*
     * **2026/10/26 新增**：从 `END CUR` / `END ALL` 中途切入时，3 号可能还在
     * 取样/检测高度。必须先抬回安全高度，否则下面"旋转至回收位"会带着探头撞盘。
     * 正常路径（检测位 → 回收位）进入前 3 号已在安全高度，标志为 false，不多走这一步。
     * 用**绝对位置**（`MODE=1`）：目标已到位时驱动器直接回"未移动"，代价只有一帧。
     */
    if (s_discardLiftFirst) {
        s_discardLiftFirst = false;
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
                            EMM_SEQ_LIFT_POS_SAFE, 1U,
                            EMM_SEQ_LIFT_POS_SAFE,
                            "回收位 0/10 升降先回安全高度（END 中途切入）")) {
            return SEQ_STEP_FAILED;
        }
    }

    /*
     * 2 号两段：`SET EMM POINT DISC1=…` / `DISC2=…`
     * 3 号两段：`SET EMM POINT LIFT_D2=…` / `LIFT_D3=…`
     * 四个都是**运行期可改**点位，这里**不要**直接写宏。
     */
    const uint32_t discard1Pos = EmmSeqPoints_Get(EMM_SEQ_POINT_DISCARD_1);
    const uint32_t discard2Pos = EmmSeqPoints_Get(EMM_SEQ_POINT_DISCARD_2);
    const uint32_t liftDiscard2Pos = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DISCARD_2);
    const uint32_t liftDiscard3Pos = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DISCARD_3);

    // if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
    //                     EMM_SEQ_LIFT_POS_DISCARD_1, 1U,
    //                     EMM_SEQ_LIFT_POS_DISCARD_1,
    //                     "回收位 1/10 升降预备")) {
    //     return SEQ_STEP_FAILED;
    // }
    if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 15U, 10U,
                        discard1Pos, 1U,
                        discard1Pos,
                        "回收位 2/10 旋转至回收位")) {
        return SEQ_STEP_FAILED;
    }
    emmSeqPlaceholderNozzleCheck("回收位 3/10");

    if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
                        liftDiscard2Pos, 1U,
                        liftDiscard2Pos,
                        "回收位 4/10 下降至回收位")) {
        return SEQ_STEP_FAILED;
    }
    emmSeqPlaceholderLiftMonitor("回收位 5/10", true);

    if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 15U, 10U,
                        discard2Pos, 1U,
                        discard2Pos,
                        "回收位 6/10 旋转至第二段")) {
        return SEQ_STEP_FAILED;
    }
    emmSeqPlaceholderLiftMonitor("回收位 7/10", false);

    if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 140U, 30U,
                        liftDiscard3Pos, 1U,
                        liftDiscard3Pos,
                        "回收位 8/10 回升至回收位第三段")) {
        return SEQ_STEP_FAILED;
    }
    if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U, EMM_SEQ_LIFT_POS_SAFE,
                        1U, EMM_SEQ_LIFT_POS_SAFE,
                        "回收位 9/10 抬回安全高度")) {
        return SEQ_STEP_FAILED;
    }
    emmSeqPlaceholderNozzleCheck("回收位 10/10");

    return SEQ_STEP_DONE;
}

/**
 * @brief 复位（`TASK_RESET_POS`）：3 号上升 → 2 号**绝对定位**回单圈回零原点 → 4 号样品盘回零。
 * @details **2026/10/19 修订（方案 C，用户选定）**：
 *            1. 3 号（升降）上升至 `EMM_SEQ_LIFT_POS_RESET`（= 2000）：
 *               `SET EMM MOVE A=3 DIR=1 RPM=100 ACC=20 PULSE=2000 MODE=1 SYNC=0`
 *            2. 2 号（旋转）**绝对定位**回单圈回零原点（绝对脉冲 0）并读回核对：
 *               见 `emmSeqRotaryReturnHome()`。
 *            3. 4 号（样品盘）**接近开关回零**：见 `EmmSystemHome_TrayHomeForSeq()`
 *               （2026/10/19 追加，用户要求"任务执行完后不仅 2 号回零，样品盘也要回零"）。
 *
 *          **为什么第 2 步从 `SET EMM HOME A=2 MODE=1` 改成绝对定位**：
 *            - `MODE=1` 回到的是"**单圈零点**"（由 `0x93/0x88` 设定的点），而整机
 *              回零建立的是"**绝对零点**"（`MODE=2` 碰撞回零的碰撞点），两者不是
 *              同一个概念，一旦 `0x93` 在别处被改写就会分叉；
 *            - 绝对定位到 `EMM_SEQ_ROT_POS_HOME`（= 0）与整机回零、与序列里其余
 *              所有 2 号动作共用**同一个基准**，不会出现"两套零点"；
 *            - 定位后可以读回 `0x43` **核对**，而 `HOME` 只能听驱动器报"完成"。
 *
 *          **第 3 步为什么用接近开关标定而不是 `HOME MODE=0`**（用户 2026/10/19 选定）：
 *            - 接近开关标定是**重新校准**，零点由物理开关决定，不受"驱动器单圈零点
 *              被谁改过"影响；`MODE=0` 只是复用上次标定结果，且"就近"方向不确定；
 *            - 与整机回零第 17 步、`SET EMM TRAY HOME` **同一套代码**（`emmHomeTrayHomeCore()`），
 *              不会出现两套零点。
 *            代价是每次都要转一圈找开关，典型 3~8 秒、最坏约 25 秒。
 *
 *          **旧流程已全部取消（2026/10/14 用户明确"完全按我说的"）**：
 *            - 取消 4 号样品盘**就近单圈回零**（注：2026/10/19 加回来的是**接近开关标定**，
 *              不是当年的 `MODE=0`）；
 *            - 取消 2/3 号写回零参数；
 *            - 取消 2+3 号同步单圈回零（`emmSeqHomeSyncStep`）。
 *          上述三条对应的辅助函数（`emmSeqWriteHomeParam` / `emmSeqHomeSyncStep` /
 *          `emmSeqHomeStep`）按既有惯例**保留备用**，当前流程不再调用。
 *
 *          **失败处理（两轴口径不同，均为用户明确要求）**：
 *            - 3 号上升失败：中止并**保留队列**（`return SEQ_STEP_FAILED`，便于排查）；
 *            - 2 号回原点失败：**仍不中止**（2026/10/14 决定），广播停机后继续做第 3 步、
 *              照常收尾、**清空队列**；但**不静默** —— 先发一帧
 *              `EVT 306 ROTARY NOT HOME` 告警；
 *            - 4 号样品盘回零失败：**中止并保留队列**（2026/10/19 决定），
 *              不发 `SEQ ALL DONE`，需人工排查后 `SEQ CLEAR`。
 *
 * @return 3 号上升失败或 4 号回零失败时返回 `FAILED`（保留队列）；
 *         其余情况返回 `DONE`（清空队列）。
 */
static SeqStepResult emmSeqRunReset(void)
{
    /* **2026/10/21**：段首那句中文说明降为 `LOGD`，改用事件帧 `350 RESET BEGIN`。
       原来这一行 + 下面"进入复位阶段" + 段尾"完成：3 号已上升…"三句在反复说同一件事，
       是复位阶段最大的噪音源。 */
    LOGD("[任务序列] 【复位】3 号上升 → 2 号回单圈回零原点 → 4 号样品盘回零"
         "（整条队列只做一次）\n");
    emmSeqEventReset(RESET_BEGIN, "BEGIN");

    /* 复位 1/3：3 号（升降电机）上升至 EMM_SEQ_LIFT_POS_RESET（2000）。
       参数沿用取样位下降的 DIR=1 / RPM=100 / ACC=20（用户确认"方向还是 DIR=1，
       电机会根据脉冲走"）。 */
    if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 100U, 20U,
                        EMM_SEQ_LIFT_POS_RESET, 1U, EMM_SEQ_LIFT_POS_RESET,
                        "复位 1/3 3号上升至2000")) {
        return SEQ_STEP_FAILED;
    }

    /*
     * 复位 2/3：2 号（旋转）绝对定位回单圈回零原点（0）+ 读回核对。
     * 失败**不中止**（用户 2026/10/14 决定；2026/10/19 追加"发失败事件码"）：
     * 先发 `ROTARY_NOT_HOME`(306) 让上位机能识别，再广播停机兜一道（此时电机可能
     * 仍在动），最后继续做第 3 步、照常收尾清空队列。
     */
    if (!emmSeqRotaryReturnHome()) {
        Serial.println("[任务序列] 【复位】2 号未回到单圈回零原点：按用户要求不中止序列，"
                       "发失败事件码后继续第 3 步并照常收尾清空");
        sendWithCRC("EVT %d ROTARY NOT HOME", (int)ROTARY_NOT_HOME);
        if (Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
            const EmmV5::Status stopStatus = gEmm.stopBroadcast(false);
            Rs485Bus_Give();
            Serial.printf("[任务序列] 【复位】已广播停止全部 EMM 电机，状态=%s\n",
                          EmmV5::statusToString(stopStatus));
        } else {
            Serial.println("[任务序列] 【复位】RS485 总线繁忙，停止全部电机的命令未下发");
        }
    } else {
        LOGD("[任务序列] 【复位】2/3 完成：2 号已回到单圈回零原点\n");
        /* 只在"核对通过"之后才报 351：2 号失败时上位机看到 350 → 306 → 352，
           中间缺 351，据此即可判断是哪一步出的问题。 */
        emmSeqEventReset(RESET_AXIS_2, "AXIS 2");
    }

    /*
     * 复位 3/3：4 号（样品盘）接近开关回零（2026/10/19 新增，用户要求
     * "任务执行完后不仅 2 号回零，样品盘也要回零"）。
     *
     * 走 `EmmSystemHome_TrayHomeForSeq()`：与 `SET EMM TRAY HOME`、整机回零第 17 步
     * **同一套标定代码**，只是跳过"序列正在运行"那道互锁（调用方就是序列自己），
     * 并注册中止钩子让 `SEQ STOP` 能立即打断。
     *
     * **失败按"中止并保留队列"处理**（用户 2026/10/19 明确选择）：
     * 与第 1 步一致，`return SEQ_STEP_FAILED` → `emmTaskSeqStepFailed()`，
     * 队列**不清空**，上位机收不到 `SEQ ALL DONE`，便于现场排查后再 `SEQ CLEAR`。
     * 这与"2 号失败仍不中止"的口径不同，是用户对两轴的分别要求。
     */

    /*
     * 前置判据：序列若已在第 1/2 步期间被 `SEQ STOP` / `SET POWER OFF` 中止，
     * 就不要再启动新的机械动作（否则日志会出现"样品盘回零开始"紧跟"收到中止请求"）。
     * 判据与 `emmSeqHandlePauseResume()` 一致：PAUSED 仍视为"可以继续"。
     */
    {
        const SeqState st = EmmTaskQueue_GetSeqState();
        if (st != RUNNING && st != PAUSED) {
            Serial.println("[任务序列] 【复位】序列已被中止，跳过 4 号样品盘回零");
            return SEQ_STEP_FAILED;
        }
    }

    if (!EmmSystemHome_TrayHomeForSeq()) {
        Serial.println("[任务序列] 【复位】3/3 4 号样品盘未回零：按用户要求中止序列并保留队列"
                       "（排查后请手动 SEQ CLEAR）");
        return SEQ_STEP_FAILED;
    }

    emmSeqEventReset(RESET_AXIS_4, "AXIS 4");

    /* 段尾那句"完成：3 号已上升、2 号已回原点、4 号样品盘已回零"降为 `LOGD`：
       紧随其后的 `EVT 300 SEQ ALL DONE` 已经表达了"整条序列结束"。 */
    LOGD("[任务序列] 【复位】完成：3 号已上升、2 号已回原点、4 号样品盘已回零\n");

    /* 恒返回 DONE：让 `emmSeqFinishIfRunning()` → `emmTaskSeqFinishAll()` 一定执行
       → `EmmTaskQueue_Clear()` 一定被调用（用户要求"跑完必清空"）。 */
    return SEQ_STEP_DONE;
}

/**
 * @brief `SEQ RESUME`（重做模式）专用阶段：回取样位上方 → 停 2s → 重新取样。
 * @details **2026/10/20 新增**。用户需求（逐字）：
 *          > 发暂停恢复命令后，不从暂停处恢复执行，而是：暂停恢复命令下发→回到取样位的
 *          > 上方，停个2s→重新取样、检测、抛弃（也就是取样之后还是原来的任务流程），
 *          > 接着执行任务序列最后的复位流程
 *
 *          子步骤（4 步）：
 *            1. 3 号抬到 `EMM_SEQ_LIFT_POS_SAMPLE_START`(5000) —— 取样流程的"起始下降位"，
 *               也是用户选定的"取样位上方"高度；
 *            2. 2 号转到取样位（`SET EMM POINT SAMPLE` 的**运行期值**，不是宏）；
 *            3. 停留 `EMM_TASK_SEQ_RESUME_SETTLE_MS`(2000 ms)；跨轮询周期计时，
 *               暂停期间不计时，`SEQ STOP` 能在下个周期打断；
 *            4. 转入 `TASK_MOVE_POS` 并把 `s_subStep` 置 **2** —— 子步骤 0（3 号降到 5000）
 *               与 1（2 号旋转到取样位）刚在 1、2 里做过，重复执行虽幂等但会多等一次
 *               到位判定，直接跳过更干净。
 *
 *          **重做范围**：本阶段结束后依次走 `TASK_MOVE_POS`(2..8) → `TASK_DETECT_POS`(0..4)
 *          → `TASK_DISCARD_POS` → `Index++`，即"取样、检测、抛弃"完整重做一遍，
 *          然后**继续队列里剩余的任务**，全部跑完再进复位（用户 2026/10/20 选定）。
 *          检测位的停留计时由 `emmTaskSeqEnterPhase()` 清零 `s_stepElapsedMs`
 *          自然实现"重新计时"（用户 2026/10/20 选定）。
 *
 *          **本阶段的失败**：按正常失败处理（广播停机 + 保留队列），与"重做"无关。
 *
 * @return RUNNING 继续；FAILED 某步失败。**不会返回 DONE**（第 4 步直接换阶段）。
 */
static SeqStepResult emmSeqRunRestartSample(void)
{
    switch (s_subStep) {
    /* ---- 1/4：3 号抬到取样位上方（起始下降位 5000） ---- */
    case 0:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 120U, 30U,
                            EMM_SEQ_LIFT_POS_SAMPLE_START, 1U,
                            EMM_SEQ_LIFT_POS_SAMPLE_START,
                            "恢复重做 1/4 3号抬至取样位上方(5000)")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 2/4：2 号转到取样位 ---- */
    case 1:
    {
        /* 取样位是运行期可改点位，必须走 `EmmSeqPoints_Get()` 而不是宏。 */
        const uint32_t samplePos = EmmSeqPoints_Get(EMM_SEQ_POINT_SAMPLE);
        if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 15U, 10U,
                            samplePos, 1U, samplePos,
                            "恢复重做 2/4 2号转至取样位")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;
    }

    /* ---- 3/4：取样位上方停留 2 秒 ---- */
    case 2:
        if (s_stepElapsedMs == 0U) {
            LOGD("[任务序列] 恢复重做 3/4 取样位上方停留 %u ms\n",
                          (unsigned)EMM_TASK_SEQ_RESUME_SETTLE_MS);
        }
        if (s_stepElapsedMs < EMM_TASK_SEQ_RESUME_SETTLE_MS) {
            return SEQ_STEP_RUNNING;
        }
        LOGD("[任务序列] 恢复重做 3/4 停留结束，开始重新取样\n");
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 4/4：转入取样流程的子步骤 2（跳过刚做过的 0/1） ---- */
    case 3:
    default:
        /*
         * `emmTaskSeqEnterPhase()` 会把 `s_subStep` 清 0、并清掉上一步的运动描述
         * 与 `s_pausedMidStep`；紧接着手动置 2，指向"3 号快降到取样高度1"。
         */
        emmTaskSeqEnterPhase(TASK_MOVE_POS);
        s_subStep = 2;
        LOGD("[任务序列] 恢复重做 4/4 转入取样流程"
                       "（从「3 号快降到取样高度1」继续，之后检测/抛弃照原流程）\n");
        return SEQ_STEP_RUNNING;
    }
}

/**
 * @brief 暂停归位（`TASK_PAUSE_PARK`，**2026/10/27 新增**）。
 * @details 只由 `SEQ PAUSE` 进入（已取样时前面还会先走一遍 `TASK_DISCARD_POS`）。
 *          目标状态是"**取样位正上方、安全高度**"，即 2 号在 `SET EMM POINT SAMPLE`
 *          （默认 985）、3 号在 `EMM_SEQ_LIFT_POS_SAFE`（5000）。操作员可以在这个
 *          状态下安全开门、取放样品。
 *
 *          子步骤只有两步，故意做得短：
 *            1/2  3 号抬到安全高度 —— **先抬再转**，否则带着探头旋转会撞样品盘；
 *            2/2  2 号转到取样位正上方。
 *
 *          ⚠️ 走完返回 `SEQ_STEP_DONE`，由 `emmTaskSeqRun()` 的 `TASK_PAUSE_PARK` 分支
 *          负责收尾（置 `SeqState = PAUSED`、黄灯慢闪、发 `EVT PAUSE PARKED`）。
 *          **本函数自己不改 `SeqState`** —— 一改就会让状态机早退，连返回的机会都没有。
 *
 *          ⚠️ 本阶段**不响应 `s_pauseRequested`**（见 `emmSeqHandlePauseResume()` 与
 *          `emmSeqHandleStepFailure()` 里的阶段排除），否则归位第一步就会被自己掐断。
 *
 * @return RUNNING 继续；DONE 归位完成；FAILED 某步失败（按正常失败处理，保留队列）。
 */
static SeqStepResult emmSeqRunPausePark(void)
{
    switch (s_subStep) {
    /* ---- 1/2：3 号抬到安全高度（先抬，避免带探头旋转） ---- */
    case 0:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 120U, 30U,
                            EMM_SEQ_LIFT_POS_SAFE, 1U,
                            EMM_SEQ_LIFT_POS_SAFE,
                            "暂停归位 1/2 3号抬至安全高度")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 2/2：2 号转到取样位正上方 ---- */
    case 1:
    default:
    {
        /* 取样位是运行期可改点位，必须走 `EmmSeqPoints_Get()` 而不是宏。 */
        const uint32_t samplePos = EmmSeqPoints_Get(EMM_SEQ_POINT_SAMPLE);
        if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 15U, 10U,
                            samplePos, 1U, samplePos,
                            "暂停归位 2/2 2号转至取样位正上方")) {
            return SEQ_STEP_FAILED;
        }
        LOGD("[任务序列] 暂停归位完成：探头已在取样位正上方（2号=%lu，3号=%u）\n",
                      (unsigned long)samplePos, (unsigned)EMM_SEQ_LIFT_POS_SAFE);
        return SEQ_STEP_DONE;
    }
    }
}

/* ==========================================================================
 * 状态机主循环
 * ========================================================================== */
/**
 * @brief 状态机单步推进。由 `emmTaskSeqTask()` 每 10ms 调用一次。
 */
static void emmTaskSeqRun(void)
{
    const SeqState st = EmmTaskQueue_GetSeqState();

    /*
     * 兜底守卫：结构电源一旦被切断（`SET POWER OFF`，或 30 分钟指令超时休眠），
     * 序列必须立刻终止。`SET POWER OFF` 里已经会调 `EmmTaskSeq_Stop()`，这里再
     * 兜一道，覆盖"指令超时休眠"这条不经过 cmdPower 的路径；否则 `SeqState`
     * 会一直停在 `RUNNING`，重新上电后与系统初始化/整机回零并行跑（实机已复现）。
     */
    if (cmd_power_off && (st == RUNNING || st == PAUSED)) {
        Serial.println("[任务序列] 检测到结构电源已切断，强制终止序列并清空队列");
        EmmTaskSeq_Stop();
        return;
    }

    /*
     * **2026/10/26 新增**：`END CUR` / `END ALL` 的"立即停机"在**非运动步**
     * （检测位停留计时、阶段之间的空档）上不会经过 `emmSeqHandlePauseResume()`，
     * 必须在这里直接消费，否则要等这一步自己走完才响应。
     *
     * ⚠️ 这段必须放在下面 `st != RUNNING` 的早退**之前**，并且显式允许 `PAUSED`：
     *    暂停在非运动步（如检测位停留）时状态是 `PAUSED`，早退会让 END 一直挂着
     *    直到 `SEQ RESUME` / `SEQ STOP`，与"电机动作立即停止、结束当前任务"不符。
     *    `STOPPED`（已被 STOP/断电收尾）不消费，避免给已中止的序列再排收尾动作。
     *
     * 已经在回收/复位阶段的，让它继续走完即可（不重入）。
     */
    if (s_endInterrupt && (st == RUNNING || st == PAUSED) &&
        s_taskPhase != TASK_DISCARD_POS && s_taskPhase != TASK_RESET_POS) {
        s_endInterrupt = false;
        Serial.printf("[任务序列] 收到结束请求（当前阶段 %s）：立即停机\n",
                      EmmTaskSeq_TaskPhaseToStr(s_taskPhase));
        emmSeqBroadcastStop("结束");
        /*
         * 若此刻是 `PAUSED`，先解除暂停：END 的语义是"结束"，不是"继续暂停"。
         * 不解除的话，下面 `enterPhase(TASK_IDLE)` / `enterPhase(TASK_DISCARD_POS)`
         * 之后下一轮会在 `st != RUNNING` 处早退，序列卡死在暂停态。
         */
        if (st == PAUSED) {
            Serial.println("[任务序列] 当前处于暂停态：按结束请求解除暂停");
            EmmTaskQueue_SetSeqState(RUNNING);
        }
        if (!emmSeqEndNeedsDiscard()) {
            Serial.println("[任务序列] 探头在安全高度，跳过回收");
            emmSeqEndAfterDiscard();
        } else {
            s_discardLiftFirst = true;   /* 探头可能还在低位，回收前先抬到安全高度 */
            emmTaskSeqEnterPhase(TASK_DISCARD_POS);
        }
        return;
    }

    /*
     * **2026/10/27 新增**：`SEQ PAUSE`（归位式暂停）的"立即停机"在**非运动步**
     * （检测位停留计时、阶段之间的空档）上不会经过 `emmSeqHandlePauseResume()`，
     * 必须在这里直接消费，否则要等这一步自己走完才响应，与"电机动作立即停止"不符。
     *
     * ⚠️ 与上面的 END 分支一样，必须放在 `st != RUNNING` 早退**之前**。但这里
     *    **只认 `RUNNING`**：暂停流程进行中 `SeqState` 一直保持 `RUNNING`，只有归位
     *    完成时才置 `PAUSED`；若已经 `PAUSED`（归位完成、`s_pauseParked`），说明
     *    暂停请求早已被消费，不该再来一次。
     *
     * 已经在回收/复位/归位阶段的，让它继续走完即可（不重入）。
     */
    if (s_pauseRequested && st == RUNNING &&
        s_taskPhase != TASK_DISCARD_POS &&
        s_taskPhase != TASK_RESET_POS &&
        s_taskPhase != TASK_PAUSE_PARK) {
        Serial.printf("[任务序列] 收到暂停请求（当前阶段 %s）：立即停机并准备归位\n",
                      EmmTaskSeq_TaskPhaseToStr(s_taskPhase));
        emmSeqBroadcastStop("暂停");
        /*
         * 判据与 `emmSeqHandleStepFailure()` 里的暂停分支保持一致（详见那里的注释）：
         * 只有"确实有样品在探头上"才回收，否则直接归位省一趟空跑。
         */
        const bool needRecycle = s_sampleTaken ||
                                 (s_taskPhase == TASK_MOVE_POS && s_subStep >= 2U);
        if (!needRecycle) {
            Serial.println("[任务序列] 还没取到样，跳过回收直接归位");
            emmTaskSeqEnterPhase(TASK_PAUSE_PARK);
        } else {
            Serial.println("[任务序列] 已取到样，先回收再归位");
            s_discardLiftFirst = true;   /* 探头可能还在低位，回收前先抬到安全高度 */
            emmTaskSeqEnterPhase(TASK_DISCARD_POS);
        }
        return;
    }

    /*
     * 只有 RUNNING 才推进。STOPPED 表示未启动或已结束；PAUSED 表示暂停；
     * HOMEING / MOVEING / CLEARNING / END_CUR_ALL 属于 KWQ1 的其它复合流程，
     * 本模块不实现，同样直接返回。所有非 RUNNING 分支都把计时基准清零，
     * 保证"暂停期间不计时"（对应 KWQ1 子状态机在 PAUSED 时冻结）。
     */
    if (st != RUNNING) {
        s_lastTickMs = 0;
        return;
    }

    const uint32_t now = millis();
    if (s_lastTickMs != 0) {
        s_stepElapsedMs += (now - s_lastTickMs);
    }
    s_lastTickMs = now;

    /*
     * **2026/10/20 新增**：消费 `SEQ RESUME`（重做模式）请求。
     *
     * 运动步中途暂停的请求，已经由 `emmSeqHandlePauseResume()`（让本步退出）+
     * `emmSeqHandleStepFailure()`（换阶段）消费掉了；**这里覆盖的是非运动步暂停**
     * ——典型是"检测位停留 T 分钟"期间暂停，那种情况没有阻塞循环，只能等状态机
     * 自己转回这里。两处靠 `s_pausedMidStep` 天然互斥。
     */
    if (s_restartSampleOnResume) {
        s_restartSampleOnResume = false;
        /*
         * **2026/10/27**：`TASK_PAUSE_PARK` 也算可重做 —— 暂停归位后阶段就停在它上面，
         * 恢复时正是要从取样位重新开始（`TASK_RESTART_SAMPLE` 的第一步就是 3 号回
         * 安全高度、2 号转取样位，与当前姿态连续，不会多跑）。
         */
        if (s_taskPhase == TASK_MOVE_POS || s_taskPhase == TASK_DETECT_POS ||
            s_taskPhase == TASK_DISCARD_POS || s_taskPhase == TASK_PAUSE_PARK) {
            Serial.println("[任务序列] 按 RESUME 重做模式：丢弃断点，回到取样位重做本任务");
            emmTaskSeqEnterPhase(TASK_RESTART_SAMPLE);
        } else {
            Serial.printf("[任务序列] 按 RESUME 重做模式：当前阶段 %s 不适用，按断点继续\n",
                          EmmTaskSeq_TaskPhaseToStr(s_taskPhase));
        }
        /* 本周期不再推进；下一周期从新阶段开始（`enterPhase` 已把计时基准清零）。 */
        return;
    }

    /* ---------------- 队列已跑完：先做一次复位，再收尾 ---------------- */
    if (EmmTaskQueue_Index() >= EmmTaskQueue_Count()) {
        /*
         * 复读一次状态再判"正常跑完"：`SEQ STOP` 会先置 `STOPPED` 再清空
         * 队列，如果本任务刚好在两次读之间被清空（Index/Count 同时归零），
         * 不复读就会把"被 STOP 清空"误判成"全部任务完成"并误发事件帧。
         */
        if (EmmTaskQueue_GetSeqState() != RUNNING) {
            return;
        }

        /* 先进入复位阶段（只做一次），复位完成后再走"全部完成"收尾。 */
        if (s_taskPhase != TASK_RESET_POS) {
            /* **2026/10/21**：这句降为 `LOGD` —— 复位开始时 `emmSeqRunReset()`
               自己会发 `EVT 350 RESET BEGIN`，两处说同一件事。 */
            LOGD("[任务序列] 队列已跑完，进入复位阶段（3 号上升 → 2 号回单圈回零原点）\n");
            emmTaskSeqEnterPhase(TASK_RESET_POS);
            return;
        }

        const SeqStepResult r = emmSeqRunReset();
        if (r == SEQ_STEP_DONE) {
            emmSeqFinishIfRunning();
        } else if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("复位", "3 号上升失败或 4 号样品盘回零失败（2 号回原点失败不中止，只发事件码）");
        }
        return;
    }

    /* ---------------- 取当前任务 ---------------- */
    const uint8_t idx = EmmTaskQueue_Index();
    const uint8_t total = EmmTaskQueue_Count();

    EmmTask t;
    if (!EmmTaskQueue_GetTask(idx, &t)) {
        /* 兜底：正常情况下 Index < Count 且 Count <= 10，不会走到这里。
           **2026/10/21**：这条路不走 `emmTaskSeqStepFailed()`，得自己补一帧 `ERR`，
           否则上位机只会看到序列"莫名其妙停了"。 */
        emmSeqEventFail("读取任务数据失败");
        emmTaskSeqAbort("读取任务数据失败");
        return;
    }

    /* ---------------- 按 TaskPhase 推进 ---------------- */
    switch (s_taskPhase) {
    /* ---------------------------------------------------------------- */
    case TASK_PREAMBLE:
    {
        const SeqStepResult r = emmSeqRunPreamble();
        if (r == SEQ_STEP_DONE) {
            emmSeqAdvanceTo(TASK_IDLE);
        } else if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("前导", "前导阶段异常（2026/10/19 起前导已无电机动作，出现本日志请排查）");
        }
        break;
    }

    /* ---------------------------------------------------------------- */
    case TASK_IDLE:
        /*
         * 进入本条任务。KWQ1 在此处做"溶剂切换 + 移动到 Axx 点位"；KWQ3 在这里
         * 只打印任务信息、点绿灯，真正的动作交给下面的位置阶段。
         */
        LOGD("[任务序列] ===== 任务 %u/%u 开始：类型=%c 工位=%02u T=%.3f min =====\n",
                      (unsigned)(idx + 1), (unsigned)total,
                      t.type, (unsigned)t.slot, (double)t.timeSec);
        rygLedOn(RYG_COLOR_GREEN);
        /* **2026/10/26**：新任务开始，样品还没取到 —— `END CUR/ALL` 据此判断是否跑回收。 */
        s_sampleTaken = false;
        /*
         * 先进 `TASK_TRAY_POS`：**先把自己的工位转到位，再取样**（2026/10/10 方案 A）。
         * 这样每条任务自包含，中途 `SEQ ADD` 的新任务也能在开始时立即校正工位，
         * 不再依赖"上一条任务提前预置"。
         */
        emmTaskSeqEnterPhase(TASK_TRAY_POS);
        break;

    /* ---------------------------------------------------------------- */
    /* 样品盘定位：**先把自己的工位转到位，再取样**（2026/10/10 方案 A）。
       注意执行顺序与枚举顺序不同 —— 本 case 排在取样（TASK_MOVE_POS）之前跑。 */
    case TASK_TRAY_POS:
    {
        const SeqStepResult r = emmSeqRunTrayPos(t);
        if (r == SEQ_STEP_DONE) {
            emmSeqAdvanceTo(TASK_MOVE_POS);
        } else if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("样品盘", "样品盘定位失败");
        }
        break;
    }

    /* -----------------------------------取样位----------------------------- */
    case TASK_MOVE_POS:
    {
        const SeqStepResult r = emmSeqRunMovePos();
        if (r == SEQ_STEP_DONE) {
            /* **2026/10/26**：取样完成 —— `END CUR/ALL` 据此判断回收是不是白跑。 */
            s_sampleTaken = true;
            /* 取样完成事件（2026/10/21）：必须在 `emmSeqAdvanceTo()` **之前**发，
             * 否则下一阶段的 `DETECT BEGIN` 会抢在 `SAMPLE DONE` 前面。 */
            emmSeqEventPhase(SEQ_SAMPLE_DONE, "SAMPLE", "DONE");
            emmSeqAdvanceTo(TASK_DETECT_POS);
        } else if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("取样位", "取样动作失败");
        }
        break;
    }

    /* ---------------------------------------------------------------- */
    case TASK_DETECT_POS:
    {
        const SeqStepResult r = emmSeqRunDetectPos(t);
        if (r == SEQ_STEP_DONE) {
            emmSeqEventPhase(SEQ_DETECT_DONE, "DETECT", "DONE");
            emmSeqAdvanceTo(TASK_DISCARD_POS);
        } else if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("检测位", "检测位动作失败");
        }
        break;
    }

    /* ---------------------------------------------------------------- */
    case TASK_DISCARD_POS:
    {
        const SeqStepResult r = emmSeqRunDiscardPos();
        if (r == SEQ_STEP_DONE) {
            emmSeqEventPhase(SEQ_RECYCLE_DONE, "RECYCLE", "DONE");
            LOGD("[任务序列] ===== 任务 %u/%u 完成 =====\n",
                          (unsigned)(idx + 1), (unsigned)total);

            /* 样品已在回收位处理掉，下一条任务需要重新取样。 */
            s_sampleTaken = false;

            /*
             * **2026/10/27**：`SEQ PAUSE` 也走回收流程，但回收之后的去向不同 ——
             * 不推进任务序号，而是转入 `TASK_PAUSE_PARK` 做归位（3 号抬安全高度、
             * 2 号转到取样位正上方），归位完成才真正置 `PAUSED`。
             *
             * 判断顺序：`s_endMode` **优先于** `s_pauseRequested`。END 是"终止"，
             * 若用户先按暂停、后按结束，不能被降级成"暂停"；此时顺手清掉暂停标志，
             * 否则下一轮顶层会再触发一次回收 + 归位。
             */
            if (s_endMode != SEQ_END_NONE) {
                s_pauseRequested = false;
                emmSeqEndAfterDiscard();
            } else if (s_pauseRequested) {
                Serial.println("[任务序列] 暂停触发的回收完成：转入归位阶段（不推进任务序号）");
                emmTaskSeqEnterPhase(TASK_PAUSE_PARK);
            } else if (EmmTaskQueue_GetSeqState() == RUNNING) {
                /* 只有序列仍在跑时才推进游标；被中止时队列已被清空，不动它。 */
                EmmTaskQueue_SetIndex((uint8_t)(idx + 1));
                emmTaskSeqEnterPhase(TASK_IDLE);
            } else {
                Serial.println("[任务序列] 序列已被中止，不再推进任务游标");
            }
        } else if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("回收位", "回收位动作失败");
        }
        break;
    }

    /* ---------------------------------------------------------------- */
    case TASK_RESTART_SAMPLE:
    {
        /*
         * `SEQ RESUME`（重做模式）专用阶段：回取样位上方 → 停 2s → 转入取样流程子步骤 2。
         * **不改变 `Index`**，所以重做的正是当前这条任务；之后依次走
         * 取样 → 检测 → 抛弃 → `Index++`，继续队列里剩余的任务（用户 2026/10/20 选定）。
         */
        const SeqStepResult r = emmSeqRunRestartSample();
        if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("恢复重做", "回到取样位途中失败");
        }
        break;
    }

    /* ---------------------------------------------------------------- */
    case TASK_PAUSE_PARK:
    {
        /*
         * **2026/10/27 新增**：归位式暂停的最后一段。`emmSeqRunPausePark()`
         * 做完"3 号抬到安全高度 → 2 号转到取样位正上方"之后，才**真正**置 `PAUSED`。
         * 这样暂停期间探头一定是安全的、且正对取样位，`SEQ RESUME` 重做时从
         * 取样位起步即可，不必再猜"断点在哪"。
         *
         * ⚠️ 归位期间 `SeqState` 一直保持 `RUNNING`（否则本函数开头就早退了，
         *    回收 + 归位根本跑不起来）。上位机侧看到的表现是：
         *    `OK @n SEQ PAUSE` 立即返回 → 若干秒后收到 `EVT 380 PAUSE PARKED`
         *    → 之后查询状态才是 `PAUSED`。
         */
        const SeqStepResult r = emmSeqRunPausePark();
        if (r == SEQ_STEP_DONE) {
            /*
             * **边界情况**：归位途中又收到 `SEQ STOP` / `END CUR`。
             * 归位阶段**不响应结束打断**（它本身就是把探头送回安全位置的动作），
             * 所以归位一定会走完；走完之后不能再置 `PAUSED` —— 否则结束请求会被
             * 卡在暂停态里，要等用户再按一次 `SEQ STOP` 才动。
             * 这里只把阶段退回 `TASK_IDLE`，顶层下一轮就会消费 `s_endInterrupt`
             * （`TASK_IDLE` 被判为"无需回收"），直接走清队列 + 复位。
             */
            if (s_endInterrupt) {
                Serial.println("[任务序列] 暂停归位完成，但已有结束请求：转由结束流程收尾（不置 PAUSED）");
                s_pauseRequested = false;
                emmTaskSeqEnterPhase(TASK_IDLE);
                break;
            }

            s_pauseRequested = false;
            s_pauseParked = true;
            EmmTaskQueue_SetSeqState(PAUSED);
            rygLedBlink(RYG_COLOR_YELLOW, EMM_TASK_SEQ_PAUSE_BLINK_MS);
            Serial.println("[任务序列] ===== 已暂停：探头在取样位正上方（安全高度）=====");
            sendWithCRC("EVT %d PAUSE PARKED", (int)SEQ_PAUSE_PARKED);
        } else if (r == SEQ_STEP_FAILED) {
            /* 归位本身失败说明电机真有问题：走失败收尾把 `ERR` 帧报出来，
               而不是重入归位（`emmSeqHandleStepFailure` 的暂停分支已排除本阶段）。 */
            emmSeqHandleStepFailure("暂停归位", "暂停归位失败（3 号抬升或 2 号回转失败）");
        }
        break;
    }

    /* ---------------------------------------------------------------- */
    case TASK_RESET_POS:
    {
        /*
         * 正常情况下复位只会在"队列已跑完"分支里被触发；这里再兜一道，
         * 避免任何异常路径下复位阶段被跳过。
         */
        const SeqStepResult r = emmSeqRunReset();
        if (r == SEQ_STEP_DONE) {
            emmSeqFinishIfRunning();
        } else if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("复位", "3 号上升失败或 4 号样品盘回零失败（2 号回原点失败不中止，只发事件码）");
        }
        break;
    }

    /* ---------------------------------------------------------------- */
    default:
        /*
         * TASK_WAIT_RESISTOR_ACK / TASK_AUX_RUN / TASK_PAUSED / TASK_END 未使用。
         * 若因异常进入，直接退回 TASK_IDLE，避免状态机卡死。
         */
        emmTaskSeqEnterPhase(TASK_IDLE);
        break;
    }
}

/**
 * @brief 序列执行常驻任务主体：10ms 周期轮询状态机。
 */
static void emmTaskSeqTask(void *argument)
{
    (void)argument;

    for (;;) {
        emmTaskSeqRun();
        vTaskDelay(pdMS_TO_TICKS(EMM_TASK_SEQ_POLL_MS));
    }
}

/* ==========================================================================
 * 对外接口：生命周期与查询
 * ========================================================================== */

bool EmmTaskSeq_Init(void)
{
    if (s_seqTaskHandle != nullptr) {
        return true;    /* 任务已创建，重复调用直接成功。 */
    }

    const BaseType_t result = xTaskCreatePinnedToCore(
        emmTaskSeqTask,
        "EmmTaskSeq",
        EMM_TASK_SEQ_TASK_STACK,
        nullptr,
        EMM_TASK_SEQ_TASK_PRIORITY,
        &s_seqTaskHandle,
        EMM_TASK_SEQ_TASK_CORE);

    if (result != pdPASS) {
        s_seqTaskHandle = nullptr;
        Serial.println("[任务序列] 序列执行任务启动失败：无法创建后台任务");
        return false;
    }

    emmTaskSeqResetPhase();
    Serial.printf("[任务序列] 序列执行任务已启动：优先级=%d 核心=%d 轮询=%u ms\n",
                  (int)EMM_TASK_SEQ_TASK_PRIORITY,
                  (int)EMM_TASK_SEQ_TASK_CORE,
                  (unsigned)EMM_TASK_SEQ_POLL_MS);
    return true;
}

bool EmmTaskSeq_IsActive(void)
{
    const SeqState st = EmmTaskQueue_GetSeqState();
    return (st == RUNNING || st == PAUSED);
}

void EmmTaskSeq_Stop(void)
{
    const SeqState st = EmmTaskQueue_GetSeqState();

    if (st == RUNNING || st == PAUSED) {
        Serial.printf("[任务序列] 强制终止序列（原状态 %s）并清空队列\n",
                      EmmTaskQueue_SeqStateToStr(st));
    } else {
        Serial.printf("[任务序列] 强制终止：当前状态 %s，按空操作处理\n",
                      EmmTaskQueue_SeqStateToStr(st));
    }

    /*
     * 顺序很重要：先把 `SeqState` 置成 `STOPPED`，再清空队列。这样状态机在
     * 下一个轮询周期会从 `st != RUNNING` 直接返回，不会看到"Index/Count 同时
     * 归零"而误判成"全部任务完成"（详见 emmTaskSeqRun 里的复读守卫）。
     */
    EmmTaskQueue_SetSeqState(STOPPED);
    EmmTaskQueue_Clear();
    emmTaskSeqResetPhase();
    rygLedOn(RYG_COLOR_YELLOW);

    /*
     * 【2026/10/10 用户要求】**立即广播停止全部 EMM 电机**。
     * 之前 STOP 只改状态、不发任何停止命令，正在跑的那一步（驱动器内部的脉冲串）
     * 会一直走完；这里补上"命令一到就停"，且不依赖序列任务是否在轮询（它可能正卡在
     * 同步步的 `vTaskDelay` / 静默间隔里）。
     * 注：`SET POWER OFF` 路径（`sysPowerOffCleanup()`）随后也会广播一次，重复一帧无害。
     */
    if (Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
        const EmmV5::Status stopStatus = gEmm.stopBroadcast(false);
        Rs485Bus_Give();
        Serial.printf("[任务序列] STOP：已广播停止全部 EMM 电机，状态=%s\n",
                      EmmV5::statusToString(stopStatus));
    } else {
        Serial.println("[任务序列] STOP：RS485 总线繁忙，停止全部电机的命令未下发");
    }
}

TaskPhase EmmTaskSeq_GetPhase(void)
{
    return s_taskPhase;
}

bool EmmTaskSeq_BlocksCommand(const char *commandName)
{
    if (!EmmTaskSeq_IsActive() || commandName == nullptr) {
        return false;
    }

    /*
     * 必须放行的例外（与 EmmSystemHome_BlocksCommand 同一套判据）：
     *   - `SET EMM SYSTEM HOME STOP`：回零中止命令，不影响序列，放行；
     *   - `SET EMM LIFT MONITOR`：只切换本地布防标志，不产生总线事务；
     *   - `SET EMM POINT`：**2026/10/21 新增**。只改内存变量 + 写 NVS，
     *     **不产生任何 RS485 总线事务、不触发运动**，与"序列独占机械机构"不冲突。
     *     安全性来自读取时机：序列每一步进入时才 `EmmSeqPoints_Get()` 读一次，
     *     随即把目标脉冲下发并阻塞等到位 —— 正在执行的那一步改不动，
     *     只有**还没走到的步骤**会用新值。`RESET=1`（一次改 9 个）仍由
     *     `cmdSetEmmPoint()` 自己拒绝，那里能看到参数、本函数看不到。
     *   - `SET EMM TRAY ZERO`：**2026/10/26 新增**。只改"下次样品盘标定用"的脉冲数
     *     （内存变量 + NVS），不产生总线事务、不触发运动；标定流程进入时只读一次
     *     快照，运行中改对本次无影响。
     * 注意这里**不**放行 `SET EMM SYSTEM HOME`——序列运行期间不允许启动整机
     * 回零，否则两套复合动作会同时抢三台电机。
     */
    if (strcmp(commandName, "SET EMM SYSTEM HOME STOP") == 0 ||
        strcmp(commandName, "SET EMM LIFT MONITOR") == 0 ||
        strcmp(commandName, "SET EMM POINT") == 0 ||
        strcmp(commandName, "SET EMM TRAY ZERO") == 0) {
        return false;
    }

    /* 序列运行期间独占机械机构：拦截所有会触发动作的 SET EMM 命令。 */
    return strncmp(commandName, "SET EMM", 7) == 0;
}

/* ==========================================================================
 * 对外接口：上位机命令
 * ========================================================================== */

/**
 * @brief `SEQ START` 开跑前：打印 9 个点位快照，并做**点位之间**的关系自检。
 * @details **为什么要有**（2026/10/20 新增）：
 *          9 个点位都是运行期可改的，改完只写 NVS，**跑序列时不会再提示**。
 *          现场排查时只能翻每一步的 `PULSE=` 日志去拼"这次到底用的哪套值"；
 *          更麻烦的是"**单值合法但关系不合理**"（例如 `LIFT_S1` 比安全高度还高）
 *          程序照做、不报错 —— "快降"实际变成上升，只是取不到样。
 *
 *          **校验只警告、不拒绝**（用户 2026/10/20 选定）：调点位期间难免出现
 *          暂时不合理，硬拒会逼着人"必须先改对才能试跑"，反而更难调。
 *
 *          规则来自 `TASK_MOVE_POS` / `TASK_DETECT_POS` / `TASK_DISCARD_POS`
 *          的实际动作顺序（**数值增大 = 下降**，与 `EMM_SEQ_LIFT_POS_SAFE` 一致）。
 *
 *          **2026/10/21 起也被"运行中改点位"复用**：`SET EMM POINT` 允许在
 *          序列运行中执行（见 `cmdSetEmmPoint`），改完立刻调本函数再自检一次 ——
 *          运行中改点位最容易改出"单值合法但关系不合理"，而这时恰恰没有开跑自检。
 * @param tag 点位快照的抬头说明；`nullptr` 用默认文案。
 * @return true 关系全部合理；false 至少一条不合理（调用方只打日志、不中止）。
 */
static bool emmSeqCheckPoints(const char *tag)
{
    EmmSeqPoints_PrintAll((tag != nullptr) ? tag
                                          : "本次序列使用的点位（2 号 4 个 / 3 号 5 个）");

    const uint32_t liftS1 = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_SAMPLE_1);
    const uint32_t liftS2 = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_SAMPLE_2);
    const uint32_t liftDet = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DETECT);
    const uint32_t liftD2 = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DISCARD_2);
    const uint32_t liftD3 = EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DISCARD_3);

    /* 3 号的"安全高度 / 起始下降位"（取样位子步骤 0 用的就是它）。 */
    const uint32_t safe = EMM_SEQ_LIFT_POS_SAMPLE_START;

    bool ok = true;

    /* ① 取样高度 1 必须比安全高度更深，否则"快降"会变成上升。 */
    if (liftS1 <= safe) {
        Serial.printf("[任务序列] 点位警告 LIFT_S1=%lu 不深于安全高度 %lu："
                      "「取样位 3/9 快降」会变成上升\n",
                      (unsigned long)liftS1, (unsigned long)safe);
        ok = false;
    }
    /* ② 取样高度 2 必须比取样高度 1 更深（先快降到 S1、再慢降到 S2）。 */
    if (liftS2 <= liftS1) {
        Serial.printf("[任务序列] 点位警告 LIFT_S2=%lu 不深于 LIFT_S1=%lu："
                      "「取样位 5/9 慢降」不会继续下降\n",
                      (unsigned long)liftS2, (unsigned long)liftS1);
        ok = false;
    }
    /* ③ 检测高度必须比安全高度深，且比取样高度 1 浅（检测在液面上方）。 */
    if (liftDet <= safe) {
        Serial.printf("[任务序列] 点位警告 LIFT_DET=%lu 不深于安全高度 %lu："
                      "「检测位 2/4 下降」会变成上升\n",
                      (unsigned long)liftDet, (unsigned long)safe);
        ok = false;
    }
    if (liftDet >= liftS1) {
        Serial.printf("[任务序列] 点位警告 LIFT_DET=%lu 不浅于 LIFT_S1=%lu："
                      "检测位应在取样高度 1 之上，否则会压到取样点\n",
                      (unsigned long)liftDet, (unsigned long)liftS1);
        ok = false;
    }
    /* ④ 回收位：第二段要比安全高度深，第三段要比第二段浅（流程是"回升"）。 */
    if (liftD2 <= safe) {
        Serial.printf("[任务序列] 点位警告 LIFT_D2=%lu 不深于安全高度 %lu："
                      "「回收位 4/10 下降」会变成上升\n",
                      (unsigned long)liftD2, (unsigned long)safe);
        ok = false;
    }
    if (liftD3 <= safe) {
        Serial.printf("[任务序列] 点位警告 LIFT_D3=%lu 不深于安全高度 %lu："
                      "「回收位 8/10 回升」会升到安全高度以上\n",
                      (unsigned long)liftD3, (unsigned long)safe);
        ok = false;
    }
    if (liftD3 >= liftD2) {
        Serial.printf("[任务序列] 点位警告 LIFT_D3=%lu 不浅于 LIFT_D2=%lu："
                      "「回收位 8/10 回升」不会往上走\n",
                      (unsigned long)liftD3, (unsigned long)liftD2);
        ok = false;
    }

    if (ok) {
        LOGD("[任务序列] 点位关系自检：通过（9 个点位都在合理位置）\n");
    }
    return ok;
}

/**
 * @brief 供**命令层**调用的点位关系自检包装（**2026/10/21 新增**）。
 * @details `SET EMM POINT` 现在允许在任务序列运行中执行，改完点位后由
 *          `cmdSetEmmPoint()` 调本函数再自检一次 —— 运行中改点位最容易改出
 *          "单值合法但关系不合理"，而这时离下一次 `SEQ START` 还很久，
 *          开跑自检根本不会跑。
 * @note 只警告、不拒绝（与 `SEQ START` 前那次口径一致）：点位可能正处在
 *       "调一半"的中间态，硬拒会让人没法边跑边调。
 * @return true 关系全部合理；false 至少一条不合理（警告已由内部打印）。
 */
bool EmmTaskSeq_CheckPoints(void)
{
    return emmSeqCheckPoints("运行中修改后的点位（2 号 4 个 / 3 号 5 个）");
}

bool cmdSeqStart(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    LOGD("[任务序列] ========== SEQ START 前置检查开始 ==========\n");

    /*
     * 前置检查开始事件（**2026/10/21 新增**）。
     * ⚠️ 顺序说明：`OK @n SEQ START` 是在**全部检查通过之后**才发的（这是原有契约：
     *    被拒时只回 `ERR @n 1xx`，不回 OK）。所以上位机看到的顺序是
     *    `EVT 310 CHECK BEGIN` → `OK @n SEQ START` → `EVT 311 CHECK OK`。
     *    前置检查本身是同步的、只有几十毫秒，不必担心"CHECK BEGIN 之后没动静"。
     */
    sendWithCRC("EVT %d CHECK BEGIN", (int)SEQ_CHECK_BEGIN);

    /* ① 结构电源必须已合上（与 L1 运动命令一致）。 */
    if (cmd_power_off) {
        Serial.println("[任务序列] 前置检查 1/9 结构电源：未接通");
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }
    LOGD("[任务序列] 前置检查 1/9 结构电源：已接通\n");

    /* ② 序列本身不能已在运行；整机回零期间也不允许启动序列（抢电机）。 */
    const SeqState st = EmmTaskQueue_GetSeqState();
    if (st == RUNNING || st == PAUSED) {
        Serial.printf("[任务序列] 前置检查 2/9 序列状态：%s（不允许重复启动）\n",
                      EmmTaskQueue_SeqStateToStr(st));
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }
    if (EmmSystemHome_IsActive()) {
        Serial.println("[任务序列] 前置检查 2/9 序列状态：整机回零正在运行，拒绝启动");
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }
    LOGD("[任务序列] 前置检查 2/9 序列状态：空闲\n");

    /* ③ 队列不能为空。 */
    if (EmmTaskQueue_Count() == 0) {
        Serial.println("[任务序列] 前置检查 3/9 任务队列：为空");
        sendWithCRC("ERR @%d %d", seq, TASK_NULL);
        return false;
    }
    LOGD("[任务序列] 前置检查 3/9 任务队列：%u 条\n",
                  (unsigned)EmmTaskQueue_Count());

    /* ④ 上位机心跳在线（PING 超时未恢复时不允许开始跑任务）。 */
    if (!ping_on_line) {
        Serial.println("[任务序列] 前置检查 4/9 心跳在线：不在线");
        sendWithCRC("ERR @%d %d", seq, NO_HEARTBEAT);
        return false;
    }
    LOGD("[任务序列] 前置检查 4/9 心跳在线：通过\n");

    /* ⑤ 舱门已关闭。 */
    if (!checkDoorState()) {
        Serial.println("[任务序列] 前置检查 5/9 舱门关闭：未到位");
        rygLedBlink(RYG_COLOR_YELLOW, EMM_TASK_SEQ_PAUSE_BLINK_MS);
        sendWithCRC("ERR @%d %d", seq, NO_DOOR);
        return false;
    }
    LOGD("[任务序列] 前置检查 5/9 舱门关闭：通过\n");

    /* ⑥ 样品盘已放置。 */
    if (!checkSampleState()) {
        Serial.println("[任务序列] 前置检查 6/9 样品盘到位：未到位");
        rygLedBlink(RYG_COLOR_YELLOW, EMM_TASK_SEQ_PAUSE_BLINK_MS);
        sendWithCRC("ERR @%d %d", seq, NO_SAMPLE);
        return false;
    }
    LOGD("[任务序列] 前置检查 6/9 样品盘到位：通过\n");

    /*
     * ⑦ 电机使能检查 —— 【占位：默认通过，未接真实检测】
     *    真实实现应为：读取 2/3/4 号电机的状态帧（EMM 0x3A bit0 Ens_TF），
     *    任一未使能则拒绝启动。按用户要求本阶段先注释占位、默认通过，等实机
     *    确认"上电是否自动使能"后再接上；届时可复用 109 NO_STEP_PWR 或新增
     *    专用错误码（见《修改日志.md》§七 的分步计划）。
     */
    // if (!emmAllMotorsEnabled()) {
    //     Serial.println("[任务序列] 前置检查 7/9 电机使能：未使能");
    //     sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
    //     return false;
    // }
    LOGD("[任务序列] 前置检查 7/9 电机使能：占位默认通过（未接真实检测）\n");

    /* ⑧ 控制板温度未超限（取 5 路驱动温度最大值与 SET TEMP 阈值比较）。 */
    const float maxDriverTemp = getMaxDriverTemp();
    if (maxDriverTemp > max_temp) {
        Serial.printf("[任务序列] 前置检查 8/9 控制板温度：超限（最高 %.1f℃ > %.1f℃）\n",
                      (double)maxDriverTemp, (double)max_temp);
        sendWithCRC("ERR @%d %d", seq, TEMP_HIGH);
        return false;
    }
    LOGD("[任务序列] 前置检查 8/9 控制板温度：通过（最高 %.1f℃ <= %.1f℃）\n",
                  (double)maxDriverTemp, (double)max_temp);

    /* ⑨ 整机已回零（由 EmmSystemHome 的 17 步序列全部成功后置位）。 */
    if (!sys_is_homed) {
        Serial.println("[任务序列] 前置检查 9/9 整机回零：尚未回零");
        sendWithCRC("ERR @%d %d", seq, NO_HOME);
        return false;
    }
    LOGD("[任务序列] 前置检查 9/9 整机回零：已建立零点\n");

    /*
     * ⑩ 点位快照 + 关系自检（**2026/10/20 新增**）。
     * 不算前置检查、**不拦截启动** —— 只把"本次用的是哪一套点位"和
     * "这套点位合不合理"留在日志里，便于事后复盘（见 `emmSeqCheckPoints`）。
     */
    if (!emmSeqCheckPoints(nullptr)) {
        Serial.println("[任务序列] 点位关系存在可疑项（见上方警告），序列仍按这些值执行");
    }

    /* ---------------- 全部通过：先做前导建立零点，再从第 1 条开始 ---------------- */
    EmmTaskQueue_SetIndex(0);
    /*
     * 先进入 `TASK_PREAMBLE`：`SET EMM ZERO A=3` + 2/3 号设单圈零点（只做一次）。
     * 样品盘不在这里处理 —— 它的零点由整机回零一次性建立，本序列按绝对位置走。
     * 前导完成后状态机自动进入第 1 条任务的 `TASK_IDLE`。
     */
    emmTaskSeqEnterPhase(TASK_PREAMBLE);
    EmmTaskQueue_SetSeqState(RUNNING);

    LOGD("[任务序列] ========== SEQ START 通过，共 %u 条任务 ==========\n",
                  (unsigned)EmmTaskQueue_Count());
    sendWithCRC("OK @%d SEQ START", seq);

    /* 前置检查通过事件（2026/10/21）：放在 OK 之后，避免"检查还没结论就先报通过"。 */
    sendWithCRC("EVT %d CHECK OK", (int)SEQ_CHECK_OK);
    return true;
}

bool cmdSeqPause(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    if (cmd_power_off) {
        Serial.println("[任务序列] SEQ PAUSE 被拒绝：结构电源未接通");
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }

    /*
     * 幂等：已经在暂停流程里（`s_pauseRequested`）或已归位暂停（`PAUSED`）时
     * 重复下发，直接回 OK —— 上位机可能因为迟迟没收到 `EVT 380 PAUSE PARKED`
     * 而重发，这时报错只会让它以为暂停失败。
     */
    if (s_pauseParked || EmmTaskQueue_GetSeqState() == PAUSED) {
        Serial.println("[任务序列] SEQ PAUSE：已处于暂停状态（探头已归位），无需重复");
        sendWithCRC("OK @%d SEQ PAUSE", seq);
        return true;
    }
    if (s_pauseRequested) {
        Serial.println("[任务序列] SEQ PAUSE：暂停已在处理中（正在回收/归位），忽略本次");
        sendWithCRC("OK @%d SEQ PAUSE", seq);
        return true;
    }

    if (EmmTaskQueue_GetSeqState() != RUNNING) {
        Serial.printf("[任务序列] SEQ PAUSE 被拒绝：当前状态 %s（非 RUNNING）\n",
                      EmmTaskQueue_SeqStateToStr(EmmTaskQueue_GetSeqState()));
        /* 110 SEQ_RUNNING 在此处语义为"序列状态不允许该操作"（用户确认复用）。 */
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }

    /*
     * `SEQ STOP` / `END CUR` 正在收尾：它们本来就会"立即停机 + 回收 + 复位"，
     * 比暂停更彻底。这里回 OK 但不置暂停标志 —— 否则归位流程会和 END 的复位
     * 流程抢同一批电机，收尾动作会互相掐断。
     */
    if (s_endMode != SEQ_END_NONE || s_endInterrupt) {
        Serial.println("[任务序列] SEQ PAUSE：已有结束请求在处理（会更彻底地停机），本次暂停忽略");
        sendWithCRC("OK @%d SEQ PAUSE", seq);
        return true;
    }

    /*
     * **2026/10/27 改版**：只置"待执行"标志，**不动 `SeqState`**。
     *
     * 归位式暂停要依次经历"回收（若已取样）→ 3 号抬到安全高度 → 2 号转到取样位
     * 正上方"，这三步都得由状态机自己跑；而 `emmTaskSeqRun()` 开头 `st != RUNNING`
     * 就早退了 —— 提前置 `PAUSED` 会把刚排好的回收/归位直接掐死。
     * （2026/10/20 的续跑式暂停正好相反：那时需要 `PAUSED` 冻结状态机。）
     * 真正置 `PAUSED` 的时机在 `TASK_PAUSE_PARK` 的完成分支，同时发
     * `EVT 380 PAUSE PARKED`。
     *
     * 命令侧先广播一次停机，不等 10 ms 轮询 —— 满足"电机动作立即停止"。
     */
    s_pauseRequested = true;
    s_pausedMidStep = false;   /* 续跑式暂停已废弃，清掉旧标记免得干扰状态机 */
    Serial.println("[任务序列] SEQ PAUSE：已请求暂停"
                   "（立即停机 → 回收 → 归位到取样位正上方）");
    emmSeqBroadcastStop("暂停");

    sendWithCRC("OK @%d SEQ PAUSE", seq);
    return true;
}

bool cmdSeqResume(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();

    /*
     * **2026/10/27 改版**：`MODE` 参数**整体移除** —— 恢复只剩"重做"一种方式
     * （用户要求"序列恢复命令不需要 MODE，只需要 `SEQ RESUME`"）。
     *
     * 老上位机若仍带 `MODE=0` / `MODE=1`：**不报错、忽略、只打印一行提示**，
     * 应答统一为 `OK @n SEQ RESUME`（不再回显 MODE）。这样现场不必同步升级上位机，
     * 也不会因为"参数错 102"而误判成恢复失败。
     *
     * **为什么续跑彻底不能留**：`SEQ PAUSE` 现在会把探头归位到取样位正上方、
     * 并把已取的样品回收掉。断点的物理意义已经消失 —— 若还按"重发本步命令
     * 从断点继续"，等于让探头在安全高度上假装还在原位置做绝对定位，
     * 后续每一步的相对动作全错位。
     */
    if (args != nullptr && args[0] != '\0') {
        Serial.printf("[任务序列] SEQ RESUME：参数 \"%s\" 已废弃"
                      "（恢复只剩重做一种方式），忽略\n", args);
    }

    if (cmd_power_off) {
        Serial.println("[任务序列] SEQ RESUME 被拒绝：结构电源未接通");
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }

    if (EmmTaskQueue_GetSeqState() != PAUSED) {
        Serial.printf("[任务序列] SEQ RESUME 被拒绝：当前状态 %s（非 PAUSED）\n",
                      EmmTaskQueue_SeqStateToStr(EmmTaskQueue_GetSeqState()));
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }

    /*
     * 恢复前复查两个物理条件（对齐 KWQ1 在 RESUME 时复查限位的行为）：
     * 暂停期间操作员可能开门或取走样品盘，若不复查就会被带着继续跑。
     * KWQ1 还查水瓶（NO_WATER），KWQ3 的 SEQ START 未包含该项，此处保持一致。
     */
    if (!checkDoorState()) {
        Serial.println("[任务序列] SEQ RESUME 被拒绝：舱门未关闭");
        rygLedBlink(RYG_COLOR_YELLOW, EMM_TASK_SEQ_PAUSE_BLINK_MS);
        sendWithCRC("ERR @%d %d", seq, NO_DOOR);
        return false;
    }
    if (!checkSampleState()) {
        Serial.println("[任务序列] SEQ RESUME 被拒绝：样品盘未到位");
        rygLedBlink(RYG_COLOR_YELLOW, EMM_TASK_SEQ_PAUSE_BLINK_MS);
        sendWithCRC("ERR @%d %d", seq, NO_SAMPLE);
        return false;
    }

    /*
     * 恢复语义只剩"重做"：从取样位重新取样 → 检测 → 回收 → 继续剩余任务。
     *
     * **先置标志、再置状态**：反过来的话序列任务可能先看到 `RUNNING` 而漏掉本次请求。
     */
    const TaskPhase curPhase = s_taskPhase;
    /*
     * `s_pauseParked` 必须算进"可重做"：暂停归位完成后阶段就停在 `TASK_PAUSE_PARK`，
     * 而恢复的第一步正是从取样位重新开始，与"重做"完全吻合。
     */
    const bool canRestart = (s_pauseParked ||
                             curPhase == TASK_MOVE_POS ||
                             curPhase == TASK_DETECT_POS ||
                             curPhase == TASK_DISCARD_POS);

    /* 暂停标志到此消费完毕：不管走哪条路，接下来都算"已恢复"。 */
    s_pauseRequested = false;
    s_pauseParked = false;

    if (canRestart) {
        s_restartSampleOnResume = true;
        Serial.printf("[任务序列] 已恢复(重做)：当前阶段 %s，将丢弃断点、"
                      "回取样位重做本任务\n", EmmTaskSeq_TaskPhaseToStr(curPhase));
    } else {
        s_restartSampleOnResume = false;
        Serial.printf("[任务序列] 已恢复：当前阶段 %s 不适用重做，按断点继续\n",
                      EmmTaskSeq_TaskPhaseToStr(curPhase));
    }

    EmmTaskQueue_SetSeqState(RUNNING);
    rygLedOn(RYG_COLOR_GREEN);
    sendWithCRC("OK @%d SEQ RESUME", seq);
    return true;
}

bool cmdSeqStop(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    if (cmd_power_off) {
        Serial.println("[任务序列] SEQ STOP 被拒绝：结构电源未接通");
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }

    /*
     * **2026/10/27 改版**：`SEQ STOP` 接管原来 `END ALL` 的语义 ——
     *   立即停机 → 回收（已取样时）→ 清空队列 → 复位（2 号回原点 + 4 号样品盘
     *   重新标定）→ `EVT 300 SEQ ALL DONE`。
     * 原来的"广播停机 + 清队列 + 机械停在原地"迁到新命令 `ESTOP` 上（用户拍板）。
     *
     * ⚠️ 请求是**异步**的：这里只置标志 + 广播一次停机，回收与复位由状态机
     *    跑完，耗时可能几十秒。所以 `OK` 只代表"已受理"，真正的收尾信号是
     *    `EVT 300`。这与旧版"STOP 一到就回 STOPPED"不同，上位机别把 OK 当完成。
     *
     * ⚠️ 序列本来就没在跑（`STOPPED`）时按幂等处理：不发 ERR，回 OK 无动作。
     */
    const bool accepted = EmmTaskSeq_RequestEndAll();
    if (!accepted) {
        Serial.println("[任务序列] SEQ STOP：序列未在运行，无动作（幂等）");
    }

    sendWithCRC("OK @%d SEQ STOP", seq);
    return true;
}

/* ==========================================================================
 * 结束语义：END CUR / SEQ STOP / ESTOP（2026/10/26 新增，2026/10/27 重排）
 * --------------------------------------------------------------------------
 * 三条命令现在分工如下（2026/10/27 用户拍板）：
 *
 *   END CUR —— 电机动作立即停止 → 结束当前任务 → 回收 → 执行下一个任务；
 *              若为最后一个任务，则执行任务阶段的复位流程；
 *   SEQ STOP —— 电机动作立即停止 → 结束全部任务 → 回收 → **清空队列** → 复位；
 *              （接管原 `END ALL` 的语义，`END ALL` 命令已删除）
 *   ESTOP ——  电机动作立即停止 → 清空队列 → 回 `STOPPED`，**机械停在原地**，
 *              不回收、不复位；任何状态都受理、永不拒绝。
 *
 * 实现要点：
 *   - `END CUR` / `SEQ STOP` 是"**收尾式结束**"：先把探头按回收流程抬回安全高度，
 *     再做复位（2 号回单圈原点 + 4 号样品盘重新标定），所以**不能清队列就完事**；
 *   - `ESTOP` 走 `EmmTaskSeq_Stop()`（与硬件急停 `exio.cpp` GPIO1 同一条路径），
 *     要的就是"立刻停、别的都不管"。
 *
 * 三个标志的分工见 `s_endMode` / `s_endInterrupt` / `s_sampleTaken` 的注释。
 * ========================================================================== */

bool EmmTaskSeq_RequestEndCurrent(void)
{
    const SeqState st = EmmTaskQueue_GetSeqState();
    if (st != RUNNING && st != PAUSED) {
        /* 幂等：没在跑就当作已经结束了，不报错（与 KWQ1 一致）。 */
        Serial.printf("[任务序列] END CUR：当前状态 %s，无需结束\n",
                      EmmTaskQueue_SeqStateToStr(st));
        return false;
    }
    if (s_endMode != SEQ_END_NONE) {
        Serial.println("[任务序列] END CUR：已有结束请求在处理，本次忽略");
        return true;
    }

    s_endMode = SEQ_END_CURRENT;
    s_endInterrupt = true;
    /*
     * 必须清掉"暂停中断步"标记：`END` 会接管这条任务，后面回收/复位各步要能正常跑。
     * 不清的话 `emmSeqHandlePauseResume()` 在回收第一步就会命中"从暂停恢复（重做模式）"
     * 分支返回 false，把回收步自己掐断。
     */
    s_pausedMidStep = false;
    /*
     * **2026/10/27**：`END` 优先于 `SEQ PAUSE`。若暂停流程（回收/归位）还没走完就来了
     * 结束请求，把暂停标志清掉 —— 否则状态机在回收完成后会按暂停路径转去归位，
     * 而不是按 END 路径推进/复位。
     */
    s_pauseRequested = false;
    s_pauseParked = false;
    Serial.println("[任务序列] END CUR：已请求结束当前任务"
                   "（立即停机 → 回收 → 下一条；已是末条则复位）");

    /* 命令侧先广播一次停机，不等 10 ms 轮询 —— 满足"电机动作立即停止"。 */
    emmSeqBroadcastStop("END CUR");
    return true;
}

bool EmmTaskSeq_RequestEndAll(void)
{
    const SeqState st = EmmTaskQueue_GetSeqState();
    if (st != RUNNING && st != PAUSED) {
        Serial.printf("[任务序列] SEQ STOP：当前状态 %s，无需结束\n",
                      EmmTaskQueue_SeqStateToStr(st));
        return false;
    }
    if (s_endMode != SEQ_END_NONE) {
        Serial.println("[任务序列] SEQ STOP：已有结束请求在处理，本次忽略");
        return true;
    }

    s_endMode = SEQ_END_ALL;
    s_endInterrupt = true;
    /* 同 `END CUR`：清掉暂停中断标记，否则回收/复位第一步会被"恢复重做"分支掐断。 */
    s_pausedMidStep = false;
    /*
     * **2026/10/27**：`SEQ STOP` 优先于 `SEQ PAUSE`。暂停流程还没走完就来了停止请求时，
     * 把暂停标志清掉 —— 否则回收完成后状态机会按暂停路径转去归位，而不是按 STOP 的
     * "清空队列 + 复位"收尾。
     */
    s_pauseRequested = false;
    s_pauseParked = false;
    Serial.println("[任务序列] SEQ STOP：已请求结束全部任务"
                   "（立即停机 → 回收 → 清空队列 → 复位）");

    emmSeqBroadcastStop("SEQ STOP");
    return true;
}

/**
 * @brief `END CUR`：结束当前任务，回收后继续下一条。
 * @details 应答：
 *            - 结构电源未接通        → `ERR @<seq> 109`（`NO_STEP_PWR`）；
 *            - 序列未在运行（幂等）  → `OK @<seq> END CUR`（无动作）；
 *            - 已受理                → `OK @<seq> END CUR`。
 *          受理后实际动作在后台任务里完成，进度看串口日志；
 *          回收/复位过程中**不再**回复任何帧，收尾仍发 `EVT 300`（若已是末条）。
 */
bool cmdEndCur(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    if (cmd_power_off) {
        Serial.println("[任务序列] END CUR 被拒绝：结构电源未接通");
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }

    (void)EmmTaskSeq_RequestEndCurrent();
    sendWithCRC("OK @%d END CUR", seq);
    return true;
}

/**
 * @brief `ESTOP`：急停（**2026/10/27 新增**，接管原 `SEQ STOP` 的"全丢"语义）。
 * @details 与 `SEQ STOP` 的分工：
 *            - `SEQ STOP`：**收尾式**停止 —— 立即停机 → 回收（已取样时）→
 *              清空队列 → 复位（2 号回原点 + 4 号样品盘重新标定），耗时可能几十秒，
 *              收尾信号是 `EVT 300 SEQ ALL DONE`；
 *            - `ESTOP`   ：**立刻就地**停止 —— 广播停机 + 清空队列 + 回 `STOPPED`，
 *              机械**停在原地**，既不回收也不复位，给"人已到场、必须马上停"的场合。
 *
 *          应答：**任何状态都回 `OK @<seq> ESTOP`，永不拒绝**。急停是最后一道保险，
 *          不能因为"结构电源未接通"或"序列本来就没在跑"就报错 —— 上位机可能据此
 *          误判成"没执行"而反复下发。断电时也照常走一遍：广播停机可能因总线没电
 *          而下发失败，但状态与队列一定会被清干净。
 *
 *          与硬件急停（`exio.cpp` 的 GPIO1 下降沿）走的是**同一条路径**
 *          （都调 `EmmTaskSeq_Stop()`），区别只是触发源。
 */
bool cmdEstop(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    Serial.println("[任务序列] ESTOP：急停请求，立即停止全部电机（不回收、不复位）");

    /*
     * **2026/10/27（用户拍板）**：`ESTOP` = "**全量复位，但不断结构电源**"。
     * 与 `SET POWER OFF` 的差别只剩"不切电源"这一条，所以顺序照抄
     * `sysPowerOffCleanup()`：先置中止请求 → 再停序列（含广播停机）→ 最后清零点。
     *
     * **为什么必须先置中止请求**：`EmmSystemHome_RequestStop()` 只是"置标志、
     * 当前步骤结束后停止"，不是立即停电机；真正的"立即停"靠 `EmmTaskSeq_Stop()`
     * 里的 `gEmm.stopBroadcast()`。但若不先置标志，整机回零在下一步检查点
     * 看不到中止请求，会继续发新命令 —— 等于急停只停了"这一帧"。
     * `SysInit_Abort()` 同理（系统初始化的实质就是启动整机回零）。
     *
     * **为什么要清 `sys_is_homed`**：急停属于异常终止，现场很可能有人手动
     * 盘动过机构，绝对位置不再可信。清掉之后 `SEQ START` 会回 `104 NO_HOME`，
     * 逼操作员先 `SET EMM SYSTEM HOME` 回零再干活。
     *
     * ⚠️ `s_emmHomeBusy` / `s_sysInitBusy` 不会在这里被强制清零（那两个标志
     *    只能由各自的流程任务收尾）—— 所以急停后立刻发 `SET EMM SYSTEM HOME`
     *    可能回 `110`，等几秒（回零任务走到检查点）即可。
     */
    EmmSystemHome_RequestStop();
    SysInit_Abort();
    EmmTaskSeq_Stop();          /* 停序列 + 清队列 + 复位状态机 + 广播停机 + 黄灯 */
    sys_is_homed = false;

    sendWithCRC("OK @%d ESTOP", seq);
    return true;
}

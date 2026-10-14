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

#include <string.h>

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

/** 队列全部跑完时上报给上位机的"事件帧"序号（与 KWQ1 一致，固定 100）。 */
#define EMM_TASK_SEQ_EVT_SEQ           100

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
#define EMM_TASK_SEQ_MOVE_TIMEOUT_MS   10000U

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
/** 旋转电机（A=2）位置：取样位正上方。 */
#define EMM_SEQ_ROT_POS_SAMPLE         985U
/** 旋转电机（A=2）位置：检测位正上方（超过限位，需提前下降电机）。 */
#define EMM_SEQ_ROT_POS_DETECT         2850U
/** 旋转电机（A=2）位置：回收位第一段（逆时针 120）。 */
#define EMM_SEQ_ROT_POS_DISCARD_1      300U
/** 旋转电机（A=2）位置：回收位第二段（再逆时针 80 → 200）。 */
#define EMM_SEQ_ROT_POS_DISCARD_2      240U

/** 升降电机（A=3）位置：安全高度（= 0）。 */
#define EMM_SEQ_LIFT_POS_SAFE          0U
/* 安全高度2（检测位临时用） */
#define EMM_SEQ_LIFT_POS_SAFE_2        5000U
/** 升降电机（A=3）位置：与取样位旋转同步的补偿高度。 */
#define EMM_SEQ_LIFT_POS_SAMPLE_SYNC   985U
/** 升降电机（A=3）位置：取样高度 1（离针孔约 50 mm）。 */
#define EMM_SEQ_LIFT_POS_SAMPLE_1      38000U
/** 升降电机（A=3）位置：取样高度 2（慢速受阻检测终点）。 */
#define EMM_SEQ_LIFT_POS_SAMPLE_2      40000U
/* 检测高度 */
#define EMM_SEQ_LIFT_POS_DETECT        30000U
/** 升降电机（A=3）位置：回收位第一段（先降 1000）。 */
#define EMM_SEQ_LIFT_POS_DISCARD_1     1000U
/** 升降电机（A=3）位置：回收位第二段（降至 35000）。 */
#define EMM_SEQ_LIFT_POS_DISCARD_2     40000U
/** 升降电机（A=3）位置：回收位第三段（升回 30000）。 */
#define EMM_SEQ_LIFT_POS_DISCARD_3     38000U

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
        "TASK_RESET_POS"          /* TASK_RESET_POS          */
    };

    if ((int)p < 0 || (size_t)p >= (sizeof(kNames) / sizeof(kNames[0]))) {
        return "UNKNOWN";
    }
    return kNames[(int)p];
}

/**
 * @brief 切换到指定执行阶段，并把子步骤索引、计时与运动描述全部清零。
 * @details 计时基准 `s_lastTickMs` 置 0，下一个轮询周期会重新取 `millis()`
 *          作为起点，因此阶段切换本身不会引入额外耗时。
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
    Serial.println("[任务序列] ========== 全部任务完成 ==========");

    /* 与 KWQ1 相同的事件帧：@100 + TASK_DONE(300)，供上位机识别序列结束。 */
    sendWithCRC("EVT @%d %d SEQ ALL DONE", EMM_TASK_SEQ_EVT_SEQ, TASK_DONE);

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
 * @brief 阶段处理器返回 `SEQ_STEP_FAILED` 时的统一处置。
 * @details 只有序列**仍在 RUNNING** 时才按"失败"收尾；若已经被 `SEQ STOP` /
 *          `SET POWER OFF` 置成 `STOPPED`，说明是主动中止，不再重复报失败。
 */
static void emmSeqHandleStepFailure(const char *label, const char *reason)
{
    if (EmmTaskQueue_GetSeqState() == RUNNING) {
        emmTaskSeqStepFailed(label, reason);
    } else {
        Serial.printf("[任务序列] %s 未完成（序列已被中止），不按失败收尾\n", label);
    }
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
    Serial.printf("[任务序列] %s：A=%u 已下发停止命令，状态=%s\n",
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
            Serial.printf("[任务序列] %s：A=%u 已停稳（实时位置=%ld）\n",
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

        Serial.printf("[任务序列] 恢复：重发 %s A=%u DIR=%u RPM=%u ACC=%u 目标=%lu\n",
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

    /* 从暂停恢复：重发本步全部命令，继续走到原目标。 */
    if (st == RUNNING && s_pausedMidStep) {
        s_pausedMidStep = false;
        Serial.println("[任务序列] 收到恢复请求：重发本步命令，从断点继续");
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
            Serial.printf("[任务序列] %s：已从暂停恢复，本步计时重新开始\n", label);
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
                Serial.printf("[任务序列] %s：A=%u 到位（位置=%ld 误差=%ld）\n",
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
            Serial.printf("[任务序列] %s：已从暂停恢复，本步计时重新开始\n", label);
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
                Serial.printf("[任务序列] %s：A=%u 回零完成\n",
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
    Serial.printf("[任务序列] %s：A=%u DIR=%u RPM=%u ACC=%u PULSE=%lu MODE=%u\n",
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
    Serial.printf("[任务序列] %s：同步缓存间隔 %u ms（驱动器需静默时间）\n",
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
 */
static bool emmSeqMoveSyncStep(uint8_t addrA, uint8_t addrB, uint8_t dir,
                               uint16_t rpm, uint8_t acc,
                               uint32_t targetA, uint32_t targetB,
                               const char *label)
{
    Serial.printf("[任务序列] %s：A=%u→%lu + A=%u→%lu 同步定位 DIR=%u RPM=%u ACC=%u\n",
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
 */
static bool emmSeqHomeSyncStep(uint8_t addrA, uint8_t addrB, uint8_t homeMode,
                               const char *label)
{
    Serial.printf("[任务序列] %s：A=%u + A=%u 同步单圈回零 MODE=%u\n",
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
 * @param homeMode 回零模式（0=单圈**就近**回零；1=单圈回零）。
 * @param label    本步中文说明（日志用，必须指向静态存储）。
 * @return true 回零完成；false 失败或序列被中止。
 */
static bool emmSeqHomeStep(uint8_t addr, uint8_t homeMode, const char *label)
{
    Serial.printf("[任务序列] %s：A=%u 单圈回零 MODE=%u\n",
                  label, (unsigned)addr, (unsigned)homeMode);

    emmSeqClearMotions();
    emmSeqRecordMotion(addr, 1U, 0U, 0U, EMM_SEQ_ROT_POS_HOME, true, homeMode, label);

    if (!emmSeqHomeSend(addr, homeMode, false /* 单台，不缓存 */, label)) {
        return false;
    }
    return emmSeqWaitHomingAll(label);
}

/**
 * @brief 写回零参数（0x4C/0xAE）。
 * @details 用户给的复位命令把 8 个字段全部写全了（`MODE/DIR/RPM/TIMEOUT/
 *          STALL_RPM/STALL_MA/STALL_MS/AUTO`），所以这里直接整组下发，
 *          不做"读-改-写"。始终 `SAVE=0`（只改运行参数，不写掉电保存区）。
 * @return true 写入成功。
 */
static bool emmSeqWriteHomeParam(uint8_t addr, uint8_t mode, uint8_t dir,
                                 uint16_t rpm, uint32_t timeoutMs,
                                 uint16_t stallRpm, uint16_t stallMa,
                                 uint16_t stallMs, bool autoHome,
                                 const char *label)
{
    Serial.printf("[任务序列] %s：A=%u 写回零参数 MODE=%u DIR=%u RPM=%u "
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
    Serial.printf("[任务序列] %s：A=%u 当前位置清零\n", label, (unsigned)addr);

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
 * @brief 把当前位置设为单圈回零零点（0x93/0x88）。
 * @details 用户步骤 1 的第 2、3 条：`SET EMM ORIGIN ZERO A=2/3 SAVE=0`。
 */
static bool emmSeqSetOriginZero(uint8_t addr, bool save, const char *label)
{
    Serial.printf("[任务序列] %s：A=%u 设为单圈回零零点 SAVE=%u\n",
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
    Serial.printf("[任务序列] %s：广播同步触发（A=ALL）\n", label);

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
    Serial.printf("[任务序列] %s：触发后静默 %u ms 再轮询\n",
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
    Serial.printf("[任务序列] %s：【喷头识别：占位，下一阶段实现】\n", label);
}

/**
 * @brief 占位步骤：受阻检测布防/撤防（用户要求"暂不接"）。
 * @note 现有 `EmmLiftCurrentMonitor`（1 ms 轮询相电流 / 460 mA / 20 ms 确认）
 *       保持不动，仍可由上位机手动 `SET EMM LIFT MONITOR ARM=1` 布防；
 *       序列侧按用户要求只打印占位日志，不做任何布防动作。
 */
static void emmSeqPlaceholderLiftMonitor(const char *label, bool arm)
{
    Serial.printf("[任务序列] %s：【受阻检测%s：占位，暂不接】\n",
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
 * @brief 序列前导（`TASK_PREAMBLE`）：建立 2/3 号单圈零点。
 * @details 对应实机验证过的命令组 1：
 *            `SET EMM ZERO A=3`
 *            `SET EMM ORIGIN ZERO A=2 SAVE=0`
 *            `SET EMM ORIGIN ZERO A=3 SAVE=0`
 *          `SEQ START` 前置检查全部通过后**只做一次**（用户确认）。
 *
 *          样品盘不在此处处理：它的零点由**整机回零**一次性建立（01 号工位 =
 *          绝对脉冲 0），此后一直有效；本序列每次都按绝对位置走（见
 *          `emmSeqRunTrayPos`），因此不需要任何软件工位计数，也不随队列复位。
 * @return DONE 完成；FAILED 失败或序列被中止。
 */
static SeqStepResult emmSeqRunPreamble(void)
{
    Serial.println("[任务序列] 【前导】建立 2/3 号单圈零点（SEQ START 后只做一次）");

    /* 1.1 3 号（升降）当前位置清零。 */
    if (!emmSeqZeroPosition(EMM_LIFT_MOTOR, "前导 1/3 3号位置清零")) {
        return SEQ_STEP_FAILED;
    }
    /* 1.2 2 号（旋转）当前位置设为单圈回零零点。 */
    if (!emmSeqSetOriginZero(EMM_ROTARY_MOTOR, false, "前导 2/3 2号设单圈零点")) {
        return SEQ_STEP_FAILED;
    }
    /* 1.3 3 号（升降）当前位置设为单圈回零零点。 */
    if (!emmSeqSetOriginZero(EMM_LIFT_MOTOR, false, "前导 3/3 3号设单圈零点")) {
        return SEQ_STEP_FAILED;
    }

    Serial.println("[任务序列] 【前导】完成：2/3 号单圈零点已建立");
    return SEQ_STEP_DONE;
}

/**
 * @brief Ⅱ 取样位（`TASK_MOVE_POS`）。
 * @details 对应实机验证过的命令组 2~8：
 *            2. 2 号旋转至取样位 + 3 号同向补偿，**同步**启动
 *            3. 3 号快速下降至取样高度 1（38000，离针孔约 50 mm）
 *            4. 开启受阻检测（**占位**）
 *            5. 3 号以 RPM=1 慢速下降至取样高度 2（40000，受阻检测区段）
 *            6. 撤销受阻检测（**占位**）
 *            7. 3 号抬回安全高度（0）
 *            8. 喷头识别（**占位**）
 *          **2026/10/14 用户要求**：下降到取样点后**不再等待**（原"停留 T 秒"
 *          已删除），停留改到 `TASK_DETECT_POS`（检测位）。因此本阶段的所有
 *          子步骤都是阻塞式完成，执行完立刻 `emmSeqNextStep()` 进入下一个。
 * @return RUNNING 继续；DONE 完成；FAILED 失败或序列被中止。
 */
static SeqStepResult emmSeqRunMovePos(void)
{
    switch (s_subStep) {
    /* ---- 子步骤 0：2+3 号同步转到取样位 ---- */
    case 0:
        /*
         * 2 号转到取样位（950），3 号同向同量补偿（也是 950）—— 因为 2 号旋转会
         * 带动丝杆升降，3 号必须一起动才能把电离头高度保持在安全高度。
         */
        if (!emmSeqMoveSyncStep(EMM_ROTARY_MOTOR, EMM_LIFT_MOTOR, 0U, 15U, 20U,
                                EMM_SEQ_ROT_POS_SAMPLE,
                                EMM_SEQ_LIFT_POS_SAMPLE_SYNC,
                                "取样位 1/8 旋转+补偿同步定位")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 1：3 号快速下降至取样高度 1 ---- */
    case 1:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
                            EMM_SEQ_LIFT_POS_SAMPLE_1, 1U,
                            EMM_SEQ_LIFT_POS_SAMPLE_1,
                            "取样位 2/8 快降至取样高度1")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 2：开启受阻检测（占位） ---- */
    case 2:
        emmSeqPlaceholderLiftMonitor("取样位 3/8", true);
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 3：3 号慢速下降至取样高度 2 ---- */
    case 3:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 10U, 10U,
                            EMM_SEQ_LIFT_POS_SAMPLE_2, 1U,
                            EMM_SEQ_LIFT_POS_SAMPLE_2,
                            "取样位 4/8 慢降至取样高度2")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 4：撤销受阻检测（占位） ---- */
    case 4:
        emmSeqPlaceholderLiftMonitor("取样位 5/8", false);
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 5：3 号抬回安全高度 ---- */
    case 5:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 0U, 180U, 40U,
                            EMM_SEQ_LIFT_POS_SAFE, 1U, EMM_SEQ_LIFT_POS_SAFE,
                            "取样位 6/8 抬回安全高度")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 6：喷头识别（占位） ---- */
    case 6:
        emmSeqPlaceholderNozzleCheck("取样位 7/8");
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 7：本阶段完成 ---- */
    case 7:
    default:
        Serial.println("[任务序列] 取样位 8/8 完成");
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
        Serial.printf("[任务序列] 样品盘：本任务已在 %02u 号工位（当前位置 %ld），无需转动\n",
                      (unsigned)targetSlot, (long)curPos);
        return SEQ_STEP_DONE;
    }

    const uint8_t dir = (delta > 0) ? 1U : 0U;
    Serial.printf("[任务序列] 样品盘：定位到本任务工位 %02u（当前位置 %ld，"
                  "绝对脉冲 → %lu，DIR=%u）\n",
                  (unsigned)targetSlot, (long)curPos,
                  (unsigned long)dstAbs, (unsigned)dir);

    if (!emmSeqMoveStep(EMM_SAMPLE_TRAY_MOTOR, dir, EMM_SEQ_TRAY_RPM,
                        EMM_SEQ_TRAY_ACC, dstAbs, 1U /* MODE=1 绝对 */,
                        dstAbs, "样品盘定位")) {
        return SEQ_STEP_FAILED;
    }

    Serial.printf("[任务序列] 样品盘：已到 %02u 号工位\n", (unsigned)targetSlot);
    return SEQ_STEP_DONE;
}

/**
 * @brief Ⅲ 检测位（`TASK_DETECT_POS`）。
 * @details 对应实机验证过的命令组 10（**2026/10/14 起加入"停留 T 分钟"**）：
 *            2 号旋转至检测位（2850）→ 3 号降至检测高度（30000）
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
        if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 10U, 10U, EMM_SEQ_ROT_POS_DETECT,
                            1U, EMM_SEQ_ROT_POS_DETECT, "检测位 1/4 旋转至检测位")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 1：3 号下降至检测高度 ---- */
    case 1:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U, EMM_SEQ_LIFT_POS_DETECT,
                            1U, EMM_SEQ_LIFT_POS_DETECT, "检测位 2/4 下降至检测位")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

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
            Serial.printf("[任务序列] 检测位 3/4 开始停留 %.3f min（原始 T=%.3f min）\n",
                          (double)dwellMin, (double)t.timeSec);
        }

        if (s_stepElapsedMs < dwellMs) {
            /* 每 EMM_TASK_SEQ_DWELL_PROGRESS_MS（1 分钟）打一条进度，避免刷屏。 */
            if (s_stepElapsedMs >= s_nextProgressMs) {
                Serial.printf("[任务序列] 检测位 3/4 停留中：%.1f/%.1f min\n",
                              (double)(s_stepElapsedMs / 60000.0f),
                              (double)(dwellMs / 60000.0f));
                s_nextProgressMs = s_stepElapsedMs + EMM_TASK_SEQ_DWELL_PROGRESS_MS;
            }
            return SEQ_STEP_RUNNING;
        }

        Serial.printf("[任务序列] 检测位 3/4 停留结束（%.3f min）\n",
                      (double)(dwellMs / 60000.0f));
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;
    }

    /* ---- 子步骤 3：3 号抬回安全高度2 ---- */
    case 3:
        if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U, EMM_SEQ_LIFT_POS_SAFE_2,
                            1U, EMM_SEQ_LIFT_POS_SAFE_2, "检测位 4/4 抬回安全高度2")) {
            return SEQ_STEP_FAILED;
        }
        emmSeqNextStep();
        return SEQ_STEP_RUNNING;

    /* ---- 子步骤 4：本阶段完成 ---- */
    case 4:
    default:
        Serial.println("[任务序列] 检测位 完成");
        return SEQ_STEP_DONE;
    }
}

/**
 * @brief Ⅳ 回收位 / 抛弃位（`TASK_DISCARD_POS`）。
 * @details 对应实机验证过的命令组 11（2026/10/10 用户改：2 号两段旋转方向由逆时针
 *          改为**顺时针** `DIR=0`）：
 *            3 号先降 1000 → 2 号顺时针转到 280 → 喷头识别（**占位**）
 *            → 3 号降至 35000 → 受阻检测（**占位**）→ 2 号顺时针再转 80 到 200
 *            → 撤销受阻检测（**占位**）→ 3 号回升到 30000 → 3 号抬回安全高度
 *            → 喷头识别（**占位**，用户给的机械流程在此处有一次）。
 * @return DONE 完成；FAILED 失败或序列被中止。
 */
static SeqStepResult emmSeqRunDiscardPos(void)
{
    // if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
    //                     EMM_SEQ_LIFT_POS_DISCARD_1, 1U,
    //                     EMM_SEQ_LIFT_POS_DISCARD_1,
    //                     "回收位 1/10 升降预备")) {
    //     return SEQ_STEP_FAILED;
    // }
    if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 10U, 10U,
                        EMM_SEQ_ROT_POS_DISCARD_1, 1U,
                        EMM_SEQ_ROT_POS_DISCARD_1,
                        "回收位 2/10 旋转至回收位")) {
        return SEQ_STEP_FAILED;
    }
    emmSeqPlaceholderNozzleCheck("回收位 3/10");

    if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
                        EMM_SEQ_LIFT_POS_DISCARD_2, 1U,
                        EMM_SEQ_LIFT_POS_DISCARD_2,
                        "回收位 4/10 下降至回收位")) {
        return SEQ_STEP_FAILED;
    }
    emmSeqPlaceholderLiftMonitor("回收位 5/10", true);

    if (!emmSeqMoveStep(EMM_ROTARY_MOTOR, 0U, 10U, 10U,
                        EMM_SEQ_ROT_POS_DISCARD_2, 1U,
                        EMM_SEQ_ROT_POS_DISCARD_2,
                        "回收位 6/10 旋转至第二段")) {
        return SEQ_STEP_FAILED;
    }
    emmSeqPlaceholderLiftMonitor("回收位 7/10", false);

    if (!emmSeqMoveStep(EMM_LIFT_MOTOR, 1U, 180U, 40U,
                        EMM_SEQ_LIFT_POS_DISCARD_3, 1U,
                        EMM_SEQ_LIFT_POS_DISCARD_3,
                        "回收位 8/10 回升至38000")) {
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
 * @brief 复位（`TASK_RESET_POS`）：4 号就近单圈回零 + 2+3 号同步单圈回零至初始位。
 * @details **整条队列跑完时只做一次**（用户确认）。命令组 12 于 2026/10/10 按用户
 *          要求扩展：**4 号样品盘也要就近单圈回零，且先执行**。
 *            12.0 4 号样品盘 `HOME MODE=0`（单圈**就近**回零）—— **最先做**
 *            12.1 写 2 号回零参数（STALL_MA=450）
 *            12.2 写 3 号回零参数（STALL_MA=400）
 *            12.3~12.5 2+3 号 `HOME MODE=1 SYNC=1` 缓存 → `SYNC A=ALL` 一起启动
 *
 *          **顺序说明（用户要求"4 号先执行"）**：4 号先回零，再回 2/3 号。这样
 *          复位结束时样品盘已回到由就近原点建立的已知坐标系（与整机回零步 12
 *          用的是同一个物理原点标记），下次 `SEQ START` 的绝对工位定位才准。
 *
 *          **失败处理（2026/10/10 用户要求，分三档）**：
 *            - 4 号回零失败：与整机回零步 12 一致 —— **只打印、不中止**（非关键步骤）；
 *            - 2/3 号**同步回零**失败：**也不中止**（用户原话"不管最后 2、3 号一起
 *              单圈回零是否失败，也是直接清空任务序列"）→ 先广播停机兜一道，再照常
 *              走收尾，让 `emmTaskSeqFinishAll()` 的 `EmmTaskQueue_Clear()` 一定执行；
 *            - 2/3 号**写回零参数**失败：仍按失败中止（`return SEQ_STEP_FAILED`，
 *              保留队列便于排查）—— 用户只点名了"同步回零"这一步，未含参数写入。
 *
 * @return 只要 2/3 号参数写入成功，**恒返回 DONE**（保证队列一定清空）；参数写入
 *         失败才返回 FAILED。
 */
static SeqStepResult emmSeqRunReset(void)
{
    Serial.println("[任务序列] 【复位】4 号样品盘就近单圈回零 → 2/3 号单圈回零至初始位"
                   "（整条队列只做一次）");

    /* 12.0 【用户要求：4 号先执行】4 号样品盘就近单圈回零（MODE=0）。失败忽略。 */
    if (!emmSeqHomeStep(EMM_SAMPLE_TRAY_MOTOR, 0U,
                        "复位 1/6 4号样品盘就近单圈回零")) {
        Serial.println("[任务序列] 【复位】4 号样品盘回零未成功，按配置忽略并继续");
    }

    /* 12.1 / 12.2 写回零参数（SAVE=0）。两台碰撞电流不同，用户实测 450 / 400。
       注：参数写入失败仍按"失败"处理（保留队列便于排查）——用户 2026/10/10 只
       要求"2/3 号同步回零失败也不中止"，未包含参数写入这一步。 */
    if (!emmSeqWriteHomeParam(EMM_ROTARY_MOTOR, 0U, 1U, 15U, 5000U, 5U, 450U, 10U,
                              false, "复位 2/6 2号写回零参数")) {
        return SEQ_STEP_FAILED;
    }
    if (!emmSeqWriteHomeParam(EMM_LIFT_MOTOR, 0U, 1U, 15U, 5000U, 5U, 400U, 10U,
                              false, "复位 3/6 3号写回零参数")) {
        return SEQ_STEP_FAILED;
    }

    /*
     * 12.3~12.5 两台缓存 HOME MODE=1 后广播一起启动，再等两台都完成。
     * 用户 2026/10/10 要求：**2/3 号同步回零失败也不中止序列**，直接走收尾清空
     * 队列（`emmTaskSeqFinishAll()` 里的 `EmmTaskQueue_Clear()`）。但"失败/超时"
     * 时电机可能仍在动（`emmSeqWaitHomingAll` 超时只返回 false、不主动停），所以
     * 这里仍要**广播停机**兜一道；中止路径下 `EmmTaskSeq_Stop()` 已广播过一次，
     * 重复一帧无害。
     */
    bool home2n3Ok = true;
    if (!emmSeqHomeSyncStep(EMM_ROTARY_MOTOR, EMM_LIFT_MOTOR, 1U,
                            "复位 4~6/6 2+3号同步单圈回零")) {
        home2n3Ok = false;
        Serial.println("[任务序列] 【复位】2/3 号同步单圈回零未成功：按用户要求不中止序列，"
                       "广播停机后照常收尾清空");
        if (Rs485Bus_Take(EMM_TASK_SEQ_BUS_TIMEOUT_MS)) {
            const EmmV5::Status stopStatus = gEmm.stopBroadcast(false);
            Rs485Bus_Give();
            Serial.printf("[任务序列] 【复位】已广播停止全部 EMM 电机，状态=%s\n",
                          EmmV5::statusToString(stopStatus));
        } else {
            Serial.println("[任务序列] 【复位】RS485 总线繁忙，停止全部电机的命令未下发");
        }
    }

    if (home2n3Ok) {
        Serial.println("[任务序列] 【复位】完成：4 号已就近单圈回零、2/3 号已回到初始位");
    } else {
        Serial.println("[任务序列] 【复位】结束（2/3 号回零未成功，已按用户要求照常收尾并清空队列）");
    }
    /* 恒返回 DONE：让 `emmSeqFinishIfRunning()` → `emmTaskSeqFinishAll()` 一定执行
       → `EmmTaskQueue_Clear()` 一定被调用（用户要求"不管 2/3 号回零是否失败都清空"）。 */
    return SEQ_STEP_DONE;
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
            Serial.println("[任务序列] 队列已跑完，进入复位阶段（2/3 号回初始位）");
            emmTaskSeqEnterPhase(TASK_RESET_POS);
            return;
        }

        const SeqStepResult r = emmSeqRunReset();
        if (r == SEQ_STEP_DONE) {
            emmSeqFinishIfRunning();
        } else if (r == SEQ_STEP_FAILED) {
            emmSeqHandleStepFailure("复位", "写回零参数失败（2/3 号同步回零失败已改为不中止）");
        }
        return;
    }

    /* ---------------- 取当前任务 ---------------- */
    const uint8_t idx = EmmTaskQueue_Index();
    const uint8_t total = EmmTaskQueue_Count();

    EmmTask t;
    if (!EmmTaskQueue_GetTask(idx, &t)) {
        /* 兜底：正常情况下 Index < Count 且 Count <= 10，不会走到这里。 */
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
            emmSeqHandleStepFailure("前导", "建立 2/3 号单圈零点失败");
        }
        break;
    }

    /* ---------------------------------------------------------------- */
    case TASK_IDLE:
        /*
         * 进入本条任务。KWQ1 在此处做"溶剂切换 + 移动到 Axx 点位"；KWQ3 在这里
         * 只打印任务信息、点绿灯，真正的动作交给下面的位置阶段。
         */
        Serial.printf("[任务序列] ===== 任务 %u/%u 开始：类型=%c 工位=%02u T=%.3f min =====\n",
                      (unsigned)(idx + 1), (unsigned)total,
                      t.type, (unsigned)t.slot, (double)t.timeSec);
        rygLedOn(RYG_COLOR_GREEN);
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
            Serial.printf("[任务序列] ===== 任务 %u/%u 完成 =====\n",
                          (unsigned)(idx + 1), (unsigned)total);
            /* 只有序列仍在跑时才推进游标；被中止时队列已被清空，不动它。 */
            if (EmmTaskQueue_GetSeqState() == RUNNING) {
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
            emmSeqHandleStepFailure("复位", "写回零参数失败（2/3 号同步回零失败已改为不中止）");
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
     *   - `SET EMM LIFT MONITOR`：只切换本地布防标志，不产生总线事务。
     * 注意这里**不**放行 `SET EMM SYSTEM HOME`——序列运行期间不允许启动整机
     * 回零，否则两套复合动作会同时抢三台电机。
     */
    if (strcmp(commandName, "SET EMM SYSTEM HOME STOP") == 0 ||
        strcmp(commandName, "SET EMM LIFT MONITOR") == 0) {
        return false;
    }

    /* 序列运行期间独占机械机构：拦截所有会触发动作的 SET EMM 命令。 */
    return strncmp(commandName, "SET EMM", 7) == 0;
}

/* ==========================================================================
 * 对外接口：上位机命令
 * ========================================================================== */

bool cmdSeqStart(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    Serial.println("[任务序列] ========== SEQ START 前置检查开始 ==========");

    /* ① 结构电源必须已合上（与 L1 运动命令一致）。 */
    if (cmd_power_off) {
        Serial.println("[任务序列] 前置检查 1/9 结构电源：未接通");
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }
    Serial.println("[任务序列] 前置检查 1/9 结构电源：已接通");

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
    Serial.println("[任务序列] 前置检查 2/9 序列状态：空闲");

    /* ③ 队列不能为空。 */
    if (EmmTaskQueue_Count() == 0) {
        Serial.println("[任务序列] 前置检查 3/9 任务队列：为空");
        sendWithCRC("ERR @%d %d", seq, TASK_NULL);
        return false;
    }
    Serial.printf("[任务序列] 前置检查 3/9 任务队列：%u 条\n",
                  (unsigned)EmmTaskQueue_Count());

    /* ④ 上位机心跳在线（PING 超时未恢复时不允许开始跑任务）。 */
    if (!ping_on_line) {
        Serial.println("[任务序列] 前置检查 4/9 心跳在线：不在线");
        sendWithCRC("ERR @%d %d", seq, NO_HEARTBEAT);
        return false;
    }
    Serial.println("[任务序列] 前置检查 4/9 心跳在线：通过");

    /* ⑤ 舱门已关闭。 */
    if (!checkDoorState()) {
        Serial.println("[任务序列] 前置检查 5/9 舱门关闭：未到位");
        rygLedBlink(RYG_COLOR_YELLOW, EMM_TASK_SEQ_PAUSE_BLINK_MS);
        sendWithCRC("ERR @%d %d", seq, NO_DOOR);
        return false;
    }
    Serial.println("[任务序列] 前置检查 5/9 舱门关闭：通过");

    /* ⑥ 样品盘已放置。 */
    if (!checkSampleState()) {
        Serial.println("[任务序列] 前置检查 6/9 样品盘到位：未到位");
        rygLedBlink(RYG_COLOR_YELLOW, EMM_TASK_SEQ_PAUSE_BLINK_MS);
        sendWithCRC("ERR @%d %d", seq, NO_SAMPLE);
        return false;
    }
    Serial.println("[任务序列] 前置检查 6/9 样品盘到位：通过");

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
    Serial.println("[任务序列] 前置检查 7/9 电机使能：占位默认通过（未接真实检测）");

    /* ⑧ 控制板温度未超限（取 5 路驱动温度最大值与 SET TEMP 阈值比较）。 */
    const float maxDriverTemp = getMaxDriverTemp();
    if (maxDriverTemp > max_temp) {
        Serial.printf("[任务序列] 前置检查 8/9 控制板温度：超限（最高 %.1f℃ > %.1f℃）\n",
                      (double)maxDriverTemp, (double)max_temp);
        sendWithCRC("ERR @%d %d", seq, TEMP_HIGH);
        return false;
    }
    Serial.printf("[任务序列] 前置检查 8/9 控制板温度：通过（最高 %.1f℃ <= %.1f℃）\n",
                  (double)maxDriverTemp, (double)max_temp);

    /* ⑨ 整机已回零（由 EmmSystemHome 的 12 步序列全部成功后置位）。 */
    if (!sys_is_homed) {
        Serial.println("[任务序列] 前置检查 9/9 整机回零：尚未回零");
        sendWithCRC("ERR @%d %d", seq, NO_HOME);
        return false;
    }
    Serial.println("[任务序列] 前置检查 9/9 整机回零：已建立零点");

    /* ---------------- 全部通过：先做前导建立零点，再从第 1 条开始 ---------------- */
    EmmTaskQueue_SetIndex(0);
    /*
     * 先进入 `TASK_PREAMBLE`：`SET EMM ZERO A=3` + 2/3 号设单圈零点（只做一次）。
     * 样品盘不在这里处理 —— 它的零点由整机回零一次性建立，本序列按绝对位置走。
     * 前导完成后状态机自动进入第 1 条任务的 `TASK_IDLE`。
     */
    emmTaskSeqEnterPhase(TASK_PREAMBLE);
    EmmTaskQueue_SetSeqState(RUNNING);

    Serial.printf("[任务序列] ========== SEQ START 通过，共 %u 条任务 ==========\n",
                  (unsigned)EmmTaskQueue_Count());
    sendWithCRC("OK @%d SEQ START", seq);
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

    if (EmmTaskQueue_GetSeqState() != RUNNING) {
        Serial.printf("[任务序列] SEQ PAUSE 被拒绝：当前状态 %s（非 RUNNING）\n",
                      EmmTaskQueue_SeqStateToStr(EmmTaskQueue_GetSeqState()));
        /* 110 SEQ_RUNNING 在此处语义为"序列状态不允许该操作"（用户确认复用）。 */
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }

    /*
     * 这里只置状态，**不动电机**：真正的"立即停机"由后台任务的等待循环发现
     * `PAUSED` 后执行（对正在运动的电机发停止命令并等停稳），这样命令回调
     * 依然是"零耗时"的，上位机可以立刻拿到 ACK。
     */
    EmmTaskQueue_SetSeqState(PAUSED);
    rygLedBlink(RYG_COLOR_YELLOW, EMM_TASK_SEQ_PAUSE_BLINK_MS);
    Serial.println("[任务序列] 已暂停：等待后台任务立即停机，计时冻结");
    sendWithCRC("OK @%d SEQ PAUSE", seq);
    return true;
}

bool cmdSeqResume(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

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

    /* 同样只置状态：后台任务发现 `RUNNING` 后会重发本步命令，从断点继续。 */
    EmmTaskQueue_SetSeqState(RUNNING);
    rygLedOn(RYG_COLOR_GREEN);
    Serial.println("[任务序列] 已恢复：后台任务将重发本步命令，计时从断点继续");
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
     * 与 KWQ1 一致：STOP 总是复位状态机并清空队列，且始终回 OK。
     * 收尾逻辑与 `SET POWER OFF` 共用 EmmTaskSeq_Stop()。
     */
    EmmTaskSeq_Stop();

    sendWithCRC("OK @%d SEQ STOP", seq);
    return true;
}

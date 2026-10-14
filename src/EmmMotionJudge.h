#ifndef EMM_MOTION_JUDGE_H
#define EMM_MOTION_JUDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "EmmV5.h"

/**
 * @file EmmMotionJudge.h
 * @brief EMM "运动是否完成 / 是否到位 / 是否回零完成" 判据的**唯一实现**。
 *
 * @details 本工程有多处需要判断"电机动作做完没有"：
 *            - L2 任务序列：`EmmTaskSeq::emmSeqWaitArriveAll`（定位）、
 *              `emmSeqWaitHomingAll`（回零）、`emmSeqStopMotorAndWait`（停稳）；
 *            - L2 整机回零：`EmmSystemHome::emmHomeWaitHoming`（回零回退路径）、
 *              `emmHomeWaitMove`（定位，**暂未迁移**，见下）。
 *          这些函数的**轮询循环**必须各自保留（中止机制、单机/多机、超时来源、
 *          日志前缀都不同），但**判据本身只应有一份** —— 否则必然分叉。
 *
 * @warning 本文件**只放判据**：纯函数、无 IO、无状态、不依赖 FreeRTOS。
 *          任何 `Rs485Bus_Take()` / `vTaskDelay()` / `Serial.printf()` 都不属于这里，
 *          那些留在各自的轮询循环里。这样才能保证"改判据只改一处"。
 *
 * @note 历史（2026/10/10）：
 *       1) 本文件诞生前，"位置到位"有两份实现 —— `EmmTaskSeq` 用 `pendingSeen`
 *          （两条证据），`EmmSystemHome` 用 `moved`（位置变 1 个计数就算起步过，
 *          太敏感）。两处行为不一致，排查时极易误判；本文件统一了常量与判据内核。
 *       2) 同日晚些时候，用户决定把**序列的定位判据简化**为"只看驱动器的到位标志
 *          `Prf_TF`"（理由见第四节）：序列每条命令都等 0x02 应答，命令真丢帧会在
 *          发送阶段就被发现并中止，不需要额外的"命令已生效证据"。
 *       `emmHomeWaitMove` 仍用旧的 `moved`，迁移列为第二批（会改变行为，需单独
 *       实机验证）。
 */

/* ==========================================================================
 * 一、单位换算：命令脉冲 ↔ 驱动器计数
 * --------------------------------------------------------------------------
 * `0x43` 读回的 `targetPosition` / `realtimePosition` / `positionError` 单位是
 * **驱动器计数**（65536 计数/圈）；而命令里的 `PULSE` 是**脉冲**（本机驱动器配置
 * 3200 脉冲/圈）。因此 1 脉冲 = 65536 / 3200 = **20.48 计数**。
 *
 * 该比例已用实机数据校准（2026/10/10）：序列命令 A=3 去绝对 950 脉冲，`0x43`
 * 回报 `TARGET` ≈ 19461 计数，而 950 × 20.48 = 19456 → 误差仅几个计数，
 * 说明比例与零点都自洽（详见《修改日志.md》§7.14）。
 * ========================================================================== */

/** 驱动器计数/圈。 */
#define EMM_JUDGE_COUNTS_PER_REV   65536L
/** 命令脉冲/圈（本机驱动器细分设置）。改驱动器细分时必须同步改这里。 */
#define EMM_JUDGE_PULSES_PER_REV   3200L

/**
 * @brief 命令脉冲 → 驱动器计数。
 * @note 用 int64 中间量，防止 40000 脉冲 × 65536 溢出 32 位。
 */
static inline int32_t emmJudgePulseToCounts(uint32_t pulses)
{
    return (int32_t)((int64_t)pulses * EMM_JUDGE_COUNTS_PER_REV /
                     EMM_JUDGE_PULSES_PER_REV);
}

/* ==========================================================================
 * 二、位置到位判据的常量
 * ========================================================================== */

/**
 * @brief 精细到位容差（驱动器计数）。
 * @note 8 计数 ≈ 8/65536×360 ≈ **0.044°**。与整机回零的
 *       `EMM_HOME_MOVE_TOLERANCE_PULSE`（同为 8）取同一量级。
 *       注意：**位置到位判据已不再使用它**（判据改为只看 `Prf_TF`，见第四节）；
 *       保留是因为样品盘"已在工位就不动"的提前跳过仍按它比容差。
 */
#define EMM_JUDGE_MOVE_TOL_COUNTS    8L

/* ==========================================================================
 * 三、起步保护
 * --------------------------------------------------------------------------
 * 刚下发（或刚恢复重发）时先等一段再判到位，避免"命令还没起步就被判到位"。
 * ========================================================================== */

/** @brief 加速时间 = RPM × (256 - ACC) × 50 µs，换算成 ms。 */
static inline uint32_t emmJudgeRampMs(uint16_t rpm, uint8_t acc)
{
    return (uint32_t)rpm * (uint32_t)(256U - (uint32_t)acc) * 50U / 1000U;
}

/**
 * @brief 起步保护时间 = max(加速时间 × 2, floorMs)。
 * @param floorMs 各模块自己的下限（序列 300 ms / 整机回零 300 ms），
 *                由调用方传入，因为"下限"属于模块策略，不是判据。
 */
static inline uint32_t emmJudgeGraceMs(uint16_t rpm, uint8_t acc, uint32_t floorMs)
{
    const uint32_t grace = emmJudgeRampMs(rpm, acc) * 2U;
    return (grace < floorMs) ? floorMs : grace;
}

/* ==========================================================================
 * 四、位置到位判据
 * --------------------------------------------------------------------------
 * 到位 = 两条同时成立：
 *   1. 已停稳（`realtimeRpm == 0`）；
 *   2. 驱动器报告"目标位置已到达"（`Prf_TF` = `MOTOR_POSITION_REACHED` 0x02）。
 *
 * **为什么这么简单（2026/10/10 用户决策，从"两条证据"简化而来）**
 * --------------------------------------------------------------------------
 * 旧版在 2 之外还要求一条"命令已生效"的证据（等待期间见过 |positionError| > 146
 * 计数，或驱动器当前目标 == 本步命令目标），用来防：
 *   "本步命令丢帧 → 驱动器仍挂旧目标 → 电机恰好停在旧目标上 → Prf_TF 照样置位
 *    → 假到位"。
 *
 * 但序列的每一条定位命令都走 `emmSeqPositionSend()` → `gEmm.positionMode()`，
 * **发完必须收到 0x02 应答**；命令真丢帧时应答也不会来，`emmSeqPositionSend()`
 * 直接返回失败、序列当场中止，**根本走不到判到位这一步**。因此那条"证据"在正常
 * 路径上是冗余的，按用户意见移除，判据回到"驱动器说到了就是到了"。
 *
 * @warning 该简化的**前提**：应答机制可靠 —— 不会出现"收到了 0x02、驱动器却没执行"。
 *          若将来在实机观察到这种情形，需要把"命令归属确认"（比对驱动器
 *          `targetPosition` 与本步命令目标）加回来。
 * ========================================================================== */

/** @brief 已停稳：实时转速为 0。 */
static inline bool emmJudgeStopped(const EmmV5::SystemState *st)
{
    return st->realtimeRpm == 0;
}

/** @brief 是否触发堵转保护（等待期间应判失败）。 */
static inline bool emmJudgeStallProtect(const EmmV5::SystemState *st)
{
    return (st->motorFlags & EmmV5::MOTOR_STALL_PROTECT) != 0;
}

/**
 * @brief 位置到位总判定：**已停稳 且 驱动器报告目标位置已到达**。
 * @note 判据的来历与简化理由见本节开头。停稳是廉价的保险：`Prf_TF` 置位时电机
 *       本应已停，但刚到窗口的那一瞬转速可能还没降到 0，多等一拍无害。
 */
static inline bool emmArriveReached(const EmmV5::SystemState *st)
{
    return emmJudgeStopped(st) &&
           ((st->motorFlags & EmmV5::MOTOR_POSITION_REACHED) != 0);
}

/* ==========================================================================
 * 五、回零完成判据（`0x3B` 回零状态标志）
 * --------------------------------------------------------------------------
 * `0x3B` 的 bit2 = Org_SF（正在回零）、bit3 = Org_CF（回零失败）。
 * **bit0/bit1 是编码器就绪位，默认恒为 1，必须屏蔽。**
 *
 * bit2/bit3 同时为 0 有两种含义（回零成功 / 刚上电从未回零），驱动器自己不区分，
 * 因此**必须先见过 bit2=1** 才允许判成功；超过确认截止时间仍未见过，判"回零未执行"。
 * ========================================================================== */

/** @brief 一次 `0x3B` 采样后的判据结论。 */
typedef enum {
    EMM_HOMING_POLL_HOMING = 0,   ///< bit2=1：正在回零，继续等。
    EMM_HOMING_POLL_DONE,         ///< 见过 bit2=1 之后回到空闲：回零完成。
    EMM_HOMING_POLL_PROTECT,      ///< 过热 / 过流保护已触发。
    EMM_HOMING_POLL_FAILED,       ///< bit3=1：驱动器报告回零失败。
    EMM_HOMING_POLL_NOT_STARTED   ///< 从未见过 bit2=1，且已超过确认截止时间。
} EmmHomingPoll;

/** @brief 只取 bit2/bit3（屏蔽 bit0/bit1 编码器就绪位）。 */
static inline uint8_t emmHomingBits(uint8_t flags)
{
    return (uint8_t)(flags & (EmmV5::ORIGIN_HOMING | EmmV5::ORIGIN_FAILED));
}

/**
 * @brief 用一次 `0x3B` 采样得出回零判据结论。
 * @param homingSeen   入出参：是否见过 `Org_SF=1`。**调用方刻意不重置**（暂停恢复后依然有效）。
 * @param graceExpired 是否已超过"必须见过 Org_SF"的确认截止时间。
 *                     （序列用 `EMM_TASK_SEQ_HOME_START_GRACE_MS`，整机回零用
 *                     `EMM_HOME_START_GRACE_MS`，两者当前同为 600 ms。）
 */
static inline EmmHomingPoll emmHomingFeed(uint8_t flags, bool *homingSeen,
                                          bool graceExpired)
{
    if ((flags & (EmmV5::ORIGIN_OVERHEAT | EmmV5::ORIGIN_OVERCURRENT)) != 0) {
        return EMM_HOMING_POLL_PROTECT;
    }

    const uint8_t bits = emmHomingBits(flags);
    if (bits == EmmV5::ORIGIN_FAILED) {
        return EMM_HOMING_POLL_FAILED;
    }
    if (bits == EmmV5::ORIGIN_HOMING) {
        *homingSeen = true;
        return EMM_HOMING_POLL_HOMING;
    }

    /* bits == 0x00：见过 Org_SF=1 才是"回零中 → 空闲"的正常收尾。 */
    if (*homingSeen) {
        return EMM_HOMING_POLL_DONE;
    }
    return graceExpired ? EMM_HOMING_POLL_NOT_STARTED : EMM_HOMING_POLL_HOMING;
}

#endif /* EMM_MOTION_JUDGE_H */

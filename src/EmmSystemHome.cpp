/**
 * @file EmmSystemHome.cpp
 * @author your name (you@domain.com)
 * @brief 整机回零（L2 复合动作）实现——ESP32-KWQ3。
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @details 实现思路：
 *          1. 一个常驻任务（优先级 3、固定 Core 1）默认阻塞在 ulTaskNotifyTake()，
 *             收到 SET EMM SYSTEM HOME 后按固定顺序执行 12 步回零序列；
 *          2. 序列里的每一步都是"发一条 EMM 命令 + 轮询驱动器状态直到该步结束"，
 *             上一步没有结束就绝不下发下一步，避免 HOME 与还在跑的 MOVE 打架；
 *          3. 每一步前后都用 `[回零]` 前缀打印，便于现场按串口日志定位卡点；
 *          4. 除样品盘外，任一步失败或超时都会广播停止全部 EMM 电机并中止流程；
 *          5. 流程运行期间由 CommandDispatcher 调用 EmmSystemHome_BlocksCommand()
 *             拦截其它 EMM 命令，保证回零独占三台电机。
 *
 *          所有 EMM 事务都遵循与 L1 相同的约定：先 Rs485Bus_Take()，再调用
 *          gEmm.*，最后 Rs485Bus_Give()，保证与阀头、激光传感器、急停监控和
 *          相电流监控不会在同一时刻争用总线。
 */

#include "common.h"
#include "EmmSystemHome.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ==========================================================================
 * 可调参数集中定义
 * --------------------------------------------------------------------------
 * 下列宏集中管理整机回零的全部时序与判据参数，逻辑代码中不出现魔数，便于
 * 现场按机械结构与实际节拍统一调整。
 * ========================================================================== */

/** 单条 EMM 命令等待驱动器 ACK 的超时，单位 ms。 */
#define EMM_HOME_CMD_TIMEOUT_MS        400U

/** 获取共享 RS485 总线的最长等待，单位 ms；超时则本轮跳过或判定该步失败。 */
#define EMM_HOME_BUS_TIMEOUT_MS        50U

/** 状态轮询周期，单位 ms；兼顾状态刷新速度与共享总线占用率。 */
#define EMM_HOME_POLL_INTERVAL_MS      100U

/**
 * 轮询兜底路径的"回零确认截止时间"，单位 ms。
 * @details 主判据是驱动器主动上报的 0x9F（见 emmHomeStepTrigger）。只有在收不到
 *          0x9F 而回退到轮询 0x3B 时才用到这里：手册 5.4.4 中 bit2/bit3 同时为 0
 *          有两种含义——「回零成功」与「默认空闲（刚上电、从未回零）」，驱动器
 *          自己不区分。因此轮询路径必须"先见过 bit2=1（正在回零）"才允许判成功；
 *          超过该时间仍未见过 bit2=1，就判"回零未执行"，**不当成功**。
 */
#define EMM_HOME_START_GRACE_MS        600U

/**
 * 位置运动起步保护时间的下限，单位 ms。
 * @details 定位命令刚下发时电机仍处于静止，若立刻按"转速为 0 且误差很小"判定
 *          会误判到位。实际起步保护取 max(本值, 加速时间 × 2)，其中
 *          加速时间 = RPM × (256 - ACC) × 50 µs（手册的加减速换算），
 *          慢速大加速度的动作会得到更长的起步保护。
 */
#define EMM_HOME_MOVE_GRACE_MS         300U

/** 单步回零等待上限，单位 ms。 */
#define EMM_HOME_TRIGGER_MAX_WAIT_MS   15000U

/** 单步绝对定位等待上限，单位 ms。 */
#define EMM_HOME_MOVE_MAX_WAIT_MS      60000U

/** 连续通信失败次数上限；达到即判定该步失败，避免故障设备永久占用总线。 */
#define EMM_HOME_MAX_COMM_ERRORS       3U

/**
 * 定位到位判据：实时转速为 0，且位置误差绝对值不超过该脉冲数，
 * 或驱动器已置位 Prf_TF（目标位置到达）。
 */
#define EMM_HOME_MOVE_TOLERANCE_PULSE  8L

/** 整机回零任务优先级：低于接近开关急停（11），高于主循环（1）。 */
#define EMM_HOME_TASK_PRIORITY         3
/** 整机回零任务固定核心。 */
#define EMM_HOME_TASK_CORE             1
/** 整机回零任务栈大小（字节）。调用嵌套为 任务→序列→步骤→等待→EmmV5 组帧，
 *  比监控类任务深，因此沿用 KWQ2 原 EmmSystemHome 的 8192 字节，留足余量。 */
#define EMM_HOME_TASK_STACK            8192U

/** 整机回零 12 步序列的总步数，用于串口打印 "步骤 n/12"。 */
#define EMM_HOME_STEP_TOTAL            12U

/* ==========================================================================
 * 模块内部状态
 * ========================================================================== */

/** 整机回零后台任务句柄；非空表示任务已创建。 */
static TaskHandle_t s_emmHomeTaskHandle = nullptr;

/**
 * 流程占用标志。
 * @details 由 EmmSystemHome_Start() 在"唤醒任务之前"同步置位，因此可以挡住
 *          两次快速连续下发；后台任务在流程结束后清零。也作为 L3 互锁的判据。
 */
static volatile bool s_emmHomeBusy = false;

/** 中止请求标志；由 SET EMM SYSTEM HOME STOP 置位，后台任务在步骤边界检查。 */
static volatile bool s_emmHomeStopRequested = false;

/** 当前步骤序号（从 1 开始），仅用于串口打印。 */
static uint16_t s_emmHomeStepIndex = 0;

/** 本次序列总步数，仅用于串口打印。 */
static uint16_t s_emmHomeStepTotal = EMM_HOME_STEP_TOTAL;

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
 * @brief 打印步骤抬头并推进步骤计数。
 * @param label 本步的中文说明，例如"升降电机碰撞回零"。
 */
static void emmHomePrintStep(const char *label)
{
    ++s_emmHomeStepIndex;
    Serial.printf("[回零] 步骤 %u/%u  %s\n",
                  (unsigned)s_emmHomeStepIndex,
                  (unsigned)s_emmHomeStepTotal, label);
}

/**
 * @brief 广播停止全部 EMM 电机。
 * @details 使用地址 0 广播，不等待设备级 ACK；总线繁忙时只打印提示。
 */
static void emmHomeStopAllMotors(void)
{
    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        Serial.println("[回零] RS485 总线繁忙，停止全部 EMM 电机的命令未下发");
        return;
    }
    const EmmV5::Status status = gEmm.stopBroadcast(false);
    Rs485Bus_Give();
    Serial.printf("[回零] 已广播停止全部 EMM 电机，状态=%s\n",
                  EmmV5::statusToString(status));
}

/**
 * @brief 写回零参数（0x4C/0xAE）。
 * @details 采用"读-改-写"：先读回驱动器当前回零参数，只覆盖本步显式给出的
 *          字段，其余字段原样写回。这样序列里写 `HOME PARAM RPM=85` 这种
 *          只给单个字段的写法不会把其它参数意外清零。
 *          始终使用 SAVE=0，即只改运行参数、不写掉电保存区。
 * @param mode      -1 表示沿用驱动器当前值，否则为回零模式 0~5。
 * @param direction -1 表示沿用当前值，否则为回零方向 0/1。
 * @param rpm       -1 表示沿用当前值，否则为回零速度（RPM）。
 * @param timeoutMs -1 表示沿用当前值，否则为驱动器回零超时（ms）。
 * @return true 表示本步成功；false 表示失败，调用方应中止流程。
 */
static bool emmHomeStepParam(uint8_t address, const char *label,
                             int32_t mode, int32_t direction,
                             int32_t rpm, int32_t timeoutMs)
{
    emmHomePrintStep(label);

    uint8_t curMode = 0, curDirection = 0;
    uint16_t curRpm = 0, curStallRpm = 0, curStallMa = 0, curStallMs = 0;
    uint32_t curTimeout = 0;
    bool curAutoHome = false;

    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        Serial.println("[回零]   失败：RS485 总线繁忙，无法读取当前回零参数");
        return false;
    }
    EmmV5::Status status = gEmm.readHomeParams(
        address, curMode, curDirection, curRpm, curTimeout,
        curStallRpm, curStallMa, curStallMs, curAutoHome,
        EMM_HOME_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK) {
        Serial.printf("[回零]   失败：A=%u 读取回零参数异常，状态=%s\n",
                      (unsigned)address, EmmV5::statusToString(status));
        return false;
    }

    const uint8_t  newMode      = (mode      >= 0) ? (uint8_t)mode      : curMode;
    const uint8_t  newDirection = (direction >= 0) ? (uint8_t)direction : curDirection;
    const uint16_t newRpm       = (rpm       >= 0) ? (uint16_t)rpm      : curRpm;
    const uint32_t newTimeout   = (timeoutMs >= 0) ? (uint32_t)timeoutMs : curTimeout;

    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        Serial.println("[回零]   失败：RS485 总线繁忙，无法写入回零参数");
        return false;
    }
    status = gEmm.modifyHomeParams(
        address, false /* SAVE=0：只改运行参数，不写掉电保存区 */,
        newMode, newDirection, newRpm, newTimeout,
        curStallRpm, curStallMa, curStallMs, curAutoHome,
        EMM_HOME_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK) {
        Serial.printf("[回零]   失败：A=%u 写入回零参数异常，状态=%s\n",
                      (unsigned)address, EmmV5::statusToString(status));
        return false;
    }

    Serial.printf("[回零]   A=%u 回零参数已写入(SAVE=0)：MODE=%u DIR=%u RPM=%u TIMEOUT=%lu ms\n",
                  (unsigned)address, (unsigned)newMode, (unsigned)newDirection,
                  (unsigned)newRpm, (unsigned long)newTimeout);
    return true;
}

/**
 * @brief 【诊断】读取 0x3A 系统状态并打印关键标志（只读，不改变控制流）。
 *
 * @details 用于定位"0xFD 定位命令被驱动器拒绝（REJECTED）"这一类问题。
 *          在每步动作的前后各调用一次，即可看出电机当时是否已使能、是否还在
 *          转动、是否挂着保护/限位标志，从而判断拒绝是"状态未就绪"还是"保护锁存"：
 *            1. 发回零命令之前 / 判定回零完成瞬间（`emmHomeStepTrigger`）；
 *            2. 发定位命令之前 / 判定定位完成瞬间（`emmHomeStepMove`）；
 *            3. 轮询兜底路径判定完成瞬间（`emmHomeWaitHoming`）。
 *
 * @note 本函数只发读命令、不写任何参数，纯粹是观测手段；确认问题后可整体删除。
 * @note 调用方必须已经持有 RS485 总线。
 * @param address 电机地址。
 * @param stage   调用时机说明，例如"发定位前"。
 */
static void emmHomeDumpMotorState(uint8_t address, const char *stage)
{
    EmmV5::SystemState st = {};
    const EmmV5::Status status =
        gEmm.readSystemState(address, st, EMM_HOME_CMD_TIMEOUT_MS);
    if (status != EmmV5::STATUS_OK) {
        Serial.printf("[回零]   [诊断] A=%u %s 读取 0x3A 失败：%s\n",
                      (unsigned)address, stage, EmmV5::statusToString(status));
        return;
    }

    Serial.printf("[回零]   [诊断] A=%u %s：MOTOR_FLAGS=0x%02X"
                  "(使能=%u 到位=%u 堵转=%u 堵转保护=%u 左限位=%u 右限位=%u)\n",
                  (unsigned)address, stage, (unsigned)st.motorFlags,
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_ENABLED) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_POSITION_REACHED) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_STALL) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_STALL_PROTECT) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_LEFT_LIMIT) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_RIGHT_LIMIT) ? 1U : 0U));
    Serial.printf("[回零]   [诊断] A=%u %s：实时转速=%d 实时位置=%ld 位置误差=%ld 0x3B原点标志=0x%02X\n",
                  (unsigned)address, stage, (int)st.realtimeRpm,
                  (long)st.realtimePosition, (long)st.positionError,
                  (unsigned)st.originFlags);
}

/**
 * @brief 【兜底】等待指定电机的回零动作结束（轮询 0x3B）。
 * @details 只在收不到驱动器主动上报 0x9F 时才走这里。轮询 0x3B 回零状态字
 *          （每次 Rs485Bus_Take，不长期占用总线），判据按手册 5.4.4 收紧：
 *          - 掩码取 bit2|bit3（Org_SF | Org_CF）：bit0/bit1 是 Cal_Rdy/Enc_Rdy，
 *            默认恒为 1，不掩码会被干扰；
 *          - `& 0x0C == 0x08`（Org_CF=1）→ 回零失败；
 *          - `& 0x0C == 0x04`（Org_SF=1）→ 记录"确实正在回零"，继续等待；
 *          - `& 0x0C == 0x00` → **只有在见过 Org_SF=1 之后才算回零成功**；
 *            否则它同时可能是"刚上电、从未回零"的默认状态，不能当成功。
 *          超过 EMM_HOME_START_GRACE_MS 仍未见过 Org_SF=1 → 报"回零未执行"并失败。
 *          同时响应中止请求，并统计连续通信失败次数。
 * @return true 表示回零完成；false 表示失败、超时或被中止。
 */
static bool emmHomeWaitHoming(uint8_t address, uint32_t maxWaitMs)
{
    const uint32_t startedAt = millis();
    bool homingSeen = false;
    uint8_t commErrors = 0;

    for (;;) {
        if (s_emmHomeStopRequested) {
            Serial.println("[回零]   收到中止请求，停止等待回零");
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(EMM_HOME_POLL_INTERVAL_MS));

        const uint32_t elapsed = millis() - startedAt;
        if (elapsed >= maxWaitMs) {
            Serial.printf("[回零]   失败：A=%u 回零等待超时（%lu ms）\n",
                          (unsigned)address, (unsigned long)maxWaitMs);
            return false;
        }

        if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
            continue;
        }
        uint8_t flags = 0;
        const EmmV5::Status status =
            gEmm.readOriginFlags(address, flags, EMM_HOME_CMD_TIMEOUT_MS);
        Rs485Bus_Give();

        if (status != EmmV5::STATUS_OK) {
            ++commErrors;
            if (commErrors >= EMM_HOME_MAX_COMM_ERRORS) {
                Serial.printf("[回零]   失败：A=%u 连续 %u 次读取回零状态异常，状态=%s\n",
                              (unsigned)address, (unsigned)commErrors,
                              EmmV5::statusToString(status));
                return false;
            }
            continue;
        }
        commErrors = 0;

        if ((flags & (EmmV5::ORIGIN_OVERHEAT | EmmV5::ORIGIN_OVERCURRENT)) != 0) {
            Serial.printf("[回零]   失败：A=%u 驱动器保护已触发（过热/过流），FLAGS=0x%02X\n",
                          (unsigned)address, flags);
            return false;
        }

        /* 只取 bit2/bit3；bit0/bit1 是编码器就绪位，默认恒为 1，必须屏蔽。 */
        const uint8_t homeBits = (uint8_t)(flags & (EmmV5::ORIGIN_HOMING |
                                                    EmmV5::ORIGIN_FAILED));

        if (homeBits == EmmV5::ORIGIN_FAILED) {
            Serial.printf("[回零]   失败：A=%u 驱动器报告回零失败，FLAGS=0x%02X\n",
                          (unsigned)address, flags);
            return false;
        }
        if (homeBits == EmmV5::ORIGIN_HOMING) {
            homingSeen = true;
            continue;
        }

        /*
         * homeBits == 0x00：见过 Org_SF=1 才是"回零中 → 空闲"的正常收尾；
         * 从未见过则可能只是"刚上电、从未回零"的默认状态，不能当成功。
         */
        if (homingSeen) {
            Serial.printf("[回零]   A=%u 回零完成（轮询 0x3B 确认），耗时 %lu ms，FLAGS=0x%02X\n",
                          (unsigned)address, (unsigned long)elapsed, (unsigned)flags);
            if (Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
                emmHomeDumpMotorState(address, "回零完成瞬间");
                Rs485Bus_Give();
            }
            return true;
        }
        if (elapsed >= EMM_HOME_START_GRACE_MS) {
            Serial.printf("[回零]   失败：A=%u 从未观察到正在回零（Org_SF），"
                          "无法确认回零已执行，FLAGS=0x%02X\n",
                          (unsigned)address, (unsigned)flags);
            return false;
        }
    }
}

/**
 * @brief 【兜底】等待指定电机的绝对定位运动结束（轮询 0x3A/系统状态）。
 * @details 只在收不到驱动器主动上报 0x9F 时才走这里。判据：
 *          - 堵转保护触发即失败；
 *          - **必须先观察到"位置变化过或转速非 0"（moved）**，再允许按
 *            "转速为 0 且（Prf_TF 或 |误差| ≤ 容差）"判到位——否则命令刚下发、
 *            电机还没起步就会被误判为到位；
 *          - 起步保护按速度动态给：max(EMM_HOME_MOVE_GRACE_MS, 加速时间 × 2)，
 *            其中加速时间 = RPM × (256 - ACC) × 50 µs。
 * @param rpm          本步定位命令使用的转速（用于算加速时间）。
 * @param acceleration 本步定位命令使用的加速度字段（0~255）。
 * @return true 表示定位完成；false 表示失败、超时或被中止。
 */
static bool emmHomeWaitMove(uint8_t address, uint32_t maxWaitMs,
                            uint16_t rpm, uint8_t acceleration)
{
    /* 加速时间 = RPM × (256 - ACC) × 50 µs；起步保护取其两倍，且不低于下限。 */
    const uint32_t rampMs = (uint32_t)rpm *
                            (uint32_t)(256U - (uint32_t)acceleration) * 50U / 1000U;
    uint32_t graceMs = rampMs * 2U;
    if (graceMs < EMM_HOME_MOVE_GRACE_MS) graceMs = EMM_HOME_MOVE_GRACE_MS;
    Serial.printf("[回零]   A=%u 定位起步保护 %lu ms（RPM=%u ACC=%u）\n",
                  (unsigned)address, (unsigned long)graceMs,
                  (unsigned)rpm, (unsigned)acceleration);
    vTaskDelay(pdMS_TO_TICKS(graceMs));

    const uint32_t startedAt = millis();
    uint8_t commErrors = 0;
    bool moved = false;
    int32_t firstPosition = 0;
    bool haveFirst = false;

    for (;;) {
        if (s_emmHomeStopRequested) {
            Serial.println("[回零]   收到中止请求，停止等待定位");
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(EMM_HOME_POLL_INTERVAL_MS));

        const uint32_t elapsed = millis() - startedAt;
        if (elapsed >= maxWaitMs) {
            Serial.printf("[回零]   失败：A=%u 定位等待超时（%lu ms）\n",
                          (unsigned)address, (unsigned long)maxWaitMs);
            return false;
        }

        if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
            continue;
        }
        EmmV5::SystemState state = {};
        const EmmV5::Status status =
            gEmm.readSystemState(address, state, EMM_HOME_CMD_TIMEOUT_MS);
        Rs485Bus_Give();

        if (status != EmmV5::STATUS_OK) {
            ++commErrors;
            if (commErrors >= EMM_HOME_MAX_COMM_ERRORS) {
                Serial.printf("[回零]   失败：A=%u 连续 %u 次读取系统状态异常，状态=%s\n",
                              (unsigned)address, (unsigned)commErrors,
                              EmmV5::statusToString(status));
                return false;
            }
            continue;
        }
        commErrors = 0;

        if ((state.motorFlags & EmmV5::MOTOR_STALL_PROTECT) != 0) {
            Serial.printf("[回零]   失败：A=%u 定位过程中触发堵转保护，MOTOR_FLAGS=0x%02X\n",
                          (unsigned)address, state.motorFlags);
            return false;
        }

        /* moved 前置：位置变过或转速非 0，才算"动作确实起步了"。 */
        if (!haveFirst) {
            firstPosition = state.realtimePosition;
            haveFirst = true;
        } else if (state.realtimePosition != firstPosition ||
                   state.realtimeRpm != 0) {
            moved = true;
        }

        const bool stopped = (state.realtimeRpm == 0);
        const bool onTarget =
            moved && stopped &&
            (((state.motorFlags & EmmV5::MOTOR_POSITION_REACHED) != 0) ||
             (state.positionError <= EMM_HOME_MOVE_TOLERANCE_PULSE &&
              state.positionError >= -EMM_HOME_MOVE_TOLERANCE_PULSE));
        if (onTarget) {
            Serial.printf("[回零]   A=%u 定位完成（轮询 0x3A 确认）：当前位置=%ld 误差=%ld，耗时 %lu ms\n",
                          (unsigned)address, (long)state.realtimePosition,
                          (long)state.positionError, (unsigned long)elapsed);
            return true;
        }
    }
}

/**
 * @brief 触发回零并等待完成（0x9A）。
 * @details 主判据是驱动器主动上报的 0x9F（回零完成）：requireDone=true 让
 *          EmmV5 在收到 0x02（命令已接受）之后继续等 0x9F，等到即完成。
 *          只有收不到 0x9F 时才回退到轮询 0x3B 兜底。
 * @param homeMode 回零模式：0=单圈就近回零，2=无限位碰撞回零。
 */
static bool emmHomeStepTrigger(uint8_t address, const char *label,
                               uint8_t homeMode, uint32_t maxWaitMs)
{
    emmHomePrintStep(label);

    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        Serial.println("[回零]   失败：RS485 总线繁忙，回零命令未下发");
        return false;
    }
    /* 【诊断】发回零命令前记录一次电机状态，用于对比"碰撞回零前 vs 之后"是否失能。 */
    emmHomeDumpMotorState(address, "发回零前");

    /*
     * requireDone=true：等驱动器主动上报 0x9F 才算完成。这里必须一直占着总线——
     * 一旦中途交还，别的任务的清缓冲会把 0x9F 丢掉（见 EmmV5::writeAndReadAck 注释）。
     */
    const EmmV5::Status status = gEmm.home(
        address, homeMode, false, maxWaitMs, /*requireDone=*/true);
    /* 【诊断】判定回零完成的这一刻再取一次电机状态。 */
    emmHomeDumpMotorState(address, "回零完成瞬间");
    Rs485Bus_Give();

    if (status == EmmV5::STATUS_DONE) {
        Serial.printf("[回零]   A=%u 回零完成（驱动器主动上报 0x9F），MODE=%u\n",
                      (unsigned)address, (unsigned)homeMode);
        return true;
    }
    if (status == EmmV5::STATUS_OK) {
        Serial.printf("[回零]   A=%u 未收到 0x9F（命令已被接受），回退到轮询 0x3B 确认\n",
                      (unsigned)address);
        return emmHomeWaitHoming(address, maxWaitMs);
    }
    if (status == EmmV5::STATUS_NOT_MOVED) {
        /*
         * 0x12/0x22：驱动器报告"已在零点或限位已触发，电机不动"。手册明确这不是
         * 错误，但它也没有执行回零动作，因此不能直接当成功——回退到轮询确认。
         */
        Serial.printf("[回零]   A=%u 驱动器报告电机未动（0x12/0x22），回退到轮询 0x3B 确认\n",
                      (unsigned)address);
        return emmHomeWaitHoming(address, maxWaitMs);
    }

    Serial.printf("[回零]   失败：A=%u 回零命令未被接受，状态=%s\n",
                  (unsigned)address, EmmV5::statusToString(status));
    return false;
}

/**
 * @brief 绝对定位到指定脉冲位置并等待完成（0xFD，MODE=1）。
 * @details 整机回零中的两处移动都使用 MODE=1（绝对位置，以回零零点为基准），
 *          与《EMM_CONTROL.md》"1 = 绝对位置零点"的约定一致：回零建立零点后
 *          直接给出目标绝对坐标，语义最直白，也省掉一次"读当前位置"的总线往返。
 *          完成判据优先用驱动器主动上报的 0x9F（到位），收不到才回退轮询 0x3A。
 */
static bool emmHomeStepMove(uint8_t address, const char *label,
                            uint8_t direction, uint16_t rpm, uint8_t acceleration,
                            uint32_t pulses, uint8_t motionMode, uint32_t maxWaitMs)
{
    emmHomePrintStep(label);

    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        Serial.println("[回零]   失败：RS485 总线繁忙，定位命令未下发");
        return false;
    }
    /* 【诊断】发定位命令前记录一次电机状态：看是否已使能、是否还在转动、是否挂保护。 */
    emmHomeDumpMotorState(address, "发定位前");

    /*
     * requireDone=true：等驱动器主动上报 0x9F（到位）才算完成，期间一直占着总线。
     */
    const EmmV5::Status status = gEmm.positionMode(
        address, direction, rpm, acceleration, pulses, motionMode,
        false, maxWaitMs, /*requireDone=*/true);
    /* 在交还总线之前取出原始应答帧，避免被其它任务的下一次通信覆盖。 */
    const EmmV5::Response ack = gEmm.lastResponse();
    /* 【诊断】判定定位完成的这一刻再取一次电机状态。 */
    emmHomeDumpMotorState(address, "定位完成瞬间");
    Rs485Bus_Give();

    if (status == EmmV5::STATUS_DONE) {
        Serial.printf("[回零]   A=%u 定位完成（驱动器主动上报 0x9F）："
                      "MODE=%u DIR=%u RPM=%u ACC=%u PULSE=%lu\n",
                      (unsigned)address, (unsigned)motionMode, (unsigned)direction,
                      (unsigned)rpm, (unsigned)acceleration, (unsigned long)pulses);
        return true;
    }
    if (status == EmmV5::STATUS_OK) {
        Serial.printf("[回零]   A=%u 未收到 0x9F（定位命令已被接受），回退到轮询 0x3A 确认\n",
                      (unsigned)address);
        return emmHomeWaitMove(address, maxWaitMs, rpm, acceleration);
    }
    if (status == EmmV5::STATUS_NOT_MOVED) {
        Serial.printf("[回零]   A=%u 驱动器报告电机未动（0x12/0x22），回退到轮询 0x3A 确认\n",
                      (unsigned)address);
        return emmHomeWaitMove(address, maxWaitMs, rpm, acceleration);
    }

    Serial.printf("[回零]   失败：A=%u 定位命令未被接受，状态=%s\n",
                  (unsigned)address, EmmV5::statusToString(status));
    /*
     * 【诊断】把驱动器返回的原始应答帧打出来。此前所有非 0x02/0x9F 的状态都被
     * 统一映射成 REJECTED，无法区分 0xE2（参数错误）与 0xEE（条件不满足），
     * 这是判断根因最关键的一条信息。
     */
    Serial.printf("[回零]   [诊断] A=%u 原始应答 %u 字节：",
                  (unsigned)address, (unsigned)ack.rawLength);
    for (uint8_t i = 0; i < ack.rawLength; ++i) {
        Serial.printf("%02X ", (unsigned)ack.raw[i]);
    }
    Serial.printf("(FUNC=0x%02X ACK_STATUS=0x%02X)\n",
                  (unsigned)ack.function, (unsigned)ack.commandStatus);
    return false;
}

/**
 * @brief 执行 12 步整机回零序列。
 * @details 顺序固定：升降电机(A=3) 6 步 → 旋转电机(A=2) 5 步 → 样品盘(A=4) 1 步。
 *          升降与旋转都是"碰撞回零建立零点 → 绝对定位粗进 → 慢速再次碰撞回零
 *          复校零点 → 绝对定位精进"的两段式回零，用慢速复校抵消快速碰撞回零
 *          的定位偏差。样品盘按配置忽略失败，不影响整机回零结果。
 *
 * @note 本序列**不再需要任何固定延时**。每一步的"动作是否做完"都由
 *       emmHomeStepTrigger()/emmHomeStepMove() 等到驱动器主动上报的 0x9F 才返回；
 *       此前散落在步骤之间的 900/4000/500/6000 ms 硬编码延时是在"猜时间"，
 *       已随 0x9F 判据一起删除（详见《修改日志.md》§5.6 / §5.7）。
 * @return true 表示序列执行完毕（样品盘失败也算完成）；false 表示某步失败需中止。
 */
static bool emmHomeRunSequence(void)
{
    s_emmHomeStepIndex = 0;
    s_emmHomeStepTotal = EMM_HOME_STEP_TOTAL;

    /* ---- 升降电机（A=3）：6 步 ---- */
    if (!emmHomeStepParam(EMM_LIFT_MOTOR, "升降电机写入回零参数(RPM=85)",
                          -1, -1, 85, -1)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_LIFT_MOTOR, "升降电机碰撞回零",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }
    if (!emmHomeStepMove(EMM_LIFT_MOTOR, "升降电机下降至绝对位置 10000",
                         1, 50, 20, 10000, 1, EMM_HOME_MOVE_MAX_WAIT_MS)) {
        return false;
    }
    if (!emmHomeStepParam(EMM_LIFT_MOTOR, "升降电机写入回零参数(RPM=40)",
                          -1, -1, 40, -1)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_LIFT_MOTOR, "升降电机碰撞回零(慢速复校)",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }
    if (!emmHomeStepMove(EMM_LIFT_MOTOR, "升降电机下降至绝对位置 1800",
                         1, 50, 20, 1800, 1, EMM_HOME_MOVE_MAX_WAIT_MS)) {
        return false;
    }

    /* ---- 旋转电机（A=2）：5 步 ---- */
    if (!emmHomeStepParam(EMM_ROTARY_MOTOR,
                          "旋转电机写入回零参数(DIR=1 RPM=40 TIMEOUT=1500)",
                          -1, 1, 40, 1500)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_ROTARY_MOTOR, "旋转电机碰撞回零",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }
    if (!emmHomeStepMove(EMM_ROTARY_MOTOR, "旋转电机移动至绝对位置 1000",
                         0, 5, 5, 800, 1, EMM_HOME_MOVE_MAX_WAIT_MS)) {
        return false;
    }
    if (!emmHomeStepParam(EMM_ROTARY_MOTOR, "旋转电机写入回零参数(RPM=8)",
                          -1, -1, 8, 10000)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_ROTARY_MOTOR, "旋转电机碰撞回零(慢速复校)",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }
    /* ---- 样品盘（A=4）：1 步，失败忽略 ---- */
    if (!emmHomeStepTrigger(EMM_SAMPLE_TRAY_MOTOR, "样品盘回零(MODE=0，失败忽略)",
                            0, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        Serial.println("[回零]   样品盘回零未成功，按配置忽略并继续");
    }

    return true;
}

/**
 * @brief 整机回零常驻任务。
 * @details 默认阻塞等待启动通知；被唤醒后执行完整序列。任一步失败（样品盘
 *          除外）都会广播停止全部 EMM 电机并中止流程。任务不主动退出，
 *          每次流程结束后回到阻塞状态等待下一次启动。
 */
static void emmHomeTask(void *argument)
{
    (void)argument;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!s_emmHomeBusy) {
            /* 通知被 Start 之外的路径误发时直接忽略。 */
            continue;
        }

        Serial.println("[回零] ========== 整机回零开始 ==========");
        const bool ok = emmHomeRunSequence();

        if (!ok) {
            Serial.println("[回零] 流程未完成，正在停止全部 EMM 电机");
            emmHomeStopAllMotors();
        }

        s_emmHomeBusy = false;
        s_emmHomeStopRequested = false;
        Serial.printf("[回零] ========== 整机回零%s ==========\n",
                      ok ? "完成" : "中止");
    }
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

bool EmmSystemHome_Init(void)
{
    if (s_emmHomeTaskHandle != nullptr) {
        return true;
    }

    const BaseType_t result = xTaskCreatePinnedToCore(
        emmHomeTask,
        "EmmSystemHome",
        EMM_HOME_TASK_STACK,
        nullptr,
        EMM_HOME_TASK_PRIORITY,
        &s_emmHomeTaskHandle,
        EMM_HOME_TASK_CORE);
    if (result != pdPASS) {
        s_emmHomeTaskHandle = nullptr;
        Serial.println("[回零] 整机回零任务启动失败：无法创建后台任务");
        return false;
    }

    Serial.printf("[回零] 整机回零任务已启动：优先级=%d 核心=%d 步数=%u\n",
                  (int)EMM_HOME_TASK_PRIORITY, (int)EMM_HOME_TASK_CORE,
                  (unsigned)EMM_HOME_STEP_TOTAL);
    return true;
}

bool EmmSystemHome_Start(void)
{
    if (!EmmSystemHome_Init()) {
        return false;
    }
    if (s_emmHomeBusy) {
        Serial.println("[回零] 已拒绝启动：整机回零正在运行");
        return false;
    }

    /*
     * 先同步置位占用标志，再唤醒任务。这样两次快速连续下发时，第二次会在
     * 这里被挡下，而不会出现"任务还没开始跑就被重复启动"的竞态。
     */
    s_emmHomeBusy = true;
    s_emmHomeStopRequested = false;
    xTaskNotifyGive(s_emmHomeTaskHandle);
    return true;
}

void EmmSystemHome_RequestStop(void)
{
    if (!s_emmHomeBusy) {
        Serial.println("[回零] 当前没有正在运行的整机回零流程");
        return;
    }
    s_emmHomeStopRequested = true;
    Serial.println("[回零] 已请求中止整机回零，当前步骤结束后停止");
}

bool EmmSystemHome_IsActive(void)
{
    return s_emmHomeBusy;
}

bool EmmSystemHome_BlocksCommand(const char *commandName)
{
    if (!s_emmHomeBusy || commandName == nullptr) {
        return false;
    }

    /*
     * 必须放行的例外：
     *   - 回零自身的中止命令：否则流程无法被打断；
     *   - 回零自身的启动命令：由 EmmSystemHome_Start() 负责回 SEQ_RUNNING；
     *   - SET EMM LIFT MONITOR：只切换本地布防标志，不产生总线事务。
     */
    if (strcmp(commandName, "SET EMM SYSTEM HOME") == 0 ||
        strcmp(commandName, "SET EMM SYSTEM HOME STOP") == 0 ||
        strcmp(commandName, "SET EMM LIFT MONITOR") == 0) {
        return false;
    }

    /*
     * 回零期间独占 2/3/4 号电机：拦截所有会触发动作或修改电机状态的
     * SET EMM 命令；查询类 GET EMM 命令保持可用，便于现场诊断。
     */
    return strncmp(commandName, "SET EMM", 7) == 0;
}

bool cmdSetEmmSystemHome(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    /* 与 L1 运动命令一致：整机断电时拒绝启动，避免上位机误以为已开始回零。 */
    if (cmd_power_off) {
        Serial.printf("[回零] @%d 拒绝启动：步进电机电源已关闭\n", seq);
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }

    if (!EmmSystemHome_Start()) {
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }

    sendWithCRC("OK @%d EMM SYSTEM HOME STARTED", seq);
    return true;
}

bool cmdSetEmmSystemHomeStop(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    EmmSystemHome_RequestStop();
    sendWithCRC("OK @%d EMM SYSTEM HOME STOP ACCEPTED", seq);
    return true;
}

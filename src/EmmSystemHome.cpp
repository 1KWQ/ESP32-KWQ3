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
 *             收到 SET EMM SYSTEM HOME 后按固定顺序执行 17 步回零序列；
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
 *
 *          **回零完成判据**：`0x3B` 的 bit2/bit3 判据已集中到 `EmmMotionJudge.h`
 *          的 `emmHomingFeed()`（2026/10/10 抽取），与任务序列的
 *          `EmmTaskSeq::emmSeqWaitHomingAll` **共用同一份实现**，避免两处行为分叉。
 *          本文件只保留"轮询 + 中止标志 + 日志 + 收尾动作"。
 *          注意 `emmHomeWaitMove()`（定位回退路径）**仍用旧的 `moved` 判据**，
 *          迁移列为第二批（会改变行为，需单独实机验证）。
 */

#include "common.h"
#include "EmmSystemHome.h"
/** 运动/回零"完成判据"的唯一实现（纯函数，无 IO）。 */
#include "EmmMotionJudge.h"

#include <string.h>
#include <Preferences.h>

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
#define EMM_HOME_MOVE_MAX_WAIT_MS      20000U

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

/**
 * 整机回零序列的总步数，用于串口打印 "步骤 n/17"。
 * @note 变更史：
 *       - **2026/10/20 由 12 改为 15**：在"2 号慢速碰撞回零"之后新增 3 步
 *         （3 号写回零参数 → 3 号慢速碰撞回零 → 3 号定位回 2500）。
 *       - **2026/10/23 由 15 改为 17**：流程改为"3 号回零 / 2 号回零"交替两轮
 *         （3 号回零 → 下 1200 → 2 号回零 → 3 号回零 → 下 1200 → 2 号回零
 *         → 顺 800 → 2 号回零 → 3 号回零 → 下 2500），回零次数由 4 次增到 6 次。
 */
#define EMM_HOME_STEP_TOTAL            17U

/*
 * **2026/10/21 下线**：原来事件帧带固定序号 `EVT …`（`EMM_HOME_EVT_SEQ`）。
 * 用户确认 `@100` 是假序号、会与真实命令序号撞车，已从所有 `EVT` / `ERR` 帧里去掉 ——
 * 现在只有**真实命令应答**才带 `@<seq>`（`OK @600 …` / `ERR @600 102`）。
 * 此常量随之下线。
 */

/* ---------------- 样品盘（A=4）接近开关零点标定（2026/10/16 新增） ---------------- */

/**
 * @brief 样品盘零点标定的旋转方向：`0` = **顺时针**（用户 2026/10/16 确认）。
 * @note 与 2 号旋转电机的既有约定一致（`DIR=0` 顺时针，见《修改日志.md》§7.17）。
 *       实机方向相反时只改这一个宏。
 */
#define EMM_TRAY_HOME_DIR               0U
/** @brief 触发后继续前进的方向：与 `EMM_TRAY_HOME_DIR` **同向**，即继续顺时针。 */
#define EMM_TRAY_HOME_ADVANCE_DIR       0U
/** @brief 防漂移时"转出触发区"的方向：与触发方向**相反**，即逆时针。 */
#define EMM_TRAY_HOME_ESCAPE_DIR        1U
/**
 * @brief 触发接近开关后**继续顺时针**前进的脉冲数 —— **出厂默认值**。
 * @details **2026/10/26 改版**：原流程是"触发 → 清零 → **逆时针退让** → 再清零"，
 *          退让量宏 `EMM_TRAY_HOME_BACKOFF_PULSES` 几经调整（16 → 32 → 55 → 65），
 *          代码里最终落在 `0`（等于没动作），零点就落在触发点上。
 *          现按用户要求改成"触发 → **关监控** → **继续顺时针**走 N 脉冲 → 清零"，
 *          即 01 号工位定在"触发点再顺时针 N 脉冲"处。
 *
 *          该值**运行期可改 + 落 NVS**（见 `EmmSystemHome_SetTrayZeroPulses()`，
 *          上位机命令 `SET EMM TRAY ZERO P=<n>`），本宏只在 NVS 无记录时兜底。
 *          参考：`3200` 脉冲 = 一圈 = `360°`，故 50 脉冲 ≈ `5.6°`
 *          （1 个工位 = 320 脉冲 = `36°`，50 脉冲 ≈ `0.16` 工位）。
 */
#define EMM_TRAY_ZERO_PULSES_DEFAULT    50UL
/** @brief 上述脉冲数的合法上限：一圈（3200 脉冲）。超过必然跨工位，直接拒绝。 */
#define EMM_TRAY_ZERO_PULSES_MAX        3200UL
/** @brief NVS 命名空间与 key（key 上限 15 字节）。 */
#define EMM_TRAY_ZERO_NVS_NS            "emmhome"
#define EMM_TRAY_ZERO_NVS_KEY           "trayzp"
/** @brief NVS 打开失败后，两次重试之间的最小间隔（ms），避免每步都去开一次。 */
#define EMM_TRAY_ZERO_NVS_RETRY_MS      2000U
/** @brief 样品盘零点标定的转速与加速度（用户给定 `RPM=10 ACC=10`）。 */
#define EMM_TRAY_HOME_RPM               10U
#define EMM_TRAY_HOME_ACC               10U
/**
 * @brief **轮询接近开关 GPIO 电平**的间隔，单位 ms。
 * @details **2026/10/26 由 100 改为 10**（用户要求）。这一项直接决定"找触发点"的位置
 *          分辨率：`RPM=10` 时转速 533 脉冲/秒，100 ms 意味着一次检查间隔内走 **53 脉冲
 *          ≈ 6.0°**（几乎等于 50 脉冲的偏移量本身）；改成 10 ms 后降到 **5.3 脉冲 ≈ 0.6°**。
 *
 *          ⚠️ 这是**纯 GPIO 读取**（`digitalRead`），**不产生任何 RS485 总线事务**，
 *          只在"速度模式起转 → 等到触发/释放"这一小段窗口里跑，所以降到 10 ms 不会
 *          增加总线负担，也不会影响其它电机通信。**不要再和下面的停稳轮询共用一个宏**
 *          —— 那一处每轮都要读一次驱动器状态，属于总线事务。
 *
 *          ⚠️ 这一项只影响**精度**（δ 中的"检测延迟"），**不影响漂移**。漂移是
 *          "前沿被上次输出顶替"的逻辑问题，由 ⓪ 防漂移根治，与本宏无关。
 */
#define EMM_TRAY_HOME_POLL_MS           10U
/**
 * @brief **轮询驱动器实时转速**（等停稳）的间隔，单位 ms。
 * @details **2026/10/26 从 `EMM_TRAY_HOME_POLL_MS` 拆出来单独定义**。该循环每一轮都要
 *          `Rs485Bus_Take()` + `readSystemState()` 读一次 `0x43`，是**真实的 RS485 总线
 *          事务**（一帧往返约 1~2 ms）。若跟着 GPIO 轮询一起降到 10 ms，总线占用率会到
 *          10~20%，还要和 `EmmEndstopMonitor`、上位机查询命令抢锁 —— 得不偿失。
 *
 *          保持 100 ms 不会拖慢流程：停车减速本身约 123 ms，且要连续两次读到 0 才算停稳，
 *          本步整体本来就在几百 ms 量级；而且停稳只是"确认"，不参与位置计算。
 */
#define EMM_TRAY_HOME_STOP_POLL_MS      100U
/**
 * @brief 发停止后等样品盘停稳的最长时间，单位 ms。
 * @note 只用于"确认停稳"，超时**不判失败**（避免因为一时读不到状态就把整步判死）。
 */
#define EMM_TRAY_HOME_STOP_MAX_WAIT_MS  5000U
/**
 * @brief 位置清零后核对"当前读数是否 ≈ 0"的容差（驱动器计数）。
 * @details **2026/10/16 新增**：`0x93/0x88` 只设"单圈回零零点"，**不会**把当前位置
 *          计数清零；要让"当前位置 = 0"必须另发 `0x0A/0x6D`（见下方 ②④ 两次位置清零）。
 *          清零后读回 `0x43.realtimePosition` 核对一下（见下方 ⑥），偏差大就打印告警
 *          （**只告警、不判失败**）。
 *          参考：`65536` 计数 = 一圈 = `360°`；1 个工位 = `320` 脉冲 = `6553.6` 计数。
 *          `64` 计数 ≈ `0.35°`：远大于编码器零漂，远小于 1 个工位。
 */
#define EMM_TRAY_HOME_ZERO_TOL_COUNTS   64L

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

/**
 * @brief 外部中止查询钩子（默认 `nullptr` = 不启用）。
 * @details **2026/10/19 新增**，只为"任务序列复位阶段的样品盘回零"服务：
 *          序列的 `SEQ STOP` 置的是 `SeqState`（在 `EmmTaskQueue` 里），**不是**本模块的
 *          `s_emmHomeStopRequested`，所以序列调用本模块时必须有外部判据才能被打断。
 *          **不注册时行为与改动前完全一致** —— 整机回零、`SET EMM TRAY HOME` 都不受影响。
 * @note 由 `EmmSystemHome_TrayHomeForSeq()` 进入时注册、退出时**无条件注销**，
 *       避免留下悬空判据影响后续的整机回零。
 */
typedef bool (*EmmHomeAbortQueryFn)(void);
static EmmHomeAbortQueryFn s_externalAbortQuery = nullptr;

/**
 * @brief 统一的中止判据：本模块标志 **或** 外部钩子。
 * @details 样品盘回零的所有轮询点（等接近开关、等停稳）都走这里，保证
 *          "整机回零 / 序列复位"两条入口用同一套判断，不会出现一个能停、一个停不下。
 */
static bool emmHomeAbortRequested(void)
{
    if (s_emmHomeStopRequested) {
        return true;
    }
    return (s_externalAbortQuery != nullptr) && s_externalAbortQuery();
}

/** 当前步骤序号（从 1 开始），仅用于串口打印。 */
static uint16_t s_emmHomeStepIndex = 0;

/** 本次序列总步数，仅用于串口打印。 */
static uint16_t s_emmHomeStepTotal = EMM_HOME_STEP_TOTAL;

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
 * @brief 上报一条整机回零事件帧（**2026/10/21 新增**，配合"精简日志"）。
 *
 * @param code `crc16.h` 里的码（`HOME_BEGIN` / `HOME_AXIS_3` / `HOME_DONE` …）。
 * @param what 事件正文，例如 `BEGIN` / `AXIS 3` / `DONE`。
 *
 * @details 帧格式 `EVT <code> HOME <what>`，例：
 *          `EVT 370 HOME AXIS 3`
 *
 *          为什么上电自动回零也需要 `HOME BEGIN`：上电走的是 `SysInit_Run()`
 *          自动触发，**没有命令应答**，不给一条 BEGIN 的话上位机无法区分
 *          "还没开始回零"和"已经开始但没报错"。
 *
 * @note 长度核对：最长 `EVT 372 HOME AXIS 4` = 26 字符，
 *       远低于 `sendWithCRC()` 的 127 字符上限。
 */
static void emmHomeEvent(int code, const char *what)
{
    sendWithCRC("EVT %d HOME %s", code, (what != nullptr) ? what : "");
}

/**
 * @brief 打印步骤抬头并推进步骤计数。
 * @param label 本步的中文说明，例如"升降电机碰撞回零"。
 */
static void emmHomePrintStep(const char *label)
{
    ++s_emmHomeStepIndex;
    LOGD("[回零] 步骤 %u/%u  %s\n",
                  (unsigned)s_emmHomeStepIndex,
                  (unsigned)s_emmHomeStepTotal, label);
}

/**
 * @brief 打印"步骤内的子动作"抬头（**不推进**步骤计数，2026/10/16 新增）。
 * @details 样品盘零点标定一步内含多个子动作（正向接近 / 位置清零 / 绝对退让 /
 *          再清零 / 设单圈零点 / 读回核对），
 *          但对外仍是"第 17 步 / 共 17 步"，所以子动作只打印、不计数，
 *          避免把 `步骤 x/14` 打爆。
 * @param label 子动作的中文说明。
 */
static void emmHomePrintSubStep(const char *label)
{
    LOGD("[回零]   └─ 样品盘 %s\n", label);
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
 * @brief 全量写回零参数（0x4C/0xAE）。
 * @details 8 个字段全部显式给出，**直接整组下发，不再"读-改-写"**。
 *          早期实现是"先 readHomeParams() 读回驱动器当前值，只覆盖本步显式
 *          给出的字段，其余原样写回"。该做法有两个隐患：
 *            1. 驱动器里存的若是未初始化值或上一次任务残留的参数，会被原样
 *               带回，等于把未知值写进回零参数；
 *            2. 每次都要多一次读往返，占用共享总线。
 *          现改为每次下发完整参数（值来源：《功能文档.txt》"程序回零过程"一节，
 *          逐条对应本序列的步 1/4/7/10）。始终 SAVE=0，即只改运行参数、
 *          不写掉电保存区。
 * @param mode      回零模式 0~5（本序列用 2=无限位碰撞回零）。
 * @param direction 回零方向 0=CW / 1=CCW。
 * @param rpm       回零速度（RPM）。
 * @param timeoutMs 驱动器回零超时（ms）。
 * @param stallRpm  碰撞检测转速（RPM）。
 * @param stallMa   碰撞检测电流（mA）。
 * @param stallMs   碰撞检测持续时间（ms）。
 * @param autoHome  是否上电自动回零。
 * @return true 表示本步成功；false 表示失败，调用方应中止流程。
 */
static bool emmHomeStepParam(uint8_t address, const char *label,
                             uint8_t mode, uint8_t direction,
                             uint16_t rpm, uint32_t timeoutMs,
                             uint16_t stallRpm, uint16_t stallMa,
                             uint16_t stallMs, bool autoHome)
{
    emmHomePrintStep(label);

    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        Serial.println("[回零]   失败：RS485 总线繁忙，无法写入回零参数");
        return false;
    }
    const EmmV5::Status status = gEmm.modifyHomeParams(
        address, false /* SAVE=0：只改运行参数，不写掉电保存区 */,
        mode, direction, rpm, timeoutMs, stallRpm, stallMa, stallMs, autoHome,
        EMM_HOME_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK) {
        Serial.printf("[回零]   失败：A=%u 写入回零参数异常，状态=%s\n",
                      (unsigned)address, EmmV5::statusToString(status));
        return false;
    }

    LOGD("[回零]   A=%u 回零参数已全量写入(SAVE=0)：MODE=%u DIR=%u RPM=%u "
                  "TIMEOUT=%lu ms STALL_RPM=%u STALL_MA=%u STALL_MS=%u AUTO=%u\n",
                  (unsigned)address, (unsigned)mode, (unsigned)direction,
                  (unsigned)rpm, (unsigned long)timeoutMs,
                  (unsigned)stallRpm, (unsigned)stallMa, (unsigned)stallMs,
                  (unsigned)autoHome);
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

    /*
     * 下面两行是**纯观测**（2026/10/10 排查"定位命令被拒"时加的），
     * 2026/10/21 起降为 `LOGD`：9 个运动步 × 前后各一次 × 每行 2 条 = **36 行**，
     * 是上电回零日志里最大的一块噪音。需要时 `SET LOG DETAIL=1` 打开。
     */
    LOGD("[回零]   [诊断] A=%u %s：MOTOR_FLAGS=0x%02X"
                  "(使能=%u 到位=%u 堵转=%u 堵转保护=%u 左限位=%u 右限位=%u)\n",
                  (unsigned)address, stage, (unsigned)st.motorFlags,
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_ENABLED) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_POSITION_REACHED) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_STALL) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_STALL_PROTECT) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_LEFT_LIMIT) ? 1U : 0U),
                  (unsigned)((st.motorFlags & EmmV5::MOTOR_RIGHT_LIMIT) ? 1U : 0U));
    LOGD("[回零]   [诊断] A=%u %s：实时转速=%d 实时位置=%ld 位置误差=%ld 0x3B原点标志=0x%02X\n",
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
 *
 *          **判据本体已抽取到 `EmmMotionJudge.h` 的 `emmHomingFeed()`**
 *          （2026/10/10），与 `EmmTaskSeq::emmSeqWaitHomingAll` 共用同一份实现；
 *          本函数只保留轮询、中止标志、日志与"回零完成瞬间"的诊断快照。
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

        /* 判据本体见 EmmMotionJudge.h 第五节（与任务序列的 emmSeqWaitHomingAll 共用）。 */
        const EmmHomingPoll verdict =
            emmHomingFeed(flags, &homingSeen,
                          elapsed >= EMM_HOME_START_GRACE_MS);

        if (verdict == EMM_HOMING_POLL_PROTECT) {
            Serial.printf("[回零]   失败：A=%u 驱动器保护已触发（过热/过流），FLAGS=0x%02X\n",
                          (unsigned)address, flags);
            return false;
        }
        if (verdict == EMM_HOMING_POLL_FAILED) {
            Serial.printf("[回零]   失败：A=%u 驱动器报告回零失败，FLAGS=0x%02X\n",
                          (unsigned)address, flags);
            return false;
        }
        if (verdict == EMM_HOMING_POLL_NOT_STARTED) {
            Serial.printf("[回零]   失败：A=%u 从未观察到正在回零（Org_SF），"
                          "无法确认回零已执行，FLAGS=0x%02X\n",
                          (unsigned)address, (unsigned)flags);
            return false;
        }
        if (verdict == EMM_HOMING_POLL_HOMING) {
            continue;   /* 正在回零，继续等 */
        }

        /*
         * EMM_HOMING_POLL_DONE：见过 Org_SF=1 之后回到空闲，才是"回零中 → 空闲"的
         * 正常收尾（从未见过则可能只是"刚上电、从未回零"的默认状态，不能当成功）。
         */
        LOGD("[回零]   A=%u 回零完成（轮询 0x3B 确认），耗时 %lu ms，FLAGS=0x%02X\n",
                      (unsigned)address, (unsigned long)elapsed, (unsigned)flags);
        if (Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
            emmHomeDumpMotorState(address, "回零完成瞬间");
            Rs485Bus_Give();
        }
        return true;
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
    LOGD("[回零]   A=%u 定位起步保护 %lu ms（RPM=%u ACC=%u）\n",
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
            LOGD("[回零]   A=%u 定位完成（轮询 0x3A 确认）：当前位置=%ld 误差=%ld，耗时 %lu ms\n",
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
        LOGD("[回零]   A=%u 回零完成（驱动器主动上报 0x9F），MODE=%u\n",
                      (unsigned)address, (unsigned)homeMode);
        return true;
    }
    if (status == EmmV5::STATUS_OK) {
        /* 正常回退分支（驱动器 `Response` 未设 `Both` 时每条命令都会走这里），降为详细日志。 */
        LOGD("[回零]   A=%u 未收到 0x9F（命令已被接受），回退到轮询 0x3B 确认\n",
                      (unsigned)address);
        return emmHomeWaitHoming(address, maxWaitMs);
    }
    if (status == EmmV5::STATUS_NOT_MOVED) {
        /*
         * 0x12/0x22：驱动器报告"已在零点或限位已触发，电机不动"。手册明确这不是
         * 错误，但它也没有执行回零动作，因此不能直接当成功——回退到轮询确认。
         * 这也属于正常分支（重复回零时必然出现），同样降为详细日志。
         */
        LOGD("[回零]   A=%u 驱动器报告电机未动（0x12/0x22），回退到轮询 0x3B 确认\n",
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
        LOGD("[回零]   A=%u 定位完成（驱动器主动上报 0x9F）："
                      "MODE=%u DIR=%u RPM=%u ACC=%u PULSE=%lu\n",
                      (unsigned)address, (unsigned)motionMode, (unsigned)direction,
                      (unsigned)rpm, (unsigned)acceleration, (unsigned long)pulses);
        return true;
    }
    if (status == EmmV5::STATUS_OK) {
        /* 正常回退分支，降为详细日志（同 emmHomeStepTrigger）。 */
        LOGD("[回零]   A=%u 未收到 0x9F（定位命令已被接受），回退到轮询 0x3A 确认\n",
                      (unsigned)address);
        return emmHomeWaitMove(address, maxWaitMs, rpm, acceleration);
    }
    if (status == EmmV5::STATUS_NOT_MOVED) {
        /* 正常分支：已在目标位置时驱动器不动，降为详细日志。 */
        LOGD("[回零]   A=%u 驱动器报告电机未动（0x12/0x22），回退到轮询 0x3A 确认\n",
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
        LOGD("%02X ", (unsigned)ack.raw[i]);
    }
    LOGD("(FUNC=0x%02X ACK_STATUS=0x%02X)\n",
                  (unsigned)ack.function, (unsigned)ack.commandStatus);
    return false;
}

/* ==========================================================================
 * 样品盘（A=4）接近开关零点标定（2026/10/16 新增；2026/10/26 改版）
 * --------------------------------------------------------------------------
 * 用户要求：整机回零的最后一步不再交给驱动器做"就近单圈回零"（原 `HOME MODE=0`），
 * 而是用接近开关（`Y_ENDSTOP2_PIN` = GPIO6，低电平触发）标定样品盘零点。
 *
 * **当前流程（2026/10/26 用户重新定义）**：
 *   ① **速度模式**（`RPM=10 ACC=10`）**顺时针**旋转，直到接近开关触发（低电平）。
 *      —— 开关绝对位置事先未知，只能靠速度模式去找；这是全流程唯一的"非定位"运动。
 *   ② **防漂移前置**：若进入本步时开关**已经**处于触发状态（托盘停在上次标定的
 *      01 工位，而 01 工位就落在触发区内），必须先**逆时针**转出触发区直到释放。
 *      否则 ① 会立刻判定"已触发"而不转，接着又走 N 脉冲 ——
 *      **零点每回零一次就顺时针漂 N 脉冲**，几轮下来工位全偏。
 *   ③ 触发后发停止并等停稳（读 `0x43` 实时转速，连续 2 次为 0）。
 *   ④ **关闭接近开关监控**：此后托盘压着开关继续走，监控若还开着会被急停。
 *   ⑤ `MODE=0` **相对**位置模式**顺时针**再走 `N` 脉冲
 *      （`N` 默认 50，运行期可改 + 落 NVS，见 `EmmSystemHome_SetTrayZeroPulses()`）。
 *      —— **为什么这里破例用相对位置**：本工程约定 4 号电机 `DIR=0`（顺时针）=
 *      位置**递减**，从触发点（清零后的 0）继续顺时针走的绝对目标是 **−N**，
 *      而 `positionMode()` 的目标是无符号 32 位，写不进去。走完立刻清零，
 *      所以不存在"相对运动累积漂移"的问题。
 *   ⑥ **位置清零**（`0x0A/0x6D`，手册 5.2.3）：把"**前进 N 脉冲后的位置**"设为
 *      绝对 0 —— 此处即 **01 号工位**。
 *   ⑦ 把此处设为**单圈回零零点**（`0x93/0x88`，手册 5.4.1）—— 下次单圈回零回到 01 号工位。
 *   ⑧ 读回 `0x43` 核对"当前位置 ≈ 0"（**只告警、不判失败**）。
 *   （监控使能在 `emmHomeTrayHomeCore()` 进入时做；④ 处关闭、外壳退出时再关一次。）
 *
 * **历史沿革**：原流程是"触发 → 清零 → **逆时针退让** → 再清零"，退让量
 * 16 → 32 → 55 → 65 几经调整，代码里最终落在 `0`（等于没动作）。2026/10/26
 * 按用户新要求整体替换为"**触发 → 关监控 → 顺时针前进 N → 清零**"，
 * 退让相关宏（`EMM_TRAY_HOME_BACKOFF_DIR` / `_PULSES`）随之下线。
 *
 * ⚠️ `0x93` ≠ `0x0A`：`setOriginZero` 只设"单圈回零零点"，**不会**把当前位置计数清零；
 * 不清零的话任务序列 `TASK_TRAY_POS` 读到的还是标定前的旧坐标 → `SEQ ADD A=01` 时
 * 样品盘会自己转（2026/10/16 实机 bug，已由 ⑥ 清零修掉）。
 *
 * 失败沿用"失败忽略"（用户 2026/10/16 确认）：只打印，不影响整机回零结果与
 * `sys_is_homed`；失败时额外打印"零点可能无效"的醒目告警。
 * ========================================================================== */

/* ---------------- "触发后前进脉冲数"的 NVS 存取（2026/10/26 新增） ---------------- */

/** 当前生效的"触发后顺时针前进"脉冲数。 */
static uint32_t s_trayZeroPulses = EMM_TRAY_ZERO_PULSES_DEFAULT;
/** 是否已成功从 NVS 载入过（懒初始化标志）。 */
static bool s_trayZeroLoaded = false;
/** 上一次尝试打开 NVS 的时间戳，用于失败后节流重试。 */
static uint32_t s_trayZeroNvsRetryAt = 0U;

/**
 * @brief 懒加载：首次使用时从 NVS 读一次 `trayzp`。
 * @details 与 `EmmSeqPoints` 同一套口径 —— **先全置默认值，NVS 里有合法值再覆盖**，
 *          这样"NVS 里没记录（首次上电）"与"NVS 打不开（真坏了）"都不会把参数弄丢，
 *          只会退回默认值。失败时按 `EMM_TRAY_ZERO_NVS_RETRY_MS` 节流重试。
 */
static void emmTrayZeroLoadOnce(void)
{
    if (s_trayZeroLoaded) {
        return;
    }
    const uint32_t now = millis();
    if (s_trayZeroNvsRetryAt != 0U &&
        (now - s_trayZeroNvsRetryAt) < EMM_TRAY_ZERO_NVS_RETRY_MS) {
        return;
    }

    Preferences prefs;
    if (!prefs.begin(EMM_TRAY_ZERO_NVS_NS, /*readOnly=*/false)) {
        s_trayZeroNvsRetryAt = now;
        Serial.println("[回零] 样品盘前进脉冲数：NVS 打开失败，本次仍用默认值，稍后自动重试");
        return;
    }

    if (prefs.isKey(EMM_TRAY_ZERO_NVS_KEY)) {
        const uint32_t stored =
            prefs.getUInt(EMM_TRAY_ZERO_NVS_KEY, EMM_TRAY_ZERO_PULSES_DEFAULT);
        if (stored <= EMM_TRAY_ZERO_PULSES_MAX) {
            s_trayZeroPulses = stored;
        } else {
            /* 例如降级固件后上限变小，或 NVS 被写脏 —— 宁可退回默认值并告警。 */
            Serial.printf("[回零] 样品盘前进脉冲数：NVS 里 %lu 超范围（<= %lu），改用默认 %lu\n",
                          (unsigned long)stored, (unsigned long)EMM_TRAY_ZERO_PULSES_MAX,
                          (unsigned long)EMM_TRAY_ZERO_PULSES_DEFAULT);
        }
    }
    prefs.end();

    s_trayZeroLoaded = true;
    s_trayZeroNvsRetryAt = 0U;
    Serial.printf("[回零] 样品盘前进脉冲数 = %lu（%.1f°）\n",
                  (unsigned long)s_trayZeroPulses,
                  (double)s_trayZeroPulses * 360.0 / 3200.0);
}

bool EmmSystemHome_SetTrayZeroPulses(uint32_t pulses)
{
    if (pulses > EMM_TRAY_ZERO_PULSES_MAX) {
        Serial.printf("[回零] 拒绝设置样品盘前进脉冲数：%lu 超过上限 %lu\n",
                      (unsigned long)pulses, (unsigned long)EMM_TRAY_ZERO_PULSES_MAX);
        return false;
    }

    /*
     * **先落盘、再生效**：NVS 写失败时完全无副作用（变量保持旧值），
     * 避免"回 OK 了但重启就变回去"这种最难排查的情况。
     */
    Preferences prefs;
    if (!prefs.begin(EMM_TRAY_ZERO_NVS_NS, /*readOnly=*/false)) {
        Serial.println("[回零] 设置样品盘前进脉冲数失败：NVS 打开失败");
        return false;
    }
    const size_t written = prefs.putUInt(EMM_TRAY_ZERO_NVS_KEY, pulses);
    prefs.end();
    if (written == 0U) {
        Serial.println("[回零] 设置样品盘前进脉冲数失败：NVS 写入失败");
        return false;
    }

    s_trayZeroPulses = pulses;
    s_trayZeroLoaded = true;
    s_trayZeroNvsRetryAt = 0U;
    Serial.printf("[回零] 样品盘前进脉冲数已改为 %lu（%.1f°），下次标定生效\n",
                  (unsigned long)pulses, (double)pulses * 360.0 / 3200.0);
    return true;
}

uint32_t EmmSystemHome_GetTrayZeroPulses(void)
{
    emmTrayZeroLoadOnce();
    return s_trayZeroPulses;
}


/**
 * @brief 速度模式转动样品盘，直到接近开关达到指定电平（或超时 / 被中止）。
 * @details 本函数只负责"起转 → 等电平 → 发停止"，不做任何零点设置。
 *          **自行轮询 GPIO**（不依赖 `EmmEndstopMonitor` 任务），因此即使监控
 *          处于关闭状态也能正常工作 —— 手动重跑 `SET EMM SYSTEM HOME` 同样成立。
 * @param direction 转动方向（0/1）
 * @param waitLow   true = 等开关**触发**（低电平）；false = 等开关**释放**（高电平）
 * @param maxWaitMs 最长等待时间，单位 ms
 * @param label     子动作中文说明（日志用，必须指向静态存储）
 * @return true 达到目标电平（且已发停止）；false 超时、被中止或命令被拒。
 */
static bool emmHomeTraySpinUntil(uint8_t direction, bool waitLow,
                                 uint32_t maxWaitMs, const char *label)
{
    emmHomePrintSubStep(label);

    /* ---- 1. 速度模式起转 ---- */
    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        emmHomePrintSubStep("失败：RS485 总线繁忙，速度模式命令未下发");
        return false;
    }
    EmmV5::Status status = gEmm.velocity(
        EMM_SAMPLE_TRAY_MOTOR, direction, EMM_TRAY_HOME_RPM, EMM_TRAY_HOME_ACC,
        false, EMM_HOME_CMD_TIMEOUT_MS);
    Rs485Bus_Give();

    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
        /*
         * 驱动器可能要求先使能才允许进速度模式。补一次 enable 再重试一次，
         * 仍失败才判失败（现场若看到这条日志，说明该驱动器需要先使能）。
         */
        Serial.printf("[回零]   └─ 速度模式命令未被接受（状态=%s），补发使能后重试\n",
                      EmmV5::statusToString(status));
        if (Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
            gEmm.enable(EMM_SAMPLE_TRAY_MOTOR, true, false, EMM_HOME_CMD_TIMEOUT_MS);
            status = gEmm.velocity(EMM_SAMPLE_TRAY_MOTOR, direction,
                                   EMM_TRAY_HOME_RPM, EMM_TRAY_HOME_ACC,
                                   false, EMM_HOME_CMD_TIMEOUT_MS);
            Rs485Bus_Give();
        }
        if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
            Serial.printf("[回零]   └─ 失败：速度模式命令未被接受，状态=%s\n",
                          EmmV5::statusToString(status));
            return false;
        }
    }
    LOGD("[回零]   └─ 速度模式已启动：A=%u DIR=%u RPM=%u ACC=%u，等待接近开关%s\n",
                  (unsigned)EMM_SAMPLE_TRAY_MOTOR, (unsigned)direction,
                  (unsigned)EMM_TRAY_HOME_RPM, (unsigned)EMM_TRAY_HOME_ACC,
                  waitLow ? "触发（低电平）" : "释放（高电平）");

    /* ---- 2. 轮询接近开关 ---- */
    const uint32_t startedAt = millis();
    bool reached = false;
    for (;;) {
        if (emmHomeAbortRequested()) {
            emmHomePrintSubStep("收到中止请求，停止等待接近开关");
            break;
        }
        if ((digitalRead(Y_ENDSTOP2_PIN) == LOW) == waitLow) {
            reached = true;
            break;
        }
        if ((millis() - startedAt) >= maxWaitMs) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(EMM_TRAY_HOME_POLL_MS));
    }

    /* ---- 3. 无论成败都先发停止，避免速度模式一直转下去 ---- */
    if (Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        const EmmV5::Status stopStatus =
            gEmm.stop(EMM_SAMPLE_TRAY_MOTOR, false, EMM_HOME_CMD_TIMEOUT_MS);
        Rs485Bus_Give();
        /* **2026/10/21**：正常路径降为 `LOGD` —— 整机回零的样品盘步与任务序列复位阶段
           都会走到这里，每次各刷一行"已发停止命令，状态=OK"，属于流程流水账。 */
        LOGD("[回零]   └─ 已发停止命令，状态=%s\n",
             EmmV5::statusToString(stopStatus));
    } else {
        emmHomePrintSubStep("警告：RS485 总线繁忙，停止命令未下发");
    }

    if (!reached) {
        Serial.printf("[回零]   └─ 失败：等待接近开关%s超时（%lu ms）\n",
                      waitLow ? "触发" : "释放", (unsigned long)maxWaitMs);
        return false;
    }
    return true;
}

/**
 * @brief 等待样品盘停稳（速度模式发"立即停止"之后）。
 * @details 读 `0x43` 系统状态里的实时转速 `realtimeRpm`，**连续两次为 0** 才认为
 *          停稳 —— 单次为 0 可能是刚发停止、驱动器状态还没更新。读不到状态时按
 *          超时兜底。**超时不判失败**：这一步只是"确认停稳"，不是"必须成功"，
 *          免得因为一时读不到状态就把整步判死（而整步失败又只能"忽略"）。
 * @param stage 阶段名（日志用，必须指向静态存储）
 * @return 恒返回 true。
 */
static bool emmHomeTrayWaitStopped(const char *stage)
{
    const uint32_t startedAt = millis();
    uint8_t stillCount = 0;

    for (;;) {
        if (emmHomeAbortRequested()) {
            return true;
        }
        if (Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
            EmmV5::SystemState state{};
            const EmmV5::Status status = gEmm.readSystemState(
                EMM_SAMPLE_TRAY_MOTOR, state, EMM_HOME_CMD_TIMEOUT_MS);
            Rs485Bus_Give();
            if (status == EmmV5::STATUS_OK) {
                if (state.realtimeRpm == 0) {
                    if (++stillCount >= 2U) {
                        LOGD("[回零]   └─ 样品盘%s已停稳（转速=0，位置=%ld）\n",
                                      stage, (long)state.realtimePosition);
                        return true;
                    }
                } else {
                    stillCount = 0;
                }
            }
        }
        if ((millis() - startedAt) >= EMM_TRAY_HOME_STOP_MAX_WAIT_MS) {
            Serial.printf("[回零]   └─ 提示：%s停稳确认超时（%lu ms），继续执行\n",
                          stage, (unsigned long)EMM_TRAY_HOME_STOP_MAX_WAIT_MS);
            return true;
        }
        /* 本处每轮都读一次驱动器状态（总线事务），用**独立的 100 ms** 宏，
           不要跟着 GPIO 轮询降到 10 ms —— 见 EMM_TRAY_HOME_STOP_POLL_MS 的注释。 */
        vTaskDelay(pdMS_TO_TICKS(EMM_TRAY_HOME_STOP_POLL_MS));
    }
}

/**
 * @brief 把样品盘**当前位置计数清零**（`0x0A/0x6D`，手册 5.2.3）。
 * @details **2026/10/16 新增**：`0x93/0x88`（设单圈回零零点）**不会**改动当前位置计数，
 *          要让"当前位置 = 0"必须单独发 `0x0A`。
 *          **2026/10/26 改版后只用一次**：把"**触发后顺时针前进 `N` 脉冲的位置**"
 *          设为绝对 0 —— 这里就是 **01 号工位**（原流程是"触发点清零 + 退让后清零"
 *          两次，退让段已下线）。
 * @param label 子动作中文说明（日志用，必须指向静态存储）。
 * @return true 成功；false 总线繁忙或驱动器未确认。
 */
static bool emmHomeTrayClearPosition(const char *label)
{
    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        emmHomePrintSubStep("失败：RS485 总线繁忙，位置清零未下发");
        return false;
    }
    const EmmV5::Status status =
        gEmm.resetCurrentPosition(EMM_SAMPLE_TRAY_MOTOR, EMM_HOME_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
        Serial.printf("[回零]   └─ 失败：%s未成功，状态=%s\n",
                      label, EmmV5::statusToString(status));
        return false;
    }
    LOGD("[回零]   └─ 样品盘%s完成（当前位置 = 绝对 0）\n", label);
    return true;
}

/**
 * @brief 样品盘零点标定的主体（不含"使能 / 关闭监控"与失败告警）。
 * @details 子动作顺序（**2026/10/26 版**，详见上方"样品盘（A=4）接近开关零点标定"）：
 *            ⓪ **防漂移**：若开关此刻已触发，先逆时针转出触发区直到释放；
 *            ① 速度模式顺时针转到接近开关触发（全流程唯一的非定位运动）；
 *            ② **关闭接近开关监控**（此后开关被压住也不会再急停）；
 *            ③ `MODE=0` **相对**位置模式顺时针再走 `N` 脉冲
 *               （`N` 运行期可改 + 落 NVS，默认 50）；
 *            ④ `0x0A` 位置清零，把**前进后的位置**设为绝对 0（= **01 号工位**）；
 *            ⑤ `0x93/0x88` 设**单圈回零零点**；
 *            ⑥ 读回 `0x43` 核对当前位置 ≈ 0（**只告警、不判失败**）。
 * @return true 标定成功；false 任一步失败。
 */
static bool emmHomeTrayZeroViaSwitchInner(void)
{
    /* 触发后前进的脉冲数：运行期可改（`SET EMM TRAY ZERO P=<n>`），此处只读一次快照。 */
    const uint32_t advancePulses = EmmSystemHome_GetTrayZeroPulses();

    /*
     * ⓪ **防漂移前置（2026/10/26 新增，必须保留）**
     *
     * 01 号工位定在"触发点再顺时针 `N` 脉冲"处，也就是**落在触发区内部**。
     * 于是下一次回零开始时托盘就停在 01 工位、开关是低电平 —— 若不先退出来，
     * 下面的"等触发"会**立刻**判定成功（`digitalRead == LOW == waitLow`），
     * 一秒都不转，接着又顺时针走 `N` 脉冲：**零点每回零一次漂 `N` 脉冲**，
     * 几轮下来工位全偏，而且是单向累积、无法自愈。
     *
     * 先逆时针转出触发区（等开关释放），保证每次都从同一条**上升沿之外**起步，
     * 找出来的触发点才是同一个物理位置。
     */
    if (digitalRead(Y_ENDSTOP2_PIN) == LOW) {
        emmHomePrintSubStep("接近开关已处于触发状态，先逆时针转出触发区");
        if (!emmHomeTraySpinUntil((uint8_t)EMM_TRAY_HOME_ESCAPE_DIR, /*waitLow=*/false,
                                  EMM_HOME_TRIGGER_MAX_WAIT_MS,
                                  "逆时针旋转至接近开关释放")) {
            return false;
        }
        emmHomeTrayWaitStopped("转出触发区后");
        if (emmHomeAbortRequested()) {
            emmHomePrintSubStep("收到中止请求，放弃零点标定（未设置任何零点）");
            return false;
        }
    }

    /*
     * ① 顺时针旋转，直到接近开关触发（**速度模式**，没有位置目标 —— 开关的绝对位置
     *    事先未知，只能靠速度模式去找）。
     */
    if (!emmHomeTraySpinUntil((uint8_t)EMM_TRAY_HOME_DIR, /*waitLow=*/true,
                              EMM_HOME_TRIGGER_MAX_WAIT_MS,
                              "顺时针旋转至接近开关触发")) {
        return false;
    }
    emmHomeTrayWaitStopped("触发后");

    /*
     * ⚠️ **中止检查（2026/10/19 新增，必须保留）**：
     * `emmHomeTrayWaitStopped()` 在收到中止时也返回 true（它只负责"等到不动"），
     * 所以这里必须显式判一次。否则"开关刚触发、正在等停稳时收到 STOP"会继续往下走
     * ②~⑤，把**半路的位置**设成单圈零点 —— 这比不回零更糟（零点被污染，后续取位全偏）。
     */
    if (emmHomeAbortRequested()) {
        emmHomePrintSubStep("收到中止请求，放弃零点标定（不设置任何零点）");
        return false;
    }

    /*
     * ② **关闭接近开关监控（用户 2026/10/26 要求："触发限位后关闭触发开关"）**
     *
     * 因为 ③ 要**继续顺时针往触发区里走**，开关会一直被压着。监控若还开着，
     * 它的"下降沿即急停"逻辑会把这一步直接打断（即便没有新下降沿，也留着风险）。
     * 外壳 `emmHomeTrayHomeCore()` 退出时还会再关一次，重复调用是幂等的。
     */
    EmmEndstopMonitor_SetEnabled(false);
    emmHomePrintSubStep("已关闭接近开关监控");

    /*
     * ③ `MODE=0` **相对**位置模式，**顺时针**再走 `advancePulses` 脉冲。
     *
     * **为什么这里破例用相对位置**：本工程约定 4 号电机 `DIR=0`（顺时针）= 位置
     * **递减**（见 `EmmTaskSeq::emmSeqRunTrayPos` 的选向规则：`dir = delta > 0 ? 1 : 0`）。
     * 从触发点（此时还没清零，但相对量为 0）继续顺时针走的**绝对目标会是 −N**，
     * 而 `positionMode()` 的目标参数是无符号 32 位，写不进去；绕圈（目标 3200−N）
     * 又依赖驱动器是否按单圈回绕，行为不确定。
     * 走完 ④ 立刻清零，所以"相对运动累积漂移"这条顾虑在这里不成立。
     */
    if (advancePulses > 0UL) {
        if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
            emmHomePrintSubStep("失败：RS485 总线繁忙，前进命令未下发");
            return false;
        }
        const EmmV5::Status advanceStatus = gEmm.positionMode(
            EMM_SAMPLE_TRAY_MOTOR, (uint8_t)EMM_TRAY_HOME_ADVANCE_DIR,
            EMM_TRAY_HOME_RPM, EMM_TRAY_HOME_ACC,
            advancePulses, /*motionMode=*/0U /* MODE=0 相对位置 */,
            false, EMM_HOME_MOVE_MAX_WAIT_MS, /*requireDone=*/true);
        Rs485Bus_Give();

        bool advanceOk = (advanceStatus == EmmV5::STATUS_DONE);
        if (!advanceOk && (advanceStatus == EmmV5::STATUS_OK ||
                           advanceStatus == EmmV5::STATUS_NOT_MOVED)) {
            /* 没收到 0x9F 时回退到轮询确认，与其它定位步骤同一套兜底。 */
            advanceOk = emmHomeWaitMove(EMM_SAMPLE_TRAY_MOTOR, EMM_HOME_MOVE_MAX_WAIT_MS,
                                        EMM_TRAY_HOME_RPM, EMM_TRAY_HOME_ACC);
        }
        if (!advanceOk) {
            Serial.printf("[回零]   └─ 失败：顺时针前进 %lu 脉冲未完成，状态=%s\n",
                          (unsigned long)advancePulses,
                          EmmV5::statusToString(advanceStatus));
            return false;
        }
        LOGD("[回零]   └─ 已顺时针前进 %lu 脉冲（%.1f°）\n",
                      (unsigned long)advancePulses,
                      (double)advancePulses * 360.0 / 3200.0);
    } else {
        LOGD("[回零]   └─ 前进脉冲数为 0：触发点即 01 号工位\n");
    }

    /* ⚠️ 中止检查：前进命令是阻塞式（`requireDone=true`），等待期间收到 STOP 无法打断，
       走完之后必须在这里拦住，否则会把"前进到一半"的位置当成 01 号工位设成零点。 */
    if (emmHomeAbortRequested()) {
        emmHomePrintSubStep("收到中止请求，放弃零点标定（前进后未设零点）");
        return false;
    }

    /* ④ 把"前进 N 脉冲后的位置"清零 —— 此处即 **01 号工位**（绝对脉冲 0）。 */
    if (!emmHomeTrayClearPosition("01 号工位位置清零")) {
        return false;
    }

    /* ⚠️ 中止检查：位置已清零但**零点还没设**，此时退出最干净 ——
       驱动器里的单圈零点仍是上一次的值，不会被这次的半成品覆盖。 */
    if (emmHomeAbortRequested()) {
        emmHomePrintSubStep("收到中止请求，放弃零点标定（当前位置已清零，单圈零点未改）");
        return false;
    }

    /* ⑤ 把此处设为**单圈回零零点**（`0x93/0x88`）—— 下次单圈回零会回到 01 号工位。 */
    if (!Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        emmHomePrintSubStep("失败：RS485 总线繁忙，单圈零点未设置");
        return false;
    }
    const EmmV5::Status zeroStatus = gEmm.setOriginZero(
        EMM_SAMPLE_TRAY_MOTOR, /*save=*/false, EMM_HOME_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (zeroStatus != EmmV5::STATUS_OK && zeroStatus != EmmV5::STATUS_DONE) {
        Serial.printf("[回零]   └─ 失败：设置单圈回零零点未成功，状态=%s\n",
                      EmmV5::statusToString(zeroStatus));
        return false;
    }

    /*
     * ⑥ 读回 `0x43` 核对"当前位置 ≈ 0"（= 01 号工位）。**只告警、不判失败**。
     *
     * ⚠️ 历史坑（2026/10/16 实机 bug，已由上面 ④ 的 `0x0A` 位置清零修掉）：
     * `0x93/0x88`（setOriginZero）只设置"**单圈回零的零点位置**"，**不会**改动驱动器的
     * **当前位置计数**；要让"当前位置 = 0"必须另发 `0x0A/0x6D`。两者是两条独立命令
     * （手册 5.4.1 vs 5.2.3），KWQ1 原流程也是两条都发
     * （`SET EMM ORIGIN ZERO` + `SET EMM ZERO`）。
     *
     * 漏发 `0x0A` 的现象：任务序列 `TASK_TRAY_POS` 用 `0x43.realtimePosition` 当基准、
     * 按"工位 N ↔ 绝对脉冲 (N−1)×320"算目标，于是 `SEQ ADD A=01` 时目标 = 0，而实际
     * 读数还是标定前的旧值 → 判成"不在 01 号工位" → **样品盘自己转起来**。
     */
    if (Rs485Bus_Take(EMM_HOME_BUS_TIMEOUT_MS)) {
        EmmV5::SystemState trayState = {};
        const EmmV5::Status readStatus = gEmm.readSystemState(
            EMM_SAMPLE_TRAY_MOTOR, trayState, EMM_HOME_CMD_TIMEOUT_MS);
        Rs485Bus_Give();
        if (readStatus == EmmV5::STATUS_OK) {
            const long pos = (long)trayState.realtimePosition;
            if (pos > EMM_TRAY_HOME_ZERO_TOL_COUNTS ||
                pos < -EMM_TRAY_HOME_ZERO_TOL_COUNTS) {
                LOGD("[回零]   └─ 警告：位置清零后读数仍为 %ld 计数（应≈0），"
                              "样品盘零点可能无效，任务取位会整体偏移\n", pos);
            } else {
                LOGD("[回零]   └─ 已核对：样品盘当前位置 %ld 计数 ≈ 0\n", pos);
            }
        }
    }

    emmHomePrintSubStep("零点已标定：当前位置 = 单圈回零零点（= 01 号工位 = 绝对脉冲 0）");
    return true;
}

/**
 * @brief 样品盘零点标定的**公共外壳**：使能监控 → 执行 → 无论成败关闭监控 → 失败告警。
 * @details **2026/10/19 抽出**。供两条路径共用，保证"逻辑与流程逐字一致"：
 *            - 整机回零第 17 步 `emmHomeStepTrayZeroViaSwitch()`；
 *            - 样品盘单独回零 `EmmSystemHome_TrayHome()`（上位机 `SET EMM TRAY HOME`）。
 *          两者唯一的差别是抬头打印 —— 本函数**不打抬头**，由调用方负责。
 * @return true 标定成功；false 失败。
 */
static bool emmHomeTrayHomeCore(void)
{
    /* 使能接近开关监控（双保险：sysPowerOffCleanup() 断电收尾时也会使能一次）。 */
    EmmEndstopMonitor_SetEnabled(true);

    const bool ok = emmHomeTrayZeroViaSwitchInner();

    /*
     * 无论成败都关闭（用户 2026/10/16 确认）：失败后系统仍会照常进入正常运行，
     * 若监控还开着，托盘正常转位经过零点会被它停住，表现为"偶发停一下"。
     */
    EmmEndstopMonitor_SetEnabled(false);

    if (!ok) {
        Serial.println("[回零]   警告：样品盘零点标定未成功 —— 零点可能无效，"
                       "后续任务取位可能偏差");
    }
    return ok;
}

/**
 * @brief 整机回零**最后一步**：样品盘（A=4）零点标定（接近开关）。
 * @details **2026/10/16 新增，取代原来的 `HOME MODE=0` 就近单圈回零**（用户要求）。
 *          对外是"第 17 步 / 共 17 步"，
 *          内部子动作只打印、不计数。
 *          本函数只负责打印步骤抬头；标定本体与失败告警在 `emmHomeTrayHomeCore()`。
 * @return true 标定成功；false 失败（调用方只打印、不中止，用户 2026/10/16 确认）。
 */
static bool emmHomeStepTrayZeroViaSwitch(void)
{
    emmHomePrintStep("样品盘零点标定(接近开关)");
    return emmHomeTrayHomeCore();
}

/**
 * @brief 执行 17 步整机回零序列。
 * @details 顺序固定（**2026/10/23 改版：3 号与 2 号交替两轮**）：
 *          3 号回零 → 下 1200 → 2 号回零 → 3 号回零 → 下 1200
 *          → 2 号回零 → 顺 800 → 2 号回零 → 3 号回零 → 下 2500 → 样品盘。
 *          换算成步数是：3 号 3 次碰撞回零 × (写参数 + 触发) = 6 步，
 *          2 号 3 次碰撞回零 × (写参数 + 触发) = 6 步，
 *          加 4 次绝对定位 + 样品盘 1 步 = **17 步**。
 *
 *          **为什么改成交替**：2 号旋转会带动 3 号的丝杆（单向耦合）。
 *          旧流程是"3 号先做完一大套（快回零 → 下 10000 → 慢回零 → 下 2200），
 *          再让 2 号转，最后补一次 3 号二次复校"。新流程把顺序反过来 ——
 *          **每次 2 号要转之前，先让 3 号回零 + 退让 1200**，用"复位前置"
 *          代替"事后补校"，3 号零点在每一轮 2 号动作开始时都是准的。
 *
 *          **速度全部统一（用户 2026/10/23 选定）**：3 号三次回零全 40 RPM
 *          （旧流程第 1 次是 45），2 号三次全 8 RPM（旧流程第 1 次是 12）。
 *          三次 2 号回零里第 2 次在物理上是冗余的（第 1 次之后 2 号没动过），
 *          用户要求保留，用来兜住"第 1 次回零残留偏差"。
 *
 *          样品盘改用**接近开关标定零点**（2026/10/16，见下方
 *          `emmHomeStepTrayZeroViaSwitch`），标定失败按配置忽略，不影响整机回零结果。
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

    /*
     * ==========================================================================
     * **2026/10/23 流程改版（用户逐条指定）**
     * --------------------------------------------------------------------------
     * 旧流程（15 步）：
     *   3 号快回零(45) → 下 10000 → 3 号慢回零(40) → 下 2200
     *   → 2 号快回零(12) → 顺 800 → 2 号慢回零(8)
     *   → 3 号二次复校(40) → 下 2500 → 样品盘
     *
     * 新流程（17 步）：**3 号回零与 2 号回零交替两轮**
     *   3 号回零(40) → 下 1200
     *   → 2 号回零(8)
     *   → 3 号回零(40) → 下 1200
     *   → 2 号回零(8) → 顺 800 → 2 号回零(8)
     *   → 3 号回零(40) → 下 2500
     *   → 样品盘
     *
     * **为什么 3 号要回零三次（2 号每转一轮就重标一次）**：
     *   2 号旋转会**带动 3 号的丝杆**（单向耦合：2 号→3 号，3 号动作不影响 2 号）。
     *   所以第 2、3 次 3 号回零都在"上一轮 2 号刚转完"之后，用来撤销被带出来的偏移；
     *   而每次 3 号回零后都要**退让 1200**，让 3 号离开碰撞限位 —— 否则下一轮 2 号
     *   带着 3 号继续往负方向走会顶死。
     *   旧流程只在 2 号转完后做一次"二次复校"，新流程改成"每轮 2 号动作前后各复位一次"。
     *
     * **三次 2 号回零全部保留（含物理上冗余的第 2 次）**：用户 2026/10/23 选定。
     *   第 1 次之后 2 号没动过（3 号动作不会移动 2 号），第 2 次在物理上是空转，
     *   但它能兜住"第 1 次回零残留偏差"，代价只有零点附近的几秒。
     *
     * **速度统一（用户 2026/10/23 选定，不再区分快/慢）**：
     *   - 3 号三次回零全用 RPM=40（旧流程第 1 次是 45）；
     *   - 2 号三次回零全用 RPM=8（旧流程第 1 次是 12）。
     * ==========================================================================
     */

    /* ================= ① 3 号第一次回零 + 退让 1200 ================= */
    emmHomeEvent(HOME_AXIS_3, "AXIS 3");
    /* HOME PARAM A=3 MODE=2 DIR=0 RPM=40 TIMEOUT=30000
       STALL_RPM=35 STALL_MA=400 STALL_MS=10 AUTO=0
       （2026/10/23：三次统一用 40 RPM；旧流程第 1 次是 45。） */
    if (!emmHomeStepParam(EMM_LIFT_MOTOR,
                          "升降电机写入回零参数(DIR=0 RPM=40 1/3)",
                          2, 0, 40, 30000, 35, 400, 10, false)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_LIFT_MOTOR, "升降电机碰撞回零 1/3",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }
    /*
     * 退让 1200（2026/10/23 由 10000 改为 1200，用户确认余量够）：
     *   - 让 3 号离开碰撞限位，给下一次 2 号转动留出被带动的空间；
     *   - 回零后零点=0，所以"下降 1200 脉冲"与"绝对定位到 1200"数值等价，
     *     这里用 MODE=1 绝对定位，与流程里其它定位步保持同一套语义。
     */
    if (!emmHomeStepMove(EMM_LIFT_MOTOR, "升降电机下降至绝对位置 1200",
                         1, 50, 20, 1250, 1, EMM_HOME_MOVE_MAX_WAIT_MS)) {
        return false;
    }

    /* ================= ② 2 号第一次回零 ================= */
    emmHomeEvent(HOME_AXIS_2, "AXIS 2");
    /* HOME PARAM A=2 MODE=2 DIR=1 RPM=8 TIMEOUT=15000
       STALL_RPM=10 STALL_MA=400 STALL_MS=10 AUTO=0
       （2026/10/23：三次统一用慢速 RPM=8；TIMEOUT 由 10000 放宽到 15000 ——
         第一次回零可能要从检测位 2850 走回 0，@8 RPM 约 6.7 s，
         原来 10000 的余量偏薄。） */
    if (!emmHomeStepParam(EMM_ROTARY_MOTOR,
                          "旋转电机写入回零参数(DIR=1 RPM=8 1/3)",
                          2, 1, 8, 15000, 10, 400, 10, false)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_ROTARY_MOTOR, "旋转电机碰撞回零 1/3",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }

    /* ================= ③ 3 号第二次回零 + 退让 1200 ================= */
    emmHomeEvent(HOME_AXIS_3, "AXIS 3");
    /* 2 号刚转过，3 号已被带动 —— 必须重新回零，再退让给下一轮。 */
    if (!emmHomeStepParam(EMM_LIFT_MOTOR,
                          "升降电机写入回零参数(DIR=0 RPM=40 2/3)",
                          2, 0, 40, 30000, 35, 400, 10, false)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_LIFT_MOTOR, "升降电机碰撞回零 2/3",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }
    if (!emmHomeStepMove(EMM_LIFT_MOTOR, "升降电机下降至绝对位置 1200(第2次)",
                         1, 50, 20, 1250, 1, EMM_HOME_MOVE_MAX_WAIT_MS)) {
        return false;
    }

    /* ================= ④ 2 号第二、三次回零（中间顺时针 800） ================= */
    emmHomeEvent(HOME_AXIS_2, "AXIS 2");
    if (!emmHomeStepParam(EMM_ROTARY_MOTOR,
                          "旋转电机写入回零参数(DIR=1 RPM=8 2/3)",
                          2, 1, 8, 15000, 10, 400, 10, false)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_ROTARY_MOTOR, "旋转电机碰撞回零 2/3",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }
    /* 顺时针走一段再回零，用来验证零点重复性（与旧流程完全一致）。 */
    if (!emmHomeStepMove(EMM_ROTARY_MOTOR, "旋转电机移动至绝对位置 800",
                         0, 5, 5, 800, 1, EMM_HOME_MOVE_MAX_WAIT_MS)) {
        return false;
    }
    if (!emmHomeStepParam(EMM_ROTARY_MOTOR,
                          "旋转电机写入回零参数(DIR=1 RPM=8 3/3)",
                          2, 1, 8, 15000, 10, 400, 10, false)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_ROTARY_MOTOR, "旋转电机碰撞回零 3/3",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }

    /* ================= ⑤ 3 号第三次回零 + 落到 2500 ================= */
    emmHomeEvent(HOME_AXIS_3, "AXIS 3");
    if (!emmHomeStepParam(EMM_LIFT_MOTOR,
                          "升降电机写入回零参数(DIR=0 RPM=40 3/3)",
                          2, 0, 40, 30000, 35, 400, 10, false)) {
        return false;
    }
    if (!emmHomeStepTrigger(EMM_LIFT_MOTOR, "升降电机碰撞回零 3/3",
                            2, EMM_HOME_TRIGGER_MAX_WAIT_MS)) {
        return false;
    }
    /*
     * 回零后 3 号停在碰撞零点（计数 0），补一条定位回 2500：
     *   ① 不让 3 号一直压在机械限位上；
     *   ② 整机回零的**结束位置与改动前一致**（用户 2026/10/20 选定，2026/10/23 沿用）。
     * 速度/加速度沿用旧流程：`DIR=1 RPM=50 ACC=20 PULSE=2500 MODE=1`。
     */
    if (!emmHomeStepMove(EMM_LIFT_MOTOR, "升降电机下降至绝对位置 2500",
                         1, 50, 20, 2500, 1, EMM_HOME_MOVE_MAX_WAIT_MS)) {
        return false;
    }

    /* ---- 样品盘（A=4）：1 步，失败忽略（2026/10/16 改为接近开关标定零点） ---- */
    emmHomeEvent(HOME_AXIS_4, "AXIS 4");
    if (!emmHomeStepTrayZeroViaSwitch()) {
        Serial.println("[回零]   样品盘零点标定未成功，按配置忽略并继续");
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

        LOGD("[回零] ========== 整机回零开始 ==========\n");
        /*
         * `HOME BEGIN` 事件（2026/10/21）：上电走 `SysInit_Run()` 自动触发，
         * **没有命令应答**，上位机只能靠这条确认"回零真的开始了"。
         * 手动 `SET EMM SYSTEM HOME` 已经有 `OK @n … STARTED`，这条是冗余但无害。
         */
        emmHomeEvent(HOME_BEGIN, "BEGIN");
        const bool ok = emmHomeRunSequence();

        if (!ok) {
            LOGD("[回零] 流程未完成，正在停止全部 EMM 电机\n");
            emmHomeStopAllMotors();
        }

        /*
         * 把回零结果写入全局标志 `sys_is_homed`：供 GET STATUS 的 HOMING 字段
         * 与后续"开始任务"的前置检查（回零检测）使用。失败或中止时清零，避免
         * 拿"上一次回零成功"的旧状态去放行任务。
         * 注意：样品盘回零按配置"失败忽略"，不影响本标志（见 emmHomeRunSequence）。
         */
        sys_is_homed = ok;
        LOGD("[回零] sys_is_homed=%d（%s）\n",
                      sys_is_homed ? 1 : 0,
                      ok ? "已建立零点" : "回零未完成");

        /*
         * **2026/10/21 新增：把回零结果上报给上位机**。
         *
         * 此前整机回零的成败**只写在串口日志里** —— 上位机只能靠轮询
         * `GET STATUS` 的 `HOMING` 字段（0/1）去猜，分不清"还在跑 / 失败 /
         * 被 `SET EMM SYSTEM HOME STOP` 主动中止"。
         *
         * 三种结局各发一帧（`EMM_HOME_EVT_SEQ` = 100，与任务序列同约定）：
         *   - 成功        → `EVT 360 HOME DONE`
         *   - 主动中止    → `EVT 362 HOME STOPPED`
         *   - 失败        → `ERR 整机回零失败`
         *
         * **2026/10/21 改版**：失败帧从 `EVT ERR 361 HOME FAIL` 改成 `ERR @…`
         * —— 上位机现在**只把 `ERR` 当板子报错**，`EVT ERR` 会被当未知事件忽略，
         * 导致"回零失败了但上位机毫无反应"。`361` 这个码位因此不再发送
         * （枚举保留，避免影响既有文档/上位机的码表）。
         * 失败的具体原因仍在串口 `[回零]   失败：…` 那一行（关键日志，不受
         * `SET LOG DETAIL` 影响）。
         *
         * ⚠️ 必须在下面把 `s_emmHomeStopRequested` 清零**之前**判断，
         *    否则"被中止"会被误判成"失败"。
         */
        if (ok) {
            sendWithCRC("EVT %d HOME DONE", (int)HOME_DONE);
        } else if (s_emmHomeStopRequested) {
            sendWithCRC("EVT %d HOME STOPPED", (int)HOME_STOPPED);
        } else {
            sendWithCRC("ERR 整机回零失败");
        }

        s_emmHomeBusy = false;
        s_emmHomeStopRequested = false;
        LOGD("[回零] ========== 整机回零%s ==========\n",
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

    LOGD("[回零] 整机回零任务已启动：优先级=%d 核心=%d 步数=%u\n",
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
        LOGD("[回零] 已拒绝启动：整机回零正在运行\n");
        return false;
    }

    /*
     * 任务序列（L2，EmmTaskSeq）运行期间独占机械机构，不允许再启动整机回零。
     * 本函数是"启动回零"的**唯一入口**（上位机命令 `SET EMM SYSTEM HOME` 与
     * 系统初始化 `SysInit` 都走它），所以互斥判据放在这里可一次覆盖两条路径：
     *   - 上位机路径本已被 EmmTaskSeq_BlocksCommand() 在 dispatch() 拦下；
     *   - 但 SysInit 是内部直接调用，不经过 dispatch()，必须在这里兜住，
     *     否则会出现"任务序列与整机回零并行跑"（实机已复现）。
     */
    if (EmmTaskSeq_IsActive()) {
        LOGD("[回零] 已拒绝启动：任务序列正在运行\n");
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
        LOGD("[回零] 当前没有正在运行的整机回零流程\n");
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
     *   - SET EMM LIFT MONITOR：只切换本地布防标志，不产生总线事务；
     *   - SET EMM POINT：**2026/10/21 新增**。只改内存变量 + 写 NVS，
     *     不产生任何 RS485 总线事务、不触发运动，与"回零独占 2/3/4 号电机"
     *     不冲突。回零流程本身**不读点位**，所以运行中改点位对本次回零毫无影响，
     *     纯粹是"趁回零时把参数提前调好，回零结束立刻能跑"。
     *     `RESET=1`（一次改 9 个）仍由 `cmdSetEmmPoint()` 自己拒绝。
     *   - SET EMM TRAY ZERO：**2026/10/26 新增**。只改"下次样品盘标定用"的脉冲数
     *     （内存变量 + NVS），不产生总线事务、不触发运动。标定流程在进入时只读一次
     *     快照，所以运行中改对本次回零无影响，下次生效。
     */
    if (strcmp(commandName, "SET EMM SYSTEM HOME") == 0 ||
        strcmp(commandName, "SET EMM SYSTEM HOME STOP") == 0 ||
        strcmp(commandName, "SET EMM LIFT MONITOR") == 0 ||
        strcmp(commandName, "SET EMM POINT") == 0 ||
        strcmp(commandName, "SET EMM TRAY ZERO") == 0) {
        return false;
    }

    /*
     * 回零期间独占 2/3/4 号电机：拦截所有会触发动作或修改电机状态的
     * SET EMM 命令；查询类 GET EMM 命令保持可用，便于现场诊断。
     */
    return strncmp(commandName, "SET EMM", 7) == 0;
}

bool EmmSystemHome_TrayHome(void)
{
    /*
     * 与整机回零共用同一批内部函数（`emmHomeTrayHomeCore()`），因此必须复用
     * 同一套互斥判据：
     *   - 整机回零运行中：两条流程会同时抢 4 号电机，还会互相清零点；
     *   - 任务序列运行中：序列正按工位取放样品，此时转盘会撞机构。
     */
    if (s_emmHomeBusy) {
        LOGD("[回零] 已拒绝样品盘单独回零：整机回零正在运行\n");
        return false;
    }
    if (EmmTaskSeq_IsActive()) {
        LOGD("[回零] 已拒绝样品盘单独回零：任务序列正在运行\n");
        return false;
    }
    /*
     * 系统初始化（`SysInit`）走到最后一定会调用 `EmmSystemHome_Start()` 启动整机
     * 回零；若此时放行单独回零，两条流程会并行抢 4 号电机。因此初始化期间一并拒绝。
     */
    if (SysInit_IsActive()) {
        LOGD("[回零] 已拒绝样品盘单独回零：系统初始化正在运行\n");
        return false;
    }

    /*
     * 复位中止标志：`emmHomeTraySpinUntil()` / `emmHomeTrayWaitStopped()` 会在
     * 每个轮询点检查它。上一次整机回零被中止时该标志可能仍为 true，若不清零，
     * 本次会在第一步就"收到中止请求"而退出。
     */
    s_emmHomeStopRequested = false;

    Serial.println("[回零] ===== 样品盘（A=4）单独回零开始 =====");
    const bool ok = emmHomeTrayHomeCore();
    Serial.printf("[回零] ===== 样品盘单独回零%s =====\n", ok ? "完成" : "失败");
    return ok;
}

/**
 * @brief 外部中止钩子：把"序列是否已被 STOP"翻译成本模块能懂的 bool。
 * @details 判据与 `EmmTaskSeq` 的 `emmSeqHandlePauseResume()` **完全一致**：
 *          只有序列**既不是 RUNNING 也不是 PAUSED**（即 STOPPED / IDLE）才算中止。
 *          这样 `SEQ PAUSE` **不会**打断找开关 —— 找开关是不可分割的动作，
 *          中途停下再恢复只能从头重找，停与不停没有区别。
 * @note 只在 `EmmSystemHome_TrayHomeForSeq()` 执行期间被注册，退出即注销。
 */
static bool emmTrayHomeSeqAbortQuery(void)
{
    const SeqState st = EmmTaskQueue_GetSeqState();
    return (st != RUNNING) && (st != PAUSED);
}

/**
 * @brief 供**任务序列复位阶段**调用的样品盘回零入口（2026/10/19 新增）。
 *
 * @details 与 `EmmSystemHome_TrayHome()` 的**唯一差别**：
 *            1. **不检查** `EmmTaskSeq_IsActive()` —— 调用方就是序列自己，那道检查必然为 true；
 *            2. 额外注册外部中止钩子，让序列的 `SEQ STOP` 能**立即打断**本流程。
 *
 *          标定本体、接近开关监控的使能/关闭、失败告警全部复用 `emmHomeTrayHomeCore()`，
 *          与 `SET EMM TRAY HOME`、整机回零第 17 步**逐字一致**，不存在第二套实现。
 *
 *          **仍然保留的互锁**：整机回零运行中、系统初始化运行中。这两种情况都会
 *          并行抢 4 号电机（正常路径下序列与它们互斥，这里是兜底）。
 *
 *          **中止语义**：
 *            - `SEQ STOP` 把 `SeqState` 置成 STOPPED → 钩子返回 true → 等待循环在
 *              下个轮询点（`EMM_TRAY_HOME_POLL_MS` = 10 ms）退出并发停止帧，
 *              `emmHomeTrayZeroViaSwitchInner()` 在三个关键点显式判中止，
 *              **绝不会把半路位置设成零点**；
 *            - 本函数返回 false → `emmSeqRunReset()` 返回 `SEQ_STEP_FAILED`，但此时
 *              序列已不是 RUNNING，`emmSeqHandleStepFailure()` 会走"已被中止、
 *              不按失败收尾"分支，不会误报失败、也不会误发 `SEQ ALL DONE`；
 *            - `SEQ PAUSE` 不打断（判据同 `emmSeqHandlePauseResume()`）。
 *
 * @return true 标定成功；false 失败或被中止。
 */
bool EmmSystemHome_TrayHomeForSeq(void)
{
    if (s_emmHomeBusy) {
        LOGD("[任务序列] 【复位】拒绝样品盘回零：整机回零正在运行\n");
        return false;
    }
    if (SysInit_IsActive()) {
        LOGD("[任务序列] 【复位】拒绝样品盘回零：系统初始化正在运行\n");
        return false;
    }

    /* 上一次整机回零被中止时该标志可能仍为 true，不清零会在第一步就"收到中止"退出。 */
    s_emmHomeStopRequested = false;

    /* 注册外部判据（STOP 立即打断）。 */
    s_externalAbortQuery = emmTrayHomeSeqAbortQuery;

    LOGD("[任务序列] 【复位】样品盘（A=4）回零开始（接近开关标定）\n");
    const bool ok = emmHomeTrayHomeCore();

    /* **无条件注销**：绝不把序列的判据留给后续的整机回零使用。 */
    s_externalAbortQuery = nullptr;

    /* **2026/10/21**：降为 `LOGD` —— 复位阶段成功时会发 `EVT 352 RESET AXIS 4`，
       失败时 `emmSeqRunReset()` 自己会打一行"4 号样品盘未回零…"。 */
    LOGD("[任务序列] 【复位】样品盘回零%s\n", ok ? "完成" : "失败或被中止");
    return ok;
}

bool cmdSetEmmTrayHome(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    /* 与 L1 运动命令一致：结构电源关闭时拒绝，避免"以为在回零其实电机没电"。 */
    if (cmd_power_off) {
        LOGD("[回零] @%d 拒绝样品盘单独回零：步进电机电源已关闭\n", seq);
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }

    /*
     * 互斥判据在 `EmmSystemHome_TrayHome()` 内部也有一道（供内部调用方兜底）；
     * 这里先判一次，是为了能把"被复合动作占用"和"回零执行失败"回成不同的错误码。
     */
    if (s_emmHomeBusy || EmmTaskSeq_IsActive() || SysInit_IsActive()) {
        LOGD("[回零] @%d 拒绝样品盘单独回零：复合动作正在运行\n", seq);
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }

    if (!EmmSystemHome_TrayHome()) {
        sendWithCRC("ERR @%d %d", seq, EMM_HOME_FAILED);
        return false;
    }

    sendWithCRC("OK @%d EMM TRAY HOME DONE", seq);
    return true;
}

/**
 * @brief `SET EMM TRAY ZERO P=<n>`：设置样品盘零点标定时"触发后顺时针前进"的脉冲数。
 * @details **2026/10/26 新增**。命令格式与应答：
 *            - `SET EMM TRAY ZERO P=50` → `OK @<seq> EMM TRAY ZERO P=50`；
 *            - 缺 `P=` 或数值非法     → `ERR @<seq> 102`（`PARAM_ERR`）；
 *            - 超过 3200（一圈）      → `ERR @<seq> 102`；
 *            - NVS 写入失败           → `ERR @<seq> 102`（值保持旧值，不生效）。
 *
 *          **不受运行期互锁限制**（与 `SET EMM POINT` 同口径）：它只改"下次标定用"
 *          的参数，不影响正在跑的任何动作。整机回零运行中改 → 本次回零仍用旧值，
 *          下次生效（因为标定流程在进入时只读一次快照）。
 */
bool cmdSetEmmTrayZero(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();

    const char *p = (args != nullptr) ? strstr(args, "P=") : nullptr;
    if (p == nullptr) {
        Serial.printf("[回零] @%d SET EMM TRAY ZERO 参数错误：缺少 P=（应为 P=<0~3200>）\n", seq);
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    p += 2;

    char *end = nullptr;
    const unsigned long parsed = strtoul(p, &end, 0);
    /* 必须整段都是数字：`P=50abc` 这种静默截断要挡掉。 */
    if (end == p || (*end != '\0' && *end != ' ')) {
        Serial.printf("[回零] @%d SET EMM TRAY ZERO 参数错误：\"%s\" 不是合法数字\n", seq, p);
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (parsed > EMM_TRAY_ZERO_PULSES_MAX) {
        Serial.printf("[回零] @%d SET EMM TRAY ZERO 参数越界：%lu 超过上限 %lu（一圈）\n",
                      seq, parsed, (unsigned long)EMM_TRAY_ZERO_PULSES_MAX);
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    if (!EmmSystemHome_SetTrayZeroPulses((uint32_t)parsed)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    sendWithCRC("OK @%d EMM TRAY ZERO P=%lu", seq, parsed);
    return true;
}

/**
 * @brief `GET EMM TRAY ZERO`：查询样品盘零点标定时"触发后顺时针前进"的脉冲数。
 * @details 应答：`OK @<seq> EMM TRAY ZERO P=<n>`。
 */
bool cmdGetEmmTrayZero(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    const uint32_t pulses = EmmSystemHome_GetTrayZeroPulses();
    sendWithCRC("OK @%d EMM TRAY ZERO P=%lu", seq, (unsigned long)pulses);
    return true;
}

bool cmdSetEmmSystemHome(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    /* 与 L1 运动命令一致：整机断电时拒绝启动，避免上位机误以为已开始回零。 */
    if (cmd_power_off) {
        LOGD("[回零] @%d 拒绝启动：步进电机电源已关闭\n", seq);
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

#include "EmmLiftSwitchTest.h"

#include "Rs485Devices.h"
#include "common.h"
#include "crc16.h"

#include <string.h>   /* strncmp：EmmLiftSwitchTest_BlocksCommand() */

/* `vTaskDelay` / `xTaskCreatePinnedToCore` / `ulTaskNotifyTake` */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ==========================================================================
 * 参数（全部集中在这里，实机调试只改这一段）
 * ========================================================================== */

/**
 * @brief 3 号升降电机的接近开关引脚。
 * @details **2026/10/27 新增**。沿用 KWQ2 `stepper.h` 里 `Z1_ENDSTOP_PIN = 2` 的接线
 *          （板子上 Z1 那一路）；KWQ3 提纯成单升降后这个脚一直空着，本次接上开关。
 *
 *          ⚠️ **本路是「高电平 = 触发」**（2026/10/27 用户确认）——
 *          与样品盘那一路（`Y_ENDSTOP2_PIN` = GPIO6，**低电平触发**）**极性相反**。
 *          所以引脚配 `INPUT_PULLDOWN` 而不是 `INPUT_PULLUP`：
 *          下拉能让"开关没接 / 线断了"时读到**低电平 = 未触发**，是最安全的失效方向；
 *          若错配成上拉，悬空就会被读成"一直触发"，⓪ 防漂移会白降 8 s 然后报错。
 * @note 故意**不**放进 `Rs485Devices.h`：那一份是"正式硬件映射"，本模块只是试验，
 *       放进去会让人以为 3 号已经有常驻的开关保护（其实没有）。
 */
#define LIFT_TEST_PIN                   2U

/** @brief 被测试的电机地址：固定 3 号（`EMM_LIFT_MOTOR`）。 */
#define LIFT_TEST_MOTOR                 EMM_LIFT_MOTOR

/**
 * @brief 向上找开关的转速与加速度。
 * @details 沿用整机回零里 3 号碰撞回零的 `RPM=40 ACC=20`，这样测出来的停止距离
 *          与将来真启用时一致。
 *          触发后的过冲 ≈ `RPM × 3200 / 60 × (轮询周期 + 减速时间)`
 *          = 2133 脉冲/秒 × (10 ms + ~120 ms) ≈ **280 脉冲**
 *          （若 40000 脉冲对应 100 mm 行程，约 0.7 mm）。
 *          实机若觉得过冲偏大，先把 `LIFT_TEST_RPM` 降到 20 再试。
 */
#define LIFT_TEST_RPM                   40U
#define LIFT_TEST_ACC                   20U

/** @brief 3 号向上 = `DIR=0`（与整机回零 `HOME PARAM A=3 MODE=2 DIR=0` 一致）。 */
#define LIFT_TEST_UP_DIR                0U
/** @brief 3 号向下 = `DIR=1`（防漂移时用它退出触发区）。 */
#define LIFT_TEST_DOWN_DIR              1U

/** @brief 轮询 GPIO 电平的间隔（纯 `digitalRead`，不产生总线事务）。 */
#define LIFT_TEST_GPIO_POLL_MS          10U
/** @brief 等停稳时读驱动器状态的间隔（每轮一次总线事务，别降到 10 ms）。 */
#define LIFT_TEST_STOP_POLL_MS          100U
/** @brief 等停稳的最长时间（超时只提示、不判失败）。 */
#define LIFT_TEST_STOP_MAX_WAIT_MS      5000U

/**
 * @brief "向上找开关"的最长等待，单位 ms。
 * @details ⚠️ **不能沿用整机回零的 `EMM_HOME_TRIGGER_MAX_WAIT_MS`（15 s）**：
 *          3 号行程按 40000 脉冲算（点位最大 37000），`RPM=40` 时 2133 脉冲/秒，
 *          从最底端走回 0 需要**约 17.3 s**，15 s 必然误判超时。
 *          这里给 30 s，留约 70% 余量。
 */
#define LIFT_TEST_UP_MAX_WAIT_MS        30000U

/**
 * @brief "下降退出触发区"的最长等待，单位 ms。
 * @details 正常情况下开关刚触发，往下走几十~几百脉冲就释放（几十 ms）。
 *          给 8 s 是为了兜住"开关坏了常高"的极端情况 —— 那时 3 号会一直往下走，
 *          8 s 约 17000 脉冲，还不至于撞到行程底，然后判失败、停机、报错。
 */
#define LIFT_TEST_ESCAPE_MAX_WAIT_MS    8000U

/** @brief 抢共享 RS485 总线的最长等待（与其它 L2 流程同量级）。 */
#define LIFT_TEST_BUS_TIMEOUT_MS        500U
/** @brief 单条 EMM 命令等驱动器 ACK 的超时。 */
#define LIFT_TEST_CMD_TIMEOUT_MS        500U

/** @brief 后台任务优先级：与整机回零同级（两者互斥，不会同时跑）。 */
#define LIFT_TEST_TASK_PRIORITY         3
/** @brief 后台任务固定核心：与其它运动控制任务保持一致。 */
#define LIFT_TEST_TASK_CORE             1
/** @brief 后台任务栈大小（字节）。调用嵌套与整机回零同量级，4096 留足余量。 */
#define LIFT_TEST_TASK_STACK            4096U

/* ==========================================================================
 * 模块内部状态
 * ========================================================================== */

/** 后台任务句柄；非空表示任务已创建（首次下发命令时惰性创建）。 */
static TaskHandle_t s_liftTestTaskHandle = nullptr;

/**
 * @brief 运行占用标志。
 * @details 由 `cmdSetEmmLiftTest()` 在**唤醒任务之前**同步置位，任务结束时清零。
 *          用途有两个：① 挡住第二次 `SET EMM LIFT TEST`；
 *          ② 作为 `EmmLiftSwitchTest_BlocksCommand()` 的判据。
 */
static volatile bool s_liftTestBusy = false;

/** 本次测试对应的命令序号，失败时用它回 `ERR @<seq> 117`。 */
static volatile int s_liftTestSeq = 0;

/** @brief 本次运行的结果，决定任务结束后发哪一帧。 */
typedef enum {
    LIFT_TEST_RESULT_DONE = 0,   ///< 触发成功、3 号已停
    LIFT_TEST_RESULT_FAILED,     ///< 超时 / 退不出触发区 / 速度模式被拒
    LIFT_TEST_RESULT_STOPPED,    ///< 被 `SET POWER OFF` / 硬件急停打断
} LiftTestResult;

/** @brief 一次"起转→等电平→停止"的结果。 */
typedef enum {
    LIFT_TEST_SPIN_OK = 0,       ///< 达到目标电平
    LIFT_TEST_SPIN_TIMEOUT,      ///< 超时
    LIFT_TEST_SPIN_ABORTED,      ///< 被断电 / 急停打断
} LiftTestSpinResult;

/* ==========================================================================
 * 内部辅助
 * ========================================================================== */

/** @brief 统一日志前缀（`[3号测试]`，与 `[回零]` / `[任务序列]` 区分开）。 */
static void liftTestLog(const char *text)
{
    Serial.printf("[3号测试] %s\n", text);
}

/**
 * @brief 读接近开关**是否处于触发状态**。
 * @details 本路开关是**高电平触发**（`HIGH` = 触发，`LOW` = 释放）。
 *          把极性判断集中到这一个函数里，是为了避免"`== HIGH` / `== LOW`"
 *          散落在四处、改一次漏一处 —— 2026/10/27 就是因为按样品盘那一路
 *          （低电平触发）写反过一次。
 * @return true = 已触发；false = 未触发。
 */
static bool liftTestTriggered(void)
{
    return digitalRead(LIFT_TEST_PIN) == HIGH;
}

/**
 * @brief **中止判据**：测试过程中是否应当立刻停下来。
 *
 * @details **2026/10/27 下午新增，这是本次改造的核心**。
 *
 *          原来这条命令是同步阻塞的，最长 30 s 占住 `loop()` → `serialPoll()`
 *          → `dispatch()` 整条链路 —— 上位机在这段时间发 `SET POWER OFF`，
 *          字节只是躺在串口缓冲区里，**根本不会被解析**，`sysPowerOffCleanup()`
 *          （广播停机 + 断结构电源）也就不会执行，电机停不下来。
 *
 *          改成异步任务后，主循环空出来了，`SET POWER OFF` 能被立刻解析并执行
 *          `sysPowerOffCleanup()`；本函数则让后台任务也在 10 ms 内感知到同一次断电，
 *          干净地发停止、退出、不再报"成功"。
 *
 *          判据只用 `cmd_power_off` 一个：
 *            - `SET POWER OFF` → `sysPowerOffCleanup()` 第一句就置位它；
 *            - 硬件急停（GPIO1 结构电源断开）→ 同样走 `sysPowerOffCleanup()`；
 *            - 指令超时休眠（`checkPCAlive()`）→ 也走同一条路径。
 *          三条入口共用同一个标志，所以这里判一次就全覆盖了。
 *
 * @note 为什么不在断电时直接 `return`、跳过发停止：`sysPowerOffCleanup()` 里
 *       `cmd_power_off = true` 是**第一步**，广播停机在第二步，本任务可能抢在
 *       广播之前看到标志。这里多发一帧停止是**冗余但无害**的，反而更保险。
 */
static bool liftTestAbortRequested(void)
{
    return cmd_power_off;
}

/**
 * @brief 速度模式转动 3 号，直到接近开关达到指定状态（或超时 / 被中止）。
 * @details 只负责"起转 → 等电平 → 发停止"，**不做任何零点设置**。
 *          自己轮询 GPIO，不依赖任何监控任务 —— 所以即使将来加了常驻监控、
 *          或者监控被关掉，这条命令的行为都不变。
 * @param direction     转动方向（`0`=上升 / `1`=下降）
 * @param waitTriggered true = 等开关**触发**（高电平）；false = 等开关**释放**（低电平）
 * @param maxWaitMs     最长等待，单位 ms
 * @param label         子动作中文说明（日志用，必须指向静态存储）
 * @return `LIFT_TEST_SPIN_OK` / `LIFT_TEST_SPIN_TIMEOUT` / `LIFT_TEST_SPIN_ABORTED`。
 */
static LiftTestSpinResult liftTestSpinUntil(uint8_t direction, bool waitTriggered,
                                            uint32_t maxWaitMs, const char *label)
{
    Serial.printf("[3号测试]   └─ %s\n", label);

    /* ---- 1. 速度模式起转 ---- */
    if (!Rs485Bus_Take(LIFT_TEST_BUS_TIMEOUT_MS)) {
        liftTestLog("  └─ 失败：RS485 总线繁忙，速度模式命令未下发");
        return LIFT_TEST_SPIN_TIMEOUT;
    }
    EmmV5::Status status = gEmm.velocity(
        LIFT_TEST_MOTOR, direction, LIFT_TEST_RPM, LIFT_TEST_ACC,
        false, LIFT_TEST_CMD_TIMEOUT_MS);
    Rs485Bus_Give();

    if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
        /*
         * 驱动器可能要求先使能才允许进速度模式。补一次 enable 再重试一次，
         * 仍失败才判失败（现场若看到这条日志，说明该驱动器需要先使能）。
         */
        Serial.printf("[3号测试]   └─ 速度模式命令未被接受（状态=%s），补发使能后重试\n",
                      EmmV5::statusToString(status));
        if (Rs485Bus_Take(LIFT_TEST_BUS_TIMEOUT_MS)) {
            gEmm.enable(LIFT_TEST_MOTOR, true, false, LIFT_TEST_CMD_TIMEOUT_MS);
            status = gEmm.velocity(LIFT_TEST_MOTOR, direction,
                                   LIFT_TEST_RPM, LIFT_TEST_ACC,
                                   false, LIFT_TEST_CMD_TIMEOUT_MS);
            Rs485Bus_Give();
        }
        if (status != EmmV5::STATUS_OK && status != EmmV5::STATUS_DONE) {
            Serial.printf("[3号测试]   └─ 失败：速度模式命令未被接受，状态=%s\n",
                          EmmV5::statusToString(status));
            return LIFT_TEST_SPIN_TIMEOUT;
        }
    }
    Serial.printf("[3号测试]   └─ 速度模式已启动：A=%u DIR=%u RPM=%u ACC=%u，等待接近开关%s\n",
                  (unsigned)LIFT_TEST_MOTOR, (unsigned)direction,
                  (unsigned)LIFT_TEST_RPM, (unsigned)LIFT_TEST_ACC,
                  waitTriggered ? "触发（高电平）" : "释放（低电平）");

    /* ---- 2. 轮询接近开关（每轮先看断电） ---- */
    const uint32_t startedAt = millis();
    bool reached = false;
    bool aborted = false;
    for (;;) {
        if (liftTestAbortRequested()) {
            aborted = true;
            break;
        }
        if (liftTestTriggered() == waitTriggered) {
            reached = true;
            break;
        }
        if ((millis() - startedAt) >= maxWaitMs) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(LIFT_TEST_GPIO_POLL_MS));
    }

    /* ---- 3. 无论成败都先发停止，避免速度模式一直转下去 ---- */
    if (Rs485Bus_Take(LIFT_TEST_BUS_TIMEOUT_MS)) {
        const EmmV5::Status stopStatus =
            gEmm.stop(LIFT_TEST_MOTOR, false, LIFT_TEST_CMD_TIMEOUT_MS);
        Rs485Bus_Give();
        Serial.printf("[3号测试]   └─ 已发停止命令，状态=%s\n",
                      EmmV5::statusToString(stopStatus));
    } else {
        liftTestLog("  └─ 警告：RS485 总线繁忙，停止命令未下发");
    }

    if (aborted) {
        liftTestLog("  └─ 检测到断电 / 急停，已停止 3 号并放弃本次测试");
        return LIFT_TEST_SPIN_ABORTED;
    }
    if (!reached) {
        Serial.printf("[3号测试]   └─ 失败：等待接近开关%s超时（%lu ms）\n",
                      waitTriggered ? "触发" : "释放", (unsigned long)maxWaitMs);
        return LIFT_TEST_SPIN_TIMEOUT;
    }
    return LIFT_TEST_SPIN_OK;
}

/**
 * @brief 等 3 号停稳（速度模式发"立即停止"之后）。
 * @details 读 `0x43` 系统状态里的实时转速 `realtimeRpm`，**连续两次为 0** 才认为
 *          停稳 —— 单次为 0 可能是刚发停止、驱动器状态还没更新。
 *          **超时不判失败**：这一步只是"确认停稳"，不是"必须成功"。
 * @param stage 阶段名（日志用，必须指向静态存储）
 * @return true 停稳（或超时兜底）；false 被断电 / 急停打断。
 */
static bool liftTestWaitStopped(const char *stage)
{
    const uint32_t startedAt = millis();
    uint8_t stillCount = 0;

    for (;;) {
        if (liftTestAbortRequested()) {
            liftTestLog("  └─ 检测到断电 / 急停，不再等停稳");
            return false;
        }
        if (Rs485Bus_Take(LIFT_TEST_BUS_TIMEOUT_MS)) {
            EmmV5::SystemState state{};
            const EmmV5::Status status = gEmm.readSystemState(
                LIFT_TEST_MOTOR, state, LIFT_TEST_CMD_TIMEOUT_MS);
            Rs485Bus_Give();
            if (status == EmmV5::STATUS_OK) {
                if (state.realtimeRpm == 0) {
                    if (++stillCount >= 2U) {
                        Serial.printf("[3号测试]   └─ 3 号%s已停稳（转速=0，位置=%ld）\n",
                                      stage, (long)state.realtimePosition);
                        return true;
                    }
                } else {
                    stillCount = 0;
                }
            }
        }
        if ((millis() - startedAt) >= LIFT_TEST_STOP_MAX_WAIT_MS) {
            Serial.printf("[3号测试]   └─ 提示：%s停稳确认超时（%lu ms），继续\n",
                          stage, (unsigned long)LIFT_TEST_STOP_MAX_WAIT_MS);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(LIFT_TEST_STOP_POLL_MS));
    }
}

/**
 * @brief 读一次 3 号的当前位置（`0x43.realtimePosition`）。
 * @param pos 输出：位置计数；读失败时保持不变。
 * @return true 读到；false 总线忙或驱动器无应答。
 */
static bool liftTestReadPosition(int32_t &pos)
{
    if (!Rs485Bus_Take(LIFT_TEST_BUS_TIMEOUT_MS)) {
        return false;
    }
    EmmV5::SystemState state{};
    const EmmV5::Status status =
        gEmm.readSystemState(LIFT_TEST_MOTOR, state, LIFT_TEST_CMD_TIMEOUT_MS);
    Rs485Bus_Give();
    if (status != EmmV5::STATUS_OK) {
        return false;
    }
    pos = state.realtimePosition;
    return true;
}

/* ==========================================================================
 * 测试主体（在后台任务里跑）
 * ========================================================================== */

/**
 * @brief 跑完整套测试流程。
 * @details 每一步之后都检查中止判据，一旦断电立刻返回 `STOPPED`，
 *          **不会**继续往下走、也不会报成功。
 * @return 见 `LiftTestResult`。
 */
static LiftTestResult liftTestRunSequence(void)
{
    /*
     * 极端时序：命令刚被接受、后台任务还没来得及跑，上位机就发了 `SET POWER OFF`。
     * 这里先看一眼，避免在已断电的驱动器上白发一轮速度模式命令。
     */
    if (liftTestAbortRequested()) {
        liftTestLog("命令已接受但执行前就断电了，放弃本次测试（3 号未动作）");
        return LIFT_TEST_RESULT_STOPPED;
    }

    Serial.println("[3号测试] ========== 开始：3 号上升直到接近开关触发 ==========");
    Serial.println("[3号测试] 只验证「触发即停」，**不设零点**（3 号零点与 5 个点位保持不变）");
    /*
     * ⚠️ 与"扎偏保护"的交互（本模块**不去撤防**，只提醒）：
     *   3 号的相电流监控（`EmmLiftCurrentMonitor`）由上位的
     *   `SET EMM LIFT MONITOR ARM=<0|1>` 布防，**默认未布防**。它一旦布防，
     *   检测到相电流持续超阈值就会"把 3 号再往上抬到安全高度再停"。
     *   触发点离机械顶死点很近，停车后的过冲可能让电流短时冲高 —— 若此时正好
     *   处于布防状态，就会出现"测试跑完 3 号反而又被抬上去"的怪现象。
     *   本命令不改那个标志（那是上位机的职责，擅自动它会让真实扎偏失去保护），
     *   只在串口把风险讲清楚。
     */
    Serial.println("[3号测试] 提示：若已下发过 SET EMM LIFT MONITOR ARM=1，"
                   "建议测试前先 ARM=0，避免扎偏保护在触发点附近误动作把 3 号再往上抬");

    /*
     * 先把引脚配成**下拉**输入：本路开关是高电平触发，下拉能保证"没接 / 断线"时
     * 读到低电平（= 未触发），是最安全的失效方向。本模块不依赖任何监控任务，
     * 必须自己配一次。
     */
    pinMode(LIFT_TEST_PIN, INPUT_PULLDOWN);
    /* 上电/换线后立刻读一次，日志里留个底，便于判断开关是不是常高。 */
    Serial.printf("[3号测试] GPIO%u 初始电平=%s（本路为高电平触发）\n",
                  (unsigned)LIFT_TEST_PIN,
                  liftTestTriggered() ? "高（已触发）" : "低（未触发）");

    /*
     * ⓪ 防漂移：若开关此刻已经是**高电平**（上一次测试就停在触发点、或者人工把 3 号
     *    开到了顶），先下降退出触发区再重新向上找。
     *    没有这一步，下面的"等触发"会**立刻**判定成功，3 号一步都不走，
     *    测出来的停止距离就是 0，等于什么都没验证。
     *    （这一条只在"测停止效果"的语境下是必要的；将来若真把触发点设成零点，
     *      它同时也是防零点漂移的前提。）
     */
    if (liftTestTriggered()) {
        liftTestLog("接近开关已处于触发状态，先下降退出触发区");
        const LiftTestSpinResult r =
            liftTestSpinUntil(LIFT_TEST_DOWN_DIR, /*waitTriggered=*/false,
                              LIFT_TEST_ESCAPE_MAX_WAIT_MS,
                              "下降至接近开关释放");
        if (r == LIFT_TEST_SPIN_ABORTED) {
            return LIFT_TEST_RESULT_STOPPED;
        }
        if (r == LIFT_TEST_SPIN_TIMEOUT) {
            liftTestLog("失败：8 s 内退不出触发区，放弃本次测试（开关可能常高）");
            return LIFT_TEST_RESULT_FAILED;
        }
        if (!liftTestWaitStopped("退出触发区后")) {
            return LIFT_TEST_RESULT_STOPPED;
        }
    }

    /* 记一下起步位置，测完可以算出实际走了多少脉冲。 */
    int32_t startPos = 0;
    const bool startPosOk = liftTestReadPosition(startPos);

    /* ① 上升找开关，触发立即停 */
    const LiftTestSpinResult upResult =
        liftTestSpinUntil(LIFT_TEST_UP_DIR, /*waitTriggered=*/true,
                          LIFT_TEST_UP_MAX_WAIT_MS,
                          "上升至接近开关触发");
    if (upResult == LIFT_TEST_SPIN_ABORTED) {
        return LIFT_TEST_RESULT_STOPPED;
    }
    if (upResult == LIFT_TEST_SPIN_TIMEOUT) {
        liftTestLog("失败：3 号上升未找到接近开关触发（检查开关接线 / 极性 / 是否已到顶）");
        return LIFT_TEST_RESULT_FAILED;
    }

    /* ② 等停稳 */
    if (!liftTestWaitStopped("触发后")) {
        return LIFT_TEST_RESULT_STOPPED;
    }

    /* ③ 打印结果：触发点读数 + 本次走了多少脉冲 */
    int32_t stopPos = 0;
    if (liftTestReadPosition(stopPos)) {
        Serial.printf("[3号测试] 触发点位置读数 = %ld 计数（相对当前零点，**本次不改零点**）\n",
                      (long)stopPos);
        if (startPosOk) {
            Serial.printf("[3号测试] 本次实际行程 = %ld 脉冲（起步 %ld → 停止 %ld）\n",
                          (long)(stopPos - startPos), (long)startPos, (long)stopPos);
        }
    } else {
        liftTestLog("提示：触发后读不到 3 号位置（不影响本次结论）");
    }

    Serial.println("[3号测试] ========== 结束：接近开关触发，3 号已停止 ==========");
    /*
     * 这里必须提醒一句：3 号此刻正**压在开关上**（开关仍是高电平）。
     * 本模块没有常驻监控，所以不会反复发停止帧；但下一次跑本命令时，⓪ 那一步
     * 会先把它降下来。若要接着跑任务，记得先用 SET EMM MOVE 把 3 号开回工作高度。
     */
    Serial.println("[3号测试] ⚠️ 3 号现停在触发点、开关仍被压住；"
                   "要跑任务请先手动把 3 号降回工作高度（零点未变，可直接用 SET EMM MOVE）");
    return LIFT_TEST_RESULT_DONE;
}

/**
 * @brief 测试命令的常驻后台任务。
 * @details 默认阻塞等待通知；被 `cmdSetEmmLiftTest()` 唤醒后跑完整套流程，
 *          按结果发一帧结局，然后清 busy 回到阻塞状态。任务不主动退出。
 *
 *          **为什么要独立任务**：命令处理是同步跑在 `loop()` → `serialPoll()`
 *          → `dispatch()` 这条链路上的，跑 30 s 就把整条链路堵死，`SET POWER OFF`
 *          读不进来（见 `liftTestAbortRequested()` 的注释）。
 */
static void liftTestTask(void *argument)
{
    (void)argument;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 通知被其它路径误发时直接忽略。 */
        if (!s_liftTestBusy) {
            continue;
        }

        const int seq = s_liftTestSeq;
        const LiftTestResult result = liftTestRunSequence();

        /*
         * 结局帧：
         *   成功   → `EVT 390 LIFT TEST DONE`
         *   被打断 → `EVT 392 LIFT TEST STOPPED`（断电 / 急停）
         *   失败   → `ERR @<seq> 117`
         * 与整机回零同口径：只有 `ERR` 才算"板子报错"，`EVT` 是过程/结果通知。
         */
        switch (result) {
        case LIFT_TEST_RESULT_DONE:
            sendWithCRC("EVT %d LIFT TEST DONE", (int)LIFT_TEST_DONE);
            break;
        case LIFT_TEST_RESULT_STOPPED:
            sendWithCRC("EVT %d LIFT TEST STOPPED", (int)LIFT_TEST_STOPPED);
            break;
        default:
            sendWithCRC("ERR @%d %d", seq, EMM_HOME_FAILED);
            break;
        }

        s_liftTestBusy = false;
        s_liftTestSeq = 0;
        LOGD("[3号测试] 后台任务回到空闲，等待下一次下发\n");
    }
}

/**
 * @brief 惰性创建后台任务（首次下发命令时调用）。
 * @return true 任务可用；false 创建失败。
 */
static bool liftTestEnsureTask(void)
{
    if (s_liftTestTaskHandle != nullptr) {
        return true;
    }

    const BaseType_t result = xTaskCreatePinnedToCore(
        liftTestTask,
        "LiftSwitchTest",
        LIFT_TEST_TASK_STACK,
        nullptr,
        LIFT_TEST_TASK_PRIORITY,
        &s_liftTestTaskHandle,
        LIFT_TEST_TASK_CORE);

    if (result != pdPASS) {
        s_liftTestTaskHandle = nullptr;
        Serial.println("[3号测试] 启动失败：无法创建后台任务");
        return false;
    }

    Serial.println("[3号测试] 后台任务已创建");
    return true;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

bool cmdSetEmmLiftTest(const char *args, int seq)
{
    (void)args;   /* 本命令无参数 */
    cmd_on_line = true;
    lasCmdTime = millis();

    /* 与 L1 运动命令一致：结构电源关闭时拒绝，避免"以为在动其实电机没电"。 */
    if (cmd_power_off) {
        Serial.printf("[3号测试] @%d 拒绝：步进电机电源已关闭\n", seq);
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }

    /*
     * 互斥：本命令正在跑时再下发一次，直接拒绝。
     * 正常情况下 `dispatch()` 会先用 `EmmLiftSwitchTest_BlocksCommand()` 拦掉
     * （它也拦 `SET EMM LIFT TEST` 自己），这里是兜底。
     */
    if (s_liftTestBusy) {
        Serial.printf("[3号测试] @%d 拒绝：上一次测试仍在运行\n", seq);
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }

    if (!liftTestEnsureTask()) {
        sendWithCRC("ERR @%d %d", seq, EMM_HOME_FAILED);
        return false;
    }

    /*
     * 置位 → **先应答** → 再唤醒。
     *
     * ⚠️ 顺序不能反：后台任务优先级 3 高于主循环，`xTaskNotifyGive()` 之后它
     *    **可能立刻抢占**并开始往串口打印（`[3号测试] ===== 开始`，甚至抢先发出
     *    `EVT 390`），于是上位机看到的是"结果帧排在 `OK @n` 之前"，会认为命令
     *    没被受理却已经出结果。先发 ACK 就能保证帧序恒定：`OK` 一定在最前。
     *
     * 先应答也不会漏跑：通知是计数型的，任务晚一点被唤醒也照样收得到。
     */
    s_liftTestSeq = seq;
    s_liftTestBusy = true;
    sendWithCRC("OK @%d EMM LIFT TEST STARTED", seq);
    xTaskNotifyGive(s_liftTestTaskHandle);
    return true;
}

bool EmmLiftSwitchTest_IsActive(void)
{
    return s_liftTestBusy;
}

bool EmmLiftSwitchTest_BlocksCommand(const char *commandName)
{
    if (!s_liftTestBusy || commandName == nullptr) {
        return false;
    }

    /*
     * 测试运行期间独占 3 号电机：拦截所有 `SET EMM *`（含 `SET EMM STOP` ——
     * 要紧急停机请用 `SET POWER OFF`，它会走 `sysPowerOffCleanup()` 广播停机 +
     * 断结构电源，同时本模块的任务也会在 10 ms 内看到 `cmd_power_off` 并退出）。
     * `GET EMM *` 查询命令放行，便于现场诊断。
     */
    return strncmp(commandName, "SET EMM", 7) == 0;
}

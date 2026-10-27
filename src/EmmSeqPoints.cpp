#include "EmmSeqPoints.h"

#include <Preferences.h>
#include <string.h>
#include <strings.h>

/** 命令层用到的公共符号（`cmd_on_line` / `lasCmdTime` / `sendWithCRC` / 错误码）。 */
#include "common.h"

/* ==========================================================================
 * 内部常量表
 * --------------------------------------------------------------------------
 * 下面 5 张表都**按下标一一对应**，长度都是 `EMM_SEQ_POINT_COUNT`(9)。
 * 加/删点位时这 5 张表必须同时改，否则会串位（NVS 值写到别的点位上）。
 * ========================================================================== */

/** NVS 命名空间（与 CH9120 的 "ch9120" 等互不干扰）。 */
static const char *const kNvsNamespace = "emmseq";

/**
 * @brief 每个点位在 NVS 里的 key。
 * @note key 长度上限 15 字节（NVS 限制），这里都在 12 字节以内。
 *       **不要改已有 key 名**，否则老设备升级后读不到旧值。
 */
static const char *const kNvsKey[EMM_SEQ_POINT_COUNT] = {
    "p_sample",     /* 2 号：取样位 */
    "p_detect",     /* 2 号：检测位 */
    "p_disc1",      /* 2 号：回收位第一段 */
    "p_disc2",      /* 2 号：回收位第二段 */
    "p_lift_s1",    /* 3 号：取样高度 1（2026/10/20 新增） */
    "p_lift_s2",    /* 3 号：取样高度 2 */
    "p_lift_det",   /* 3 号：检测高度 */
    "p_lift_d2",    /* 3 号：回收位第二段 */
    "p_lift_d3"     /* 3 号：回收位第三段 */
};

/** 上位机命令里用的短名（`GET EMM POINT` 回显、日志都用它）。 */
static const char *const kShortName[EMM_SEQ_POINT_COUNT] = {
    "SAMPLE",
    "DETECT",
    "DISC1",
    "DISC2",
    "LIFT_S1",
    "LIFT_S2",
    "LIFT_DET",
    "LIFT_D2",
    "LIFT_D3"
};

/** 出厂默认值表（直接引用头文件里的宏，保证"只有一个数值来源"）。 */
static const uint32_t kDefault[EMM_SEQ_POINT_COUNT] = {
    EMM_SEQ_ROT_POS_SAMPLE,
    EMM_SEQ_ROT_POS_DETECT,
    EMM_SEQ_ROT_POS_DISCARD_1,
    EMM_SEQ_ROT_POS_DISCARD_2,
    EMM_SEQ_LIFT_POS_SAMPLE_1,
    EMM_SEQ_LIFT_POS_SAMPLE_2,
    EMM_SEQ_LIFT_POS_DETECT,
    EMM_SEQ_LIFT_POS_DISCARD_2,
    EMM_SEQ_LIFT_POS_DISCARD_3
};

/**
 * @brief 每个点位允许的最大脉冲数（**按轴分开**）。
 * @details 2 号是单圈机构（3200 脉冲 = 一圈），3 号是丝杆长行程（当前用到 37000）。
 *          用一个统一上限是不行的：取 3200 则 3 号根本设不进去，取 40000 则
 *          2 号能被设成转十几圈 —— 所以必须逐点带上限。
 */
static const uint32_t kMax[EMM_SEQ_POINT_COUNT] = {
    EMM_SEQ_POINT_PULSE_MAX,    /* 2 号：取样位 */
    EMM_SEQ_POINT_PULSE_MAX,    /* 2 号：检测位 */
    EMM_SEQ_POINT_PULSE_MAX,    /* 2 号：回收位第一段 */
    EMM_SEQ_POINT_PULSE_MAX,    /* 2 号：回收位第二段 */
    EMM_SEQ_LIFT_PULSE_MAX,     /* 3 号：取样高度 1 */
    EMM_SEQ_LIFT_PULSE_MAX,     /* 3 号：取样高度 2 */
    EMM_SEQ_LIFT_PULSE_MAX,     /* 3 号：检测高度 */
    EMM_SEQ_LIFT_PULSE_MAX,     /* 3 号：回收位第二段 */
    EMM_SEQ_LIFT_PULSE_MAX      /* 3 号：回收位第三段 */
};

/**
 * @brief 运行期当前值。
 * @note `volatile` 只防编译器把它缓存到寄存器：本模块的值会被序列任务（Core 1）
 *       和命令任务同时读写。`uint32_t` 在 ESP32 上是 32 位对齐的单次读写，本身原子，
 *       因此不需要临界区。
 */
static volatile uint32_t s_value[EMM_SEQ_POINT_COUNT] = {
    EMM_SEQ_ROT_POS_SAMPLE,
    EMM_SEQ_ROT_POS_DETECT,
    EMM_SEQ_ROT_POS_DISCARD_1,
    EMM_SEQ_ROT_POS_DISCARD_2,
    EMM_SEQ_LIFT_POS_SAMPLE_1,
    EMM_SEQ_LIFT_POS_SAMPLE_2,
    EMM_SEQ_LIFT_POS_DETECT,
    EMM_SEQ_LIFT_POS_DISCARD_2,
    EMM_SEQ_LIFT_POS_DISCARD_3
};

/** 是否已成功从 NVS 载入过（懒初始化标志）。 */
static bool s_inited = false;

/**
 * @brief 是否已经**尝试**过初始化（不管成败）。
 * @details 与 `s_inited` 分开：`s_inited` 表示"成功"，本标志只用于失败后的重试节流。
 */
static bool s_initAttempted = false;

/** 上次尝试初始化的时刻（`millis()`），用于失败后节流。 */
static uint32_t s_lastInitAttemptMs = 0U;

/**
 * @brief NVS 打开失败后，两次重试之间的最小间隔（ms）。
 * @details `EmmSeqPoints_Get()` 每步都会懒初始化一次。若 NVS 真的坏了又没有节流，
 *          序列每一步都会去开一次 NVS 并打一行日志 —— 既刷屏又拖慢步进。
 */
static const uint32_t kInitRetryMs = 2000U;

/* ==========================================================================
 * 内部辅助
 * ========================================================================== */

static bool isValidId(EmmSeqPointId id)
{
    return (int)id >= 0 && (int)id < (int)EMM_SEQ_POINT_COUNT;
}

/**
 * @brief 打印全部点位当前值（初始化完成、恢复默认时用）。
 * @details 分两行打印（2 号一行、3 号一行）：一行 9 个字段会超过 128 字节的
 *          串口行宽习惯，日志也不好看。
 */
static void printAll(const char *tag)
{
    /*
     * **2026/10/21 起降为详细日志（LOGD）**：这 3 行只在 `SET LOG DETAIL=1`
     * 时输出。开机与 `SEQ START` 都会调用本函数，如果始终打印，光点位快照
     * 就占 3 行，与"只保留关键消息"的目标冲突。
     * 需要看当前点位时用 `GET EMM POINT` / `GET EMM POINT LIFT`，那两条是
     * 结构化应答，比日志更好用。
     */
    LOGD("[序列点位] %s\n", tag);
    LOGD("[序列点位]   2号(旋转)：SAMPLE=%lu DETECT=%lu DISC1=%lu DISC2=%lu\n",
         (unsigned long)s_value[EMM_SEQ_POINT_SAMPLE],
         (unsigned long)s_value[EMM_SEQ_POINT_DETECT],
         (unsigned long)s_value[EMM_SEQ_POINT_DISCARD_1],
         (unsigned long)s_value[EMM_SEQ_POINT_DISCARD_2]);
    LOGD("[序列点位]   3号(升降)：LIFT_S1=%lu LIFT_S2=%lu LIFT_DET=%lu "
         "LIFT_D2=%lu LIFT_D3=%lu\n",
         (unsigned long)s_value[EMM_SEQ_POINT_LIFT_SAMPLE_1],
         (unsigned long)s_value[EMM_SEQ_POINT_LIFT_SAMPLE_2],
         (unsigned long)s_value[EMM_SEQ_POINT_LIFT_DETECT],
         (unsigned long)s_value[EMM_SEQ_POINT_LIFT_DISCARD_2],
         (unsigned long)s_value[EMM_SEQ_POINT_LIFT_DISCARD_3]);
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

bool EmmSeqPoints_Init(void)
{
    if (s_inited) {
        return true;
    }

    /*
     * 失败后 `kInitRetryMs` 内不重复尝试（见 `kInitRetryMs` 的说明）。
     * 注意 `millis()` 在启动早期可能是 0，所以用独立标志 `s_initAttempted`
     * 而不是"时间戳 != 0"来判断"是否试过"。
     */
    const uint32_t now = millis();
    if (s_initAttempted &&
        (uint32_t)(now - s_lastInitAttemptMs) < kInitRetryMs) {
        return false;
    }
    s_initAttempted = true;
    s_lastInitAttemptMs = now;

    /* 先全部置为出厂默认值，NVS 里有合法值再逐个覆盖。 */
    for (int i = 0; i < (int)EMM_SEQ_POINT_COUNT; i++) {
        s_value[i] = kDefault[i];
    }

    Preferences prefs;
    /*
     * **用可写方式打开**（`readOnly=false`，不是 `true`）—— 目的是区分两种情况：
     *   - 命名空间还不存在（**首次上电，从没存过点位**）：会自动创建，`begin()`
     *     返回 true，下面 `isKey()` 全为 false → 用默认值，**算初始化成功**；
     *   - NVS 真的打不开（未初始化 / 空间满 / 损坏）：`begin()` 返回 false →
     *     **保持 `s_inited = false`**，下次 Get/Set 时自动重试。
     *
     * 旧实现用的是 `readOnly=true`，上面两种情况都返回 false，只能一律当成功，
     * 于是"NVS 真的坏了"也被当成"首次上电"：**永久停留在宏默认值、从不重试**，
     * 而 `SET EMM POINT` 明明回了 OK、NVS 里也有值，序列却一直用默认值跑 ——
     * 这是"点位设置不落实"最隐蔽的一条路径（2026/10/20 修）。
     */
    if (!prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
        Serial.println("[序列点位] NVS 打开失败：本次仍用出厂默认值，稍后自动重试");
        return false;
    }

    int loaded = 0;
    for (int i = 0; i < (int)EMM_SEQ_POINT_COUNT; i++) {
        if (!prefs.isKey(kNvsKey[i])) {
            continue;
        }
        const uint32_t stored = prefs.getUInt(kNvsKey[i], kDefault[i]);
        if (stored > kMax[i]) {
            /* 例如降级固件后 MAX 变小，或 NVS 被写脏 —— 宁可退回默认值并告警。 */
            Serial.printf("[序列点位] NVS 里 %s=%lu 超出范围（<= %lu），改用默认 %lu\n",
                          kShortName[i], (unsigned long)stored,
                          (unsigned long)kMax[i],
                          (unsigned long)kDefault[i]);
            continue;
        }
        s_value[i] = stored;
        loaded++;
    }
    prefs.end();

    s_inited = true;
    if (loaded == 0) {
        printAll("已就绪（无 NVS 记录，使用出厂默认值）");
    } else {
        printAll("已从 NVS 载入");
    }
    return true;
}

uint32_t EmmSeqPoints_Get(EmmSeqPointId id)
{
    if (!isValidId(id)) {
        return 0U;
    }
    if (!s_inited) {
        EmmSeqPoints_Init();    /* 懒初始化：调用方忘了显式 Init 也不会拿到脏值 */
    }
    return s_value[id];
}

uint32_t EmmSeqPoints_Default(EmmSeqPointId id)
{
    if (!isValidId(id)) {
        return 0U;
    }
    return kDefault[id];
}

uint32_t EmmSeqPoints_Max(EmmSeqPointId id)
{
    if (!isValidId(id)) {
        return 0U;
    }
    return kMax[id];
}

bool EmmSeqPoints_Set(EmmSeqPointId id, uint32_t pulse)
{
    if (!isValidId(id)) {
        return false;
    }
    if (pulse > kMax[id]) {
        return false;
    }
    if (!s_inited) {
        EmmSeqPoints_Init();
    }

    const uint32_t old = s_value[id];

    /*
     * **先落盘、再生效**：NVS 写失败时完全无副作用（变量保持旧值），
     * 这样命令层回 `ERR` 的语义才和实际状态一致。
     * 序列运行中已被命令层拒绝，因此不存在"落盘期间被读到旧值"的竞争。
     */
    Preferences prefs;
    if (!prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
        Serial.printf("[序列点位] %s：NVS 打开失败，未修改（仍为 %lu）\n",
                      kShortName[id], (unsigned long)old);
        return false;
    }
    const size_t written = prefs.putUInt(kNvsKey[id], pulse);
    prefs.end();

    if (written == 0U) {
        Serial.printf("[序列点位] %s：NVS 写入失败，未修改（仍为 %lu）\n",
                      kShortName[id], (unsigned long)old);
        return false;
    }

    s_value[id] = pulse;
    Serial.printf("[序列点位] %s：%lu → %lu（已保存）\n",
                  kShortName[id], (unsigned long)old, (unsigned long)pulse);
    return true;
}

bool EmmSeqPoints_ResetAll(void)
{
    if (!s_inited) {
        EmmSeqPoints_Init();
    }

    for (int i = 0; i < (int)EMM_SEQ_POINT_COUNT; i++) {
        s_value[i] = kDefault[i];
    }

    Preferences prefs;
    if (!prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
        Serial.println("[序列点位] NVS 打开失败：已恢复默认值，但 NVS 里的旧记录未清除");
        printAll("已恢复默认值（NVS 未清）");
        return false;
    }
    for (int i = 0; i < (int)EMM_SEQ_POINT_COUNT; i++) {
        prefs.remove(kNvsKey[i]);
    }
    prefs.end();

    printAll("已恢复出厂默认值并清除 NVS");
    return true;
}

void EmmSeqPoints_PrintAll(const char *tag)
{
    if (!s_inited) {
        EmmSeqPoints_Init();
    }
    printAll((tag != nullptr) ? tag : "当前点位");
}

const char *EmmSeqPoints_Name(EmmSeqPointId id)
{
    if (!isValidId(id)) {
        return "?";
    }
    return kShortName[id];
}

bool EmmSeqPoints_ParseName(const char *name, EmmSeqPointId *out)
{
    if (name == nullptr || out == nullptr) {
        return false;
    }

    /* 主名：SAMPLE / DETECT / DISC1 / DISC2 / LIFT_S1 / …（大小写不敏感） */
    for (int i = 0; i < (int)EMM_SEQ_POINT_COUNT; i++) {
        if (strcasecmp(name, kShortName[i]) == 0) {
            *out = (EmmSeqPointId)i;
            return true;
        }
    }

    /* ---- 2 号别名：方便上位机按中文语义拼命令，容忍几种常见写法 ---- */
    if (strcasecmp(name, "DISCARD1") == 0 ||
        strcasecmp(name, "DISCARD_1") == 0 ||
        strcasecmp(name, "RECYCLE1") == 0 ||
        strcasecmp(name, "RECYCLE_1") == 0) {
        *out = EMM_SEQ_POINT_DISCARD_1;
        return true;
    }
    if (strcasecmp(name, "DISCARD2") == 0 ||
        strcasecmp(name, "DISCARD_2") == 0 ||
        strcasecmp(name, "RECYCLE2") == 0 ||
        strcasecmp(name, "RECYCLE_2") == 0) {
        *out = EMM_SEQ_POINT_DISCARD_2;
        return true;
    }

    /* ---- 3 号别名：全称写法（LIFT_SAMPLE_1 / LIFT_DISCARD_2 …） ---- */
    if (strcasecmp(name, "LIFT_SAMPLE1") == 0 ||
        strcasecmp(name, "LIFT_SAMPLE_1") == 0 ||
        strcasecmp(name, "LIFT_HEIGHT1") == 0) {
        *out = EMM_SEQ_POINT_LIFT_SAMPLE_1;
        return true;
    }
    if (strcasecmp(name, "LIFT_SAMPLE2") == 0 ||
        strcasecmp(name, "LIFT_SAMPLE_2") == 0 ||
        strcasecmp(name, "LIFT_HEIGHT2") == 0) {
        *out = EMM_SEQ_POINT_LIFT_SAMPLE_2;
        return true;
    }
    if (strcasecmp(name, "LIFT_DETECT") == 0 ||
        strcasecmp(name, "LIFT_DET") == 0) {
        *out = EMM_SEQ_POINT_LIFT_DETECT;
        return true;
    }
    if (strcasecmp(name, "LIFT_DISC2") == 0 ||
        strcasecmp(name, "LIFT_DISCARD_2") == 0 ||
        strcasecmp(name, "LIFT_DISCARD2") == 0) {
        *out = EMM_SEQ_POINT_LIFT_DISCARD_2;
        return true;
    }
    if (strcasecmp(name, "LIFT_DISC3") == 0 ||
        strcasecmp(name, "LIFT_DISCARD_3") == 0 ||
        strcasecmp(name, "LIFT_DISCARD3") == 0) {
        *out = EMM_SEQ_POINT_LIFT_DISCARD_3;
        return true;
    }

    return false;
}

/* ==========================================================================
 * L3 命令处理
 * ========================================================================== */

bool cmdSetEmmPoint(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();

    if (args == nullptr || args[0] == '\0') {
        Serial.println("[序列点位] 参数为空：用法 SET EMM POINT SAMPLE=985 "
                       "或 SET EMM POINT LIFT_S1=32000");
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    /*
     * 解析 "<NAME>=<VALUE>"。
     *   - `%15[^=]` 读名字（最多 15 字符，配合 char[16] 不会溢出）；
     *   - `%lu` 读数值（ESP32 上 unsigned long 为 32 位，正好装 uint32_t）；
     *   - `%n` 记录已消费的字符数，用来**拒绝尾随垃圾**（2026/10/20 新增）。
     * `args` 已由 CommandDispatcher::dispatch() 跳过前导空格，这里不必再 trim。
     *
     * **为什么必须查尾随字符**：`%lu` 遇到小数点会停下并返回 2，
     * 于是 `SAMPLE=985.5` 会被**静默截断成 985**、`SAMPLE=985abc` 也会被当成 985。
     * 现场看到 `OK` 就以为设成了 985.5，实际差半格，极难排查 —— 一律报 `ERR 102`。
     */
    char name[16] = {0};
    unsigned long value = 0;
    int consumed = 0;
    if (sscanf(args, "%15[^=]=%lu%n", name, &value, &consumed) != 2) {
        Serial.printf("[序列点位] 参数格式错误：%s（用法 SAMPLE=985）\n", args);
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    /* 数值之后只允许空白，其它字符（如 `.5`、`abc`）一律判格式错。 */
    {
        const char *tail = args + consumed;
        while (*tail == ' ' || *tail == '\t') {
            tail++;
        }
        if (*tail != '\0') {
            Serial.printf("[序列点位] 参数尾部有非法字符：%s（只接受整数脉冲）\n", tail);
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
    }

    /* ---- 特例：SET EMM POINT RESET=1 → 全部恢复出厂默认值并清除 NVS ---- */
    if (strcasecmp(name, "RESET") == 0) {
        /*
         * **2026/10/21 修订**：单点设置已允许在复合动作运行中执行（见下方），
         * 但 `RESET=1` 会**一次性改掉 9 个点位**，运行中放开的爆炸半径明显更大，
         * 用户选定"仍拒绝"。
         *
         * 判据从"只看任务序列"扩展成"**任何复合动作**"：因为
         * `EmmSystemHome_BlocksCommand()` 已对 `SET EMM POINT` 整体放行，
         * 若这里仍只判 `EmmTaskSeq_IsActive()`，整机回零中发 `RESET=1`
         * 就会一路走到 `EmmSeqPoints_ResetAll()` —— 绕过全部拦截。
         */
        if (EmmTaskSeq_IsActive() || EmmSystemHome_IsActive()) {
            Serial.println("[序列点位] 拒绝恢复默认值：复合动作（任务序列/整机回零）正在运行");
            sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
            return false;
        }
        if (!EmmSeqPoints_ResetAll()) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
        sendWithCRC("OK @%d EMM POINT RESET", seq);
        return true;
    }

    EmmSeqPointId id;
    if (!EmmSeqPoints_ParseName(name, &id)) {
        Serial.printf("[序列点位] 未知点位名：%s（2号：SAMPLE/DETECT/DISC1/DISC2；"
                      "3号：LIFT_S1/LIFT_S2/LIFT_DET/LIFT_D2/LIFT_D3）\n", name);
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    const uint32_t maxPulse = EmmSeqPoints_Max(id);
    if (value > (unsigned long)maxPulse) {
        Serial.printf("[序列点位] %s=%lu 超范围（0~%lu）\n",
                      EmmSeqPoints_Name(id), value, (unsigned long)maxPulse);
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    /*
     * **2026/10/21 起允许在复合动作运行中改单个点位**（用户要求："回零、任务序列时
     * 也可以更改"）。原来这里和 `CommandDispatcher` 各有一道 `SEQ_RUNNING` 拦截，
     * 两处都已放开。
     *
     * **为什么安全**：点位只改内存变量 + 写 NVS，不产生任何 RS485 总线事务、
     * 不触发运动。序列每一步是**进入时**才 `EmmSeqPoints_Get()` 读一次、随即把
     * 目标脉冲下发给驱动器并阻塞等到位，所以：
     *   - 正在执行的那一步：目标已下发，改点位**改不动它**（不会中途换目标）；
     *   - 还没走到的步骤：下一步就用新值。
     *
     * **两个"不是下一步就生效"的例外**（已在手册里写明）：
     *   - 回收阶段 `emmSeqRunDiscardPos()` 一次性读 4 个点位、连跑 5 个动作，
     *     回收中途改 `DISC1/DISC2/LIFT_D2/LIFT_D3` 要等**下一轮回收**；
     *   - 整机回零**不读点位**，回零中改点位对本次回零无影响。
     */
    if (!EmmSeqPoints_Set(id, (uint32_t)value)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    /*
     * 运行中改点位时补两件事（**2026/10/21 新增**）：
     *   ① 明确告知"当前动作不变、从下一步起生效"，免得现场以为没改成功；
     *   ② 立刻重跑一次点位关系自检 —— 运行中改点位最容易改出"单值合法但关系
     *      不合理"（例如把 `LIFT_S2` 改得比 `LIFT_S1` 还浅，"慢降"会变成上升），
     *      而这时离下一次 `SEQ START` 还很久，开跑自检根本不会跑。
     *      自检**只警告、不拒绝**（与开跑口径一致：可能正处在"调一半"的中间态）。
     */
    if (EmmTaskSeq_IsActive()) {
        Serial.println("[序列点位] 注意：任务序列正在运行，本条从下一步起生效（当前动作不变）");
        if (!EmmTaskSeq_CheckPoints()) {
            Serial.println("[序列点位] 点位关系存在可疑项（见上方警告），序列仍按这些值执行");
        }
    } else if (EmmSystemHome_IsActive()) {
        Serial.println("[序列点位] 注意：整机回零正在运行，本条对本次回零无影响（回零不读点位）");
    }

    sendWithCRC("OK @%d EMM POINT %s=%lu", seq, EmmSeqPoints_Name(id), value);
    return true;
}

bool cmdGetEmmPoint(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    /* 只回 2 号的 4 个：9 个字段一行会超过 sendWithCRC() 的 128 字节缓冲。 */
    sendWithCRC("OK @%d EMM POINT SAMPLE=%lu DETECT=%lu DISC1=%lu DISC2=%lu",
                seq,
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_SAMPLE),
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_DETECT),
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_DISCARD_1),
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_DISCARD_2));
    return true;
}

bool cmdGetEmmPointLift(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    sendWithCRC("OK @%d EMM POINT LIFT LIFT_S1=%lu LIFT_S2=%lu LIFT_DET=%lu "
                "LIFT_D2=%lu LIFT_D3=%lu",
                seq,
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_SAMPLE_1),
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_SAMPLE_2),
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DETECT),
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DISCARD_2),
                (unsigned long)EmmSeqPoints_Get(EMM_SEQ_POINT_LIFT_DISCARD_3));
    return true;
}

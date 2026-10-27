#ifndef EMM_SEQ_POINTS_H
#define EMM_SEQ_POINTS_H

#include <Arduino.h>

/**
 * @file EmmSeqPoints.h
 * @brief 任务序列（L2）机械点位的**运行期可改**参数 + NVS 掉电保存。
 *
 * @details **2026/10/19 新增（2 号 4 个点）**，**2026/10/20 扩展（3 号 5 个点）**。
 *          起因：现场需要频繁微调 —— 2 号（旋转）的取样位 / 检测位 / 回收位 1 /
 *          回收位 2，3 号（升降）的取样高度 1 / 取样高度 2 / 检测高度 /
 *          回收位第二段 / 回收位第三段，原先只能改 `#define` 再重新编译烧录，效率很低。
 *
 *          ## 为什么必须从宏改成变量
 *          `#define` 是**编译期文本替换**，运行期物理上改不了。所以"发命令改点位"
 *          的唯一技术路径是：**宏退化为"出厂默认值"，运行期用变量承载当前值**。
 *          命令只改变量（并写 NVS），不（也无法）改宏。
 *
 *          ## 数据流
 *          ```
 *          上电 → EmmSeqPoints_Init()
 *                   ├─ NVS 有值且合法 → 用 NVS 值
 *                   ├─ 无值 / 非法     → 用下面的宏默认值
 *                   └─ NVS 打不开     → 用宏默认值 + **保持未初始化**，
 *                                        下次 Get/Set 时自动重试（2 s 节流）
 *          SET EMM POINT SAMPLE=985          （2 号）
 *          SET EMM POINT LIFT_S1=32000       （3 号）
 *                   → 校验范围 → 改变量 → 写 NVS → 回 OK
 *          GET EMM POINT                     → 一次返回 2 号的 4 个值
 *          GET EMM POINT LIFT                → 一次返回 3 号的 5 个值
 *          SET EMM POINT RESET=1             → 9 个值全部恢复成宏默认值 + 清除 NVS
 *          ```
 *
 *          ## 单位与范围（**两条轴各一套上限**）
 *          单位是**命令脉冲**（3200 脉冲 = 一圈 = 360°）。
 *          - 2 号是单圈机构 → `0 ~ EMM_SEQ_POINT_PULSE_MAX`（= 3200）；
 *          - 3 号是丝杆长行程 → `0 ~ EMM_SEQ_LIFT_PULSE_MAX`（= 40000，比当前
 *            最大值 37000 高约 8%，主要用来拦住"多打一位"的笔误）。
 *          超范围一律回 `ERR 102 PARAM_ERR`，避免误填大数让电机多走几圈撞限位。
 *
 *          ## 为什么 `GET` 拆成两条（2026/10/20）
 *          `sendWithCRC()` 的格式化缓冲只有 **128 字节**（`crc16.cpp`）。9 个字段拼一行
 *          最坏约 140 字符，会被 `vsnprintf` **静默截断**；更糟的是 CRC 算在截断后的串上，
 *          **连换行都发不出去** —— 上位机会把下一条应答当成这一条的一部分。
 *          故拆成 `GET EMM POINT`（2 号 4 个）+ `GET EMM POINT LIFT`（3 号 5 个）：
 *          单条最长约 75 字符，远低于上限，且**不必改动全局发送函数**。
 *
 *          ## 并发
 *          点位由序列任务（Core 1）读、由命令任务写。`uint32_t` 在 ESP32 上是
 *          32 位对齐的单次读写，本身原子；用 `volatile` 只是防止编译器把它缓存到
 *          寄存器里，读不到别的任务刚写入的新值。
 *
 *          ## 与旧代码的关系
 *          9 个宏原先定义在 `EmmTaskSeq.cpp` 顶部，现已**移到本文件**（语义变成
 *          "出厂默认值"）。`EmmTaskSeq.cpp` 里的引用点改为
 *          `EmmSeqPoints_Get(EMM_SEQ_POINT_xxx)`，宏本身不再直接参与运动。
 *          例外：`EMM_SEQ_LIFT_POS_DISCARD_1`（回收位第一段，1000）**仍留在
 *          `EmmTaskSeq.cpp`** —— 它对应的动作已被注释停用，不纳入运行期可改。
 */

/* ==========================================================================
 * 出厂默认值（宏）
 * --------------------------------------------------------------------------
 * ⚠️ 这些是"**出厂默认值**"，不是运行期的当前值。
 *    运行期当前值请用 EmmSeqPoints_Get()；NVS 里一旦存过值，改这里的宏
 *    **不会**自动生效，需要先发 `SET EMM POINT RESET=1` 清掉 NVS。
 * ========================================================================== */

/* ---- 2 号（旋转，单圈 3200 脉冲）---- */

/** 旋转电机（A=2）位置：取样位正上方。 */
#define EMM_SEQ_ROT_POS_SAMPLE          985U
/** 旋转电机（A=2）位置：检测位正上方。 */
#define EMM_SEQ_ROT_POS_DETECT          2850U
/** 旋转电机（A=2）位置：回收位第一段。 */
#define EMM_SEQ_ROT_POS_DISCARD_1       320U
/** 旋转电机（A=2）位置：回收位第二段。 */
#define EMM_SEQ_ROT_POS_DISCARD_2       265U

/* ---- 3 号（升降，丝杆长行程）---- */

/** 升降电机（A=3）位置：取样高度 1（离针孔约 50 mm）。 */
#define EMM_SEQ_LIFT_POS_SAMPLE_1       32000U
/** 升降电机（A=3）位置：取样高度 2（慢速受阻检测终点）。 */
#define EMM_SEQ_LIFT_POS_SAMPLE_2       35000U
/** 升降电机（A=3）位置：检测高度。 */
#define EMM_SEQ_LIFT_POS_DETECT         26000U
/** 升降电机（A=3）位置：回收位第二段。 */
#define EMM_SEQ_LIFT_POS_DISCARD_2      37000U
/** 升降电机（A=3）位置：回收位第三段。 */
#define EMM_SEQ_LIFT_POS_DISCARD_3      31000U

/**
 * @brief 2 号（旋转）单个点位允许的最大脉冲数（单圈 = 3200 脉冲）。
 * @note 只用于 `SET EMM POINT` 的范围校验；`EMM_SEQ_ROT_POS_HOME`(0) 不在此模块管理。
 */
#define EMM_SEQ_POINT_PULSE_MAX         3200U

/**
 * @brief 3 号（升降）单个点位允许的最大脉冲数。
 * @note 当前实际用到的最大值是 `EMM_SEQ_LIFT_POS_DISCARD_2`(37000)，留约 8% 余量。
 *       主要作用是拦住"多打一位"的笔误（37000 → 370000），不是行程硬限位。
 */
#define EMM_SEQ_LIFT_PULSE_MAX          40000U

/**
 * @brief 可设置的序列点位编号。
 * @note 顺序与内部表一一对应，**不要改动已有取值**（NVS key 按下标映射）。
 *       2026/10/20 在末尾追加 3 号的 5 个点位，已有 0~3 保持不变，
 *       因此老设备 NVS 里的 4 个值不需要迁移。
 */
typedef enum
{
    EMM_SEQ_POINT_SAMPLE = 0,       ///< 取样位（2 号）
    EMM_SEQ_POINT_DETECT,           ///< 检测位（2 号）
    EMM_SEQ_POINT_DISCARD_1,        ///< 回收位第一段（2 号）
    EMM_SEQ_POINT_DISCARD_2,        ///< 回收位第二段（2 号）
    EMM_SEQ_POINT_LIFT_SAMPLE_1,    ///< 取样高度 1（3 号）**2026/10/20 新增**
    EMM_SEQ_POINT_LIFT_SAMPLE_2,    ///< 取样高度 2（3 号）
    EMM_SEQ_POINT_LIFT_DETECT,      ///< 检测高度（3 号）
    EMM_SEQ_POINT_LIFT_DISCARD_2,   ///< 回收位第二段（3 号）
    EMM_SEQ_POINT_LIFT_DISCARD_3,   ///< 回收位第三段（3 号）
    EMM_SEQ_POINT_COUNT             ///< 点位总数（哨兵 = 9）
} EmmSeqPointId;

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化点位模块：从 NVS 载入，缺失/非法则用宏默认值。
 * @details 可重复调用（已成功后直接返回 true）。若调用方忘了初始化，
 *          其余接口内部也会懒初始化一次，因此**不依赖调用顺序**。
 *          **NVS 打开失败时返回 false 且不置"已初始化"** —— 下次 Get/Set 会自动重试
 *          （2 s 节流，避免序列每一步都去开 NVS）。
 * @return true 初始化完成（含"首次上电没有 NVS 记录、使用默认值"）；
 *         false 仅表示 NVS 打不开（变量已退回宏默认值，稍后可重试）。
 */
bool EmmSeqPoints_Init(void);

/**
 * @brief 读取某个点位的**当前值**（命令脉冲）。
 * @param id 点位编号；越界返回 0。
 * @return 当前脉冲数。
 */
uint32_t EmmSeqPoints_Get(EmmSeqPointId id);

/**
 * @brief 读取某个点位的**出厂默认值**（即上面的宏）。
 * @param id 点位编号；越界返回 0。
 */
uint32_t EmmSeqPoints_Default(EmmSeqPointId id);

/**
 * @brief 读取某个点位允许的**最大脉冲数**（2 号 3200 / 3 号 40000）。
 * @param id 点位编号；越界返回 0。
 * @note 供命令层拼错误提示用；`EmmSeqPoints_Set()` 内部也用它做校验。
 */
uint32_t EmmSeqPoints_Max(EmmSeqPointId id);

/**
 * @brief 设置某个点位并写入 NVS。
 * @param id    点位编号。
 * @param pulse 目标脉冲数，必须 `<= EmmSeqPoints_Max(id)`。
 * @return true 设置并保存成功；false 编号/范围非法，或 NVS 写入失败。
 * @note **NVS 写失败时变量不会被更新**（实现是"先落盘、成功后再生效"），
 *       所以回 `ERR` 时设备状态与应答语义完全一致，不留"回了错但值已变"的半更新状态。
 */
bool EmmSeqPoints_Set(EmmSeqPointId id, uint32_t pulse);

/**
 * @brief 把全部 9 个点位恢复成宏默认值，并清除 NVS 里的记录。
 * @return true 全部成功；false NVS 打开失败（变量仍会被恢复成默认值）。
 */
bool EmmSeqPoints_ResetAll(void);

/**
 * @brief 把 9 个点位的**当前值**打印到串口（分两行：2 号一行、3 号一行）。
 * @param tag 抬头说明，例如 `"本次序列使用的点位"`。
 * @note **2026/10/20 新增**，供 `SEQ START` 开跑前打一份快照 —— 点位是运行期可改的，
 *       改完只写 NVS，跑序列时不再提示；有一份开跑快照才能事后对照"这次用的是哪套值"。
 */
void EmmSeqPoints_PrintAll(const char *tag);

/**
 * @brief 点位编号 → 上位机命令里用的短名。
 * @details 2 号：`SAMPLE` / `DETECT` / `DISC1` / `DISC2`；
 *          3 号：`LIFT_S1` / `LIFT_S2` / `LIFT_DET` / `LIFT_D2` / `LIFT_D3`。
 * @param id 点位编号；越界返回 `"?"`。
 */
const char *EmmSeqPoints_Name(EmmSeqPointId id);

/**
 * @brief 把上位机命令里的名字解析成点位编号（大小写不敏感）。
 * @details 2 号：`SAMPLE` / `DETECT` / `DISC1` / `DISC2`，别名
 *          `DISCARD1` / `DISCARD_1` / `RECYCLE1` / `RECYCLE_1`、
 *          `DISCARD2` / `DISCARD_2` / `RECYCLE2` / `RECYCLE_2`；
 *          3 号：`LIFT_S1` / `LIFT_S2` / `LIFT_DET` / `LIFT_D2` / `LIFT_D3`，别名
 *          `LIFT_SAMPLE1` / `LIFT_SAMPLE_1`、`LIFT_SAMPLE2` / `LIFT_SAMPLE_2`、
 *          `LIFT_DETECT`、`LIFT_DISC2` / `LIFT_DISCARD_2`、
 *          `LIFT_DISC3` / `LIFT_DISCARD_3`。
 * @param name 输入名字（不含 `=` 与数值）。
 * @param out  解析成功时写入编号。
 * @return true 解析成功。
 */
bool EmmSeqPoints_ParseName(const char *name, EmmSeqPointId *out);

/* ==========================================================================
 * L3 命令处理（由 CommandDispatcher 的命令表调用）
 * ========================================================================== */

/**
 * @brief `SET EMM POINT <NAME>=<PULSE>` —— 设置一个点位并掉电保存。
 * @details 支持的 NAME：
 *          - 2 号：`SAMPLE` / `DETECT` / `DISC1` / `DISC2`（范围 `0~3200`）
 *          - 3 号：`LIFT_S1` / `LIFT_S2` / `LIFT_DET` / `LIFT_D2` / `LIFT_D3`（范围 `0~40000`）
 *          别名见 `EmmSeqPoints_ParseName()`。
 *          特例 `SET EMM POINT RESET=1`：9 个全部恢复出厂默认值并清除 NVS。
 *
 *          | 情形 | 应答 |
 *          | ---- | ---- |
 *          | 成功 | `OK @n EMM POINT SAMPLE=985` / `OK @n EMM POINT LIFT_S1=32000` |
 *          | 恢复默认成功 | `OK @n EMM POINT RESET` |
 *          | 名字非法 / 数值超范围 / 格式错 | `ERR @n 102`（`PARAM_ERR`） |
 *          | `RESET=1` 但复合动作（任务序列 / 整机回零）正在运行 | `ERR @n 110`（`SEQ_RUNNING`） |
 *
 * @note **单点设置允许在复合动作运行中执行**（2026/10/21 用户要求："回零、任务序列时
 *       也可以更改"）。原来这里和 `CommandDispatcher` 各有一道 `SEQ_RUNNING` 拦截，
 *       两处都已放开。安全性来自读取时机 —— 序列每一步**进入时**才读点位、随即把
 *       目标下发并阻塞等到位，所以正在执行的那一步改不动，只有**还没走到的步骤**
 *       会用新值。改完会补一次点位关系自检（只警告、不拒绝）。
 *
 *       两个例外：
 *       - 回收阶段一次性读 4 个点位、连跑 5 个动作 → 回收中途改点位要等**下一轮回收**；
 *       - 整机回零**不读点位** → 回零中改点位对本次回零无影响。
 *
 * @note `RESET=1` **仍拒绝**（用户选定）：一次改 9 个点位，运行中放开的爆炸半径
 *       明显更大。判据覆盖"任务序列 **或** 整机回零"—— 因为两个 `BlocksCommand()`
 *       都已对 `SET EMM POINT` 整体放行，只判序列会让回零中发 `RESET=1` 绕过拦截。
 */
bool cmdSetEmmPoint(const char *args, int seq);

/**
 * @brief `GET EMM POINT` —— 返回 **2 号（旋转）的 4 个**点位当前值。
 * @details 应答：`OK @n EMM POINT SAMPLE=985 DETECT=2850 DISC1=320 DISC2=265`
 * @note 3 号的 5 个值见 `cmdGetEmmPointLift()`。拆两条是因为 `sendWithCRC()`
 *       的缓冲只有 128 字节，9 个字段一行会被静默截断（见文件头说明）。
 */
bool cmdGetEmmPoint(const char *args, int seq);

/**
 * @brief `GET EMM POINT LIFT` —— 返回 **3 号（升降）的 5 个**点位当前值。
 * @details 应答：`OK @n EMM POINT LIFT LIFT_S1=32000 LIFT_S2=35000 LIFT_DET=26000
 *          LIFT_D2=37000 LIFT_D3=31000`
 * @note 命令表里靠**最长前缀匹配**与 `GET EMM POINT` 区分（`findCommand()`），
 *       两者注册顺序无关。
 */
bool cmdGetEmmPointLift(const char *args, int seq);

#endif /* EMM_SEQ_POINTS_H */

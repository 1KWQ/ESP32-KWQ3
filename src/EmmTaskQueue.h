#ifndef EMM_TASK_QUEUE_H
#define EMM_TASK_QUEUE_H

#include <Arduino.h>

/**
 * @file  EmmTaskQueue.h
 * @brief 整机任务队列（L2）：只负责"建队列"，不负责执行。
 *
 * @details
 *   上位机通过 `SEQ ADD <c>=<A> T=<T>` 逐条追加任务，`SEQ CLEAR` 清空队列。
 *   本模块**只保存队列与序列状态**，不含执行引擎——它不会驱动任何电机，
 *   也不会占用 RS485 总线，因此可以在任意时刻安全调用。
 *
 *   命名与语义刻意对齐 KWQ1 的 `process.h`（`Task` / `taskQueue[]` /
 *   `taskLen` / `taskIdx` / `SeqState`），目的是后续把 KWQ1 的
 *   `processTaskQueue()` 序列状态机骨架平移过来时，改动量最小。
 *
 *   与 KWQ1 的差异（用户确认）：
 *     1. 队列上限由 36 条收紧到 **10** 条（`EMM_TASK_QUEUE_MAX`）；
 *     2. 取消 `S`（速度）字段，只保留 `A`（工位）与 `T`（时间）；
 *     3. `T` 的单位为**分钟**（与 KWQ1 一致；2026/10/14 由"秒"改回"分钟"），
 *        命令层**不设上限**（序列引擎侧有 30 分钟兜底）。
 *
 *   队列为纯 RAM，掉电丢失——与 KWQ3"参数不做掉电保存"的约定一致。
 *
 * @note 执行引擎（按队列真正驱动电机跑动作）属于后续的"开始任务"功能，
 *       见《修改日志.md》§七 的分步计划；本模块只到"队列管理"为止。
 */

/* ==========================================================================
 * 模块常量
 * ========================================================================== */

/** @brief 任务队列上限（条）。用户要求由 KWQ1 的 36 条收紧到 10 条。 */
#define EMM_TASK_QUEUE_MAX 10U

/** @brief 样品盘工位号下限（A 的取值下限）。 */
#define EMM_TASK_SLOT_MIN 1U

/** @brief 样品盘工位号上限（A 的取值上限）。 */
#define EMM_TASK_SLOT_MAX 10U

/* ==========================================================================
 * 数据结构
 * ========================================================================== */

/**
 * @brief 单条任务。
 * @details 对齐 KWQ1 的 `struct Task`，但去掉 `S`（速度）。
 */
struct EmmTask
{
    char    type;     ///< 任务类型：'A' 或 'B'（具体语义待"开始任务"阶段定义）。
    uint8_t slot;     ///< 样品盘工位号：1~10（对应 4 号样品盘电机 EMM_SAMPLE_TRAY_MOTOR）。
    /**
     * @brief 时间 T，单位**分钟**，必须 > 0。
     * @warning **字段名 `timeSec` 是历史命名，实际单位是分钟！**
     *          2026/10/14 用户要求把 `SEQ ADD` 的 `T` 由秒改为分钟，但保留字段名
     *          不改（避免大范围重命名）。读代码时务必按"分钟"理解：换算成毫秒
     *          要 `× 60000`（不是 `× 1000`）。
     */
    float   timeSec;
};

/**
 * @brief 序列运行状态（与 KWQ1 `process.h` 的 `enum SeqState` 同名同序）。
 * @details 本阶段只维护 `STOPPED`；其余取值预先定义好，一方面让 `GET STATUS`
 *          的 `STATUS` 字段与 KWQ1 格式保持一致，另一方面序列状态机可以直接
 *          使用，不必再改协议。
 *
 * @warning **枚举值不要加 `SEQ_` 前缀！** `crc16.h` 的 `ErrCode` 已经占用了
 *          `SEQ_RUNNING`（错误码 110，含义是"序列正在运行，命令被拒绝"），
 *          若这里写成 `SEQ_RUNNING` 会与它**重定义冲突**，编译直接报错
 *          （这是实际踩到过的编译错误）。因此枚举值与 KWQ1 保持一致，用
 *          不带前缀的 `STOPPED` / `RUNNING` / `PAUSED` / …，
 *          与错误码 `SEQ_RUNNING` 在代码里也天然区分得开。
 */
enum SeqState
{
    STOPPED = 0,   ///< 停止（队列未执行 / 已跑完）
    RUNNING,       ///< 正在执行
    PAUSED,        ///< 暂停
    END_CUR_ALL,   ///< 结束当前 / 全部（预留，本骨架未用）
    CLEARNING,     ///< 清洗中（预留，本骨架未用）
    MOVEING,       ///< 移动点位中（预留，本骨架未用）
    HOMEING        ///< 回零中（预留，本骨架未用）
};

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 队列初始化：清零队列、索引与序列状态。
 * @note 只清软件状态，不访问硬件，也不占用 RS485 总线。
 */
void EmmTaskQueue_Init(void);

/**
 * @brief 追加一条任务到队尾。
 * @param type     任务类型，只接受 'A' / 'B'。
 * @param slot     样品盘工位号，1~10。
 * @param timeSec  时间 T（**分钟**；字段名为历史命名，实际单位是分钟），必须 > 0。
 * @return true 入队成功；false 参数非法或队列已满。
 * @note 本函数只做"最后一道防线"的兜底校验；面向上位机的完整校验（含按
 *       失败原因回不同错误码）在 `cmdSeqAdd()` 里完成，因为错误码属于
 *       协议层，不应下沉到本模块。
 */
bool EmmTaskQueue_Add(char type, uint8_t slot, float timeSec);

/**
 * @brief 清空队列（`taskLen` / `taskIdx` 归零）。
 * @note 不改变 `seqState`；后续序列状态机运行时，是否允许清空由命令层判断。
 */
void EmmTaskQueue_Clear(void);

/**
 * @brief 当前队列长度（已入队条数）。
 */
uint8_t EmmTaskQueue_Count(void);

/**
 * @brief 当前执行索引（0 基）。本阶段恒为 0，供后续状态机使用。
 */
uint8_t EmmTaskQueue_Index(void);

/**
 * @brief 队列是否已满（用于 `SEQ ADD` 回 TASK_FULL）。
 */
bool EmmTaskQueue_IsFull(void);

/**
 * @brief 队列上限（条），供日志与 `GET STATUS` 使用。
 */
uint8_t EmmTaskQueue_Capacity(void);

/**
 * @brief 取序列运行状态，供 `GET STATUS` 使用。
 */
SeqState EmmTaskQueue_GetSeqState(void);

/* --------------------------------------------------------------------------
 * 以下三个接口供 L2 序列执行引擎（`EmmTaskSeq`）读写队列内部状态使用。
 * 队列本身的"存/取"语义不变，只是把原来封在 .cpp 里的游标与状态暴露出来，
 * 避免引擎重复实现一份队列。
 * -------------------------------------------------------------------------- */

/**
 * @brief 设置序列运行状态。
 * @details 状态发生变化时打印一条 `[任务队列] 序列状态：X -> Y` 日志，便于现场
 *          按串口日志还原整条序列的推进过程；重复设置同一状态不打印。
 * @param s 目标状态。
 */
void EmmTaskQueue_SetSeqState(SeqState s);

/**
 * @brief 读取第 `idx` 条任务的**只读副本**。
 * @param idx 0 基下标。
 * @param out 输出缓冲，调用方保证非空。
 * @return true 下标有效且已填充；false 下标越界（`out` 不被修改）。
 */
bool EmmTaskQueue_GetTask(uint8_t idx, EmmTask *out);

/**
 * @brief 设置当前执行索引（序列执行引擎在任务切换时调用）。
 * @details 合法范围是 `[0, EMM_TASK_QUEUE_MAX]`——恰好等于 `EMM_TASK_QUEUE_MAX`
 *          表示"已越过最后一条"（序列跑完的判定依据），因此允许写入。
 *          仅当 `idx > EMM_TASK_QUEUE_MAX` 时忽略并打印告警，保证游标永远不会
 *          指到队列数组之外。
 */
void EmmTaskQueue_SetIndex(uint8_t idx);

/**
 * @brief 序列状态枚举转字符串（仅用于日志，不参与协议）。
 * @return 固定字面量，如 `"STOPPED"`；未知取值返回 `"UNKNOWN"`。
 */
const char *EmmTaskQueue_SeqStateToStr(SeqState s);

#endif  /* EMM_TASK_QUEUE_H */

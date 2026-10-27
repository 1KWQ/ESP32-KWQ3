/**
 * @file  EmmTaskQueue.cpp
 * @brief 整机任务队列实现（ESP32-KWQ3，L2，只建队列不执行）。
 *
 * @details
 *   本文件只维护一份静态队列与序列状态，**不做任何电机动作、不占用 RS485
 *   总线**，因此所有接口都是瞬时返回的纯内存操作，可以直接在串口命令
 *   回调里同步调用，无需额外 FreeRTOS 任务。
 *
 *   与命令层（CommandDispatcher）的分工：
 *     - `cmdSeqAdd()`  负责解析文本、按失败原因回不同错误码，再调用本模块入队；
 *     - 本模块只负责"存"与"取"，并提供兜底校验，避免被误用时把非法数据写进队列。
 */

#include "EmmTaskQueue.h"
#include "common.h"   /* LOGD（日志分级，2026/10/21） */

#include <string.h>

/* ==========================================================================
 * 模块内部状态
 * ========================================================================== */

/** @brief 任务队列本体。上限 `EMM_TASK_QUEUE_MAX`（10）条，纯 RAM，掉电丢失。 */
static EmmTask s_queue[EMM_TASK_QUEUE_MAX];

/** @brief 当前队列长度（已入队条数，也是下一条任务的写入下标）。 */
static uint8_t s_taskLen = 0;

/** @brief 当前执行索引（本阶段固定为 0，供后续序列状态机使用）。 */
static uint8_t s_taskIdx = 0;

/** @brief 序列运行状态。由 `EmmTaskSeq` 推进，未启动时为 `STOPPED`。 */
static SeqState s_seqState = STOPPED;

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

void EmmTaskQueue_Init(void)
{
    memset(s_queue, 0, sizeof(s_queue));
    s_taskLen = 0;
    s_taskIdx = 0;
    s_seqState = STOPPED;

    Serial.printf("[任务队列] 初始化完成：上限 %u 条，当前 0 条\n",
                  (unsigned)EMM_TASK_QUEUE_MAX);
}

bool EmmTaskQueue_Add(char type, uint8_t slot, float timeSec)
{
    /*
     * 兜底校验。命令层 cmdSeqAdd() 已按协议做过完整校验并回了具体错误码，
     * 这里再拦一次是为了防止其它调用方（或将来新增的批量导入路径）把非法
     * 数据直接塞进队列，导致后续状态机读到脏数据。
     */
    
    if (type != 'A' && type != 'B') {
        Serial.printf("[任务队列] 拒绝入队：类型 '%c' 非法（只允许 A/B）\n", type);
        return false;
    }
    if (slot < EMM_TASK_SLOT_MIN || slot > EMM_TASK_SLOT_MAX) {
        Serial.printf("[任务队列] 拒绝入队：工位 %u 超出 %u~%u\n",
                      (unsigned)slot,
                      (unsigned)EMM_TASK_SLOT_MIN, (unsigned)EMM_TASK_SLOT_MAX);
        return false;
    }
    if (!(timeSec > 0.0f)) {
        Serial.printf("[任务队列] 拒绝入队：T=%.3f min 必须大于 0\n", (double)timeSec);
        return false;
    }
    if (s_taskLen >= EMM_TASK_QUEUE_MAX) {
        Serial.printf("[任务队列] 拒绝入队：队列已满（%u/%u）\n",
                      (unsigned)s_taskLen, (unsigned)EMM_TASK_QUEUE_MAX);
        return false;
    }

    s_queue[s_taskLen].type = type;
    s_queue[s_taskLen].slot = slot;
    s_queue[s_taskLen].timeSec = timeSec;
    s_taskLen++;

    /* 打印"刚入队的那条"与"当前总长度"，便于上位机侧核对与排错。 */
    Serial.printf("[任务队列] 已添加第 %u 条：类型=%c 工位=%02u T=%.3f min（当前 %u/%u 条）\n",
                  (unsigned)s_taskLen, type, (unsigned)slot, (double)timeSec,
                  (unsigned)s_taskLen, (unsigned)EMM_TASK_QUEUE_MAX);
    return true;
}

void EmmTaskQueue_Clear(void)
{
    const uint8_t before = s_taskLen;

    s_taskLen = 0;
    s_taskIdx = 0;
    memset(s_queue, 0, sizeof(s_queue));

    /* **2026/10/21**：降为 `LOGD` —— 每次序列跑完都会刷这一行，属于收尾流水账；
       真正表示"跑完了"的是 `EVT 300 SEQ ALL DONE`。 */
    LOGD("[任务队列] 已清空：原有 %u 条，当前 0/%u 条\n",
         (unsigned)before, (unsigned)EMM_TASK_QUEUE_MAX);
}

uint8_t EmmTaskQueue_Count(void)
{
    return s_taskLen;
}

uint8_t EmmTaskQueue_Index(void)
{
    return s_taskIdx;
}

bool EmmTaskQueue_IsFull(void)
{
    return s_taskLen >= EMM_TASK_QUEUE_MAX;
}

uint8_t EmmTaskQueue_Capacity(void)
{
    return (uint8_t)EMM_TASK_QUEUE_MAX;
}

SeqState EmmTaskQueue_GetSeqState(void)
{
    return s_seqState;
}

/* ==========================================================================
 * 供 L2 序列执行引擎（EmmTaskSeq）使用的读写接口
 * ========================================================================== */

const char *EmmTaskQueue_SeqStateToStr(SeqState s)
{
    /* 顺序必须与 EmmTaskQueue.h 里的 enum SeqState 完全一致。
       注意枚举值不带 SEQ_ 前缀（避免与错误码 SEQ_RUNNING 重定义冲突）。 */
    static const char *const kNames[] = {
        "STOPPED",      /* STOPPED      */
        "RUNNING",      /* RUNNING      */
        "PAUSED",       /* PAUSED       */
        "END_CUR_ALL",  /* END_CUR_ALL  */
        "CLEARNING",    /* CLEARNING    */
        "MOVEING",      /* MOVEING      */
        "HOMEING"       /* HOMEING      */
    };

    if ((int)s < 0 || (size_t)s >= (sizeof(kNames) / sizeof(kNames[0]))) {
        return "UNKNOWN";
    }
    return kNames[(int)s];
}

void EmmTaskQueue_SetSeqState(SeqState s)
{
    if (s == s_seqState) {
        return;     /* 状态未变，不重复打印，避免刷屏。 */
    }

    Serial.printf("[任务队列] 序列状态：%s -> %s\n",
                  EmmTaskQueue_SeqStateToStr(s_seqState),
                  EmmTaskQueue_SeqStateToStr(s));
    s_seqState = s;
}

bool EmmTaskQueue_GetTask(uint8_t idx, EmmTask *out)
{
    if (out == nullptr) {
        return false;
    }
    if (idx >= s_taskLen) {
        /* 只允许读到已入队的条目，读"未写入的槽位"属于调用方逻辑错误。 */
        return false;
    }

    *out = s_queue[idx];
    return true;
}

void EmmTaskQueue_SetIndex(uint8_t idx)
{
    /*
     * 合法取值范围是 [0, EMM_TASK_QUEUE_MAX]：
     *   - 0 ~ MAX-1 指向某条具体任务；
     *   - 恰好等于 MAX 表示"已越过最后一条"，是序列跑完的判定依据
     *     （`Index() >= Count()`），因此**必须允许**写入 MAX。
     * 只有 > MAX 才判越界。
     */
    if (idx > EMM_TASK_QUEUE_MAX) {
        Serial.printf("[任务队列] 忽略越界的执行索引：%u（上限 %u）\n",
                      (unsigned)idx, (unsigned)EMM_TASK_QUEUE_MAX);
        return;
    }
    s_taskIdx = idx;
}

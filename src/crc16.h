#ifndef CRC16_H
#define CRC16_H

#include  <Arduino.h>

uint16_t crc16_modbus(const uint8_t* data,size_t len);
void sendWithCRC(const char *fmt, ...);

typedef enum
{
    CRC_ERR = 101, // CRCC 错误
    PARAM_ERR,     // 参数错误
    TASK_NULL,     // 循环任务队列为空
    NO_HOME,       // 没有回零
    NO_SAMPLE,     // 样品盘没有到位
    NO_WATER,      // 水瓶没有放到位
    NO_DOOR,       // 门没有到位
    NO_PWR,        // 外部电源没通电
    NO_STEP_PWR,   // 结构没通电, 软件控制/休眠的断电，，没有重新打开电源
    SEQ_RUNNING,    // 序列正在运行
    ST_PARAM_ERR,   // 循环任务 ST参数错误
    RS485_BUSY,     // RS485 总线忙
    SV04M_CHANGE_FAILED, // 切换 SV-04M 通道失败
    TASK_FULL,      // 任务队列已满（SEQ ADD；队列上限 10 条，用户要求回明确错误）
    NO_HEARTBEAT,   // 上位机心跳不在线（SEQ START 前置检查①；PING 超时未恢复）
    TEMP_HIGH,      // 控制板温度超限（SEQ START 前置检查；5 路驱动温度最大值 > SET TEMP 阈值）
    EMM_HOME_FAILED // 样品盘单独回零（SET EMM TRAY HOME）执行失败（2026/10/19 新增，值 117）
}ErrCode;


typedef enum
{
    TASK_DONE = 300,  // 任务完成
    CUR_TASK_DONE = 301,  // 当前任务完成
    CUR_TASK_SKIP = 302,  // 当前任务跳过
    CHANGE_SOLVENT = 303,  // 更换溶剂
    FAILED_REMOVE = 304,  // 放料失败
    MOVE_D_TIMEOUT = 305,  // 移动D点超时
                           // （2026/10/21 起用于任务序列：任一步没做完时发
                           //   `EVT ERR 305 STEP FAILED`，见 EmmTaskSeq::emmTaskSeqStepFailed）
    ROTARY_NOT_HOME = 306, // 任务序列收尾时 2 号（旋转）未回到单圈回零原点
                           // （2026/10/19 新增：复位阶段改为绝对定位到 0 + 读回核对，
                           //   失败时先发本码、再照常发 TASK_DONE，旧上位机不受影响）

    /* ======================================================================
     * 阶段事件（**2026/10/21 新增**）：配合"精简日志"使用。
     * ----------------------------------------------------------------------
     * 背景：详细流水账默认关闭（`LOGD`），上位机只收这些阶段级事件。
     * 码位按**十位分组**，方便扩展、也方便上位机做区间判断：
     *   31x = 前置检查   32x = 取样   33x = 检测   34x = 回收
     *   35x = 复位（队列跑完后的收尾复位）        36x = 整机回零
     *
     * 帧格式：
     *   正常：`EVT <码> <阶段> BEGIN|DONE <任务号>/<总数>`（320 额外带 `A=`/`T=`，
     *         332 额外带 `T=`）
     *   失败：`ERR @<seq> <原因> <任务号>/<总数>`（**2026/10/21 修订**）
     * 例：
     *   EVT 320 SAMPLE BEGIN 1/3 A=01 T=0.100
     *   EVT 332 DWELL BEGIN 1/3 T=0.100
     *   ERR 取样动作失败 1/3
     *
     * ⚠️ **失败上报 2026/10/21 改版**：原来发 `EVT ERR 322/332/342 <阶段> FAIL`
     *    + `EVT ERR 305 STEP FAILED` 两帧，现统一成**一帧 `ERR @…`** ——
     *    上位机只把 `ERR` 当板子报错，`EVT ERR` 会被当未知事件忽略。
     *    原因用中文短句（`取样动作失败` / `检测位动作失败` / `复位失败` …）。
     *    注意这与命令应答 `ERR @<seq> <数字错误码>` 共用 `ERR` 前缀，
     *    解析时**不要把第二个字段强行转数字**。
     *
     * 老上位机只认 300/305/306，收到 31x~36x 会当未知事件忽略，不会崩。
     * ====================================================================== */

    SEQ_CHECK_BEGIN = 310,   // 前置检查开始（`SEQ START` 已回 OK 之后）
    SEQ_CHECK_OK,            // 311 前置检查全部通过（失败仍走 `ERR @n 1xx`，不重复报）

    SEQ_SAMPLE_BEGIN = 320,  // 开始取样（含样品盘定位 + 取样位 9 步）
                             //   帧带工位与停留时间：`EVT 320 SAMPLE BEGIN 1/1 A=01 T=0.100`
    SEQ_SAMPLE_DONE,         // 321 取样完成

    SEQ_DETECT_BEGIN = 330,  // 开始检测（含检测位定位）
    SEQ_DETECT_DONE,         // 331 检测完成

    /* ---- 检测位停留（**2026/10/21 新增**）----
     * 探头降到检测位后开始计时 / 计时到后抬起之前，各发一条。
     * 上位机据此开 CA + 开始采集 / 停采集 + 关 CA。
     *   EVT 332 DWELL BEGIN 1/1 T=0.100   （T= 是**实际**停留时间，已按 30 min 上限钳位）
     *   EVT 333 DWELL END 1/1
     * ⚠️ 332 原本是 `DETECT FAIL`，2026/10/21 起失败统一改发 `ERR <原因> <n/m>`，
     *    该码位退役后腾给 `DWELL BEGIN`。
     */
    SEQ_DWELL_BEGIN,         // 332 开始停留（探头已在检测位，开始计时）
    SEQ_DWELL_END,           // 333 停留结束（计时到，抬起之前）

    SEQ_RECYCLE_BEGIN = 340, // 开始回收
    SEQ_RECYCLE_DONE,        // 341 回收完成

    /* ---- 复位阶段（**2026/10/21 新增**）：队列跑完后的收尾复位 ----
     * 与整机回零同一套思路：3 号上升 / 2 号回原点 / 4 号样品盘回零，
     * 只在"2 号核对通过"和"4 号回零完成"两个节点各报一条，
     * 3 号上升由 RESET BEGIN 代表（失败会给 RESET FAIL）。
     *   EVT 350 RESET BEGIN
     *   EVT 351 RESET AXIS 2      （2 号已回单圈回零原点，读回核对通过）
     *   EVT 352 RESET AXIS 4      （4 号样品盘接近开关回零完成）
     *   EVT 300 SEQ ALL DONE      （复位完成即整条序列完成，不再单发 RESET DONE）
     * 失败：`ERR 复位失败 <n>/<m>`（2 号失败不中止，只发 306）
     */
    RESET_BEGIN = 350,       // 350 复位开始（3 号开始上升）
    RESET_AXIS_2,            // 351 2 号已回到单圈回零原点且核对通过
    RESET_AXIS_4,            // 352 4 号样品盘接近开关回零完成

    HOME_DONE = 360,         // 整机回零 17 步全部成功
    HOME_FAIL,               // 361 整机回零失败（`HOMING` 保持 0）
    HOME_STOPPED,            // 362 整机回零被 `SET EMM SYSTEM HOME STOP` 主动中止

    /* ---- 整机回零进度（**2026/10/21 新增**）：按机械轴分段 ----
     * 17 步逐条上报太碎（违背"只保留关键消息"），全静默又看不出卡在哪；
     * 折中成"**换轴时报一条**"，配合 HOME DONE 就能定位到哪一段没走完。
     *
     * ⚠️ **2026/10/23 起轴段不是 3 段而是 6 段**：回零流程改成"3 号与 2 号交替"
     * （3 号回零/退让 → 2 号回零 → 3 号回零/退让 → 2 号回零/顺 800/2 号回零
     * → 3 号回零/下 2500 → 样品盘），所以同一条码会出现多次：
     *   EVT 363 HOME BEGIN
     *   EVT 370 HOME AXIS 3      （第 1 段：3 号第 1 次回零 + 下 1200）
     *   EVT 371 HOME AXIS 2      （第 2 段：2 号第 1 次回零）
     *   EVT 370 HOME AXIS 3      （第 3 段：3 号第 2 次回零 + 下 1200）
     *   EVT 371 HOME AXIS 2      （第 4 段：2 号第 2、3 次回零 + 顺 800）
     *   EVT 370 HOME AXIS 3      （第 5 段：3 号第 3 次回零 + 下 2500）
     *   EVT 372 HOME AXIS 4      （第 6 段：4 号样品盘接近开关标定）
     *   EVT 360 HOME DONE
     * 上位机**不要假设 370/371 各只出现一次**，按"收到即刷新当前轴"处理。
     */
    HOME_BEGIN = 363,        // 363 整机回零开始（上电自动回零没有命令应答，靠这条确认已启动）
    HOME_AXIS_3 = 370,       // 370 进入 3 号（升降）段（新流程出现 3 次）
    HOME_AXIS_2,             // 371 进入 2 号（旋转）段（新流程出现 2 次）
    HOME_AXIS_4,             // 372 进入 4 号（样品盘）段（1 次）

    /* ---- 暂停归位（**2026/10/27 新增**）----
     * `SEQ PAUSE` 改成"归位式暂停"：命令**立即**回 `OK @n SEQ PAUSE`，但真正的
     * `PAUSED` 状态要等"回收（若已取样）→ 3 号抬到安全高度 → 2 号转到取样位
     * 正上方"全部做完才置位。上位机据此判断"现在可以 `SEQ RESUME` 了"。
     *   EVT 380 PAUSE PARKED
     * ⚠️ 380 与 `OK @n SEQ PAUSE` 之间可能相隔数秒（回收 + 归位），
     *    **不要**把命令应答当成"已暂停"。
     */
    SEQ_PAUSE_PARKED = 380,
}EVentCode;


#endif

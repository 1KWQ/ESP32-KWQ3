#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include <Arduino.h>

/**
 * @brief 命令处理函数类型
 * @param args 参数字符串
 * @param seq 当前命令序号
 * @return true 表示处理成功，false 表示失败
 */
typedef bool (*CmdHandler)(const char* args, int seq);

/**
 * @brief 命令表条目
 */
struct CmdEntry {
    const char* name;   ///< 命令名称
    CmdHandler handler; ///< 处理函数
};

/**
 * @brief 命令分发器
 */
class CommandDispatcher {
public:
    /**
     * @brief 构造函数
     */
    CommandDispatcher();

    /**
     * @brief 查找命令
     * @param cmd 命令字符串
     * @return 指向 CmdEntry 的指针，如果没找到返回 nullptr
     */
    const CmdEntry* findCommand(const char* cmd);

    /**
     * @brief 分发命令
     * @param cmd 命令字符串
     * @param seq 序号
     * @return true 成功，false 失败
     */
    bool dispatch(const char* cmd, int seq);
};

/**
 * @brief 启动 3 号升降电机的相电流轮询监控后台任务（电离头扎偏保护）。
 * @details 该任务按固定周期轮询升降电机的相电流，一旦超过安全阈值即判定为
 *          “电离头扎偏”故障：先让升降电机以相对位置模式上升固定脉冲数到安全
 *          高度，随后广播停止所有 EMM 电机，并打印告警文本。实现位于
 *          CommandDispatcher.cpp。
 * @note 必须在 Rs485Devices_Init() 之后调用，确保共享 RS485 总线、EMM 驱动
 *       对象与总线互斥锁均已就绪。重复调用安全。
 * @return true 监控任务已启动或早已运行；false 任务创建失败。
 */
bool EmmLiftCurrentMonitor_Init(void);

/**
 * @brief 布防/撤防 3 号升降电机的扎偏电流检测。
 * @details ARM=1 时置位布防标志并唤醒监控任务开始快速轮询相电流；ARM=0 时清除
 *          标志，监控任务在当轮结束后退出轮询并重新阻塞。调试阶段由上位机通过
 *          SET EMM LIFT MONITOR ARM=<0|1> 显式调用；建议只在两段下降的慢速段
 *          布防，避免快速段大电流（可达 900mA）造成误触发。
 * @param enable true 布防开始检测；false 撤防停止检测。
 */
void EmmLiftCurrentMonitor_Arm(bool enable);

/**
 * @brief 断电 / 指令超时休眠的统一收尾：复位所有复合动作与瞬态标志/状态位。
 * @details 对应 KWQ1 `SET POWER OFF` 的 `resetAllSubStateMachines()` +
 *          `clearAllTaskState()`。执行顺序：
 *            ① 停任务序列并清队列（`EmmTaskSeq_Stop()`）；
 *            ② 请求中止整机回零（`EmmSystemHome_RequestStop()`）；
 *            ③ 请求中止系统初始化（`SysInit_Abort()`）；
 *            ④ 清回零跟踪表 / 升降过流计时 / 升降监控布防；
 *            ⑤ 广播停止所有 EMM 电机；
 *            ⑥ 断开结构电源（`sysPoweronoFF(0)`）；
 *            ⑦ `sys_is_homed = false`（零点失效，重新上电必须重新回零）；
 *            ⑧ 三色灯回待机黄色。
 *
 *          本函数是 `SET POWER OFF`（`cmdPower`）与指令超时休眠
 *          （`main.cpp::checkPCAlive()`）**共用**的唯一收尾入口，避免两条路径
 *          各自维护一份收尾逻辑而逐渐走偏。函数内部会把 `cmd_power_off` 置 true，
 *          调用方无需重复设置。
 *
 * @note 正在运行的机械动作（回零 / 初始化）只做"请求中止"，由各自的后台任务在
 *       动作边界干净退出——**不会**在这里强制清它们的 busy 门闩，否则会造成
 *       两套流程并行驱动同一批电机。
 */
void sysPowerOffCleanup(void);

#endif

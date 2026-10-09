/**
 * @file CommandDispatcher.cpp
 * @author your name (you@domain.com)
 * @brief 上位机指令解析（ESP32-KWQ3：仅 EMM 仪器 + 保留外设）
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @details 本文件由 ESP32-KWQ2 的 CommandDispatcher.cpp 提纯而来，只保留：
 *          - EMM V5 / X42S 单一动作命令（与《EMM电机上位机命令手册.md》一一对应）；
 *          - 本工程保留的外设命令：PING / SET POWER / SET RYG / SET TEMP /
 *            GET TEMP / SET FA / SET VALVE / GET VALVE / GET LASER。
 *          复合回零（SET EMM SYSTEM HOME，L2 层，实现见 EmmSystemHome.cpp）、
 *          EMM 任务流程（SET EMM TASK *）、XYZ/ZS/序列相关命令均已移除。
 *          GET STATUS 仅保留命令名，实现留空，待后续增加任务序列时补充。
 */

#include "CommandDispatcher.h"
#include <string.h>
#include "common.h"
#include "Rs485Devices.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "CMD";

// ================== 命令处理函数声明 ==================
static bool cmdPing(const char *args, int seq);
static bool cmdPower(const char *args, int seq);
static bool cmdGetStatus(const char *args, int seq);
static bool cmdGetTemp(const char *args, int seq);
static bool cmdSetFa(const char *args, int seq);
static bool cmdSetRYGvoltage(const char *args, int seq);
static bool cmdSetMaxTemp(const char *args, int seq);
static bool cmdSetRESTART(const char *args, int seq);
static bool cmdSetValveChannel(const char *args, int seq);
static bool cmdGetLaserDistance(const char *args, int seq);
static bool cmdGetValveStatus(const char *args, int seq);

/**
 * @brief EMM V5/X42S 上位机命令处理函数声明。
 * @details 每个处理函数都接收命令参数和序号：参数负责解析 A、DIR、RPM 等键值，
 *          序号用于生成带 @seq 和 CRC 的回复。真正的协议帧由 gEmm 统一生成。
 */
static bool cmdSetEmmEnable(const char *args, int seq);
static bool cmdSetEmmSpeed(const char *args, int seq);
static bool cmdSetEmmMove(const char *args, int seq);
static bool cmdSetEmmStop(const char *args, int seq);
static bool cmdSetEmmLiftMonitor(const char *args, int seq);
static bool cmdSetEmmHome(const char *args, int seq);
static bool cmdSetEmmHomeStop(const char *args, int seq);
static bool cmdSetEmmHomeParams(const char *args, int seq);
static bool cmdSetEmmZero(const char *args, int seq);
static bool cmdSetEmmSync(const char *args, int seq);
static bool cmdSetEmmOriginZero(const char *args, int seq);
static bool cmdSetEmmResetProtect(const char *args, int seq);
static bool cmdSetEmmCalibrate(const char *args, int seq);
static bool cmdSetEmmRestart(const char *args, int seq);
static bool cmdSetEmmFactory(const char *args, int seq);
static bool cmdSetEmmFastParam(const char *args, int seq);
static bool cmdSetEmmFastMove(const char *args, int seq);
static bool cmdSetEmmTimer(const char *args, int seq);
static bool cmdSetEmmAddress(const char *args, int seq);
static bool cmdSetEmmMicrostep(const char *args, int seq);
static bool cmdSetEmmPowerLoss(const char *args, int seq);
static bool cmdSetEmmMotorType(const char *args, int seq);
static bool cmdSetEmmFirmwareType(const char *args, int seq);
static bool cmdSetEmmControlMode(const char *args, int seq);
static bool cmdSetEmmDirection(const char *args, int seq);
static bool cmdSetEmmKeyLock(const char *args, int seq);
static bool cmdSetEmmSpeedScale(const char *args, int seq);
static bool cmdSetEmmOpenCurrent(const char *args, int seq);
static bool cmdSetEmmMaxCurrent(const char *args, int seq);
static bool cmdSetEmmLockLevel(const char *args, int seq);
static bool cmdSetEmmPid(const char *args, int seq);
static bool cmdSetEmmAutoSpeed(const char *args, int seq);
static bool cmdSetEmmConfig(const char *args, int seq);
static bool cmdSetEmmMulti(const char *args, int seq);
static bool cmdSetEmmPositionWindow(const char *args, int seq);
static bool cmdSetEmmProtection(const char *args, int seq);
static bool cmdSetEmmHeartbeat(const char *args, int seq);
static bool cmdSetEmmIntegral(const char *args, int seq);
static bool cmdSetEmmCollisionAngle(const char *args, int seq);
static bool cmdSetEmmDmx(const char *args, int seq);
static bool cmdGetEmmPos(const char *args, int seq);
static bool cmdGetEmmSpeed(const char *args, int seq);
static bool cmdGetEmmFlags(const char *args, int seq);
static bool cmdGetEmmState(const char *args, int seq);
static bool cmdGetEmmOrigin(const char *args, int seq);
static bool cmdGetEmmHomeParams(const char *args, int seq);
static bool cmdGetEmmVersion(const char *args, int seq);
static bool cmdGetEmmPhase(const char *args, int seq);
static bool cmdGetEmmBusVoltage(const char *args, int seq);
static bool cmdGetEmmPhaseCurrent(const char *args, int seq);
static bool cmdGetEmmEncoder(const char *args, int seq);
static bool cmdGetEmmInputPulse(const char *args, int seq);
static bool cmdGetEmmBusCurrent(const char *args, int seq);
static bool cmdGetEmmIo(const char *args, int seq);
static bool cmdGetEmmOptions(const char *args, int seq);
static bool cmdGetEmmTemperature(const char *args, int seq);
static bool cmdGetEmmPositionWindow(const char *args, int seq);
static bool cmdGetEmmProtection(const char *args, int seq);
static bool cmdGetEmmHeartbeat(const char *args, int seq);
static bool cmdGetEmmIntegral(const char *args, int seq);
static bool cmdGetEmmCollisionAngle(const char *args, int seq);
static bool cmdGetEmmBroadcastAddress(const char *args, int seq);
static bool cmdGetEmmDmx(const char *args, int seq);
static bool cmdGetEmmPid(const char *args, int seq);
static bool cmdGetEmmConfig(const char *args, int seq);
static bool cmdScanEmm(const char *args, int seq);

// ================== 命令表 ==================
static const CmdEntry cmdTable[] = {
    {"PING", cmdPing},
    {"SET POWER", cmdPower},
    {"SET RYG", cmdSetRYGvoltage},
    {"SET TEMP", cmdSetMaxTemp},
    {"SET FA", cmdSetFa},
    {"SET RESTART", cmdSetRESTART},
    {"GET STATUS", cmdGetStatus},
    {"GET TEMP", cmdGetTemp},
    {"GET LASER", cmdGetLaserDistance},
    {"SET VALVE", cmdSetValveChannel},
    {"GET VALVE", cmdGetValveStatus},

    /********* EMM V5 闭环步进电机：命令名必须与上位机前缀匹配 *********/
    {"SET EMM ENABLE", cmdSetEmmEnable},
    {"SET EMM SPEED", cmdSetEmmSpeed},
    {"SET EMM MOVE", cmdSetEmmMove},
    {"SET EMM STOP", cmdSetEmmStop},
    {"SET EMM LIFT MONITOR", cmdSetEmmLiftMonitor},//升降电机扎偏检测 布防/撤防
    {"SET EMM HOME STOP", cmdSetEmmHomeStop},
    {"SET EMM HOME PARAM", cmdSetEmmHomeParams},
    {"SET EMM HOME", cmdSetEmmHome},
    {"SET EMM SYSTEM HOME STOP", cmdSetEmmSystemHomeStop},//L2 整机回零：中止
    {"SET EMM SYSTEM HOME", cmdSetEmmSystemHome},//L2 整机回零：启动（12 步序列）
    {"SET EMM ZERO", cmdSetEmmZero},//将当前角度清零5.2.3
    {"SET EMM SYNC", cmdSetEmmSync},
    {"SET EMM ORIGIN ZERO", cmdSetEmmOriginZero},//单圈回零的零点设置
    {"SET EMM RESET PROTECT", cmdSetEmmResetProtect},//解除堵转/过热/过流保护
    {"SET EMM CALIBRATE", cmdSetEmmCalibrate},//触发编码器校准
    {"SET EMM RESTART", cmdSetEmmRestart},//重启电机
    {"SET EMM FACTORY", cmdSetEmmFactory},//恢复出厂设置
    {"SET EMM FAST PARAM", cmdSetEmmFastParam},
    {"SET EMM FAST MOVE", cmdSetEmmFastMove},
    {"SET EMM TIMER", cmdSetEmmTimer},
    {"SET EMM ADDRESS", cmdSetEmmAddress},
    {"SET EMM MICROSTEP", cmdSetEmmMicrostep},//修改电机细分参数
    {"SET EMM POWER LOSS", cmdSetEmmPowerLoss},
    {"SET EMM MOTOR TYPE", cmdSetEmmMotorType},
    {"SET EMM FIRMWARE TYPE", cmdSetEmmFirmwareType},
    {"SET EMM CONTROL MODE", cmdSetEmmControlMode},
    {"SET EMM DIRECTION", cmdSetEmmDirection},
    {"SET EMM KEY LOCK", cmdSetEmmKeyLock},
    {"SET EMM SPEED SCALE", cmdSetEmmSpeedScale},
    {"SET EMM OPEN CURRENT", cmdSetEmmOpenCurrent},
    {"SET EMM MAX CURRENT", cmdSetEmmMaxCurrent},
    {"SET EMM LOCK LEVEL", cmdSetEmmLockLevel},
    {"SET EMM PID", cmdSetEmmPid},
    {"SET EMM AUTO SPEED", cmdSetEmmAutoSpeed},
    {"SET EMM CONFIG", cmdSetEmmConfig},
    {"SET EMM MULTI", cmdSetEmmMulti},
    {"SET EMM POSITION WINDOW", cmdSetEmmPositionWindow},
    {"SET EMM PROTECT", cmdSetEmmProtection},
    {"SET EMM HEARTBEAT", cmdSetEmmHeartbeat},
    {"SET EMM INTEGRAL", cmdSetEmmIntegral},
    {"SET EMM COLLISION ANGLE", cmdSetEmmCollisionAngle},
    {"SET EMM DMX", cmdSetEmmDmx},
    /********** EMM V5 闭环步进电机：获取电机状态/参数指令 *********  */
    {"GET EMM POS", cmdGetEmmPos},
    {"GET EMM SPEED", cmdGetEmmSpeed},
    {"GET EMM ORIGIN", cmdGetEmmOrigin},
    {"GET EMM HOME PARAM", cmdGetEmmHomeParams},//获取回零参数
    {"GET EMM VERSION", cmdGetEmmVersion},
    {"GET EMM PHASE", cmdGetEmmPhase},
    {"GET EMM BUS VOLT", cmdGetEmmBusVoltage},
    {"GET EMM PHASE CURRENT", cmdGetEmmPhaseCurrent},//读取相电流
    {"GET EMM ENCODER", cmdGetEmmEncoder},
    {"GET EMM INPUT PULSE", cmdGetEmmInputPulse},
    {"GET EMM BUS CURRENT", cmdGetEmmBusCurrent},
    {"GET EMM IO", cmdGetEmmIo},
    {"GET EMM OPTIONS", cmdGetEmmOptions},
    {"GET EMM TEMP", cmdGetEmmTemperature},
    {"GET EMM POSITION WINDOW", cmdGetEmmPositionWindow},
    {"GET EMM PROTECT", cmdGetEmmProtection},
    {"GET EMM HEARTBEAT", cmdGetEmmHeartbeat},
    {"GET EMM INTEGRAL", cmdGetEmmIntegral},
    {"GET EMM COLLISION ANGLE", cmdGetEmmCollisionAngle},
    {"GET EMM BROADCAST ADDRESS", cmdGetEmmBroadcastAddress},//
    {"GET EMM DMX", cmdGetEmmDmx},
    {"GET EMM PID", cmdGetEmmPid},
    {"GET EMM CONFIG", cmdGetEmmConfig},
    {"GET EMM FLAGS", cmdGetEmmFlags},
    {"GET EMM STATE", cmdGetEmmState},
    {"SCAN EMM", cmdScanEmm},
    // 添加更多命令
};

const size_t CMD_COUNT = sizeof(cmdTable) / sizeof(cmdTable[0]);

// ================== CommandDispatcher 实现 ==================
CommandDispatcher::CommandDispatcher() {}

const CmdEntry *CommandDispatcher::findCommand(const char *cmd)
{
    /*
     * 命令名允许出现“前缀包含”关系，例如：
     *
     *   GET EMM PHASE
     *   GET EMM PHASE CURRENT
     *
     * 旧实现按表中顺序找到第一个匹配项。这样当上位机发送
     * "GET EMM PHASE CURRENT A=2" 时，会先命中较短的
     * "GET EMM PHASE"，从而错误调用 0x20（读取相电阻/相电感），
     * 而不是调用 0x27（读取相电流）。
     *
     * 这里遍历全部命令并保存“最长匹配项”，使长命令优先于其短前缀。
     * 同时保留空格/字符串结束边界检查，避免把例如 "GET EMM POSX"
     * 错误识别成 "GET EMM POS"。
     */
    const CmdEntry *bestMatch = nullptr;
    size_t bestLength = 0;
    for (size_t i = 0; i < CMD_COUNT; i++)
    {
        const char *name = cmdTable[i].name;
        size_t len = strlen(name);
        if (len <= bestLength) {
            continue;
        }
        if (strncmp(cmd, name, len) == 0 &&
            (cmd[len] == '\0' || cmd[len] == ' ')) {
            bestMatch = &cmdTable[i];
            bestLength = len;
        }
    }
    return bestMatch;
}

bool CommandDispatcher::dispatch(const char *cmd, int seq)
{
    const CmdEntry *entry = findCommand(cmd);
    if (!entry)
        return false;
    const char *args = cmd + strlen(entry->name);
    while (*args == ' ')
        args++;

    /*
     * 所有 EMM 命令统一从这里输出“收到”和“处理结果”日志。超长 HEX 参数只打印
     * 前 120 个字符，防止原始配置/复合命令长时间占用 115200 波特率的调试串口。
     */
    const bool isEmmCommand = strstr(entry->name, "EMM") != nullptr;
    if (isEmmCommand) {
        cmd_on_line = true;
        lasCmdTime = millis();
        Serial.printf("[EMM命令] 收到 @%d %s", seq, entry->name);
        if (*args != '\0') {
            Serial.printf(" 参数=%.120s%s", args,
                          strlen(args) > 120 ? "..." : "");
        }
        Serial.println();
    }

    /*
     * 整机回零（L2 复合动作）运行期间独占 2/3/4 号 EMM 电机，此时拒绝其它
     * 会触发动作或修改电机状态的 EMM 命令，避免上位机在回零途中插入单步运动
     * 造成机械冲突。回零自身的启动/中止命令与 GET 查询命令不在拦截范围内，
     * 具体判据见 EmmSystemHome_BlocksCommand()。
     */
    if (isEmmCommand && EmmSystemHome_BlocksCommand(entry->name)) {
        Serial.printf("[EMM命令] @%d %s 被拒绝：整机回零正在运行\n",
                      seq, entry->name);
        sendWithCRC("ERR @%d %d", seq, SEQ_RUNNING);
        return false;
    }

    bool handled = entry->handler(args, seq);
    if (isEmmCommand) {
        Serial.printf("[EMM命令] @%d %s，%s\n", seq, entry->name,
                      handled ? "处理成功" : "处理失败");
    }
    return handled;
}

// ================== 保留的外设命令处理函数 ==================
static bool cmdPing(const char *args, int seq)
{
    (void)args;


    if (!ping_on_line)   // 上次ping 超时了， 这次ping 收到了，说明串口恢复了
    {
        rygLedOn(RYG_COLOR_YELLOW);
        ESP_LOGI(TAG, "PING RECOVERED");
    }


    ping_on_line = true;
    lastPingTime = millis();
    sendWithCRC("PING @%d", seq);
    return true;
}

/**
 * @brief 系统电源通断。
 * @cmd   SET POWER ON | SET POWER OFF
 * @note  KWQ3 仅控制结构电源，不再联动 XYZ/ZS/序列任务。
 *        `SET POWER ON` 合上结构电源后，会**自动触发一次系统初始化流程**
 *        （SysInit_Run）：等待上电稳定 → 前置检查（心跳/开关门/样品盘/温度）
 *        → 六通阀（暂跳过）→ 启动整机回零。初始化在后台任务中执行，
 *        本函数不等待其结束；初始化结果仅通过串口日志体现，因此无论初始化
 *        是否通过，此处都回复 `OK @<seq> POWER ON`。
 */
static bool cmdPower(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();

    if (strstr(args, "ON"))
    {
        cmd_power_off = false;
        sysPoweronoFF(1);
        sendWithCRC("OK @%d POWER ON", seq);
        /* 先回复上位机，再启动初始化，保证 ACK 不被初始化耗时拖延。 */
        SysInit_Run();
        return true;
    }
    if (strstr(args, "OFF"))
    {
        cmd_power_off = true;
        /* 断电前先广播停止所有 EMM 电机，避免结构断电时电机仍在运动。 */
        if (Rs485Bus_Take(500)) {
            gEmm.stopBroadcast(false);
            Rs485Bus_Give();
        }
        sysPoweronoFF(0);
        sendWithCRC("OK @%d POWER OFF", seq);
        return true;
    }
    sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
    return false;
}

/**
 * @brief 获取系统状态（命令保留，实现待补）。
 * @details 原 KWQ1 的 GET STATUS 返回 XYZ 坐标、序列状态与任务进度；这些数据源
 *          在 KWQ3 中已移除。此处保留命令名以兼容旧上位机，具体实现将在后续
 *          增加复合回零与任务序列时补充。
 * @todo 复合回零 / 任务序列实现完成后，在此返回 EMM 电机与外设状态。
 */
static bool cmdGetStatus(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();
    sendWithCRC("OK @%d STATUS=NA", seq);
    return true;
}
/**
 * @brief 获取驱动芯片温度
 */
static bool cmdGetTemp(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime =  millis();
    sendWithCRC("OK @%d TEMP T1=%.1f T2=%.1f T3=%.1f T4=%.1f T5=%.1f", seq, driverTemp[0], driverTemp[1], driverTemp[2], driverTemp[3], driverTemp[4]);
    return true;
}
/**
 * @brief 电磁阀开关
 */
static bool cmdSetFa(const char *args, int seq){
   (void)args; 
    cmd_on_line = true;
    lasCmdTime =  millis();
    if (strstr(args, "S=ON"))
    {
      
        setFa(1); 
        sendWithCRC("OK @%d FA ON", seq);
        return true;
    }
    if (strstr(args, "S=OFF"))
    {
  
        setFa(0); 
        sendWithCRC("OK @%d FA OFF", seq);
        return true;
    }
     sendWithCRC("ERR @%d %d", seq, PARAM_ERR); // 参数错误
    return false;
}

/**
 * @brief 设置 RYG 三色灯 PWM 电压。
 * @cmd   SET RYG V=<0..24>
 * @note  KWQ3 不做掉电保存，仅更新运行时电压。
 */
static bool cmdSetRYGvoltage(const char *args, int seq){
    uint8_t v;
    cmd_on_line = true;
    lasCmdTime = millis();
    if (sscanf(args, "V=%d", &v) < 1) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    ESP_LOGI(TAG, "set RYG voltage %d", v);
    setVoltage(v);
    sendWithCRC("OK @%d V SET", seq);
    return true;
}

/**
 * @brief 设置驱动芯片高温报警阈值。
 * @cmd   SET TEMP T=<float>
 * @note  KWQ3 不做掉电保存，仅更新运行时阈值。
 */
static bool cmdSetMaxTemp(const char *args, int seq){
    float t = 0.0;
    cmd_on_line = true;
    lasCmdTime = millis();
    if (sscanf(args, "T=%f", &t) != 1) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    max_temp = t;
    sendWithCRC("OK @%d SETTEMP", seq);
    return true;
}
/**
 * @brief 判断 SV-04M 返回值是否属于通信层错误
 */
static bool isSv04mCommError(SV04M::Status status)
{
    return status == SV04M::STATUS_TIMEOUT ||
           status == SV04M::STATUS_CHECKSUM_ERROR ||
           status == SV04M::STATUS_FRAME_FORMAT_ERR ||
           status == SV04M::STATUS_SERIAL_NOT_BEGIN;
}
/**
 * @brief 读取 GFL-Z 激光测距
 * @cmd   GET LASER ADDR=1
 * @cmd   GET LASER A=1
 * @resp  OK @seq LASER A=1  MM=71.990
 */
static bool cmdGetLaserDistance(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();

    int addr = 1;  // 默认读地址 1 的激光测距

    if (args != nullptr && args[0] != '\0') {
        const char *pAddr = strstr(args, "ADDR=");
        const char *pA = strstr(args, "A=");

        if (pAddr != nullptr) {
            if (sscanf(pAddr, "ADDR=%d", &addr) != 1) {
                sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
                return false;
            }
        } else if (pA != nullptr) {
            if (sscanf(pA, "A=%d", &addr) != 1) {
                sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
                return false;
            }
        } else {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
    }

    if (addr < 1 || addr > 247) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }

    GFLZLaser::DistanceData data;
    GFLZLaser::Status ret = gLaser.readDistance((uint8_t)addr, data, 300);

    Rs485Bus_Give();

    if (ret != GFLZLaser::STATUS_OK) {
        ESP_LOGE(TAG, "GET LASER failed, addr=%d, ret=%s", addr, GFLZLaser::statusToString(ret));
        sendWithCRC("ERR @%d LASER A=%d %s", seq, addr, GFLZLaser::statusToString(ret));
        return false;
    }

    ESP_LOGI(TAG, "GET LASER addr=%d, distance=%ld um, %.3f mm",
             addr,
             (long)data.distanceUm,
             (double)data.distanceMm);

    sendWithCRC("OK @%d LASER A=%d  MM=%.3f",
                seq,
                addr,
                (double)data.distanceMm);
    return true;
}
/**
 * @brief 控制 SV-04M 切换到指定通道，非阻塞
 * @cmd   SET VALVE CH=3
 * @cmd   SET VALVE C=3
 * @resp  OK @seq VALVE CH=3 MOVING
 * @note  本函数只发送切换指令并等待设备接收响应，不等待阀真正切换完成。
 *        切换是否完成由上位机继续发送 GET VALVE 查询。
 */
static bool cmdSetValveChannel(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();

    int channel = 0;

    if (args == nullptr || args[0] == '\0') {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    const char *pCh = strstr(args, "CH=");
    const char *pC = strstr(args, "C=");
    if (pCh != nullptr) {
        if (sscanf(pCh, "CH=%d", &channel) != 1) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
    } else if (pC != nullptr) {
        if (sscanf(pC, "C=%d", &channel) != 1) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
    } else {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    if (channel <= 0 || channel > gSv04m.channelCount()) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    /* KWQ3 暂无序列运行状态，不再做 RUNNING 门禁；后续加入任务序列时在此恢复。 */

    if (!Rs485Bus_Take(1000)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }

    /* waitDone 固定为 false：这里只等待 SV-04M 对 0x44 指令的初始响应，不等待动作完成。 */
    SV04M::Status ret = gSv04m.moveToChannel((uint8_t)channel, false, 0);

    Rs485Bus_Give();

     
    if (ret != SV04M::STATUS_OK && ret != SV04M::STATUS_TASK_PENDING) {
         // 
        ESP_LOGE(TAG, "SET VALVE failed, channel=%d, ret=%s", channel, SV04M::statusToString(ret));
        sendWithCRC("ERR @%d %d", seq, SV04M_CHANGE_FAILED);
        return false;
    }

    ESP_LOGI(TAG, "SET VALVE accepted, channel=%d, ret=%s", channel, SV04M::statusToString(ret));

    if (ret == SV04M::STATUS_TASK_PENDING) {
        sendWithCRC("OK @%d VALVE CH=%d MOVING", seq, channel);
    } else {
        sendWithCRC("OK @%d VALVE CH=%d ACCEPTED", seq, channel);
    }
    return true;
}
/**
 * @brief 查询 SV-04M 电机状态和当前通道
 * @cmd   GET VALVE
 * @resp  OK @seq VALVE STATUS=IDLE CUR=3
 * @resp  OK @seq VALVE STATUS=MOTOR_BUSY
 * @resp  OK @seq VALVE STATUS=TASK_PENDING
 */
static bool cmdGetValveStatus(const char *args, int seq)
{
    (void)args;
    cmd_on_line = true;
    lasCmdTime = millis();

    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }

    SV04M::Status status = gSv04m.queryMotorStatus();

    if (isSv04mCommError(status)) {
        Rs485Bus_Give();
        ESP_LOGE(TAG, "GET VALVE queryMotorStatus failed: %s", SV04M::statusToString(status));
        // 通信错误
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }

    if (status == SV04M::STATUS_OK) {
        uint8_t currentChannel = 0;
        SV04M::Status chRet = gSv04m.queryCurrentChannel(currentChannel);
        Rs485Bus_Give();

        if (chRet == SV04M::STATUS_OK) {
            sendWithCRC("OK @%d VALVE STATUS=IDLE CUR=%d", seq, currentChannel);
            return true;
        }

        if (isSv04mCommError(chRet)) {
            ESP_LOGE(TAG, "GET VALVE queryCurrentChannel failed: %s", SV04M::statusToString(chRet));
            sendWithCRC("ERR @%d VALVE CUR %s", seq, SV04M::statusToString(chRet));
            return false;
        }

        sendWithCRC("OK @%d VALVE STATUS=IDLE CUR_STATUS=%s", seq, SV04M::statusToString(chRet));
        return true;
    }

    Rs485Bus_Give();

    /* MOTOR_BUSY/TASK_PENDING/STALL 等属于设备状态，通信本身是成功的，所以返回 OK 并带状态。 */
    ESP_LOGI(TAG, "GET VALVE status=%s", SV04M::statusToString(status));
    sendWithCRC("OK @%d VALVE STATUS=%s", seq, SV04M::statusToString(status));
    return true;
}
/* ============================================================================
 * EMM V5 / X42S 命令层
 * ============================================================================
 *
 * 本区负责把 PC 文本命令转换成 EmmV5 协议调用。所有处理函数遵循相同流程：
 * 1. 解析 A、RPM、SAVE 等键值参数，并在访问数组/窄化类型前检查范围；
 * 2. 对会导致电机动作或修改参数的命令检查整机电源状态；
 * 3. 取得共享 RS485 总线互斥锁，保证阀头、激光传感器和 EMM 帧不会交叉；
 * 4. 调用 EmmV5 完成二进制组帧、发送、应答匹配和字段解码；
 * 5. 无论成功失败都释放互斥锁，再分别输出 PC 协议回复和人类可读串口日志。
 *
 * “ACCEPTED”只代表驱动器已经接受单播命令，或广播帧已经写入 UART，不代表
 * 机械动作完成。回零属于长动作，因此另有后台状态任务周期查询 0x3B，并输出
 * “A=n 回零中/回零完成/回零失败”。
 * ============================================================================ */

/** @brief 读取允许负数的 32 位 EMM 参数，供快速位置命令使用。 */
static bool emmGetI32(const char *args, const char *key, int32_t &value);

/**
 * @brief 从 EMM 上位机命令参数中读取无符号整数。
 * @param args 已去掉命令名称的参数字符串，例如 "A=1 RPM=100"。
 * @param key 要查找的键名，必须包含等号，例如 "RPM="。
 * @param value 输出解析结果；支持十进制和 C 语言 0x 前缀格式。
 * @return 找到键且数值可解析时返回 true，否则返回 false。
 * @note 该辅助函数按当前协议的简单键值格式工作，不负责检查数值业务范围。
 */
static bool emmGetU32(const char *args, const char *key, uint32_t &value)
{
    if (args == nullptr || key == nullptr) return false;
    const char *p = strstr(args, key);
    if (p == nullptr) return false;
    p += strlen(key);
    if (*p == '\0') return false;
    char *end = nullptr;
    unsigned long parsed = strtoul(p, &end, 0);
    if (end == p) return false;
    value = (uint32_t)parsed;
    return true;
}

/**
 * @brief 从 EMM 文本参数中读取浮点数。
 * @details 系统回零命令使用 RATIO 表示 5 号电机与 10 号电机的补偿脉冲比例，
 *          该参数可能包含小数，因此不能复用只接受整数的 emmGetU32()。
 */
static bool emmGetFloat(const char *args, const char *key, float &value)
{
    if (args == nullptr || key == nullptr) return false;
    const char *p = strstr(args, key);
    if (p == nullptr) return false;
    p += strlen(key);
    if (*p == '\0') return false;
    char *end = nullptr;
    float parsed = strtof(p, &end);
    if (end == p || !isfinite(parsed)) return false;
    value = parsed;
    return true;
}

/**
 * @brief 读取布尔型 EMM 参数。
 * @param defaultValue 参数缺省时使用的值；当前 SYNC 等可选参数默认为 false。
 * @return 参数缺省或值为 0/1 时返回 true，出现非法值时返回 false。
 */
static bool emmGetBool(const char *args, const char *key, bool &value,
                       bool defaultValue = false)
{
    uint32_t parsed = 0;
    if (args == nullptr || strstr(args, key) == nullptr) {
        value = defaultValue;
        return true;
    }
    if (!emmGetU32(args, key, parsed) || parsed > 1) return false;
    value = parsed != 0;
    return true;
}

/**
 * @brief 解析 EMM 目标地址。
 * @details 支持 A=1~255 的单播地址，以及 A=ALL 的广播目标。地址 0 不接受为
 *          单播地址，因为它在 EMM 协议中专用于广播。
 * @param all 输出是否为广播目标。
 */
static bool emmGetAddress(const char *args, uint8_t &address, bool &all)
{
    all = false;
    if (args == nullptr) return false;
    const char *p = strstr(args, "A=");
    if (p == nullptr) return false;
    p += 2;
    if (strncmp(p, "ALL", 3) == 0 && (p[3] == '\0' || p[3] == ' ')) {
        all = true;
        address = 0;
        return true;
    }
    char *end = nullptr;
    unsigned long parsed = strtoul(p, &end, 0);
    if (end == p || parsed == 0 || parsed > 255) return false;
    address = (uint8_t)parsed;
    return true;
}

/**
 * @brief 检查当前整机是否允许执行 EMM 控制动作。
 * @details 设备总电源关闭时，运动类命令统一返回 NO_STEP_PWR，避免在上位机
 *          看似成功但电机未动作。
 */
static bool emmControlAllowed(int seq)
{
    if (cmd_power_off) {
        Serial.printf("[EMM执行] @%d 拒绝执行：步进电机电源已关闭\n", seq);
        sendWithCRC("ERR @%d %d", seq, NO_STEP_PWR);
        return false;
    }
    return true;
}

/**
 * @brief 把 EmmV5::Status 转换成统一的上位机回复。
 * @details 单播和广播均使用 ACCEPTED 表示协议帧已被接受/写出；它不等价于
 *          位置运动或回零动作已经完成，完成状态需通过 GET 查询确认。
 */
static bool emmReplyStatus(int seq, const char *operation,
                           bool all, uint8_t address, EmmV5::Status status)
{
    if (status != EmmV5::STATUS_OK) {
        if (all) {
            Serial.printf("[EMM执行] A=ALL %s失败，协议状态=%s\n",
                          operation, EmmV5::statusToString(status));
        } else {
            Serial.printf("[EMM执行] A=%u %s失败，协议状态=%s\n",
                          (unsigned)address, operation,
                          EmmV5::statusToString(status));
        }
        if (all) {
            sendWithCRC("ERR @%d EMM %s A=ALL %s", seq, operation,
                        EmmV5::statusToString(status));
        } else {
            sendWithCRC("ERR @%d EMM %s A=%u %s", seq, operation,
                        (unsigned)address, EmmV5::statusToString(status));
        }
        return false;
    }
    if (all) {
        Serial.printf("[EMM执行] A=ALL %s命令已发送（广播无单机ACK）\n",
                      operation);
        sendWithCRC("OK @%d EMM %s A=ALL ACCEPTED", seq, operation);
    } else {
        Serial.printf("[EMM执行] A=%u %s命令已被电机接受\n",
                      (unsigned)address, operation);
        sendWithCRC("OK @%d EMM %s A=%u ACCEPTED", seq, operation,
                    (unsigned)address);
    }
    return true;
}

/* ---------------- EMM 回零异步状态跟踪 ----------------
 *
 * 手册 5.4.4 定义：
 *   bit2 / Org_SF = 1：正在回零；
 *   bit3 / Org_CF = 1：回零失败；
 *   bit2=0 且 bit3=0：默认空闲或回零成功。
 *
 * 最后一种状态存在歧义，所以本任务只有在“0x9A 命令已成功下发”之后才跟踪，
 * 并等待一段启动保护时间再把空闲状态判为完成。SYNC=1 的命令先标记为
 * WAIT_SYNC，收到 0xFF 同步触发后才开始计时和查询。
 */
enum EmmHomeTrackPhase : uint8_t {
    EMM_HOME_IDLE = 0,       ///< 当前地址没有需要跟踪的回零动作。
    EMM_HOME_WAIT_SYNC,      ///< 0x9A 已缓存，等待 0xFF 同步触发。
    EMM_HOME_ACTIVE          ///< 回零已触发，后台正在轮询 0x3B 状态字。
};

struct EmmHomeTrackRecord {
    EmmHomeTrackPhase phase;
    uint8_t mode;
    uint8_t consecutiveCommErrors;
    bool homingFlagSeen;
    uint32_t startedAtMs;
    uint32_t lastProgressPrintMs;
};

/** 地址 0 是广播地址，不作为单机跟踪槽使用；有效槽为 1~255。 */
static EmmHomeTrackRecord s_emmHomeTracks[256] = {};
static SemaphoreHandle_t s_emmHomeTrackMutex = nullptr;
static TaskHandle_t s_emmHomeMonitorTaskHandle = nullptr;

/** 轮询间隔兼顾状态更新速度和共享 RS485 总线占用率。 */
static const uint32_t EMM_HOME_POLL_INTERVAL_MS = 200;
/** 防止命令 ACK 后驱动器尚未来得及置位 Org_SF 就被误报为完成。 */
static const uint32_t EMM_HOME_START_GRACE_MS = 600;
/** 回零时间过长时停止后台轮询；这是监控超时，不直接等同于驱动器回零失败。 */
static const uint32_t EMM_HOME_MONITOR_TIMEOUT_MS = 10UL * 60UL * 1000UL;
/** 连续多次读不到状态时结束跟踪，避免故障设备永久占用 RS485 总线。 */
static const uint8_t EMM_HOME_MAX_COMM_ERRORS = 3;

/** @brief 把回零模式转换为便于现场诊断的中文名称。 */
static const char *emmHomeModeName(uint8_t mode)
{
    switch (mode) {
    case 0: return "单圈就近回零";
    case 1: return "单圈方向回零";
    case 2: return "无限位碰撞回零";
    case 3: return "限位回零";
    case 4: return "绝对位置回零";
    case 5: return "掉电位置回零";
    default: return "未知模式";
    }
}

/**
 * @brief 在记录仍属于同一次回零动作时将其清除。
 * @return true 表示记录成功清除；false 表示期间有新命令覆盖了旧记录。
 */
static bool emmHomeTrackClearIfCurrent(uint8_t address, uint32_t startedAtMs)
{
    if (s_emmHomeTrackMutex == nullptr ||
        xSemaphoreTake(s_emmHomeTrackMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }
    EmmHomeTrackRecord &record = s_emmHomeTracks[address];
    bool current = record.phase == EMM_HOME_ACTIVE &&
                   record.startedAtMs == startedAtMs;
    if (current) {
        record = {};
    }
    xSemaphoreGive(s_emmHomeTrackMutex);
    return current;
}

/**
 * @brief 后台读取每台活动电机的 0x3B 回零状态并输出状态变化。
 * @details 查询同样使用 Rs485Bus_Take()，不会与 PC 命令或 GPIO6 急停帧交叉。
 *          获取总线失败时本轮直接跳过，让前台命令优先完成。
 */
static void emmHomeMonitorTask(void *argument)
{
    (void)argument;

    for (;;) {
        bool hasActiveMotor = false;

        for (uint16_t rawAddress = 1; rawAddress <= 255; ++rawAddress) {
            const uint8_t address = (uint8_t)rawAddress;
            EmmHomeTrackRecord snapshot = {};

            if (xSemaphoreTake(s_emmHomeTrackMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
                continue;
            }
            snapshot = s_emmHomeTracks[address];
            xSemaphoreGive(s_emmHomeTrackMutex);

            if (snapshot.phase != EMM_HOME_ACTIVE) {
                continue;
            }
            hasActiveMotor = true;

            const uint32_t elapsedMs = millis() - snapshot.startedAtMs;
            if (elapsedMs >= EMM_HOME_MONITOR_TIMEOUT_MS) {
                if (emmHomeTrackClearIfCurrent(address, snapshot.startedAtMs)) {
                    Serial.printf("[EMM状态] A=%u 回零状态监控超时，已停止轮询（不等同于驱动器已判定失败）\n",
                                  (unsigned)address);
                }
                continue;
            }

            /* 前台正在使用总线时不等待，下一轮再查，避免状态日志阻塞控制命令。 */
            if (!Rs485Bus_Take(20)) {
                continue;
            }
            uint8_t flags = 0;
            EmmV5::Status status = gEmm.readOriginFlags(address, flags, 80);
            Rs485Bus_Give();

            if (status != EmmV5::STATUS_OK) {
                uint8_t errorCount = 0;
                bool stillCurrent = false;
                if (xSemaphoreTake(s_emmHomeTrackMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    EmmHomeTrackRecord &record = s_emmHomeTracks[address];
                    stillCurrent = record.phase == EMM_HOME_ACTIVE &&
                                   record.startedAtMs == snapshot.startedAtMs;
                    if (stillCurrent) {
                        if (record.consecutiveCommErrors < 0xFF) {
                            record.consecutiveCommErrors++;
                        }
                        errorCount = record.consecutiveCommErrors;
                        if (errorCount >= EMM_HOME_MAX_COMM_ERRORS) {
                            record = {};
                        }
                    }
                    xSemaphoreGive(s_emmHomeTrackMutex);
                }

                if (stillCurrent && errorCount == 1) {
                    Serial.printf("[EMM状态] A=%u 回零状态读取异常，状态=%s，准备重试\n",
                                  (unsigned)address,
                                  EmmV5::statusToString(status));
                } else if (stillCurrent && errorCount >= EMM_HOME_MAX_COMM_ERRORS) {
                    Serial.printf("[EMM状态] A=%u 回零状态监控失败：连续%u次通信异常，最后状态=%s\n",
                                  (unsigned)address,
                                  (unsigned)EMM_HOME_MAX_COMM_ERRORS,
                                  EmmV5::statusToString(status));
                }
                continue;
            }

            /* 通信恢复后清零连续错误计数，并确认记录没有被新命令替换。 */
            bool stillCurrent = false;
            if (xSemaphoreTake(s_emmHomeTrackMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                EmmHomeTrackRecord &record = s_emmHomeTracks[address];
                stillCurrent = record.phase == EMM_HOME_ACTIVE &&
                               record.startedAtMs == snapshot.startedAtMs;
                if (stillCurrent) {
                    record.consecutiveCommErrors = 0;
                }
                xSemaphoreGive(s_emmHomeTrackMutex);
            }
            if (!stillCurrent) {
                continue;
            }

            /* Org_CF、过热或过流任一出现，都输出明确失败原因并结束本次跟踪。 */
            const bool homeFailed = (flags & EmmV5::ORIGIN_FAILED) != 0;
            const bool overheat = (flags & EmmV5::ORIGIN_OVERHEAT) != 0;
            const bool overcurrent = (flags & EmmV5::ORIGIN_OVERCURRENT) != 0;
            if (homeFailed || overheat || overcurrent) {
                if (emmHomeTrackClearIfCurrent(address, snapshot.startedAtMs)) {
                    const char *reason = homeFailed ? "驱动器回零失败标志" :
                                         overheat ? "驱动器过热保护" :
                                                    "驱动器过流保护";
                    Serial.printf("[EMM状态] A=%u 回零失败，原因=%s，FLAGS=0x%02X\n",
                                  (unsigned)address, reason, flags);
                }
                continue;
            }

            if ((flags & EmmV5::ORIGIN_HOMING) != 0) {
                bool printProgress = false;
                if (xSemaphoreTake(s_emmHomeTrackMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    EmmHomeTrackRecord &record = s_emmHomeTracks[address];
                    if (record.phase == EMM_HOME_ACTIVE &&
                        record.startedAtMs == snapshot.startedAtMs) {
                        record.homingFlagSeen = true;
                        if ((uint32_t)(millis() - record.lastProgressPrintMs) >= 5000) {
                            record.lastProgressPrintMs = millis();
                            printProgress = true;
                        }
                    }
                    xSemaphoreGive(s_emmHomeTrackMutex);
                }
                if (printProgress) {
                    Serial.printf("[EMM状态] A=%u 回零中，已运行%lu ms，FLAGS=0x%02X\n",
                                  (unsigned)address, (unsigned long)elapsedMs,
                                  flags);
                }
                continue;
            }

            /*
             * bit2/bit3 均为 0：若此前已经观察到 Org_SF=1，说明状态确实从
             * “回零中”切回空闲，可以立即判定完成；若动作非常快而未观察到
             * Org_SF，则必须经过启动保护时间后才判定，避免 ACK 后立刻误报。
             */
            if ((snapshot.homingFlagSeen || elapsedMs >= EMM_HOME_START_GRACE_MS) &&
                emmHomeTrackClearIfCurrent(address, snapshot.startedAtMs)) {
                Serial.printf("[EMM状态] A=%u 回零完成，耗时%lu ms，FLAGS=0x%02X\n",
                              (unsigned)address, (unsigned long)elapsedMs,
                              flags);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(hasActiveMotor ? EMM_HOME_POLL_INTERVAL_MS : 500));
    }
}

/** @brief 按需创建回零跟踪互斥锁和后台任务。 */
static bool emmEnsureHomeMonitorTask(void)
{
    if (s_emmHomeMonitorTaskHandle != nullptr) {
        return true;
    }
    if (s_emmHomeTrackMutex == nullptr) {
        s_emmHomeTrackMutex = xSemaphoreCreateMutex();
    }
    if (s_emmHomeTrackMutex == nullptr) {
        Serial.println("[EMM状态] 回零监控启动失败：无法创建状态互斥锁");
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        emmHomeMonitorTask, "EmmHomeMonitor", 4096, nullptr, 1,
        &s_emmHomeMonitorTaskHandle, 0);
    if (result != pdPASS) {
        s_emmHomeMonitorTaskHandle = nullptr;
        Serial.println("[EMM状态] 回零监控启动失败：无法创建后台任务");
        return false;
    }
    Serial.println("[EMM状态] 回零状态监控任务已启动");
    return true;
}

/** @brief 登记一条已被驱动器接受的单机回零命令。 */
static void emmHomeTrackStart(uint8_t address, uint8_t mode, bool waitForSync)
{
    if (!emmEnsureHomeMonitorTask()) {
        Serial.printf("[EMM状态] A=%u 回零命令已接受，但无法自动跟踪完成状态\n",
                      (unsigned)address);
        return;
    }

    EmmHomeTrackRecord record = {};
    record.phase = waitForSync ? EMM_HOME_WAIT_SYNC : EMM_HOME_ACTIVE;
    record.mode = mode;
    record.startedAtMs = millis();
    record.lastProgressPrintMs = millis();

    bool registered = false;
    if (xSemaphoreTake(s_emmHomeTrackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_emmHomeTracks[address] = record;
        xSemaphoreGive(s_emmHomeTrackMutex);
        registered = true;
    }
    if (!registered) {
        Serial.printf("[EMM状态] A=%u 回零命令已接受，但状态记录失败，无法自动跟踪完成状态\n",
                      (unsigned)address);
        return;
    }

    if (waitForSync) {
        Serial.printf("[EMM状态] A=%u 回零命令已缓存，MODE=%u（%s），等待同步触发\n",
                      (unsigned)address, (unsigned)mode,
                      emmHomeModeName(mode));
    } else {
        Serial.printf("[EMM状态] A=%u 回零中，MODE=%u（%s）\n",
                      (unsigned)address, (unsigned)mode,
                      emmHomeModeName(mode));
    }
}

/** @brief 同步触发后，把此前缓存的单机回零记录切换为活动状态。 */
static void emmHomeTrackSynchronized(uint8_t address, bool all)
{
    if (s_emmHomeTrackMutex == nullptr) return;
    const uint16_t first = all ? 1 : address;
    const uint16_t last = all ? 255 : address;

    for (uint16_t rawAddress = first; rawAddress <= last; ++rawAddress) {
        bool started = false;
        uint8_t mode = 0;
        if (xSemaphoreTake(s_emmHomeTrackMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            EmmHomeTrackRecord &record = s_emmHomeTracks[rawAddress];
            if (record.phase == EMM_HOME_WAIT_SYNC) {
                record.phase = EMM_HOME_ACTIVE;
                record.startedAtMs = millis();
                record.lastProgressPrintMs = millis();
                record.consecutiveCommErrors = 0;
                record.homingFlagSeen = false;
                mode = record.mode;
                started = true;
            }
            xSemaphoreGive(s_emmHomeTrackMutex);
        }
        if (started) {
            Serial.printf("[EMM状态] A=%u 同步触发成功，回零中，MODE=%u（%s）\n",
                          (unsigned)rawAddress, (unsigned)mode,
                          emmHomeModeName(mode));
        }
    }
}

/** @brief 回零中断命令成功后取消对应地址的状态跟踪。 */
static void emmHomeTrackCancel(uint8_t address, bool all)
{
    if (s_emmHomeTrackMutex == nullptr) return;
    const uint16_t first = all ? 1 : address;
    const uint16_t last = all ? 255 : address;

    for (uint16_t rawAddress = first; rawAddress <= last; ++rawAddress) {
        bool cancelled = false;
        if (xSemaphoreTake(s_emmHomeTrackMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            EmmHomeTrackRecord &record = s_emmHomeTracks[rawAddress];
            cancelled = record.phase != EMM_HOME_IDLE;
            record = {};
            xSemaphoreGive(s_emmHomeTrackMutex);
        }
        if (cancelled) {
            Serial.printf("[EMM状态] A=%u 回零已被中断，状态跟踪已取消\n",
                          (unsigned)rawAddress);
        }
    }
}

/* ==========================================================================
 * 升降电机（3 号）相电流轮询监控 —— 电离头扎偏保护
 * --------------------------------------------------------------------------
 * 该后台任务默认处于“撤防”状态并阻塞等待布防通知；只有收到布防命令
 * （SET EMM LIFT MONITOR ARM=1）后才开始快速轮询 3 号升降电机相电流，
 * 工作流程如下：
 *
 *   1. 撤防时任务阻塞等待通知，不占用 CPU 与 RS485 总线；
 *   2. 布防后按 EMM_LIFT_CURRENT_POLL_MS 快速轮询相电流；
 *   3. 相电流超过安全阈值、且持续时间达到 EMM_LIFT_IONIZATION_CONFIRM_MS，
 *      才判定为“电离头扎偏”故障（短暂尖峰不会误触发）；
 *   4. 触发后立即通过串口输出警告信号（先警告）；
 *   5. 随后让升降电机以相对位置模式上升固定脉冲数到安全高度（再上升）；
 *   6. 等待固定时长后，广播停止所有 EMM 电机（最后停止）；
 *   7. 收尾输出告警文本，并自动撤防，等待下一次布防命令。
 *
 * 该任务在布防期间只读取电流、下发两条命令，属于轻量轮询；优先级与回零监控
 * 任务一致（1）并固定到 Core 0，不会抢占实时运动/回零任务。
 *
 * 下列宏集中管理本保护功能的全部可调参数（电机地址、阈值、脉冲数、速度、
 * 加速度、等待时间等），逻辑代码中不出现魔数，便于现场按机械结构统一调整。
 * ========================================================================== */

/** 被监控的 EMM 电机地址：3 号升降电机（复用 Rs485Devices.h 的统一地址宏）。 */
#define EMM_LIFT_CURRENT_MONITOR_ADDR   EMM_LIFT_MOTOR

/** 电离头扎偏电流安全阈值，单位 mA；相电流超过该值即判定为故障。 */
#define EMM_LIFT_IONIZATION_CURRENT_MA  460U

/**
 * 布防期间的电流轮询周期，单位 ms。
 * @details 只有布防后才按该周期快速轮询；撤防时任务阻塞，不占用 CPU 与总线。
 *          1ms 与单次读电流事务耗时相当，等价于“连续快速采样”。
 */
#define EMM_LIFT_CURRENT_POLL_MS        1U

/**
 * 阻碍持续时长阈值，单位 ms。
 * @details 相电流连续超过阈值、且持续时间达到该值，才判定为电离头扎偏，
 *          用于滤除扎入瞬间的电流尖峰。
 */
#define EMM_LIFT_IONIZATION_CONFIRM_MS  20U

/** 故障后升降电机相对上升的脉冲数（对应“上升到安全高度”）。 */
#define EMM_LIFT_SAFE_UP_PULSES         10000UL

/** 上升运动转速，单位 RPM。 */
#define EMM_LIFT_SAFE_UP_RPM            180U

/** 上升运动加速度字段（EMM 协议取值范围 0~255）。 */
#define EMM_LIFT_SAFE_UP_ACC            40U

/** 上升方向：手册约定升降电机 CW(0)=上升、CCW(1)=下降。 */
#define EMM_LIFT_SAFE_UP_DIR            0U

/** 发送上升命令后等待的时长，单位 ms；到点后立即广播停止所有 EMM 电机。 */
#define EMM_LIFT_SAFE_UP_HOLD_MS        3000U

/** 单条 EMM 命令等待驱动器 ACK 的超时，单位 ms。 */
#define EMM_LIFT_CMD_TIMEOUT_MS         150U

/** 获取共享 RS485 总线的最长等待时间，单位 ms；超时则本轮跳过。 */
#define EMM_LIFT_BUS_TIMEOUT_MS         20U

/** 触发瞬间立即输出的警告文本（此时尚未执行上升/停止动作）。 */
#define EMM_LIFT_FAULT_WARNING \
    "[警告]检测到电离头扎偏，正在上升至安全高度并停止所有电机"

/** 完成保护动作后输出的告警文本（此时所有 EMM 电机已停止）。 */
#define EMM_LIFT_FAULT_ALERT \
    "[警告]请检查样品位置是否正确，已停止所有电机"

/** 升降电机电流监控后台任务句柄。 */
static TaskHandle_t s_emmLiftCurrentMonitorTaskHandle = nullptr;

/**
 * 本次“连续超阈值”计时的起点，单位 ms；0 表示当前未处于连续超阈值状态。
 * @details 相电流连续超过阈值且持续时间达到 EMM_LIFT_IONIZATION_CONFIRM_MS
 *          才触发保护；中途电流回落或读取失败都会把该值清零。
 */
static uint32_t s_emmLiftOverCurrentSince = 0;

/**
 * 升降电机电流监控的布防标志。
 * @details true 表示正在快速轮询检测；false 表示撤防（任务阻塞等待布防通知）。
 *          由上位的 SET EMM LIFT MONITOR ARM=<0|1> 命令设置，故障处理完成后
 *          自动清零，避免快速段大电流再次误触发。
 */
static volatile bool s_emmLiftMonitorArmed = false;

/**
 * @brief 执行“电离头扎偏”故障保护动作。
 * @details 动作顺序严格为“先警告、再上升、最后停止”：
 *          1) 立即串口输出警告信号；
 *          2) 升降电机相对上升至安全高度；
 *          3) 等待固定时长后广播停止所有 EMM 电机；
 *          4) 收尾输出告警文本。
 *          该函数在监控任务上下文同步执行，执行期间暂停轮询，避免保护动作
 *          尚未完成就重复触发。
 * @param currentMa 触发本次故障时读到的相电流，单位 mA，仅用于日志。
 */
static void emmLiftHandleIonizationFault(uint16_t currentMa)
{
    /* ---- 第一步（先警告）：在机械动作之前立即输出警告信号。 ---- */
    Serial.printf("[EMM监控] 升降电机A=%u 相电流=%u mA 持续≥%u ms 超过阈值%u mA，判定为电离头扎偏\n",
                  (unsigned)EMM_LIFT_CURRENT_MONITOR_ADDR,
                  (unsigned)currentMa,
                  (unsigned)EMM_LIFT_IONIZATION_CONFIRM_MS,
                  (unsigned)EMM_LIFT_IONIZATION_CURRENT_MA);
    Serial.println(EMM_LIFT_FAULT_WARNING);

    /* ---- 第二步（再上升）：以相对位置模式（MODE=0）上升至安全高度。 ---- */
    EmmV5::Status moveStatus = EmmV5::STATUS_TIMEOUT;
    if (Rs485Bus_Take(EMM_LIFT_CMD_TIMEOUT_MS)) {
        moveStatus = gEmm.positionMode(
            EMM_LIFT_CURRENT_MONITOR_ADDR,
            (uint8_t)EMM_LIFT_SAFE_UP_DIR,
            (uint16_t)EMM_LIFT_SAFE_UP_RPM,
            (uint8_t)EMM_LIFT_SAFE_UP_ACC,
            (uint32_t)EMM_LIFT_SAFE_UP_PULSES,
            0,                     /* MODE=0：相对位置模式 */
            false,                 /* 不使用同步触发，立即执行 */
            EMM_LIFT_CMD_TIMEOUT_MS);
        Rs485Bus_Give();
    } else {
        Serial.println("[EMM监控] RS485总线繁忙，升降电机上升命令未能下发");
    }

    if (moveStatus == EmmV5::STATUS_OK) {
        Serial.printf("[EMM监控] 升降电机已启动上升：DIR=%u RPM=%u ACC=%u PULSE=%lu\n",
                      (unsigned)EMM_LIFT_SAFE_UP_DIR,
                      (unsigned)EMM_LIFT_SAFE_UP_RPM,
                      (unsigned)EMM_LIFT_SAFE_UP_ACC,
                      (unsigned long)EMM_LIFT_SAFE_UP_PULSES);
    } else {
        Serial.printf("[EMM监控] 升降电机上升命令未确认，状态=%s\n",
                      EmmV5::statusToString(moveStatus));
    }

    /* ---- 第三步（等待）：等待固定时长，让升降动作运行到接近安全高度。 ---- */
    vTaskDelay(pdMS_TO_TICKS(EMM_LIFT_SAFE_UP_HOLD_MS));

    /* ---- 第四步（最后停止）：广播停止所有 EMM 电机（地址 0 为广播地址）。 ---- */
    if (Rs485Bus_Take(EMM_LIFT_CMD_TIMEOUT_MS)) {
        EmmV5::Status stopStatus = gEmm.stopBroadcast(false);
        Rs485Bus_Give();
        if (stopStatus != EmmV5::STATUS_OK) {
            Serial.printf("[EMM监控] 停止所有 EMM 电机的命令未确认，状态=%s\n",
                          EmmV5::statusToString(stopStatus));
        }
    } else {
        Serial.println("[EMM监控] RS485总线繁忙，停止所有 EMM 电机的命令未能下发");
    }

    /* ---- 第五步（收尾提示）：停止完成后输出“请检查样品位置”告警文本。 ---- */
    Serial.println(EMM_LIFT_FAULT_ALERT);
}

/**
 * @brief 升降电机相电流轮询监控任务。
 * @details 常驻后台。默认撤防并阻塞等待布防通知；布防后按
 *          EMM_LIFT_CURRENT_POLL_MS 周期读取 3 号升降电机的相电流（功能码
 *          0x27）。只有相电流连续超过阈值、且持续时间达到
 *          EMM_LIFT_IONIZATION_CONFIRM_MS 才判定为电离头扎偏；读取失败或电流
 *          回落都会清零计时，避免瞬时尖峰与通信异常造成误触发。触发保护后
 *          自动撤防，等待下一次布防命令。
 */
static void emmLiftCurrentMonitorTask(void *argument)
{
    (void)argument;

    for (;;) {
        /* 默认撤防：阻塞等待布防通知，期间不占用 CPU 与 RS485 总线。 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        s_emmLiftOverCurrentSince = 0;

        /* 布防期间持续快速轮询相电流，直到收到撤防或判定故障。 */
        while (s_emmLiftMonitorArmed) {
            /* 1ms 在 100Hz tick 下会被截断为 0，这里兜底至少延时 1 tick。 */
            const TickType_t pollTicks = pdMS_TO_TICKS(EMM_LIFT_CURRENT_POLL_MS);
            vTaskDelay(pollTicks > 0 ? pollTicks : 1);

            /* 延时期间可能收到撤防命令，立即退出轮询。 */
            if (!s_emmLiftMonitorArmed) {
                break;
            }

            /* 前台命令或其它监控正在使用总线时本轮跳过，保证控制指令优先。 */
            if (!Rs485Bus_Take(EMM_LIFT_BUS_TIMEOUT_MS)) {
                continue;
            }
            uint16_t currentMa = 0;
            EmmV5::Status status = gEmm.readPhaseCurrent(
                EMM_LIFT_CURRENT_MONITOR_ADDR, currentMa, EMM_LIFT_CMD_TIMEOUT_MS);
            Rs485Bus_Give();

            /*
             * 判定“阻碍持续超过 EMM_LIFT_IONIZATION_CONFIRM_MS”：
             *   - 读取失败或电流回落到阈值以内，说明阻碍未持续，清零计时；
             *   - 电流超阈值时记录首次超阈值时刻，持续时间达标才真正触发。
             * 该逻辑天然滤除扎入瞬间的电流尖峰与偶发通信异常。
             */
            if (status != EmmV5::STATUS_OK ||
                currentMa <= (uint16_t)EMM_LIFT_IONIZATION_CURRENT_MA) {
                s_emmLiftOverCurrentSince = 0;
                continue;
            }
            if (s_emmLiftOverCurrentSince == 0) {
                s_emmLiftOverCurrentSince = millis();
                continue;
            }
            if ((uint32_t)(millis() - s_emmLiftOverCurrentSince) <
                (uint32_t)EMM_LIFT_IONIZATION_CONFIRM_MS) {
                continue;
            }

            /*
             * 阻碍已持续足够时长：先撤防（防止保护动作执行期间重复触发），
             * 再执行电离头扎偏保护动作；处理完成后退出轮询回到阻塞状态。
             */
            s_emmLiftOverCurrentSince = 0;
            s_emmLiftMonitorArmed = false;
            emmLiftHandleIonizationFault(currentMa);
        }
    }
}

/**
 * @brief 布防/撤防升降电机电流监控。
 * @details ARM=1 时置位布防标志并唤醒监控任务开始快速轮询；ARM=0 时清除标志，
 *          监控任务在当轮延时结束后退出轮询并重新阻塞。调试阶段由上位机显式
 *          调用，只在慢速下降段布防，避免快速下降的大电流造成误触发。
 * @param enable true 布防开始检测；false 撤防停止检测。
 */
void EmmLiftCurrentMonitor_Arm(bool enable)
{
    s_emmLiftMonitorArmed = enable;
    if (s_emmLiftCurrentMonitorTaskHandle != nullptr) {
        /* 唤醒可能正阻塞等待布防通知的监控任务；撤防时任务在轮询循环内自检。 */
        xTaskNotifyGive(s_emmLiftCurrentMonitorTaskHandle);
    }
    Serial.printf("[EMM监控] 升降电机电流监控%s：A=%u 阈值=%u mA 持续≥%u ms\n",
                  enable ? "已布防" : "已撤防",
                  (unsigned)EMM_LIFT_CURRENT_MONITOR_ADDR,
                  (unsigned)EMM_LIFT_IONIZATION_CURRENT_MA,
                  (unsigned)EMM_LIFT_IONIZATION_CONFIRM_MS);
}

/**
 * @brief 创建升降电机相电流轮询监控后台任务。
 * @details 必须在 Rs485Devices_Init() 之后调用，确保共享 RS485 总线、EMM 驱动
 *          对象与总线互斥锁均已就绪。重复调用安全。
 * @return true 任务已启动或早已运行；false 任务创建失败。
 */
bool EmmLiftCurrentMonitor_Init(void)
{
    if (s_emmLiftCurrentMonitorTaskHandle != nullptr) {
        return true;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        emmLiftCurrentMonitorTask,
        "EmmLiftCurrentMonitor",
        3072,
        nullptr,
        1,                        /* 低优先级，避免抢占运动/回零任务 */
        &s_emmLiftCurrentMonitorTaskHandle,
        0);                       /* 与回零监控同核，减少跨核干扰 */
    if (result != pdPASS) {
        s_emmLiftCurrentMonitorTaskHandle = nullptr;
        Serial.println("[EMM监控] 升降电机电流监控启动失败：无法创建后台任务");
        return false;
    }

    Serial.printf("[EMM监控] 升降电机电流监控已启动：A=%u 阈值=%u mA 周期=%u ms\n",
                  (unsigned)EMM_LIFT_CURRENT_MONITOR_ADDR,
                  (unsigned)EMM_LIFT_IONIZATION_CURRENT_MA,
                  (unsigned)EMM_LIFT_CURRENT_POLL_MS);
    return true;
}
/**
 * @brief 处理 SET EMM ENABLE。
 * @cmd SET EMM ENABLE A=<addr|ALL> EN=<0|1> [SYNC=<0|1>]
 * @details 对单播目标等待 ACK；A=ALL 时发送地址 0 广播帧且不等待响应。
 */
static bool cmdSetEmmEnable(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    bool enabled = false;
    bool sync = false;
    uint32_t raw = 0;
    if (!emmGetAddress(args, address, all) ||
        !emmGetU32(args, "EN=", raw) || raw > 1 ||
        !emmGetBool(args, "SYNC=", sync)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    enabled = raw != 0;
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = all ? gEmm.enableBroadcast(enabled, sync)
                            : gEmm.enable(address, enabled, sync);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "ENABLE", all, address, ret);
}

/**
 * @brief 处理 SET EMM SPEED（手册 5.3.7，功能码 0xF6）。
 * @cmd SET EMM SPEED A=<addr|ALL> DIR=<0|1> RPM=<0..3000> ACC=<0..255> [SYNC=<0|1>]
 */
static bool cmdSetEmmSpeed(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    bool sync = false;
    uint32_t dir = 0, rpm = 0, acc = 0;
    if (!emmGetAddress(args, address, all) || !emmGetU32(args, "DIR=", dir) ||
        !emmGetU32(args, "RPM=", rpm) || !emmGetU32(args, "ACC=", acc) ||
        !emmGetBool(args, "SYNC=", sync) || dir > 1 || rpm > 3000 || acc > 255) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = all
        ? gEmm.velocityBroadcast((uint8_t)dir, (uint16_t)rpm, (uint8_t)acc, sync)
        : gEmm.velocity(address, (uint8_t)dir, (uint16_t)rpm, (uint8_t)acc, sync);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "SPEED", all, address, ret);
}

/**
 * @brief 处理 SET EMM MOVE（手册 5.3.12，功能码 0xFD）。
 * @cmd SET EMM MOVE A=<addr|ALL> DIR=<0|1> RPM=<...> ACC=<...> PULSE=<u32> [MODE=<0..2>|ABS=<0|1>] [SYNC=<0|1>]
 * @note MODE 是新接口；ABS 为旧接口兼容别名，二者同时出现时优先使用 MODE。
 */
static bool cmdSetEmmMove(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    bool sync = false;
    uint32_t dir = 0, rpm = 0, acc = 0, pulses = 0, mode = 0, absFlag = 0;
    if (!emmGetAddress(args, address, all) || !emmGetU32(args, "DIR=", dir) ||
        !emmGetU32(args, "RPM=", rpm) || !emmGetU32(args, "ACC=", acc) ||
        !emmGetU32(args, "PULSE=", pulses) ||
        !emmGetBool(args, "SYNC=", sync) || dir > 1 || rpm > 3000 || acc > 255) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (strstr(args, "MODE=") != nullptr) {
        if (!emmGetU32(args, "MODE=", mode) || mode > 2) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
    } else if (strstr(args, "ABS=") != nullptr) {
        if (!emmGetU32(args, "ABS=", absFlag) || absFlag > 1) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
        mode = absFlag;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = all
        ? gEmm.positionBroadcastMode((uint8_t)dir, (uint16_t)rpm,
                                      (uint8_t)acc, pulses, (uint8_t)mode, sync)
        : gEmm.positionMode(address, (uint8_t)dir, (uint16_t)rpm,
                             (uint8_t)acc, pulses, (uint8_t)mode, sync);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "MOVE", all, address, ret);
}

/**
 * @brief 处理 SET EMM STOP（功能码 0xFE/0x98）。
 * @cmd SET EMM STOP A=<addr|ALL> [SYNC=<0|1>]
 */
static bool cmdSetEmmStop(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    bool sync = false;
    if (!emmGetAddress(args, address, all) || !emmGetBool(args, "SYNC=", sync)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = all ? gEmm.stopBroadcast(sync) : gEmm.stop(address, sync);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "STOP", all, address, ret);
}

/**
 * @brief 处理 SET EMM LIFT MONITOR（升降电机扎偏电流检测的布防/撤防）。
 * @cmd SET EMM LIFT MONITOR ARM=<0|1>
 * @details 该命令只切换监控任务的本地状态，不产生 RS485 总线事务，因此不经过
 *          emmControlAllowed() 门禁，可在调试阶段随时布防/撤防。
 */
static bool cmdSetEmmLiftMonitor(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();

    uint32_t arm = 0;
    if (!emmGetU32(args, "ARM=", arm) || arm > 1) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    EmmLiftCurrentMonitor_Arm(arm != 0);
    sendWithCRC("OK @%d EMM LIFT MONITOR ARM=%u", seq, (unsigned)arm);
    return true;
}

/**
 * @brief 处理 SET EMM HOME（手册 5.4.2，功能码 0x9A）。
 * @cmd SET EMM HOME A=<addr|ALL> MODE=<0..5> [SYNC=<0|1>]
 */
static bool cmdSetEmmHome(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    bool sync = false;
    uint32_t mode = 0;
    if (!emmGetAddress(args, address, all) || !emmGetU32(args, "MODE=", mode) ||
        !emmGetBool(args, "SYNC=", sync) || mode > 5) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = all ? gEmm.homeBroadcast((uint8_t)mode, sync)
                            : gEmm.home(address, (uint8_t)mode, sync);
    Rs485Bus_Give();
    bool replied = emmReplyStatus(seq, "HOME", all, address, ret);
    if (!replied) {
        return false;
    }

    if (all) {
        /*
         * 广播帧不会收到单机 ACK，也无法在不知道在线地址的情况下安全查询
         * 每台电机，所以这里只如实报告“广播已开始”，不伪造逐台完成结果。
         */
        Serial.printf("[EMM状态] A=ALL %s，MODE=%u（%s）；广播模式无法逐台确认完成状态\n",
                      sync ? "回零命令已缓存，等待同步触发" : "回零中",
                      (unsigned)mode, emmHomeModeName((uint8_t)mode));
    } else {
        emmHomeTrackStart(address, (uint8_t)mode, sync);
    }
    return true;
}

/**
 * @brief 处理 5.4.6 回零参数修改命令。
 * @cmd SET EMM HOME PARAM A=1 SAVE=1 MODE=0 DIR=0 RPM=30 TIMEOUT=10000
 *      STALL_RPM=300 STALL_MA=800 STALL_MS=60 AUTO=0
 * @details 该命令只能针对单个地址执行；TIMEOUT 使用毫秒，STALL_MA 使用毫安，
 *          其余字段直接对应 X42S 手册 0x4C/0xAE 帧中的字段。
 * @return ACCEPTED 仅表示驱动器接受了参数写入请求，是否掉电保存由 SAVE 决定。
 */
static bool cmdSetEmmHomeParams(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;

    uint8_t address = 0;
    bool all = false;
    bool save = false;
    bool autoHome = false;
    uint32_t mode = 0, direction = 0, rpm = 0, homeTimeout = 0;
    uint32_t stallRpm = 0, stallMa = 0, stallMs = 0;
    if (!emmGetAddress(args, address, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "MODE=", mode) ||
        !emmGetU32(args, "DIR=", direction) ||
        !emmGetU32(args, "RPM=", rpm) ||
        !emmGetU32(args, "TIMEOUT=", homeTimeout) ||
        !emmGetU32(args, "STALL_RPM=", stallRpm) ||
        !emmGetU32(args, "STALL_MA=", stallMa) ||
        !emmGetU32(args, "STALL_MS=", stallMs) ||
        !emmGetBool(args, "AUTO=", autoHome) || mode > 5 || direction > 1 ||
        rpm > 3000 || stallRpm > 3000 || stallMa > 5000 || stallMs > 65535) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.modifyHomeParams(
        address, save, (uint8_t)mode, (uint8_t)direction, (uint16_t)rpm,
        homeTimeout, (uint16_t)stallRpm, (uint16_t)stallMa, (uint16_t)stallMs,
        autoHome);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "HOME_PARAM", false, address, ret);
}

/**
 * @brief 处理 SET EMM HOME STOP（手册 5.4.3，功能码 0x9C/0x48）。
 * @cmd SET EMM HOME STOP A=<addr|ALL>
 * @note 该命令只中断回零流程；单播等待 ACK，广播只确认帧已写入总线。
 */
static bool cmdSetEmmHomeStop(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    if (!emmGetAddress(args, address, all)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = all ? gEmm.interruptHomeBroadcast()
                            : gEmm.interruptHome(address);
    Rs485Bus_Give();
    bool replied = emmReplyStatus(seq, "HOME_STOP", all, address, ret);
    if (replied) {
        emmHomeTrackCancel(address, all);
        if (all) {
            Serial.println("[EMM状态] A=ALL 已广播强制退出回零命令");
        }
    }
    return replied;
}

/**
 * @brief 处理 SET EMM ZERO（功能码 0x0A/0x6D）。
 * @cmd SET EMM ZERO A=<addr|ALL>
 * @details 该命令清零当前位置计数，不等同于 5.4.1 的单圈回零零点设置。
 */
static bool cmdSetEmmZero(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    if (!emmGetAddress(args, address, all)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = all ? gEmm.resetCurrentPositionBroadcast()
                            : gEmm.resetCurrentPosition(address);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "ZERO", all, address, ret);
}

/**
 * @brief 处理 SET EMM SYNC（功能码 0xFF/0x66）。
 * @cmd SET EMM SYNC A=<addr|ALL>
 * @details 用于触发此前以 SYNC=1 下发并暂存的运动、停止或回零命令。
 */
static bool cmdSetEmmSync(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    if (!emmGetAddress(args, address, all)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = all ? gEmm.synchronousMotionBroadcast()
                            : gEmm.synchronousMotion(address);
    Rs485Bus_Give();
    bool replied = emmReplyStatus(seq, "SYNC", all, address, ret);
    if (replied) {
        emmHomeTrackSynchronized(address, all);
    }
    return replied;
}

/**
 * @brief 处理 SET EMM ORIGIN ZERO（手册 5.4.1，功能码 0x93/0x88）。
 * @cmd SET EMM ORIGIN ZERO A=<addr> SAVE=<0|1>
 * @note 手册定义该功能为单电机操作，因此拒绝 A=ALL 广播。
 */
static bool cmdSetEmmOriginZero(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    bool save = false;
    if (!emmGetAddress(args, address, all) || all ||
        !emmGetBool(args, "SAVE=", save)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.setOriginZero(address, save);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "ORIGIN_ZERO", false, address, ret);
}

/**
 * @brief 处理 SET EMM RESET PROTECT（功能码 0x0E/0x52）。
 * @cmd SET EMM RESET PROTECT A=<addr>
 * @note 该命令用于清除保护状态，属于单地址维护操作，不发送广播。
 */
static bool cmdSetEmmResetProtect(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    if (!emmControlAllowed(seq)) return false;
    uint8_t address = 0;
    bool all = false;
    if (!emmGetAddress(args, address, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.resetClogProtection(address);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "RESET_PROTECT", false, address, ret);
}

/* ---------------- 编码器维护、驱动器维护与快速运动 ----------------
 * 这些命令均为单播操作，需要驱动器返回 4 字节 ACK。校准、重启和恢复出厂
 * 设置可能让电机短时离线或运动；ACK 只表示请求被接受，不能当作全过程完成。
 */

/**
 * @brief 处理编码器线性化校准命令（手册 5.2.1，0x06/0x45）。
 * @cmd SET EMM CALIBRATE A=<addr>
 * @details 校准期间电机可能旋转，只允许单机执行。成功返回后串口明确提示校准
 *          已启动；最终 Cal_Rdy 可通过 GET EMM ORIGIN 查询。
 */
static bool cmdSetEmmCalibrate(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.calibrateEncoder(a);
    Rs485Bus_Give();
    bool replied = emmReplyStatus(seq, "CALIBRATE", false, a, r);
    if (replied) {
        Serial.printf("[EMM状态] A=%u 编码器校准已启动，完成状态请查询 GET EMM ORIGIN\n",
                      (unsigned)a);
    }
    return replied;
}

/**
 * @brief 处理驱动器重启命令（手册 5.2.2，0x08/0x97）。
 * @cmd SET EMM RESTART A=<addr>
 * @note ACK 返回后驱动器会离线并重新启动，紧接着发送的查询可能暂时超时。
 */
static bool cmdSetEmmRestart(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.restart(a);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "RESTART", false, a, r);
}

/**
 * @brief 处理恢复出厂设置命令（手册 5.2.5，0x0F/0x5F）。
 * @cmd SET EMM FACTORY A=<addr>
 * @warning 会改变地址、运动和保护参数，执行后通常需要重新上电并校准编码器。
 */
static bool cmdSetEmmFactory(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.restoreFactory(a);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "FACTORY", false, a, r);
}

/**
 * @brief 设置快速位置模式公共参数（手册 5.3.13，0xF1）。
 * @cmd SET EMM FAST PARAM A=<addr> RPM=<0..3000> ACC=<0..255>
 *      MODE=<0..2> [SYNC=<0|1>]
 * @details 该步骤只缓存速度、加速度和模式，真正运动由后续 FAST MOVE 触发。
 */
static bool cmdSetEmmFastParam(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool sync = false;
    uint32_t rpm = 0, acc = 0, mode = 0;
    if (!emmGetAddress(args, a, all) || all || !emmGetU32(args, "RPM=", rpm) ||
        !emmGetU32(args, "ACC=", acc) || !emmGetU32(args, "MODE=", mode) ||
        !emmGetBool(args, "SYNC=", sync) || rpm > 3000 || acc > 255 || mode > 2) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.fastPositionParams(a, (uint16_t)rpm, (uint8_t)acc, (uint8_t)mode, sync);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "FAST_PARAM", false, a, r);
}

/**
 * @brief 下发快速位置模式的有符号脉冲数（手册 5.3.13，0xFC）。
 * @cmd SET EMM FAST MOVE A=<addr> PULSE=<-2147483648..2147483647>
 * @details 负脉冲表示反方向，使用此前 FAST PARAM 保存的速度和模式。
 */
static bool cmdSetEmmFastMove(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    int32_t pulses = 0;
    if (!emmGetAddress(args, a, all) || all || !emmGetI32(args, "PULSE=", pulses)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.fastPosition(a, pulses);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "FAST_MOVE", false, a, r);
}

/**
 * @brief 设置驱动器定时主动返回的信息（手册 5.5.1，0x11/0x18）。
 * @cmd SET EMM TIMER A=<addr> FUNC=<hex> MS=<0..65535>
 * @details FUNC 是需要周期返回的查询功能码，MS=0 停止定时返回。启用后返回帧
 *          是异步数据，PC 端必须按地址和功能码分流，避免与普通查询混淆。
 */
static bool cmdSetEmmTimer(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint32_t fn = 0, ms = 0;
    if (!emmGetAddress(args, a, all) || all || !emmGetU32(args, "FUNC=", fn) ||
        !emmGetU32(args, "MS=", ms) || fn > 255 || ms > 65535) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.timedInfo(a, (uint8_t)fn, (uint16_t)ms);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "TIMER", false, a, r);
}

/* ---------------- 驱动器参数写入 ----------------
 * SAVE=0 只修改当前运行参数，SAVE=1 同时写入非易失存储器。为避免 Flash
 * 寿命损耗，调试阶段应优先使用 SAVE=0；所有参数在发送前都执行范围检查。
 */

/**
 * @brief 修改电机 ID/地址（手册 5.6.1，0xAE/0x4B）。
 * @cmd SET EMM ADDRESS A=<old> SAVE=<0|1> NEW=<1..255>
 * @note 命令成功后，后续单播必须使用 NEW 地址；旧地址将不再响应。
 */
static bool cmdSetEmmAddress(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t n = 0;
    if (!emmGetAddress(args, a, all) || all || !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "NEW=", n) || n == 0 || n > 255) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyAddress(a, save, (uint8_t)n);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "ADDRESS", false, a, r);
}

/** @brief 5.6.2 修改细分（0x84/0x8A）。 */
static bool cmdSetEmmMicrostep(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all || !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "VALUE=", v) || v > 255) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyMicrostep(a, save, (uint8_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "MICROSTEP", false, a, r);
}

/** @brief 5.6.3 修改掉电标志（0x50）。 */
static bool cmdSetEmmPowerLoss(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool flag = false;
    if (!emmGetAddress(args, a, all) || all || !emmGetBool(args, "FLAG=", flag)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyPowerLossFlag(a, flag);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "POWER_LOSS", false, a, r);
}

/** @brief 5.6.5 修改电机类型（0xD7/0x35）。 */
static bool cmdSetEmmMotorType(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all || !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "TYPE=", v) || (v != 19 && v != 32)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyMotorType(a, save, (uint8_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "MOTOR_TYPE", false, a, r);
}

/** @brief 5.6.6 修改固件类型（0xD5/0x69）。 */
static bool cmdSetEmmFirmwareType(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all || !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "TYPE=", v) || v > 2) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyFirmwareType(a, save, (uint8_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "FIRMWARE_TYPE", false, a, r);
}

/** @brief 5.6.7~5.6.13 参数修改命令的具体处理函数。 */
/** @brief 手册 5.6.7：修改开环/闭环控制模式（0x46/0x69）。 */
static bool cmdSetEmmControlMode(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "MODE=", v) || v > 1) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyControlMode(a, save, (uint8_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "CONTROL_MODE", false, a, r);
}
/** @brief 手册 5.6.8：修改运动方向（0xD4/0x60）。 */
static bool cmdSetEmmDirection(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "DIR=", v) || v > 1) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyDirection(a, save, (uint8_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "DIRECTION", false, a, r);
}
/** @brief 手册 5.6.9：修改按键锁定（0xD0/0xB3）。 */
static bool cmdSetEmmKeyLock(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    bool v = false;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetBool(args, "LOCK=", v)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyKeyLock(a, save, v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "KEY_LOCK", false, a, r);
}
/** @brief 手册 5.6.11：修改 EMM 速度 0.1RPM 缩放开关（0x4F/0x71）。 */
static bool cmdSetEmmSpeedScale(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    bool v = false;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetBool(args, "SCALE=", v)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifySpeedScale(a, save, v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "SPEED_SCALE", false, a, r);
}
/** @brief 手册 5.6.12：修改开环工作电流（0x44/0x33）。 */
static bool cmdSetEmmOpenCurrent(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "MA=", v) || v > 5000) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyOpenCurrent(a, save, (uint16_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "OPEN_CURRENT", false, a, r);
}
/** @brief 手册 5.6.13：修改闭环/堵转最大电流（0x45/0x66）。 */
static bool cmdSetEmmMaxCurrent(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "MA=", v) || v > 5000) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyMaxCurrent(a, save, (uint16_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "MAX_CURRENT", false, a, r);
}
/** @brief 手册 5.6.31：修改参数锁定等级（0xD6/0x4B）。 */
static bool cmdSetEmmLockLevel(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "LEVEL=", v) || v > 3) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyLockLevel(a, save, (uint8_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "LOCK_LEVEL", false, a, r);
}

/** @brief 5.6.16/5.6.17 写入 PID 参数（0x4A/0xC3）。 */
static bool cmdSetEmmPid(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t kp = 0, ki = 0, kd = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "KP=", kp) ||
        !emmGetU32(args, "KI=", ki) ||
        !emmGetU32(args, "KD=", kd)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    EmmV5::PidParameters p = {kp, ki, kd};
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyPid(a, save, p);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "PID", false, a, r);
}

/** @brief 5.7.2 设置上电自动运行速度（0xF7/0x1C）。 */
static bool cmdSetEmmAutoSpeed(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    bool en = false;
    uint32_t dir = 0, rpm = 0, acc = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "DIR=", dir) ||
        !emmGetU32(args, "RPM=", rpm) ||
        !emmGetU32(args, "ACC=", acc) ||
        !emmGetBool(args, "EN=", en) ||
        dir > 1 || rpm > 3000 || acc > 255) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyAutoSpeed(a, save, (uint8_t)dir, (uint16_t)rpm, (uint8_t)acc, en);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "AUTO_SPEED", false, a, r);
}

/**
 * @brief 写入完整 Emm 驱动配置（手册 5.8.6，0x48/0xD1）。
 * @cmd SET EMM CONFIG A=<addr> SAVE=<0|1> HEX=<58个十六进制字符>
 * @details HEX 正好表示 29 字节配置字段，不含地址、功能码、辅助码和 0x6B 帧尾。
 * @warning 原始配置可以同时改变多个关键参数，调用方应先 GET EMM CONFIG 备份。
 */
static bool cmdSetEmmConfig(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    const char *p = strstr(args ? args : "", "HEX=");
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) || p == nullptr) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    p += 4;
    if (strlen(p) != 58) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    uint8_t data[29] = {0};
    for (size_t i = 0; i < 29; i++) {
        char s[3] = {p[i * 2], p[i * 2 + 1], 0};
        if (!isxdigit((unsigned char)s[0]) || !isxdigit((unsigned char)s[1])) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
        data[i] = (uint8_t)strtoul(s, nullptr, 16);
    }
    if (!Rs485Bus_Take(900)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyDriverConfig(a, save, data, 29);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "CONFIG", false, a, r);
}

/**
 * @brief 发送多电机复合命令（手册 5.3.1，0xAA）。
 * @cmd SET EMM MULTI HEX=<子命令十六进制字节流>
 * @details 每条内层子命令仍必须自带 0x6B 帧尾。本接口把整段数据作为广播
 *          复合帧写出，不等待多台电机应答，因此 ACCEPTED 只表示 UART 写出成功。
 */
static bool cmdSetEmmMulti(const char *args, int seq)
{
    const char *p = strstr(args ? args : "", "HEX=");
    if (!p) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    p += 4;
    size_t n = strlen(p);
    if (!n || (n & 1) || n > 500) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    uint8_t data[250] = {0};
    for (size_t i = 0; i < n / 2; i++) {
        char s[3] = {p[2 * i], p[2 * i + 1], 0};
        if (!isxdigit((unsigned char)s[0]) || !isxdigit((unsigned char)s[1])) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
        data[i] = (uint8_t)strtoul(s, nullptr, 16);
    }
    if (!Rs485Bus_Take(900)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.multiMotorCommand(data, n / 2);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "MULTI", true, 0, r);
}

/** @brief 5.6.21 修改位置到达窗口（0xD1/0x07）。 */
static bool cmdSetEmmPositionWindow(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "WINDOW=", v) || v > 65535) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyPositionWindow(a, save, (uint16_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "POSITION_WINDOW", false, a, r);
}

/** @brief 5.6.23 修改过热/过流保护阈值（0xD3/0x56）。 */
static bool cmdSetEmmProtection(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t t = 0, ma = 0, ms = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "TEMP=", t) ||
        !emmGetU32(args, "MA=", ma) ||
        !emmGetU32(args, "MS=", ms) ||
        t > 65535 || ma > 65535 || ms > 65535) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    EmmV5::ProtectionThresholds p = {(uint16_t)t, (uint16_t)ma, (uint16_t)ms};
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyProtectionThresholds(a, save, p);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "PROTECT", false, a, r);
}

/** @brief 5.6.25 修改心跳保护时间（0x68/0x38）。 */
static bool cmdSetEmmHeartbeat(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t ms = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "MS=", ms)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyHeartbeatTimeout(a, save, ms);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "HEARTBEAT", false, a, r);
}

/** @brief 5.6.27 修改积分限幅/刚性系数（0x4B/0x57）。 */
static bool cmdSetEmmIntegral(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "VALUE=", v)) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyIntegralLimit(a, save, v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "INTEGRAL", false, a, r);
}

/** @brief 5.6.29 修改碰撞回零返回角度（0x5C/0xAC）。 */
static bool cmdSetEmmCollisionAngle(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "ANGLE=", v) || v > 65535) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyCollisionReturnAngle(a, save, (uint16_t)v);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "COLLISION_ANGLE", false, a, r);
}

/** @brief 5.6.19 修改 DMX512 参数（0xD9/0x90）。 */
static bool cmdSetEmmDmx(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    bool save = false;
    uint32_t ch = 0, per = 0, mode = 0, speed = 0, acc = 0, step = 0, motion = 0;
    if (!emmGetAddress(args, a, all) || all ||
        !emmGetBool(args, "SAVE=", save) ||
        !emmGetU32(args, "CHANNELS=", ch) ||
        !emmGetU32(args, "PER=", per) ||
        !emmGetU32(args, "MODE=", mode) ||
        !emmGetU32(args, "RPM=", speed) ||
        !emmGetU32(args, "ACC=", acc) ||
        !emmGetU32(args, "SPEED_STEP=", step) ||
        !emmGetU32(args, "MOTION_STEP=", motion) ||
        ch < 1 || ch > 65535 || (per != 1 && per != 2) || mode > 1 ||
        speed < 1 || speed > 3000 || acc == 0 || step == 0 || motion == 0) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    EmmV5::DmxParameters p = {(uint16_t)ch, (uint8_t)per, (uint8_t)mode,
                              (uint16_t)speed, (uint16_t)acc,
                              (uint16_t)step, motion};
    if (!Rs485Bus_Take(800)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.modifyDmxParameters(a, save, p);
    Rs485Bus_Give();
    return emmReplyStatus(seq, "DMX", false, a, r);
}

/**
 * @brief 处理 GET EMM POS（手册 5.5.13，查询功能码 0x36）。
 * @cmd GET EMM POS A=<addr>
 * @return 返回 RAW 原始位置以及按 65536 单位换算的 DEG 角度。
 */
static bool cmdGetEmmPos(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    uint8_t address = 0;
    bool all = false;
    int32_t position = 0;
    if (!emmGetAddress(args, address, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.readCurrentPosition(address, position);
    Rs485Bus_Give();
    if (ret != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM POS A=%u %s", seq, (unsigned)address,
                    EmmV5::statusToString(ret));
        return false;
    }
    double degrees = (double)position * 360.0 / 65536.0;
    sendWithCRC("OK @%d EMM POS A=%u RAW=%ld DEG=%.4f", seq,
                (unsigned)address, (long)position, degrees);
    return true;
}

/**
 * @brief 处理 GET EMM SPEED（手册 5.5.11，查询功能码 0x35）。
 * @cmd GET EMM SPEED A=<addr>
 * @return RPM 为带方向的实时转速，负值表示反向。
 */
static bool cmdGetEmmSpeed(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    uint8_t address = 0;
    bool all = false;
    int16_t rpm = 0;
    if (!emmGetAddress(args, address, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.readVelocity(address, rpm);
    Rs485Bus_Give();
    if (ret != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM SPEED A=%u %s", seq, (unsigned)address,
                    EmmV5::statusToString(ret));
        return false;
    }
    sendWithCRC("OK @%d EMM SPEED A=%u RPM=%d", seq, (unsigned)address, (int)rpm);
    return true;
}

/** @brief 读取允许负数的 32 位 EMM 参数（定义见后方快速位置命令辅助区）。 */
static bool emmGetI32(const char *args, const char *key, int32_t &value);

/**
 * @brief 读取允许负数的 32 位 EMM 参数。
 * @details 快速位置模式的脉冲数按手册 5.3.13 定义为有符号 int32_t，不能使用
 *          仅接受无符号数的 emmGetU32()，否则反向运动命令会被错误拒绝。
 */
static bool emmGetI32(const char *args, const char *key, int32_t &value)
{
    if (args == nullptr || key == nullptr) return false;
    const char *p = strstr(args, key);
    if (p == nullptr) return false;
    p += strlen(key);
    char *end = nullptr;
    long parsed = strtol(p, &end, 0);
    if (end == p || parsed < INT32_MIN || parsed > INT32_MAX) return false;
    value = (int32_t)parsed;
    return true;
}

/* ---------------- EMM 状态和参数查询 ----------------
 * 查询命令只允许单播地址，因为多台电机同时回复会造成 RS485 碰撞。每个处理
 * 函数在持锁期间完成一次“发送查询帧+等待固定长度响应”，释放总线后再输出
 * PC 回复。通信异常统一返回 TIMEOUT/FRAME_ERROR/REJECTED 等稳定文本。
 */

/**
 * @brief 读取固件和硬件版本（手册 5.5.2，0x1F）。
 * @cmd GET EMM VERSION A=<addr>
 * @return FW、硬件系列、硬件类型和硬件版本字段。
 */
static bool cmdGetEmmVersion(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    EmmV5::VersionInfo v = {};
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readVersion(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM VERSION A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM VERSION A=%u FW=%u HW_SERIES=%u HW_TYPE=%u HW_VER=%u",
                seq, (unsigned)a, (unsigned)v.firmwareVersion,
                (unsigned)v.hardwareSeries, (unsigned)v.hardwareType,
                (unsigned)v.hardwareVersion);
    return true;
}

/** @brief 5.5.3 读取相电阻/相电感（0x20）。 */
static bool cmdGetEmmPhase(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    EmmV5::PhaseParameters p = {};
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readPhaseParameters(a, p);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM PHASE A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM PHASE A=%u R_MOHM=%u L_UH=%u",
                seq, (unsigned)a, (unsigned)p.resistanceMOhm, (unsigned)p.inductanceUh);
    return true;
}

/** @brief 5.5.4 读取总线电压（0x24）。 */
static bool cmdGetEmmBusVoltage(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint16_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readBusVoltage(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM BUS_VOLT A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM BUS_VOLT A=%u MV=%u", seq, (unsigned)a, (unsigned)v);
    return true;
}

/** @brief 5.5.6 读取相电流（0x27）。 */
static bool cmdGetEmmPhaseCurrent(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint16_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readPhaseCurrent(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM PHASE_CURRENT A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM PHASE_CURRENT A=%u MA=%u", seq, (unsigned)a, (unsigned)v);
    return true;
}

/** @brief 5.5.7 读取线性化编码器值（0x31）。 */
static bool cmdGetEmmEncoder(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint16_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readEncoder(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM ENCODER A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM ENCODER A=%u RAW=%u", seq, (unsigned)a, (unsigned)v);
    return true;
}

/** @brief 5.5.8 读取输入脉冲数（0x32）。 */
static bool cmdGetEmmInputPulse(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    int32_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readInputPulses(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM INPUT_PULSE A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM INPUT_PULSE A=%u PULSE=%ld", seq, (unsigned)a, (long)v);
    return true;
}

/** @brief 5.5.5 读取总线电流（0x26）。 */
static bool cmdGetEmmBusCurrent(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint16_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readBusCurrent(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM BUS_CURRENT A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM BUS_CURRENT A=%u MA=%u", seq, (unsigned)a, (unsigned)v);
    return true;
}

/** @brief 5.5.17 读取 En/Stp/Dir IO 状态（0x3D）。 */
static bool cmdGetEmmIo(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    EmmV5::IoState s = {};
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readIoState(a, s);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM IO A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM IO A=%u RAW=0x%02X EN=%d STP=%d DIR=%d DIR_OUT=%d",
                seq, (unsigned)a, s.raw, s.enLevel ? 1 : 0, s.stpLevel ? 1 : 0,
                s.dirLevel ? 1 : 0, s.dirOutputMode ? 1 : 0);
    return true;
}

/** @brief 5.6.4 读取选项参数状态（0x1A）。 */
static bool cmdGetEmmOptions(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    EmmV5::OptionFlags f = {};
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readOptionFlags(a, f);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM OPTIONS A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM OPTIONS A=%u RAW=0x%02X MOTOR09=%d EMM=%d CLOSED=%d "
                "REVERSE=%d LOCK=%d SCALE10X=%d",
                seq, (unsigned)a, f.raw, f.motorType09 ? 1 : 0,
                f.emmFirmware ? 1 : 0, f.closedLoop ? 1 : 0,
                f.reverseDirection ? 1 : 0, f.keyLocked ? 1 : 0,
                f.scale10x ? 1 : 0);
    return true;
}

/** @brief 5.5.12 读取实时温度（0x39）。 */
static bool cmdGetEmmTemperature(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    int16_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readTemperature(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM TEMP A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM TEMP A=%u C=%d", seq, (unsigned)a, (int)v);
    return true;
}

/** @brief 5.6.20 读取位置到达窗口（0x41）。 */
static bool cmdGetEmmPositionWindow(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint16_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readPositionWindow(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM POSITION_WINDOW A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM POSITION_WINDOW A=%u RAW=%u", seq, (unsigned)a, (unsigned)v);
    return true;
}

/** @brief 5.6.22 读取过热/过流保护阈值（0x13）。 */
static bool cmdGetEmmProtection(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    EmmV5::ProtectionThresholds p = {};
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readProtectionThresholds(a, p);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM PROTECT A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM PROTECT A=%u TEMP=%u MA=%u MS=%u",
                seq, (unsigned)a, (unsigned)p.overTemperatureC,
                (unsigned)p.overCurrentMa, (unsigned)p.durationMs);
    return true;
}

/** @brief 5.6.24 读取心跳保护时间（0x16）。 */
static bool cmdGetEmmHeartbeat(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readHeartbeatTimeout(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM HEARTBEAT A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM HEARTBEAT A=%u MS=%lu", seq, (unsigned)a, (unsigned long)v);
    return true;
}

/** @brief 5.6.26 读取积分限幅/刚性系数（0x23）。 */
static bool cmdGetEmmIntegral(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint32_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readIntegralLimit(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM INTEGRAL A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM INTEGRAL A=%u VALUE=%lu", seq, (unsigned)a, (unsigned long)v);
    return true;
}

/** @brief 5.6.28 读取碰撞回零返回角度（0x3F）。 */
static bool cmdGetEmmCollisionAngle(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint16_t v = 0;
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readCollisionReturnAngle(a, v);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM COLLISION_ANGLE A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM COLLISION_ANGLE A=%u DEG10=%u", seq, (unsigned)a, (unsigned)v);
    return true;
}

/** @brief 5.6.30 广播读取 ID；总线上必须只有一台待识别电机。 */
static bool cmdGetEmmBroadcastAddress(const char *args, int seq)
{
    (void)args;
    uint8_t a = 0;
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readBroadcastAddress(a);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM BROADCAST_ADDRESS %s", seq, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM BROADCAST_ADDRESS A=%u", seq, (unsigned)a);
    return true;
}

/** @brief 5.6.18 读取 DMX512 参数（0x49/0x78）。 */
static bool cmdGetEmmDmx(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    EmmV5::DmxParameters p = {};
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readDmxParameters(a, p);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM DMX A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM DMX A=%u CHANNELS=%u PER=%u MODE=%u RPM=%u ACC=%u "
                "SPEED_STEP=%u MOTION_STEP=%lu",
                seq, (unsigned)a, (unsigned)p.channelCount,
                (unsigned)p.channelsPerMotor, (unsigned)p.motionMode,
                (unsigned)p.singleSpeedRpm, (unsigned)p.acceleration,
                (unsigned)p.dualSpeedStep, (unsigned long)p.dualMotionStep);
    return true;
}

/** @brief 5.6.16 读取 PID 参数（0x21）。 */
static bool cmdGetEmmPid(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    EmmV5::PidParameters p = {};
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(600)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readPid(a, p);
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM PID A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    sendWithCRC("OK @%d EMM PID A=%u KP=%lu KI=%lu KD=%lu",
                seq, (unsigned)a, (unsigned long)p.kp, (unsigned long)p.ki,
                (unsigned long)p.kd);
    return true;
}

/** @brief 5.8.5 读取完整 Emm 驱动配置（0x42/0x6C）。 */
static bool cmdGetEmmConfig(const char *args, int seq)
{
    uint8_t a = 0;
    bool all = false;
    uint8_t data[33] = {0};
    if (!emmGetAddress(args, a, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(900)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status r = gEmm.readDriverConfig(a, data, sizeof(data));
    Rs485Bus_Give();
    if (r != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM CONFIG A=%u %s", seq, (unsigned)a, EmmV5::statusToString(r));
        return false;
    }
    char hex[67] = {0};
    for (size_t i = 0; i < sizeof(data); i++) {
        snprintf(hex + i * 2, 3, "%02X", data[i]);
    }
    sendWithCRC("OK @%d EMM CONFIG A=%u HEX=%s", seq, (unsigned)a, hex);
    return true;
}

/**
 * @brief 处理 5.4.4 回零状态标志读取命令。
 * @cmd GET EMM ORIGIN A=1
 * @details 查询功能码为 0x3B；回复中的 FLAGS 同时展开为编码器校准、回零中、
 *          回零错误、过温和过流等 bit 状态，便于上位机直接判断。
 */
static bool cmdGetEmmOrigin(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    uint8_t address = 0;
    bool all = false;
    uint8_t flags = 0;
    if (!emmGetAddress(args, address, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.readOriginFlags(address, flags);
    Rs485Bus_Give();
    if (ret != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM ORIGIN A=%u %s", seq,
                    (unsigned)address, EmmV5::statusToString(ret));
        return false;
    }
    sendWithCRC("OK @%d EMM ORIGIN A=%u FLAGS=0x%02X ENC=%d CAL=%d "
                "HOMING=%d HOME_ERR=%d OVERTEMP=%d OVERCUR=%d", seq,
                (unsigned)address, flags,
                flags & EmmV5::ORIGIN_ENCODER_READY ? 1 : 0,
                flags & EmmV5::ORIGIN_CALIBRATED ? 1 : 0,
                flags & EmmV5::ORIGIN_HOMING ? 1 : 0,
                flags & EmmV5::ORIGIN_FAILED ? 1 : 0,
                flags & EmmV5::ORIGIN_OVERHEAT ? 1 : 0,
                flags & EmmV5::ORIGIN_OVERCURRENT ? 1 : 0);
    return true;
}

/**
 * @brief 处理 5.4.5 回零参数读取命令。
 * @cmd GET EMM HOME PARAM A=1
 * @details 查询功能码为 0x22；EmmV5 负责按大端格式解析 18 字节返回帧，
 *          本函数只负责组织为上位机使用的字段名。
 */
static bool cmdGetEmmHomeParams(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    uint8_t address = 0;
    bool all = false;
    uint8_t mode = 0, direction = 0;
    uint16_t rpm = 0, stallRpm = 0, stallMa = 0, stallMs = 0;
    uint32_t homeTimeout = 0;
    bool autoHome = false;
    if (!emmGetAddress(args, address, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.readHomeParams(
        address, mode, direction, rpm, homeTimeout, stallRpm, stallMa,
        stallMs, autoHome);
    Rs485Bus_Give();
    if (ret != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM HOME_PARAM A=%u %s", seq,
                    (unsigned)address, EmmV5::statusToString(ret));
        return false;
    }
    sendWithCRC("OK @%d EMM HOME_PARAM A=%u MODE=%u DIR=%u RPM=%u "
                "TIMEOUT=%lu STALL_RPM=%u STALL_MA=%u STALL_MS=%u AUTO=%d",
                seq, (unsigned)address, (unsigned)mode, (unsigned)direction,
                (unsigned)rpm, (unsigned long)homeTimeout, (unsigned)stallRpm,
                (unsigned)stallMa, (unsigned)stallMs, autoHome ? 1 : 0);
    return true;
}

/**
 * @brief 扫描 RS485 总线上的 EMM 电机地址。
 * @details 使用 0x36 实时位置查询逐地址探测，避免广播读取造成响应冲突。
 *          默认扫描 1~24；指定 FULL 后扫描 1~255，也可使用 MIN/MAX 限定范围。
 * @cmd SCAN EMM [MIN=1] [MAX=24] [FULL]
 * @return 结果可能拆成多条 PART 回复；COUNT 是本次扫描发现的总数，ADDR 是当前分片地址列表。
 * @note 扫描期间独占 RS485 总线，单地址查询超时为 40 ms，适合启动发现或人工诊断。
 */
static bool cmdScanEmm(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();

    uint32_t minAddress = 1;
    uint32_t maxAddress = strstr(args ? args : "", "FULL") ? 255 : 24;
    uint32_t parsed = 0;
    if (strstr(args ? args : "", "MIN=") != nullptr) {
        if (!emmGetU32(args, "MIN=", parsed)) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
        minAddress = parsed;
    }
    if (strstr(args ? args : "", "MAX=") != nullptr) {
        if (!emmGetU32(args, "MAX=", parsed)) {
            sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
            return false;
        }
        maxAddress = parsed;
    }
    if (minAddress == 0 || minAddress > 255 || maxAddress == 0 ||
        maxAddress > 255 || minAddress > maxAddress) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }

    const uint32_t scanStartedAtMs = millis();
    Serial.printf("[EMM扫描] 开始扫描地址%lu~%lu，请等待扫描完成\n",
                  (unsigned long)minAddress, (unsigned long)maxAddress);

    uint8_t found[255] = {0};
    size_t foundCount = 0;
    if (!Rs485Bus_Take(2000)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }

    for (uint32_t candidate = minAddress; candidate <= maxAddress; ++candidate) {
        int32_t position = 0;
        EmmV5::Status ret = gEmm.readCurrentPosition(
            (uint8_t)candidate, position, 40);
        if (ret == EmmV5::STATUS_OK && foundCount < sizeof(found)) {
            found[foundCount++] = (uint8_t)candidate;
            Serial.printf("[EMM扫描] 发现电机 A=%lu，当前位置RAW=%ld\n",
                          (unsigned long)candidate, (long)position);
        }
        delay(1);
    }
    Rs485Bus_Give();
    Serial.printf("[EMM扫描] 扫描完成，发现%u台电机，耗时%lu ms\n",
                  (unsigned)foundCount,
                  (unsigned long)(millis() - scanStartedAtMs));

    /* 每条响应限制地址数量，避免超过上位机单行和 CRC 缓冲区长度。 */
    const size_t addressesPerReply = 8;
    if (foundCount == 0) {
        sendWithCRC("OK @%d EMM SCAN COUNT=0 ADDR=NONE", seq);
        return true;
    }

    size_t parts = (foundCount + addressesPerReply - 1) / addressesPerReply;
    for (size_t part = 0; part < parts; ++part) {
        char addressList[64] = {0};
        size_t begin = part * addressesPerReply;
        size_t end = begin + addressesPerReply;
        if (end > foundCount) end = foundCount;
        size_t offset = 0;
        for (size_t i = begin; i < end; ++i) {
            int written = snprintf(addressList + offset,
                                   sizeof(addressList) - offset, "%s%u",
                                   i == begin ? "" : ",",
                                   (unsigned)found[i]);
            if (written <= 0 || (size_t)written >= sizeof(addressList) - offset) {
                break;
            }
            offset += (size_t)written;
        }
        sendWithCRC("OK @%d EMM SCAN COUNT=%u PART=%u/%u ADDR=%s", seq,
                    (unsigned)foundCount, (unsigned)(part + 1),
                    (unsigned)parts, addressList);
    }
    return true;
}

/**
 * @brief 处理 GET EMM FLAGS（手册 5.5.16，查询功能码 0x3C）。
 * @cmd GET EMM FLAGS A=<addr>
 * @details 一次读取回零标志和电机状态标志，并在回复中展开关键 bit；该查询为
 *          单播操作，避免多台电机同时应答造成 RS485 冲突。
 */
static bool cmdGetEmmFlags(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    uint8_t address = 0;
    bool all = false;
    uint8_t origin = 0, motor = 0;
    if (!emmGetAddress(args, address, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(500)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.readCombinedFlags(address, origin, motor);
    Rs485Bus_Give();
    if (ret != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM FLAGS A=%u %s", seq, (unsigned)address,
                    EmmV5::statusToString(ret));
        return false;
    }
    sendWithCRC("OK @%d EMM FLAGS A=%u ORIGIN=0x%02X MOTOR=0x%02X "
                "ENC=%d CAL=%d HOMING=%d HOME_ERR=%d OVERTEMP=%d OVERCUR=%d "
                "ENABLE=%d REACHED=%d STALL=%d PROTECT=%d LEFT=%d RIGHT=%d "
                "POWER_LOSS=%d", seq, (unsigned)address, origin, motor,
                origin & EmmV5::ORIGIN_ENCODER_READY ? 1 : 0,
                origin & EmmV5::ORIGIN_CALIBRATED ? 1 : 0,
                origin & EmmV5::ORIGIN_HOMING ? 1 : 0,
                origin & EmmV5::ORIGIN_FAILED ? 1 : 0,
                origin & EmmV5::ORIGIN_OVERHEAT ? 1 : 0,
                origin & EmmV5::ORIGIN_OVERCURRENT ? 1 : 0,
                motor & EmmV5::MOTOR_ENABLED ? 1 : 0,
                motor & EmmV5::MOTOR_POSITION_REACHED ? 1 : 0,
                motor & EmmV5::MOTOR_STALL ? 1 : 0,
                motor & EmmV5::MOTOR_STALL_PROTECT ? 1 : 0,
                motor & EmmV5::MOTOR_LEFT_LIMIT ? 1 : 0,
                motor & EmmV5::MOTOR_RIGHT_LIMIT ? 1 : 0,
                motor & EmmV5::MOTOR_POWER_LOSS ? 1 : 0);
    return true;
}

/**
 * @brief 处理 GET EMM STATE（手册 5.8.2，查询功能码 0x43，子功能 0x7A）。
 * @cmd GET EMM STATE A=<addr>
 * @details 返回母线电压、相电流、编码器值、目标位置、实时转速、实时位置、
 *          位置误差和两个状态字；位置字段同时按 65536 单位换算为角度。
 */
static bool cmdGetEmmState(const char *args, int seq)
{
    cmd_on_line = true;
    lasCmdTime = millis();
    uint8_t address = 0;
    bool all = false;
    EmmV5::SystemState state = {};
    if (!emmGetAddress(args, address, all) || all) {
        sendWithCRC("ERR @%d %d", seq, PARAM_ERR);
        return false;
    }
    if (!Rs485Bus_Take(700)) {
        sendWithCRC("ERR @%d %d", seq, RS485_BUSY);
        return false;
    }
    EmmV5::Status ret = gEmm.readSystemState(address, state);
    Rs485Bus_Give();
    if (ret != EmmV5::STATUS_OK) {
        sendWithCRC("ERR @%d EMM STATE A=%u %s", seq, (unsigned)address,
                    EmmV5::statusToString(ret));
        return false;
    }
    double targetDeg = (double)state.targetPosition * 360.0 / 65536.0;
    double positionDeg = (double)state.realtimePosition * 360.0 / 65536.0;
    double errorDeg = (double)state.positionError * 360.0 / 65536.0;
    sendWithCRC("OK @%d EMM STATE A=%u VBUS=%u IPHA=%u ENC=%u RPM=%d "
                "TARGET=%ld TARGET_DEG=%.4f POS=%ld POS_DEG=%.4f "
                "ERR=%ld ERR_DEG=%.4f ORIGIN=0x%02X MOTOR=0x%02X", seq,
                (unsigned)address, (unsigned)state.busVoltageMv,
                (unsigned)state.phaseCurrentMa, (unsigned)state.encoderValue,
                (int)state.realtimeRpm, (long)state.targetPosition, targetDeg,
                (long)state.realtimePosition, positionDeg,
                (long)state.positionError, errorDeg, state.originFlags,
                state.motorFlags);
    return true;
}
/**
 * @brief  重启系统
 */
static bool cmdSetRESTART(const char *args, int seq){
    
    ESP.restart();
    return true;
}

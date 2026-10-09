#ifndef EMM_V5_H
#define EMM_V5_H

#include <Arduino.h>

/**
 * @brief ZDT X42S Emm 固件自由协议驱动。
 *
 * @details 本类只负责“协议层”工作：检查参数、按照手册规定的字节序组帧、
 *          在共享 HardwareSerial 上发送数据、匹配应答并把返回字段转换为 C++
 *          数据类型。它不负责解析 PC 文本命令，也不判断整机电源状态；这些
 *          “业务层”工作由 CommandDispatcher 完成。
 *
 *          本工程的 SV-04M、GFL-Z 和 EMM 共用一条 RS485 总线，因此调用本类
 *          任一通信函数前，调用方必须先取得 Rs485Bus_Take() 互斥锁，并在函数
 *          返回后调用 Rs485Bus_Give()。这样可防止两个任务的帧互相穿插。
 *
 * @note 自由协议帧尾固定为 0x6B，内层 EMM 帧不附加 Modbus CRC；PC 与 ESP32
 *       之间的文本命令 CRC16 是另一层协议，不能混入这里。
 */
class EmmV5
{
public:
    /**
     * @brief 底层串口/协议操作结果。
     * @details 必须区分"命令被接受"与"动作已完成"两件事：
     *          - STATUS_DONE：驱动器主动上报了动作完成帧 [addr][func][0x9F][0x6B]，
     *            表示动作确实做完（到位 FD+9F、回零完成 9A+9F）。**仅当调用方显式
     *            要求 requireDone=true 时才会返回该状态**；未要求时 0x9F 仍按历史
     *            语义映射为 STATUS_OK，以保证既有调用方行为不变；
     *          - STATUS_OK：只表示命令被驱动器接受（收到 0x02），或广播帧已写出；
     *            广播没有设备级 ACK，动作是否完成需另行确认；
     *          - STATUS_TIMEOUT：规定时间内没有收到任何有效响应；
     *          - STATUS_FRAME_ERROR：收到数据但无法拼出合法帧；
     *          - STATUS_REJECTED：驱动器明确返回拒绝状态（0xE2/0xEE 等）；
     *          - STATUS_NOT_MOVED：驱动器返回 0x12/0x22，即"触发回零时已在零点 /
     *            左右限位已触发"，电机不动——手册明确这不是错误，但与"动作完成"
     *            也不是一回事，故单独一态交上层判断。
     */
    enum Status : int8_t {
        STATUS_DONE = 1,
        STATUS_OK = 0,
        STATUS_TIMEOUT = -1,
        STATUS_FRAME_ERROR = -2,
        STATUS_REJECTED = -3,
        STATUS_INVALID_ARGUMENT = -4,
        STATUS_SERIAL_NOT_BEGIN = -5,
        STATUS_WRITE_ERROR = -6,
        STATUS_NOT_MOVED = -7
    };

    /**
     * @brief 手册 5.4.4 的回零状态字位掩码（查询功能码 0x3B）。
     * @details ORIGIN_HOMING=1 表示正在回零；ORIGIN_FAILED=1 表示本次回零失败。
     *          两位都为 0 时，手册定义为回零成功或尚未启动的默认空闲状态，
     *          因此上层异步监控必须结合“已经下发回零命令”和启动保护时间判断。
     */
    enum OriginFlag : uint8_t {
        ORIGIN_ENCODER_READY = 0x01,  ///< Enc_Rdy：编码器正常。
        ORIGIN_CALIBRATED    = 0x02,  ///< Cal_Rdy：编码器已经校准。
        ORIGIN_HOMING        = 0x04,  ///< Org_SF：驱动器正在执行回零。
        ORIGIN_FAILED        = 0x08,  ///< Org_CF：回零失败。
        ORIGIN_OVERHEAT      = 0x10,  ///< Otp_TF：过热保护已触发。
        ORIGIN_OVERCURRENT   = 0x20   ///< Ocp_TF：过流保护已触发。
    };

    /**
     * @brief 手册 5.5.15 的电机状态字位掩码（查询功能码 0x3A）。
     * @note bit6 为保留位，不能用于状态判断。
     */
    enum MotorFlag : uint8_t {
        MOTOR_ENABLED        = 0x01,  ///< Ens_TF：电机已使能。
        MOTOR_POSITION_REACHED = 0x02,///< Prf_TF：目标位置已到达。
        MOTOR_STALL          = 0x04,  ///< Cgi_TF：检测到堵转。
        MOTOR_STALL_PROTECT  = 0x08,  ///< Cgp_TF：堵转保护已触发。
        MOTOR_LEFT_LIMIT     = 0x10,  ///< Esi_LF：左限位输入为高电平。
        MOTOR_RIGHT_LIMIT    = 0x20,  ///< Esi_RF：右限位输入为高电平。
        MOTOR_POWER_LOSS     = 0x80   ///< Oac_TF：掉电标志。
    };

    /**
     * @brief 最近一次收到的原始响应帧及其摘要。
     * @details raw 保存完整帧（最多 64 字节），rawLength 是有效长度；address、function
     *          和 commandStatus 对应响应的前三个字段，便于记录故障而无需重新解析串口。
     */
    struct Response {
        uint8_t raw[64];
        uint8_t rawLength;
        uint8_t address;
        uint8_t function;
        uint8_t commandStatus;
    };

    /**
     * @brief 5.8.2 系统状态参数的结构化表示。
     * @note 位置量使用驱动器原始计数单位（65536 计数/圈），电压/电流分别为 mV/mA；
     *       originFlags 和 motorFlags 保留协议原始 bit，命令层负责展开显示。
     */
    struct SystemState {
        uint16_t busVoltageMv;
        uint16_t phaseCurrentMa;
        uint16_t encoderValue;
        int32_t targetPosition;
        int16_t realtimeRpm;
        int32_t realtimePosition;
        int32_t positionError;
        uint8_t originFlags;
        uint8_t motorFlags;
    };

    /**
     * @brief 手册 5.5.2 的固件/硬件版本信息。
     * @note firmwareVersion 例如 200 表示 V2.0.0；hardwareSeries、
     *       hardwareType 和 hardwareVersion 对应返回帧中的硬件信息位域。
     */
    struct VersionInfo {
        uint16_t firmwareVersion;
        uint8_t hardwareSeries;
        uint8_t hardwareType;
        uint8_t hardwareVersion;
    };

    /** @brief 手册 5.5.3 返回的相电阻和相电感，单位分别为 mΩ 和 μH。 */
    struct PhaseParameters {
        uint16_t resistanceMOhm;
        uint16_t inductanceUh;
    };

    /**
     * @brief 手册 5.6.16/5.6.17 的 Emm PID 参数。
     * @details 三个参数均为驱动器原始无符号 32 位系数，具体量纲由固件定义。
     */
    struct PidParameters {
        uint32_t kp;
        uint32_t ki;
        uint32_t kd;
    };

    /** @brief 5.6.4 读取选项参数状态的原始 bit 字段。 */
    struct OptionFlags {
        uint8_t raw;
        bool motorType09;
        bool emmFirmware;
        bool closedLoop;
        bool reverseDirection;
        bool keyLocked;
        bool scale10x;
    };

    /** @brief 5.5.17 读取引脚 IO 电平状态。 */
    struct IoState {
        uint8_t raw;
        bool enLevel;
        bool stpLevel;
        bool dirLevel;
        bool dirOutputMode;
    };

    /**
     * @brief 创建 EMM V5 协议驱动对象。
     * @param serial 与 EMM 电机连接的硬件串口引用。
     * @note 该构造函数只保存串口对象指针，不会立即配置波特率、引脚或打开串口。
     *       串口初始化由 Rs485Devices_Init() 统一完成，避免同一条 RS485 总线被重复配置。
     */
    explicit EmmV5(HardwareSerial &serial);
    /**
     * @brief 标记驱动已准备好通信，并设置默认目标地址。
     * @param defaultAddress 上电后使用的默认电机地址，合法范围为 1~255；
     *        本工程默认使用 0x02，因为 0x01 已分配给 GFL-Z 激光传感器。
     * @note EMM 协议规定地址 0 为广播地址，因此这里不会接受地址 0；
     *       该函数不会再次调用 HardwareSerial.begin()。
     */
    void begin(uint8_t defaultAddress = 0x02);
    /** @brief 修改当前默认地址；地址 0 被保留为广播地址，不会被设置。 */
    void setAddress(uint8_t address);
    /** @brief 返回当前默认地址。 */
    uint8_t address() const;
    /**
     * @brief 开关协议帧调试日志。
     * @param enabled 为 true 时打印发送和接收的十六进制原始帧。
     * @note 日志只用于诊断，不会改变协议帧内容或超时策略。
     */
    void setDebugEnabled(bool enabled);
    /** @brief 返回当前是否启用了原始帧调试日志。 */
    bool debugEnabled() const;
    /**
     * @brief 获取最近一次成功匹配到的响应帧。
     * @return Response 结构体引用；广播发送没有响应时不会产生新的有效响应。
     */
    const Response &lastResponse() const;

    /**
     * @brief 5.3.2 电机使能控制（0xF3/0xAB）。
     * @param address 目标单播地址 1~255。
     * @param enabled true 使能，false 失能。
     * @param synchronized true 时等待同步触发。
     * @return 单播 ACK 结果。
     */
    Status enable(uint8_t address, bool enabled, bool synchronized = false,
                  uint32_t timeoutMs = 300);
    /**
     * @brief 5.3.7 速度模式控制（0xF6）。
     * @param address 目标地址；direction 为 0/1，rpm 为目标转速。
     * @param acceleration 0~255 的协议加速度字段。
     */
    Status velocity(uint8_t address, uint8_t direction, uint16_t rpm,
                    uint8_t acceleration, bool synchronized = false,
                    uint32_t timeoutMs = 300);
    /**
     * @brief 位置控制兼容接口。
     * @details absolute=false 对应 MODE=0，true 对应 MODE=1；脉冲数为无符号 32 位值。
     */
    Status position(uint8_t address, uint8_t direction, uint16_t rpm,
                    uint8_t acceleration, uint32_t pulses, bool absolute,
                    bool synchronized = false, uint32_t timeoutMs = 300);
    /**
     * @brief 5.3.12 位置模式控制（0xFD），MODE 取 0/1/2。
     * @param pulses 目标位移脉冲数，按大端 32 位写入。
     * @param timeoutMs 等待应答的最长时间；requireDone=true 时它同时是等待
     *        "动作完成上报 0x9F"的最长时间，应给足整段运动的时长。
     * @param requireDone true 时要求驱动器主动上报 [addr][0xFD][0x9F][0x6B] 才算完成：
     *        收到 0x02（命令已接受）后继续在同一个接收循环内等 0x9F；
     *        等到则返回 STATUS_DONE，只收到 0x02 而超时则返回 STATUS_OK。
     * @note requireDone=true 期间会一直占用 RS485 总线（调用方持锁），
     *       因为一旦中途交还总线，其它任务的清缓冲会把 0x9F 丢掉。
     */
    Status positionMode(uint8_t address, uint8_t direction, uint16_t rpm,
                        uint8_t acceleration, uint32_t pulses,
                        uint8_t motionMode, bool synchronized = false,
                        uint32_t timeoutMs = 300, bool requireDone = false);
    /**
     * @brief 立即停止指定电机当前运动。
     * @param synchronized 为 true 时先缓存停止命令，等待同步触发后执行。
     */
    Status stop(uint8_t address, bool synchronized = false,
                uint32_t timeoutMs = 300);
    /**
     * @brief 5.4.2 触发回零（0x9A）。
     * @param mode 回零模式 0~5，具体方向/限位策略由驱动器参数决定。
     * @param requireDone true 时要求驱动器主动上报 [addr][0x9A][0x9F][0x6B]
     *        （回零完成）才算完成；语义与 positionMode() 的 requireDone 相同。
     */
    Status home(uint8_t address, uint8_t mode, bool synchronized = false,
                uint32_t timeoutMs = 300, bool requireDone = false);
    /** @brief 5.4.3 强制中断并退出回零（0x9C/0x48）。 */
    Status interruptHome(uint8_t address, uint32_t timeoutMs = 300);
    /**
     * @brief 将指定电机当前位置清零（不修改单圈回零参数）。
     * @note 对应 0x0A 位置清零命令；若需要修改回零零点，应使用 setOriginZero()。
     */
    Status resetCurrentPosition(uint8_t address, uint32_t timeoutMs = 300);
    /**
     * @brief 5.4.1 设置单圈回零零点（0x93/0x88）。
     * @param save 是否把零点写入掉电保存区；false 仅修改当前运行参数。
     */
    Status setOriginZero(uint8_t address, bool save, uint32_t timeoutMs = 300);
    /** @brief 解除堵转、过热或过流保护。 */
    Status resetClogProtection(uint8_t address, uint32_t timeoutMs = 300);

    /** @brief 5.2.1 触发编码器线性化校准（0x06/0x45）。 */
    Status calibrateEncoder(uint8_t address, uint32_t timeoutMs = 300);
    /** @brief 5.2.2 重启驱动器（0x08/0x97）。 */
    Status restart(uint8_t address, uint32_t timeoutMs = 300);
    /** @brief 5.2.5 恢复出厂设置（0x0F/0x5F）。 */
    Status restoreFactory(uint8_t address, uint32_t timeoutMs = 300);

    /**
     * @brief 5.3.13 第一步：设置快速位置模式的速度、加速度和模式（0xF1）。
     * @note 该功能要求 Emm 固件 V2.0.0 以上；设置后使用 fastPosition() 快速下发脉冲。
     */
    Status fastPositionParams(uint8_t address, uint16_t rpm,
                              uint8_t acceleration, uint8_t motionMode,
                              bool synchronized = false,
                              uint32_t timeoutMs = 300);
    /** @brief 5.3.13 第二步：发送快速位置脉冲数（0xFC）。 */
    Status fastPosition(uint8_t address, int32_t pulses,
                        uint32_t timeoutMs = 300);

    /**
     * @brief 5.5.1 启停定时返回信息（0x11/0x18）。
     * @param infoFunction 要周期返回的查询功能码，例如 0x36 或 0x3A。
     * @param intervalMs 周期，0 表示停止定时返回。
     */
    Status timedInfo(uint8_t address, uint8_t infoFunction,
                     uint16_t intervalMs, uint32_t timeoutMs = 300);

    /** @brief 5.5.2 读取固件版本和硬件版本（0x1F）。 */
    Status readVersion(uint8_t address, VersionInfo &info,
                       uint32_t timeoutMs = 300);
    /** @brief 5.5.3 读取相电阻和相电感（0x20）。 */
    Status readPhaseParameters(uint8_t address, PhaseParameters &params,
                               uint32_t timeoutMs = 300);
    /** @brief 5.5.4 读取总线电压，单位 mV（0x24）。 */
    Status readBusVoltage(uint8_t address, uint16_t &millivolts,
                          uint32_t timeoutMs = 300);
    /** @brief 5.5.6 读取相电流，单位 mA（0x27）。 */
    Status readPhaseCurrent(uint8_t address, uint16_t &milliamps,
                            uint32_t timeoutMs = 300);
    /** @brief 5.5.7 读取线性化编码器单圈值（0x31）。 */
    Status readEncoder(uint8_t address, uint16_t &encoder,
                       uint32_t timeoutMs = 300);
    /** @brief 5.5.8 读取输入脉冲数（0x32）。 */
    Status readInputPulses(uint8_t address, int32_t &pulses,
                           uint32_t timeoutMs = 300);
    /** @brief 5.5.5 读取总线电流，单位 mA（0x26）。 */
    Status readBusCurrent(uint8_t address, uint16_t &milliamps,
                          uint32_t timeoutMs = 300);
    /** @brief 5.5.17 读取 En/Stp/Dir 引脚状态（0x3D）。 */
    Status readIoState(uint8_t address, IoState &state,
                       uint32_t timeoutMs = 300);
    /**
     * @brief 读取驱动器实时温度（手册 5.5.12，功能码 0x39）。
     * @param temperatureC 输出温度，单位为摄氏度；手册规定符号 0 表示负温度，
     *                     符号 1 表示正温度。
     * @details 该命令属于 X42S/Y42 通用诊断命令，EMM 固件支持时可用于监视驱动器
     *          过热风险；不支持的固件会返回拒绝状态。
     */
    Status readTemperature(uint8_t address, int16_t &temperatureC,
                           uint32_t timeoutMs = 300);
    /** @brief 5.6.4 读取选项状态（0x1A）。 */
    Status readOptionFlags(uint8_t address, OptionFlags &flags,
                           uint32_t timeoutMs = 300);

    /** @brief 5.6.20 读取位置到达窗口（0x41），单位为驱动器原始窗口值。 */
    Status readPositionWindow(uint8_t address, uint16_t &window,
                              uint32_t timeoutMs = 300);
    /**
     * @brief 5.6.22 读取过热、过流保护阈值（0x13）。
     * @details 返回温度阈值（°C）、电流阈值（mA）和持续时间（ms）。
     */
    struct ProtectionThresholds {
        uint16_t overTemperatureC;
        uint16_t overCurrentMa;
        uint16_t durationMs;
    };
    /** @brief 手册 5.6.18/5.6.19 的 DMX512 参数集合。 */
    struct DmxParameters {
        uint16_t channelCount;
        uint8_t channelsPerMotor;
        uint8_t motionMode;
        uint16_t singleSpeedRpm;
        uint16_t acceleration;
        uint16_t dualSpeedStep;
        uint32_t dualMotionStep;
    };
    /** @brief 5.6.18 读取 DMX512 协议参数（0x49/0x78）。 */
    Status readDmxParameters(uint8_t address, DmxParameters &params,
                             uint32_t timeoutMs = 300);
    /**
     * @brief 5.6.22 读取过热、过流保护阈值（0x13）。
     * @details 将返回帧中的三个大端 16 位字段依次解析为温度、电流和持续时间。
     */
    Status readProtectionThresholds(uint8_t address,
                                    ProtectionThresholds &thresholds,
                                    uint32_t timeoutMs = 300);
    /** @brief 5.6.24 读取心跳保护时间（0x16），单位为毫秒；0 表示关闭。 */
    Status readHeartbeatTimeout(uint8_t address, uint32_t &timeoutMsValue,
                                uint32_t timeoutMs = 300);
    /** @brief 5.6.26 读取积分限幅/刚性系数（0x23）。 */
    Status readIntegralLimit(uint8_t address, uint32_t &value,
                             uint32_t timeoutMs = 300);
    /** @brief 5.6.28 读取碰撞回零返回角度（0x3F），单位为 0.1°。 */
    Status readCollisionReturnAngle(uint8_t address, uint16_t &angleTenthDeg,
                                    uint32_t timeoutMs = 300);

    /** @brief 5.6.1 修改电机 ID/地址（0xAE/0x4B）。 */
    Status modifyAddress(uint8_t address, bool save, uint8_t newAddress,
                         uint32_t timeoutMs = 300);
    /** @brief 5.6.2 修改细分值（0x84/0x8A），0 表示 256 细分。 */
    Status modifyMicrostep(uint8_t address, bool save, uint8_t microstep,
                           uint32_t timeoutMs = 300);
    /** @brief 5.6.3 修改掉电标志（0x50）。 */
    Status modifyPowerLossFlag(uint8_t address, bool flag,
                               uint32_t timeoutMs = 300);
    /** @brief 5.6.5 修改电机类型（0xD7/0x35），19=1.8°，32=0.9°。 */
    Status modifyMotorType(uint8_t address, bool save, uint8_t motorType,
                           uint32_t timeoutMs = 300);
    /** @brief 5.6.6 修改固件类型（0xD5/0x69），1=Emm，2=Emm 狂暴模式。 */
    Status modifyFirmwareType(uint8_t address, bool save, uint8_t firmwareType,
                              uint32_t timeoutMs = 300);
    /** @brief 5.6.7 修改开环/闭环控制模式（0x46/0x69）。 */
    Status modifyControlMode(uint8_t address, bool save, uint8_t controlMode,
                             uint32_t timeoutMs = 300);
    /** @brief 5.6.8 修改电机运动正方向（0xD4/0x60）。 */
    Status modifyDirection(uint8_t address, bool save, uint8_t direction,
                           uint32_t timeoutMs = 300);
    /** @brief 5.6.9 修改按键锁定状态（0xD0/0xB3）。 */
    Status modifyKeyLock(uint8_t address, bool save, bool locked,
                         uint32_t timeoutMs = 300);
    /** @brief 5.6.11 修改 Emm 命令速度是否按 0.1RPM 缩放（0x4F/0x71）。 */
    Status modifySpeedScale(uint8_t address, bool save, bool scale10x,
                            uint32_t timeoutMs = 300);
    /** @brief 5.6.12 修改开环工作电流，单位 mA（0x44/0x33）。 */
    Status modifyOpenCurrent(uint8_t address, bool save, uint16_t milliamps,
                             uint32_t timeoutMs = 300);
    /** @brief 5.6.13 修改闭环最大/堵转最大电流，单位 mA（0x45/0x66）。 */
    Status modifyMaxCurrent(uint8_t address, bool save, uint16_t milliamps,
                            uint32_t timeoutMs = 300);
    /** @brief 5.6.31 修改锁定参数等级（0xD6/0x4B）。 */
    Status modifyLockLevel(uint8_t address, bool save, uint8_t level,
                           uint32_t timeoutMs = 300);
    /** @brief 5.6.21 修改位置到达窗口（0xD1/0x07）。 */
    Status modifyPositionWindow(uint8_t address, bool save, uint16_t window,
                                uint32_t timeoutMs = 300);
    /**
     * @brief 5.6.23 修改过热、过流保护阈值（0xD3/0x56）。
     * @param thresholds 温度、电流和持续时间阈值；具体单位见结构体注释。
     */
    Status modifyProtectionThresholds(uint8_t address, bool save,
                                      const ProtectionThresholds &thresholds,
                                      uint32_t timeoutMs = 300);
    /** @brief 5.6.25 修改心跳保护时间（0x68/0x38），单位为毫秒。 */
    Status modifyHeartbeatTimeout(uint8_t address, bool save,
                                  uint32_t heartbeatMs,
                                  uint32_t timeoutMs = 300);
    /** @brief 5.6.27 修改积分限幅/刚性系数（0x4B/0x57）。 */
    Status modifyIntegralLimit(uint8_t address, bool save, uint32_t value,
                               uint32_t timeoutMs = 300);
    /** @brief 5.6.29 修改碰撞回零返回角度（0x5C/0xAC），单位为 0.1°。 */
    Status modifyCollisionReturnAngle(uint8_t address, bool save,
                                      uint16_t angleTenthDeg,
                                      uint32_t timeoutMs = 300);
    /** @brief 5.6.19 修改 DMX512 协议参数（0xD9/0x90）。 */
    Status modifyDmxParameters(uint8_t address, bool save,
                               const DmxParameters &params,
                               uint32_t timeoutMs = 300);

    /**
     * @brief 5.6.30 广播读取未知电机 ID（地址 0、功能码 0x15）。
     * @param detectedAddress 输出收到响应的电机地址。
     * @details 手册要求总线上只能连接待查找的一台电机，否则多个响应会发生碰撞；
     *          因而该接口仅用于单电机诊断，不用于正常多机扫描。
     */
    Status readBroadcastAddress(uint8_t &detectedAddress,
                                uint32_t timeoutMs = 300);

    /** @brief 5.6.16 读取 Emm PID 参数（0x21）。 */
    Status readPid(uint8_t address, PidParameters &pid,
                   uint32_t timeoutMs = 300);
    /** @brief 5.6.17 修改 Emm PID 参数（0x4A/0xC3）。 */
    Status modifyPid(uint8_t address, bool save, const PidParameters &pid,
                     uint32_t timeoutMs = 300);
    /** @brief 5.7.2 设置/清除上电自动运行速度参数（0xF7/0x1C）。 */
    Status modifyAutoSpeed(uint8_t address, bool save, uint8_t direction,
                           uint16_t rpm, uint8_t acceleration, bool enControl,
                           uint32_t timeoutMs = 300);

    /**
     * @brief 5.8.5 读取 Emm 驱动配置原始帧（0x42/0x6C）。
     * @param response 输出完整 33 字节配置响应。
     */
    Status readDriverConfig(uint8_t address, uint8_t *response,
                            size_t responseSize, uint32_t timeoutMs = 300);
    /**
     * @brief 5.8.6 修改 Emm 驱动配置原始帧（0x48/0xD1）。
     * @param payload 不含地址、功能码、辅助码和帧尾的配置字段，共 29 字节。
     */
    Status modifyDriverConfig(uint8_t address, bool save,
                              const uint8_t *payload, size_t payloadLength,
                              uint32_t timeoutMs = 300);

    /**
     * @brief 5.3.1 发送 Emm 多电机复合命令（0xAA）。
     * @param commandBytes 不含地址 0x00、功能码 0xAA、长度和帧尾的子命令字节。
     * @param commandLength 子命令总长度，最大 250 字节。
     * @note 复合帧的每条子命令仍需按手册格式以 0x6B 结束；电机 1 可能返回确认帧。
     */
    Status multiMotorCommand(const uint8_t *commandBytes, size_t commandLength,
                             uint32_t timeoutMs = 300);

    /**
     * @brief 5.4.6 修改回零参数（0x4C/0xAE）。
     * @details 同时配置回零模式、方向、速度、超时、碰撞检测转速/电流/时间和上电自动回零。
     */
    Status modifyHomeParams(uint8_t address, bool save, uint8_t mode,
                            uint8_t direction, uint16_t speedRpm,
                            uint32_t timeoutMs, uint16_t stallSpeedRpm,
                            uint16_t stallCurrentMa, uint16_t stallTimeMs,
                            bool powerOnHome, uint32_t timeout = 300);

    /** @brief 向地址 0 广播使能控制。 */
    Status enableBroadcast(bool enabled, bool synchronized = false);
    /** @brief 向地址 0 广播速度控制。 */
    Status velocityBroadcast(uint8_t direction, uint16_t rpm,
                             uint8_t acceleration, bool synchronized = false);
    /** @brief 兼容接口：广播位置模式。 */
    Status positionBroadcast(uint8_t direction, uint16_t rpm,
                             uint8_t acceleration, uint32_t pulses,
                             bool absolute, bool synchronized = false);
    /** @brief 广播 Emm MODE=0/1/2 位置控制。 */
    Status positionBroadcastMode(uint8_t direction, uint16_t rpm,
                                 uint8_t acceleration, uint32_t pulses,
                                 uint8_t motionMode,
                                 bool synchronized = false);
    /** @brief 广播立即停止。 */
    Status stopBroadcast(bool synchronized = false);
    /** @brief 广播触发回零。 */
    Status homeBroadcast(uint8_t mode, bool synchronized = false);
    /** @brief 广播中断回零。 */
    Status interruptHomeBroadcast();
    /** @brief 广播当前位置清零。 */
    Status resetCurrentPositionBroadcast();
    /** @brief 触发指定电机执行之前缓存的同步命令。 */
    Status synchronousMotion(uint8_t address, uint32_t timeoutMs = 300);
    /** @brief 广播触发同步执行。 */
    Status synchronousMotionBroadcast();

    /** @brief 5.5.13 读取实时位置原始值。 */
    Status readCurrentPosition(uint8_t address, int32_t &position,
                               uint32_t timeoutMs = 300);
    /** @brief 5.5.11 读取实时转速。 */
    Status readVelocity(uint8_t address, int16_t &rpm,
                        uint32_t timeoutMs = 300);
    /** @brief 5.5.9 读取目标位置。 */
    Status readTargetPosition(uint8_t address, int32_t &position,
                              uint32_t timeoutMs = 300);
    /** @brief 5.5.10 读取实时设定目标位置。 */
    Status readCommandPosition(uint8_t address, int32_t &position,
                               uint32_t timeoutMs = 300);
    /** @brief 5.5.14 读取位置误差。 */
    Status readPositionError(uint8_t address, int32_t &error,
                             uint32_t timeoutMs = 300);
    /** @brief 5.5.15 读取电机状态标志。 */
    Status readMotorFlags(uint8_t address, uint8_t &flags,
                          uint32_t timeoutMs = 300);
    /**
     * @brief 5.4.4 读取回零状态标志（0x3B）。
     * @details 返回值可使用 OriginFlag 位掩码解析；该接口只读取一次，不会等待
     *          回零动作结束。持续状态跟踪由命令层的回零监控任务完成。
     */
    Status readOriginFlags(uint8_t address, uint8_t &flags,
                           uint32_t timeoutMs = 300);
    /** @brief 5.4.5 读取回零参数。 */
    Status readHomeParams(uint8_t address, uint8_t &mode,
                          uint8_t &direction, uint16_t &speedRpm,
                          uint32_t &homeTimeoutMs, uint16_t &stallSpeedRpm,
                          uint16_t &stallCurrentMa, uint16_t &stallTimeMs,
                          bool &powerOnHome, uint32_t timeoutMs = 300);
    /** @brief 5.5.16 同时读取回零和电机状态标志。 */
    Status readCombinedFlags(uint8_t address, uint8_t &originFlags,
                             uint8_t &motorFlags, uint32_t timeoutMs = 300);
    /** @brief 5.8.2 读取 Emm 系统状态参数。 */
    Status readSystemState(uint8_t address, SystemState &state,
                           uint32_t timeoutMs = 300);

    /**
     * @brief 将驱动状态码转换为稳定的 ASCII 名称。
     * @param status EmmV5::Status 状态码。
     * @return 便于上位机回复和日志记录的常量字符串。
     */
    static const char *statusToString(Status status);

private:
    HardwareSerial *_serial;
    uint8_t _address;
    bool _begun;
    bool _debugEnabled;
    Response _lastResponse;

    /**
     * @brief 发送单地址控制帧并等待驱动器应答。
     * @details 响应格式为 [地址][功能码][状态][0x6B]。状态映射为：
     *          0x02 → 命令被接受；0x9F → 动作已完成（requireDone=true 时返回
     *          STATUS_DONE，否则按历史语义返回 STATUS_OK）；0x12/0x22 → 合法但
     *          电机不动；其它（0xE2/0xEE 等）→ 拒绝。
     * @param requireDone true 时收到 0x02 后不返回，继续在同一个接收循环内等 0x9F；
     *        等到返回 STATUS_DONE，只等到 0x02 而超时返回 STATUS_OK。
     *        @note 必须在同一个循环内等完——若先返回再等，残留的 0x9F 会被下一次
     *        clearRxBuffer() 清掉，永远收不到。
     */
    Status writeAndReadAck(uint8_t address, uint8_t function,
                           const uint8_t *frame, size_t frameLength,
                           uint32_t timeoutMs, bool requireDone = false);
    /**
     * @brief 发送广播帧或无需应答的帧。
     * @note 地址 0 广播后多个电机可能同时动作，因此协议层不等待响应。
     */
    Status writeFrameNoAck(const uint8_t *frame, size_t frameLength);
    /**
     * @brief 发送查询帧并从串口流中提取固定长度响应。
     * @details 函数允许串口中夹杂无关字节，并通过地址、功能码和帧尾
     *          重新同步；同时识别 [地址][0x00][0xEE][0x6B] 拒绝帧。
     */
    Status readFixed(uint8_t address, uint8_t function,
                     const uint8_t *extra, size_t extraLength,
                     uint8_t *response, size_t expectedLength,
                     uint32_t timeoutMs);
    /** @brief 读取“符号字节+大端 32 位幅值”格式的有符号参数。 */
    Status readSigned32(uint8_t address, uint8_t function, int32_t &value,
                        uint32_t timeoutMs);
    /** @brief 组装并发送带辅助字段的单播控制帧。 */
    Status writePayloadAck(uint8_t address, uint8_t function,
                           const uint8_t *payload, size_t payloadLength,
                           uint32_t timeoutMs);
    /** @brief 读取固定长度响应中的大端 16 位字段。 */
    Status readUnsigned16(uint8_t address, uint8_t function, uint16_t &value,
                          uint32_t timeoutMs);
    /**
     * @brief 发送 5.6.30 广播读取 ID 帧并等待任意地址响应。
     * @details 与 readFixed() 不同，该事务发送地址 0 且响应地址未知，必须在串口流中
     *          按 [非零地址][0x15][同地址][0x6B] 模式重新同步。
     */
    Status readBroadcastAddressFrame(uint8_t &detectedAddress,
                                     uint32_t timeoutMs);
    /** @brief 丢弃上一次事务遗留的串口输入，防止旧响应被误配对。 */
    void clearRxBuffer();
    /** @brief 按十六进制打印协议帧，仅在调试开关打开时生效。 */
    void logFrame(const char *prefix, const uint8_t *data, size_t length) const;
    /** @brief 将原始响应复制到 lastResponse，并提取地址、功能码和状态字段。 */
    void saveResponse(const uint8_t *data, size_t length);
    /** @brief 判断地址是否为单播地址（地址 0 专用于广播）。 */
    static bool isValidAddress(uint8_t address);
    /** @brief 将 EMM 的符号-幅值编码转换为 int32_t，并检查溢出。 */
    static bool decodeSignedMagnitude(uint8_t sign, uint32_t magnitude,
                                      int32_t &value);
    /** @brief 读取两个字节的大端无符号整数。 */
    static uint16_t be16(const uint8_t *p);
    /** @brief 读取四个字节的大端无符号整数。 */
    static uint32_t be32(const uint8_t *p);
};

#endif

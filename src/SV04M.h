#ifndef SV04M_H
#define SV04M_H

#include <Arduino.h>

/**
 * @brief 南京润泽 SV-04M 多通道切换阀 Arduino ESP32 驱动
 *
 * 协议：SV-04M 自定义协议
 * 接口：RS485，自动收发 485 芯片无需 DE/RE 方向控制脚
 * 默认：地址 0x00，波特率 9600，8N1
 */
class SV04M
{
public:
    /**
     * @brief SV-04M 响应状态码
     */
    enum Status : int16_t
    {
        STATUS_OK               = 0x00,  ///< 状态正常
        STATUS_FRAME_ERROR      = 0x01,  ///< 帧错误
        STATUS_PARAM_ERROR      = 0x02,  ///< 参数错误
        STATUS_OPTO_ERROR       = 0x03,  ///< 光耦错误
        STATUS_MOTOR_BUSY       = 0x04,  ///< 电机忙
        STATUS_MOTOR_STALL      = 0x05,  ///< 电机堵转
        STATUS_UNKNOWN_POSITION = 0x06,  ///< 未知位置
        STATUS_REJECTED         = 0x07,  ///< 指令被拒绝
        STATUS_TASK_PENDING     = 0xFE,  ///< 任务挂起，RS485 动作指令已收到并正在执行
        STATUS_UNKNOWN_ERROR    = 0xFF,  ///< 未知错误

        STATUS_TIMEOUT          = -1,    ///< 接收超时
        STATUS_CHECKSUM_ERROR   = -2,    ///< 和校验错误
        STATUS_FRAME_FORMAT_ERR = -3,    ///< 帧头/帧尾格式错误
        STATUS_INVALID_ARGUMENT = -4,    ///< 函数输入参数错误
        STATUS_SERIAL_NOT_BEGIN = -5     ///< 串口未初始化
    };

    /**
     * @brief 波特率索引，对应手册中的 B7/B3B4 参数值
     */
    enum BaudIndex : uint8_t
    {
        BAUD_9600   = 0x00,
        BAUD_19200  = 0x01,
        BAUD_38400  = 0x02,
        BAUD_57600  = 0x03,
        BAUD_115200 = 0x04
    };

    /**
     * @brief 一帧 8 字节响应数据解析结果
     */
    struct Response
    {
        uint8_t raw[8];       ///< 原始响应帧
        uint8_t address;      ///< 响应地址 B1
        uint8_t statusByte;   ///< 状态码 B2
        uint16_t param;       ///< 参数 B3/B4，小端格式
        Status status;        ///< 解析后的状态
    };

    explicit SV04M(HardwareSerial &serial);

    /**
     * @brief 初始化串口
     * @param baud 波特率，出厂通常为 9600
     * @param rxPin ESP32 接收脚。你的接线为 IO19 接 485 RX/RO 时填 19
     * @param txPin ESP32 发送脚。你的接线为 IO20 接 485 TX/DI 时填 20
     * @param address 阀地址，出厂默认 0x00
     * @param channelCount 阀通道数，按实际阀头填写，例如 6/8/10/12/16/24/28
     */
    void begin(uint32_t baud = 9600,
               int8_t rxPin = 19,
               int8_t txPin = 20,
               uint8_t address = 0x00,
               uint8_t channelCount = 10);

    void setAddress(uint8_t address);
    uint8_t address() const;

    void setChannelCount(uint8_t channelCount);
    uint8_t channelCount() const;

    /**
     * @brief 打开或关闭 ESP_LOG 调试输出
     * @note 打开后会通过 ESP_LOGI/ESP_LOGE 输出 TX/RX 帧和错误信息。
     */
    void setDebugEnabled(bool enabled);
    bool debugEnabled() const;

    /**
     * @brief 兼容旧版本接口
     * @note 新版本不再使用 Serial/Stream 打印；debug != nullptr 时等效于 setDebugEnabled(true)。
     */
    void setDebugStream(Stream *debug);

    const Response &lastResponse() const;

    /**
     * @brief 发送普通 8 字节指令
     * @param func 功能码 B2
     * @param param 参数 B3/B4，小端发送
     * @param response 响应结构体，可传 nullptr
     * @param timeoutMs 接收超时时间
     */
    Status sendCommand(uint8_t func,
                       uint16_t param,
                       Response *response = nullptr,
                       uint32_t timeoutMs = 1000);

    /**
     * @brief 发送工厂 14 字节指令，内置密码 FF EE BB AA
     * @note 设地址、设波特率、设上电自动复位等属于工厂指令，请谨慎使用。
     */
    Status sendFactoryCommand(uint8_t func,
                              uint32_t param,
                              Response *response = nullptr,
                              uint32_t timeoutMs = 1000);

    /**
     * @brief 切换到指定通道，使用 0x44 自动选择最优路径
     */
    Status moveToChannel(uint8_t channel,
                         bool waitDone = true,
                         uint32_t timeoutMs = 6000);

    /**
     * @brief 原点复位，复位到 1 号孔，使用 0x45
     */
    Status resetToHome(bool waitDone = true,
                       uint32_t timeoutMs = 6000);

    /**
     * @brief 强停，使用 0x49
     */
    Status stop();

    /**
     * @brief 按需求方向切换孔位，使用 0xA4
     * @param b3 手册中的 B3 参数
     * @param b4 手册中的 B4 参数
     * @note 手册写“参数 0x0304”时，调用 moveByDirection(0x04, 0x03)。
     */
    Status moveByDirection(uint8_t b3,
                           uint8_t b4,
                           bool waitDone = true,
                           uint32_t timeoutMs = 6000);

    /**
     * @brief 按需求方向切换到两个孔位之间，使用 0xB4
     * @param b3 手册中的 B3 参数
     * @param b4 手册中的 B4 参数
     */
    Status moveToBetween(uint8_t b3,
                         uint8_t b4,
                         bool waitDone = true,
                         uint32_t timeoutMs = 6000);

    /**
     * @brief 查询当前通道位置，使用 0x3E
     */
    Status queryCurrentChannel(uint8_t &channel);

    /**
     * @brief 查询电机状态，使用 0x4A
     */
    Status queryMotorStatus();

    /**
     * @brief 轮询 0x4A，等待电机状态变为 STATUS_OK
     */
    Status waitUntilIdle(uint32_t timeoutMs = 6000,
                         uint16_t pollIntervalMs = 100);

    Status queryAddress(uint8_t &addr);
    Status queryRS485Baud(BaudIndex &baudIndex);
    Status queryVersion(uint8_t &major, uint8_t &minor);
    Status queryAutoReset(bool &enabled);

    Status setDeviceAddress(uint8_t newAddress);
    Status setRS485Baud(BaudIndex baudIndex);
    Status setAutoReset(bool enabled);
    Status setGroupAddress(uint8_t groupIndex, uint8_t groupAddress);
    Status lockParameters();
    Status factoryReset();

    static const char *statusToString(Status status);
    static uint32_t baudFromIndex(BaudIndex baudIndex);

private:
    HardwareSerial *_serial;
    uint8_t _address;
    uint8_t _channelCount;
    bool _begun;
    bool _debugEnabled;
    Response _lastResponse;

    void clearRxBuffer();
    static uint16_t sum16(const uint8_t *data, size_t len);
    static Status statusFromByte(uint8_t statusByte);

    Status writeAndRead(const uint8_t *frame,
                        size_t len,
                        Response *response,
                        uint32_t timeoutMs);

    /**
     * @brief 只负责读取一帧合法 8 字节响应，返回值表示读帧是否成功
     */
    Status readFrame(Response &response, uint32_t timeoutMs);

    void logFrame(const char *prefix, const uint8_t *data, size_t len);
};

#endif

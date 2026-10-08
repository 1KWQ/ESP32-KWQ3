#ifndef GFLZ_LASER_H
#define GFLZ_LASER_H

#include <Arduino.h>

/**
 * @brief DADISICK / 大迪 GFL-Z 系列 RS485 激光位移传感器 Arduino ESP32 驱动
 *
 * 协议：Modbus RTU
 * 接口：RS485，自动收发 485 芯片无需 DE/RE 方向控制脚
 * 读取距离：功能码 0x04，起始寄存器 0x0000，读取 2 个寄存器，返回 32 位距离值，单位 um
 *
 * 注意：若和 SV-04M 共用同一条 RS485 总线，所有设备波特率、数据位、校验位、停止位必须一致，
 *      并且每个激光测距设备地址必须不同。
 */
class GFLZLaser
{
public:
    enum Status : int16_t
    {
        STATUS_OK               = 0,    ///< 通信和解析正常
        STATUS_TIMEOUT          = -1,   ///< 接收超时
        STATUS_CRC_ERROR        = -2,   ///< CRC 校验错误
        STATUS_FRAME_ERROR      = -3,   ///< 帧格式错误
        STATUS_EXCEPTION        = -4,   ///< Modbus 异常响应
        STATUS_INVALID_ARGUMENT = -5,   ///< 参数错误
        STATUS_SERIAL_NOT_BEGIN = -6    ///< 串口未初始化
    };

    struct DistanceData
    {
        uint8_t address;        ///< 传感器 Modbus 地址
        int32_t distanceUm;     ///< 距离原始值，单位 um
        float distanceMm;       ///< 距离换算值，单位 mm
        uint8_t raw[9];         ///< 读取距离的原始响应帧，通常 9 字节
        uint8_t rawLength;      ///< 原始响应长度
    };

    explicit GFLZLaser(HardwareSerial &serial);

    /**
     * @brief 使用本类初始化串口。
     * @note 如果同一条总线已经由 SV04M::begin() 初始化，通常不需要再调用这个重载。
     */
    void begin(uint32_t baud,
               int8_t rxPin,
               int8_t txPin,
               uint8_t defaultAddress = 0x01);

    /**
     * @brief 只设置默认地址并标记串口已可用。
     * @note 用于和 SV-04M 共享同一个 HardwareSerial，且串口已经被 valve.begin() 初始化的场景。
     */
    void begin(uint8_t defaultAddress = 0x01);

    void setAddress(uint8_t address);
    uint8_t address() const;

    void setDebugEnabled(bool enabled);
    bool debugEnabled() const;

    uint8_t lastExceptionCode() const;
    const DistanceData &lastDistance() const;

    /**
     * @brief 读取默认地址传感器的距离，单位 um
     */
    Status readDistanceUm(int32_t &distanceUm, uint32_t timeoutMs = 200);

    /**
     * @brief 读取指定地址传感器的距离，单位 um
     */
    Status readDistanceUm(uint8_t address, int32_t &distanceUm, uint32_t timeoutMs = 200);

    /**
     * @brief 读取默认地址传感器的距离，单位 mm
     */
    Status readDistanceMm(float &distanceMm, uint32_t timeoutMs = 200);

    /**
     * @brief 读取指定地址传感器的距离，单位 mm
     */
    Status readDistanceMm(uint8_t address, float &distanceMm, uint32_t timeoutMs = 200);

    /**
     * @brief 读取指定地址传感器的完整距离数据
     */
    Status readDistance(uint8_t address, DistanceData &data, uint32_t timeoutMs = 200);

    /**
     * @brief 通用 Modbus 0x04 读输入寄存器
     */
    Status readInputRegisters(uint8_t address,
                              uint16_t startRegister,
                              uint16_t registerCount,
                              uint8_t *outData,
                              size_t outDataSize,
                              uint32_t timeoutMs = 200);

    static const char *statusToString(Status status);
    static uint16_t crc16(const uint8_t *data, size_t len);

private:
    HardwareSerial *_serial;
    uint8_t _address;
    bool _begun;
    bool _debugEnabled;
    uint8_t _lastExceptionCode;
    DistanceData _lastDistance;

    void clearRxBuffer();
    void logFrame(const char *prefix, const uint8_t *data, size_t len);

    Status writeAndReadModbus(uint8_t address,
                              uint8_t function,
                              const uint8_t *request,
                              size_t requestLen,
                              uint8_t expectedByteCount,
                              uint8_t *response,
                              size_t responseSize,
                              size_t &responseLen,
                              uint32_t timeoutMs);

    static bool isValidAddress(uint8_t address);
    static int32_t parseSigned32BigEndian(const uint8_t *data);
};

#endif

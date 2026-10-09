#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#endif

#include "EmmV5.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

/**
 * @file EmmV5.cpp
 * @brief ZDT X42S/Emm V5 自由协议的组帧、收发和字段解码实现。
 *
 * 实现分为四层：
 * 1. 公共控制/查询接口：每个函数对应用户手册中的一条命令；
 * 2. 广播接口：地址固定为 0，写出后不等待多台电机同时应答；
 * 3. 通用事务函数：统一处理清空旧数据、发送、超时和滑动窗口匹配；
 * 4. 字节序/符号解码函数：集中处理大端整数和 EMM 特有的“符号+幅值”格式。
 *
 * 人类可读的“命令开始/成功/失败”日志由 CommandDispatcher 输出；本文件的
 * 原始十六进制帧日志由 setDebugEnabled(true) 控制，默认关闭，避免占用串口带宽。
 */
static const char *const TAG = "EMM_V5";
/** 所有自由协议帧都以 0x6B 结束；它是帧尾而不是 CRC。 */
static const uint8_t FRAME_END = 0x6B;
/** 普通命令执行成功时驱动器返回的状态码。 */
static const uint8_t ACK_OK = 0x02;
/** 部分运动命令在目标已经到达时返回的成功状态码；也是主动上报"动作完成"的状态码。 */
static const uint8_t ACK_REACHED = 0x9F;
/**
 * 触发回零时"已在零点"（0x12）与"左右限位已触发"（0x22）。
 * @details 手册 4.1.2 明确这两种应答表示电机不动，但**不是错误**；因此既不能
 *          当成 STATUS_OK（动作并没做完），也不能当成 STATUS_REJECTED（命令没错），
 *          这里单独映射为 STATUS_NOT_MOVED 交上层判断。
 */
static const uint8_t ACK_AT_ZERO = 0x12;
static const uint8_t ACK_LIMIT_TRIGGERED = 0x22;

/* ==================== 对象初始化与公共状态 ==================== */

/**
 * @brief 构造 EMM V5 驱动对象。
 * @details 仅保存共享硬件串口的引用，并清空最近响应缓存；串口参数由
 *          Rs485Devices_Init() 统一配置，避免同一 RS485 总线被重复初始化。
 */
EmmV5::EmmV5(HardwareSerial &serial)
    : _serial(&serial), _address(0x02), _begun(false), _debugEnabled(false)
{
    memset(&_lastResponse, 0, sizeof(_lastResponse));
}

/**
 * @brief 启用协议层并设置默认地址。
 * @param defaultAddress 默认单播地址，地址 0 为广播地址，不能作为默认单播地址。
 * @note 此函数不会再次调用 HardwareSerial.begin()，只清理旧输入数据。
 */
void EmmV5::begin(uint8_t defaultAddress)
{
    if (isValidAddress(defaultAddress)) _address = defaultAddress;
    _begun = true;
    clearRxBuffer();
}

/** @brief 设置默认单播地址；非法地址 0 会被忽略。 */
void EmmV5::setAddress(uint8_t address) { if (isValidAddress(address)) _address = address; }
/** @brief 返回当前默认单播地址。 */
uint8_t EmmV5::address() const { return _address; }
/** @brief 开启或关闭协议原始帧日志。 */
void EmmV5::setDebugEnabled(bool enabled) { _debugEnabled = enabled; }
/** @brief 返回协议原始帧日志是否开启。 */
bool EmmV5::debugEnabled() const { return _debugEnabled; }
/** @brief 返回最近一次匹配到的响应帧缓存。 */
const EmmV5::Response &EmmV5::lastResponse() const { return _lastResponse; }

/* ==================== 基础运动、回零与维护命令 ==================== */

/**
 * @brief 发送 0xF3 使能/失能帧。
 * @details 数据字段为固定子命令 0xAB、使能值和同步标志，单播时等待 ACK。
 */
EmmV5::Status EmmV5::enable(uint8_t address, bool enabled,
                            bool synchronized, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0xF3, 0xAB, (uint8_t)enabled,
                             (uint8_t)synchronized, FRAME_END};
    return writeAndReadAck(address, 0xF3, frame, sizeof(frame), timeoutMs);
}

/**
 * @brief 发送 0xF6 速度模式帧。
 * @param direction 方向位，允许 0 或 1。
 * @param rpm 目标转速，当前驱动限制为 0~3000 RPM。
 * @param acceleration 1 字节加速度参数，范围由调用方约束为 0~255。
 */
EmmV5::Status EmmV5::velocity(uint8_t address, uint8_t direction,
                              uint16_t rpm, uint8_t acceleration,
                              bool synchronized, uint32_t timeoutMs)
{
    if (!isValidAddress(address) || direction > 1 || rpm > 3000) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t frame[] = {address, 0xF6, direction,
                             (uint8_t)(rpm >> 8), (uint8_t)rpm,
                             acceleration, (uint8_t)synchronized, FRAME_END};
    return writeAndReadAck(address, 0xF6, frame, sizeof(frame), timeoutMs);
}

/**
 * @brief 兼容接口：将 absolute 参数映射为位置 MODE=0 或 MODE=1。
 * @note 需要使用手册定义的 MODE=2 时请直接调用 positionMode()。
 */
EmmV5::Status EmmV5::position(uint8_t address, uint8_t direction,
                              uint16_t rpm, uint8_t acceleration,
                              uint32_t pulses, bool absolute,
                              bool synchronized, uint32_t timeoutMs)
{
    return positionMode(address, direction, rpm, acceleration, pulses,
                        absolute ? 1 : 0, synchronized, timeoutMs);
}

/**
 * @brief 发送 0xFD 位置模式帧。
 * @details 脉冲数采用大端 32 位编码；MODE 允许 0、1、2。
 *          synchronized=true 时仅缓存运动命令，需另发 0xFF 同步触发。
 */
EmmV5::Status EmmV5::positionMode(uint8_t address, uint8_t direction,
                                  uint16_t rpm, uint8_t acceleration,
                                  uint32_t pulses, uint8_t motionMode,
                                  bool synchronized, uint32_t timeoutMs,
                                  bool requireDone)
{
    if (!isValidAddress(address) || direction > 1 || rpm > 3000 ||
        motionMode > 2) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {
        address, 0xFD, direction, (uint8_t)(rpm >> 8), (uint8_t)rpm,
        acceleration, (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)pulses, motionMode,
        (uint8_t)synchronized, FRAME_END};
    return writeAndReadAck(address, 0xFD, frame, sizeof(frame), timeoutMs,
                           requireDone);
}

/**
 * @brief 发送 0xFE/0x98 停止帧。
 * @note 停止动作不会自动清零位置，也不会修改回零参数。
 */
EmmV5::Status EmmV5::stop(uint8_t address, bool synchronized,
                          uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0xFE, 0x98, (uint8_t)synchronized,
                             FRAME_END};
    return writeAndReadAck(address, 0xFE, frame, sizeof(frame), timeoutMs);
}

/**
 * @brief 发送手册 5.4.2 的 0x9A 回零触发帧。
 * @param mode 回零模式，协议允许 0~5。
 * @param synchronized 是否将回零命令缓存到同步队列。
 */
EmmV5::Status EmmV5::home(uint8_t address, uint8_t mode,
                          bool synchronized, uint32_t timeoutMs,
                          bool requireDone)
{
    if (!isValidAddress(address) || mode > 5) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0x9A, mode, (uint8_t)synchronized,
                             FRAME_END};
    return writeAndReadAck(address, 0x9A, frame, sizeof(frame), timeoutMs,
                           requireDone);
}

/**
 * @brief 发送手册 5.4.3 的 0x9C/0x48 强制退出回零帧。
 */
EmmV5::Status EmmV5::interruptHome(uint8_t address, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0x9C, 0x48, FRAME_END};
    return writeAndReadAck(address, 0x9C, frame, sizeof(frame), timeoutMs);
}

/**
 * @brief 发送 0x0A/0x6D，将当前位置计数清零。
 */
EmmV5::Status EmmV5::resetCurrentPosition(uint8_t address,
                                           uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0x0A, 0x6D, FRAME_END};
    return writeAndReadAck(address, 0x0A, frame, sizeof(frame), timeoutMs);
}

/**
 * @brief 发送手册 5.4.1 的 0x93/0x88 单圈回零零点设置帧。
 * @param save 为 true 时把新零点写入驱动器掉电保存区。
 */
EmmV5::Status EmmV5::setOriginZero(uint8_t address, bool save,
                                   uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0x93, 0x88, (uint8_t)save, FRAME_END};
    return writeAndReadAck(address, 0x93, frame, sizeof(frame), timeoutMs);
}

/**
 * @brief 5.4.6 修改 Emm 固件的回零参数。
 * @param address 电机地址。
 * @param save 是否保存到驱动器掉电参数。
 * @param mode 回零模式，0~5。
 * @param direction 回零方向，0=CW，1=CCW。
 * @param speedRpm 回零速度，单位 RPM。
 * @param timeoutMs 回零超时时间，单位 ms。
 * @param stallSpeedRpm 碰撞回零检测转速。
 * @param stallCurrentMa 碰撞回零检测电流，单位 mA。
 * @param stallTimeMs 碰撞回零检测持续时间，单位 ms。
 * @param powerOnHome 是否上电自动回零。
 */
EmmV5::Status EmmV5::modifyHomeParams(
    uint8_t address, bool save, uint8_t mode, uint8_t direction,
    uint16_t speedRpm, uint32_t timeoutMs, uint16_t stallSpeedRpm,
    uint16_t stallCurrentMa, uint16_t stallTimeMs, bool powerOnHome,
    uint32_t timeout)
{
    /*
     * 0x4C/0xAE 帧包含回零模式、方向、速度、超时时间以及碰撞检测参数。
     * 所有多字节字段均按手册要求以高字节在前的顺序写入。
     */
    if (!isValidAddress(address) || mode > 5 || direction > 1 ||
        speedRpm > 3000 || stallSpeedRpm > 3000 || stallCurrentMa > 5000) {
        return STATUS_INVALID_ARGUMENT;
    }

    const uint8_t frame[] = {
        address, 0x4C, 0xAE, (uint8_t)save, mode, direction,
        (uint8_t)(speedRpm >> 8), (uint8_t)speedRpm,
        (uint8_t)(timeoutMs >> 24), (uint8_t)(timeoutMs >> 16),
        (uint8_t)(timeoutMs >> 8), (uint8_t)timeoutMs,
        (uint8_t)(stallSpeedRpm >> 8), (uint8_t)stallSpeedRpm,
        (uint8_t)(stallCurrentMa >> 8), (uint8_t)stallCurrentMa,
        (uint8_t)(stallTimeMs >> 8), (uint8_t)stallTimeMs,
        (uint8_t)powerOnHome, FRAME_END};
    return writeAndReadAck(address, 0x4C, frame, sizeof(frame), timeout);
}

/**
 * @brief 清除驱动器堵转/过热/过流保护锁存（0x0E/0x52）。
 * @details 该命令只针对单个地址发送并等待 ACK；它不会自动重新执行之前
 *          的运动命令，保护解除后是否继续运动由驱动器固件状态决定。
 */
EmmV5::Status EmmV5::resetClogProtection(uint8_t address,
                                         uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0x0E, 0x52, FRAME_END};
    return writeAndReadAck(address, 0x0E, frame, sizeof(frame), timeoutMs);
}

/**
 * @brief 5.2.1 触发编码器线性化校准（0x06/0x45）。
 * @details 电机需要在安全、空载状态下执行该操作；函数只负责发送命令并等待
 *          驱动器确认，不会等待整个校准运动结束。
 */
EmmV5::Status EmmV5::calibrateEncoder(uint8_t address, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0x45};
    return writePayloadAck(address, 0x06, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.2.2 发送 0x08/0x97 重启驱动器命令。 */
EmmV5::Status EmmV5::restart(uint8_t address, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0x97};
    return writePayloadAck(address, 0x08, payload, sizeof(payload), timeoutMs);
}

/**
 * @brief 5.2.5 发送 0x0F/0x5F 恢复出厂设置命令。
 * @warning 恢复后驱动器会丢失地址、运动和保护参数，通常需要断电重启并重新校准。
 */
EmmV5::Status EmmV5::restoreFactory(uint8_t address, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0x5F};
    return writePayloadAck(address, 0x0F, payload, sizeof(payload), timeoutMs);
}

/**
 * @brief 5.3.13 第一步：设置快速位置模式参数（0xF1）。
 * @details 速度和模式被驱动器缓存，后续 fastPosition() 只需发送 0xFC 脉冲数。
 */
EmmV5::Status EmmV5::fastPositionParams(uint8_t address, uint16_t rpm,
                                         uint8_t acceleration, uint8_t motionMode,
                                         bool synchronized, uint32_t timeoutMs)
{
    if (!isValidAddress(address) || rpm > 3000 || motionMode > 2) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {
        (uint8_t)(rpm >> 8), (uint8_t)rpm, acceleration, motionMode,
        (uint8_t)synchronized
    };
    return writePayloadAck(address, 0xF1, payload, sizeof(payload), timeoutMs);
}

/**
 * @brief 5.3.13 第二步：发送快速位置脉冲数（0xFC）。
 * @details 手册定义该字段为有符号 int32_t，负值按二进制补码大端发送。
 */
EmmV5::Status EmmV5::fastPosition(uint8_t address, int32_t pulses,
                                   uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    uint32_t raw = (uint32_t)pulses;
    const uint8_t payload[] = {
        (uint8_t)(raw >> 24), (uint8_t)(raw >> 16),
        (uint8_t)(raw >> 8), (uint8_t)raw
    };
    return writePayloadAck(address, 0xFC, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.5.1 设置或停止周期信息返回（0x11/0x18）。 */
EmmV5::Status EmmV5::timedInfo(uint8_t address, uint8_t infoFunction,
                                uint16_t intervalMs, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {
        0x18, infoFunction, (uint8_t)(intervalMs >> 8), (uint8_t)intervalMs
    };
    return writePayloadAck(address, 0x11, payload, sizeof(payload), timeoutMs);
}

/* ==================== 运行状态与诊断参数查询 ==================== */

/** @brief 5.5.2 读取固件/硬件版本并解码硬件位域。 */
EmmV5::Status EmmV5::readVersion(uint8_t address, VersionInfo &info,
                                  uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x1F, nullptr, 0, response, 7, timeoutMs);
    if (ret != STATUS_OK) return ret;
    info.firmwareVersion = be16(&response[2]);
    uint16_t hardware = be16(&response[4]);
    info.hardwareSeries = (uint8_t)((hardware >> 12) & 0x0F);
    info.hardwareType = (uint8_t)((hardware >> 8) & 0x0F);
    info.hardwareVersion = (uint8_t)(hardware & 0xFF);
    return STATUS_OK;
}

/** @brief 5.5.3 读取相电阻和相电感。 */
EmmV5::Status EmmV5::readPhaseParameters(uint8_t address,
                                          PhaseParameters &params,
                                          uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x20, nullptr, 0, response, 7, timeoutMs);
    if (ret == STATUS_OK) {
        params.resistanceMOhm = be16(&response[2]);
        params.inductanceUh = be16(&response[4]);
    }
    return ret;
}

/** @brief 5.5.4 读取总线电压。 */
EmmV5::Status EmmV5::readBusVoltage(uint8_t address, uint16_t &millivolts,
                                    uint32_t timeoutMs)
{
    return readUnsigned16(address, 0x24, millivolts, timeoutMs);
}

/** @brief 5.5.6 读取电机相电流。 */
EmmV5::Status EmmV5::readPhaseCurrent(uint8_t address, uint16_t &milliamps,
                                      uint32_t timeoutMs)
{
    return readUnsigned16(address, 0x27, milliamps, timeoutMs);
}

/** @brief 5.5.7 读取线性化编码器单圈值。 */
EmmV5::Status EmmV5::readEncoder(uint8_t address, uint16_t &encoder,
                                  uint32_t timeoutMs)
{
    return readUnsigned16(address, 0x31, encoder, timeoutMs);
}

/** @brief 5.5.8 读取带符号输入脉冲数。 */
EmmV5::Status EmmV5::readInputPulses(uint8_t address, int32_t &pulses,
                                     uint32_t timeoutMs)
{
    return readSigned32(address, 0x32, pulses, timeoutMs);
}

/** @brief 5.5.5 读取总线电流。 */
EmmV5::Status EmmV5::readBusCurrent(uint8_t address, uint16_t &milliamps,
                                    uint32_t timeoutMs)
{
    return readUnsigned16(address, 0x26, milliamps, timeoutMs);
}

/**
 * @brief 5.5.12 读取驱动器实时温度（功能码 0x39）。
 * @details 手册规定返回帧为 [Addr][39][符号][温度][6B]，其中符号 0 表示负数、
 *          符号 1 表示正数。这里把协议的无符号温度字段转换成便于上层使用的有符号值。
 */
EmmV5::Status EmmV5::readTemperature(uint8_t address, int16_t &temperatureC,
                                     uint32_t timeoutMs)
{
    /**
     * @details 先按手册 5.5.12 的固定 5 字节响应读取原始帧，再检查符号字段，
     *          避免把驱动器错误响应误当成温度值。
     */
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x39, nullptr, 0, response, 5, timeoutMs);
    if (ret != STATUS_OK) return ret;
    if (response[2] > 1) return STATUS_FRAME_ERROR;
    int16_t magnitude = (int16_t)response[3];
    temperatureC = response[2] == 0 ? (int16_t)-magnitude : magnitude;
    return STATUS_OK;
}

/**
 * @brief 5.5.17 读取 En/Stp/Dir IO 状态并展开 bit。
 * @details bit0=En，bit2=Stp，bit4=Dir，bit5=Dir 输出模式；其余 bit 保留。
 */
EmmV5::Status EmmV5::readIoState(uint8_t address, IoState &state,
                                 uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x3D, nullptr, 0, response, 4, timeoutMs);
    if (ret != STATUS_OK) return ret;
    state.raw = response[2];
    state.enLevel = (state.raw & 0x01) != 0;
    state.stpLevel = (state.raw & 0x04) != 0;
    state.dirLevel = (state.raw & 0x10) != 0;
    state.dirOutputMode = (state.raw & 0x20) != 0;
    return STATUS_OK;
}

/** @brief 5.6.4 读取选项 bit 字段并转换为布尔成员。 */
EmmV5::Status EmmV5::readOptionFlags(uint8_t address, OptionFlags &flags,
                                     uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x1A, nullptr, 0, response, 4, timeoutMs);
    if (ret != STATUS_OK) return ret;
    flags.raw = response[2];
    flags.motorType09 = (flags.raw & 0x01) != 0;
    flags.emmFirmware = (flags.raw & 0x02) != 0;
    flags.closedLoop = (flags.raw & 0x04) != 0;
    flags.reverseDirection = (flags.raw & 0x10) != 0;
    flags.keyLocked = (flags.raw & 0x20) != 0;
    flags.scale10x = (flags.raw & 0x80) != 0;
    return STATUS_OK;
}

/**
 * @brief 5.6.20 读取位置到达窗口（功能码 0x41）。
 * @details 返回值是驱动器内部的窗口计数；在 EMM 固件中通常按 0.1° 缩小处理，
 *          因此命令层同时输出原始数值和换算后的角度。
 */
EmmV5::Status EmmV5::readPositionWindow(uint8_t address, uint16_t &window,
                                         uint32_t timeoutMs)
{
    /** @details 0x41 查询返回两个字节的大端窗口值，统一复用通用 16 位解码器。 */
    return readUnsigned16(address, 0x41, window, timeoutMs);
}

/**
 * @brief 5.6.22 读取过热、过流保护阈值（功能码 0x13）。
 * @details 返回帧为 [Addr][13][过热16][过流16][时间16][6B]，共 9 字节。
 */
EmmV5::Status EmmV5::readProtectionThresholds(
    uint8_t address, ProtectionThresholds &thresholds, uint32_t timeoutMs)
{
    /** @details 0x13 的响应依次携带温度、电流和时间三个 16 位字段。 */
    uint8_t response[12] = {0};
    Status ret = readFixed(address, 0x13, nullptr, 0, response, 9, timeoutMs);
    if (ret != STATUS_OK) return ret;
    thresholds.overTemperatureC = be16(&response[2]);
    thresholds.overCurrentMa = be16(&response[4]);
    thresholds.durationMs = be16(&response[6]);
    return STATUS_OK;
}

/**
 * @brief 5.6.18 读取 DMX512 参数（0x49/0x78）。
 * @details X42S/Y42 的响应固定为 17 字节，包含单/双通道模式的速度和步长字段。
 */
EmmV5::Status EmmV5::readDmxParameters(uint8_t address, DmxParameters &params,
                                        uint32_t timeoutMs)
{
    uint8_t response[20] = {0};
    const uint8_t extra[] = {0x78};
    Status ret = readFixed(address, 0x49, extra, sizeof(extra), response,
                           17, timeoutMs);
    if (ret != STATUS_OK) return ret;
    params.channelCount = be16(&response[2]);
    params.channelsPerMotor = response[4];
    params.motionMode = response[5];
    params.singleSpeedRpm = be16(&response[6]);
    params.acceleration = be16(&response[8]);
    params.dualSpeedStep = be16(&response[10]);
    params.dualMotionStep = be32(&response[12]);
    return STATUS_OK;
}

/** @brief 5.6.24 读取心跳保护时间（功能码 0x16），0 表示关闭保护。 */
EmmV5::Status EmmV5::readHeartbeatTimeout(uint8_t address,
                                           uint32_t &timeoutMsValue,
                                           uint32_t timeoutMs)
{
    /** @details 0x16 响应中的四字节字段为毫秒数，按网络字节序解码。 */
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x16, nullptr, 0, response, 7, timeoutMs);
    if (ret == STATUS_OK) timeoutMsValue = be32(&response[2]);
    return ret;
}

/** @brief 5.6.26 读取积分限幅/刚性系数（功能码 0x23）。 */
EmmV5::Status EmmV5::readIntegralLimit(uint8_t address, uint32_t &value,
                                       uint32_t timeoutMs)
{
    /** @details 0x23 与 0x16 使用相同的 32 位大端返回格式，但语义是积分限幅/刚性系数。 */
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x23, nullptr, 0, response, 7, timeoutMs);
    if (ret == STATUS_OK) value = be32(&response[2]);
    return ret;
}

/** @brief 5.6.28 读取碰撞回零返回角度（功能码 0x3F）。 */
EmmV5::Status EmmV5::readCollisionReturnAngle(uint8_t address,
                                               uint16_t &angleTenthDeg,
                                               uint32_t timeoutMs)
{
    /** @details 0x3F 返回一个大端 16 位角度值，单位为手册定义的 0.1 度。 */
    return readUnsigned16(address, 0x3F, angleTenthDeg, timeoutMs);
}

/* ==================== 驱动参数写入与掉电保存 ==================== */

/** @brief 5.6.1 修改电机 ID/地址（0xAE/0x4B）。 */
EmmV5::Status EmmV5::modifyAddress(uint8_t address, bool save,
                                    uint8_t newAddress, uint32_t timeoutMs)
{
    if (!isValidAddress(address) || !isValidAddress(newAddress)) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {0x4B, (uint8_t)save, newAddress};
    return writePayloadAck(address, 0xAE, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.2 修改细分值，0 表示 256 细分。 */
EmmV5::Status EmmV5::modifyMicrostep(uint8_t address, bool save,
                                      uint8_t microstep, uint32_t timeoutMs)
{
    /** @details 辅助码 0x07 必须位于保存标志之前，随后写入大端窗口值和帧尾 0x6B。 */
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0x8A, (uint8_t)save, microstep};
    return writePayloadAck(address, 0x84, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.3 修改掉电标志（0x50）。 */
EmmV5::Status EmmV5::modifyPowerLossFlag(uint8_t address, bool flag,
                                          uint32_t timeoutMs)
{
    /** @details 构造 0xD3/0x56 的完整参数负载，三个阈值均保持手册的 16 位范围。 */
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0x50, (uint8_t)flag, FRAME_END};
    return writeAndReadAck(address, 0x50, frame, sizeof(frame), timeoutMs);
}

/** @brief 5.6.5 修改电机类型（19=1.8°，32=0.9°）。 */
EmmV5::Status EmmV5::modifyMotorType(uint8_t address, bool save,
                                     uint8_t motorType, uint32_t timeoutMs)
{
    if (!isValidAddress(address) || (motorType != 19 && motorType != 32)) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {0x35, (uint8_t)save, motorType};
    return writePayloadAck(address, 0xD7, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.6 修改固件类型（0=X，1=Emm，2=Emm 狂暴模式）。 */
EmmV5::Status EmmV5::modifyFirmwareType(uint8_t address, bool save,
                                        uint8_t firmwareType,
                                        uint32_t timeoutMs)
{
    if (!isValidAddress(address) || firmwareType > 2) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {0x69, (uint8_t)save, firmwareType};
    return writePayloadAck(address, 0xD5, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.7 修改开环/闭环控制模式（0=开环，1=闭环）。 */
EmmV5::Status EmmV5::modifyControlMode(uint8_t address, bool save,
                                       uint8_t controlMode, uint32_t timeoutMs)
{
    if (!isValidAddress(address) || controlMode > 1) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {0x69, (uint8_t)save, controlMode};
    return writePayloadAck(address, 0x46, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.8 修改电机运动正方向（0=CW，1=CCW）。 */
EmmV5::Status EmmV5::modifyDirection(uint8_t address, bool save,
                                     uint8_t direction, uint32_t timeoutMs)
{
    if (!isValidAddress(address) || direction > 1) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {0x60, (uint8_t)save, direction};
    return writePayloadAck(address, 0xD4, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.9 修改按键锁定状态。 */
EmmV5::Status EmmV5::modifyKeyLock(uint8_t address, bool save, bool locked,
                                   uint32_t timeoutMs)
{
    /** @details 0x38 是辅助码，保存标志和 32 位毫秒值按高字节到低字节发送。 */
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0xB3, (uint8_t)save, (uint8_t)locked};
    return writePayloadAck(address, 0xD0, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.11 修改 Emm 命令速度是否使用 0.1RPM 输入。 */
EmmV5::Status EmmV5::modifySpeedScale(uint8_t address, bool save,
                                      bool scale10x, uint32_t timeoutMs)
{
    /** @details 0x57 是辅助码；不对系数施加额外范围限制，保留固件可用的完整 32 位空间。 */
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0x71, (uint8_t)save, (uint8_t)scale10x};
    return writePayloadAck(address, 0x4F, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.12 修改开环模式工作电流。 */
EmmV5::Status EmmV5::modifyOpenCurrent(uint8_t address, bool save,
                                       uint16_t milliamps, uint32_t timeoutMs)
{
    if (!isValidAddress(address) || milliamps > 5000) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {0x33, (uint8_t)save,
                               (uint8_t)(milliamps >> 8), (uint8_t)milliamps};
    return writePayloadAck(address, 0x44, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.13 修改闭环最大/堵转最大电流。 */
EmmV5::Status EmmV5::modifyMaxCurrent(uint8_t address, bool save,
                                      uint16_t milliamps, uint32_t timeoutMs)
{
    if (!isValidAddress(address) || milliamps > 5000) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {0x66, (uint8_t)save,
                               (uint8_t)(milliamps >> 8), (uint8_t)milliamps};
    return writePayloadAck(address, 0x45, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.31 修改锁定参数等级（0=解锁，1~3 为不同限制等级）。 */
EmmV5::Status EmmV5::modifyLockLevel(uint8_t address, bool save, uint8_t level,
                                     uint32_t timeoutMs)
{
    if (!isValidAddress(address) || level > 3) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0x4B, (uint8_t)save, level};
    return writePayloadAck(address, 0xD6, payload, sizeof(payload), timeoutMs);
}

/**
 * @brief 5.6.21 修改位置到达窗口（功能码 0xD1，辅助码 0x07）。
 * @details 帧字段依次为是否存储和大端 16 位窗口值；该设置影响 5.5.15 的
 *          Prf_TF 到位标志以及 Reached/Both/Other 应答策略。
 */
EmmV5::Status EmmV5::modifyPositionWindow(uint8_t address, bool save,
                                           uint16_t window,
                                           uint32_t timeoutMs)
{
    /** @details 角度字段不做缩放，调用者直接传入手册定义的 0.1°单位。 */
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0x07, (uint8_t)save,
                               (uint8_t)(window >> 8), (uint8_t)window};
    return writePayloadAck(address, 0xD1, payload, sizeof(payload), timeoutMs);
}

/**
 * @brief 5.6.23 修改过热、过流保护阈值（功能码 0xD3，辅助码 0x56）。
 * @details 三个字段均为大端 16 位整数，分别表示摄氏度、毫安和毫秒。
 */
EmmV5::Status EmmV5::modifyProtectionThresholds(
    uint8_t address, bool save, const ProtectionThresholds &thresholds,
    uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {
        0x56, (uint8_t)save,
        (uint8_t)(thresholds.overTemperatureC >> 8),
        (uint8_t)thresholds.overTemperatureC,
        (uint8_t)(thresholds.overCurrentMa >> 8),
        (uint8_t)thresholds.overCurrentMa,
        (uint8_t)(thresholds.durationMs >> 8),
        (uint8_t)thresholds.durationMs
    };
    return writePayloadAck(address, 0xD3, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.25 修改心跳保护时间（功能码 0x68，辅助码 0x38）。 */
EmmV5::Status EmmV5::modifyHeartbeatTimeout(uint8_t address, bool save,
                                             uint32_t heartbeatMs,
                                             uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {
        0x38, (uint8_t)save,
        (uint8_t)(heartbeatMs >> 24), (uint8_t)(heartbeatMs >> 16),
        (uint8_t)(heartbeatMs >> 8), (uint8_t)heartbeatMs
    };
    return writePayloadAck(address, 0x68, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.27 修改积分限幅/刚性系数（功能码 0x4B，辅助码 0x57）。 */
EmmV5::Status EmmV5::modifyIntegralLimit(uint8_t address, bool save,
                                          uint32_t value, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {
        0x57, (uint8_t)save,
        (uint8_t)(value >> 24), (uint8_t)(value >> 16),
        (uint8_t)(value >> 8), (uint8_t)value
    };
    return writePayloadAck(address, 0x4B, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.29 修改碰撞回零返回角度（功能码 0x5C，辅助码 0xAC）。 */
EmmV5::Status EmmV5::modifyCollisionReturnAngle(
    uint8_t address, bool save, uint16_t angleTenthDeg, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {0xAC, (uint8_t)save,
                               (uint8_t)(angleTenthDeg >> 8),
                               (uint8_t)angleTenthDeg};
    return writePayloadAck(address, 0x5C, payload, sizeof(payload), timeoutMs);
}

/**
 * @brief 5.6.19 修改 DMX512 参数（0xD9/0x90）。
 * @details 参数按手册顺序编码；调用者负责根据实际 DMX 控制台设置通道数量和模式。
 */
EmmV5::Status EmmV5::modifyDmxParameters(uint8_t address, bool save,
                                          const DmxParameters &params,
                                          uint32_t timeoutMs)
{
    if (!isValidAddress(address) || params.channelCount < 1 ||
        (params.channelsPerMotor != 1 && params.channelsPerMotor != 2) ||
        params.motionMode > 1 || params.singleSpeedRpm < 1 ||
        params.singleSpeedRpm > 3000 || params.acceleration == 0 ||
        params.dualSpeedStep == 0 || params.dualMotionStep == 0) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {
        0x90, (uint8_t)save,
        (uint8_t)(params.channelCount >> 8), (uint8_t)params.channelCount,
        params.channelsPerMotor, params.motionMode,
        (uint8_t)(params.singleSpeedRpm >> 8), (uint8_t)params.singleSpeedRpm,
        (uint8_t)(params.acceleration >> 8), (uint8_t)params.acceleration,
        (uint8_t)(params.dualSpeedStep >> 8), (uint8_t)params.dualSpeedStep,
        (uint8_t)(params.dualMotionStep >> 24),
        (uint8_t)(params.dualMotionStep >> 16),
        (uint8_t)(params.dualMotionStep >> 8),
        (uint8_t)params.dualMotionStep
    };
    return writePayloadAck(address, 0xD9, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.6.16 读取 Emm PID 参数。 */
EmmV5::Status EmmV5::readPid(uint8_t address, PidParameters &pid,
                              uint32_t timeoutMs)
{
    uint8_t response[16] = {0};
    Status ret = readFixed(address, 0x21, nullptr, 0, response, 15, timeoutMs);
    if (ret != STATUS_OK) return ret;
    pid.kp = be32(&response[2]);
    pid.ki = be32(&response[6]);
    pid.kd = be32(&response[10]);
    return STATUS_OK;
}

/** @brief 5.6.17 修改 Emm PID 参数。 */
EmmV5::Status EmmV5::modifyPid(uint8_t address, bool save,
                                const PidParameters &pid, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t payload[] = {
        0xC3, (uint8_t)save,
        (uint8_t)(pid.kp >> 24), (uint8_t)(pid.kp >> 16),
        (uint8_t)(pid.kp >> 8), (uint8_t)pid.kp,
        (uint8_t)(pid.ki >> 24), (uint8_t)(pid.ki >> 16),
        (uint8_t)(pid.ki >> 8), (uint8_t)pid.ki,
        (uint8_t)(pid.kd >> 24), (uint8_t)(pid.kd >> 16),
        (uint8_t)(pid.kd >> 8), (uint8_t)pid.kd
    };
    return writePayloadAck(address, 0x4A, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.7.2 设置或清除上电自动运行的 Emm 速度参数。 */
EmmV5::Status EmmV5::modifyAutoSpeed(uint8_t address, bool save,
                                     uint8_t direction, uint16_t rpm,
                                     uint8_t acceleration, bool enControl,
                                     uint32_t timeoutMs)
{
    if (!isValidAddress(address) || direction > 1 || rpm > 3000) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t payload[] = {
        0x1C, (uint8_t)save, direction, (uint8_t)(rpm >> 8), (uint8_t)rpm,
        acceleration, (uint8_t)enControl
    };
    return writePayloadAck(address, 0xF7, payload, sizeof(payload), timeoutMs);
}

/** @brief 5.8.5 读取 33 字节 Emm 驱动配置响应。 */
EmmV5::Status EmmV5::readDriverConfig(uint8_t address, uint8_t *response,
                                      size_t responseSize, uint32_t timeoutMs)
{
    if (response == nullptr || responseSize < 33) return STATUS_INVALID_ARGUMENT;
    const uint8_t extra[] = {0x6C};
    return readFixed(address, 0x42, extra, sizeof(extra), response, 33, timeoutMs);
}

/** @brief 5.8.6 发送 0x48/0xD1 的 Emm 驱动配置修改帧。 */
EmmV5::Status EmmV5::modifyDriverConfig(uint8_t address, bool save,
                                        const uint8_t *payload,
                                        size_t payloadLength,
                                        uint32_t timeoutMs)
{
    if (!isValidAddress(address) || payload == nullptr || payloadLength != 29) {
        return STATUS_INVALID_ARGUMENT;
    }
    uint8_t frame[34] = {0};
    frame[0] = address;
    frame[1] = 0x48;
    frame[2] = 0xD1;
    frame[3] = (uint8_t)save;
    memcpy(&frame[4], payload, payloadLength);
    frame[33] = FRAME_END;
    return writeAndReadAck(address, 0x48, frame, sizeof(frame), timeoutMs);
}

/**
 * @brief 5.3.1 发送多电机复合命令。
 * @details 协议格式为 [00][AA][长度高][长度低][子命令...][6B]；长度字段表示
 *          从长度字段之后到最后一个子命令帧尾的字节数。该接口不等待某一台电机
 *          的响应，避免多台电机同时回复造成 RS485 冲突。
 */
EmmV5::Status EmmV5::multiMotorCommand(const uint8_t *commandBytes,
                                       size_t commandLength,
                                       uint32_t timeoutMs)
{
    (void)timeoutMs;
    if (!_begun) return STATUS_SERIAL_NOT_BEGIN;
    if (commandBytes == nullptr || commandLength == 0 || commandLength > 250) {
        return STATUS_INVALID_ARGUMENT;
    }
    uint8_t frame[256] = {0};
    frame[0] = 0x00;
    frame[1] = 0xAA;
    frame[2] = (uint8_t)(commandLength >> 8);
    frame[3] = (uint8_t)commandLength;
    memcpy(&frame[4], commandBytes, commandLength);
    frame[4 + commandLength] = FRAME_END;
    return writeFrameNoAck(frame, commandLength + 5);
}

/* ==================== 地址 0 广播控制命令 ==================== */

/**
 * @brief 5.6.30 广播读取电机 ID 地址（功能码 0x15）。
 * @details 手册规定主机发送 [00][15][6B]，唯一连接的电机返回
 *          [Addr][15][Addr][6B]。由于返回地址未知，不能复用 readFixed() 的单播
 *          地址匹配逻辑，改由 readBroadcastAddressFrame() 在串口流中搜索合法帧。
 */
EmmV5::Status EmmV5::readBroadcastAddress(uint8_t &detectedAddress,
                                           uint32_t timeoutMs)
{
    return readBroadcastAddressFrame(detectedAddress, timeoutMs);
}

/**
 * @brief 向地址 0 广播使能命令。
 * @note 广播帧没有单台应答，返回 STATUS_OK 只代表串口写入成功。
 */
EmmV5::Status EmmV5::enableBroadcast(bool enabled, bool synchronized)
{
    const uint8_t frame[] = {0x00, 0xF3, 0xAB, (uint8_t)enabled,
                             (uint8_t)synchronized, FRAME_END};
    return writeFrameNoAck(frame, sizeof(frame));
}

/** @brief 向地址 0 广播速度模式命令。 */
EmmV5::Status EmmV5::velocityBroadcast(uint8_t direction, uint16_t rpm,
                                       uint8_t acceleration, bool synchronized)
{
    if (direction > 1 || rpm > 3000) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {0x00, 0xF6, direction, (uint8_t)(rpm >> 8),
                             (uint8_t)rpm, acceleration,
                             (uint8_t)synchronized, FRAME_END};
    return writeFrameNoAck(frame, sizeof(frame));
}

/** @brief 兼容旧接口的广播位置模式封装。 */
EmmV5::Status EmmV5::positionBroadcast(uint8_t direction, uint16_t rpm,
                                       uint8_t acceleration, uint32_t pulses,
                                       bool absolute, bool synchronized)
{
    return positionBroadcastMode(direction, rpm, acceleration, pulses,
                                 absolute ? 1 : 0, synchronized);
}

/**
 * @brief 向地址 0 广播 0xFD 位置模式命令。
 * @details 多台电机可能同时收到该帧，因此不会等待响应；需要同步执行时，
 *          先把 synchronized 置 1，再广播 synchronousMotionBroadcast()。
 */
EmmV5::Status EmmV5::positionBroadcastMode(uint8_t direction, uint16_t rpm,
                                            uint8_t acceleration,
                                            uint32_t pulses,
                                            uint8_t motionMode,
                                            bool synchronized)
{
    if (direction > 1 || rpm > 3000 || motionMode > 2) {
        return STATUS_INVALID_ARGUMENT;
    }
    const uint8_t frame[] = {
        0x00, 0xFD, direction, (uint8_t)(rpm >> 8), (uint8_t)rpm,
        acceleration, (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)pulses, motionMode,
        (uint8_t)synchronized, FRAME_END};
    return writeFrameNoAck(frame, sizeof(frame));
}

/** @brief 广播立即停止命令。 */
EmmV5::Status EmmV5::stopBroadcast(bool synchronized)
{
    const uint8_t frame[] = {0x00, 0xFE, 0x98, (uint8_t)synchronized,
                             FRAME_END};
    return writeFrameNoAck(frame, sizeof(frame));
}

/** @brief 广播触发回零命令。 */
EmmV5::Status EmmV5::homeBroadcast(uint8_t mode, bool synchronized)
{
    if (mode > 5) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {0x00, 0x9A, mode, (uint8_t)synchronized,
                             FRAME_END};
    return writeFrameNoAck(frame, sizeof(frame));
}

/** @brief 广播强制退出回零命令。 */
EmmV5::Status EmmV5::interruptHomeBroadcast()
{
    const uint8_t frame[] = {0x00, 0x9C, 0x48, FRAME_END};
    return writeFrameNoAck(frame, sizeof(frame));
}

/** @brief 广播当前位置清零命令。 */
EmmV5::Status EmmV5::resetCurrentPositionBroadcast()
{
    const uint8_t frame[] = {0x00, 0x0A, 0x6D, FRAME_END};
    return writeFrameNoAck(frame, sizeof(frame));
}

/**
 * @brief 发送 0xFF/0x66，触发指定电机执行已经缓存的同步命令。
 */
EmmV5::Status EmmV5::synchronousMotion(uint8_t address, uint32_t timeoutMs)
{
    if (!isValidAddress(address)) return STATUS_INVALID_ARGUMENT;
    const uint8_t frame[] = {address, 0xFF, 0x66, FRAME_END};
    return writeAndReadAck(address, 0xFF, frame, sizeof(frame), timeoutMs);
}

/** @brief 广播同步触发帧，让所有已缓存命令的电机同时执行。 */
EmmV5::Status EmmV5::synchronousMotionBroadcast()
{
    const uint8_t frame[] = {0x00, 0xFF, 0x66, FRAME_END};
    return writeFrameNoAck(frame, sizeof(frame));
}

/* ==================== 位置、速度及状态字查询 ==================== */

/**
 * @brief 读取实时位置（0x36）。
 * @details 返回帧采用 [地址][0x36][符号][大端 32 位幅值][0x6B] 格式，
 *          统一交由 readSigned32() 校验符号和数值范围。
 */
EmmV5::Status EmmV5::readCurrentPosition(uint8_t address, int32_t &position,
                                          uint32_t timeoutMs)
{
    return readSigned32(address, 0x36, position, timeoutMs);
}

/**
 * @brief 读取实时转速（0x35）。
 * @details 响应中的符号字节位于 response[2]，后续 2 字节是 RPM 幅值；
 *          负号表示反向旋转，非法符号或帧长会返回 STATUS_FRAME_ERROR。
 */
EmmV5::Status EmmV5::readVelocity(uint8_t address, int16_t &rpm,
                                  uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x35, nullptr, 0, response, 6, timeoutMs);
    if (ret != STATUS_OK) return ret;
    if (response[2] > 1) return STATUS_FRAME_ERROR;
    uint16_t magnitude = be16(&response[3]);
    rpm = (int16_t)(response[2] ? -(int32_t)magnitude : (int32_t)magnitude);
    return STATUS_OK;
}

/** @brief 读取目标位置（0x33），结果采用符号-幅值格式。 */
EmmV5::Status EmmV5::readTargetPosition(uint8_t address, int32_t &position,
                                         uint32_t timeoutMs)
{
    return readSigned32(address, 0x33, position, timeoutMs);
}

/** @brief 读取实时设定目标位置（0x34）。 */
EmmV5::Status EmmV5::readCommandPosition(uint8_t address, int32_t &position,
                                          uint32_t timeoutMs)
{
    return readSigned32(address, 0x34, position, timeoutMs);
}

/** @brief 读取位置误差（0x37），结果为带符号的 32 位位置量。 */
EmmV5::Status EmmV5::readPositionError(uint8_t address, int32_t &error,
                                        uint32_t timeoutMs)
{
    return readSigned32(address, 0x37, error, timeoutMs);
}

/**
 * @brief 读取电机状态标志（0x3A）。
 * @param flags 输出 1 字节状态位；各 bit 的语义由 X42S 手册定义。
 */
EmmV5::Status EmmV5::readMotorFlags(uint8_t address, uint8_t &flags,
                                    uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x3A, nullptr, 0, response, 4, timeoutMs);
    if (ret == STATUS_OK) flags = response[2];
    return ret;
}

/**
 * @brief 读取回零状态标志（0x3B，手册 5.4.4）。
 * @param flags 输出回零状态字，命令层会进一步展开各 bit。
 */
EmmV5::Status EmmV5::readOriginFlags(uint8_t address, uint8_t &flags,
                                     uint32_t timeoutMs)
{
    /*
     * 手册 5.4.4：bit2=Org_SF（正在回零），bit3=Org_CF（回零失败），
     * bit4/bit5=过热/过流保护。这里保留完整原始字节，由上层结合命令历史
     * 判断“空闲”究竟是尚未回零还是回零已经成功。
     */
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x3B, nullptr, 0, response, 4, timeoutMs);
    if (ret == STATUS_OK) flags = response[2];
    return ret;
}

/**
 * @brief 5.4.5 读取驱动器当前回零参数。
 * @details 发送 0x22 查询帧，期望收到 18 字节响应；参数顺序与 0x4C/0xAE
 *          写入帧一致，读取后逐项检查模式、方向和自动回零标志的取值范围。
 * @return 返回模式、方向、速度、超时、碰撞检测参数和上电自动回零标志。
 */
EmmV5::Status EmmV5::readHomeParams(
    uint8_t address, uint8_t &mode, uint8_t &direction, uint16_t &speedRpm,
    uint32_t &homeTimeoutMs, uint16_t &stallSpeedRpm,
    uint16_t &stallCurrentMa, uint16_t &stallTimeMs, bool &powerOnHome,
    uint32_t timeoutMs)
{
    uint8_t response[24] = {0};
    Status ret = readFixed(address, 0x22, nullptr, 0, response, 18, timeoutMs);
    if (ret != STATUS_OK) return ret;
    if (response[2] > 5 || response[3] > 1 || response[16] > 1) {
        return STATUS_FRAME_ERROR;
    }

    mode = response[2];
    direction = response[3];
    speedRpm = be16(&response[4]);
    homeTimeoutMs = be32(&response[6]);
    stallSpeedRpm = be16(&response[10]);
    stallCurrentMa = be16(&response[12]);
    stallTimeMs = be16(&response[14]);
    powerOnHome = response[16] != 0;
    return STATUS_OK;
}

/**
 * @brief 读取合并状态字（0x3C，手册 5.5.16）。
 * @param originFlags 输出回零状态标志。
 * @param motorFlags 输出电机运行/保护状态标志。
 */
EmmV5::Status EmmV5::readCombinedFlags(uint8_t address, uint8_t &originFlags,
                                       uint8_t &motorFlags, uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, 0x3C, nullptr, 0, response, 5, timeoutMs);
    if (ret == STATUS_OK) {
        originFlags = response[2];
        motorFlags = response[3];
    }
    return ret;
}

/**
 * @brief 读取 X42S 系统状态（0x43/0x7A，手册 5.8.2）。
 * @details 响应固定为 31 字节，函数校验版本字段 0x1F/0x09，然后按字段偏移
 *          解码电压、电流、编码器、目标/实时位置及状态标志。
 */
EmmV5::Status EmmV5::readSystemState(uint8_t address, SystemState &state,
                                     uint32_t timeoutMs)
{
    const uint8_t extra[] = {0x7A};
    uint8_t response[64] = {0};
    Status ret = readFixed(address, 0x43, extra, sizeof(extra), response,
                           31, timeoutMs);
    if (ret != STATUS_OK) return ret;
    if (response[2] != 0x1F || response[3] != 0x09) return STATUS_FRAME_ERROR;

    state.busVoltageMv = be16(&response[4]);
    state.phaseCurrentMa = be16(&response[6]);
    state.encoderValue = be16(&response[8]);
    if (!decodeSignedMagnitude(response[10], be32(&response[11]),
                               state.targetPosition)) return STATUS_FRAME_ERROR;
    if (response[15] > 1) return STATUS_FRAME_ERROR;
    uint16_t speed = be16(&response[16]);
    state.realtimeRpm = (int16_t)(response[15] ? -(int32_t)speed : (int32_t)speed);
    if (!decodeSignedMagnitude(response[18], be32(&response[19]),
                               state.realtimePosition)) return STATUS_FRAME_ERROR;
    if (!decodeSignedMagnitude(response[23], be32(&response[24]),
                               state.positionError)) return STATUS_FRAME_ERROR;
    state.originFlags = response[28];
    state.motorFlags = response[29];
    return STATUS_OK;
}

/* ==================== 通用组帧与串口事务实现 ==================== */

/**
 * @brief 组装带辅助码/参数的单播控制帧并等待 ACK。
 * @details payload 不包含地址、功能码和帧尾；该辅助函数统一保证最后一个字节
 *          为 0x6B，避免新增手册命令在拼帧时遗漏帧尾或字节序错误。
 */
EmmV5::Status EmmV5::writePayloadAck(uint8_t address, uint8_t function,
                                     const uint8_t *payload,
                                     size_t payloadLength,
                                     uint32_t timeoutMs)
{
    if (payload == nullptr || payloadLength > 36) {
        return STATUS_INVALID_ARGUMENT;
    }
    uint8_t frame[40] = {0};
    frame[0] = address;
    frame[1] = function;
    if (payloadLength > 0) memcpy(&frame[2], payload, payloadLength);
    frame[2 + payloadLength] = FRAME_END;
    return writeAndReadAck(address, function, frame, payloadLength + 3, timeoutMs);
}

/**
 * @brief 读取格式为 [地址][功能码][大端16位值][帧尾] 的查询响应。
 */
EmmV5::Status EmmV5::readUnsigned16(uint8_t address, uint8_t function,
                                    uint16_t &value, uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, function, nullptr, 0, response, 5, timeoutMs);
    if (ret == STATUS_OK) value = be16(&response[2]);
    return ret;
}

/**
 * @brief 执行 5.6.30 广播读取 ID 的串口事务。
 * @details 输入流可能包含 RS485 收发器回显或其他残留字节，因此采用滑动窗口；
 *          只有地址非零、功能码为 0x15、数据地址与源地址相同且帧尾为 0x6B 时才接受。
 */
EmmV5::Status EmmV5::readBroadcastAddressFrame(uint8_t &detectedAddress,
                                                uint32_t timeoutMs)
{
    if (!_begun) return STATUS_SERIAL_NOT_BEGIN;
    clearRxBuffer();
    const uint8_t frame[] = {0x00, 0x15, FRAME_END};
    logFrame("TX_BCAST_READ_ID", frame, sizeof(frame));
    if (_serial->write(frame, sizeof(frame)) != sizeof(frame)) {
        return STATUS_WRITE_ERROR;
    }
    _serial->flush();

    uint8_t rx[32] = {0};
    size_t rxLength = 0;
    bool receivedData = false;
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < timeoutMs) {
        while (_serial->available() > 0) {
            receivedData = true;
            uint8_t value = (uint8_t)_serial->read();
            if (rxLength < sizeof(rx)) rx[rxLength++] = value;
            else {
                memmove(rx, rx + 1, sizeof(rx) - 1);
                rx[sizeof(rx) - 1] = value;
            }
            for (size_t i = 0; i + 4 <= rxLength; ++i) {
                if (rx[i] == 0 || rx[i + 1] != 0x15 ||
                    rx[i] != rx[i + 2] || rx[i + 3] != FRAME_END) {
                    continue;
                }
                detectedAddress = rx[i];
                saveResponse(&rx[i], 4);
                logFrame("RX_BCAST_READ_ID", &rx[i], 4);
                return STATUS_OK;
            }
        }
        delay(1);
    }
    return receivedData ? STATUS_FRAME_ERROR : STATUS_TIMEOUT;
}

/**
 * @brief 发送单播控制帧并等待驱动器应答。
 * @details 发送前清空输入缓存，随后在超时时间内从串口流中寻找匹配的
 *          [地址][功能码][状态][0x6B]。如果把本次 TX 回显到 RX，会跳过该回显，
 *          防止 RS485 收发器回显被误判为电机应答。
 *
 *          requireDone=true 时语义变为"必须等到动作完成"：驱动器把
 *          Response 菜单设为 Both 后，会先回 0x02（命令已接受），动作真正做完
 *          再主动回 0x9F。因此这里收到 0x02 后**不返回**，而是继续在同一个接收
 *          循环里等 0x9F。这一点必须成立：一旦先返回、把总线交还出去，下一次
 *          事务的 clearRxBuffer() 会把后面到达的 0x9F 清掉，永远等不到。
 *
 * @return STATUS_DONE  收到 0x9F，动作已完成；
 *         STATUS_OK    收到 0x02（requireDone=false 时即返回；true 时表示命令
 *                      已被接受但未等到 0x9F，由上层决定是否回退轮询）；
 *         STATUS_NOT_MOVED 收到 0x12/0x22（合法但电机不动）；
 *         STATUS_REJECTED / STATUS_TIMEOUT / STATUS_FRAME_ERROR 见 Status 注释。
 */
EmmV5::Status EmmV5::writeAndReadAck(uint8_t address, uint8_t function,
                                     const uint8_t *frame, size_t frameLength,
                                     uint32_t timeoutMs, bool requireDone)
{
    if (!_begun) return STATUS_SERIAL_NOT_BEGIN;
    if (!isValidAddress(address) || frame == nullptr || frameLength == 0) {
        return STATUS_INVALID_ARGUMENT;
    }
    clearRxBuffer();
    logFrame("TX", frame, frameLength);
    if (_serial->write(frame, frameLength) != frameLength) return STATUS_WRITE_ERROR;
    _serial->flush();

    uint8_t rx[64] = {0};
    size_t rxLength = 0;
    /*
     * 已被消费掉的应答帧起点。
     * @details requireDone=true 时收到 0x02 后要继续等 0x9F；若不记录这条 0x02
     *          已被消费，下一轮滑动扫描会反复匹配到同一条帧，导致 0x9F 永远排不上。
     */
    size_t scanFrom = 0;
    bool receivedData = false;
    bool accepted = false;   ///< 是否已经收到 0x02（命令被接受）。
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < timeoutMs) {
        while (_serial->available() > 0) {
            receivedData = true;
            uint8_t value = (uint8_t)_serial->read();
            if (rxLength < sizeof(rx)) {
                rx[rxLength++] = value;
            } else {
                memmove(rx, rx + 1, sizeof(rx) - 1);
                rx[sizeof(rx) - 1] = value;
                if (scanFrom > 0) --scanFrom;   /* 缓冲区整体左移一字节。 */
            }
            if (rxLength < 4) continue;
            for (size_t i = scanFrom; i + 4 <= rxLength; ++i) {
                if (rx[i] != address || rx[i + 1] != function ||
                    rx[i + 3] != FRAME_END) continue;
                if (frameLength == 4 && memcmp(&rx[i], frame, 4) == 0) continue;

                const uint8_t ackStatus = rx[i + 2];
                saveResponse(&rx[i], 4);
                logFrame("RX", &rx[i], 4);

                /* 0x12/0x22：驱动器明确告知"电机不会动"，既不是成功也不是故障。 */
                if (ackStatus == ACK_AT_ZERO || ackStatus == ACK_LIMIT_TRIGGERED) {
                    ESP_LOGW(TAG,
                             "[应答] A=%u FUNC=0x%02X ACK_STATUS=0x%02X："
                             "已在零点或限位已触发，电机不动（手册 4.1.2，非错误）",
                             (unsigned)address, (unsigned)function,
                             (unsigned)ackStatus);
                    return STATUS_NOT_MOVED;
                }

                /* 0x02：命令被接受。 */
                if (ackStatus == ACK_OK) {
                    accepted = true;
                    if (!requireDone) return STATUS_OK;
                    /*
                     * 需要等动作完成：把这条 0x02 记为"已消费"，继续留在同一个
                     * 接收循环里等后面的 0x9F。
                     */
                    scanFrom = i + 4;
                    break;
                }

                /* 0x9F：驱动器主动上报"动作已完成"。 */
                if (ackStatus == ACK_REACHED) {
                    /*
                     * 行为兼容：requireDone=false 时沿用历史语义，把 0x9F 也算作
                     * STATUS_OK（既有调用方一律以 "== STATUS_OK" 判成功，不能改变
                     * 它们看到的结果）。只有显式 requireDone=true 才区分为
                     * STATUS_DONE，表示"动作确实做完"。
                     */
                    return requireDone ? STATUS_DONE : STATUS_OK;
                }

                /* 其余（0xE2/0xEE/...）：明确的拒绝。 */
                ESP_LOGW(TAG,
                         "[诊断] 驱动器拒绝：A=%u FUNC=0x%02X ACK_STATUS=0x%02X"
                         "（既非 0x02 成功也非 0x9F 到位）",
                         (unsigned)address, (unsigned)function,
                         (unsigned)ackStatus);
                return STATUS_REJECTED;
            }
        }
        delay(1);
    }

    /*
     * 超时收尾：
     *   - 收到过 0x02 但没等到 0x9F：命令确实被接受了，只是"动作完成"事件没到，
     *     返回 STATUS_OK，由上层决定是否回退到轮询判据（0x3B / 0x3A）；
     *   - 什么都没收到：超时；
     *   - 收到过字节但拼不出合法帧：帧错误。
     */
    if (accepted) return STATUS_OK;
    return receivedData ? STATUS_FRAME_ERROR : STATUS_TIMEOUT;
}

/**
 * @brief 写出广播/无应答帧。
 * @details 地址 0 广播时多个驱动器可能同时响应，因此这里刻意不读取串口；
 *          调用者只应把 STATUS_OK 理解为“已写入 UART”。
 */
EmmV5::Status EmmV5::writeFrameNoAck(const uint8_t *frame, size_t frameLength)
{
    if (!_begun) return STATUS_SERIAL_NOT_BEGIN;
    if (frame == nullptr || frameLength == 0) return STATUS_INVALID_ARGUMENT;
    clearRxBuffer();
    logFrame("TX_BCAST", frame, frameLength);
    if (_serial->write(frame, frameLength) != frameLength) return STATUS_WRITE_ERROR;
    _serial->flush();
    return STATUS_OK;
}

/**
 * @brief 发送查询帧并提取固定长度响应。
 * @details 函数允许 RX 中存在回显或无关字节，使用地址、功能码和帧尾重新同步；
 *          若检测到 [地址][0x00][0xEE][0x6B]，则转换为 STATUS_REJECTED。
 */
EmmV5::Status EmmV5::readFixed(uint8_t address, uint8_t function,
                               const uint8_t *extra, size_t extraLength,
                               uint8_t *response, size_t expectedLength,
                               uint32_t timeoutMs)
{
    if (!_begun) return STATUS_SERIAL_NOT_BEGIN;
    if (!isValidAddress(address) || response == nullptr || expectedLength < 4 ||
        expectedLength > sizeof(_lastResponse.raw)) return STATUS_INVALID_ARGUMENT;

    uint8_t frame[16] = {0};
    size_t frameLength = 0;
    frame[frameLength++] = address;
    frame[frameLength++] = function;
    for (size_t i = 0; i < extraLength && frameLength < sizeof(frame) - 1; ++i) {
        frame[frameLength++] = extra[i];
    }
    frame[frameLength++] = FRAME_END;
    clearRxBuffer();
    logFrame("TX_READ", frame, frameLength);
    if (_serial->write(frame, frameLength) != frameLength) return STATUS_WRITE_ERROR;
    _serial->flush();

    uint8_t rx[64] = {0};
    size_t rxLength = 0;
    bool receivedData = false;
    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < timeoutMs) {
        while (_serial->available() > 0) {
            receivedData = true;
            uint8_t value = (uint8_t)_serial->read();
            if (rxLength < sizeof(rx)) rx[rxLength++] = value;
            else { memmove(rx, rx + 1, sizeof(rx) - 1); rx[sizeof(rx) - 1] = value; }
            if (rxLength >= 4) {
                for (size_t i = 0; i + 4 <= rxLength; ++i) {
                    if (rx[i] == address && rx[i + 1] == 0x00 &&
                        rx[i + 2] == 0xEE && rx[i + 3] == FRAME_END) {
                        saveResponse(&rx[i], 4);
                        return STATUS_REJECTED;
                    }
                }
            }
            if (rxLength < expectedLength) continue;
            for (size_t i = 0; i + expectedLength <= rxLength; ++i) {
                if (rx[i] == address && rx[i + 1] == function &&
                    rx[i + expectedLength - 1] == FRAME_END) {
                    memcpy(response, &rx[i], expectedLength);
                    saveResponse(&rx[i], expectedLength);
                    logFrame("RX_READ", &rx[i], expectedLength);
                    return STATUS_OK;
                }
            }
        }
        delay(1);
    }
    return receivedData ? STATUS_FRAME_ERROR : STATUS_TIMEOUT;
}

/**
 * @brief 读取符号-幅值编码的 32 位参数。
 * @details EMM 不使用二进制补码，而是以 1 字节符号（0/1）加大端幅值表示，
 *          因此必须通过 decodeSignedMagnitude() 做边界检查后再转换为 int32_t。
 */
EmmV5::Status EmmV5::readSigned32(uint8_t address, uint8_t function,
                                   int32_t &value, uint32_t timeoutMs)
{
    uint8_t response[8] = {0};
    Status ret = readFixed(address, function, nullptr, 0, response, 8, timeoutMs);
    if (ret != STATUS_OK) return ret;
    return decodeSignedMagnitude(response[2], be32(&response[3]), value)
               ? STATUS_OK : STATUS_FRAME_ERROR;
}

/** @brief 清除串口接收 FIFO 中残留的字节。 */
void EmmV5::clearRxBuffer() { while (_serial->available() > 0) _serial->read(); }

/**
 * @brief 按十六进制打印一帧协议数据。
 * @details 仅用于调试；默认关闭以免日志影响实时控制和上位机串口带宽。
 */
void EmmV5::logFrame(const char *prefix, const uint8_t *data, size_t length) const
{
    if (!_debugEnabled || data == nullptr) return;
    char text[192] = {0};
    size_t offset = 0;
    for (size_t i = 0; i < length && offset + 4 < sizeof(text); ++i) {
        int written = snprintf(text + offset, sizeof(text) - offset,
                               "%02X%s", data[i], i + 1 < length ? " " : "");
        if (written <= 0) break;
        offset += (size_t)written;
    }
    ESP_LOGI(TAG, "%s [%u]: %s", prefix, (unsigned)length, text);
}

/**
 * @brief 保存最近一次响应帧。
 * @details 保存最多 64 字节，并从前三个字段提取地址、功能码和命令状态，
 *          方便故障诊断接口读取而无需再次访问串口。
 */
void EmmV5::saveResponse(const uint8_t *data, size_t length)
{
    memset(&_lastResponse, 0, sizeof(_lastResponse));
    size_t copyLength = length > sizeof(_lastResponse.raw)
                            ? sizeof(_lastResponse.raw) : length;
    memcpy(_lastResponse.raw, data, copyLength);
    _lastResponse.rawLength = (uint8_t)copyLength;
    if (copyLength > 0) _lastResponse.address = data[0];
    if (copyLength > 1) _lastResponse.function = data[1];
    if (copyLength > 2) _lastResponse.commandStatus = data[2];
}

/** @brief 判断地址是否为合法单播地址；地址 0 留给广播。 */
bool EmmV5::isValidAddress(uint8_t address) { return address != 0; }

/**
 * @brief 解码 EMM 的符号-幅值整数。
 * @param sign 符号字节，0 表示正数，1 表示负数。
 * @param magnitude 无符号幅值，允许覆盖 int32_t 的负数最小值 0x80000000。
 * @return 字段合法且未溢出时返回 true。
 */
bool EmmV5::decodeSignedMagnitude(uint8_t sign, uint32_t magnitude,
                                  int32_t &value)
{
    if (sign > 1 || (sign == 0 && magnitude > 0x7FFFFFFFUL) ||
        (sign == 1 && magnitude > 0x80000000UL)) return false;
    if (sign == 0) value = (int32_t)magnitude;
    else if (magnitude == 0x80000000UL) value = (int32_t)0x80000000UL;
    else value = -(int32_t)magnitude;
    return true;
}

/** @brief 将两个网络字节序字节转换为 uint16_t。 */
uint16_t EmmV5::be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/** @brief 将四个网络字节序字节转换为 uint32_t。 */
uint32_t EmmV5::be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

/** @brief 将内部状态码映射为上位机协议中的文本。 */
const char *EmmV5::statusToString(Status status)
{
    switch (status) {
    case STATUS_DONE: return "DONE";
    case STATUS_OK: return "OK";
    case STATUS_TIMEOUT: return "TIMEOUT";
    case STATUS_FRAME_ERROR: return "FRAME_ERROR";
    case STATUS_REJECTED: return "REJECTED";
    case STATUS_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
    case STATUS_SERIAL_NOT_BEGIN: return "SERIAL_NOT_BEGIN";
    case STATUS_WRITE_ERROR: return "WRITE_ERROR";
    case STATUS_NOT_MOVED: return "NOT_MOVED";
    default: return "UNKNOWN";
    }
}

#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#endif

#include "SV04M.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *const SV04M_TAG = "SV04M";
static const uint8_t SV04M_STX = 0xCC;
static const uint8_t SV04M_ETX = 0xDD;

SV04M::SV04M(HardwareSerial &serial)
    : _serial(&serial),
      _address(0x00),
      _channelCount(10),
      _begun(false),
      _debugEnabled(false)
{
    memset(&_lastResponse, 0, sizeof(_lastResponse));
    _lastResponse.status = STATUS_TIMEOUT;
}

void SV04M::begin(uint32_t baud,
                  int8_t rxPin,
                  int8_t txPin,
                  uint8_t address,
                  uint8_t channelCount)
{
    _address = address;
    _channelCount = channelCount;

    /* ESP32 Arduino begin 参数顺序：baud, config, RX 引脚, TX 引脚 */
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
    _serial->setTimeout(50);
    _begun = true;

    delay(20);
    clearRxBuffer();

    if (_debugEnabled) {
        ESP_LOGI(SV04M_TAG,
                 "begin ok, baud=%lu, rx=%d, tx=%d, addr=0x%02X, channels=%u",
                 (unsigned long)baud,
                 (int)rxPin,
                 (int)txPin,
                 _address,
                 _channelCount);
    }
}

void SV04M::setAddress(uint8_t address)
{
    _address = address;
}

uint8_t SV04M::address() const
{
    return _address;
}

void SV04M::setChannelCount(uint8_t channelCount)
{
    _channelCount = channelCount;
}

uint8_t SV04M::channelCount() const
{
    return _channelCount;
}

void SV04M::setDebugEnabled(bool enabled)
{
    _debugEnabled = enabled;
}

bool SV04M::debugEnabled() const
{
    return _debugEnabled;
}

void SV04M::setDebugStream(Stream *debug)
{
    /* 兼容旧版本接口：不再使用 Serial/Stream 打印，只根据是否为空打开或关闭 ESP_LOG 调试。 */
    _debugEnabled = (debug != nullptr);
}

const SV04M::Response &SV04M::lastResponse() const
{
    return _lastResponse;
}

SV04M::Status SV04M::sendCommand(uint8_t func,
                                 uint16_t param,
                                 Response *response,
                                 uint32_t timeoutMs)
{
    if (!_begun) {
        ESP_LOGE(SV04M_TAG, "sendCommand failed: serial not begin");
        return STATUS_SERIAL_NOT_BEGIN;
    }

    uint8_t frame[8];
    frame[0] = SV04M_STX;
    frame[1] = _address;
    frame[2] = func;
    frame[3] = (uint8_t)(param & 0xFF);        // B3 参数低字节
    frame[4] = (uint8_t)((param >> 8) & 0xFF); // B4 参数高字节
    frame[5] = SV04M_ETX;

    uint16_t sum = sum16(frame, 6);
    frame[6] = (uint8_t)(sum & 0xFF);
    frame[7] = (uint8_t)((sum >> 8) & 0xFF);

    return writeAndRead(frame, sizeof(frame), response, timeoutMs);
}

SV04M::Status SV04M::sendFactoryCommand(uint8_t func,
                                        uint32_t param,
                                        Response *response,
                                        uint32_t timeoutMs)
{
    if (!_begun) {
        ESP_LOGE(SV04M_TAG, "sendFactoryCommand failed: serial not begin");
        return STATUS_SERIAL_NOT_BEGIN;
    }

    uint8_t frame[14];
    frame[0] = SV04M_STX;
    frame[1] = _address;
    frame[2] = func;

    /* 工厂指令密码位：FF EE BB AA */
    frame[3] = 0xFF;
    frame[4] = 0xEE;
    frame[5] = 0xBB;
    frame[6] = 0xAA;

    /* 4 字节参数，小端格式 */
    frame[7]  = (uint8_t)(param & 0xFF);
    frame[8]  = (uint8_t)((param >> 8) & 0xFF);
    frame[9]  = (uint8_t)((param >> 16) & 0xFF);
    frame[10] = (uint8_t)((param >> 24) & 0xFF);
    frame[11] = SV04M_ETX;

    uint16_t sum = sum16(frame, 12);
    frame[12] = (uint8_t)(sum & 0xFF);
    frame[13] = (uint8_t)((sum >> 8) & 0xFF);

    return writeAndRead(frame, sizeof(frame), response, timeoutMs);
}

SV04M::Status SV04M::moveToChannel(uint8_t channel,
                                   bool waitDone,
                                   uint32_t timeoutMs)
{
    if (channel == 0 || channel > _channelCount) {
        ESP_LOGE(SV04M_TAG, "moveToChannel invalid channel=%u, channelCount=%u", channel, _channelCount);
        return STATUS_INVALID_ARGUMENT;
    }

    Response resp;
    Status ret = sendCommand(0x44, (uint16_t)channel, &resp, 1000);

    if (ret != STATUS_OK && ret != STATUS_TASK_PENDING) {
        ESP_LOGE(SV04M_TAG, "moveToChannel command failed: %s", statusToString(ret));
        return ret;
    }

    if (!waitDone) {
        return ret;
    }

    return waitUntilIdle(timeoutMs);
}

SV04M::Status SV04M::resetToHome(bool waitDone, uint32_t timeoutMs)
{
    Response resp;
    Status ret = sendCommand(0x45, 0x0000, &resp, 1000);

    if (ret != STATUS_OK && ret != STATUS_TASK_PENDING) {
        ESP_LOGE(SV04M_TAG, "resetToHome command failed: %s", statusToString(ret));
        return ret;
    }

    if (!waitDone) {
        return ret;
    }

    return waitUntilIdle(timeoutMs);
}

SV04M::Status SV04M::stop()
{
    Response resp;
    Status ret = sendCommand(0x49, 0x0000, &resp, 1000);
    if (ret != STATUS_OK) {
        ESP_LOGE(SV04M_TAG, "stop failed: %s", statusToString(ret));
    }
    return ret;
}

SV04M::Status SV04M::moveByDirection(uint8_t b3,
                                     uint8_t b4,
                                     bool waitDone,
                                     uint32_t timeoutMs)
{
    if (b3 == 0 || b4 == 0 || b3 > _channelCount || b4 > _channelCount) {
        ESP_LOGE(SV04M_TAG, "moveByDirection invalid b3=%u, b4=%u, channelCount=%u", b3, b4, _channelCount);
        return STATUS_INVALID_ARGUMENT;
    }

    uint16_t param = ((uint16_t)b4 << 8) | b3;
    Response resp;
    Status ret = sendCommand(0xA4, param, &resp, 1000);

    if (ret != STATUS_OK && ret != STATUS_TASK_PENDING) {
        ESP_LOGE(SV04M_TAG, "moveByDirection command failed: %s", statusToString(ret));
        return ret;
    }

    if (!waitDone) {
        return ret;
    }

    return waitUntilIdle(timeoutMs);
}

SV04M::Status SV04M::moveToBetween(uint8_t b3,
                                   uint8_t b4,
                                   bool waitDone,
                                   uint32_t timeoutMs)
{
    if (b3 == 0 || b4 == 0 || b3 > _channelCount || b4 > _channelCount) {
        ESP_LOGE(SV04M_TAG, "moveToBetween invalid b3=%u, b4=%u, channelCount=%u", b3, b4, _channelCount);
        return STATUS_INVALID_ARGUMENT;
    }

    uint16_t param = ((uint16_t)b4 << 8) | b3;
    Response resp;
    Status ret = sendCommand(0xB4, param, &resp, 1000);

    if (ret != STATUS_OK && ret != STATUS_TASK_PENDING) {
        ESP_LOGE(SV04M_TAG, "moveToBetween command failed: %s", statusToString(ret));
        return ret;
    }

    if (!waitDone) {
        return ret;
    }

    return waitUntilIdle(timeoutMs);
}

SV04M::Status SV04M::queryCurrentChannel(uint8_t &channel)
{
    Response resp;
    Status ret = sendCommand(0x3E, 0x0000, &resp, 1000);

    if (ret == STATUS_OK) {
        channel = (uint8_t)(resp.param & 0xFF);
    } else {
        ESP_LOGE(SV04M_TAG, "queryCurrentChannel failed: %s", statusToString(ret));
    }

    return ret;
}

SV04M::Status SV04M::queryMotorStatus()
{
    Response resp;
    return sendCommand(0x4A, 0x0000, &resp, 1000);
}

SV04M::Status SV04M::waitUntilIdle(uint32_t timeoutMs, uint16_t pollIntervalMs)
{
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < timeoutMs) {
        Status ret = queryMotorStatus();

        if (ret == STATUS_OK) {
            return STATUS_OK;
        }

        if (ret != STATUS_MOTOR_BUSY && ret != STATUS_TASK_PENDING) {
            ESP_LOGE(SV04M_TAG, "waitUntilIdle failed: %s", statusToString(ret));
            return ret;
        }

        delay(pollIntervalMs);
    }

    ESP_LOGE(SV04M_TAG, "waitUntilIdle timeout, timeoutMs=%lu", (unsigned long)timeoutMs);
    return STATUS_TIMEOUT;
}

SV04M::Status SV04M::queryAddress(uint8_t &addr)
{
    Response resp;
    Status ret = sendCommand(0x20, 0x0000, &resp, 1000);

    if (ret == STATUS_OK) {
        addr = (uint8_t)(resp.param & 0xFF);
    } else {
        ESP_LOGE(SV04M_TAG, "queryAddress failed: %s", statusToString(ret));
    }

    return ret;
}

SV04M::Status SV04M::queryRS485Baud(BaudIndex &baudIndex)
{
    Response resp;
    Status ret = sendCommand(0x22, 0x0000, &resp, 1000);

    if (ret == STATUS_OK) {
        uint8_t index = (uint8_t)(resp.param & 0xFF);
        if (index <= BAUD_115200) {
            baudIndex = (BaudIndex)index;
        } else {
            ESP_LOGE(SV04M_TAG, "queryRS485Baud invalid baud index=%u", index);
            return STATUS_PARAM_ERROR;
        }
    } else {
        ESP_LOGE(SV04M_TAG, "queryRS485Baud failed: %s", statusToString(ret));
    }

    return ret;
}

SV04M::Status SV04M::queryVersion(uint8_t &major, uint8_t &minor)
{
    Response resp;
    Status ret = sendCommand(0x3F, 0x0000, &resp, 1000);

    if (ret == STATUS_OK) {
        major = (uint8_t)(resp.param & 0xFF);          // B3
        minor = (uint8_t)((resp.param >> 8) & 0xFF);  // B4
    } else {
        ESP_LOGE(SV04M_TAG, "queryVersion failed: %s", statusToString(ret));
    }

    return ret;
}

SV04M::Status SV04M::queryAutoReset(bool &enabled)
{
    Response resp;
    Status ret = sendCommand(0x2E, 0x0000, &resp, 1000);

    if (ret == STATUS_OK) {
        enabled = ((resp.param & 0xFF) != 0);
    } else {
        ESP_LOGE(SV04M_TAG, "queryAutoReset failed: %s", statusToString(ret));
    }

    return ret;
}

SV04M::Status SV04M::setDeviceAddress(uint8_t newAddress)
{
    if (newAddress > 0x7F) {
        ESP_LOGE(SV04M_TAG, "setDeviceAddress invalid address=0x%02X", newAddress);
        return STATUS_INVALID_ARGUMENT;
    }

    Response resp;
    Status ret = sendFactoryCommand(0x00, (uint32_t)newAddress, &resp, 1000);

    if (ret == STATUS_OK) {
        _address = newAddress;
        ESP_LOGI(SV04M_TAG, "device address changed to 0x%02X", _address);
    } else {
        ESP_LOGE(SV04M_TAG, "setDeviceAddress failed: %s", statusToString(ret));
    }

    return ret;
}

SV04M::Status SV04M::setRS485Baud(BaudIndex baudIndex)
{
    if (baudIndex > BAUD_115200) {
        ESP_LOGE(SV04M_TAG, "setRS485Baud invalid baudIndex=%u", baudIndex);
        return STATUS_INVALID_ARGUMENT;
    }

    Status ret = sendFactoryCommand(0x02, (uint32_t)baudIndex, nullptr, 1000);
    if (ret != STATUS_OK) {
        ESP_LOGE(SV04M_TAG, "setRS485Baud failed: %s", statusToString(ret));
    }
    return ret;
}

SV04M::Status SV04M::setAutoReset(bool enabled)
{
    Status ret = sendFactoryCommand(0x0E, enabled ? 0x01UL : 0x00UL, nullptr, 1000);
    if (ret != STATUS_OK) {
        ESP_LOGE(SV04M_TAG, "setAutoReset failed: %s", statusToString(ret));
    }
    return ret;
}

SV04M::Status SV04M::setGroupAddress(uint8_t groupIndex, uint8_t groupAddress)
{
    if (groupIndex < 1 || groupIndex > 4) {
        ESP_LOGE(SV04M_TAG, "setGroupAddress invalid groupIndex=%u", groupIndex);
        return STATUS_INVALID_ARGUMENT;
    }

    if (groupAddress < 0x80 || groupAddress > 0xFE) {
        ESP_LOGE(SV04M_TAG, "setGroupAddress invalid groupAddress=0x%02X", groupAddress);
        return STATUS_INVALID_ARGUMENT;
    }

    uint8_t func = 0x50 + (groupIndex - 1);
    Status ret = sendFactoryCommand(func, (uint32_t)groupAddress, nullptr, 1000);
    if (ret != STATUS_OK) {
        ESP_LOGE(SV04M_TAG, "setGroupAddress failed: %s", statusToString(ret));
    }
    return ret;
}

SV04M::Status SV04M::lockParameters()
{
    Status ret = sendFactoryCommand(0xFC, 0x00000000UL, nullptr, 1000);
    if (ret != STATUS_OK) {
        ESP_LOGE(SV04M_TAG, "lockParameters failed: %s", statusToString(ret));
    }
    return ret;
}

SV04M::Status SV04M::factoryReset()
{
    Status ret = sendFactoryCommand(0xFF, 0x00000000UL, nullptr, 1000);
    if (ret != STATUS_OK) {
        ESP_LOGE(SV04M_TAG, "factoryReset failed: %s", statusToString(ret));
    }
    return ret;
}

void SV04M::clearRxBuffer()
{
    while (_serial->available() > 0) {
        _serial->read();
    }

    delay(2);

    while (_serial->available() > 0) {
        _serial->read();
    }
}

uint16_t SV04M::sum16(const uint8_t *data, size_t len)
{
    uint16_t sum = 0;

    for (size_t i = 0; i < len; i++) {
        sum = (uint16_t)(sum + data[i]);
    }

    return sum;
}

SV04M::Status SV04M::statusFromByte(uint8_t statusByte)
{
    switch (statusByte) {
    case 0x00:
        return STATUS_OK;
    case 0x01:
        return STATUS_FRAME_ERROR;
    case 0x02:
        return STATUS_PARAM_ERROR;
    case 0x03:
        return STATUS_OPTO_ERROR;
    case 0x04:
        return STATUS_MOTOR_BUSY;
    case 0x05:
        return STATUS_MOTOR_STALL;
    case 0x06:
        return STATUS_UNKNOWN_POSITION;
    case 0x07:
        return STATUS_REJECTED;
    case 0xFE:
        return STATUS_TASK_PENDING;
    case 0xFF:
        return STATUS_UNKNOWN_ERROR;
    default:
        return STATUS_UNKNOWN_ERROR;
    }
}

SV04M::Status SV04M::writeAndRead(const uint8_t *frame,
                                  size_t len,
                                  Response *response,
                                  uint32_t timeoutMs)
{
    if (!_begun) {
        ESP_LOGE(SV04M_TAG, "writeAndRead failed: serial not begin");
        return STATUS_SERIAL_NOT_BEGIN;
    }

    clearRxBuffer();
    logFrame("TX", frame, len);

    _serial->write(frame, len);
    _serial->flush();

    uint32_t start = millis();
    Response temp;

    while ((uint32_t)(millis() - start) < timeoutMs) {
        uint32_t elapsed = (uint32_t)(millis() - start);
        uint32_t remain = (elapsed >= timeoutMs) ? 0 : (timeoutMs - elapsed);

        Status readStatus = readFrame(temp, remain);
        if (readStatus != STATUS_OK) {
            if (_debugEnabled || readStatus != STATUS_TIMEOUT) {
                ESP_LOGE(SV04M_TAG, "readFrame failed: %s", statusToString(readStatus));
            }
            return readStatus;
        }

        /* 部分 485 自动收发模块可能会回读自己的 8 字节发送帧，这里自动丢弃回显。 */
        if (len == 8 && memcmp(temp.raw, frame, 8) == 0) {
            if (_debugEnabled) {
                ESP_LOGI(SV04M_TAG, "ignore local echo");
            }
            continue;
        }

        _lastResponse = temp;
        if (response != nullptr) {
            *response = temp;
        }

        logFrame("RX", temp.raw, 8);

        if (temp.status != STATUS_OK &&
            temp.status != STATUS_TASK_PENDING &&
            temp.status != STATUS_MOTOR_BUSY) {
            ESP_LOGE(SV04M_TAG, "device status error: %s", statusToString(temp.status));
        }

        return temp.status;
    }

    ESP_LOGE(SV04M_TAG, "writeAndRead timeout, timeoutMs=%lu", (unsigned long)timeoutMs);
    return STATUS_TIMEOUT;
}

SV04M::Status SV04M::readFrame(Response &response, uint32_t timeoutMs)
{
    uint8_t buf[8];
    size_t index = 0;
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < timeoutMs) {
        while (_serial->available() > 0) {
            uint8_t b = (uint8_t)_serial->read();

            if (index == 0) {
                if (b != SV04M_STX) {
                    continue;
                }
            }

            buf[index++] = b;

            if (index >= sizeof(buf)) {
                if (buf[5] != SV04M_ETX) {
                    /* 帧尾不对时重新找帧头，增强抗干扰能力 */
                    uint8_t lastByte = buf[7];
                    index = 0;
                    if (lastByte == SV04M_STX) {
                        buf[index++] = lastByte;
                    }
                    continue;
                }

                uint16_t calc = sum16(buf, 6);
                uint16_t recv = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

                if (calc != recv) {
                    ESP_LOGE(SV04M_TAG,
                             "checksum error, calc=0x%04X, recv=0x%04X",
                             calc,
                             recv);
                    return STATUS_CHECKSUM_ERROR;
                }

                memcpy(response.raw, buf, sizeof(buf));
                response.address = buf[1];
                response.statusByte = buf[2];
                response.param = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
                response.status = statusFromByte(response.statusByte);

                return STATUS_OK;
            }
        }

        delay(1);
    }

    return STATUS_TIMEOUT;
}

void SV04M::logFrame(const char *prefix, const uint8_t *data, size_t len)
{
    if (!_debugEnabled) {
        return;
    }

    char hexText[128];
    size_t pos = 0;
    hexText[0] = '\0';

    for (size_t i = 0; i < len; i++) {
        int n = snprintf(hexText + pos,
                         sizeof(hexText) - pos,
                         "%02X%s",
                         data[i],
                         (i + 1 < len) ? " " : "");

        if (n < 0) {
            hexText[0] = '\0';
            break;
        }

        if ((size_t)n >= (sizeof(hexText) - pos)) {
            hexText[sizeof(hexText) - 1] = '\0';
            break;
        }

        pos += (size_t)n;
    }

    ESP_LOGI(SV04M_TAG, "%s: %s", prefix, hexText);
}

const char *SV04M::statusToString(Status status)
{
    switch (status) {
    case STATUS_OK:
        return "OK";
    case STATUS_FRAME_ERROR:
        return "FRAME_ERROR";
    case STATUS_PARAM_ERROR:
        return "PARAM_ERROR";
    case STATUS_OPTO_ERROR:
        return "OPTO_ERROR";
    case STATUS_MOTOR_BUSY:
        return "MOTOR_BUSY";
    case STATUS_MOTOR_STALL:
        return "MOTOR_STALL";
    case STATUS_UNKNOWN_POSITION:
        return "UNKNOWN_POSITION";
    case STATUS_REJECTED:
        return "REJECTED";
    case STATUS_TASK_PENDING:
        return "TASK_PENDING";
    case STATUS_UNKNOWN_ERROR:
        return "UNKNOWN_ERROR";
    case STATUS_TIMEOUT:
        return "TIMEOUT";
    case STATUS_CHECKSUM_ERROR:
        return "CHECKSUM_ERROR";
    case STATUS_FRAME_FORMAT_ERR:
        return "FRAME_FORMAT_ERROR";
    case STATUS_INVALID_ARGUMENT:
        return "INVALID_ARGUMENT";
    case STATUS_SERIAL_NOT_BEGIN:
        return "SERIAL_NOT_BEGIN";
    default:
        return "UNDEFINED_STATUS";
    }
}

uint32_t SV04M::baudFromIndex(BaudIndex baudIndex)
{
    switch (baudIndex) {
    case BAUD_9600:
        return 9600;
    case BAUD_19200:
        return 19200;
    case BAUD_38400:
        return 38400;
    case BAUD_57600:
        return 57600;
    case BAUD_115200:
        return 115200;
    default:
        return 9600;
    }
}

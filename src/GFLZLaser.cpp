#ifndef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#endif

#include "GFLZLaser.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *const GFLZ_TAG = "GFLZ";
static const uint8_t GFLZ_FUNC_READ_INPUT_REG = 0x04;
static const uint8_t GFLZ_FUNC_EXCEPTION_MASK = 0x80;
static const uint16_t GFLZ_DISTANCE_REG = 0x0000;
static const uint16_t GFLZ_DISTANCE_REG_COUNT = 0x0002;

GFLZLaser::GFLZLaser(HardwareSerial &serial)
    : _serial(&serial),
      _address(0x01),
      _begun(false),
      _debugEnabled(false),
      _lastExceptionCode(0)
{
    memset(&_lastDistance, 0, sizeof(_lastDistance));
}

void GFLZLaser::begin(uint32_t baud,
                      int8_t rxPin,
                      int8_t txPin,
                      uint8_t defaultAddress)
{
    _address = defaultAddress;

    /* ESP32 Arduino begin 参数顺序：baud, config, RX 引脚, TX 引脚 */
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
    _serial->setTimeout(50);
    _begun = true;

    delay(20);
    clearRxBuffer();

    if (_debugEnabled) {
        ESP_LOGI(GFLZ_TAG,
                 "begin ok, baud=%lu, rx=%d, tx=%d, defaultAddr=0x%02X",
                 (unsigned long)baud,
                 (int)rxPin,
                 (int)txPin,
                 _address);
    }
}

void GFLZLaser::begin(uint8_t defaultAddress)
{
    _address = defaultAddress;
    _begun = true;
    clearRxBuffer();

    if (_debugEnabled) {
        ESP_LOGI(GFLZ_TAG, "attach to existing RS485 serial, defaultAddr=0x%02X", _address);
    }
}

void GFLZLaser::setAddress(uint8_t address)
{
    _address = address;
}

uint8_t GFLZLaser::address() const
{
    return _address;
}

void GFLZLaser::setDebugEnabled(bool enabled)
{
    _debugEnabled = enabled;
}

bool GFLZLaser::debugEnabled() const
{
    return _debugEnabled;
}

uint8_t GFLZLaser::lastExceptionCode() const
{
    return _lastExceptionCode;
}

const GFLZLaser::DistanceData &GFLZLaser::lastDistance() const
{
    return _lastDistance;
}

GFLZLaser::Status GFLZLaser::readDistanceUm(int32_t &distanceUm, uint32_t timeoutMs)
{
    return readDistanceUm(_address, distanceUm, timeoutMs);
}

GFLZLaser::Status GFLZLaser::readDistanceUm(uint8_t address, int32_t &distanceUm, uint32_t timeoutMs)
{
    DistanceData data;
    Status ret = readDistance(address, data, timeoutMs);
    if (ret == STATUS_OK) {
        distanceUm = data.distanceUm;
    }
    return ret;
}

GFLZLaser::Status GFLZLaser::readDistanceMm(float &distanceMm, uint32_t timeoutMs)
{
    return readDistanceMm(_address, distanceMm, timeoutMs);
}

GFLZLaser::Status GFLZLaser::readDistanceMm(uint8_t address, float &distanceMm, uint32_t timeoutMs)
{
    DistanceData data;
    Status ret = readDistance(address, data, timeoutMs);
    if (ret == STATUS_OK) {
        distanceMm = data.distanceMm;
    }
    return ret;
}

GFLZLaser::Status GFLZLaser::readDistance(uint8_t address, DistanceData &data, uint32_t timeoutMs)
{
    uint8_t payload[4];
    Status ret = readInputRegisters(address,
                                    GFLZ_DISTANCE_REG,
                                    GFLZ_DISTANCE_REG_COUNT,
                                    payload,
                                    sizeof(payload),
                                    timeoutMs);
    if (ret != STATUS_OK) {
        ESP_LOGE(GFLZ_TAG,
                 "readDistance addr=0x%02X failed: %s",
                 address,
                 statusToString(ret));
        return ret;
    }

    data.address = address;
    data.distanceUm = parseSigned32BigEndian(payload);
    data.distanceMm = (float)data.distanceUm / 1000.0f;

    memcpy(data.raw, _lastDistance.raw, _lastDistance.rawLength);
    data.rawLength = _lastDistance.rawLength;

    _lastDistance = data;

    if (_debugEnabled) {
        ESP_LOGI(GFLZ_TAG,
                 "distance addr=0x%02X: %ld um, %.3f mm",
                 address,
                 (long)data.distanceUm,
                 (double)data.distanceMm);
    }

    return STATUS_OK;
}

GFLZLaser::Status GFLZLaser::readInputRegisters(uint8_t address,
                                                uint16_t startRegister,
                                                uint16_t registerCount,
                                                uint8_t *outData,
                                                size_t outDataSize,
                                                uint32_t timeoutMs)
{
    if (!_begun) {
        ESP_LOGE(GFLZ_TAG, "readInputRegisters failed: serial not begin");
        return STATUS_SERIAL_NOT_BEGIN;
    }

    if (!isValidAddress(address) || registerCount == 0 || registerCount > 32 || outData == nullptr) {
        ESP_LOGE(GFLZ_TAG,
                 "readInputRegisters invalid arg, addr=0x%02X, count=%u",
                 address,
                 registerCount);
        return STATUS_INVALID_ARGUMENT;
    }

    const uint8_t expectedByteCount = (uint8_t)(registerCount * 2U);
    if (outDataSize < expectedByteCount) {
        ESP_LOGE(GFLZ_TAG,
                 "readInputRegisters outData too small, need=%u, have=%u",
                 expectedByteCount,
                 (unsigned int)outDataSize);
        return STATUS_INVALID_ARGUMENT;
    }

    uint8_t request[8];
    request[0] = address;
    request[1] = GFLZ_FUNC_READ_INPUT_REG;
    request[2] = (uint8_t)((startRegister >> 8) & 0xFF);
    request[3] = (uint8_t)(startRegister & 0xFF);
    request[4] = (uint8_t)((registerCount >> 8) & 0xFF);
    request[5] = (uint8_t)(registerCount & 0xFF);

    uint16_t crc = crc16(request, 6);
    request[6] = (uint8_t)(crc & 0xFF);         // Modbus CRC 低字节在前
    request[7] = (uint8_t)((crc >> 8) & 0xFF);  // Modbus CRC 高字节在后

    uint8_t response[64];
    size_t responseLen = 0;

    Status ret = writeAndReadModbus(address,
                                    GFLZ_FUNC_READ_INPUT_REG,
                                    request,
                                    sizeof(request),
                                    expectedByteCount,
                                    response,
                                    sizeof(response),
                                    responseLen,
                                    timeoutMs);
    if (ret != STATUS_OK) {
        return ret;
    }

    if (responseLen < (size_t)(5U + expectedByteCount)) {
        ESP_LOGE(GFLZ_TAG, "response length too short: %u", (unsigned int)responseLen);
        return STATUS_FRAME_ERROR;
    }

    memcpy(outData, &response[3], expectedByteCount);

    memset(&_lastDistance, 0, sizeof(_lastDistance));
    size_t copyLen = responseLen;
    if (copyLen > sizeof(_lastDistance.raw)) {
        copyLen = sizeof(_lastDistance.raw);
    }
    memcpy(_lastDistance.raw, response, copyLen);
    _lastDistance.rawLength = (uint8_t)copyLen;

    return STATUS_OK;
}

GFLZLaser::Status GFLZLaser::writeAndReadModbus(uint8_t address,
                                                uint8_t function,
                                                const uint8_t *request,
                                                size_t requestLen,
                                                uint8_t expectedByteCount,
                                                uint8_t *response,
                                                size_t responseSize,
                                                size_t &responseLen,
                                                uint32_t timeoutMs)
{
    if (!_begun) {
        ESP_LOGE(GFLZ_TAG, "writeAndReadModbus failed: serial not begin");
        return STATUS_SERIAL_NOT_BEGIN;
    }

    if (request == nullptr || response == nullptr || responseSize < (size_t)(5U + expectedByteCount)) {
        return STATUS_INVALID_ARGUMENT;
    }

    _lastExceptionCode = 0;
    responseLen = 0;

    clearRxBuffer();
    logFrame("TX", request, requestLen);

    _serial->write(request, requestLen);
    _serial->flush();

    uint8_t rxBuf[96];
    size_t rxLen = 0;
    bool crcCandidateSeen = false;
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < timeoutMs) {
        while (_serial->available() > 0) {
            uint8_t b = (uint8_t)_serial->read();
            if (rxLen < sizeof(rxBuf)) {
                rxBuf[rxLen++] = b;
            } else {
                /* 缓冲区满时左移，继续保留最新数据 */
                memmove(rxBuf, rxBuf + 1, sizeof(rxBuf) - 1);
                rxBuf[sizeof(rxBuf) - 1] = b;
                rxLen = sizeof(rxBuf);
            }

            const size_t normalLen = (size_t)(5U + expectedByteCount);
            const size_t exceptionLen = 5;

            for (size_t offset = 0; offset < rxLen; offset++) {
                /* 正常响应：ADDR FUNC BYTE_COUNT DATA... CRC_LO CRC_HI */
                if ((rxLen - offset) >= normalLen &&
                    rxBuf[offset] == address &&
                    rxBuf[offset + 1] == function &&
                    rxBuf[offset + 2] == expectedByteCount) {

                    uint16_t calc = crc16(&rxBuf[offset], normalLen - 2);
                    uint16_t recv = (uint16_t)rxBuf[offset + normalLen - 2] |
                                    ((uint16_t)rxBuf[offset + normalLen - 1] << 8);

                    if (calc == recv) {
                        memcpy(response, &rxBuf[offset], normalLen);
                        responseLen = normalLen;
                        logFrame("RX", response, responseLen);
                        return STATUS_OK;
                    }

                    crcCandidateSeen = true;
                }

                /* 异常响应：ADDR FUNC|0x80 EXCEPTION_CODE CRC_LO CRC_HI */
                if ((rxLen - offset) >= exceptionLen &&
                    rxBuf[offset] == address &&
                    rxBuf[offset + 1] == (uint8_t)(function | GFLZ_FUNC_EXCEPTION_MASK)) {

                    uint16_t calc = crc16(&rxBuf[offset], exceptionLen - 2);
                    uint16_t recv = (uint16_t)rxBuf[offset + exceptionLen - 2] |
                                    ((uint16_t)rxBuf[offset + exceptionLen - 1] << 8);

                    if (calc == recv) {
                        _lastExceptionCode = rxBuf[offset + 2];
                        memcpy(response, &rxBuf[offset], exceptionLen);
                        responseLen = exceptionLen;
                        logFrame("RX_EX", response, responseLen);
                        ESP_LOGE(GFLZ_TAG,
                                 "modbus exception, addr=0x%02X, func=0x%02X, code=0x%02X",
                                 address,
                                 function,
                                 _lastExceptionCode);
                        return STATUS_EXCEPTION;
                    }

                    crcCandidateSeen = true;
                }
            }
        }

        delay(1);
    }

    if (rxLen > 0 && _debugEnabled) {
        logFrame("RX_PART", rxBuf, rxLen);
    }

    if (crcCandidateSeen) {
        ESP_LOGE(GFLZ_TAG, "crc error or incomplete valid frame, addr=0x%02X", address);
        return STATUS_CRC_ERROR;
    }

    ESP_LOGE(GFLZ_TAG, "timeout, addr=0x%02X, timeoutMs=%lu", address, (unsigned long)timeoutMs);
    return STATUS_TIMEOUT;
}

void GFLZLaser::clearRxBuffer()
{
    while (_serial->available() > 0) {
        _serial->read();
    }

    delay(2);

    while (_serial->available() > 0) {
        _serial->read();
    }
}

void GFLZLaser::logFrame(const char *prefix, const uint8_t *data, size_t len)
{
    if (!_debugEnabled) {
        return;
    }

    char hexText[192];
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

    ESP_LOGI(GFLZ_TAG, "%s: %s", prefix, hexText);
}

uint16_t GFLZLaser::crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

bool GFLZLaser::isValidAddress(uint8_t address)
{
    return (address >= 0x01 && address <= 0xF7);
}

int32_t GFLZLaser::parseSigned32BigEndian(const uint8_t *data)
{
    uint32_t value = ((uint32_t)data[0] << 24) |
                     ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) |
                     ((uint32_t)data[3]);
    return (int32_t)value;
}

const char *GFLZLaser::statusToString(Status status)
{
    switch (status) {
    case STATUS_OK:
        return "OK";
    case STATUS_TIMEOUT:
        return "TIMEOUT";
    case STATUS_CRC_ERROR:
        return "CRC_ERROR";
    case STATUS_FRAME_ERROR:
        return "FRAME_ERROR";
    case STATUS_EXCEPTION:
        return "MODBUS_EXCEPTION";
    case STATUS_INVALID_ARGUMENT:
        return "INVALID_ARGUMENT";
    case STATUS_SERIAL_NOT_BEGIN:
        return "SERIAL_NOT_BEGIN";
    default:
        return "UNDEFINED_STATUS";
    }
}

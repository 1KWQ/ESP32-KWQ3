#include "crc16.h"


/**
 * @brief CRC16 (Modbus) 计算
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC16 值
 */
uint16_t crc16_modbus(const uint8_t* data,size_t len){
    uint16_t crc=0xFFFF;
    for(size_t i=0;i<len;i++){
        crc ^= data[i];
        for(uint8_t j=0;j<8;j++){
            if(crc&0x0001) crc=(crc>>1)^0xA001;
            else crc>>=1;
        }
    }
    return crc;
}


/**
 * @brief 发送带 CRC 的应答
 * @param fmt printf 风格格式（不包含 #CRC）
 */
void sendWithCRC(const char* fmt, ...)
{
    char buf[128];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // 计算 CRC（不包含 # 和 \n）
    uint16_t crc = crc16_modbus((const uint8_t*)buf, strlen(buf));

    // 输出：<BODY>#<CRC>\n
    Serial.printf("%s#%04X\n", buf, crc);
}

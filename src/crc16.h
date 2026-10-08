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
    SV04M_CHANGE_FAILED // 切换 SV-04M 通道失败
}ErrCode;


typedef enum
{
    TASK_DONE = 300,  // 任务完成
    CUR_TASK_DONE = 301,  // 当前任务完成
    CUR_TASK_SKIP = 302,  // 当前任务跳过
    CHANGE_SOLVENT = 303,  // 更换溶剂
    FAILED_REMOVE = 304,  // 放料失败
    MOVE_D_TIMEOUT = 305,  // 移动D点超时
}EVentCode;


#endif

#ifndef PCA_GPIO_H
#define PCA_GPIO_H
#pragma once
#include <Arduino.h>
#include <PCA95x5.h>

#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


typedef void (*pca_isr_t)(void);

class PCAGPIO {
public:
    bool begin(TwoWire& wire, uint8_t addr);

    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t val);
    int  digitalRead(uint8_t pin);

    void attachInterrupt(uint8_t pin, pca_isr_t cb, int mode);
    void detachInterrupt(uint8_t pin);

    void pollInterrupt();

    inline void isrNotify() { irq_flag = true; }

private:
    /* ===== I2C Command ===== */
    enum CmdType {
        CMD_WRITE,
        CMD_READ,
        CMD_DIR
    };

    struct Cmd {
        CmdType type;
        uint16_t value;
        uint16_t *readBack;
        TaskHandle_t notify;
    };

    static void i2cTask(void *arg);
    void sendCmd(Cmd &cmd);

private:
    TwoWire *wire = nullptr;
    uint8_t address = 0;

    uint16_t dir_cache = 0xFFFF;
    uint16_t out_cache = 0x0000;
    uint16_t last_input = 0;

    QueueHandle_t cmdQueue = nullptr;
    TaskHandle_t  i2cTaskHandle = nullptr;

    volatile bool irq_flag = false;

    pca_isr_t isr_cb[16]   = {nullptr};
    uint8_t   isr_mode[16] = {0};
};



#endif

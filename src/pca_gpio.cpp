#include "pca_gpio.h"
#include "esp_log.h"
static PCA9555 pca;    // 只在 I2C Task 使用

bool PCAGPIO::begin(TwoWire& w, uint8_t addr)
{
    wire = &w;
    address = addr;

    cmdQueue = xQueueCreate(10, sizeof(Cmd));
    if (!cmdQueue) return false;

    BaseType_t res = xTaskCreatePinnedToCore(
        i2cTask,
        "pcaI2C",
        4096,
        this,
        2,
        &i2cTaskHandle,
        0
    );
    if(pdTRUE != res){
         ESP_LOGE("PCA", "i2cTask creat failed");
        return false;
        
    }
    // 初始化缓存
    dir_cache = 0xFFFF;
    out_cache = 0x0000;

    // 设置方向
    Cmd cmd1 = { CMD_DIR, dir_cache, nullptr, nullptr };
    sendCmd(cmd1);

    // 设置输出
    Cmd cmd2 = { CMD_WRITE, out_cache, nullptr, nullptr };
    sendCmd(cmd2);

    // 清中断
    Cmd cmd3 = { CMD_READ, 0, &last_input, xTaskGetCurrentTaskHandle() };
    sendCmd(cmd3);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    return true;
}

/* ================= GPIO ================= */

void PCAGPIO::pinMode(uint8_t pin, uint8_t mode)
{
    if (pin >= 16) return;

    if (mode == OUTPUT)
        dir_cache &= ~(1 << pin);
    else
        dir_cache |= (1 << pin);

    Cmd cmd = { CMD_DIR, dir_cache, nullptr, nullptr };
    sendCmd(cmd);
}

void PCAGPIO::digitalWrite(uint8_t pin, uint8_t val)
{
    if (pin >= 16) return;

    if (val == HIGH)
        out_cache |= (1 << pin);
    else
        out_cache &= ~(1 << pin);

    Cmd cmd = { CMD_WRITE, out_cache, nullptr, nullptr };
    sendCmd(cmd);
}

int PCAGPIO::digitalRead(uint8_t pin)
{
    if (pin >= 16) return LOW;

    uint16_t value = 0;
    Cmd cmd = { CMD_READ, 0, &value, xTaskGetCurrentTaskHandle() };
    sendCmd(cmd);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    return (value & (1 << pin)) ? HIGH : LOW;
}

/* ================= Interrupt ================= */

void PCAGPIO::attachInterrupt(uint8_t pin, pca_isr_t cb, int mode)
{
    if (pin >= 16) return;

    isr_cb[pin]   = cb;
    isr_mode[pin] = mode;
    pinMode(pin, INPUT);
}

void PCAGPIO::detachInterrupt(uint8_t pin)
{
    if (pin >= 16) return;
    isr_cb[pin] = nullptr;
}

void PCAGPIO::pollInterrupt()
{
    static uint32_t last_ts[16] = {0};

    if (!irq_flag) return;
    irq_flag = false;

    uint16_t now = 0;
    Cmd cmd = { CMD_READ, 0, &now, xTaskGetCurrentTaskHandle() };
    sendCmd(cmd);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint16_t diff = now ^ last_input;
    uint32_t now_ms = millis();

    for (int i = 0; i < 16; i++) {
        if (!(diff & (1 << i))) continue;
        if (!isr_cb[i]) continue;

        bool old_state = last_input & (1 << i);
        bool new_state = now & (1 << i);

        bool trigger = false;
        switch (isr_mode[i]) {
        case CHANGE:  trigger = true; break;
        case RISING:  trigger = (!old_state && new_state); break;
        case FALLING: trigger = (old_state && !new_state); break;
        }

        if (trigger) {
            if (now_ms - last_ts[i] < 20) continue;
            last_ts[i] = now_ms;
            isr_cb[i]();
        }
    }

    last_input = now;
}

/* ================= I2C Task ================= */

void PCAGPIO::sendCmd(Cmd &cmd)
{
    xQueueSend(cmdQueue, &cmd, portMAX_DELAY);
}

void PCAGPIO::i2cTask(void *arg)
{
    PCAGPIO *self = static_cast<PCAGPIO*>(arg);

    pca.attach(*self->wire, self->address);

    Cmd cmd;
    while (1) {
        if (xQueueReceive(self->cmdQueue, &cmd, portMAX_DELAY)) {

            switch (cmd.type) {
            case CMD_WRITE:
                pca.write(cmd.value);
                break;

            case CMD_DIR:
                pca.direction(cmd.value);
                break;

            case CMD_READ:
                if (cmd.readBack)
                    *cmd.readBack = pca.read();
                break;
            }

            if (cmd.notify) {
                xTaskNotifyGive(cmd.notify);
            }
        }
    }
}

/* 消抖时间
| 场景    | 建议       |
| ----- | -------- |
| 机械按键  | 15~30 ms |
| 编码器   | 3~5 ms   |
| 光耦    | 1~2 ms   |
| 继电器触点 | 20~50 ms |

*/


#include "driver_ch9120_interface.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

/**
 * @brief 项目侧提供的 CH9120 板级 GPIO 钩子。
 *
 * @details
 * CH9120 库位于 lib 目录，不能直接依赖工程 src 目录里的 XL9555/PCA95x5
 * 头文件，否则 PlatformIO 编译库时会缺少 include 路径。这里仅声明一组
 * C 链接的板级钩子，具体的 XL9555 引脚操作由 `src/ch9120_net.cpp` 实现。
 */
extern "C" {
uint8_t ch9120_board_reset_gpio_init(void);
uint8_t ch9120_board_reset_gpio_deinit(void);
uint8_t ch9120_board_reset_gpio_write(uint8_t data);
uint8_t ch9120_board_cfg_gpio_init(void);
uint8_t ch9120_board_cfg_gpio_deinit(void);
uint8_t ch9120_board_cfg_gpio_write(uint8_t data);
}

/**
 * @brief 项目侧提供的 Web 日志写入接口。
 *
 * @details
 * 该函数是 C++ 链接接口，因此不能放进上面的 `extern "C"` 块。这里仅声明
 * 函数原型，避免 CH9120 库目录直接依赖项目 `src` 目录里的头文件搜索路径。
 */

/**
 * @brief CH9120 配置阶段默认串口波特率。
 *
 * @details
 * CH9120 出厂串口配置波特率通常是 9600。网络透传工作波特率会在
 * `driver_ch9120_basic.h` 中写成 115200，使芯片配置完成后和本工程
 * 的 `Serial.begin(115200)` 保持一致。
 */
#define CH9120_CONFIG_UART_BAUD 9600U

static bool s_uart_initialized = false;               /**< Serial 是否已被 CH9120 接口层打开 */
static uint32_t s_uart_baud = CH9120_CONFIG_UART_BAUD; /**< 当前用于配置 CH9120 的 Serial 波特率 */

/**
 * @brief 设置 CH9120 配置通信使用的 Serial 波特率。
 *
 * @param baud 目标波特率。
 *
 * @details
 * ESP32 默认串口同时接电脑调试口和 CH9120 串口透传口。配置 CH9120 时，
 * 需要先用芯片当前保存的波特率通信；配置完成后再恢复为业务指令使用的
 * 115200。该函数只记录目标波特率，真正重新打开串口由
 * `ch9120_interface_uart_init()` 完成。
 */
void ch9120_interface_set_uart_baud(uint32_t baud)
{
    s_uart_baud = baud;
}

/**
 * @brief 初始化 CH9120 配置串口。
 * @return 0 表示成功。
 *
 * @details
 * 当前硬件把 CH9120 串口连接到 ESP32 默认 Serial，因此这里不能再使用
 * Serial1，也不能占用 GPIO6/GPIO7，避免和现有 Y 轴限位 IO 冲突。
 */
uint8_t ch9120_interface_uart_init(void)
{
    if (s_uart_initialized)
    {
        Serial.flush();
        Serial.end();
        s_uart_initialized = false;
        delay(20);
    }

    Serial.begin(s_uart_baud);
    delay(50);
    while (Serial.available() > 0)
    {
        (void)Serial.read();
    }
    s_uart_initialized = true;

    return 0;
}

/**
 * @brief 关闭 CH9120 配置串口。
 * @return 0 表示成功。
 *
 * @details
 * 这里只关闭由接口层打开的 Serial，调用方会在配置结束后重新
 * `Serial.begin(115200)`，让现有串口命令解析继续工作。
 */
uint8_t ch9120_interface_uart_deinit(void)
{
    if (s_uart_initialized)
    {
        Serial.flush();
        Serial.end();
        s_uart_initialized = false;
    }
    return 0;
}

/**
 * @brief 从 CH9120 配置串口读取数据。
 * @param[out] buf 接收缓冲区。
 * @param[in] len 期望读取长度。
 * @return 实际读取到的字节数。
 */
uint16_t ch9120_interface_uart_read(uint8_t *buf, uint16_t len)
{
    uint16_t count = 0;
    uint32_t start_ms;

    if ((buf == NULL) || (len == 0) || !s_uart_initialized)
    {
        return 0;
    }

    start_ms = millis();
    while ((count < len) && ((millis() - start_ms) < 10U))
    {
        while ((count < len) && (Serial.available() > 0))
        {
            buf[count++] = (uint8_t)Serial.read();
        }
        if (count > 0)
        {
            break;
        }
        delay(1);
    }

    return count;
}

/**
 * @brief 向 CH9120 配置串口写入数据。
 * @param[in] buf 待发送缓冲区。
 * @param[in] len 待发送长度。
 * @return 0 表示成功，1 表示失败。
 */
uint8_t ch9120_interface_uart_write(uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0) || !s_uart_initialized)
    {
        return 1;
    }

    size_t written = Serial.write(buf, len);
    Serial.flush();

    return written == len ? 0 : 1;
}

/**
 * @brief 清空 CH9120 配置串口接收缓存。
 * @return 0 表示成功，1 表示串口未初始化。
 */
uint8_t ch9120_interface_uart_flush(void)
{
    if (!s_uart_initialized)
    {
        return 1;
    }

    while (Serial.available() > 0)
    {
        (void)Serial.read();
    }

    return 0;
}

/**
 * @brief 初始化 CH9120 RSTI 外部复位脚。
 * @return 0 表示成功。
 *
 * @details
 * LibDriver 内部的 reset_gpio 是短脉冲复位用途，应连接到 CH9120 的
 * RSTI，而不是 RESET 恢复出厂设置脚。RSTI 低电平有效，空闲时保持高电平。
 */
uint8_t ch9120_interface_reset_gpio_init(void)
{
    return ch9120_board_reset_gpio_init();
}

/**
 * @brief 释放 CH9120 RSTI 外部复位脚。
 * @return 0 表示成功。
 *
 * @details
 * 为避免 CH9120 被误复位，退出时仍保持 RSTI 高电平输出。
 */
uint8_t ch9120_interface_reset_gpio_deinit(void)
{
    return ch9120_board_reset_gpio_deinit();
}

/**
 * @brief 写 CH9120 RSTI 外部复位脚。
 * @param[in] data 0 输出低电平复位，非 0 输出高电平释放复位。
 * @return 0 表示成功。
 */
uint8_t ch9120_interface_reset_gpio_write(uint8_t data)
{
    return ch9120_board_reset_gpio_write(data);
}

/**
 * @brief 初始化 CH9120 CFGO 配置模式脚。
 * @return 0 表示成功。
 *
 * @details
 * CFGO 内部上拉，低电平进入串口配置模式，高电平退出配置模式。
 * 因为该脚接在 XL9555 上，这里必须通过 pca 控制。
 */
uint8_t ch9120_interface_cfg_gpio_init(void)
{
    return ch9120_board_cfg_gpio_init();
}

/**
 * @brief 释放 CH9120 CFGO 配置模式脚。
 * @return 0 表示成功。
 *
 * @details
 * 退出配置后保持高电平，使 CH9120 留在正常网络透传模式。
 */
uint8_t ch9120_interface_cfg_gpio_deinit(void)
{
    return ch9120_board_cfg_gpio_deinit();
}

/**
 * @brief 写 CH9120 CFGO 配置模式脚。
 * @param[in] data 0 进入配置模式，非 0 退出配置模式。
 * @return 0 表示成功。
 */
uint8_t ch9120_interface_cfg_gpio_write(uint8_t data)
{
    return ch9120_board_cfg_gpio_write(data);
}

/**
 * @brief 毫秒延时。
 * @param[in] ms 延时时间，单位 ms。
 */
void ch9120_interface_delay_ms(uint32_t ms)
{
    delay(ms);
}

/**
 * @brief CH9120 驱动调试输出。
 * @param[in] fmt printf 风格格式化字符串。
 *
 * @details
 * 当前 CH9120 和业务通信共用默认 Serial。配置阶段如果继续
 * `Serial.print()` 输出调试文本，会把文本混进 CH9120 配置帧，导致配置
 * 超时或写入错误。因此这里故意不输出任何内容。
 */
void ch9120_interface_debug_print(const char *const fmt, ...)
{
    char buffer[256];
    va_list args;
    size_t len;

    if (fmt == NULL)
    {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    /**
     * @brief 去掉驱动原始日志末尾的换行符。
     *
     * @details
     * LibDriver 的 debug 字符串大多自带 `\n`。Web 日志输出时会统一补换行，
     * 如果不清理末尾换行，Web 日志页面会出现额外空行。
     */
    len = strlen(buffer);
    while ((len > 0U) && ((buffer[len - 1U] == '\n') || (buffer[len - 1U] == '\r')))
    {
        buffer[len - 1U] = '\0';
        len--;
    }

    if (len == 0U)
    {
        return;
    }

    /**
     * @brief 将 CH9120 驱动内部日志写入 Web 日志缓存。
     *
     * @details
     * 当前 CH9120 和业务控制共用默认 Serial，不能使用 Serial.print 或
     * ESP_LOG 输出驱动调试信息，否则可能污染 CH9120 配置帧或业务协议。
     * 这里直接调用项目侧 `WiFiLog_Statusf()`，只进入 Web 日志环形缓存。
     */
    /* Serial is the CH9120 configuration/data channel; never log to it. */
    (void)buffer;
}

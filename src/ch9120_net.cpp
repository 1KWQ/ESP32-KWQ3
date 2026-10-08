#include "ch9120_net.h"

#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver_ch9120_basic.h"
#include "driver_ch9120_interface.h"
#include "exio.h"

/*
 * CH9120 shares the default Serial with the PC command channel.  Do not write
 * diagnostic text to Serial here: while the chip is in configuration mode it
 * would be interpreted as a CH9120 command, and in normal mode it would be
 * mistaken for an application response.  Keep the status hook deliberately
 * quiet; the configuration result is available through Ch9120Net_IsTcpConnected().
 */
static void Ch9120Net_Logf(const char *fmt, ...)
{
    (void)fmt;
}
#define WiFiLog_Statusf Ch9120Net_Logf


/**
 * @brief CH9120 EEPROM 配置版本号。
 *
 * @details
 * 只有当该版本号变化，或者 `CH9120_NET_FORCE_CONFIG` 被置 1 时，程序才会
 * 重新写 CH9120 EEPROM。这样可以减少 EEPROM 重复擦写，也能在修改 IP/端口
 * 后通过递增版本号触发重新配置。
 */
#define CH9120_NET_CONFIG_VERSION 1U

/**
 * @brief 是否每次启动都强制写入 CH9120 配置。
 *
 * @details
 * 调试 CH9120 网络参数时可以临时改成 1；正式使用建议保持 0，避免每次启动
 * 都写 EEPROM。
 */
#define CH9120_NET_FORCE_CONFIG 0

/**
 * @brief CH9120 出厂/配置模式常见串口波特率。
 *
 * @details
 * 首次配置通常需要用 9600 和 CH9120 通信。若芯片已经被配置成 115200，
 * 初始化流程会自动再用 115200 尝试一次。
 */
#define CH9120_NET_CONFIG_UART_BAUD 9600U

/**
 * @brief Web 手动重配 CH9120 的后台任务栈大小。
 */
#define CH9120_NET_RECONFIG_TASK_STACK_SIZE 4096U

/**
 * @brief Web 手动重配 CH9120 的后台任务优先级。
 */
#define CH9120_NET_RECONFIG_TASK_PRIORITY 1U

/**
 * @brief TCPCS 状态重复打印间隔。
 *
 * @details
 * TCP 长时间连不上时，仅靠状态变化日志不够直观，因此每隔固定时间重复打印一次
 * TCPCS 电平和当前网络参数，方便现场对照服务器 IP、端口和网线状态。
 */
#define CH9120_NET_TCPCS_LOG_INTERVAL_MS 5000U

/**
 * @brief CH9120 控制脚写入后的最小稳定等待时间。
 *
 * @details
 * CH9120 的 CFGO/RSTI/RESET 都接在 XL9555 上，而 `PCAGPIO::digitalWrite()`
 * 只是把 I2C 写命令放入队列。配置驱动会在拉低 CFGO 后立刻发送串口配置帧，
 * 如果不等待 XL9555 实际完成输出，CH9120 可能还没进入配置模式就收到了配置帧，
 * 最终表现为 `ch9120: timeout`。这里通过短延时配合后续 `digitalRead()` 同步队列。
 */
#define CH9120_NET_GPIO_SETTLE_MS 20U

/**
 * @brief CH9120 网络工作模式。
 *
 * @details
 * 根据你提供的 TCPCS 说明，当前按 TCP Client 模式配置。TCPCS 低电平时
 * 表示客户端已经连接到服务器。
 */
#define CH9120_NET_WORK_MODE CH9120_MODE_TCP_CLIENT

/**
 * @brief CH9120 本机静态 IP。
 *
 * @details
 * 根据现场网段不同，修改这些数组和端口后，把
 * `CH9120_NET_CONFIG_VERSION` 加 1，即可触发重新写入 CH9120 EEPROM。
 */
static uint8_t s_local_ip[4] = {192, 168, 254, 251};

/** @brief CH9120 子网掩码。 */
static uint8_t s_subnet_mask[4] = {255, 255, 255, 0};

/** @brief CH9120 默认网关。 */
static uint8_t s_gateway[4] = {192, 168, 254, 1};

/** @brief CH9120 本地端口。 */
static uint16_t s_local_port = 5000;

/** @brief TCP Client 模式下要连接的服务器 IP。 */
static uint8_t s_server_ip[4] = {192, 168, 254, 179};

/** @brief TCP Client 模式下要连接的服务器端口。 */
static uint16_t s_server_port = 50000;

static volatile bool s_is_configuring = false; /**< CH9120 是否正在占用默认 Serial 做配置 */
static int8_t s_last_tcpcs_state = -1;         /**< 最近一次打印的 TCPCS 状态，-1 表示尚未打印 */
static uint32_t s_last_tcpcs_log_ms = 0;       /**< 最近一次打印 TCPCS 状态的时间戳 */

/**
 * @brief 初始化 CH9120 RSTI 外部复位脚。
 * @return 0 表示成功。
 *
 * @details
 * 该函数给 `lib/CH9120` 中的接口层调用。LibDriver 的 reset_gpio 语义是
 * “低有效短脉冲复位”，所以这里映射到 CH9120 的 RSTI，而不是会恢复出厂
 * 设置的 RESET 脚。
 */
/**
 * @brief 将电平值转换成便于 Web 页面查看的字符串。
 *
 * @param level 电平值，`LOW` 表示低电平，非 `LOW` 表示高电平。
 * @return const char* `"LOW"` 或 `"HIGH"`。
 */
static const char *Ch9120Net_LevelText(int level)
{
    return (level == LOW) ? "LOW" : "HIGH";
}

/**
 * @brief 同步写入 XL9555 上的 CH9120 控制脚。
 *
 * @param pin XL9555 逻辑引脚编号。
 * @param level 目标输出电平。
 * @return int 写入后从 XL9555 输入寄存器读回的实际电平。
 *
 * @details
 * `PCAGPIO::digitalWrite()` 当前是异步入队实现，函数返回并不代表 I2C 已经写完。
 * 本函数在写入后延时并读取一次同一芯片，利用 I2C 队列的 FIFO 顺序等待前面的写命令完成。
 * 对 CH9120 的 CFGO/RSTI 这类时序敏感脚，必须用这个入口，避免配置帧早于控制脚电平到达。
 */
static int Ch9120Net_WriteGpioSynced(uint8_t pin, uint8_t level)
{
    pca.digitalWrite(pin, level);
    delay(CH9120_NET_GPIO_SETTLE_MS);
    int readback = pca.digitalRead(pin);
    delay(CH9120_NET_GPIO_SETTLE_MS);

    return readback;
}

/**
 * @brief 记录 CH9120 相关控制脚的当前实际电平。
 *
 * @param stage 日志阶段名称，用于区分配置前、配置后和恢复工作模式等时刻。
 *
 * @details
 * Web 日志里的 timeout 只能说明没有收到 CH9120 应答，不能直接区分是串口线问题、
 * CH9120 未进入配置模式，还是 TCP 连接问题。这里把 RESET/RSTI/CFGO/TCPCS 同时打印，
 * 现场只要点“CH9120日志”即可看到配置模式脚是否真的被拉低、复位脚是否保持高电平。
 */
static void Ch9120Net_LogGpioState(const char *stage)
{
    int reset_level = pca.digitalRead(CH9120_RESET_PIN);
    int rsti_level = pca.digitalRead(CH9120_RSTI_PIN);
    int cfgo_level = pca.digitalRead(CH9120_CFGO_PIN);
    int tcpcs_level = pca.digitalRead(CH9120_TCPCS_PIN);

    WiFiLog_Statusf("[CH9120_NET] GPIO %s: RESET=%s, RSTI=%s, CFGO=%s, TCPCS=%s\r\n",
                    (stage != NULL) ? stage : "unknown",
                    Ch9120Net_LevelText(reset_level),
                    Ch9120Net_LevelText(rsti_level),
                    Ch9120Net_LevelText(cfgo_level),
                    Ch9120Net_LevelText(tcpcs_level));
}

extern "C" uint8_t ch9120_board_reset_gpio_init(void)
{
    pca.pinMode(CH9120_RSTI_PIN, OUTPUT);
    (void)Ch9120Net_WriteGpioSynced(CH9120_RSTI_PIN, HIGH);

    return 0;
}

/**
 * @brief 释放 CH9120 RSTI 外部复位脚。
 * @return 0 表示成功。
 *
 * @details
 * 退出驱动配置时仍保持 RSTI 高电平输出，避免 CH9120 被误复位。
 */
extern "C" uint8_t ch9120_board_reset_gpio_deinit(void)
{
    pca.pinMode(CH9120_RSTI_PIN, OUTPUT);
    (void)Ch9120Net_WriteGpioSynced(CH9120_RSTI_PIN, HIGH);

    return 0;
}

/**
 * @brief 写 CH9120 RSTI 外部复位脚。
 * @param[in] data 0 输出低电平复位，非 0 输出高电平释放复位。
 * @return 0 表示成功。
 */
extern "C" uint8_t ch9120_board_reset_gpio_write(uint8_t data)
{
    (void)Ch9120Net_WriteGpioSynced(CH9120_RSTI_PIN, data ? HIGH : LOW);

    return 0;
}

/**
 * @brief 初始化 CH9120 CFGO 配置模式脚。
 * @return 0 表示成功。
 *
 * @details
 * CFGO 低电平进入串口配置模式，高电平退出配置模式。该脚接到 XL9555 P15，
 * 因此必须通过 `pca` 操作。
 */
extern "C" uint8_t ch9120_board_cfg_gpio_init(void)
{
    pca.pinMode(CH9120_CFGO_PIN, OUTPUT);
    (void)Ch9120Net_WriteGpioSynced(CH9120_CFGO_PIN, HIGH);

    return 0;
}

/**
 * @brief 释放 CH9120 CFGO 配置模式脚。
 * @return 0 表示成功。
 *
 * @details
 * 配置结束后保持 CFGO 高电平，使 CH9120 工作在正常网络透传模式。
 */
extern "C" uint8_t ch9120_board_cfg_gpio_deinit(void)
{
    pca.pinMode(CH9120_CFGO_PIN, OUTPUT);
    (void)Ch9120Net_WriteGpioSynced(CH9120_CFGO_PIN, HIGH);

    return 0;
}

/**
 * @brief 写 CH9120 CFGO 配置模式脚。
 * @param[in] data 0 进入配置模式，非 0 退出配置模式。
 * @return 0 表示成功。
 */
extern "C" uint8_t ch9120_board_cfg_gpio_write(uint8_t data)
{
    (void)Ch9120Net_WriteGpioSynced(CH9120_CFGO_PIN, data ? HIGH : LOW);

    return 0;
}

/**
 * @brief 初始化 CH9120 接在 XL9555 上的控制脚。
 *
 * @details
 * - RESET 是恢复出厂设置脚，低电平保持 3 秒会清空参数，所以这里只保持高电平；
 * - RSTI 是外部复位输入，低电平有效，交给 CH9120 驱动做短脉冲复位；
 * - CFGO 是配置模式脚，低电平进入配置，高电平退出配置；
 * - TCPCS 是 TCP Client 连接状态输入，低电平表示连接成功。
 */
static void Ch9120Net_GpioInit(void)
{
    pca.pinMode(CH9120_RESET_PIN, OUTPUT);
    (void)Ch9120Net_WriteGpioSynced(CH9120_RESET_PIN, HIGH);

    (void)ch9120_board_reset_gpio_init();
    (void)ch9120_board_cfg_gpio_init();

    pca.pinMode(CH9120_TCPCS_PIN, INPUT);
    Ch9120Net_LogGpioState("after gpio init");
}

/**
 * @brief 判断是否需要重新写 CH9120 EEPROM。
 *
 * @return true 需要重新配置。
 * @return false 配置版本未变化，可以跳过 EEPROM 写入。
 */
static bool Ch9120Net_NeedConfig(void)
{
#if CH9120_NET_FORCE_CONFIG
    return true;
#else
    Preferences preferences;
    if (!preferences.begin("ch9120", true))
    {
        return true;
    }

    uint32_t saved_version = preferences.getUInt("cfg_ver", 0);
    preferences.end();

    return saved_version != CH9120_NET_CONFIG_VERSION;
#endif
}

/**
 * @brief 保存已成功写入的 CH9120 配置版本。
 *
 * @return true 保存成功。
 * @return false 保存失败。
 */
static bool Ch9120Net_SaveConfigVersion(void)
{
    Preferences preferences;
    if (!preferences.begin("ch9120", false))
    {
        return false;
    }

    size_t written = preferences.putUInt("cfg_ver", CH9120_NET_CONFIG_VERSION);
    preferences.end();

    return written == sizeof(uint32_t);
}

/**
 * @brief 清空默认 Serial 中残留的启动或配置数据。
 *
 * @details
 * CH9120 与业务命令共用默认 Serial。配置完成后清一次接收缓存，可以避免配置
 * 应答或上电杂字节被 `serialPoll()` 当成业务命令解析。
 */
static void Ch9120Net_ClearSerialRx(void)
{
    while (Serial.available() > 0)
    {
        (void)Serial.read();
    }
}

/**
 * @brief 使用指定波特率尝试写入 CH9120 网络参数。
 *
 * @param baud 用于和 CH9120 配置口通信的当前波特率。
 * @return true 配置成功。
 * @return false 配置失败。
 *
 * @details
 * 首次出厂芯片通常是 9600；已经配置过的芯片可能是 115200。调用方会按顺序
 * 尝试这两个波特率。函数结束时只关闭接口层打开的 Serial，不调用
 * `ch9120_basic_deinit()`，因为配置成功后芯片已经切到 115200，再用旧波特率
 * 发送 deinit 里的复位命令反而可能超时。
 */
static bool Ch9120Net_ConfigureAtBaud(uint32_t baud)
{
    /**
     * @brief 记录本次配置尝试使用的波特率。
     *
     * @details
     * CH9120 首次出厂常见配置波特率是 9600，已经配置过后可能变成 115200。
     * 这里把每次尝试写入 Web 日志，便于现场判断失败发生在哪个波特率阶段。
    */
    WiFiLog_Statusf("[CH9120_NET] try CH9120 config at %lu baud\r\n", (unsigned long)baud);
    Ch9120Net_LogGpioState("before baud try");

    ch9120_interface_set_uart_baud(baud);

    bool ok = false;
    if (ch9120_basic_init() == 0)
    {
        ok = (ch9120_basic_config(CH9120_NET_WORK_MODE,
                                  s_local_ip,
                                  s_local_port,
                                  s_subnet_mask,
                                  s_gateway,
                                  s_server_ip,
                                  s_server_port) == 0);
    }
    else
    {
        WiFiLog_Statusf("[CH9120_NET] CH9120 basic init failed at %lu baud\r\n", (unsigned long)baud);
        Ch9120Net_LogGpioState("basic init failed");
    }

    (void)ch9120_interface_uart_deinit();
    delay(100);

    if (ok)
    {
        WiFiLog_Statusf("[CH9120_NET] CH9120 config success at %lu baud\r\n", (unsigned long)baud);
        Ch9120Net_LogGpioState("config success");
    }
    else
    {
        WiFiLog_Statusf("[CH9120_NET] CH9120 config failed at %lu baud\r\n", (unsigned long)baud);
        Ch9120Net_LogGpioState("config failed");
    }

    return ok;
}

/**
 * @brief 恢复 CH9120 正常透传模式和默认 Serial 波特率。
 *
 * @details
 * 配置完成或失败后都必须把 RESET/RSTI/CFGO 保持高电平，并恢复
 * `Serial.begin(115200)`。否则 CH9120 可能停留在配置模式，现有串口命令
 * 解析也无法继续接收局域网透传数据。
 */
static void Ch9120Net_RestoreWorkMode(void)
{
    (void)Ch9120Net_WriteGpioSynced(CH9120_RESET_PIN, HIGH);
    (void)Ch9120Net_WriteGpioSynced(CH9120_RSTI_PIN, HIGH);
    (void)Ch9120Net_WriteGpioSynced(CH9120_CFGO_PIN, HIGH);
    Ch9120Net_LogGpioState("restore work mode");

    Serial.begin(CH9120_NET_UART_BAUD);
    delay(100);
    Ch9120Net_ClearSerialRx();
}

/**
 * @brief 执行一次 CH9120 配置检查或强制重配。
 *
 * @param force_config true 表示忽略版本号，强制重新写入 CH9120 EEPROM。
 * @param source 本次配置来源，用于 Web 日志区分开机初始化和手动按钮触发。
 * @return true 配置成功，或版本未变化且无需写入。
 * @return false 写入 CH9120 配置失败。
 *
 * @details
 * CH9120 配置阶段会临时关闭默认 Serial，再按 9600、115200 两个波特率尝试
 * 进入配置模式。由于默认 Serial 同时也是业务命令通道，函数开始时会置位
 * `s_is_configuring`，主循环据此暂停 `serialPoll()`，避免抢读 CH9120 应答。
 */
static bool Ch9120Net_ApplyConfig(bool force_config, const char *source)
{
    const char *source_text = (source != NULL) ? source : "unknown";
    bool config_ok = true;
    bool need_config = force_config || Ch9120Net_NeedConfig();

    s_is_configuring = true;
    Ch9120Net_GpioInit();

    WiFiLog_Statusf("[CH9120_NET] config start, source=%s, force=%d, need_config=%d\r\n",
                    source_text,
                    force_config ? 1 : 0,
                    need_config ? 1 : 0);

    if (need_config)
    {
        /**
         * @brief 进入 CH9120 配置前释放默认 Serial。
         *
         * @details
         * LibDriver 会重新按指定波特率打开 Serial。这里先 flush/end，可以避免
         * 上一次业务透传残留字节影响 CH9120 配置指令和应答判断。
         */
        Serial.flush();
        Serial.end();
        delay(50);

        config_ok = Ch9120Net_ConfigureAtBaud(CH9120_NET_CONFIG_UART_BAUD);
        if (!config_ok)
        {
            WiFiLog_Statusf("[CH9120_NET] retry CH9120 config at work baud %lu\r\n",
                            (unsigned long)CH9120_NET_UART_BAUD);
            config_ok = Ch9120Net_ConfigureAtBaud(CH9120_NET_UART_BAUD);
        }

        if (config_ok && !Ch9120Net_SaveConfigVersion())
        {
            WiFiLog_Statusf("[CH9120_NET] CH9120 config ok, but save config version failed\r\n");
        }
    }
    else
    {
        WiFiLog_Statusf("[CH9120_NET] CH9120 config version unchanged, skip EEPROM write\r\n");
    }

    Ch9120Net_RestoreWorkMode();
    s_is_configuring = false;

    /**
     * @brief 下次轮询强制打印 TCPCS 状态。
     *
     * @details
     * 手动重配后 TCP 重连可能需要几秒钟。把上次状态清空后，页面能立刻看到
     * 当前 TCPCS 电平，之后仍会按固定间隔继续打印。
     */
    s_last_tcpcs_state = -1;
    s_last_tcpcs_log_ms = 0;

    if (config_ok)
    {
        bool connected = Ch9120Net_IsTcpConnected();
        WiFiLog_Statusf("[CH9120_NET] ready, local %u.%u.%u.%u:%u, server %u.%u.%u.%u:%u, TCPCS=%s, TCP %s\r\n",
                        s_local_ip[0],
                        s_local_ip[1],
                        s_local_ip[2],
                        s_local_ip[3],
                        s_local_port,
                        s_server_ip[0],
                        s_server_ip[1],
                        s_server_ip[2],
                        s_server_ip[3],
                        s_server_port,
                        connected ? "LOW" : "HIGH",
                        connected ? "connected" : "disconnected");
    }
    else
    {
        WiFiLog_Statusf("[CH9120_NET] CH9120 config failed, Serial command parser still uses 115200\r\n");
    }

    return config_ok;
}

/**
 * @brief Web 手动重配 CH9120 的后台任务。
 *
 * @param param FreeRTOS 任务参数，未使用。
 *
 * @details
 * HTTP 请求只负责创建该任务并立即返回，实际配置过程在此任务内执行。这样 Web
 * 页面不会因为 CH9120 配置超时而长时间无响应，日志任务也能继续刷新页面内容。
 */
static void Ch9120Net_ReconfigTask(void *param)
{
    (void)param;

    WiFiLog_Statusf("[CH9120_NET] Web reconfigure task started\r\n");
    (void)Ch9120Net_ApplyConfig(true, "web");
    WiFiLog_Statusf("[CH9120_NET] Web reconfigure task finished\r\n");

    vTaskDelete(NULL);
}

/**
 * @brief 初始化 CH9120 网络透传功能。
 */
void Ch9120Net_Init(void)
{
    (void)Ch9120Net_ApplyConfig(false, "boot");
}

/**
 * @brief 请求后台任务强制重新配置 CH9120。
 */
bool Ch9120Net_RequestReconfigure(void)
{
    if (s_is_configuring)
    {
        WiFiLog_Statusf("[CH9120_NET] reconfigure request ignored, CH9120 is busy\r\n");
        return false;
    }

    /**
     * @brief 先置忙再创建任务，避免用户连续点击按钮创建多个配置任务。
     */
    s_is_configuring = true;
    BaseType_t res = xTaskCreate(Ch9120Net_ReconfigTask,
                                 "Ch9120Cfg",
                                 CH9120_NET_RECONFIG_TASK_STACK_SIZE,
                                 NULL,
                                 CH9120_NET_RECONFIG_TASK_PRIORITY,
                                 NULL);
    if (res != pdTRUE)
    {
        s_is_configuring = false;
        WiFiLog_Statusf("[CH9120_NET] create reconfigure task failed\r\n");
        return false;
    }

    WiFiLog_Statusf("[CH9120_NET] reconfigure request accepted\r\n");
    return true;
}

/**
 * @brief 查询 CH9120 当前是否处于配置流程。
 */
bool Ch9120Net_IsConfiguring(void)
{
    return s_is_configuring;
}

/**
 * @brief 周期性记录 CH9120 TCP 客户端连接状态。
 */
void Ch9120Net_Poll(void)
{
    if (s_is_configuring)
    {
        return;
    }

    uint32_t now = millis();
    bool connected = Ch9120Net_IsTcpConnected();
    int8_t state = connected ? 1 : 0;
    bool state_changed = state != s_last_tcpcs_state;
   // bool time_to_log = (uint32_t)(now - s_last_tcpcs_log_ms) >= CH9120_NET_TCPCS_LOG_INTERVAL_MS;

    if (!state_changed )
    {
        return;
    }

    s_last_tcpcs_state = state;
    s_last_tcpcs_log_ms = now;

    WiFiLog_Statusf("[CH9120_NET] TCPCS=%s, TCP %s, mode=TCP_CLIENT, local %u.%u.%u.%u:%u, server %u.%u.%u.%u:%u\r\n",
                    connected ? "LOW" : "HIGH",
                    connected ? "connected" : "disconnected",
                    s_local_ip[0],
                    s_local_ip[1],
                    s_local_ip[2],
                    s_local_ip[3],
                    s_local_port,
                    s_server_ip[0],
                    s_server_ip[1],
                    s_server_ip[2],
                    s_server_ip[3],
                    s_server_port);
}

/**
 * @brief 查询 CH9120 TCP 客户端连接状态。
 */
bool Ch9120Net_IsTcpConnected(void)
{
    return pca.digitalRead(CH9120_TCPCS_PIN) == LOW;
}

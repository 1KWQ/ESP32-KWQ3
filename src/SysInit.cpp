/**
 * @file SysInit.cpp
 * @author your name (you@domain.com)
 * @brief 系统初始化流程（ESP32-KWQ3）
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @details 由上位机 `SET POWER ON` 触发的"结构上电后一次性初始化"后台任务：
 *          上电稳定等待 → 前置检查（心跳/开关门/样品盘/温度）→ 六通阀（暂跳过）
 *          → 启动整机回零（EmmSystemHome 的 12 步序列）。
 *
 *          设计要点：
 *            - 全程**非阻塞**：命令分发只在 SysInit_Run() 里置位并唤醒任务，
 *              真正的等待与检查在后台任务中完成，不会拖慢串口命令轮询；
 *            - 前置检查全部复用 exio 的现成接口，本文件不直接访问硬件，
 *              也不产生任何 RS485 总线事务；
 *            - 回零复用 EmmSystemHome 的 L2 序列，本文件只负责"启动"，
 *              回零过程与结果由 EmmSystemHome 任务自行打印；
 *            - 结果**仅通过串口日志**体现，不向上位机发送额外协议帧。
 */

#include "common.h"
#include "SysInit.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SYSINIT";

/* ==========================================================================
 * 模块配置
 * ========================================================================== */

/**
 * @brief 上电稳定等待时间（ms）。
 * @details `SET POWER ON` 合上结构电源继电器后，EMM 驱动器需要一定时间完成
 *          上电自检；过早下发回零命令会因驱动器未就绪而失败。这里等待 1.5s，
 *          与 KWQ2 上电流程的等待量级保持一致。
 */
#define SYS_INIT_POWER_SETTLE_MS       1500U

/** @brief 初始化任务优先级：低于整机回零（3）与安全监控，高于主循环（1）。 */
#define SYS_INIT_TASK_PRIORITY         2
/** @brief 初始化任务固定核心。 */
#define SYS_INIT_TASK_CORE             1
/** @brief 初始化任务栈大小（字节）。调用嵌套较浅，4096 足够。 */
#define SYS_INIT_TASK_STACK            4096U

/** @brief 前置检查失败时的三色灯闪烁周期（ms）。 */
#define SYS_INIT_FAIL_BLINK_MS         500U

/* ==========================================================================
 * 模块内部状态
 * ========================================================================== */

/** 初始化后台任务句柄；非空表示任务已创建。 */
static TaskHandle_t s_sysInitTaskHandle = nullptr;

/**
 * @brief 流程占用标志。
 * @details 由 SysInit_Run() 在"唤醒任务之前"同步置位，因此可以挡住两次快速
 *          连续下发；后台任务在流程结束后清零。
 */
static volatile bool s_sysInitBusy = false;

/** 最近一次初始化流程的结果状态。 */
static volatile SysInitState s_sysInitState = SYS_INIT_IDLE;

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
 * @brief 前置检查失败时的统一处理：打印原因 + 三色灯黄色闪烁 + 置失败状态。
 * @param reason 失败原因（中文，直接打印）。
 */
static void sysInitFail(const char *reason)
{
    Serial.printf("[初始化] 前置检查未通过：%s，初始化中止\n", reason);
    ESP_LOGE(TAG, "pre-check failed: %s", reason);

    /* 按需求：任一前置检查失败 → 统一黄色闪烁。 */
    rygLedBlink(RYG_COLOR_YELLOW, SYS_INIT_FAIL_BLINK_MS);

    s_sysInitState = SYS_INIT_FAILED;
}

/**
 * @brief 逐项执行 4 个前置检查。
 * @return true 全部通过；false 已由 sysInitFail() 处理失败原因。
 */
static bool sysInitPreChecks(void)
{
    /* ① PC 心跳在线：ping_on_line 由 PING 命令置位、超时清零（见 checkPCAlive）。 */
    if (!ping_on_line) {
        sysInitFail("上位机心跳不在线");
        return false;
    }
    Serial.println("[初始化] 前置检查 1/4 心跳在线：通过");

    /* ② 门已关闭：checkDoorState() 返回 true 表示门到位。 */
    if (!checkDoorState()) {
        sysInitFail("舱门未关闭");
        return false;
    }
    Serial.println("[初始化] 前置检查 2/4 舱门关闭：通过");

    /* ③ 样品盘已放置：checkSampleState() 返回 true 表示样品盘到位。 */
    if (!checkSampleState()) {
        sysInitFail("样品盘未放置");
        return false;
    }
    Serial.println("[初始化] 前置检查 3/4 样品盘到位：通过");

    /*
     * ④ 控制板温度未超限：取 5 路驱动芯片温度的最大值，与 SET TEMP 设置的
     *    阈值 max_temp 比较。上电初期 driverTemp[] 尚未刷新时全为 0，
     *    该检查会自然通过，不会误报。
     */
    const float maxDriverTemp = getMaxDriverTemp();
    if (maxDriverTemp > max_temp) {
        char reason[64];
        snprintf(reason, sizeof(reason), "控制板温度超限(%.1f>%.1f)",
                 (double)maxDriverTemp, (double)max_temp);
        sysInitFail(reason);
        return false;
    }
    Serial.printf("[初始化] 前置检查 4/4 控制板温度：通过(最高 %.1f℃ <= %.1f℃)\n",
                  (double)maxDriverTemp, (double)max_temp);

    return true;
}

/**
 * @brief 初始化后台任务主体：默认阻塞等待启动通知，被唤醒后执行完整流程。
 * @param argument 未使用。
 */
static void sysInitTask(void *argument)
{
    (void)argument;

    for (;;) {
        /* 阻塞等待 SysInit_Run() 的启动通知，不占用 CPU。 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        s_sysInitState = SYS_INIT_RUNNING;

        Serial.println("[初始化] ================= 系统初始化开始 =================");

        /* 0. 等待后级电源与 EMM 驱动器上电稳定。 */
        Serial.printf("[初始化] 等待结构电源稳定 %u ms ...\n",
                      (unsigned)SYS_INIT_POWER_SETTLE_MS);
        vTaskDelay(pdMS_TO_TICKS(SYS_INIT_POWER_SETTLE_MS));

        /* 1. 前置检查：任一项失败即中止。 */
        if (!sysInitPreChecks()) {
            Serial.println("[初始化] ================= 系统初始化中止 =================");
            s_sysInitBusy = false;
            continue;
        }

        /*
         * 2. 六通阀：KWQ2 中无对应实现，当前跳过并视为通过。
         *    待后续接入 SV04M 实际切阀动作时，在此处替换为真实检查。
         */
        Serial.println("[初始化] 六通阀检查：暂未实现，跳过并视为通过");

        /* 3. 启动整机回零（异步，不等待结束）。 */
        if (EmmSystemHome_IsActive()) {
            Serial.println("[初始化] 整机回零已在运行，跳过重复启动");
        } else if (EmmSystemHome_Start()) {
            Serial.println("[初始化] 已启动整机回零，后续进度见 [回零] 日志");
        } else {
            /* 启动被拒绝：视为初始化失败，同样以黄灯闪烁提示。 */
            sysInitFail("整机回零启动失败");
            Serial.println("[初始化] ================= 系统初始化中止 =================");
            s_sysInitBusy = false;
            continue;
        }

        s_sysInitState = SYS_INIT_PASSED;
        Serial.println("[初始化] ================= 系统初始化完成 =================");

        s_sysInitBusy = false;
    }
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

bool SysInit_Init(void)
{
    if (s_sysInitTaskHandle != nullptr) {
        return true;    /* 任务已创建，重复调用直接成功。 */
    }

    const BaseType_t result = xTaskCreatePinnedToCore(
        sysInitTask,
        "sysInitTask",
        SYS_INIT_TASK_STACK,
        nullptr,
        SYS_INIT_TASK_PRIORITY,
        &s_sysInitTaskHandle,
        SYS_INIT_TASK_CORE);

    if (result != pdPASS) {
        s_sysInitTaskHandle = nullptr;
        Serial.println("[初始化] 系统初始化任务启动失败：无法创建后台任务");
        return false;
    }

    Serial.printf("[初始化] 系统初始化任务已启动：优先级=%d 核心=%d\n",
                  (int)SYS_INIT_TASK_PRIORITY, (int)SYS_INIT_TASK_CORE);
    return true;
}

bool SysInit_Run(void)
{
    if (!SysInit_Init()) {
        return false;
    }
    if (s_sysInitBusy) {
        Serial.println("[初始化] 已拒绝启动：系统初始化正在运行");
        return false;
    }

    /*
     * 先同步置位占用标志，再唤醒任务。这样两次快速连续下发时，第二次会在
     * 这里被挡下，而不会出现"任务还没开始跑就被重复启动"的竞态。
     */
    s_sysInitBusy = true;
    s_sysInitState = SYS_INIT_RUNNING;
    xTaskNotifyGive(s_sysInitTaskHandle);
    return true;
}

bool SysInit_IsActive(void)
{
    return s_sysInitBusy;
}

SysInitState SysInit_GetState(void)
{
    return s_sysInitState;
}

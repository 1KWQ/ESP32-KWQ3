/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief ESP32-KWQ3 主程序：仅 EMM 仪器的独立固件。
 *
 * @details 本工程在 ESP32-KWQ2 的基础上完成"提纯"：只保留 EMM V5 闭环步进
 *          电机仪器所需的运行环境与外设，彻底移除 XYZ 步进、ZS 注射器、
 *          序列流程、坐标点位与掉电保存等旧代码。上位机协议保持 KWQ1 的
 *          文本命令 + CRC16 约定不变，通信链路仍为 UART(115200)。
 *
 *          保留并初始化的模块：
 *            - 扩展 IO（PCA9555）：三色灯、电磁阀、门/样品盘/水瓶检测、
 *              系统电源控制、5 路驱动温度轮询；
 *            - 三色灯任务、风扇、ADC；
 *            - 共享 RS485 总线（SV04M 阀头 / GFL-Z 激光 / EMM V5 电机）；
 *            - EMM 安全监控：接近开关急停监控、升降电机相电流扎偏监控；
 *            - EMM 整机回零（L2 复合动作，EmmSystemHome）：常驻任务，
 *              由 SET EMM SYSTEM HOME 触发 12 步回零序列；
 *            - 系统初始化流程（SysInit）：常驻任务，由 SET POWER ON 触发，
 *              上电稳定等待 → 前置检查（心跳/开关门/样品盘/温度）→
 *              六通阀（暂跳过）→ 自动启动整机回零。
 *
 *          暂不激活：CH9120 网络透传（源码保留，待后续需要时再调用
 *          Ch9120Net_Init()/Ch9120Net_Poll() 激活）。
 */

#include "common.h"
#include "version.h"

static const char *TAG = "MAIN";

static void printSystemBootInfo() {
    Serial.println();
    Serial.println("[BOOT] ESP32 boot debug info");
    Serial.printf("[BOOT] Chip model: %s\n", ESP.getChipModel());
    Serial.printf("[BOOT] Chip revision: %d\n", ESP.getChipRevision());
    Serial.printf("[BOOT] CPU freq: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("[BOOT] Flash size: %u bytes\n", ESP.getFlashChipSize());
    Serial.printf("[BOOT] PSRAM size: %u bytes\n", ESP.getPsramSize());
    Serial.printf("[BOOT] Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.println();
}

static void printModuleStatus(const char *module, bool ok) {
    Serial.printf("[INIT] %-24s: %s\n", module, ok ? "OK" : "FAIL");
}

static void printConfigSummary() {
    Serial.println("[CFG] Config summary:");
    Serial.printf("[CFG] MAX TEMP: %.2f C\n", max_temp);
    Serial.printf("[CFG] RYG Voltage: %dV\n", ryg_voltage);
    Serial.printf("[CFG] Compile Ver: %s\n", Get_Compile_Ver());
}

// ================== 串口接收缓冲 ==================
#define MAX_LINE_LEN 150
char rx_line[MAX_LINE_LEN];
uint8_t rx_len=0;

CommandDispatcher dispatcher;

/**
 * @brief 检查命令行 CRC
 * @param line 输入行，末尾包含 #CRC
 * @param crcRx 输出 CRC 值
 * @return true 校验通过，false 失败
 */
bool checkCRC(char* line,uint16_t& crcRx){
    char* p=strrchr(line,'#');
    if(!p) return false;
    crcRx=(uint16_t)strtol(p+1,nullptr,16);
    *p='\0';
    uint16_t crcCalc=crc16_modbus((const uint8_t*)line,strlen(line));
    return crcCalc==crcRx;
}

/**
 * @brief 去除字符串首尾空白字符（空格、\\t、\\r、\\n）
 * 
 * @param str 待处理字符串（会被原地修改）
 */
static void trim(char* str)
{
    if (!str) return;
        // 去掉前导空白
        char* start = str;
        while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') {
            start++;
    }

    // 整体左移
    if (start != str) {
        memmove(str, start, strlen(start) + 1);
    }

    // 去掉尾部空白
    char* end = str + strlen(str) - 1;
    while (end >= str &&
          (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n'))
    {
        *end-- = '\0';
    }
}

/**
 * @brief 处理单行命令
 * @param line 输入命令行
 */
void handleLine(char* line){
    uint16_t crcRx;
   if(!checkCRC(line,crcRx)){ // crc16检查
      sendWithCRC("ERR CRC");  // ERR @<SEQ> <ERR_CODE>#<CRC16>\n 101 检验错误
      return; 
    } 
    int seq = -1;

    // 找 @SEQ（从右往左）
    char* at = strrchr(line, '@');
    if (at) {
        seq = atoi(at + 1);
        *at = '\0';   // 截断，去掉 @SEQ
    }

    // 去掉首尾空格
    trim(line);

    // 派发命令
    if (!dispatcher.dispatch(line, seq)) {
       // Serial.printf("ERR @%d CMD\n", seq);
    }
}

/**
 * @brief 轮询串口，逐行读取命令
 */
void serialPoll() {
    while(Serial.available()){
        char c = Serial.read();
        if(c=='\r') continue;
        if(c=='\n'){
            rx_line[rx_len]='\0';
            handleLine(rx_line);
            rx_len=0;
            return;
        }
        if(rx_len<MAX_LINE_LEN-1) rx_line[rx_len++]=c;
        else rx_len=0;
    }
}

/**
 * @brief 串口断链判断（PING 超时 / 长时间无指令进入休眠）。
 * @details KWQ3 仅联动 EMM 电机：超时后广播停止所有 EMM 电机并断开结构电源。
 */
void checkPCAlive()
{
    // -------- PING 超时 -------- // 
    if (ping_on_line && ( (millis() - lastPingTime) > (PING_TIMEOUT *1000)))  
    {
         // 串口断连
        ping_on_line = false;
        rygLedOn(RYG_COLOR_RED); // 红色长亮
        Serial.printf("[PING] 超时，串口断连，PING_TIMEOUT=%d s\n", PING_TIMEOUT);
    }

     // -------- CMD 超时 --------
    if ( cmd_on_line && ( (millis() - lasCmdTime) > (CMD_TIMEOUT*60*1000) ))  
    {
         // 尝试时间没有收到指令消息，进入休眠
        cmd_on_line = false;
        rygLedOn(RYG_COLOR_YELLOW); // 黄色长亮
        cmd_power_off = true;
        /* 断电前广播停止所有 EMM 电机。 */
        if (Rs485Bus_Take(500)) {
            gEmm.stopBroadcast(false);
            Rs485Bus_Give();
        }
        sysPoweronoFF(0);    // 关电
        Serial.printf("[CMD] 超时，进入休眠，CMD_TIMEOUT=%d min\n", CMD_TIMEOUT);   
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_INFO);
    printSystemBootInfo();

    // PCA 中断输入
    pinMode(46, INPUT_PULLUP);
    printModuleStatus("pinMode(46)", true);

    // 外设初始化（PCA9555 扩展 IO、温度/电磁阀/门/样品盘/系统电源）
    exIOInit();
    printModuleStatus("exIOInit", true);
    delay(500);

    // 三色灯控制任务
    rygLedInitTask();
    printModuleStatus("rygLedInitTask", true);

    // 温度采样 ADC
    adc_init();
    printModuleStatus("adc_init", true);

    // 风扇 PWM
    fanInit();
    printModuleStatus("fanInit", true);

    delay(500);
    rygLedOn(RYG_COLOR_YELLOW); // 开机自检完成，黄色长亮
    printModuleStatus("rygLedOn", true);

    // 共享 RS485 总线（SV04M 阀头 / GFL-Z 激光 / EMM V5 电机）
    Rs485Devices_Init();
    printModuleStatus("Rs485Devices_Init", true);

    /*
     * 启动 GPIO6（Y_ENDSTOP2_PIN）接近开关监控。监控任务会在引脚变为
     * 低电平时，立即向 MONITOR_EMM 地址发送 EMM 停止命令。
     */
    bool emmMonitorOk = EmmEndstopMonitor_Init();
    printModuleStatus("EmmEndstopMonitor", emmMonitorOk);
    if (!emmMonitorOk) {
        /* 监控属于电机安全功能，创建失败时不允许系统继续进入正常运行状态。 */
        ESP_LOGE(TAG, "EMM endstop monitor task create failed");
        rygLedBlink(RYG_COLOR_RED, 1000);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    /*
     * 启动 3 号升降电机的相电流轮询监控（电离头扎偏保护）。该任务依赖
     * Rs485Devices_Init() 建立的共享总线，因此必须在其之后创建。属于新增
     * 安全功能，创建失败只记录日志，不阻断系统进入正常运行。
     */
    bool liftCurrentMonitorOk = EmmLiftCurrentMonitor_Init();
    printModuleStatus("EmmLiftCurrentMonitor", liftCurrentMonitorOk);
    if (!liftCurrentMonitorOk) {
        ESP_LOGE(TAG, "EMM lift current monitor task create failed");
    }

    /*
     * 创建整机回零（L2 复合动作）常驻任务。该任务默认阻塞等待
     * SET EMM SYSTEM HOME 通知，收到命令后才驱动 12 步回零序列；依赖
     * Rs485Devices_Init() 建立的共享总线，因此必须在其之后创建。
     */
    bool systemHomeOk = EmmSystemHome_Init();
    printModuleStatus("EmmSystemHome", systemHomeOk);
    if (!systemHomeOk) {
        ESP_LOGE(TAG, "EMM system home task create failed");
    }

    /*
     * 创建系统初始化流程（SysInit）常驻任务。该任务默认阻塞等待通知，收到
     * `SET POWER ON` 后由 cmdPower 唤醒，执行：上电稳定等待 → 前置检查
     * （心跳/开关门/样品盘/温度）→ 六通阀（暂跳过）→ 启动整机回零。
     * 它依赖 EmmSystemHome_Init() 建立的回零任务，因此必须在其之后创建。
     */
    bool sysInitOk = SysInit_Init();
    printModuleStatus("SysInit", sysInitOk);
    if (!sysInitOk) {
        ESP_LOGE(TAG, "SysInit task create failed");
    }

    printConfigSummary();
    ESP_LOGI(TAG, "System start");
    Serial.println("[BOOT] System start");
}

void loop()
{
    // 串口命令轮询（KWQ3 未启用 CH9120 透传，直接轮询默认 Serial）
    serialPoll();

    /*
     * 仅在已建立上位机连接（收到过 PING 或指令）时进行超时检测；
     * 未连接时持续刷新时间基准，避免刚上电就误判超时。
     */
    if (ping_on_line || cmd_on_line) {
        checkPCAlive();  // 休眠/ ping 超时检测
    }
    else {
        lastPingTime = millis();
        lasCmdTime = millis();
    }
}

#ifndef EMM_LIFT_SWITCH_TEST_H
#define EMM_LIFT_SWITCH_TEST_H

#include <Arduino.h>

/**
 * @file EmmLiftSwitchTest.h
 * @brief 3 号升降电机"接近开关触发即停"的**试验性**命令（2026/10/27 新增）。
 *
 * @details 独立模块，**不改动任何现有流程**：整机回零仍然是 17 步的碰撞回零，
 *          接近开关监控仍然只有样品盘（GPIO6）那一路，3 号零点、`sys_is_homed`、
 *          5 个点位全部不受影响。
 *
 *          这条命令只做一件事 —— 验证"用接近开关让 3 号停下来"是否可行：
 *            ⓪ 若开关此刻已触发，先下降退出触发区（防漂移）；
 *            ① 速度模式向上（`DIR=0`）起转；
 *            ② 轮询 GPIO2，**触发立即发停止**；
 *            ③ 等停稳；
 *            ④ 打印触发点读数（相对当前零点，供日后判断零点会挪多少）。
 *
 *          ⚠️ **本路开关是「高电平 = 触发」**（2026/10/27 用户确认，第一版按样品盘
 *          那一路的低电平触发写反了）。引脚配 `INPUT_PULLDOWN`：断线/悬空读低电平
 *          = 未触发，是最安全的失效方向。与样品盘（`Y_ENDSTOP2_PIN` = GPIO6，
 *          低电平触发）**极性相反**，改代码时别照抄那一份。
 *
 *          ⚠️ **它不设零点**。这是用户 2026/10/27 的明确要求（"只测试触发停止的
 *          效果"）。所以跑完这条命令后，3 号的零点仍然是上次整机回零建立的那个，
 *          5 个点位不需要重标。想正式启用接近开关回零，再另开方案。
 *
 * @note **2026/10/27 下午改为异步执行**。原来这条命令是同步阻塞的（照抄
 *       `SET EMM TRAY HOME`），结果跑起来最长 30 s 占住 `loop()` → `serialPoll()`
 *       → `dispatch()` 整条链路，**`SET POWER OFF` 在这段时间根本读不进来**，
 *       电机停不下来。现在改成：
 *         - 命令只做校验 + 置 busy + 唤醒后台任务，**立刻**回
 *           `OK @<seq> EMM LIFT TEST STARTED`；
 *         - 真正的动作在独立任务里跑，轮询循环每 10 ms 检查一次 `cmd_power_off`，
 *           一旦断电立刻发停止并退出；
 *         - 结局用事件帧上报（`EVT 390 LIFT TEST DONE` / `EVT 392 LIFT TEST STOPPED`
 *           / `ERR @<seq> 117`）。
 *       任务在**第一次下发命令时惰性创建**（`xTaskCreatePinnedToCore`），
 *       不需要改 `main.cpp` 的启动序列。
 *
 * @note 与其它 `SET EMM *` 一样，整机回零 / 任务序列运行期间会被
 *       `EmmSystemHome_BlocksCommand()` / `EmmTaskSeq_BlocksCommand()` 自动拦掉
 *       （它们对"以 `SET EMM` 开头且不在白名单里的命令"一律返回 `SEQ_RUNNING`）。
 *       反过来，本命令**运行期间**也会拦住其它 `SET EMM *`（见
 *       `EmmLiftSwitchTest_BlocksCommand()`），避免有人在中途插一条
 *       `SET EMM MOVE A=3` 跟测试抢电机。
 *       `SET POWER OFF` **不在拦截范围**（它不是 `SET EMM` 命令），这正是需求要的。
 */

/**
 * @brief 上位机命令 `SET EMM LIFT TEST`：3 号上升直到接近开关触发并立即停止。
 *
 * @details 命令格式与应答（**异步**）：
 *            - 已接受                 → `OK @<seq> EMM LIFT TEST STARTED`（立即）
 *            - 结构电源已关闭         → `ERR @<seq> 109`（`NO_STEP_PWR`）
 *            - 本命令或整机回零 / 任务序列正在跑 → `ERR @<seq> 110`（`SEQ_RUNNING`）
 *            - 任务创建失败           → `ERR @<seq> 117`
 *
 *          接受之后的结局帧（异步发出，**不带 `@seq`**，与其它 `EVT` 一致）：
 *            - `EVT 390 LIFT TEST DONE`    触发成功、3 号已停
 *            - `EVT 392 LIFT TEST STOPPED` 被 `SET POWER OFF` / 硬件急停打断
 *            - `ERR @<seq> 117`            上升 30 s 未触发 / 8 s 退不出触发区 /
 *                                          速度模式被驱动器拒绝
 *
 * @param args 忽略（本命令无参数；带了也会被忽略，与 `SET EMM TRAY HOME` 一致）。
 * @param seq  命令序号，用于应答回显（异步失败帧也用它回显）。
 * @return true 表示已接受并回了 `STARTED`；false 表示回了 `ERR`。
 */
bool cmdSetEmmLiftTest(const char *args, int seq);

/**
 * @brief 查询试验命令是否正在运行。
 * @return true 表示后台任务正在跑（3 号可能正在动）。
 */
bool EmmLiftSwitchTest_IsActive(void);

/**
 * @brief 判断某条命令在试验运行期间是否应被拒绝。
 * @details 与 `EmmSystemHome_BlocksCommand()` 同构：试验期间独占 3 号电机，
 *          因此拦截所有 `SET EMM *`（含 `SET EMM STOP` —— 要紧急停机请用
 *          `SET POWER OFF`，它会走 `sysPowerOffCleanup()` 广播停机 + 断结构电源，
 *          同时本模块的任务也会在 10 ms 内看到 `cmd_power_off` 并退出）。
 *          `GET EMM *` 查询命令放行，便于现场诊断。
 * @param commandName 已由 `CommandDispatcher` 匹配出的完整命令名。
 * @return true 表示应当拒绝。
 */
bool EmmLiftSwitchTest_BlocksCommand(const char *commandName);

#endif

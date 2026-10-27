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
 *          ⚠️ **它不设零点**。这是用户 2026/10/27 的明确要求（"只测试触发停止的
 *          效果"）。所以跑完这条命令后，3 号的零点仍然是上次整机回零建立的那个，
 *          5 个点位不需要重标。想正式启用接近开关回零，再另开方案。
 *
 * @note 与 `SET EMM TRAY HOME` 一样是**同步阻塞**：跑完才回一条应答，最长约
 *       30 s（3 号满行程 37000 脉冲 @40 RPM ≈ 17.3 s）。上位机 `CMD_TIMEOUT`
 *       是 30 分钟，不会因这条命令进入休眠。
 *
 * @note 与其它 `SET EMM *` 一样，整机回零 / 任务序列运行期间会被
 *       `EmmSystemHome_BlocksCommand()` / `EmmTaskSeq_BlocksCommand()` 自动拦掉
 *       （它们对"以 `SET EMM` 开头且不在白名单里的命令"一律返回 `SEQ_RUNNING`），
 *       本模块不需要再写一遍互锁。
 */

/**
 * @brief 上位机命令 `SET EMM LIFT TEST`：3 号上升直到接近开关触发并立即停止。
 *
 * @details 命令格式与应答：
 *            - `SET EMM LIFT TEST`            → `OK @<seq> EMM LIFT TEST DONE`
 *            - 结构电源已关闭                 → `ERR @<seq> 109`（`NO_STEP_PWR`）
 *            - 整机回零 / 任务序列运行中      → `ERR @<seq> 110`（`SEQ_RUNNING`，
 *                                               由 `dispatch()` 统一拦截）
 *            - 上升 30 s 未触发（开关坏/没接）→ `ERR @<seq> 117`（`EMM_HOME_FAILED`）
 *            - 已在触发区且 8 s 内退不出来    → `ERR @<seq> 117`
 *            - 速度模式命令被驱动器拒绝       → `ERR @<seq> 117`
 *
 * @param args 忽略（本命令无参数；带了也会被忽略，与 `SET EMM TRAY HOME` 一致）。
 * @param seq  命令序号，用于应答回显。
 * @return true 表示已经回过应答（成功）；false 表示回过 `ERR`。
 */
bool cmdSetEmmLiftTest(const char *args, int seq);

#endif

/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_ch9120_basic.c
 * @brief     driver ch9120 basic source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2025-09-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2025/09/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_ch9120_basic.h"

static ch9120_handle_t gs_handle;        /**< ch9120 handle */

/**
 * @brief  basic example init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t ch9120_basic_init(void)
{
    uint8_t res;
    
    /* link interface function */
    DRIVER_CH9120_LINK_INIT(&gs_handle, ch9120_handle_t);
    DRIVER_CH9120_LINK_UART_INIT(&gs_handle, ch9120_interface_uart_init);
    DRIVER_CH9120_LINK_UART_DEINIT(&gs_handle, ch9120_interface_uart_deinit);
    DRIVER_CH9120_LINK_UART_READ(&gs_handle, ch9120_interface_uart_read);
    DRIVER_CH9120_LINK_UART_WRITE(&gs_handle, ch9120_interface_uart_write);
    DRIVER_CH9120_LINK_UART_FLUSH(&gs_handle, ch9120_interface_uart_flush);
    DRIVER_CH9120_LINK_RESET_GPIO_INIT(&gs_handle, ch9120_interface_reset_gpio_init);
    DRIVER_CH9120_LINK_RESET_GPIO_DEINIT(&gs_handle, ch9120_interface_reset_gpio_deinit);
    DRIVER_CH9120_LINK_RESET_GPIO_WRITE(&gs_handle, ch9120_interface_reset_gpio_write);
    DRIVER_CH9120_LINK_CFG_GPIO_INIT(&gs_handle, ch9120_interface_cfg_gpio_init);
    DRIVER_CH9120_LINK_CFG_GPIO_DEINIT(&gs_handle, ch9120_interface_cfg_gpio_deinit);
    DRIVER_CH9120_LINK_CFG_GPIO_WRITE(&gs_handle, ch9120_interface_cfg_gpio_write);
    DRIVER_CH9120_LINK_DELAY_MS(&gs_handle, ch9120_interface_delay_ms);
    DRIVER_CH9120_LINK_DEBUG_PRINT(&gs_handle, ch9120_interface_debug_print);
    
    /* ch9120 init */
    res = ch9120_init(&gs_handle);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: init failed.\n");

        return 1;
    }
    
    return 0;
}

/**
 * @brief 保存 CH9120 当前配置并让配置生效复位。
 *
 * @return 状态码。
 *         - 0 成功。
 *         - 1 保存 EEPROM 或配置复位失败。
 *
 * @details
 * `save to eeprom` 和 `config and reset` 必须在 CH9120 当前仍能响应的配置波特率下
 * 发送。上层会先用当前配置波特率尝试一次；如果 CH9120 在设置 UART baud 后已经
 * 立即切换到目标波特率，再由上层切换 ESP32 配置串口后重试本函数。
 */
static uint8_t ch9120_basic_save_and_reset(void)
{
    uint8_t res;

    /* save to eeprom */
    res = ch9120_save_to_eeprom(&gs_handle);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: save to eeprom failed.\n");

        return 1;
    }

    /* config and reset */
    res = ch9120_config_and_reset(&gs_handle);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: config and reset failed.\n");

        return 1;
    }

    return 0;
}

/**
 * @brief     basic example config
 * @param[in] mode chip mode
 * @param[in] *ip pointer to an ip buffer
 * @param[in] port local port
 * @param[in] *mask pointer to a mask buffer
 * @param[in] *gateway pointer to a gateway buffer
 * @param[in] *dest_ip pointer to a dest ip buffer
 * @param[in] dest_port dest port
 * @return    status code
 *            - 0 success
 *            - 1 config failed
 * @note      none
 */
uint8_t ch9120_basic_config(ch9120_mode_t mode,
                            uint8_t ip[4], uint16_t port, 
                            uint8_t mask[4], uint8_t gateway[4],
                            uint8_t dest_ip[4], uint16_t dest_port)
{
    uint8_t res;
    uint8_t reg;
    
    /* set default dhcp */
    res = ch9120_set_dhcp(&gs_handle, CH9120_BASIC_DEFAULT_DHCP);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set dhcp failed.\n");
        
        return 1;
    }
    
    /* set local ip */
    res = ch9120_set_ip(&gs_handle, ip);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set ip failed.\n");
        
        return 1;
    }
    
    /* set subnet mask */
    res = ch9120_set_subnet_mask(&gs_handle, mask);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set subnet mask failed.\n");
        
        return 1;
    }
    
    /* set gateway */
    res = ch9120_set_gateway(&gs_handle, gateway);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set gateway failed.\n");
        
        return 1;
    }

    /* set source port */
    res = ch9120_set_source_port(&gs_handle, port);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set source port failed.\n");
        
        return 1;
    }
    
    /* set dest ip */
    res = ch9120_set_dest_ip(&gs_handle, dest_ip);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set dest ip failed.\n");
        
        return 1;
    }
    
    /* set dest port */
    res = ch9120_set_dest_port(&gs_handle, dest_port);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set dest port failed.\n");
        
        return 1;
    }
    
    /* set default uart config */
    res = ch9120_set_uart_config(&gs_handle, CH9120_BASIC_DEFAULT_UART_DATA_BIT,
                                 CH9120_BASIC_DEFAULT_UART_PARITY, CH9120_BASIC_DEFAULT_UART_STOP_BIT);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set uart config failed.\n");
        
        return 1;
    }
    
    /* set default uart timeout */
    res = ch9120_uart_timeout_convert_to_register(&gs_handle, CH9120_BASIC_DEFAULT_UART_TIMEOUT, &reg);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: uart timeout convert to register failed.\n");
        
        return 1;
    }
    
    /* set uart timeout */
    res = ch9120_set_uart_timeout(&gs_handle, reg);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set uart timeout failed.\n");
        
        return 1;
    }
    
    /* set default source port random */
    res = ch9120_set_source_port_random(&gs_handle, CH9120_BASIC_DEFAULT_UART_SOURCE_PORT_RANDOM);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set source port random failed.\n");
        
        return 1;
    }
    
    /* set default uart buffer length */
    res = ch9120_set_uart_buffer_length(&gs_handle, CH9120_BASIC_DEFAULT_UART_BUFFER_LENGTH);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set uart buffer length failed.\n");
        
        return 1;
    }
    
    /* set default uart flush */
    res = ch9120_set_uart_flush(&gs_handle, CH9120_BASIC_DEFAULT_UART_FLUSH);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set uart flush failed.\n");
        
        return 1;
    }
    
    /* set default disconnect with no rj45 */
    res = ch9120_set_disconnect_with_no_rj45(&gs_handle, CH9120_BASIC_DEFAULT_DISCONNECT_WITH_NO_RJ45);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set disconnect with no rj45 failed.\n");
        
        return 1;
    }
    
    /* set mode */
    res = ch9120_set_mode(&gs_handle, mode);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set mode failed.\n");
        
        return 1;
    }

    /**
     * @brief 最后写入 CH9120 工作波特率。
     *
     * @details
     * CH9120 在收到 `set uart baud` 后，不同固件可能有两种表现：有的仍用当前配置波特率
     * 等待 `save to eeprom` 和 `config and reset`，有的会立即切到新的工作波特率。
     * 因此这里先不立刻切换 ESP32 串口，而是先用当前波特率保存；若保存超时，再切到
     * 目标 115200 重试保存和复位，从而兼容两种时序。
     */
    res = ch9120_set_uart_baud(&gs_handle, CH9120_BASIC_DEFAULT_UART_PORT);
    if (res != 0)
    {
        ch9120_interface_debug_print("ch9120: set uart baud failed.\n");

        return 1;
    }

    if (ch9120_basic_save_and_reset() == 0)
    {
        return 0;
    }

    /**
     * @brief 当前波特率保存失败后，切到目标工作波特率再重试。
     *
     * @details
     * 如果上面的保存失败是因为 CH9120 已经立即切到 115200，这次重开 ESP32 默认 Serial
     * 后应能继续收到应答；如果仍失败，日志会明确显示两种波特率路径都没有成功。
     */
    ch9120_interface_debug_print("ch9120: retry save/reset at target uart baud.\n");
    ch9120_interface_set_uart_baud(CH9120_BASIC_DEFAULT_UART_PORT);
    if (ch9120_interface_uart_deinit() != 0)
    {
        ch9120_interface_debug_print("ch9120: switch uart baud deinit failed.\n");

        return 1;
    }
    if (ch9120_interface_uart_init() != 0)
    {
        ch9120_interface_debug_print("ch9120: switch uart baud init failed.\n");

        return 1;
    }

    if (ch9120_basic_save_and_reset() != 0)
    {
        ch9120_interface_debug_print("ch9120: save/reset failed after uart baud switch.\n");

        return 1;
    }

    return 0;
}

/**
 * @brief         basic example read data
 * @param[out]    *buf pointer to a data buffer
 * @param[in,out] *len pointer to a length buffer
 * @return        status code
 *                - 0 success
 *                - 1 read failed
 * @note          none
 */
uint8_t ch9120_basic_read(uint8_t *buf, uint16_t *len)
{
    if (ch9120_read(&gs_handle, buf, len) != 0)
    {
        return 1;
    }
    
    return 0;
}

/**
 * @brief     basic example write data
 * @param[in] *buf pointer to a data buffer
 * @param[in] len data length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t ch9120_basic_write(uint8_t *buf, uint16_t len)
{
    if (ch9120_write(&gs_handle, buf, len) != 0)
    {
        return 1;
    }
    
    return 0;
}

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t ch9120_basic_deinit(void)
{
    /* deinit ch9120 */
    if (ch9120_deinit(&gs_handle) != 0)
    {
        return 1;
    }
    
    return 0;
}

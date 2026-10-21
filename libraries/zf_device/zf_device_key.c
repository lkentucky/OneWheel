/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_device_key
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
* 2023-04-28       pudding            增加中文注释说明
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   // 一般是主板按键对应的引脚
*                   KEY1/S1             查看 zf_device_key.h 中 KEY_LIST[0]
*                   KEY2/S2             查看 zf_device_key.h 中 KEY_LIST[1]
*                   KEY3/S3             查看 zf_device_key.h 中 KEY_LIST[2]
*                   KEY4/S4             查看 zf_device_key.h 中 KEY_LIST[3]
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_device_key.h"
#include "zf_common_interrupt.h"

static uint32 scanner_period;
static uint32 debounce_ms[KEY_NUMBER], held_ms[KEY_NUMBER];
static uint8 stable_pressed[KEY_NUMBER], long_reported[KEY_NUMBER];
/* ISR 只增加计数，主循环在短临界区内消费，避免事件被下一次扫描覆盖。 */
static volatile uint8 short_pending[KEY_NUMBER], long_pending[KEY_NUMBER];
static const gpio_pin_enum key_index[KEY_NUMBER] = KEY_LIST;

void key_scanner(void)
{
    uint8 i;
    if (!scanner_period) return;
    for (i = 0; i < KEY_NUMBER; ++i) {
        uint8 pressed = gpio_get_level(key_index[i]) != KEY_RELEASE_LEVEL;
        if (pressed != stable_pressed[i]) {
            debounce_ms[i] += scanner_period;
            if (debounce_ms[i] >= KEY_MAX_SHOCK_PERIOD) {
                stable_pressed[i] = pressed;
                debounce_ms[i] = held_ms[i] = 0;
                long_reported[i] = 0;
                /* 按下稳定后立即响应一次，释放同样消抖，长按不重复导航。 */
                if (pressed && short_pending[i] < 255) ++short_pending[i];
            }
        } else {
            debounce_ms[i] = 0;
            if (pressed && !long_reported[i]) {
                held_ms[i] += scanner_period;
                if (held_ms[i] >= KEY_LONG_PRESS_PERIOD) {
                    long_reported[i] = 1;
                    if (long_pending[i] < 255) ++long_pending[i];
                }
            }
        }
    }
}

key_state_enum key_get_state(key_index_enum key_n)
{
    zf_assert(key_n < KEY_NUMBER);
    if (short_pending[key_n]) return KEY_SHORT_PRESS;
    if (long_pending[key_n]) return KEY_LONG_PRESS;
    return KEY_RELEASE;
}

void key_clear_state(key_index_enum key_n)
{
    uint32 mask;
    zf_assert(key_n < KEY_NUMBER);
    mask = interrupt_global_disable();
    if (short_pending[key_n]) --short_pending[key_n];
    else if (long_pending[key_n]) --long_pending[key_n];
    interrupt_global_enable(mask);
}

void key_consume_state(key_index_enum key_n, key_state_enum event)
{
    uint32 mask;
    zf_assert(key_n < KEY_NUMBER);
    mask = interrupt_global_disable();
    if (event == KEY_SHORT_PRESS && short_pending[key_n]) --short_pending[key_n];
    if (event == KEY_LONG_PRESS && long_pending[key_n]) --long_pending[key_n];
    interrupt_global_enable(mask);
}

void key_clear_all_state(void)
{
    uint8 i;
    uint32 mask = interrupt_global_disable();
    for (i = 0; i < KEY_NUMBER; ++i) short_pending[i] = long_pending[i] = 0;
    interrupt_global_enable(mask);
}

void key_init(uint32 period)
{
    uint8 i;
    zf_assert(period > 0 && period <= KEY_MAX_SHOCK_PERIOD);
    scanner_period = 0;
    for (i = 0; i < KEY_NUMBER; ++i) {
        gpio_init(key_index[i], GPI, GPIO_HIGH, GPI_PULL_UP);
        debounce_ms[i] = held_ms[i] = 0;
        stable_pressed[i] = long_reported[i] = 0;
        short_pending[i] = long_pending[i] = 0;
    }
    scanner_period = period;
}

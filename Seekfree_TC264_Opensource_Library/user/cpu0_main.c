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
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ***********************mt9v03x_image*****

/*img_threshold为阈值
 * camera.c文件为二值化、红外检测，误差输出
 * posture_control.c中dynamic_value_control函数中设置固定PWM用来转弯
 */
uint8 data_buffer[32];
uint8 data_len;
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
//    wireless_uart_init();          // 初始化正常 输出测试信息
//    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
//    #if(0 == INCLUDE_BOUNDARY_TYPE)
//        // 发送总钻风图像信息(仅包含原始图像信息)
//    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, base_image[0], MT9V03X_W, MT9V03X_H);
//    #endif
    // 此处编写用户代码 例如外设初始化代码等
    // core1_main 里没有调用 cpu_wait_event_ready()，如果在这里等待会永久阻塞
    // 因此 CPU0 不再等待，把空主循环交给 WiFi 链路状态机（SPI 事务全部在 CPU0）
    wifi_link_init();
    while (TRUE)
    {
        wifi_link_service();
//        bujin_motor_contron(4000);

//       printf("%d,%f\r\n",encoder_data_1,balance_cascade.pitch_speed_cycle.out);
//        printf("%d,%d,%f,%d,%d,%f\r\n",icm_data_lpf.gyroZ,icm_data_lpf.accY,balance_cascade.pitch_cascade_value.filtering_angle,icm_data_lpf.gyroY,icm_data_lpf.accZ,balance_cascade.cascade_value.filtering_angle);
//        seekfree_assistant_camera_send();
//       system_delay_ms(10);

        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************

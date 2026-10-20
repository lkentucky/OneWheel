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
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// Same SPI interface as Seekfree E8_14_mt9v03x_ips200_display_demo.
#define ONEWHEEL_IPS200_INTERFACE    (IPS200_TYPE_SPI)
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// **************************** 代码区域 ****************************

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
    ips200_init(ONEWHEEL_IPS200_INTERFACE);
    mt9v03x_init();

    gpio_init(P21_4, GPO, 0, GPO_PUSH_PULL);
    key_init(10);
    imu660rb_init();
    Motor_init();
    balance_cascade_init();
    small_driver_uart_init();       // 初始化驱动通讯功能
    icm_init();
    wireless_uart_init();          // 初始化正常 输出测试信息

    pit_ms_init(CCU60_CH0, 1);
    // 此处编写用户代码 例如外设初始化代码等
//    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
//        bujin_motor_contron(1000);
//        small_driver_set_duty(2000,-2000);
        key_scanner ();
       if(key_get_state(KEY_1)==KEY_SHORT_PRESS  || key_get_state(KEY_2)== KEY_SHORT_PRESS || key_get_state(KEY_3)== KEY_SHORT_PRESS || key_get_state(KEY_4)== KEY_SHORT_PRESS)
       {

           run_flag=0;

       }
        memcpy(base_image[0],mt9v03x_image[0],MT9V03X_IMAGE_SIZE);
        img_threshold=Ostu(base_image);
        if(img_threshold<180)                    //固定最小阈值
        {
            img_threshold=180;
        }

        Image_Binarization(img_threshold);      //二值化
        detect_beacon(image_binary,&beacon);    //检测信标
        error_back=erro;
        erro_yawan_back=erro_yawan;
//        ips200_show_gray_image(0,0,mt9v03x_image[0],MT9V03X_W,MT9V03X_H,MT9V03X_W/2,MT9V03X_H/2,0);
        ips200_show_gray_image(0,0,image_binary[0],MT9V03X_W,MT9V03X_H,MT9V03X_W/2,MT9V03X_H/2,0);
        ips200_draw_point(beacon.centerX/2,beacon.centerY/2,RGB565_RED);
        ips200_draw_line(beacon.centerX/2,beacon.centerY/2,beacon.centerX/2+5,beacon.centerY/2,RGB565_RED);
        ips200_draw_line(beacon.centerX/2,beacon.centerY/2,beacon.centerX/2,beacon.centerY/2+5,RGB565_RED);
        ips200_draw_line(beacon.centerX/2+5,beacon.centerY/2,beacon.centerX/2+5,beacon.centerY/2+5,RGB565_RED);
        ips200_draw_line(beacon.centerX/2,beacon.centerY/2+5,beacon.centerX/2+5,beacon.centerY/2+5,RGB565_RED);

       ips200_draw_line(beacon.last_centerX/2,beacon.last_centerY/2,beacon.last_centerX/2+5,beacon.last_centerY/2,    RGB565_GREEN);
       ips200_draw_line(beacon.last_centerX/2,beacon.last_centerY/2,beacon.last_centerX/2,beacon.last_centerY/2+5,    RGB565_GREEN);
       ips200_draw_line(beacon.last_centerX/2+5,beacon.last_centerY/2,beacon.last_centerX/2+5,beacon.last_centerY/2+5,    RGB565_GREEN);
       ips200_draw_line(beacon.last_centerX/2,beacon.last_centerY/2+5,beacon.last_centerX/2+5,beacon.last_centerY/2+5,    RGB565_GREEN);

       ips200_show_float(0,60,erro,4,2);
//       printf("%d,%f,%d,%d,%f,%d\r\n",icm_data_lpf.accY,balance_cascade.pitch_cascade_value.filtering_angle,icm_data_lpf.gyroZ,icm_data_lpf.accZ,balance_cascade.cascade_value.filtering_angle,icm_data_lpf.gyroY);
       if (run_flag)
       {
           send_two_floats(1.0,encoder_data_1,0.1);
       }
//       send_two_floats(-100,100);
//       wireless_uart_send_image(&mt9v03x_image[0][0], MT9V03X_IMAGE_SIZE);
       system_delay_ms(10);
    }

}
#pragma section all restore
// **************************** 代码区域 ****************************

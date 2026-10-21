/* key_state_update.c：按键和 EC11 输入处理。创建日期：2025年1月10日；上游作者姓名已损坏。 */
#include "key_state_update.h"

int KEY1_STATUS;
int KEY2_STATUS;
int KEY3_STATUS;
int KEY4_STATUS;
int EC11_STATUS;

// 保存上次按键状态。
int last_KEY1_STATUS;
int last_KEY2_STATUS;
int last_KEY3_STATUS;
int last_KEY4_STATUS;
int last_KEY5_STATUS;
int last_EC11_STATUS;
int last_KEY_Value;

int key1_count = 0;
int key2_count = 0;
int key3_count = 0;
int key4_count = 0;
int key5_count = 0;
int EC11_count = 0;

float stride;
int16 EC11_step;

void stride_update(int key_count)
{
    switch (key_count%4)
    {
        case 0:stride=0.01;break;
        case 1:stride=0.1;break;
        case 2:stride=1;break;
        case 3:stride=10;break;
        default:break;
    }
}

void key_state_update(void)
{
    last_KEY1_STATUS = KEY1_STATUS;
    last_KEY2_STATUS = KEY2_STATUS;
    last_KEY3_STATUS = KEY3_STATUS;
    last_EC11_STATUS = EC11_STATUS;

    KEY1_STATUS = gpio_get_level(QCAR_KEY1);
    KEY2_STATUS = gpio_get_level(QCAR_KEY2);
    KEY3_STATUS = gpio_get_level(QCAR_KEY3);
    EC11_STATUS = gpio_get_level(QCAR_EC11_KEY);

    // 检测有效按键事件。
    if(!KEY1_STATUS && last_KEY1_STATUS)    key1_count ++;
    if(!KEY2_STATUS && last_KEY2_STATUS)    key2_count ++;
    if(!KEY3_STATUS && last_KEY3_STATUS)    key3_count ++;
    if(!EC11_STATUS && last_EC11_STATUS)    EC11_count ++;

    if((last_KEY_Value==3) && (KEY_Value==2))    key2_count ++;
    if((last_KEY_Value==3) && (KEY_Value==1))    key4_count ++;

    last_KEY1_STATUS = KEY1_STATUS;
    last_KEY2_STATUS = KEY2_STATUS;
    last_KEY3_STATUS = KEY3_STATUS;
    last_EC11_STATUS = EC11_STATUS;
    last_KEY_Value = KEY_Value;
}

// 读取 EC11 旋转增量。
void ec11_change_pid(void)
{
    encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);// 初始化 EC11 编码器接口。
    EC11_step = -encoder_get_count(TIM5_ENCODER);
    encoder_clear_count(TIM5_ENCODER);
}

void key_ecll_test(void)
{
    int a=0;
    while(1)
    {
        key_state_update();
        ec11_change_pid();
        if(key1_count)
        {
            key1_count=0;
            a+=1;
        }
        if(key2_count)
        {
            key2_count=0;
            a+=2;
        }
        if(key3_count)
        {
            key3_count=0;
            a+=3;
        }
        if(EC11_count)
        {
            EC11_count=0;
            a+=4;
        }
        if(EC11_step!=0)
        {
            a+=EC11_step;
        }
        sprintf(txt,"a=%d",a);
        ips200_show_string(40,40,txt);
    }
}

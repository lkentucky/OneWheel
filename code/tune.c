/* tune.c：参数调节。创建日期：2025年2月27日；作者：yuan。 */
#include "tune.h"
static seekfree_assistant_debug_param_struct qcar_debug_params;
static seekfree_assistant_oscilloscope_struct qcar_scope;
static float qcar_scope_data[8];

float data_Get;
uint8_t DataBuff[512];
uint8_t RxBuffer[1];
uint16_t RxLine = 0;

float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数值字段的起始下标。
    uint8_t data_End_Num = 0; // 记录数值字段的结束下标。
    uint8_t data_Num = 0; // 记录数值字段的字符数量。
    uint8_t minus_Flag = 0; // 记录是否为负数。
    float data_return = 0; // 保存解析后的数值。
    for(uint16_t i=0;i<512;i++) // 查找等号和结束标记的位置。
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // 数值起始位置位于等号的下一字符。
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // 查找参数数值的起始位置。
    {
        data_Start_Num += 1; // 第一位为负号时，将起始位置向后移动一位。
        minus_Flag = 1; // 设置负数标记。
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 解析长度为四个字符的数值字段。
    {
        data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                (DataBuff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 解析长度为五个字符的数值字段。
    {
        data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                (DataBuff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // 解析长度为六个字符的数值字段。
    {
        data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
    return data_return;
}

void USART_PID_Adjust(void)
{
    data_Get = Get_Data(); // 查找参数数值的结束位置。
//    switch(DataBuff[0])
//    {
//        case 'P':
//        switch(DataBuff[1])
//        {
//            case '1':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel.kp = data_Get;break;
//                case '2':cascade_ang.kp = data_Get;break;
//                case '3':cascade_vel.kp = data_Get;break;
//            }break;
//            case '2':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel_p.kp = data_Get;break;
//                case '2':cascade_ang_p.kp = data_Get;break;
//                case '3':cascade_vel_p.kp = data_Get;break;
//            }break;
//            case '3':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel_y.kp = data_Get;break;
//                case '2':cascade_ang_y.kp = data_Get;break;
//                case '3':cascade_vel_y.kp = data_Get;break;
//            }break;
//        }break;
//        case 'I':
//        switch(DataBuff[1])
//        {
//            case '1':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel.ki = data_Get;break;
//                case '2':cascade_ang.ki = data_Get;break;
//                case '3':cascade_vel.ki = data_Get;break;
//            }break;
//            case '2':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel_p.ki = data_Get;break;
//                case '2':cascade_ang_p.ki = data_Get;break;
//                case '3':cascade_vel_p.ki = data_Get;break;
//                //case '4':K_angle_yawan    = data_Get;break;
//                //case '5':K_angle_yawan_cam= data_Get;break;
//            }
//            break;
//            case '3':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel_y.ki = data_Get;break;
//                case '2':cascade_ang_y.ki = data_Get;break;
//                case '3':cascade_vel_y.ki = data_Get;break;
//            }break;
//        }break;
//        case 'D':
//        switch(DataBuff[1])
//        {
//            case '1':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel.kd = data_Get;break;
//                case '2':cascade_ang.kd= data_Get;break;
//                case '3':cascade_vel.kd = data_Get;break;
//            }break;
//            case '2':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel_p.kd = data_Get;break;
//                case '2':cascade_ang_p.kd = data_Get;break;
//                case '3':cascade_vel_p.kd = data_Get;break;
//            }break;
//            case '3':
//            switch(DataBuff[2])
//            {
//                case '1':cascade_ang_vel_y.kd = data_Get;break;
//                case '2':cascade_ang_y.kd = data_Get;break;
//                case '3':cascade_vel_y.kd = data_Get;break;
//            }break;
//        }break;
//        case 'M':
//            switch(DataBuff[1])
//            {
//                case '1':
//                    {
//                        balance_angle_y_cam  = data_Get;
//                        balance_angle_y = balance_angle_y_cam;
// 历史参数分支结束。
//                case '2':
//                    {
//                        balance_angle_x_cam = data_Get;
//                        balance_angle_x = balance_angle_x_cam;
// 历史俯仰控制参数分支结束。
//                case '3':
//                    {
// 历史参数保存逻辑。
// 历史速度环与转向控制参数写入逻辑。
//                        pid_write_eeprom(&cascade_vel, &cascade_vel_p, &cascade_ang, &cascade_ang_p, &cascade_ang_vel, &cascade_ang_vel_p,&cascade_vel_y,&cascade_ang_y,&cascade_ang_vel_y);
//                        ctrl_write_eeprom();
//                        ctrl_write_eeprom_rtk();
//                    }break;
//                case '4':
//                    {
//                        balance_angle_y_rtk  = data_Get;
//                        balance_angle_y = balance_angle_y_rtk;
//                    }break;
//                case '5':
//                    {
//                        balance_angle_x_rtk  = data_Get;
//                        balance_angle_x = balance_angle_x_rtk;
//                    }break;
//                case '6': Yaw_err= data_Get;break;
//            }break;
//          case 'R': my_core_1();break;
//    }
}

void set_parameter(float *variable, int channel)
{
    if (channel >= 0 && channel <= 7) {
        if (qcar_debug_params.update_flag[channel])
        {
            qcar_debug_params.update_flag[channel] = 0;
            *variable = qcar_debug_params.data[channel];
        }
    }
}

void set_parameter_update(void)
{
    seekfree_assistant_debug_param_analysis(&qcar_debug_params);
//    set_parameter(&mahony_filter.Kp, 0,0,10);
//    set_parameter(&mahony_filter.Ki, 1,0,10);
    set_parameter(&cascade_ang_vel_roll.kp, 0);
    set_parameter(&cascade_ang_vel_roll.ki, 1);
    set_parameter(&cascade_ang_roll.kp, 2);
    set_parameter(&cascade_vel_roll.kp, 3);
    set_parameter(&cascade_ang_vel_pitch.kp, 4);
    set_parameter(&cascade_ang_vel_pitch.ki, 5);
    set_parameter(&cascade_ang_pitch.kp, 6);
    set_parameter(&cascade_vel_pitch.kp, 7);
}

void set_parameter_update_y(void)
{
    seekfree_assistant_debug_param_analysis(&qcar_debug_params);
    set_parameter(&mono_params.turn_k1_l, 0);
    set_parameter(&mono_params.turn_k2_l, 1);
    set_parameter(&mono_params.turn_k1_r, 2);
    set_parameter(&mono_params.turn_k2_r, 3);
    set_parameter(&mono_params.turn_k1_p_l, 4);
}

void oscilloscope_send_update(void)
{
//    seekfree_assistant_oscilloscope_config(&qcar_scope, 8, qcar_scope_data);
//    qcar_scope_data[0]=cascade_vel_out;
//    qcar_scope_data[1]=mahony_filter.roll;
//    qcar_scope_data[2]=cascade_angle_out;
//    qcar_scope_data[3]=mahony_filter.gyro.x;
//    qcar_scope_data[4]=cascade_vel_out_p;
//    qcar_scope_data[5]=mahony_filter.pitch;
//    qcar_scope_data[6]=cascade_angle_out_p;
//    qcar_scope_data[7]=mahony_filter.gyro.y;
//    seekfree_assistant_oscilloscope_send(&qcar_scope);

    seekfree_assistant_oscilloscope_config(&qcar_scope, 6, qcar_scope_data);
//    qcar_scope_data[0]=cascade_vel_out_p;
//    qcar_scope_data[0]=cascade_angle_out_p;
//    qcar_scope_data[1]=cascade_angle_vel_out_p;
//    qcar_scope_data[3]=real_mid_speed;
//    qcar_scope_data[4]=mahony_filter.pitch;
    qcar_scope_data[0]=imu660rb_gyro_x;
    qcar_scope_data[1]=imu660rb_acc_y;
    qcar_scope_data[2]=imu660rb_gyro_y;
    qcar_scope_data[3]=imu660rb_acc_x;
    qcar_scope_data[4]=imu660rb_gyro_z;
    qcar_scope_data[5]=imu660rb_acc_z;
    seekfree_assistant_oscilloscope_send(&qcar_scope);
}

void oscilloscope_send_update_y(void)
{
    seekfree_assistant_oscilloscope_config(&qcar_scope, 8, qcar_scope_data);
    qcar_scope_data[0]=mono_beacon.Yaw_err;
    qcar_scope_data[1]=cascade_angle_out_y;
    qcar_scope_data[2]=mahony_filter.gyro.z;
    qcar_scope_data[3]=cascade_angle_vel_out_y;
    qcar_scope_data[4]=real_mid_speed*0.001f;
    qcar_scope_data[5]=mono_beacon.dynamic_zero_roll;
    qcar_scope_data[6]=mahony_filter.roll;
    qcar_scope_data[7]=mono_beacon.dynamic_zero_pitch;
    seekfree_assistant_oscilloscope_send(&qcar_scope);
}

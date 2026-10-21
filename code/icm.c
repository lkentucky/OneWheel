#include "icm.h"
#include "zf_common_headfile.h"

Offset GYRO_offset;//陀螺仪零漂
Offset ACC_offset;

float gyro_x, gyro_y, gyro_z;
float icm_gyro_x, icm_gyro_y, icm_gyro_z;
float acc_x, acc_y, acc_z;

char Offset_OK; ////判断是否存在零漂标志位
float  q0=1,q1=0,q2=0,q3=0;
float  exInt=0,eyInt=0,ezInt=0;

float     AngleX=0;                    //翻滚角
float     AngleY=0;                    //俯仰角四元数解算出来的欧拉角
float     AngleZ=0;                    //航向角

//----------------------------零漂初始化---------------------------------
void icm20602_offset(void)
{
    int16 i, Count = 2000;
    int64 temp[6] = {0};

    ACC_offset.X = 0;
    ACC_offset.Y = 0;
    ACC_offset.Z = 0;

    GYRO_offset.X = 0;
    GYRO_offset.Y = 0;
    GYRO_offset.Z = 0;

    for(i = 0; i < Count; i++)
    {
        imu660ra_get_acc();
        imu660ra_get_gyro();

//        temp[0] += imu660ra_acc_x;
//        temp[1] += imu660ra_acc_y;
//        temp[2] += imu660ra_acc_z;
        temp[3] += imu660ra_gyro_x;
        temp[4] += imu660ra_gyro_y;
        temp[5] += imu660ra_gyro_z;

        system_delay_ms(1);

    }

//    ACC_offset.X = (float)(temp[0] / Count);
//    ACC_offset.Y = (float)(temp[1] / Count);
//    ACC_offset.Z = (float)(temp[2] / Count);
    GYRO_offset.X = (float)(temp[3] / Count);
    GYRO_offset.Y = (float)(temp[4] / Count);
    GYRO_offset.Z = (float)(temp[5] / Count);

    Offset_OK = 1;
}

//void icm20602_GetData(void)
//{
//    imu660ra_get_acc();
//    imu660ra_get_gyro();
//
//    acc_x = (imu660ra_acc_x)*0.000244140625; //除以4096
//    acc_y = (imu660ra_acc_y)*0.000244140625;
//    acc_z = (imu660ra_acc_z)*0.000244140625;
//
//    gyro_x = ((float)imu660ra_gyro_x)*0.060975609756*0.0174533f;//*0.061038881767686*0.0174533f;//   1/16.4   弧度每秒
//    gyro_y = ((float)imu660ra_gyro_y)*0.060975609756*0.0174533f;//*0.061038881767686*0.0174533f;
//    gyro_z = ((float)imu660ra_gyro_z)*0.060975609756*0.0174533f;//*0.061038881767686*0.0174533f;
//
//    icm_gyro_x = ((float)imu660ra_gyro_x)*0.060975609756;//*0.061038881767686;//   1/16.4   弧度每秒
//    icm_gyro_y = ((float)imu660ra_gyro_y)*0.060975609756;//*0.061038881767686;
//    icm_gyro_z = ((float)imu660ra_gyro_z)*0.060975609756;//*0.061038881767686;
//
//}

void icm20602_GetData(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();

    acc_x = (imu660ra_acc_x)*0.0002441f; //除以4096
    acc_y = (imu660ra_acc_y)*0.0002441f;
    acc_z = (imu660ra_acc_z)*0.0002441f;

    gyro_x = ((float)imu660ra_gyro_x)*0.0609756f*0.0174533f;//*0.061038881767686*0.0174533f;//   1/16.4   弧度每秒
    gyro_y = ((float)imu660ra_gyro_y)*0.0609756f*0.0174533f;//*0.061038881767686*0.0174533f;
    gyro_z = ((float)imu660ra_gyro_z)*0.0609756f*0.0174533f;//*0.061038881767686*0.0174533f;

    icm_gyro_x = ((float)imu660ra_gyro_x)*0.0609756f;//*0.061038881767686;//   1/16.4   弧度每秒
    icm_gyro_y = ((float)imu660ra_gyro_y)*0.0609756f;//*0.061038881767686;
    icm_gyro_z = ((float)imu660ra_gyro_z)*0.0609756f;//*0.061038881767686;

}


float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y*(1.5f - (halfx * y * y));
    return y;
}

//---------------------四元数计算角度----------------
void IMUupdate(float G_X, float G_Y, float G_Z, float A_X, float A_Y, float A_Z)
{
    float norm;
    float vx,vy,vz;
    float ex,ey,ez;
    float Kp=2.0f;
    float Ki=0.005f;
    float halfT=0.0025f;

    norm = invSqrt(A_X*A_X + A_Y*A_Y + A_Z*A_Z);
    A_X = A_X * norm;
    A_Y = A_Y * norm;
    A_Z = A_Z * norm;           //加速度归一化得到载体坐标系下的加速度

    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;  //提取姿态矩阵中的重力分量（载体坐标系下重力在三个轴上的分量）


    ex = (A_Y*vz - A_Z*vy);//求姿态误差
    ey = (A_Z*vx - A_X*vz);
    ez = (A_X*vy - A_Y*vx);

    exInt = exInt + ex * Ki;//对误差积分
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    G_X = G_X + Kp*ex + exInt;//互补滤波
    G_Y = G_Y + Kp*ey + eyInt;
    G_Z = G_Z + Kp*ez + ezInt;

    q0 = q0 + (-q1*G_X - q2*G_Y - q3*G_Z) * halfT;
    q1 = q1 + ( q0*G_X + q2*G_Z - q3*G_Y) * halfT;
    q2 = q2 + ( q0*G_Y - q1*G_Z + q3*G_X) * halfT;
    q3 = q3 + ( q0*G_Z + q1*G_Y - q2*G_Z) * halfT;

    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    AngleY = asinf(2*(q0*q2 - q1*q3)) * 57.2957795f; //俯仰角 绕y轴旋转的角 pitch
    AngleX = asinf(2*(q0*q1 + q2*q3 )) * 57.2957795f; //  翻滚   roll
    AngleZ = atan2(2*(q1*q2 + q0*q3), (-2*q2*q2 - 2*q3*q3 + 1)) * 57.2957795f;//偏航角yaw绕z轴旋转角
}

void icm_update(void)
{
    icm20602_GetData();
    IMUupdate(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
}

void icm_test(void)
{
    imu660ra_init();
    char txt[32];
    while (TRUE)
    {
        icm_update();//计算角度
        sprintf(txt,"AngleX:%f",AngleX);
        ips200_show_string(40,20,txt);
        sprintf(txt,"AngleY:%f",AngleY);
        ips200_show_string(40,40,txt);
        sprintf(txt,"AngleZ:%f",AngleZ);
        ips200_show_string(40,60,txt);
        sprintf(txt,"icm_gyro_x:%f",icm_gyro_x);
        ips200_show_string(40,80,txt);
        sprintf(txt,"icm_gyro_y:%f",icm_gyro_y);
        ips200_show_string(40,100,txt);
        sprintf(txt,"icm_gyro_z:%f",icm_gyro_z);
        ips200_show_string(40,120,txt);
    }
}

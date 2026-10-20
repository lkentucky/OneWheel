///*
// * four_posture.c
// *
// *  Created on: 2025年4月15日
// *      Author: lenovo
// */
//#include "four_posture.h"
//
//ICM_Angle icm_angle;
//
//#define fullT 0.005f // 采样周期
//float a_zi = 0.07, j_z;                // 陀螺仪比例因子0.07 //对应陀螺仪得2g量程
//Quaternion_t Q = {1, 0, 0, 0};
//float exInt = 0, eyInt = 0, ezInt = 0; // 误差积分项
//float Kp = 3, Ki = 0.00; // PID 控制参数
//#define halfT fullT / 2 // 采样周期的一半4
//int16 acc[3];
//int16 gyro[3];
//ICM_DATA icm_data_lpf;
//void Attitude_get(void)
//{
//
//    imu660rb_get_acc();
//     imu660rb_get_gyro();
//     gyro[0] = imu660rb_gyro_x-7.0;    //偏移纠正
//     gyro[1] = imu660rb_gyro_y+10;
//     gyro[2] = imu660rb_gyro_z-1.5;
//     acc[0]  = imu660rb_acc_x;
//     acc[1]  = imu660rb_acc_y;
//     acc[2]  = imu660rb_acc_z;
//
//    // 低通滤波处理加速度计数据
//     icm_data_lpf.accX = icm_data_lpf.accX + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)acc[0] - icm_data_lpf.accX)); // 低通滤波
//      icm_data_lpf.accY = icm_data_lpf.accY + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)acc[1] - icm_data_lpf.accY));
//      icm_data_lpf.accZ = icm_data_lpf.accZ + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)acc[2] - icm_data_lpf.accZ));
//      // 低通滤波处理陀螺仪数据
//      icm_data_lpf.gyroX = icm_data_lpf.gyroX + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)gyro[0] - icm_data_lpf.gyroX));
//      icm_data_lpf.gyroY = icm_data_lpf.gyroY + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)gyro[1] - icm_data_lpf.gyroY));
//      icm_data_lpf.gyroZ = icm_data_lpf.gyroZ + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3+ balance_cascade.cascade_value.call_cycle)) * ((float)gyro[2] - icm_data_lpf.gyroZ));
//
//    IMUupdate(&icm_data_lpf, &icm_angle); // 姿态更新
//}
//void IMUupdate(ICM_DATA *pMpu, ICM_Angle *pAngle)
//{
//    float norm;
//    float vx, vy, vz;
//    float ex, ey, ez;
//    float a, b;
//    float gx, gy, gz, ax, ay, az;
//
//    gx = pMpu->gyroX;
//    gy = pMpu->gyroY;
//    gz = pMpu->gyroZ;
//    ax = pMpu->accX;
//    ay = pMpu->accY;
//    az = pMpu->accZ;
//
//    // 角速度单位转换（度 -> 弧度）
//    gx = gx * a_zi * 0.017453;
//    gy = gy * a_zi * 0.017453;
//    gz = gz * a_zi * 0.017453;
//
//    // 归一化加速度数据
//    norm = newton_sqrt(ax * ax + ay * ay + az * az);
//    if (norm != 0)
//    {
//        ax = ax / norm;
//        ay = ay / norm;
//        az = az / norm;
//    }
//
//    // 计算当前的重力方向
//    vx = 2 * (Q.q1 * Q.q3 - Q.q0 * Q.q2);
//    vy = 2 * (Q.q0 * Q.q1 + Q.q2 * Q.q3);
//    vz = Q.q0 * Q.q0 - Q.q1 * Q.q1 - Q.q2 * Q.q2 + Q.q3 * Q.q3;
//
//    // 计算测量误差（加速度计与估计重力方向的误差）
//    ex = (ay * vz - az * vy);
//    ey = (az * vx - ax * vz);
//    ez = (ax * vy - ay * vx);
//
//    // 误差积分
//    exInt = exInt + ex * Ki;
//    eyInt = eyInt + ey * Ki;
//    ezInt = ezInt + ez * Ki;
//
//    // 调整角速度
//    gx = gx + Kp * ex + exInt;
//    gy = gy + Kp * ey + eyInt;
//    gz = gz + Kp * ez + ezInt;
//    j_z = gy;
//
//    // 四元数更新（积分）
//    Q.q0 = Q.q0 + (-Q.q1 * gx - Q.q2 * gy - Q.q3 * gz) * halfT;
//    Q.q1 = Q.q1 + (Q.q0 * gx + Q.q2 * gz - Q.q3 * gy) * halfT;
//    Q.q2 = Q.q2 + (Q.q0 * gy - Q.q1 * gz + Q.q3 * gx) * halfT;
//    Q.q3 = Q.q3 + (Q.q0 * gz + Q.q1 * gy - Q.q2 * gx) * halfT;
//
//    // 归一化四元数
//    norm = newton_sqrt(Q.q0 * Q.q0 + Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3);
//    if (norm != 0)
//    {
//        Q.q0 = Q.q0 / norm;
//        Q.q1 = Q.q1 / norm;
//        Q.q2 = Q.q2 / norm;
//        Q.q3 = Q.q3 / norm;
//    }
//
//    // 计算姿态角
//    a = -asin(-2 * Q.q1 * Q.q3 + 2 * Q.q0 * Q.q2) * 57.324841;                                   // Pitch
//    b = atan2(2 * Q.q2 * Q.q3 + 2 * Q.q0 * Q.q1, -2 * Q.q1 * Q.q1 - 2 * Q.q2 * Q.q2 + 1) * 57.3; // Roll
//    pAngle->pitch = a;
//    pAngle->roll = b;
//}
//double newton_sqrt(double n)
//{
//    double x = n;
//    double next_x;
//
//    while (1)
//    {
//        next_x = 0.5 * (x + n / x);
//        if (fabs(next_x - x) < 1e3 - 1)
//        {
//            break;
//        }
//        x = next_x;
//    }
//    return x;
//}

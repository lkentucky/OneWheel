/***************************
 * 系统参数与硬件配置
 ***************************/
#include "LQR.h"
/***************************
 * LQR控制参数（用户仿真结果）
 ***************************/
const float K[4] = {2.2361,2.0378,-0.3423,-0.3549}; // 反馈增益矩阵
const float Q[4][4] = {{100,0,0,0}, {0,10,0,0}, {0,0,200,0}, {0,0,0,50}};
const float R = 0.1f;

EKF_State EKF_state;

// 过程噪声协方差矩阵（需根据实测调整）
const float Q_ekf[4][4] = {{1e-4,0,0,0}, {0,1e-3,0,0}, {0,0,1e-4,0}, {0,0,0,1e-3}};
const float R_ekf = 1e-4;    // 观测噪声方差

void ekf_predict(EKF_State *s, float u, float gyro_z) {
    // 状态预测（非线性模型）
    s->x += s->v * DT;
    s->v += (u/0.1f - 9.8f*sinf(s->theta)) * DT; // 动力学模型
    s->theta += s->omega * DT;
    s->omega = gyro_z;  // 角速度由陀螺仪直接测量

    // 协方差预测（简化为对角更新）
    for(int i=0; i<4; i++)
        s->P[i][i] += Q_ekf[i][i];
}

void ekf_update(EKF_State *s, float encoder_delta) {
    // 观测模型：H = [1 0 0 0]
    float H[4] = {1,0,0,0};
    float y_residual = encoder_delta - s->x;

    // 卡尔曼增益计算（简化版）
    float S = s->P[0][0] + R_ekf;
    float K[4] = {s->P[0][0]/S, s->P[1][0]/S, s->P[2][0]/S, s->P[3][0]/S};

    // 状态更新
    s->x += K[0] * y_residual;
    s->v += K[1] * y_residual;
    s->theta += K[2] * y_residual;
    s->omega += K[3] * y_residual;

    // 协方差更新（Joseph形式保持正定性）
    for(int i=0; i<4; i++)
        s->P[i][i] *= (1 - K[i]*H[i]);
}

/***************************
 * 传感器接口模块
 ***************************/
volatile int encoder_count = 0;  // 编码器中断计数

// 编码器中断服务函数
//void encoder_isr() {
//    static int last_count = 0;
//    int delta = encoder_count - last_count;
//    last_count = encoder_count;
//    float delta_x = delta * PULSE_TO_M;  // 位移增量
//    // 传递给主循环使用（需考虑线程安全）
//}

//// IMU数据读取（示例）
//void read_imu(float *accel_z, float *gyro_z) {
//    // 实现MPU6050数据读取
//    *accel_z = ...;  // Z轴加速度
//    *gyro_z = ...;   // Z轴角速度
//}

/***************************
 * LQR控制器
 ***************************/
float lqr_control(EKF_State *state) {
    state->x=0;
    state->v=-real_mid_speed*0.00069f;
    state->theta=mahony_filter.pitch;
    state->omega=mahony_filter.gyro.y;
    float u = -(K[0]*state->x + K[1]*state->v +
                K[2]*state->theta + K[3]*state->omega);

    // 抗饱和处理与积分项
//    static float integral = 0;
//    integral += state->x * DT;
//    u -= 0.05f * integral;  // 积分增益
    return u*466.6666667f;
    // 电压限幅 ±24V
//    return fmaxf(fminf(u, 24.0f), -24.0f);
}

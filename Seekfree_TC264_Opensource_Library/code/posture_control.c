/* 直立控制：1ms IMU/角速度、5ms 角度、20ms 速度；主循环只发启动/停车请求。 */
#include "zf_common_headfile.h"
#include "posture_control.h"
#include <float.h>

int16 acc[3];
int16 gyro[3];
ICM_DATA icm_data_lpf;
ICM_DATA re_icm_data;
float pitch_filiter;
volatile int16 momentum_left_duty;
volatile int16 momentum_right_duty;
static volatile uint8 start_pending;
static volatile uint8 stop_pending;
static volatile uint32 processed_stop_epoch;
static volatile uint32 start_epoch;
static uint8 was_running;

void imu_data_get(void)
{
    float alpha = IMU_SAMPLE_DT_S / (IMU_LOWPASS_TIME_CONSTANT_S + IMU_SAMPLE_DT_S);
    imu660rb_get_acc();
    imu660rb_get_gyro();
    gyro[0] = (int16)(imu660rb_gyro_x - IMU_GYRO_BIAS_X);
    gyro[1] = (int16)(imu660rb_gyro_y - IMU_GYRO_BIAS_Y);
    gyro[2] = (int16)(imu660rb_gyro_z - IMU_GYRO_BIAS_Z);
    acc[0] = imu660rb_acc_x; acc[1] = imu660rb_acc_y; acc[2] = imu660rb_acc_z;
    if (func_abs(gyro[0]) < IMU_GYRO_DEADBAND_RAW) { gyro[0] = 0; }
    if (func_abs(gyro[1]) < IMU_GYRO_DEADBAND_RAW) { gyro[1] = 0; }
    if (func_abs(gyro[2]) < IMU_GYRO_DEADBAND_RAW) { gyro[2] = 0; }
    icm_data_lpf.accX += alpha * ((float)acc[0] - icm_data_lpf.accX);
    icm_data_lpf.accY += alpha * ((float)acc[1] - icm_data_lpf.accY);
    icm_data_lpf.accZ += alpha * ((float)acc[2] - icm_data_lpf.accZ);
    icm_data_lpf.gyroX += alpha * ((float)gyro[0] - icm_data_lpf.gyroX);
    icm_data_lpf.gyroY += alpha * ((float)gyro[1] - icm_data_lpf.gyroY);
    icm_data_lpf.gyroZ += alpha * ((float)gyro[2] - icm_data_lpf.gyroZ);
}

void icm_init(void)
{
    icm_data_lpf.accX = imu660rb_acc_x;
    icm_data_lpf.accY = imu660rb_acc_y;
    icm_data_lpf.accZ = imu660rb_acc_z;
    // 低通滤波处理陀螺仪数据
    icm_data_lpf.gyroX = imu660rb_gyro_x;
    icm_data_lpf.gyroY = imu660rb_gyro_y;
    icm_data_lpf.gyroZ = imu660rb_gyro_z;


    re_icm_data.accX = imu660rb_acc_x;
    re_icm_data.accY = imu660rb_acc_y;
    re_icm_data.accZ = imu660rb_acc_z;
    // 低通滤波处理陀螺仪数据
    re_icm_data.gyroX = imu660rb_gyro_x;
    re_icm_data.gyroY = imu660rb_gyro_y;
    re_icm_data.gyroZ = imu660rb_gyro_z;
}

void control_stop(void)
{
    safety_output_request_stop();
    start_pending = 0;
    stop_pending = 0;
    was_running = 0;
    expect_speed = 0;
    retarder_reset();
    balance_cascade_reset();
    momentum_left_duty = 0;
    momentum_right_duty = 0;
    bujin_motor_contron(0);
    processed_stop_epoch = safety_output_epoch();
}

void control_request_stop(void)
{
    safety_output_request_stop();
    stop_pending = 1;
    __dsync();
}

uint8 control_request_start(void)
{
    uint32 observed_stop = safety_output_epoch();
    if (!APP_DRIVE_ENABLE || !app_imu_ready || system_count < APP_CONTROL_WARMUP_MS ||
        run_flag || start_pending || stop_pending || processed_stop_epoch != observed_stop ||
        !safety_output_can_start()) { return 0; }
    start_epoch = observed_stop;
    __dsync();
    start_pending = 1;
    return 1;
}

void dynamic_value_control(void)
{
    float roll = CONTROL_ROLL_ENABLE ? balance_cascade.angular_speed_cycle.out : 0.0f;
    float yaw = CONTROL_YAW_ENABLE ? balance_cascade.yaw_angular_speed_cycle.out : 0.0f;
    if (!run_flag)
    {
        momentum_left_duty = 0; momentum_right_duty = 0;
        return;
    }
    if (!(roll >= -FLT_MAX && roll <= FLT_MAX) || !(yaw >= -FLT_MAX && yaw <= FLT_MAX))
    {
        safety_output_trip(SAFETY_FAULT_COMMAND);
        control_stop();
        return;
    }
    if (func_abs(balance_cascade.cascade_value.filtering_angle) > PROTECT_ROLL_RAW ||
        func_abs(roll) > PROTECT_ROLL_DUTY)
    {
        control_stop();
        return;
    }
    momentum_left_duty = (int16)func_limit_ab(-roll + yaw, -MOMENTUM_DUTY_LIMIT, MOMENTUM_DUTY_LIMIT) * MOMENTUM_LEFT_SIGN;
    momentum_right_duty = (int16)func_limit_ab(-roll - yaw, -MOMENTUM_DUTY_LIMIT, MOMENTUM_DUTY_LIMIT) * MOMENTUM_RIGHT_SIGN;
    /* Publish after the entire control cycle; CPU1 owns runtime hardware TX. */
}

static void control_feedback_poll(void)
{
    if (system_count % CONTROL_SPEED_PERIOD_MS == 0)
    {
        int16 left, right;
        encoder_data_1 = (int16)(encoder_get_count(ENCODER_1) * ENCODER_DIRECTION_SIGN);
        wifi_encoder_total += (uint32)(int32)encoder_data_1;
        encoder_clear_count(ENCODER_1);
        if (small_driver_feedback_snapshot(&left, &right))
        {
            car_speed = (int16)(((int32)left - (int32)right) / 2);
        }
    }
    if (system_count % CONTROL_ANGLE_PERIOD_MS == 0)
    {
        first_order_complementary_filtering(&balance_cascade.cascade_value, ROLL_GYRO_RAW, ROLL_ACCEL_RAW);
        first_order_complementary_filtering(&balance_cascade.pitch_cascade_value, PITCH_GYRO_RAW, PITCH_ACCEL_RAW);
    }
}

static void control_step(void)
{
    if (stop_pending || processed_stop_epoch != safety_output_epoch()) { control_stop(); }
    if (safety_output_faults())
    {
        if (run_flag || was_running || start_pending) { control_stop(); }
        return;
    }
    /* Initialization failure means no valid configured sensor; report imu=0
     * through the background path instead of polling failed hardware in ISR. */
    if (!app_imu_ready)
    {
        if (run_flag) { control_stop(); }
        return;
    }
    imu_data_get();
    control_feedback_poll(); /* 停车时继续测量，便于上位机观察零偏和反馈。 */
    if (safety_output_faults()) { control_stop(); return; }
    if (!APP_DRIVE_ENABLE) { if (run_flag) { control_stop(); } return; }
    if (start_pending)
    {
        start_pending = 0;
        __dsync();
        if (start_epoch != safety_output_epoch() || stop_pending || !safety_output_can_start()) { control_stop(); return; }
        balance_cascade_reset();
        retarder_reset();
        run_flag = 1;
    }
    if (!run_flag || system_count < APP_CONTROL_WARMUP_MS)
    {
        if (was_running) { control_stop(); }
        return;
    }
    was_running = 1;
    expect_speed = retarder(wifi_target_speed, wifi_ramp_step);
    if (system_count % CONTROL_SPEED_PERIOD_MS == 0)
    {
        if (CONTROL_ROLL_ENABLE) { pid_control(&balance_cascade.speed_cycle, 0, car_speed); }
        if (CONTROL_PITCH_ENABLE) { pid_control(&balance_cascade.pitch_speed_cycle, expect_speed, encoder_data_1); }
        if (CONTROL_MOTOR_ENABLE) { pid_control(&balance_cascade.motor_cycle, expect_speed, encoder_data_1); }
    }
    if (system_count % CONTROL_ANGLE_PERIOD_MS == 0)
    {
        if (CONTROL_ROLL_ENABLE)
        {
            pid_control(&balance_cascade.angle_cycle, balance_cascade.speed_cycle.out,
                        balance_cascade.cascade_value.filtering_angle - (APP_CAMERA_ENABLE ? k1 * erro_yawan : 0.0f));
        }
        if (CONTROL_PITCH_ENABLE)
        {
            pid_control(&balance_cascade.pitch_angle_cycle, balance_cascade.pitch_speed_cycle.out,
                        balance_cascade.pitch_cascade_value.filtering_angle);
        }
        if (CONTROL_YAW_ENABLE) { pid_control(&balance_cascade.yaw_erro_cycle, 0, APP_CAMERA_ENABLE ? wifi_vision_erro(erro) : 0.0f); }
    }
    if (CONTROL_ROLL_ENABLE) { pid_control_incremental(&balance_cascade.angular_speed_cycle, balance_cascade.angle_cycle.out, ROLL_GYRO_RAW); }
    if (CONTROL_PITCH_ENABLE) { pid_control_incremental(&balance_cascade.pitch_angular_speed_cycle, -balance_cascade.pitch_angle_cycle.out, PITCH_GYRO_RAW); }
    if (CONTROL_YAW_ENABLE) { pid_control(&balance_cascade.yaw_angular_speed_cycle, -balance_cascade.yaw_erro_cycle.out, YAW_GYRO_RAW); }
    if (!(balance_cascade.pitch_angular_speed_cycle.out >= -FLT_MAX && balance_cascade.pitch_angular_speed_cycle.out <= FLT_MAX) ||
        !(balance_cascade.motor_cycle.out >= -FLT_MAX && balance_cascade.motor_cycle.out <= FLT_MAX))
    {
        safety_output_trip(SAFETY_FAULT_COMMAND);
        control_stop();
        return;
    }
    /* 先检查两组倾角和行进轮指令，防止一组已下发后另一组才触发保护。 */
    if (func_abs(balance_cascade.cascade_value.filtering_angle) > PROTECT_ROLL_RAW ||
        func_abs(balance_cascade.pitch_cascade_value.filtering_angle) > PROTECT_PITCH_RAW ||
        func_abs(balance_cascade.pitch_angular_speed_cycle.out + balance_cascade.motor_cycle.out) > PROTECT_TRAVEL_DUTY)
    {
        control_stop();
        return;
    }
    dynamic_value_control();
    bujin_motor_contron((int)(balance_cascade.pitch_angular_speed_cycle.out + balance_cascade.motor_cycle.out));
}

void pit_isr_callback(void)
{
    system_count += CONTROL_PERIOD_MS;
    control_step();
    safety_output_publish(momentum_left_duty, momentum_right_duty,
                          motor_requested_output(), run_flag ? start_epoch : safety_output_epoch());
}

/* 保留旧接口供已有调用方使用；正常路径使用 pit_isr_callback。 */
void find_ma(void) { imu_data_get(); }
void imu_filiter(void) { re_icm_data = icm_data_lpf; }

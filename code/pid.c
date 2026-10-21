#include "pid.h"

uint8_t cnt_1ms;
uint8_t cnt2_1ms;
uint8_t cnt3_1ms;

int16 real_speed;

uint8_t shacheflag;
uint8_t turn_flag;
volatile uint8_t cam_flag;
uint8_t tele_flag;

float cascade_vel_out;
float cascade_vel_out_p;
float cascade_vel_out_y;

float cascade_angle_out;
float cascade_angle_out_p;
float cascade_angle_out_y;

float cascade_angle_vel_out;
float cascade_angle_vel_out_p;
float cascade_angle_vel_out_y;

int16 costrained_flywheel_speed_left;
int16 costrained_flywheel_speed_right;
int16 costrained_wheel_speed;

Mono_params mono_params;
Mono_beacon mono_beacon;

pid_param_t cascade_vel_roll;// 级联 PID 速度环参数。
pid_param_t cascade_vel_pitch;
pid_param_t cascade_vel_yaw;
pid_param_t cascade_ang_roll;// 级联 PID 角度环参数。
pid_param_t cascade_ang_pitch;
pid_param_t cascade_ang_yaw;
pid_param_t cascade_ang_vel_roll;// 级联 PID 角速度环参数。
pid_param_t cascade_ang_vel_pitch;
pid_param_t cascade_ang_vel_yaw;

float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

float cascade_angle_vel(pid_param_t * pid, float target_angle_vel, float now_angle_vel)
{
    static float err;
    err = target_angle_vel - now_angle_vel;// 计算实际角速度与目标角速度的误差。

    // 累计误差用于积分项。
    pid->integrator += err;

    pid->out_p = pid->kp * err;
    pid->out_i = pid->ki * constrain_float(pid->integrator, -3, 3);
    pid->out_d = pid->kd * (err-pid->last_error);

    pid->last_error = err;

    pid->out =( pid->out_p  + pid->out_i+ pid->out_d ) ;

    return pid->out;
}

// 计算级联角度环。

float cascade_angle(pid_param_t * pid, float target_angle, float now_angle, float gyro)
{
    static float err;
    err = target_angle - now_angle;// 计算角度环控制量。

    pid->integrator += err;

    pid->integrator = constrain_float(pid->integrator, -30, 30);

    pid->out_p = pid->kp * err;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * gyro;

    pid->last_error = err;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;

}

float cascade_angle_y(pid_param_t * pid, float err)
{

    pid->integrator += err;

    pid->integrator = constrain_float(pid->integrator, -30, 30);

    pid->out_p = pid->kp * err;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (err - pid->last_error);

    pid->last_error = err;

    pid->out = pid->out_p + pid->out_i + pid->out_d;
    return pid->out;
}
float increment_cascade_angle_y(pid_param_t * pid, float err)
{

    pid->derivative = err-pid->last_error;

    pid->out_p = pid->kp * (err-pid->last_error);
    pid->out_i = pid->ki * err;
    pid->out_d = pid->kd * (pid->derivative-pid->last_derivative);

    pid->last_error = err;
    pid->last_derivative = pid->derivative;

    pid->out += pid->out_p  + pid->out_i+ pid->out_d  ;
    pid->out = constrain_float(pid->out,-180.0f,180.0f);

    return pid->out;
}

float cascade_velocity(pid_param_t * pid, float target_vel, float now_vel)
{
    static float err;
    err = target_vel - now_vel;// 计算角速度环控制量。

    // 累计误差用于积分项。
    pid->integrator += err;

    pid->integrator = constrain_float(pid->integrator, -70, 70);

    pid->out_p = pid->kp * err;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (err - pid->last_error);

    pid->last_error = err;

    pid->out = pid->out_p + pid->out_i + pid->out_d;
    return pid->out;
}

float increment_cascade_angle_vel(pid_param_t * pid,float target_angle_vel, float now_angle_vel)
{
    static float err;

    err = target_angle_vel - now_angle_vel;
    pid->derivative = err-pid->last_error;

    pid->out_p = pid->kp * (err-pid->last_error);
    pid->out_i = pid->ki * err;
    pid->out_d = pid->kd * (pid->derivative-pid->last_derivative);

    pid->last_error = err;
    pid->last_derivative = pid->derivative;

    pid->out += pid->out_p  + pid->out_i+ pid->out_d  ;
    pid->out = constrain_float(pid->out,-7500.0f,7500.0f);

    return pid->out;
}

void init_mono_params(void)
{
    mono_params.setspeed=0;
    mono_params.balance_angle_x=0.9;
    mono_params.balance_angle_y=1.5;
    mono_params.dead_zone=0;
    mono_params.dead_zone_p=0;
    mono_params.turn_k1_l=8;
    mono_params.turn_k1_r=9;
    mono_params.turn_k1_p_l=-8;
    mono_params.turn_k1_p_r=-9;
    mono_params.fast_speed=0.17;
    mono_params.slow_speed=0.15;
}

void init_mono_beacon_params(void)
{
    mono_beacon.yaw_increment=1;
    mono_beacon.dynamic_zero_threshold=10;
    mono_beacon.track_threshold=50;
    mono_beacon.turn_threshold=110;
    mono_beacon.dynamic_mid_line_kp=0.8;
    mono_beacon.dynamic_mid_line_offset=41;
    mono_beacon.img_mid_line=93;
    mono_beacon.angle_err_max=60;
}

void param_pid_set(void)
{
    cascade_ang_vel_roll.kp = 5;
    cascade_ang_vel_roll.ki = 1;
    cascade_ang_vel_roll.kd = 0;

    cascade_ang_roll.kp = 6;
    cascade_ang_roll.ki = 0;
    cascade_ang_roll.kd = 0;

    cascade_vel_roll.kp = 0.5;
    cascade_vel_roll.ki = 0;
    cascade_vel_roll.kd = 0;

    cascade_ang_vel_pitch.kp = 25;
    cascade_ang_vel_pitch.ki = 0.5;
    cascade_ang_vel_pitch.kd = 0;

    cascade_ang_pitch.kp = 6;
    cascade_ang_pitch.ki = 0;
    cascade_ang_pitch.kd = 0;

    cascade_vel_pitch.kp = 50;
    cascade_vel_pitch.ki = 0;
    cascade_vel_pitch.kd = 0;

    cascade_ang_vel_yaw.kp= 80;
    cascade_ang_vel_yaw.ki= 0;
    cascade_ang_vel_yaw.kd= 0;

    cascade_ang_yaw.kp= 2;
    cascade_ang_yaw.ki= 0;
    cascade_ang_yaw.kd= 0;
}

void turn_info_init(void)
{
    turn_info.cx=0;
    turn_info.cy=0;
    turn_info.last_turn_flag=0;
    turn_info.turn_flag=0;
    turn_info.track_flag=0;
}

float Yaw_err_update(float Yaw_err,float angle_err)
{
    float output;
    output=Yaw_err;
    if(output<angle_err)
    {
        output+=mono_beacon.yaw_increment;
        if(output>angle_err)
        {
            output=angle_err;
        }
    }
    else
    {
        output-=mono_beacon.yaw_increment;
        if(output<angle_err)
        {
            output=angle_err;
        }
    }
    return output;
}

//void track_turn_update(ImageInfo* imageinfo,Turn_info* turn_info)
//{
//    if(turn_info->turn_flag==left)
//    {
//        turn_info->track_flag=0;
//        mono_beacon.angle_err=-mono_beacon.angle_err_max;
//    }
//    if(turn_info->turn_flag==right)
//    {
//        turn_info->track_flag=0;
//        mono_beacon.angle_err=mono_beacon.angle_err_max;
//    }
//    if(imageinfo->numContours)
//    {
//        turn_info->last_turn_flag=turn_info->turn_flag;
//        turn_info->turn_flag=0;
//        if(turn_info->cy<mono_beacon.track_threshold)
//        {
//            mono_beacon.dynamic_mid_line=mono_beacon.img_mid_line;
//            mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//            if(turn_info->track_flag==0)
//            {
//                if(turn_info->last_turn_flag==right)
//                {
//                    turn_info->track_flag=right_left;
//                }
//                if(turn_info->last_turn_flag==left)
//                {
//                    turn_info->track_flag=left_right;
//                }
//                if(turn_info->last_turn_flag==0)
//                {
//                    turn_info->track_flag=right_left;
//                }
//            }
//        }
//        else
//        {
//            if(turn_info->track_flag==0)
//            {
//                if(turn_info->last_turn_flag==left)
//                {
//                    mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
//                    mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                    if(turn_info->cy>=mono_beacon.turn_threshold)
//                    {
//                        turn_info->turn_flag=right;
//                    }
//                }
//                if(turn_info->last_turn_flag==right||turn_info->last_turn_flag==0)
//                {
//                    mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
//                    mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                    if(turn_info->cy>=mono_beacon.turn_threshold)
//                    {
//                        turn_info->turn_flag=left;
//                    }
//                }
//            }
//            if(turn_info->track_flag==left_left)
//            {
//                mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
//                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                if(turn_info->cy>=mono_beacon.turn_threshold)
//                {
//                    turn_info->turn_flag=left;
//                }
//            }
//            if(turn_info->track_flag==right_right)
//            {
//                mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
//                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                if(turn_info->cy>=mono_beacon.turn_threshold)
//                {
//                    turn_info->turn_flag=right;
//                }
//            }
//            if(turn_info->track_flag==left_right)
//            {
//                mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
//                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                if(turn_info->cy>=mono_beacon.turn_threshold)
//                {
//                    turn_info->turn_flag=right;
//                }
//            }
//            if(turn_info->track_flag==right_left)
//            {
//                mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
//                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                if(turn_info->cy>=mono_beacon.turn_threshold)
//                {
//                    turn_info->turn_flag=left;
//                }
//            }
//        }
//    }
//}

void track_turn_update(ImageInfo* imageinfo,Turn_info* turn_info)
{
    if(turn_info->turn_flag==left)
    {
        turn_info->last_turn_flag=turn_info->turn_flag;
        turn_info->track_flag=0;
        mono_beacon.angle_err=-mono_beacon.angle_err_max;
    }
    if(turn_info->turn_flag==right)
    {
        turn_info->last_turn_flag=turn_info->turn_flag;
        turn_info->track_flag=0;
        mono_beacon.angle_err=mono_beacon.angle_err_max;
    }
    if(imageinfo->numContours)
    {
        turn_info->turn_flag=0;
        if(turn_info->cy<mono_beacon.track_threshold)
        {
            mono_beacon.dynamic_mid_line=mono_beacon.img_mid_line;
            mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
            if(turn_info->track_flag==0)
            {
                if(turn_info->last_turn_flag==right)
                {
                    turn_info->track_flag=right_right;
                }
                if(turn_info->last_turn_flag==left||turn_info->last_turn_flag==0)
                {
                    turn_info->track_flag=left_left;
                }
            }
        }
        else
        {
            if(turn_info->track_flag==0)
            {
                if(turn_info->last_turn_flag==left||turn_info->last_turn_flag==0)
                {
                    mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
                    mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
                    if(turn_info->cy>=mono_beacon.turn_threshold)
                    {
                        turn_info->turn_flag=left;
                    }
                }
                if(turn_info->last_turn_flag==right)
                {
                    mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
                    mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
                    if(turn_info->cy>=mono_beacon.turn_threshold)
                    {
                        turn_info->turn_flag=right;
                    }
                }
            }
            if(turn_info->track_flag==left_left)
            {
                mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
                if(turn_info->cy>=mono_beacon.turn_threshold)
                {
                    turn_info->turn_flag=left;
                }
            }
            if(turn_info->track_flag==right_right)
            {
                mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
                if(turn_info->cy>=mono_beacon.turn_threshold)
                {
                    turn_info->turn_flag=right;
                }
            }
        }
    }
}

void speed_ctrl(ImageInfo* imageinfo,Turn_info* turn_info)
{
    if(mono_params.speed_flag==fast)
    {
        mono_params.setspeed=mono_params.fast_speed;
    }

    if(mono_params.speed_flag==slow)
    {
        mono_params.setspeed=mono_params.slow_speed;
    }
}

/* States: 0 off, 1 calibration, 2 running;
 * -1 IMU failure/stale, -2 tilt, -3 invalid parameter. Faults latch until Start. */
volatile uint8 roll_rate_test_enabled;
volatile int32 roll_rate_test_state;
volatile float roll_rate_test_actual, roll_rate_test_error, roll_rate_test_pwm;
float roll_rate_test_kp = 5.0f;
float roll_rate_test_limit = 1000.0f;
float roll_rate_test_sign = 1.0f;
static uint8 rate_divider;
static uint32 rate_stale_ms, rate_sequence;

void roll_rate_test_stop(void)
{
    roll_rate_test_enabled = 0;
    roll_rate_test_state = 0;
    roll_rate_test_actual = roll_rate_test_error = roll_rate_test_pwm = 0.0f;
    rate_divider = 0;
    rate_stale_ms = 0;
}

void roll_rate_test_arm(void)
{
    roll_rate_test_stop();
    rate_sequence = imu_sample_sequence;
    roll_rate_test_state = 1;
    roll_rate_test_enabled = 1;
}

static void roll_rate_test_update(void)
{
    float rate, roll, pitch, kp, limit, direction;
    uint32 sequence, irq;
    int32 status;
    int output = 0;
    if (++rate_divider < 5) return;
    rate_divider = 0;
    irq = interrupt_global_disable();
    rate = mahony_filter.gyro.x;
    roll = imu_roll_deg;
    pitch = imu_pitch_deg;
    sequence = imu_sample_sequence;
    status = imu_attitude_status;
    kp = roll_rate_test_kp;
    limit = roll_rate_test_limit;
    direction = roll_rate_test_sign;
    interrupt_global_enable(irq);
    roll_rate_test_actual = rate;
    roll_rate_test_error = -rate; /* Target is always zero deg/s. */
    if (roll_rate_test_state == 1 || roll_rate_test_state == 2) {
        if (!shacheflag) roll_rate_test_state = 0;
        else if (status < 0) roll_rate_test_state = -1;
        else if (status == IMU_ATTITUDE_CALIBRATING) {
            /* Calibration never drives either flywheel. */
            roll_rate_test_state = 1;
            rate_stale_ms = 0;
            rate_sequence = sequence;
        } else if (!sequence || !isfinite(rate) || !isfinite(roll) || !isfinite(pitch))
            roll_rate_test_state = -1;
        else if (fabsf(roll) > 30.0f || fabsf(pitch) > 30.0f)
            roll_rate_test_state = -2;
        else if (!isfinite(kp) || kp < 0.0f || kp > 100.0f ||
                 !isfinite(limit) || limit < 0.0f || limit > 2000.0f ||
                 (direction != 1.0f && direction != -1.0f))
            roll_rate_test_state = -3;
        else {
            if (sequence == rate_sequence) rate_stale_ms += 5;
            else { rate_stale_ms = 0; rate_sequence = sequence; }
            if (rate_stale_ms >= 25) roll_rate_test_state = -1;
            else {
                roll_rate_test_state = 2;
                output = (int)roundf(constrain_float(direction * kp * (-rate), -limit, limit));
            }
        }
    }
    roll_rate_test_pwm = (float)output;
    cascade_angle_out = 0.0f;
    cascade_angle_vel_out = (float)output;
    costrained_flywheel_speed_left = (int16)output;
    costrained_flywheel_speed_right = (int16)output;
    costrained_wheel_speed = 0;
    /* Same mounting-direction convention as the existing roll mixer. */
    small_driver_set_duty(output, -output);
    motor_m_ctrl(0);
}

void cascade_ctrl(void)
{
    if (roll_rate_test_enabled) { roll_rate_test_update(); return; }
    if (imu_attitude_status!=IMU_ATTITUDE_RUNNING) {
        clear_all_cascade_out();
        small_driver_set_duty(0,0);
        motor_m_ctrl(0);
        return;
    }
    cnt_1ms++;
    cnt2_1ms++;
    if(cnt_1ms==20)
    {
        cnt_1ms=0;
        real_mid_speed = encoder_get_count(TIM6_ENCODER);
        encoder_clear_count(TIM6_ENCODER);
        real_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) * 0.005f;

        if(cam_flag)
        {
            speed_ctrl(&imageinfo,&turn_info);
        }
        if(tele_flag)
        {
            if(mono_params.speed_flag==fast)
            {
                mono_params.setspeed=mono_params.fast_speed*0.00390625f*L_UpDown;
            }

            if(mono_params.speed_flag==slow)
            {
                mono_params.setspeed=mono_params.slow_speed*0.00390625f*L_UpDown;
            }
        }

        cascade_vel_out = cascade_velocity(&cascade_vel_roll, 0.0f, real_speed);//fly_wheel
        cascade_vel_out_p = cascade_velocity(&cascade_vel_pitch, mono_params.setspeed, real_mid_speed*0.00069f);
    }

    if(cnt2_1ms==5)
    {
        cnt2_1ms=0;
        if(cam_flag){track_turn_update(&imageinfo,&turn_info);}

        if(tele_flag){mono_beacon.angle_err=(float)0.01*mono_beacon.angle_err_max*R_Around;}

        mono_beacon.angle_err=constrain_float(mono_beacon.angle_err,-mono_beacon.angle_err_max,mono_beacon.angle_err_max);
        mono_beacon.Yaw_err = Yaw_err_update(mono_beacon.Yaw_err,mono_beacon.angle_err);

        if(mono_params.speed_flag==fast)
        {
            if(fabs(mono_beacon.Yaw_err)>=mono_beacon.dynamic_zero_threshold)
            {
                if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_pitch=mono_params.turn_k1_p_l*fabs(mono_beacon.Yaw_err)*DEG2RAD;}
                else{mono_beacon.dynamic_zero_pitch=mono_params.turn_k1_p_r*mono_beacon.Yaw_err*DEG2RAD;}
            }
            else
            {
                if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_pitch=mono_beacon.dynamic_zero_threshold*0.0166667f*mono_params.turn_k1_p_l*fabs(mono_beacon.Yaw_err)*DEG2RAD;}
                else{mono_beacon.dynamic_zero_pitch=mono_beacon.dynamic_zero_threshold*0.0166667f*mono_params.turn_k1_p_r*mono_beacon.Yaw_err*DEG2RAD;}
            }

            if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_roll=mono_params.turn_k1_l*mono_beacon.Yaw_err*DEG2RAD;}
            if(mono_beacon.Yaw_err>0){mono_beacon.dynamic_zero_roll=mono_params.turn_k1_r*mono_beacon.Yaw_err*DEG2RAD;}
        }

        if(mono_params.speed_flag==slow)
        {
            if(fabs(mono_beacon.Yaw_err)>=mono_beacon.dynamic_zero_threshold)
            {
                if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_pitch=mono_params.turn_k2_p_l*fabs(mono_beacon.Yaw_err)*DEG2RAD;}
                else{mono_beacon.dynamic_zero_pitch=mono_params.turn_k2_p_r*mono_beacon.Yaw_err*DEG2RAD;}
            }
            else
            {
                if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_pitch=mono_beacon.dynamic_zero_threshold*0.0166667f*mono_params.turn_k2_p_l*fabs(mono_beacon.Yaw_err)*DEG2RAD;}
                else{mono_beacon.dynamic_zero_pitch=mono_beacon.dynamic_zero_threshold*0.0166667f*mono_params.turn_k2_p_r*mono_beacon.Yaw_err*DEG2RAD;}
            }

            if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_roll=mono_params.turn_k2_l*mono_beacon.Yaw_err*DEG2RAD;}
            if(mono_beacon.Yaw_err>0){mono_beacon.dynamic_zero_roll=mono_params.turn_k2_r*mono_beacon.Yaw_err*DEG2RAD;}

        }

        cascade_angle_out = cascade_angle(&cascade_ang_roll, -cascade_vel_out + mono_params.balance_angle_x-mono_beacon.dynamic_zero_roll, mahony_filter.roll, -mahony_filter.gyro.x);// 计算级联 PID 角度环输出。
        cascade_angle_out_p = cascade_angle(&cascade_ang_pitch, -cascade_vel_out_p + mono_params.balance_angle_y-mono_beacon.dynamic_zero_pitch, mahony_filter.pitch, -mahony_filter.gyro.y);
        cascade_angle_out_y = cascade_angle_y(&cascade_ang_yaw,mono_beacon.Yaw_err);
    }

    /* IMU samples are published by the 5ms pipeline; no nested SPI reads. */

    cascade_angle_vel_out   = increment_cascade_angle_vel(&cascade_ang_vel_roll, cascade_angle_out, mahony_filter.gyro.x); // 计算级联 PID 角速度环输出。
    cascade_angle_vel_out_p = increment_cascade_angle_vel(&cascade_ang_vel_pitch, cascade_angle_out_p, mahony_filter.gyro.y);// 累计误差。
    cascade_angle_vel_out_y = cascade_angle_vel(&cascade_ang_vel_yaw, cascade_angle_out_y, -mahony_filter.gyro.z);

//    cascade_angle_vel_out +=(cascade_angle_vel_out>0?mono_params.dead_zone:-mono_params.dead_zone);
//    cascade_angle_vel_out_p +=(cascade_angle_vel_out_p>0?mono_params.dead_zone_p:-mono_params.dead_zone_p);

    if(fabs(mahony_filter.pitch)>30||(mahony_filter.roll)>25)
    {
        cnt3_1ms++;
    }
    else
    {
        cnt3_1ms=0;
    }

    if(cnt3_1ms==1000)
    {
        cnt3_1ms=0;
        shacheflag=0;
    }

    costrained_flywheel_speed_left = (int)round(constrain_float(cascade_angle_vel_out-cascade_angle_vel_out_y*turn_flag, -7500.0f, 7500.0f)*shacheflag);
    costrained_flywheel_speed_right = (int)round(constrain_float(cascade_angle_vel_out+cascade_angle_vel_out_y*turn_flag, -7500.0f, 7500.0f)*shacheflag);
    costrained_wheel_speed = (int)round(constrain_float(cascade_angle_vel_out_p, -7500.0f, 7500.0f)*shacheflag);

    small_driver_set_duty(costrained_flywheel_speed_left , -costrained_flywheel_speed_right);
    motor_m_ctrl(costrained_wheel_speed);
}

void clear_all_cascade_out(void)
{
    cascade_ang_vel_roll.out=0;
    cascade_ang_vel_pitch.out=0;
    cascade_ang_vel_yaw.out=0;

    cascade_ang_roll.out=0;
    cascade_ang_pitch.out=0;
    cascade_ang_yaw.out=0;

    cascade_vel_roll.out=0;
    cascade_vel_roll.out=0;

    cascade_vel_out=0;
    cascade_vel_out_p=0;
    cascade_vel_out_y=0;

    cascade_angle_out=0;
    cascade_angle_out_p=0;
    cascade_angle_out_y=0;

    cascade_angle_vel_out=0;
    cascade_angle_vel_out_p=0;
    cascade_angle_vel_out_y=0;

    costrained_flywheel_speed_left=0;
    costrained_flywheel_speed_right=0;
    costrained_wheel_speed=0;
}

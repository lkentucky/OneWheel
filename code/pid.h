#ifndef _pid_h_
#define _pid_h_
#include "zf_common_headfile.h"

#define fast 1
#define slow 2

extern uint8_t shacheflag;
extern uint8_t turn_flag;
extern volatile uint8_t cam_flag;
extern uint8_t tele_flag;

extern int16 real_speed;

extern float cascade_vel_out;
extern float cascade_vel_out_p;
extern float cascade_vel_out_y;

extern float cascade_angle_out;
extern float cascade_angle_out_p;
extern float cascade_angle_out_y;

extern float cascade_angle_vel_out;
extern float cascade_angle_vel_out_p;
extern float cascade_angle_vel_out_y;

extern int16 costrained_flywheel_speed_left;
extern int16 costrained_flywheel_speed_right;
extern int16 costrained_wheel_speed;

extern Mono_params mono_params;
extern Mono_beacon mono_beacon;
extern   pid_param_t cascade_vel_roll;// 级联 PID 速度环参数。
extern   pid_param_t cascade_vel_pitch;
extern   pid_param_t cascade_vel_yaw;
extern  pid_param_t cascade_ang_roll;// 级联 PID 角度环参数。
extern   pid_param_t cascade_ang_pitch;
extern   pid_param_t cascade_ang_yaw;
extern  pid_param_t cascade_ang_vel_roll;// 级联 PID 角速度环参数。
extern  pid_param_t cascade_ang_vel_pitch;
extern  pid_param_t cascade_ang_vel_yaw;
extern  pid_param_t increment_cascade_ang_vel_p;

float constrain_float(float amt, float low, float high);        // 将数值限制在指定上下界之间。
void param_pid_set(void);
void init_mono_params(void);
void init_mono_beacon_params(void);
void turn_info_init(void);
float cascade_angle_vel(pid_param_t * pid, float target_angle_vel, float now_angle_vel);
float cascade_angle(pid_param_t * pid, float target_angle, float now_angle, float gyro);
float cascade_angle_y(pid_param_t * pid, float err);
float cascade_velocity(pid_param_t * pid, float target_vel, float now_vel);
float increment_cascade_angle_vel(pid_param_t * pid,float target_angle_vel, float now_angle_vel);
/* Isolated roll-rate test: P only, zero target, 5ms update. */
extern volatile uint8 roll_rate_test_enabled;
extern volatile int32 roll_rate_test_state;
extern volatile float roll_rate_test_actual, roll_rate_test_error, roll_rate_test_pwm;
extern float roll_rate_test_kp, roll_rate_test_limit, roll_rate_test_sign;
void roll_rate_test_arm(void);
void roll_rate_test_stop(void);
void cascade_ctrl(void);
void clear_all_cascade_out(void);
void speed_ctrl(ImageInfo* imageinfo,Turn_info* turn_info);

#endif

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
/* Flywheel Pitch/Y angle P -> rate P, 5ms update; angle KP=0 gives rate-only.
 * roll_rate_test_* names retained for Flash/host compatibility. */
extern volatile uint8 roll_rate_test_enabled;
extern volatile uint32 roll_rate_test_sequence;
extern volatile int32 roll_rate_test_state;
extern volatile float roll_rate_test_actual, roll_rate_test_error, roll_rate_test_pwm;
extern volatile float pitch_balance_target_rate, pitch_balance_error;
extern volatile float pitch_balance_actual_angle, pitch_balance_target_angle;
extern float pitch_balance_kp, pitch_balance_rate_limit;
extern float roll_rate_test_kp, roll_rate_test_limit, roll_rate_test_sign;
extern volatile uint8 ground_rate_test_enabled;
extern float ground_rate_test_kp, ground_rate_test_limit, ground_rate_test_sign;
extern float ground_angle_kp, ground_angle_rate_limit;
extern volatile uint8 dual_balance_enabled;
extern volatile float dual_angle[2],dual_target[2],dual_angle_error[2];
extern volatile float dual_rate[2],dual_target_rate[2],dual_rate_error[2],dual_pwm[2];
extern volatile uint8 rate_only_test;
extern volatile int32 control_fault_reason,control_fault_imu_status;
extern volatile uint32 control_fault_seq;
extern volatile float control_fault_dt_ms,control_startup_scale;
extern uint32 control_startup_ramp_ms;
void roll_rate_only_arm(void);
void ground_rate_only_arm(void);
void dual_balance_arm(void);
void ground_rate_test_arm(void);
void roll_rate_test_arm(void);
void roll_rate_test_stop(void);
void cascade_ctrl(void);
void clear_all_cascade_out(void);
void speed_ctrl(ImageInfo* imageinfo,Turn_info* turn_info);

#endif

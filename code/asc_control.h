/* Portable balance-only extraction of ASC control.c @ 0eed51b. */
#ifndef QCAR_ASC_CONTROL_H
#define QCAR_ASC_CONTROL_H
#include "asc_pid.h"
typedef struct {
 float roll_zero,pitch_zero;
 float recovery_kp,recovery_ki,recovery_kd;
 float roll_angle_kp,roll_angle_ki,roll_angle_kd;
 float roll_rate_kp,roll_rate_ki,roll_rate_kd;
 float pitch_speed_kp,pitch_speed_ki,pitch_speed_kd;
 float pitch_angle_kp,pitch_angle_ki,pitch_angle_kd;
 float pitch_rate_kp,pitch_rate_ki,pitch_rate_kd;
 float yaw_angle_kp,yaw_angle_ki,yaw_angle_kd;
 float yaw_rate_kp,yaw_rate_ki,yaw_rate_kd;
 float fly_limit,ground_limit,roll_target_limit,pitch_tilt_limit;
 float ground_sign,roll_sign;
 int yaw_enabled;
} asc_config_t;
typedef struct {
 float roll,pitch,yaw,roll_rate,pitch_rate,yaw_rate;
 float left_rpm,right_rpm,ground_counts_20ms;
 int speed_fresh;
} asc_input_t;
typedef struct {
 float roll_target,pitch_target,roll_target_rate,pitch_target_rate;
 float roll_rate_error,pitch_rate_error,pitch_feedback_rate,recovery,ground_tilt;
 float roll_pwm,pitch_pwm,yaw_pwm,left_pwm,right_pwm,ground_pwm;
} asc_output_t;
typedef struct {
 asc_pid_t recovery,roll_angle,roll_rate,pitch_speed,pitch_angle,pitch_rate,yaw_angle,yaw_rate;
 asc_config_t last_config;
 unsigned ticks;
 int initialized,have_config;
 float yaw_target;
 asc_output_t output;
} asc_controller_t;
void asc_control_reset(asc_controller_t *s);
/* Rotate measured legacy gravity/zero orientation, not just Euler names. */
void asc_reference_zero(float legacy_roll,float legacy_pitch,float *roll,float *pitch);
/* Exactly 1ms, attitude and raw bias-corrected gyro from the same sample.
 * Speed/recovery loops run every 20ms; angle PID values use a 5ms reference.
 * 0 on invalid input/config, resets all dynamic state and zeroes output. */
int asc_control_update(asc_controller_t *s,const asc_config_t *p,const asc_input_t *in,asc_output_t *out);
#endif

#include "balance_control.h"
volatile float imu_asc_roll_deg,imu_asc_pitch_deg,imu_asc_yaw_deg;
#ifndef ASC_TEST_REAL_DRIVER
volatile uint8 small_driver_feedback_fresh=1;
volatile uint32 small_driver_feedback_age_ms,small_driver_speed_frames;
#endif
void asc_test_legacy_defaults(void){
 roll_rate_test_kp=FLY_RATE_KP;roll_rate_test_ki=0;roll_rate_test_limit=FLY_PWM_LIMIT;roll_rate_test_sign=FLY_OUTPUT_SIGN;
 pitch_balance_kp=FLY_ANGLE_KP;pitch_balance_kd=0;pitch_balance_rate_limit=FLY_RATE_LIMIT_DPS;
 ground_rate_test_kp=GROUND_RATE_KP;ground_rate_test_kd=0;ground_rate_test_limit=GROUND_PWM_LIMIT;ground_rate_test_sign=GROUND_OUTPUT_SIGN;
 ground_angle_kp=GROUND_ANGLE_KP;ground_angle_rate_limit=GROUND_RATE_LIMIT_DPS;
}

/* Prevent Windows assertion dialogs from holding failed host fixtures open. */
#include <signal.h>
#include <stdlib.h>
static void host_abort(int n){(void)n;_Exit(134);}
#if defined(__GNUC__)
__attribute__((constructor)) static void host_assert_setup(void){signal(SIGABRT,host_abort);}
#endif

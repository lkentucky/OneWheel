"""Execute the yaw controller for Path, Balance and camera Run."""
from test_lean_limits import ROOT, _compile_run, _function


def test_real_path_yaw_limits_stationary_damping_and_mode_isolation(tmp_path):
    control = (ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    source = r'''
#include "config.h"
#include "Common/param.h"
#include "Common/pid.c"
#include "Controller/Planner/path_nav.h"
#include <assert.h>
#include <math.h>
static pid_t y_angle_pid, y_rate_pid;
static int s_path_active=1, moving=1;
static int s_run_active, s_remote_active, s_odom_test_state, start_flag;
static uint8 s_path_teach_remote;
static float s_remote_yaw_rate_cmd, s_remote_steer_angle;
enum { START_BALANCE=2, ODOM_TEST_IDLE=0, TRACK_MODE_HOLD=3 };
static float speed, yaw, g_yaw_target, g_pwm_roll, s_path_yaw_rate_cmd;
static float s_path_yaw_rate_target;
static float s_direction_yaw_rate_cmd;
static uint8 s_direction_last_mode;
static float g_yaw_momentum_scale=1;
static int16 rpm_a, rpm_b;
static uint8 motor_link_lost;
param_t g_param;
static struct { float gyro_z; } imu;
static float Y_Motor_GetSpeedMps(void) { return speed; }
static float imu_get_angle_yaw(void) { return yaw; }
static int16 W_Motor_GetSpeed1(void) { return rpm_a; }
static int16 W_Motor_GetSpeed2(void) { return rpm_b; }
static uint8 W_Motor_LinkLost(void) { return motor_link_lost; }
int path_nav_running(void) { return moving; }
static int ctrl_is_finite(float v) { return isfinite(v); }
''' + _function(control, 'yaw_cascade_ctrl') + r'''
int main(void) {
    int i;
    pid_set(&y_angle_pid,18,.01f,0,100);
    pid_set(&y_rate_pid,-56,0,0,100);
    g_yaw_target=20;
    for(i=0;i<1000;i++) assert(yaw_cascade_ctrl(1)==0);
    assert(y_angle_pid.integrator==0 && s_path_yaw_rate_cmd==0);
    imu.gyro_z=5; assert(yaw_cascade_ctrl(1)==280); /* Damping remains. */
    imu.gyro_z=0; speed=.2f;
    for(i=0;i<1000;i++) {
        float before=s_path_yaw_rate_cmd;
        yaw_cascade_ctrl(1);
        assert(fabsf(s_path_yaw_rate_cmd-before)<=.1801f);
        assert(fabsf(s_path_yaw_rate_cmd)<=45);
    }
    assert(fabsf(s_path_yaw_rate_cmd-45)<.001f);
    g_yaw_momentum_scale=.25f; yaw_cascade_ctrl(0);
    assert(s_path_yaw_rate_cmd<=11.25f); /* Hard new permission, even between angle ticks. */
    g_pwm_roll=FLYWHEEL_OUT_LIMIT-100; assert(fabsf(yaw_cascade_ctrl(1))<=100);
    g_pwm_roll=FLYWHEEL_OUT_LIMIT; assert(yaw_cascade_ctrl(1)==0);
    g_pwm_roll=0; speed=0; assert(yaw_cascade_ctrl(1)==0);
    assert(s_path_yaw_rate_cmd==0 && y_angle_pid.integrator==0);
    speed=.2f; g_yaw_target=-20;
    for(i=0;i<1000;i++) yaw_cascade_ctrl(1);
    assert(s_path_yaw_rate_cmd<0 && y_rate_pid.out>0);
    moving=0; speed=0; g_yaw_target=1; /* DONE keeps bounded balanced heading hold. */
    for(i=0;i<1000;i++) yaw_cascade_ctrl(1);
    assert(s_path_yaw_rate_cmd>0);
    s_path_active=0; pid_set(&y_angle_pid,18,0,0,100); pid_set(&y_rate_pid,-56,0,0,100);
    g_yaw_target=1; assert(yaw_cascade_ctrl(1)==-1008); /* Other modes unchanged. */
    start_flag=START_BALANCE;
    pid_set(&y_angle_pid,24,0,0,250);
    pid_set(&y_rate_pid,60,0,0,100);
    g_yaw_target=2;
    assert(yaw_cascade_ctrl(1)==2880);
    g_yaw_target=5; g_pwm_roll=FLYWHEEL_OUT_LIMIT-200;
    assert(yaw_cascade_ctrl(1)==7200); /* Raw command, limited later by the mixer. */
    g_yaw_target=-5;
    assert(yaw_cascade_ctrl(1)==-7200);
    imu.gyro_z=-10;
    assert(yaw_cascade_ctrl(1)==-6600);
    imu.gyro_z=0;
    rpm_a=-1000; rpm_b=-1000;
    assert(yaw_cascade_ctrl(1)==-8050);
    g_yaw_target=5;
    assert(yaw_cascade_ctrl(1)==7200); /* No boost while braking. */
    rpm_a=1000; rpm_b=1000;
    assert(yaw_cascade_ctrl(1)==8050);
    g_yaw_target=0;
    assert(yaw_cascade_ctrl(1)==0); /* No self-driven motor output. */
    g_yaw_target=0.1f;
    assert(fabsf(yaw_cascade_ctrl(1)-216)<.01f);
    g_yaw_target=5; rpm_a=2200; rpm_b=2200;
    assert(yaw_cascade_ctrl(1)==8400); /* Absolute cap on compensation. */
    g_param.fly_speed_limit=2800;
    assert(yaw_cascade_ctrl(1)==7200); /* Configured overspeed margin. */
    g_param.fly_speed_limit=8000; rpm_a=4500; rpm_b=-500;
    assert(yaw_cascade_ctrl(1)==7200); /* Individual wheel limits boost. */
    rpm_a=1000; rpm_b=1000; motor_link_lost=1;
    assert(yaw_cascade_ctrl(1)==7200);
    motor_link_lost=0; g_yaw_target=-5; rpm_a=-1000; rpm_b=-1000;
    g_pwm_roll=0; s_remote_active=1; s_path_teach_remote=1;
    assert(yaw_cascade_ctrl(1)==-3600);

    s_path_teach_remote=0; s_remote_steer_angle=10; s_remote_yaw_rate_cmd=-10;
    imu.gyro_z=-4;
    assert(yaw_cascade_ctrl(1)==-360 && y_angle_pid.last_error==-5);
    s_remote_steer_angle=-10; s_remote_yaw_rate_cmd=10;
    imu.gyro_z=4;
    assert(yaw_cascade_ctrl(1)==360);
    s_remote_steer_angle=0; s_remote_yaw_rate_cmd=0; y_rate_pid.integrator=90;
    assert(yaw_cascade_ctrl(1)==-240 && y_rate_pid.integrator==0);

    s_remote_active=0; s_run_active=1;
    g_yaw_target=345; yaw=-103; imu.gyro_z=4;
    s_direction_yaw_rate_cmd=10;
    assert(yaw_cascade_ctrl(0)==360);
    assert(yaw_cascade_ctrl(1)==360 && y_angle_pid.last_error==-5);
    s_direction_yaw_rate_cmd=-10; imu.gyro_z=-4;
    assert(yaw_cascade_ctrl(1)==-360);
    g_pwm_roll=FLYWHEEL_OUT_LIMIT-80;
    assert(yaw_cascade_ctrl(1)==-80);
    g_pwm_roll=0; imu.gyro_z=0; s_direction_yaw_rate_cmd=120;
    assert(yaw_cascade_ctrl(1)==YAW_PWM_LIMIT);
    y_rate_pid.ki=2; y_rate_pid.integrator=90;
    s_direction_yaw_rate_cmd=0; s_direction_last_mode=TRACK_MODE_HOLD;
    assert(yaw_cascade_ctrl(1)==0 && y_rate_pid.integrator==0);
    y_rate_pid.ki=0; imu.gyro_z=5;
    assert(yaw_cascade_ctrl(1)==-300);
    return 0;
}
'''
    _compile_run(tmp_path, source)


def test_remote_signed_speed_rate_targets_release_and_timeout(tmp_path):
    control = (ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    source = r'''
#include "config.h"
#include "Common/param.h"
#include "Common/pid.c"
#include "Controller/Planner/path_nav.h"
#include <assert.h>
#include <math.h>
enum { START_STOP, START_BALANCE=2, ODOM_TEST_IDLE=0, ODOM_TEST_RUNNING, ODOM_TEST_DONE };
static uint8 start_flag, s_run_active, s_path_active, s_path_teach_remote;
static uint8 s_remote_active, s_remote_cmd_seen, s_remote_timeout_latched;
static uint16 s_remote_cmd_age_ms;
static float s_remote_steer_angle, s_remote_speed_mps, s_remote_yaw_target, s_remote_yaw_rate_cmd;
static float g_yaw_target, s_balance_yaw_ref, s_yaw_desat_offset;
static float s_odom_test_yaw_zero, s_speed_ramp, g_yaw_momentum_scale=1;
static int s_odom_test_state, s_odom_test_start_count, g_target_distance;
static struct { float speed_mps, yaw_target_deg; } s_path_command;
static pid_t p_vel_pid, y_rate_pid;
param_t g_param;
static float yaw=31;
static float imu_get_angle_yaw(void){return yaw;}
static int ctrl_is_finite(float x){return isfinite(x);}
static float Y_Motor_MpsToCount20ms(float v){return 200*v;}
static int Y_Motor_GetTotalCount(void){return 0;}
static float Y_Motor_CountToMeter(int x){return (float)x;}
static void run_target_update(void){}
'''
    for name in ('speed_target_from_mps', 'remote_yaw_rate_target',
                 'motion_target_update', 'control_remote_command'):
        source += _function(control, name) + '\n'
    source += r'''
int main(void){
    g_param.steer_dir=1;
    assert(!control_remote_command(0,.15f));
    start_flag=START_BALANCE; s_remote_active=1;
    assert(control_remote_command(0,.15f)); motion_target_update();
    assert(g_target_distance==30 && s_remote_yaw_rate_cmd==0);
    assert(control_remote_command(10,-.15f)); motion_target_update();
    assert(g_target_distance==-30 && fabsf(s_remote_yaw_rate_cmd-.6f)<.001f);
    for(int i=0;i<30;i++)motion_target_update();
    assert(s_remote_yaw_rate_cmd==10);
    assert(control_remote_command(-10,0));
    for(int i=0;i<40;i++)motion_target_update();
    assert(g_target_distance==0 && s_remote_yaw_rate_cmd==-10);
    assert(control_remote_command(0,0)); motion_target_update();
    assert(g_target_distance==0 && s_remote_yaw_rate_cmd==0);

    assert(control_remote_command(10,.15f)); motion_target_update();
    s_speed_ramp=30; y_rate_pid.integrator=20;
    s_remote_cmd_age_ms=REMOTE_CMD_TIMEOUT_MS;
    motion_target_update();
    assert(s_remote_timeout_latched && s_remote_yaw_rate_cmd==0 && s_remote_steer_angle==0);
    assert(s_speed_ramp==0 && g_target_distance==0 && y_rate_pid.integrator==0);
    assert(control_remote_command(-10,.15f)); motion_target_update();
    assert(!s_remote_timeout_latched && g_target_distance==30 && s_remote_yaw_rate_cmd<0);

    s_path_teach_remote=1;
    assert(!control_remote_command(10,-.15f));
    assert(control_remote_command(10,.15f)); motion_target_update();
    assert(g_yaw_target==21 && g_target_distance==30);
    return 0;
}
'''
    _compile_run(tmp_path, source)

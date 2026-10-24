"""Exercise the real isolated rate loop and its fault/stop transitions."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
s = (root / "code/pid.c").read_text(encoding="utf-8")
part = s[s.index("/* States: 0 off"):s.index("void cascade_ctrl(void)")]
assert "if (roll_rate_test_enabled) { roll_rate_test_update(); return; }" in s
prefix = r"""
#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
typedef uint8_t uint8; typedef uint32_t uint32; typedef int32_t int32; typedef int16_t int16;
#define IMU_ATTITUDE_CALIBRATING 0
#define IMU_ATTITUDE_RUNNING 1
#define left 1
#define right 2
static struct { struct { float x, y; } gyro; } attitude_state;
static struct { float balance_angle_x, balance_angle_y; } mono_params;
static float imu_roll_deg, imu_pitch_deg, cascade_angle_out, cascade_angle_out_p, cascade_angle_vel_out, cascade_angle_vel_out_p;
static uint32 imu_sample_sequence;
static int32 imu_attitude_status;
static float imu_sample_dt_ms=5;
static int shacheflag;
static int16 costrained_flywheel_speed_left, costrained_flywheel_speed_right, costrained_wheel_speed;
static int last_a,last_b,last_ground,writes;
static uint32 interrupt_global_disable(void) { return 0; }
static void interrupt_global_enable(uint32 irq) { (void)irq; }
static void small_driver_set_duty(int a,int b) { last_a=a; last_b=b; writes++; }
static void motor_m_ctrl(int duty) { last_ground=duty; }
static float constrain_float(float v,float lo,float hi) { return v<lo?lo:v>hi?hi:v; }
"""
main = r"""
static void step(void) { for (int i=0;i<5;i++) roll_rate_test_update(); assert(last_ground==0); }
static void arm(void) {
    control_startup_ramp_ms=0;
    ground_angle_kp=0;ground_angle_rate_limit=10;mono_params.balance_angle_x=0;
    pitch_balance_kp=0; pitch_balance_rate_limit=20; mono_params.balance_angle_y=0;
    roll_rate_test_kp=5; roll_rate_test_limit=1000; roll_rate_test_sign=1;
    imu_roll_deg=imu_pitch_deg=0; imu_attitude_status=1; shacheflag=1;
    imu_sample_sequence++; roll_rate_test_arm();
}
int main(void) {
    attitude_state.gyro.x=123; /* Other axis must not drive the flywheel. */
    arm(); imu_attitude_status=0; attitude_state.gyro.y=50; step();
    assert(roll_rate_test_state==1 && !last_a && !last_b);
    imu_attitude_status=1; imu_sample_sequence++; attitude_state.gyro.y=10; step();
    assert(roll_rate_test_state==2 && last_a==-50 && last_b==50 && cascade_angle_out==0);
    attitude_state.gyro.y=0; imu_sample_sequence++; step(); assert(!last_a && !last_b);
    attitude_state.gyro.y=-1000; imu_sample_sequence++; step(); assert(last_a==1000 && last_b==-1000);
    roll_rate_test_sign=-1; imu_sample_sequence++; step(); assert(last_a==-1000 && last_b==1000);
    /* Internal roll is now the displayed, noncontrolled Pitch axis. */
    arm(); imu_roll_deg=-60; imu_sample_sequence++; step(); assert(roll_rate_test_state==2);
    imu_roll_deg=60; imu_sample_sequence++; step(); assert(roll_rate_test_state==2);
    imu_pitch_deg=-31; imu_sample_sequence++; step(); assert(roll_rate_test_state==-2 && !last_a && !last_b);
    imu_pitch_deg=0; imu_sample_sequence++; step(); assert(roll_rate_test_state==-2 && !last_a);
    arm(); imu_pitch_deg=31; step(); assert(roll_rate_test_state==-2 && !last_a);
    arm(); imu_attitude_status=-2; step(); assert(roll_rate_test_state==-1 && !last_a);
    arm(); for(int i=0;i<5;i++) step(); assert(roll_rate_test_state==-1 && !last_a);
    arm(); attitude_state.gyro.y=2; roll_rate_test_kp=150; step();
    assert(roll_rate_test_state==2 && last_a==-300 && last_b==300);
    arm(); attitude_state.gyro.y=3; roll_rate_test_kp=500; step();
    assert(roll_rate_test_state==2 && last_a==-1000 && last_b==1000);
    arm(); roll_rate_test_kp=501; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); roll_rate_test_kp=NAN; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); roll_rate_test_sign=0; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); attitude_state.gyro.y=NAN; step(); assert(roll_rate_test_state==-1 && !last_a);
    arm(); attitude_state.gyro.y=5;
    for(int i=0;i<12000;i++) { imu_sample_sequence++; step(); }
    assert(roll_rate_test_state==2 && last_a==-25 && last_b==25);
    imu_sample_sequence++; step(); assert(roll_rate_test_state==2 && last_a==-25);
    arm(); shacheflag=0; step(); assert(roll_rate_test_state==0 && !last_a);
    arm(); roll_rate_test_stop(); assert(!roll_rate_test_enabled && !roll_rate_test_pwm);
    writes=0; for(int i=0;i<4;i++) roll_rate_test_update(); assert(writes==0);
    roll_rate_test_update(); assert(writes==1 && !last_a);
    arm(); pitch_balance_kp=0.5f; mono_params.balance_angle_y=2;
    imu_pitch_deg=6; attitude_state.gyro.y=0; roll_rate_test_kp=50; roll_rate_test_sign=-1;
    imu_sample_sequence++; step();
    assert(pitch_balance_error==-4 && pitch_balance_target_rate==-2 && last_a==100);
    imu_pitch_deg=-6; imu_sample_sequence++; step();
    assert(pitch_balance_target_rate==4 && last_a==-200);
    pitch_balance_kp=50; imu_sample_sequence++; step();
    assert(pitch_balance_target_rate==20 && last_a==-1000);
    pitch_balance_rate_limit=0; step(); assert(roll_rate_test_state==-3 && !last_a && !pitch_balance_target_rate);
    arm(); attitude_state.gyro.y=-100; roll_rate_test_kp=50; roll_rate_test_limit=3000;
    pitch_balance_kp=100; imu_sample_sequence++; step();
    assert(roll_rate_test_state==2 && last_a==3000 && last_b==-3000);
    arm(); pitch_balance_kp=101; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); roll_rate_test_limit=4001; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); mono_params.balance_angle_y=-0.3f; pitch_balance_kp=2;
    roll_rate_test_kp=50; roll_rate_test_sign=-1; attitude_state.gyro.y=0;
    imu_pitch_deg=1.7f; imu_sample_sequence++; step();
    assert(fabsf(pitch_balance_error+2)<1e-5f && fabsf(pitch_balance_target_rate+4)<1e-5f && last_a==200);
    imu_pitch_deg=-2.3f; imu_sample_sequence++; step();
    assert(fabsf(pitch_balance_error-2)<1e-5f && fabsf(pitch_balance_target_rate-4)<1e-5f && last_a==-200);
    arm(); mono_params.balance_angle_y=-0.3f; pitch_balance_kp=36.5f;
    pitch_balance_rate_limit=40;roll_rate_test_kp=25;roll_rate_test_limit=3000;
    attitude_state.gyro.y=0;imu_pitch_deg=1.7f;imu_sample_sequence++;step();
    assert(pitch_balance_target_rate==-40 && last_a==-1000);
    assert(pitch_balance_actual_angle==1.7f && pitch_balance_target_angle==-0.3f);
    imu_pitch_deg=0.2f;imu_sample_sequence++;step();
    assert(fabsf(pitch_balance_target_rate+18.25f)<1e-4f && last_a==-456);
    arm(); mono_params.balance_angle_y=-0.3f;pitch_balance_kp=36.5f;
    pitch_balance_rate_limit=80;roll_rate_test_kp=25;roll_rate_test_limit=3000;
    attitude_state.gyro.y=0;imu_pitch_deg=1.7f;imu_sample_sequence++;step();
    assert(fabsf(pitch_balance_target_rate+73)<1e-4f && last_a==-1825);
    imu_pitch_deg=2.7f;imu_sample_sequence++;step();
    assert(pitch_balance_target_rate==-80 && last_a==-2000);
    arm(); mono_params.balance_angle_y=-0.3f;pitch_balance_kp=36.5f;
    pitch_balance_rate_limit=80;roll_rate_test_kp=35;roll_rate_test_limit=3000;
    attitude_state.gyro.y=0;imu_pitch_deg=2.7f;imu_sample_sequence++;step();
    assert(pitch_balance_target_rate==-80 && last_a==-2800);
    attitude_state.gyro.y=10;imu_sample_sequence++;step();assert(last_a==-3000);
    arm();roll_rate_test_kp=35;roll_rate_test_limit=4000;
    attitude_state.gyro.y=-200;imu_sample_sequence++;step();
    assert(roll_rate_test_state==2 && last_a==4000 && last_b==-4000);
    arm(); pitch_balance_kp=NAN; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); mono_params.balance_angle_y=NAN; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); pitch_balance_rate_limit=INFINITY; step(); assert(roll_rate_test_state==-3 && !last_a);
    roll_rate_test_stop(); assert(!pitch_balance_target_rate && !pitch_balance_error);
    arm();ground_rate_test_arm();ground_rate_test_kp=25;ground_rate_test_limit=1000;ground_rate_test_sign=1;
    pitch_balance_kp=50;imu_roll_deg=0;imu_pitch_deg=50;
    attitude_state.gyro.x=10;attitude_state.gyro.y=99;imu_sample_sequence++;
    for(int j=0;j<5;j++)roll_rate_test_update();
    assert(roll_rate_test_state==2 && pitch_balance_target_rate==0 && roll_rate_test_actual==10);
    assert(last_ground==-250 && !last_a && !last_b && costrained_wheel_speed==-250);
    attitude_state.gyro.x=-100;imu_sample_sequence++;
    for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(last_ground==1000);
    ground_rate_test_sign=-1;imu_sample_sequence++;
    for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(last_ground==-1000);
    imu_roll_deg=31;imu_sample_sequence++;
    for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(roll_rate_test_state==-2 && !last_ground && !last_a && !last_b);
    roll_rate_test_stop();assert(!ground_rate_test_enabled && !roll_rate_test_enabled);
    arm();ground_rate_test_arm();ground_rate_test_kp=100;ground_rate_test_limit=2000;ground_rate_test_sign=1;
    attitude_state.gyro.x=5;imu_sample_sequence++;
    for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(last_ground==-500 && !last_a && !last_b);
    attitude_state.gyro.x=30;imu_sample_sequence++;
    for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(last_ground==-2000 && !last_a && !last_b);
    roll_rate_test_stop();
    arm();ground_rate_test_arm();ground_angle_kp=1;ground_angle_rate_limit=10;
    ground_rate_test_kp=100;ground_rate_test_limit=2000;ground_rate_test_sign=1;
    mono_params.balance_angle_x=.9f;imu_roll_deg=3.9f;attitude_state.gyro.x=.5f;
    imu_sample_sequence++;for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(fabsf(pitch_balance_error+3)<1e-5f && fabsf(pitch_balance_target_rate+3)<1e-5f);
    assert(last_ground==-350 && !last_a && !last_b);
    imu_roll_deg=-15;attitude_state.gyro.x=0;imu_sample_sequence++;
    for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(pitch_balance_target_rate==10 && last_ground==1000 && !last_a && !last_b);
    arm();dual_balance_arm();
    pitch_balance_kp=2;pitch_balance_rate_limit=20;roll_rate_test_kp=10;roll_rate_test_limit=500;roll_rate_test_sign=1;
    ground_angle_kp=3;ground_angle_rate_limit=10;ground_rate_test_kp=100;ground_rate_test_limit=1000;ground_rate_test_sign=1;
    imu_pitch_deg=1;imu_roll_deg=2;attitude_state.gyro.x=attitude_state.gyro.y=0;
    imu_sample_sequence++;for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(roll_rate_test_state==2 && last_a==-20 && last_b==20 && last_ground==-600);
    assert(dual_target_rate[0]==-2 && dual_target_rate[1]==-6);
    imu_roll_deg=-2;imu_sample_sequence++;for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(last_a==-20 && last_ground==600);
    imu_pitch_deg=31;imu_sample_sequence++;for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(roll_rate_test_state==-2 && !last_a && !last_b && !last_ground);
    imu_pitch_deg=0;imu_sample_sequence++;for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(roll_rate_test_state==-2 && !last_ground);
    roll_rate_test_stop();assert(!dual_balance_enabled && !ground_rate_test_enabled);
    arm();dual_balance_arm();imu_attitude_status=0;for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(roll_rate_test_state==1 && !last_a && !last_ground);
    imu_attitude_status=1;for(int j=0;j<25;j++){roll_rate_test_update();}
    assert(roll_rate_test_state==-1 && !last_a && !last_ground);
    arm();dual_balance_arm();ground_rate_test_sign=0;imu_sample_sequence++;
    for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(roll_rate_test_state==-3 && !last_a && !last_ground);
    arm();ground_rate_only_arm();ground_angle_kp=100;ground_rate_test_kp=100;
    ground_rate_test_limit=4000;ground_rate_test_sign=1;attitude_state.gyro.x=-100;
    control_startup_ramp_ms=1000;
    imu_sample_sequence++;for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(last_ground==20 && pitch_balance_target_rate==0);
    for(int n=1;n<200;n++){imu_sample_sequence++;for(int j=0;j<5;j++){roll_rate_test_update();}}
    assert(last_ground==4000 && control_startup_scale==1);
    imu_attitude_status=-3;imu_sample_dt_ms=65;
    for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(roll_rate_test_state==-1 && control_fault_reason==1 && control_fault_imu_status==-3 && control_fault_dt_ms==65 && !last_ground);
    arm();roll_rate_only_arm();pitch_balance_kp=100;attitude_state.gyro.y=0;
    imu_sample_sequence++;for(int j=0;j<5;j++){roll_rate_test_update();}
    assert(pitch_balance_target_rate==0 && !last_a && !last_ground);
    for(int n=0;n<5;n++){for(int j=0;j<5;j++){roll_rate_test_update();}}
    assert(roll_rate_test_state==-1 && control_fault_reason==4);
    puts("PASS: P feedback/sign, clamp, no integral residual, calibration zero, both tilt signs, IMU faults/stale, latched faults, continuous 60s run and manual stop, 5ms cadence");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-rate-") as folder:
    folder=Path(folder); src=folder/"test.c"; exe=folder/"test.exe"
    src.write_text(prefix+part+main,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-lm","-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)

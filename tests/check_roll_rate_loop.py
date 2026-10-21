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
static struct { struct { float x; } gyro; } mahony_filter;
static float imu_roll_deg, imu_pitch_deg, cascade_angle_out, cascade_angle_vel_out;
static uint32 imu_sample_sequence;
static int32 imu_attitude_status;
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
    roll_rate_test_kp=5; roll_rate_test_limit=1000; roll_rate_test_sign=1;
    imu_roll_deg=imu_pitch_deg=0; imu_attitude_status=1; shacheflag=1;
    imu_sample_sequence++; roll_rate_test_arm();
}
int main(void) {
    arm(); imu_attitude_status=0; mahony_filter.gyro.x=50; step();
    assert(roll_rate_test_state==1 && !last_a && !last_b);
    imu_attitude_status=1; imu_sample_sequence++; mahony_filter.gyro.x=10; step();
    assert(roll_rate_test_state==2 && last_a==-50 && last_b==50 && cascade_angle_out==0);
    mahony_filter.gyro.x=0; imu_sample_sequence++; step(); assert(!last_a && !last_b);
    mahony_filter.gyro.x=-1000; imu_sample_sequence++; step(); assert(last_a==1000 && last_b==-1000);
    roll_rate_test_sign=-1; imu_sample_sequence++; step(); assert(last_a==-1000 && last_b==1000);
    imu_roll_deg=-31; step(); assert(roll_rate_test_state==-2 && !last_a && !last_b);
    imu_roll_deg=0; imu_sample_sequence++; step(); assert(roll_rate_test_state==-2 && !last_a);
    arm(); imu_pitch_deg=31; step(); assert(roll_rate_test_state==-2 && !last_a);
    arm(); imu_attitude_status=-2; step(); assert(roll_rate_test_state==-1 && !last_a);
    arm(); for(int i=0;i<5;i++) step(); assert(roll_rate_test_state==-1 && !last_a);
    arm(); roll_rate_test_kp=NAN; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); roll_rate_test_sign=0; step(); assert(roll_rate_test_state==-3 && !last_a);
    arm(); mahony_filter.gyro.x=NAN; step(); assert(roll_rate_test_state==-1 && !last_a);
    arm(); mahony_filter.gyro.x=5;
    for(int i=0;i<12000;i++) { imu_sample_sequence++; step(); }
    assert(roll_rate_test_state==2 && last_a==-25 && last_b==25);
    imu_sample_sequence++; step(); assert(roll_rate_test_state==2 && last_a==-25);
    arm(); shacheflag=0; step(); assert(roll_rate_test_state==0 && !last_a);
    arm(); roll_rate_test_stop(); assert(!roll_rate_test_enabled && !roll_rate_test_pwm);
    writes=0; for(int i=0;i<4;i++) roll_rate_test_update(); assert(writes==0);
    roll_rate_test_update(); assert(writes==1 && !last_a);
    puts("PASS: P feedback/sign, clamp, no integral residual, calibration zero, both tilt signs, IMU faults/stale, latched faults, continuous 60s run and manual stop, 5ms cadence");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-rate-") as folder:
    folder=Path(folder); src=folder/"test.c"; exe=folder/"test.exe"
    src.write_text(prefix+part+main,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-lm","-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)

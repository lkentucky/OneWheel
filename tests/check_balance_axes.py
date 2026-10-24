"""Exercise production cascade wiring with independent X/Y sensor inputs."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
s = (root / "code/pid.c").read_text(encoding="utf-8")
control = s[s.index("void cascade_ctrl(void)"):s.index("void clear_all_cascade_out(void)")]
lines = [line.split("//")[0] for line in control.splitlines() if any(key in line for key in (
    "cascade_vel_out = cascade_velocity", "cascade_vel_out_p = cascade_velocity",
    "cascade_angle_out = cascade_angle", "cascade_angle_out_p = cascade_angle",
    "cascade_angle_vel_out   = increment", "cascade_angle_vel_out_p = increment"))]
assert len(lines) == 6
prefix = r"""
#include <assert.h>
#include <stdio.h>
typedef struct { float kp; } pid;
static pid cascade_vel_pitch={2},cascade_vel_roll={3};
static pid cascade_ang_pitch={4},cascade_ang_roll={5};
static pid cascade_ang_vel_pitch={6},cascade_ang_vel_roll={7};
static struct { float setspeed,balance_angle_x,balance_angle_y; } mono_params={8,1,2};
static struct { float dynamic_zero_roll,dynamic_zero_pitch; } mono_beacon={0.5f,0.25f};
static struct { float pitch,roll; struct {float x,y;} gyro;} attitude_state={3,4,{5,6}};
static float real_speed=10,real_mid_speed=20;
static float cascade_vel_out,cascade_vel_out_p,cascade_angle_out,cascade_angle_out_p;
static float cascade_angle_vel_out,cascade_angle_vel_out_p;
static float cascade_velocity(pid *p,float target,float actual) {return p->kp*(target-actual);}
static float cascade_angle(pid *p,float target,float actual,float gyro) {
    if(p==&cascade_ang_pitch) {assert(actual==3 && gyro==-6);}
    else {assert(p==&cascade_ang_roll && actual==4 && gyro==-5);}
    return p->kp*(target-actual);
}
static float increment_cascade_angle_vel(pid *p,float target,float actual) {
    if(p==&cascade_ang_vel_pitch) assert(actual==6);
    else assert(p==&cascade_ang_vel_roll && actual==5);
    return p->kp*(target-actual);
}
int main(void) {
"""
suffix = r"""
assert(cascade_vel_out==-20);
assert(cascade_vel_out_p>23.95f && cascade_vel_out_p<23.96f);
assert(cascade_angle_out==74);
assert(cascade_angle_vel_out==408);
assert(cascade_angle_out_p < -135 && cascade_angle_out_p > -137);
assert(cascade_angle_vel_out_p < -980 && cascade_angle_vel_out_p > -1000);
puts("PASS: flywheel Pitch/Y and ground-wheel Roll/X speed, offset, angle and rate wiring");
}
"""
with tempfile.TemporaryDirectory(prefix="qcar-axes-") as folder:
    src=Path(folder)/"axes.c"; exe=Path(folder)/"axes.exe"
    src.write_text(prefix+"\n".join(lines)+suffix,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

"""Run real C: direct angle feedback, mode consistency and braking timing.

The trajectory is prescribed sensor input, not a mechanical-plant simulation.
"""
from pathlib import Path
import ast
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
tree = ast.parse((root / 'tests/check_balance_control.py').read_text(encoding='utf-8-sig'))
texts = [n.args[0].value for n in ast.walk(tree)
         if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
         and n.func.attr == 'write_text' and n.args
         and isinstance(n.args[0], ast.Constant)]
boundary = next(s for s in texts if '#ifndef HOST_BOUNDARY' in s)
fixture = next(s for s in texts if 'int main(void)' in s).split('int main(void)')[0]

test = r'''
static void ground_input(int k) {
    imu_roll_deg=BALANCE_ZERO_X_DEG+((k%19)-9)*0.2f;
    attitude_state.gyro.x=((k%7)-3)*0.5f;
}
static void fly_input(int k) {
    imu_pitch_deg=BALANCE_ZERO_Y_DEG+((k%17)-8)*0.15f;
    attitude_state.gyro.y=((k%5)-2)*0.7f;
}
static int zero_crossing(int mode) {
    arm(mode); /* Disable the ramp to isolate the running controller. */
    imu_pitch_deg=BALANCE_ZERO_Y_DEG-5.0f;
    for(int k=0;k<100;k++)tick(1);
    int steps=(int)roundf(5.0f/(20.0f*APP_IMU_PERIOD_US*0.000001f));
    for(int k=1;k<=steps;k++) {
        imu_pitch_deg=BALANCE_ZERO_Y_DEG-5.0f+5.0f*(float)k/(float)steps;
        attitude_state.gyro.y=20.0f;
        tick(1);
        assert(roll_rate_test_state==2 && control_fault_reason==0);
    }
    assert(fabsf(imu_pitch_deg-mono_params.balance_angle_y)<0.0001f);
    return fly_l;
}
int main(void) {
    int fly_reference[60],ground_reference[60];
    /* Single-axis and dual-axis paths must consume the same current angles. */
    arm(3);
    for(int k=0;k<60;k++){fly_input(k);tick(1);fly_reference[k]=fly_l;}
    arm(4);
    for(int k=0;k<60;k++){ground_input(k);tick(1);ground_reference[k]=ground;}
    arm(5);
    for(int k=0;k<60;k++) {
        fly_input(k);ground_input(k);tick(1);
        assert(fly_l==fly_reference[k] && fly_r==-fly_l);
        assert(ground==ground_reference[k]);
    }
    /* A one-degree step must be visible in full on the first control update. */
    arm(5);imu_pitch_deg+=1.0f;imu_roll_deg+=1.0f;tick(1);
    assert(fabsf(dual_angle_error[0]+1.0f)<0.0001f);
    assert(fabsf(dual_angle_error[1]+1.0f)<0.0001f);
    /* Rearm must use current attitude, never old feedback history. */
    arm(3);imu_pitch_deg=mono_params.balance_angle_y-2.0f;
    roll_rate_test_arm();tick(1);assert(fabsf(pitch_balance_error-2.0f)<0.0001f);
    imu_pitch_deg=mono_params.balance_angle_y+2.0f;
    roll_rate_test_arm();tick(1);assert(fabsf(pitch_balance_error+2.0f)<0.0001f);
    /* Same physical-angle crossing with 20deg/s: compare recorded C baselines. */
    int single=zero_crossing(3),dual=zero_crossing(5);
    assert(single==dual);
    assert(abs(single-(int)roundf(-FLY_RATE_KP*20.0f))<=1);
    /* Direct-angle protection still stops both axes immediately. */
    arm(5);imu_roll_deg=31.0f;tick(1);
    assert(roll_rate_test_state==-2 && !fly_l && !ground);
    printf("PASS: direct angle steps, Mode3/4/5 consistency, rearm, tilt; prescribed zero-crossing PWM=%d\n",single);
}
'''

with tempfile.TemporaryDirectory(prefix='qcar-angle-feedback-') as d:
    work = Path(d)
    (work / 'zf_common_headfile.h').write_text(boundary)
    (work / 'test.c').write_text(fixture + test)
    exe = work / 'test.exe'
    subprocess.run(['gcc', '-std=c99', '-I' + str(work), '-I' + str(root / 'code'),
                    str(root / 'code/balance_control.c'), str(root / 'code/wheel_hold.c'),
                    str(root / 'tests/encoder_snapshot_stub.c'), str(root / 'code/pid.c'),
                    str(work / 'test.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)

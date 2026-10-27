"""Run real C: independent angle filters, mode consistency and braking timing.

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
    for(int k=1;k<=50;k++) {
        imu_pitch_deg=BALANCE_ZERO_Y_DEG-5.0f+0.1f*(float)k;
        attitude_state.gyro.y=20.0f;
        tick(1);
        assert(roll_rate_test_state==2 && control_fault_reason==0);
    }
    assert(fabsf(imu_pitch_deg-mono_params.balance_angle_y)<0.0001f);
    return fly_l;
}
int main(void) {
    int fly_reference[60],ground_reference[60];
    /* Single-axis and dual-axis paths must select the same per-axis filter. */
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
    /* A one-degree step exposes axis swaps and independent bypass settings. */
    arm(5);imu_pitch_deg+=1.0f;imu_roll_deg+=1.0f;tick(1);
    float fly_fraction=0.005f/(FLY_ANGLE_LPF_TAU_S+0.005f);
    float ground_fraction=0.005f/(GROUND_ANGLE_LPF_TAU_S+0.005f);
    assert(fabsf(dual_angle_error[0]+fly_fraction)<0.0001f);
    assert(fabsf(dual_angle_error[1]+ground_fraction)<0.0001f);
    /* Arm must seed the filter from current attitude, never from zero/history. */
    arm(3);imu_pitch_deg=mono_params.balance_angle_y-2.0f;
    roll_rate_test_arm();tick(1);assert(fabsf(pitch_balance_error-2.0f)<0.0001f);
    imu_pitch_deg=mono_params.balance_angle_y+2.0f;
    roll_rate_test_arm();tick(1);assert(fabsf(pitch_balance_error+2.0f)<0.0001f);
    /* Same physical-angle crossing with 20deg/s: compare recorded C baselines. */
    int single=zero_crossing(3),dual=zero_crossing(5);
    assert(single==dual);
    if(FLY_ANGLE_LPF_TAU_S==0.010f) assert(abs(single-(-370))<=2);
    if(FLY_ANGLE_LPF_TAU_S==0.040f) assert(abs(single-1095)<=2);
    if(FLY_ANGLE_LPF_TAU_S==0.0f) assert(abs(single-(-860))<=2);
    /* The slow ground filter must not defer raw-angle protection. */
    arm(5);imu_roll_deg=31.0f;tick(1);
    assert(roll_rate_test_state==-2 && !fly_l && !ground);
    printf("PASS: angle tau fly=%.3fs ground=%.3fs; Mode3/4/5 consistency, independent step/bypass, arm seed, raw tilt; prescribed zero-crossing PWM=%d\n",
           FLY_ANGLE_LPF_TAU_S,GROUND_ANGLE_LPF_TAU_S,single);
}
'''

variants = (
    (),
    ('-DFLY_ANGLE_LPF_TAU_S=0.040f',),
    ('-DFLY_ANGLE_LPF_TAU_S=0.0f',),
    ('-DGROUND_ANGLE_LPF_TAU_S=0.0f',),
)
with tempfile.TemporaryDirectory(prefix='qcar-angle-filter-') as d:
    work = Path(d)
    (work / 'zf_common_headfile.h').write_text(boundary)
    (work / 'test.c').write_text(fixture + test)
    for index, definitions in enumerate(variants):
        exe = work / f'test_{index}.exe'
        subprocess.run(['gcc', '-std=c99', *definitions,
                        '-I' + str(work), '-I' + str(root / 'code'),
                        str(root / 'code/balance_control.c'), str(root / 'code/pid.c'),
                        str(work / 'test.c'), '-lm', '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)

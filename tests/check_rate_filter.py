"""Exercise flywheel feedback filtering and fault gates in real control C."""
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
static void close_rate(float expected) {
    assert(fabsf(roll_rate_test_actual-expected)<0.001f);
    assert(fabsf(roll_rate_test_error+expected)<0.001f);
    assert(abs(fly_l-(int)roundf(-FLY_RATE_KP*expected))<=1);
}
int main(void) {
    /* Seed from the first valid sample, rather than inventing zero motion. */
    arm(6);attitude_state.gyro.y=10;tick(1);close_rate(10);
    attitude_state.gyro.y=0;tick(1);
#if FILTER_BYPASS
    close_rate(0);
#else
    const float dt=(float)APP_IMU_PERIOD_US*0.000001f;
    const float alpha=dt/(FLY_RATE_LPF_TAU_S+dt);
    const float step1=10.0f*(1.0f-alpha);
    const float step2=step1*(1.0f-alpha);
    close_rate(step1);
    /* Holding an old IMU sample must not consume it repeatedly. */
    tick(0);close_rate(step1);tick(1);close_rate(step2);
    /* Alternating Nyquist noise: steady amplitude 10 * alpha / (2 - alpha). */
    arm(6);tick(1);
    for(int k=0;k<80;k++) {attitude_state.gyro.y=(k&1)?-10:10;tick(1);}
    assert(fabsf(fabsf(roll_rate_test_actual)-10.0f*alpha/(2.0f-alpha))<0.001f);
    /* DC motion is not removed by a deadband or a lasting gain reduction. */
    attitude_state.gyro.y=10;for(int k=0;k<60;k++)tick(1);close_rate(10);
#endif
    /* Rearm, calibration and mode changes cannot reuse the previous motion. */
    arm(6);tick(1);close_rate(0);
    attitude_state.gyro.y=10;tick(1);
    imu_attitude_status=0;tick(1);assert(!fly_l);
    imu_attitude_status=1;attitude_state.gyro.y=-10;tick(1);close_rate(-10);
    arm(3);pitch_balance_kp=0;attitude_state.gyro.y=10;tick(1);close_rate(10);
    /* Dual mode uses the same flywheel feedback; ground feedback is unfiltered. */
    arm(5);attitude_state.gyro.y=10;attitude_state.gyro.x=10;tick(1);
    attitude_state.gyro.y=0;attitude_state.gyro.x=0;tick(1);
    assert(ground==0 && dual_rate[1]==0);
#if FILTER_BYPASS
    assert(dual_rate[0]==0 && fly_l==0);
#else
    assert(fabsf(dual_rate[0]-step1)<0.001f && fly_l==(int)roundf(-FLY_RATE_KP*step1));
#endif
    /* Mode2 shares the same filtered flywheel path while position hold stays active. */
    arm(5);speed_balance_arm();attitude_state.gyro.y=10;tick(1);
    attitude_state.gyro.y=0;tick(1);
#if FILTER_BYPASS
    assert(dual_rate[0]==0 && fly_l==0);
#else
    assert(fabsf(dual_rate[0]-step1)<0.001f && fly_l==(int)roundf(-FLY_RATE_KP*step1));
#endif
    assert(balance_control_mode==2 && wheel_hold_enabled && roll_rate_test_state==2);
    /* Raw faults are never hidden by a healthy filter history. */
    arm(6);tick(1);attitude_state.gyro.y=NAN;tick(0);
    assert(roll_rate_test_state==-1 && !fly_l && !fly_r);
    arm(6);tick(1);attitude_state.gyro.y=INFINITY;tick(1);
    assert(roll_rate_test_state==-1 && !fly_l);
    arm(6);attitude_state.gyro.y=10;tick(1);
    for(unsigned ms=0;ms<BALANCE_IMU_STALE_MS;ms+=APP_IMU_PERIOD_US/1000u)tick(0);
    assert(control_fault_reason==4 && !fly_l && !fly_r);
    arm(6);tick(1);imu_pitch_deg=31;tick(1);
    assert(roll_rate_test_state==-2 && !fly_l);
    puts("PASS: feedback LPF seed, step, noise/DC, fresh samples, reset, axes, bypass and raw fault gates");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-rate-filter-') as directory:
    work = Path(directory)
    (work / 'zf_common_headfile.h').write_text(boundary, encoding='utf-8')
    (work / 'test.c').write_text('#include <stdlib.h>\n#include "wheel_hold.h"\n' + fixture + test, encoding='utf-8')
    for bypass in (0, 1):
        defines = ['-DFILTER_BYPASS=' + str(bypass)]
        if bypass:
            defines += ['-DFLY_RATE_LPF_TAU_S=0.0f']
        exe = work / 'filter.exe'
        subprocess.run(['gcc', '-std=c99', *defines, '-I'+str(work), '-I'+str(root/'code'),
                        str(root/'code/balance_control.c'),str(root/'code/wheel_hold.c'),str(root/'tests/encoder_snapshot_stub.c'), str(root/'code/pid.c'),
                        str(work/'test.c'), '-lm', '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True, timeout=5)

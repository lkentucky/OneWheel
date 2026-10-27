"""Exercise direct body-rate feedback and fault gates in real control C."""
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
    /* Step and sign changes must reach the PID on the next control update. */
    arm(6);attitude_state.gyro.y=10;tick(1);close_rate(10);
    attitude_state.gyro.y=0;tick(1);
    close_rate(0);
    tick(0);close_rate(0);
    /* No hidden low-pass: alternating samples remain full amplitude. */
    arm(6);tick(1);
    for(int k=0;k<60;k++) {attitude_state.gyro.y=(k&1)?-10:10;tick(1);close_rate(attitude_state.gyro.y);}
    /* DC motion is not removed by a deadband or a lasting gain reduction. */
    attitude_state.gyro.y=10;for(int k=0;k<30;k++)tick(1);close_rate(10);
    /* Rearm, calibration and mode changes cannot reuse the previous motion. */
    arm(6);tick(1);close_rate(0);
    attitude_state.gyro.y=10;tick(1);
    imu_attitude_status=0;tick(1);assert(!fly_l);
    imu_attitude_status=1;attitude_state.gyro.y=-10;tick(1);close_rate(-10);
    arm(3);pitch_balance_kp=0;attitude_state.gyro.y=10;tick(1);close_rate(10);
    /* Dual mode also consumes both body rates without extra filtering. */
    arm(5);attitude_state.gyro.y=10;attitude_state.gyro.x=10;tick(1);
    attitude_state.gyro.y=0;attitude_state.gyro.x=0;tick(1);
    assert(ground==0 && dual_rate[1]==0);
    assert(dual_rate[0]==0 && fly_l==0);
    /* Raw faults are never hidden by a healthy filter history. */
    arm(6);tick(1);attitude_state.gyro.y=NAN;tick(0);
    assert(roll_rate_test_state==-1 && !fly_l && !fly_r);
    arm(6);tick(1);attitude_state.gyro.y=INFINITY;tick(1);
    assert(roll_rate_test_state==-1 && !fly_l);
    arm(6);attitude_state.gyro.y=10;tick(1);
    for(int k=0;k<5;k++)tick(0);
    assert(control_fault_reason==4 && !fly_l && !fly_r);
    arm(6);tick(1);imu_pitch_deg=31;tick(1);
    assert(roll_rate_test_state==-2 && !fly_l);
    puts("PASS: direct rate step/reversal, full-amplitude samples, calibration, axes and fault gates");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-rate-feedback-') as directory:
    work = Path(directory)
    (work / 'zf_common_headfile.h').write_text(boundary, encoding='utf-8')
    (work / 'test.c').write_text('#include <stdlib.h>\n' + fixture + test, encoding='utf-8')
    exe = work / 'feedback.exe'
    subprocess.run(['gcc', '-std=c99', '-I'+str(work), '-I'+str(root/'code'),
                    str(root/'code/balance_control.c'), str(root/'code/pid.c'),
                    str(work/'test.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=5)

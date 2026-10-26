"""Run real control C: angle PWM compensation, signs, modes, ramp and faults."""
from pathlib import Path
import ast, subprocess, tempfile
root=Path(__file__).resolve().parents[1]
tree=ast.parse((root/'tests/check_balance_control.py').read_text(encoding='utf-8-sig'))
texts=[n.args[0].value for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='write_text' and n.args and isinstance(n.args[0],ast.Constant)]
boundary=next(s for s in texts if '#ifndef HOST_BOUNDARY' in s)
fixture=next(s for s in texts if 'int main(void)' in s).split('int main(void)')[0]
test=r'''
int main(void) {
    for(int mode=3;mode<=5;mode+=2) {
        arm(mode);roll_rate_test_kp=0;
        mono_params.balance_angle_y+=1;tick(1);
        assert(fly_l==(int)FLY_ANGLE_FF_KP && fly_r==-fly_l && ground==0);
        roll_rate_test_sign=-1;tick(1);assert(fly_l==-(int)FLY_ANGLE_FF_KP);
        roll_rate_test_limit=100;tick(1);assert(fly_l==-100 && fly_r==100);
        arm(mode);roll_rate_test_kp=0;mono_params.balance_angle_y+=1;
        control_startup_ramp_ms=1000;tick(1);assert(fly_l==40 && fly_r==-40);
        shacheflag=0;tick(1);assert(!fly_l && !fly_r);
        arm(mode);roll_rate_test_kp=0;mono_params.balance_angle_y+=1;
        imu_attitude_status=0;tick(1);assert(!fly_l);
        arm(mode);roll_rate_test_kp=0;mono_params.balance_angle_y+=1;
        for(int k=0;k<5;k++)tick(0);
        assert(control_fault_reason==4 && !fly_l && !fly_r);
        arm(mode);roll_rate_test_kp=0;mono_params.balance_angle_y+=1;
        imu_pitch_deg=31;tick(1);assert(roll_rate_test_state==-2 && !fly_l);
    }
    arm(3);pitch_balance_kp=0;roll_rate_test_kp=0;
    mono_params.balance_angle_y+=1;tick(1);assert(!fly_l);
    arm(4);ground_rate_test_kp=0;mono_params.balance_angle_x+=1;
    tick(1);assert(!ground && !fly_l);
    arm(6);roll_rate_test_kp=0;mono_params.balance_angle_y+=1;
    tick(1);assert(!fly_l);
    puts("PASS: angle compensation in Mode3/5, direction, total PWM/ramp clamp, off/calibration/stale/tilt gates, zero angle Kp and other modes");
}
'''
with tempfile.TemporaryDirectory(prefix='qcar-angle-compensation-') as d:
    w=Path(d);(w/'zf_common_headfile.h').write_text(boundary);(w/'test.c').write_text(fixture+test)
    exe=w/'test.exe'
    subprocess.run(['gcc','-std=c99','-I'+str(w),'-I'+str(root/'code'),str(root/'code/balance_control.c'),str(root/'code/pid.c'),str(w/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

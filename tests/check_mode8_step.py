"""Execute fixed bench sequence against the real controller, no hardware."""
from pathlib import Path
import ast, subprocess, tempfile
root=Path(__file__).resolve().parents[1]
tree=ast.parse((root/'tests/check_balance_control.py').read_text(encoding='utf-8-sig'))
texts=[n.args[0].value for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='write_text' and n.args and isinstance(n.args[0],ast.Constant)]
boundary=next(s for s in texts if '#ifndef HOST_BOUNDARY' in s)
fixture=next(s for s in texts if 'int main(void)' in s).split('int main(void)')[0]
test=r'''
int main(void) {
    balance_control_defaults(); assert(!pwm_step_command(1));
    pwm_debug_arm();shacheflag=1;control_startup_ramp_ms=1000;
    assert(!pwm_step_command(1));
    for(int i=0;i<200;i++)tick(0);
    assert(control_startup_scale==1);
    pwm_debug_fly_right=1;assert(!pwm_step_command(1));pwm_debug_fly_right=0;
    assert(pwm_step_command(1));
    for(unsigned t=0;t<=7500;t+=5) {
        tick(0);
        int expected=t<1000?0:t<2500?300:t<4000?600:t<5500?300:0;
        assert(pwm_step_ms==t && fly_l==expected && !fly_r && !ground);
        if(t==2000) {assert(pwm_step_command(1));assert(pwm_step_ms==2000);}
    }
    assert(pwm_step_state==2);tick(0);assert(!fly_l && pwm_step_ms==7500);
    assert(pwm_step_command(1));for(int i=0;i<250;i++)tick(0);assert(fly_l==300);
    assert(pwm_step_command(0));tick(0);assert(!fly_l && !fly_r && !ground && pwm_step_state==-1);
    assert(pwm_step_command(1));for(int i=0;i<250;i++)tick(0);shacheflag=0;
    tick(0);assert(pwm_step_state==-1 && !fly_l && !ground);
    roll_rate_test_stop();assert(!pwm_step_state && !pwm_step_ms);
    arm(5);assert(!pwm_step_command(1));tick(1);assert(!fly_l && !fly_r && !ground);
    puts("PASS: Mode8 start gates, exact stages, duplicate trigger, local completion, cancel, Stop and other modes");
}
'''
with tempfile.TemporaryDirectory(prefix='qcar-step-') as d:
    w=Path(d);(w/'zf_common_headfile.h').write_text(boundary,encoding='utf-8')
    (w/'test.c').write_text(fixture+test,encoding='utf-8')
    subprocess.run(['gcc','-std=c99','-UNDEBUG','-I'+str(w),'-I'+str(root/'code'),str(root/'code/balance_control.c'),str(root/'code/pid.c'),str(w/'test.c'),'-lm','-o',str(w/'test.exe')],check=True)
    subprocess.run([str(w/'test.exe')],check=True,timeout=5)
